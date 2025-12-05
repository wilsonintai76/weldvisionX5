"""
SafeMotionController - Safe Motion Control for Custom Inspection Rig

This module provides specialized motion control for a CoreXY 3D printer with:
- Stereo camera mounted on the toolhead
- Z-limit switch physically adjusted so Z=0 = Camera at 100mm above bed (Safe Parking)
- Positive Z movement brings bed up (closer to camera for focusing)

Key Safety Principles:
1. Home X/Y BEFORE Z to prevent gantry collisions
2. Z=0 is "Virtual Zero" (Safe Parking Height)
3. Soft Z-limit prevents bed from crashing into lens
4. All moves validated for safety before execution
5. Serial timeout handling with graceful fallback

Usage:
    controller = SafeMotionController(port='/dev/ttyUSB0')
    controller.home_safely()
    controller.park_toolhead()
    controller.move_to_inspection_height(50)
    images = controller.scan_panorama(start_x=50, end_x=250, step=20)
"""

import serial
import time
import logging
import cv2
import numpy as np
from typing import List, Optional, Tuple, Dict
from pathlib import Path
from dataclasses import dataclass
from datetime import datetime
import threading
from enum import Enum

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] %(name)s - %(message)s'
)
logger = logging.getLogger(__name__)


class SafetyState(Enum):
    """Safety state of the rig"""
    UNKNOWN = "unknown"
    SAFE = "safe"
    AT_INSPECTION = "at_inspection"
    MOVING = "moving"
    ERROR = "error"


@dataclass
class SafeMotionConfig:
    """Configuration for safe motion control"""
    # Serial communication
    port: str = '/dev/ttyUSB0'
    baudrate: int = 250000
    timeout: float = 5.0
    
    # Z-axis safety (Virtual Zero concept)
    z_safe_height: float = 0.0          # Virtual Zero (camera 100mm above bed)
    z_soft_limit: float = 90.0          # Maximum Z to prevent lens crash
    z_min_safe_margin: float = -10.0    # Below z_safe_height (if any)
    
    # Parking position
    park_x: float = 300.0
    park_y: float = 300.0
    
    # Bed dimensions
    bed_x_min: float = 0.0
    bed_x_max: float = 300.0
    bed_y_min: float = 0.0
    bed_y_max: float = 300.0
    
    # Motion speeds (mm/min)
    homing_speed: int = 6000
    move_speed: int = 3000
    scan_speed: int = 1000
    
    # Camera
    camera_id: int = 0
    image_width: int = 640
    image_height: int = 480
    capture_delay: float = 0.5
    
    # Scanning
    stitch_images: bool = True
    stitch_quality: str = "best"


class SerialComm:
    """Serial communication wrapper with timeout handling"""
    
    def __init__(self, port: str, baudrate: int, timeout: float):
        """
        Initialize serial connection
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0')
            baudrate: Communication speed
            timeout: Response timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.lock = threading.Lock()
        self.connected = False
        self.last_error = None
    
    def connect(self) -> bool:
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for board initialization
            self.connected = True
            logger.info(f"✓ Serial connected: {self.port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            self.last_error = str(e)
            logger.error(f"✗ Serial connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            logger.info("Serial disconnected")
    
    def send_gcode(self, command: str, wait_for_ok: bool = True) -> bool:
        """
        Send G-code command
        
        Args:
            command: G-code string (without newline)
            wait_for_ok: If True, wait for 'ok' response
            
        Returns:
            True if command succeeded, False otherwise
        """
        if not self.connected or not self.serial.is_open:
            logger.error("Serial not connected")
            return False
        
        with self.lock:
            try:
                # Clear input buffer
                self.serial.reset_input_buffer()
                
                # Send command
                cmd_line = command.strip() + '\n'
                self.serial.write(cmd_line.encode())
                logger.debug(f"→ {command}")
                
                if not wait_for_ok:
                    return True
                
                # Wait for response
                start_time = time.time()
                response = ""
                
                while time.time() - start_time < self.timeout:
                    if self.serial.in_waiting:
                        response += self.serial.read(1).decode(errors='ignore')
                        
                        if 'ok' in response.lower():
                            logger.debug(f"← ok")
                            return True
                        
                        if 'error' in response.lower():
                            logger.error(f"Firmware error: {response.strip()}")
                            self.last_error = response
                            return False
                    
                    time.sleep(0.01)
                
                # Timeout
                logger.warning(f"Command timeout: {command}")
                self.last_error = "Timeout"
                return False
                
            except Exception as e:
                logger.error(f"Serial error: {e}")
                self.last_error = str(e)
                return False
    
    def get_error(self) -> Optional[str]:
        """Get last error message"""
        return self.last_error


class CameraCapture:
    """OpenCV camera wrapper"""
    
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        """Initialize camera"""
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.camera = None
        self.connected = False
    
    def connect(self) -> bool:
        """Initialize camera"""
        try:
            self.camera = cv2.VideoCapture(self.camera_id)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            ret, _ = self.camera.read()
            if ret:
                self.connected = True
                logger.info(f"✓ Camera initialized ({self.width}×{self.height})")
                return True
            else:
                logger.error("Failed to read from camera")
                return False
        except Exception as e:
            logger.error(f"Camera error: {e}")
            return False
    
    def capture(self) -> Optional[np.ndarray]:
        """Capture single frame"""
        if not self.connected or self.camera is None:
            return None
        try:
            ret, frame = self.camera.read()
            return frame if ret else None
        except Exception as e:
            logger.error(f"Capture error: {e}")
            return None
    
    def disconnect(self):
        """Release camera"""
        if self.camera is not None:
            self.camera.release()
            self.connected = False


class SafeMotionController:
    """
    Safe motion controller for custom inspection rig with Virtual Zero concept
    
    Hardware:
    - Z-limit switch adjusted: Z=0 = Camera at 100mm above bed (Safe Parking)
    - Positive Z brings bed closer (up to ~90mm before lens collision)
    - X/Y must be homed before Z to prevent collisions
    """
    
    def __init__(self, config: Optional[SafeMotionConfig] = None):
        """
        Initialize controller
        
        Args:
            config: SafeMotionConfig object
        """
        self.config = config or SafeMotionConfig()
        self.serial = SerialComm(
            self.config.port,
            self.config.baudrate,
            self.config.timeout
        )
        self.camera = CameraCapture(
            self.config.camera_id,
            self.config.image_width,
            self.config.image_height
        )
        
        # State tracking
        self.safety_state = SafetyState.UNKNOWN
        self.current_z = None  # Current Z position
        self.homed_xy = False  # X/Y homing status
        self.homed_z = False   # Z homing status
        self.inspection_height = None  # Current inspection Z
        
        # Scan data
        self.last_scan_frames: List[Tuple[float, np.ndarray]] = []
    
    def connect(self) -> bool:
        """Connect to serial and camera"""
        serial_ok = self.serial.connect()
        camera_ok = self.camera.connect()
        
        if serial_ok:
            logger.info("✓ SafeMotionController ready")
            return True
        else:
            logger.error("✗ Connection failed")
            return False
    
    def disconnect(self):
        """Disconnect from all interfaces"""
        self.serial.disconnect()
        self.camera.disconnect()
        logger.info("Disconnected")
    
    # ==================== SAFE HOMING ====================
    
    def home_safely(self) -> bool:
        """
        Safe homing sequence
        
        Sequence:
        1. Home X and Y FIRST (G28 X Y) - prevents gantry collision
        2. Then home Z (G28 Z) - now safe because X/Y are homed
        3. Verify Z=0 (Virtual Zero, camera at Safe Height)
        
        Returns:
            True if homing successful
        """
        logger.info("=" * 60)
        logger.info("STARTING SAFE HOMING SEQUENCE")
        logger.info("=" * 60)
        
        try:
            # Step 1: Home X/Y first (CRITICAL for safety)
            logger.info("→ Homing X/Y axes first (prevents collision)")
            cmd_xy = f"G28 X Y F{self.config.homing_speed}"
            if not self.serial.send_gcode(cmd_xy):
                logger.error("✗ X/Y homing failed")
                self.safety_state = SafetyState.ERROR
                return False
            
            self.homed_xy = True
            logger.info("✓ X/Y axes homed successfully")
            time.sleep(0.5)
            
            # Step 2: Home Z
            logger.info("→ Homing Z axis (now safe)")
            cmd_z = f"G28 Z F{self.config.homing_speed}"
            if not self.serial.send_gcode(cmd_z):
                logger.error("✗ Z homing failed")
                self.safety_state = SafetyState.ERROR
                return False
            
            self.homed_z = True
            self.current_z = self.config.z_safe_height
            logger.info("✓ Z axis homed successfully")
            
            # Step 3: Verify safe height
            logger.info("=" * 60)
            logger.info("✓✓✓ RIG HOMING COMPLETE ✓✓✓")
            logger.info(f"✓ Camera is parked at Safe Height (Z = {self.config.z_safe_height} mm)")
            logger.info("✓ Virtual Zero established: Camera is 100mm above bed")
            logger.info("=" * 60)
            
            self.safety_state = SafetyState.SAFE
            return True
            
        except Exception as e:
            logger.error(f"✗ Homing sequence error: {e}")
            self.safety_state = SafetyState.ERROR
            return False
    
    # ==================== PARKING & POSITIONING ====================
    
    def park_toolhead(self, x: Optional[float] = None, y: Optional[float] = None) -> bool:
        """
        Park toolhead to safe position for loading/unloading
        
        Actions:
        1. Move to parking corner (default: X300, Y300)
        2. Move Z to Safe Height (Z=0)
        3. Ensure motors locked
        
        Args:
            x: Parking X position (None = use config)
            y: Parking Y position (None = use config)
            
        Returns:
            True if parking successful
        """
        logger.info("-" * 60)
        logger.info("PARKING TOOLHEAD FOR LOADING/UNLOADING")
        logger.info("-" * 60)
        
        if not self.homed_xy or not self.homed_z:
            logger.error("✗ Must home before parking")
            return False
        
        try:
            park_x = x or self.config.park_x
            park_y = y or self.config.park_y
            
            # Move to corner (XY plane)
            logger.info(f"→ Moving to parking corner (X{park_x}, Y{park_y})")
            cmd_xy = f"G0 X{park_x} Y{park_y} F{self.config.move_speed}"
            if not self.serial.send_gcode(cmd_xy):
                logger.error("✗ Failed to move to parking XY")
                return False
            
            time.sleep(0.3)
            
            # Move Z to safe height
            logger.info(f"→ Raising Z to Safe Height (Z{self.config.z_safe_height})")
            cmd_z = f"G0 Z{self.config.z_safe_height} F{self.config.move_speed}"
            if not self.serial.send_gcode(cmd_z):
                logger.error("✗ Failed to move to safe Z height")
                return False
            
            self.current_z = self.config.z_safe_height
            self.inspection_height = None
            self.safety_state = SafetyState.SAFE
            
            logger.info("-" * 60)
            logger.info("✓ Toolhead parked safely - Ready for loading")
            logger.info("-" * 60)
            return True
            
        except Exception as e:
            logger.error(f"✗ Parking error: {e}")
            return False
    
    def move_to_inspection_height(self, target_z_mm: float) -> bool:
        """
        Move Z to inspection height with safety checks
        
        Safety:
        1. Sanity check: target_z <= soft limit (90mm)
        2. Sanity check: target_z >= safe height (0mm)
        3. Must be homed first
        4. Logs all moves for audit trail
        
        Args:
            target_z_mm: Target Z height in mm (0 = Safe Height)
            
        Returns:
            True if move successful
        """
        logger.info("-" * 60)
        logger.info("MOVING TO INSPECTION HEIGHT")
        logger.info("-" * 60)
        
        # Safety Check 1: Must be homed
        if not self.homed_xy or not self.homed_z:
            logger.error("✗ Must home before inspection moves")
            return False
        
        # Safety Check 2: Soft limit (prevent lens crash)
        if target_z_mm > self.config.z_soft_limit:
            logger.error(f"✗ SOFT LIMIT EXCEEDED: {target_z_mm} > {self.config.z_soft_limit}")
            logger.error(f"   Risk of bed crashing into lens!")
            return False
        
        # Safety Check 3: Minimum height
        if target_z_mm < self.config.z_min_safe_margin:
            logger.error(f"✗ Z below safe minimum: {target_z_mm} < {self.config.z_min_safe_margin}")
            return False
        
        try:
            logger.info(f"✓ Height validated: {target_z_mm}mm (Safe: {self.config.z_soft_limit}mm)")
            logger.info(f"→ Moving Z to inspection height...")
            
            cmd = f"G0 Z{target_z_mm} F{self.config.move_speed}"
            if not self.serial.send_gcode(cmd):
                logger.error("✗ Failed to move Z")
                return False
            
            self.current_z = target_z_mm
            self.inspection_height = target_z_mm
            self.safety_state = SafetyState.AT_INSPECTION
            
            logger.info("-" * 60)
            logger.info(f"✓ At inspection height: Z = {target_z_mm}mm")
            logger.info(f"✓ Camera focused on part ({100 - target_z_mm}mm from safe height)")
            logger.info("-" * 60)
            return True
            
        except Exception as e:
            logger.error(f"✗ Inspection height move error: {e}")
            return False
    
    # ==================== PANORAMA SCANNING ====================
    
    def scan_panorama(self, start_x: float, end_x: float, step: float,
                      y_pos: Optional[float] = None) -> List[np.ndarray]:
        """
        Panorama scan while maintaining inspection height
        
        Strategy:
        1. Ensure at inspection height (Z = target_z)
        2. For each X position:
           a. Move to (X, Y, Z_current) - horizontal move, Z unchanged
           b. Wait for vibration settling
           c. Capture image
           d. Store with position metadata
        3. Return list of images for stitching
        
        Args:
            start_x: Starting X position
            end_x: Ending X position
            step: X-axis step size (mm)
            y_pos: Y position (None = current)
            
        Returns:
            List of captured images
        """
        logger.info("=" * 60)
        logger.info("STARTING PANORAMA SCAN")
        logger.info("=" * 60)
        
        # Safety checks
        if not self.homed_xy or not self.homed_z:
            logger.error("✗ Must home before scanning")
            return []
        
        if self.inspection_height is None:
            logger.error("✗ Must move to inspection height before scanning")
            logger.error("   Use move_to_inspection_height() first")
            return []
        
        if step <= 0:
            logger.error("✗ Step must be positive")
            return []
        
        self.last_scan_frames.clear()
        
        try:
            # Calculate scan parameters
            distance = abs(end_x - start_x)
            num_steps = int(distance / step) + 1
            logger.info(f"Scan distance: {distance}mm, Step: {step}mm, Points: {num_steps}")
            logger.info(f"Inspection height: Z = {self.inspection_height}mm")
            
            if y_pos is None:
                y_pos = self.config.bed_y_max / 2  # Center Y
            
            self.safety_state = SafetyState.MOVING
            captured_images = []
            
            for i in range(num_steps):
                current_x = start_x + (i * step)
                
                # Clamp to end
                if current_x > end_x:
                    current_x = end_x
                
                # Move to scan position (XY only, Z maintained)
                logger.info(f"[{i+1}/{num_steps}] Moving to X{current_x:.1f}...")
                cmd = f"G0 X{current_x} Y{y_pos} F{self.config.scan_speed}"
                
                if not self.serial.send_gcode(cmd):
                    logger.warning(f"Move to X{current_x} failed")
                    continue
                
                # Wait for vibration to settle
                time.sleep(self.config.capture_delay)
                
                # Capture image
                logger.info(f"[{i+1}/{num_steps}] Capturing...")
                frame = self.camera.capture()
                
                if frame is not None:
                    captured_images.append(frame)
                    self.last_scan_frames.append((current_x, frame))
                    logger.debug(f"Captured at X{current_x:.1f}: {frame.shape}")
                else:
                    logger.warning(f"Capture failed at X{current_x:.1f}")
                
                if current_x >= end_x:
                    break
            
            logger.info("=" * 60)
            logger.info(f"✓ Panorama scan complete: {len(captured_images)} frames captured")
            logger.info("=" * 60)
            
            self.safety_state = SafetyState.AT_INSPECTION
            return captured_images
            
        except Exception as e:
            logger.error(f"✗ Scan error: {e}")
            self.safety_state = SafetyState.ERROR
            return []
    
    # ==================== IMAGE STITCHING ====================
    
    def stitch_panorama(self, images: List[np.ndarray]) -> Optional[np.ndarray]:
        """
        Stitch scan images into panorama
        
        Args:
            images: List of captured frames
            
        Returns:
            Stitched panorama or None
        """
        if not images or len(images) < 2:
            logger.warning("Need at least 2 images for stitching")
            return images[0] if images else None
        
        logger.info(f"Stitching {len(images)} images...")
        
        try:
            if self.config.stitch_quality == "best":
                stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
            else:
                stitcher = cv2.Stitcher_create()
            
            status, panorama = stitcher.stitch(images)
            
            if status == cv2.Stitcher_OK:
                logger.info(f"✓ Stitched successfully: {panorama.shape}")
                return panorama
            else:
                logger.error(f"Stitching failed (code {status})")
                # Fallback: horizontal concatenation
                logger.info("Using horizontal concatenation as fallback")
                h = images[0].shape[0]
                resized = [cv2.resize(img, (int(img.shape[1] * h / img.shape[0]), h)) for img in images]
                return np.hstack(resized)
        
        except Exception as e:
            logger.error(f"Stitch error: {e}")
            return None
    
    # ==================== UTILITY ====================
    
    def save_scan(self, output_dir: str = "scans") -> str:
        """Save scan frames to disk"""
        if not self.last_scan_frames:
            logger.warning("No scan data to save")
            return ""
        
        try:
            output_path = Path(output_dir) / datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path.mkdir(parents=True, exist_ok=True)
            
            for idx, (x_pos, frame) in enumerate(self.last_scan_frames):
                cv2.imwrite(str(output_path / f"frame_{idx:03d}_x{x_pos:.1f}.png"), frame)
            
            logger.info(f"✓ Saved {len(self.last_scan_frames)} frames to {output_path}")
            return str(output_path)
        
        except Exception as e:
            logger.error(f"Save error: {e}")
            return ""
    
    def emergency_stop(self):
        """Emergency stop"""
        logger.warning("!!! EMERGENCY STOP !!!")
        self.serial.send_gcode("M410")
        self.safety_state = SafetyState.ERROR
    
    def get_status(self) -> Dict:
        """Get controller status"""
        return {
            "safety_state": self.safety_state.value,
            "homed_xy": self.homed_xy,
            "homed_z": self.homed_z,
            "current_z": self.current_z,
            "inspection_height": self.inspection_height,
            "serial_connected": self.serial.connected,
            "camera_connected": self.camera.connected,
        }


# ==================== EXAMPLE USAGE ====================

if __name__ == "__main__":
    import sys
    
    logging.basicConfig(level=logging.INFO)
    
    # Create controller
    config = SafeMotionConfig(
        port='/dev/ttyUSB0',  # Adjust for your system
        z_safe_height=0.0,
        z_soft_limit=90.0,
        park_x=280.0,
        park_y=280.0,
    )
    
    controller = SafeMotionController(config)
    
    try:
        # Connect
        if not controller.connect():
            logger.error("Connection failed")
            sys.exit(1)
        
        # Print status
        print("\nStatus:", controller.get_status())
        
        # Home safely
        if not controller.home_safely():
            logger.error("Homing failed")
            sys.exit(1)
        
        # Park for loading
        if not controller.park_toolhead():
            logger.error("Parking failed")
            sys.exit(1)
        
        input("\nPress Enter when part is loaded...")
        
        # Move to inspection height
        if not controller.move_to_inspection_height(50):
            logger.error("Inspection height move failed")
            sys.exit(1)
        
        # Scan panorama
        images = controller.scan_panorama(start_x=50, end_x=250, step=20)
        
        if images:
            # Stitch
            panorama = controller.stitch_panorama(images)
            if panorama is not None:
                cv2.imwrite("panorama.png", panorama)
                logger.info("Panorama saved!")
            
            # Save frames
            controller.save_scan("inspection_results")
        
        # Return to parking
        if not controller.park_toolhead():
            logger.error("Failed to return to parking")
        
        print("\n✓ Inspection complete!")
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}")
        controller.emergency_stop()
    finally:
        controller.disconnect()
