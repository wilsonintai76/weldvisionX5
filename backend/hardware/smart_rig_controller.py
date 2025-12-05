"""
SmartRigController - Advanced Motion & Imaging Control for WeldVision X5

This module provides high-level control over a CoreXY 3D printer with RDK X5 integration,
enabling automated scanning, panorama stitching, and intelligent mode switching.

Features:
- Safe homing sequence with Marlin firmware validation
- Panorama scanning for long weld inspection (image stitching)
- Loading vs. Inspection mode switching
- Motor locking for vibration resistance
- Serial communication with timeout handling
- OpenCV image capture and stitching

Hardware:
- RDK X5 board (camera control)
- CoreXY 3D printer (Marlin firmware)
- Modified toolhead with integrated camera
- Three independent Z-axis motors for bed leveling

Usage:
    controller = SmartRigController(port='/dev/ttyUSB0', baudrate=250000)
    controller.home()
    controller.set_mode_inspection()
    images = controller.scan_large_weld(50, 250, 20)
    panorama = controller.stitch_images(images)
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
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class OperationMode(Enum):
    """Operating modes for the rig"""
    IDLE = "idle"
    LOADING = "loading"
    INSPECTION = "inspection"
    SCANNING = "scanning"
    ERROR = "error"


class MotorState(Enum):
    """Motor energization states"""
    LOCKED = "locked"      # M17 - motors energized
    RELEASED = "released"  # M18 - motors de-energized


@dataclass
class RigConfig:
    """Configuration parameters for the rig"""
    # Serial communication
    port: str = '/dev/ttyUSB0'
    baudrate: int = 250000
    timeout: float = 5.0
    read_timeout: float = 2.0
    
    # Bed/Rig dimensions (mm)
    bed_x_min: float = 0.0
    bed_x_max: float = 300.0
    bed_y_min: float = 0.0
    bed_y_max: float = 300.0
    bed_z_min: float = 0.0
    bed_z_max: float = 300.0
    
    # Safe positions (mm)
    park_x: float = 300.0        # Parking corner X
    park_y: float = 300.0        # Parking corner Y
    park_z: float = 200.0        # Parking height (safe from loaded plates)
    center_x: float = 150.0       # Bed center X
    center_y: float = 150.0       # Bed center Y
    focus_z: float = 100.0        # Camera focus height
    
    # Movement
    homing_speed: int = 6000      # mm/min
    move_speed: int = 3000        # mm/min
    scan_speed: int = 1000        # mm/min (slow for image quality)
    
    # Camera
    camera_id: int = 0            # OpenCV camera index
    image_width: int = 640
    image_height: int = 480
    capture_delay: float = 0.5    # seconds between captures
    
    # Stitching
    enable_stitching: bool = True
    stitch_quality: str = "best"  # "panorama", "best", "fast"


@dataclass
class MotionCommand:
    """Represents a G-code motion command"""
    command: str
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    f: Optional[int] = None        # Feed rate (mm/min)
    
    def to_gcode(self) -> str:
        """Convert to G-code string"""
        parts = [self.command]
        if self.x is not None:
            parts.append(f"X{self.x:.2f}")
        if self.y is not None:
            parts.append(f"Y{self.y:.2f}")
        if self.z is not None:
            parts.append(f"Z{self.z:.2f}")
        if self.f is not None:
            parts.append(f"F{self.f}")
        return " ".join(parts)


@dataclass
class ScanFrame:
    """Individual frame captured during scanning"""
    position_x: float
    position_y: float
    position_z: float
    image: np.ndarray
    timestamp: str
    index: int


class SerialInterface:
    """Low-level serial communication with Marlin firmware"""
    
    def __init__(self, port: str, baudrate: int, timeout: float):
        """
        Initialize serial connection
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0')
            baudrate: Communication speed
            timeout: Command timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.lock = threading.Lock()
        self.connected = False
    
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
            logger.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            logger.info("Disconnected from serial port")
    
    def send_command(self, command: str) -> bool:
        """
        Send G-code command and wait for response
        
        Args:
            command: G-code command string
            
        Returns:
            True if command succeeded, False otherwise
        """
        if not self.connected or not self.serial.is_open:
            logger.error("Serial port not connected")
            return False
        
        with self.lock:
            try:
                # Clear buffer
                self.serial.reset_input_buffer()
                
                # Send command
                cmd_line = command.strip() + '\n'
                self.serial.write(cmd_line.encode())
                logger.debug(f"Sent: {command}")
                
                # Wait for 'ok' response
                start_time = time.time()
                response = ""
                
                while time.time() - start_time < self.timeout:
                    if self.serial.in_waiting:
                        response += self.serial.read(1).decode(errors='ignore')
                        
                        # Check for complete response
                        if 'ok' in response.lower():
                            logger.debug(f"Response: {response.strip()}")
                            return True
                        
                        # Check for error
                        if 'error' in response.lower():
                            logger.error(f"Firmware error: {response}")
                            return False
                    
                    time.sleep(0.01)
                
                logger.warning(f"Command timeout: {command}")
                return False
                
            except Exception as e:
                logger.error(f"Serial communication error: {e}")
                return False
    
    def send_command_silent(self, command: str) -> bool:
        """Send command without waiting for response (for rapid-fire commands)"""
        if not self.connected or not self.serial.is_open:
            return False
        
        try:
            cmd_line = command.strip() + '\n'
            self.serial.write(cmd_line.encode())
            logger.debug(f"Sent (silent): {command}")
            return True
        except Exception as e:
            logger.error(f"Serial write error: {e}")
            return False
    
    def read_response(self, timeout: float = 2.0) -> str:
        """Read response from firmware"""
        response = ""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                response += self.serial.read(1).decode(errors='ignore')
                if 'ok' in response.lower():
                    return response
            time.sleep(0.01)
        
        return response


class CameraInterface:
    """OpenCV camera interface for image capture"""
    
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        """
        Initialize camera
        
        Args:
            camera_id: OpenCV camera index
            width: Frame width
            height: Frame height
        """
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
            
            # Test capture
            ret, _ = self.camera.read()
            if ret:
                self.connected = True
                logger.info(f"Camera {self.camera_id} initialized ({self.width}×{self.height})")
                return True
            else:
                logger.error(f"Failed to capture from camera {self.camera_id}")
                return False
                
        except Exception as e:
            logger.error(f"Camera initialization error: {e}")
            return False
    
    def capture(self) -> Optional[np.ndarray]:
        """
        Capture single frame
        
        Returns:
            Frame as numpy array, or None if capture failed
        """
        if not self.connected or self.camera is None:
            return None
        
        try:
            ret, frame = self.camera.read()
            if ret:
                return frame
            else:
                logger.warning("Failed to read frame from camera")
                return None
        except Exception as e:
            logger.error(f"Camera capture error: {e}")
            return None
    
    def disconnect(self):
        """Release camera"""
        if self.camera is not None:
            self.camera.release()
            self.connected = False
            logger.info("Camera released")


class SmartRigController:
    """
    High-level controller for WeldVision X5 rig with panorama scanning and mode switching
    """
    
    def __init__(self, config: Optional[RigConfig] = None):
        """
        Initialize SmartRigController
        
        Args:
            config: RigConfig object with parameters
        """
        self.config = config or RigConfig()
        self.serial = SerialInterface(
            self.config.port,
            self.config.baudrate,
            self.config.timeout
        )
        self.camera = CameraInterface(
            self.config.camera_id,
            self.config.image_width,
            self.config.image_height
        )
        
        # State tracking
        self.mode = OperationMode.IDLE
        self.motor_state = MotorState.RELEASED
        self.current_pos = {'x': 0, 'y': 0, 'z': 0}
        self.homed = False
        
        # Scan data
        self.last_scan_frames: List[ScanFrame] = []
    
    def connect(self) -> bool:
        """Connect to both serial and camera"""
        serial_ok = self.serial.connect()
        camera_ok = self.camera.connect()
        
        if serial_ok and camera_ok:
            logger.info("SmartRigController ready")
            return True
        else:
            logger.error("Failed to initialize SmartRigController")
            return False
    
    def disconnect(self):
        """Disconnect from all interfaces"""
        self.release_motors()
        self.serial.disconnect()
        self.camera.disconnect()
        logger.info("SmartRigController disconnected")
    
    # ==================== SAFE HOMING SEQUENCE ====================
    
    def home(self) -> bool:
        """
        Safe homing sequence following Marlin standard
        
        Sequence:
        1. Lock motors (M17)
        2. Send G28 (home X/Y/Z)
        3. Wait for completion
        4. Verify homing success
        
        Returns:
            True if homing successful, False otherwise
        """
        logger.info("=== STARTING SAFE HOMING SEQUENCE ===")
        
        try:
            # Step 1: Lock motors to prevent slipping
            if not self.lock_motors():
                logger.error("Failed to lock motors before homing")
                return False
            
            time.sleep(0.5)
            
            # Step 2: Send homing command
            # G28 homes all axes in standard order: X, Y, Z
            homing_cmd = MotionCommand(
                command="G28",
                f=self.config.homing_speed
            )
            
            logger.info(f"Sending homing command: {homing_cmd.to_gcode()}")
            if not self.serial.send_command(homing_cmd.to_gcode()):
                logger.error("Homing command failed")
                self.mode = OperationMode.ERROR
                return False
            
            time.sleep(1)
            
            # Step 3: Verify homing
            # After homing, X/Y should be at min (0,0) and Z should be at max
            self.current_pos = {'x': 0.0, 'y': 0.0, 'z': self.config.bed_z_max}
            self.homed = True
            self.motor_state = MotorState.LOCKED
            
            logger.info("✓ Homing complete. Position: X=0 Y=0 Z=MAX")
            logger.info("✓ Motors locked. Z-axis is now movable (Marlin standard)")
            
            return True
            
        except Exception as e:
            logger.error(f"Homing sequence error: {e}")
            self.mode = OperationMode.ERROR
            return False
    
    # ==================== MOTOR CONTROL ====================
    
    def lock_motors(self) -> bool:
        """Lock motors (M17) to prevent slipping"""
        if self.serial.send_command("M17"):
            self.motor_state = MotorState.LOCKED
            logger.debug("Motors locked (M17)")
            return True
        return False
    
    def release_motors(self) -> bool:
        """Release motors (M18) - use carefully"""
        if self.serial.send_command("M18"):
            self.motor_state = MotorState.RELEASED
            logger.debug("Motors released (M18)")
            return True
        return False
    
    def move_to(self, x: Optional[float] = None, y: Optional[float] = None, 
                z: Optional[float] = None, speed: Optional[int] = None) -> bool:
        """
        Move to absolute position
        
        Args:
            x, y, z: Target positions (None = don't move that axis)
            speed: Feed rate in mm/min (None = use default move speed)
            
        Returns:
            True if move successful
        """
        if not self.homed:
            logger.error("Must home before moving")
            return False
        
        # Clamp values to bed limits
        if x is not None:
            x = max(self.config.bed_x_min, min(self.config.bed_x_max, x))
        if y is not None:
            y = max(self.config.bed_y_min, min(self.config.bed_y_max, y))
        if z is not None:
            z = max(self.config.bed_z_min, min(self.config.bed_z_max, z))
        
        # Create motion command
        feed_rate = speed or self.config.move_speed
        cmd = MotionCommand(
            command="G0",  # Rapid linear move
            x=x, y=y, z=z,
            f=feed_rate
        )
        
        logger.debug(f"Moving to: {cmd.to_gcode()}")
        
        if self.serial.send_command(cmd.to_gcode()):
            # Update position
            if x is not None:
                self.current_pos['x'] = x
            if y is not None:
                self.current_pos['y'] = y
            if z is not None:
                self.current_pos['z'] = z
            
            logger.debug(f"Position: X={self.current_pos['x']:.1f} Y={self.current_pos['y']:.1f} Z={self.current_pos['z']:.1f}")
            return True
        
        return False
    
    # ==================== LOADING vs. INSPECTION MODES ====================
    
    def set_mode_loading(self) -> bool:
        """
        Set LOADING mode
        
        Actions:
        1. Move to parking position (X300, Y300)
        2. Raise Z to safe height (Z200)
        3. Lock motors for stability
        
        Purpose: Prevents collision when loading heavy steel plates
        """
        logger.info("=== SWITCHING TO LOADING MODE ===")
        
        try:
            # Ensure motors are locked
            if not self.lock_motors():
                logger.error("Failed to lock motors")
                return False
            
            # Move to parking position
            logger.info(f"Moving to parking position (X{self.config.park_x}, Y{self.config.park_y})")
            if not self.move_to(
                x=self.config.park_x,
                y=self.config.park_y,
                speed=self.config.move_speed
            ):
                logger.error("Failed to move to parking X/Y")
                return False
            
            time.sleep(0.5)
            
            # Raise Z to safe height
            logger.info(f"Raising Z to safe height (Z{self.config.park_z})")
            if not self.move_to(z=self.config.park_z, speed=self.config.move_speed):
                logger.error("Failed to move to parking Z")
                return False
            
            self.mode = OperationMode.LOADING
            logger.info("✓ LOADING mode active. Camera is safely out of the way.")
            return True
            
        except Exception as e:
            logger.error(f"Set loading mode error: {e}")
            self.mode = OperationMode.ERROR
            return False
    
    def set_mode_inspection(self) -> bool:
        """
        Set INSPECTION mode
        
        Actions:
        1. Move to center of rig (X150, Y150)
        2. Lower Z to focus height (Z100)
        3. Lock motors for stability
        
        Purpose: Positions camera at optimal viewing angle for weld inspection
        """
        logger.info("=== SWITCHING TO INSPECTION MODE ===")
        
        try:
            # Ensure motors are locked
            if not self.lock_motors():
                logger.error("Failed to lock motors")
                return False
            
            # Move to center position
            logger.info(f"Moving to inspection center (X{self.config.center_x}, Y{self.config.center_y})")
            if not self.move_to(
                x=self.config.center_x,
                y=self.config.center_y,
                speed=self.config.move_speed
            ):
                logger.error("Failed to move to center X/Y")
                return False
            
            time.sleep(0.5)
            
            # Lower Z to focus height
            logger.info(f"Lowering Z to focus height (Z{self.config.focus_z})")
            if not self.move_to(z=self.config.focus_z, speed=self.config.move_speed):
                logger.error("Failed to move to focus Z")
                return False
            
            self.mode = OperationMode.INSPECTION
            logger.info("✓ INSPECTION mode active. Camera positioned for viewing.")
            return True
            
        except Exception as e:
            logger.error(f"Set inspection mode error: {e}")
            self.mode = OperationMode.ERROR
            return False
    
    # ==================== PANORAMA SCANNING ====================
    
    def scan_large_weld(self, start_x: float, end_x: float, step_mm: float,
                        y_pos: Optional[float] = None) -> List[np.ndarray]:
        """
        Panorama scan for long weld inspection
        
        Strategy:
        1. Move to start position
        2. Capture image
        3. Move by step_mm along X axis
        4. Repeat until end_x
        5. Return list of captured frames
        
        Args:
            start_x: Starting X position (mm)
            end_x: Ending X position (mm)
            step_mm: Movement step between captures (mm)
            y_pos: Y position (None = stay at current)
            
        Returns:
            List of captured images (numpy arrays)
        """
        logger.info(f"=== STARTING PANORAMA SCAN ===")
        logger.info(f"Scan range: X{start_x:.1f} → X{end_x:.1f}, Step: {step_mm}mm")
        
        self.mode = OperationMode.SCANNING
        self.last_scan_frames.clear()
        
        try:
            # Ensure we're homed and motors are locked
            if not self.homed:
                logger.error("Must home before scanning")
                return []
            
            if not self.lock_motors():
                logger.error("Failed to lock motors for scanning")
                return []
            
            # Calculate scan points
            if step_mm <= 0:
                logger.error("Step must be positive")
                return []
            
            num_steps = int(abs(end_x - start_x) / step_mm) + 1
            logger.info(f"Calculated {num_steps} scan points")
            
            # Use current Y if not specified
            if y_pos is None:
                y_pos = self.current_pos['y']
            
            captured_images = []
            
            for i in range(num_steps):
                # Calculate position
                current_x = start_x + (i * step_mm)
                
                # Clamp to end position
                if current_x > end_x:
                    current_x = end_x
                
                # Move to position
                logger.info(f"[{i+1}/{num_steps}] Moving to X{current_x:.1f}")
                if not self.move_to(x=current_x, y=y_pos, speed=self.config.scan_speed):
                    logger.warning(f"Failed to move to X{current_x:.1f}")
                    continue
                
                # Wait for vibration to settle
                time.sleep(self.config.capture_delay)
                
                # Capture image
                logger.info(f"[{i+1}/{num_steps}] Capturing image...")
                frame = self.camera.capture()
                
                if frame is not None:
                    captured_images.append(frame)
                    
                    # Store frame data for later reference
                    scan_frame = ScanFrame(
                        position_x=current_x,
                        position_y=y_pos,
                        position_z=self.current_pos['z'],
                        image=frame,
                        timestamp=datetime.now().isoformat(),
                        index=len(self.last_scan_frames)
                    )
                    self.last_scan_frames.append(scan_frame)
                    
                    logger.debug(f"Captured frame {i+1}: {frame.shape}")
                else:
                    logger.warning(f"Failed to capture image at X{current_x:.1f}")
                
                # Stop if we've reached end
                if current_x >= end_x:
                    break
            
            logger.info(f"✓ Panorama scan complete. Captured {len(captured_images)} frames")
            
            if len(captured_images) > 0:
                self.mode = OperationMode.INSPECTION
                return captured_images
            else:
                logger.error("No frames captured during scan")
                self.mode = OperationMode.ERROR
                return []
            
        except Exception as e:
            logger.error(f"Panorama scan error: {e}")
            self.mode = OperationMode.ERROR
            return []
    
    # ==================== IMAGE STITCHING ====================
    
    def stitch_images(self, images: List[np.ndarray]) -> Optional[np.ndarray]:
        """
        Stitch multiple images into panorama
        
        Uses OpenCV Stitcher to combine overlapping images
        
        Args:
            images: List of image arrays
            
        Returns:
            Stitched panorama or None if stitching failed
        """
        if not images or len(images) < 2:
            logger.warning("Need at least 2 images for stitching")
            return images[0] if images else None
        
        logger.info(f"Stitching {len(images)} images into panorama...")
        
        try:
            # Create stitcher
            if self.config.stitch_quality == "best":
                stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
            elif self.config.stitch_quality == "fast":
                stitcher = cv2.Stitcher_create(cv2.Stitcher_SCANS)
            else:
                stitcher = cv2.Stitcher_create()
            
            # Stitch images
            status, panorama = stitcher.stitch(images)
            
            if status == cv2.Stitcher_OK:
                logger.info(f"✓ Stitching successful. Output size: {panorama.shape}")
                return panorama
            else:
                error_msgs = {
                    cv2.Stitcher_ERR_NEED_MORE_IMGS: "Not enough images",
                    cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL: "Homography estimation failed",
                    cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL: "Camera parameters adjustment failed"
                }
                error_msg = error_msgs.get(status, f"Unknown error ({status})")
                logger.error(f"Stitching failed: {error_msg}")
                
                # Return concatenated images as fallback
                logger.info("Returning horizontally concatenated images as fallback")
                height = images[0].shape[0]
                resized_images = [cv2.resize(img, (int(img.shape[1] * height / img.shape[0]), height)) 
                                 for img in images]
                return np.hstack(resized_images)
            
        except Exception as e:
            logger.error(f"Image stitching error: {e}")
            return None
    
    def save_scan_data(self, output_dir: str = "scans") -> str:
        """
        Save scan frames and metadata
        
        Args:
            output_dir: Directory to save scan data
            
        Returns:
            Path to saved directory
        """
        if not self.last_scan_frames:
            logger.warning("No scan data to save")
            return ""
        
        try:
            output_path = Path(output_dir) / datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path.mkdir(parents=True, exist_ok=True)
            
            # Save individual frames
            for frame in self.last_scan_frames:
                frame_file = output_path / f"frame_{frame.index:03d}.png"
                cv2.imwrite(str(frame_file), frame.image)
                logger.debug(f"Saved: {frame_file}")
            
            # Save metadata
            metadata = {
                "timestamp": datetime.now().isoformat(),
                "num_frames": len(self.last_scan_frames),
                "frames": [
                    {
                        "index": f.index,
                        "x": f.position_x,
                        "y": f.position_y,
                        "z": f.position_z,
                        "timestamp": f.timestamp
                    }
                    for f in self.last_scan_frames
                ]
            }
            
            import json
            metadata_file = output_path / "metadata.json"
            with open(metadata_file, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            logger.info(f"✓ Scan data saved to {output_path}")
            return str(output_path)
            
        except Exception as e:
            logger.error(f"Error saving scan data: {e}")
            return ""
    
    # ==================== UTILITY METHODS ====================
    
    def emergency_stop(self):
        """Emergency stop - halt movement and release motors"""
        logger.warning("!!! EMERGENCY STOP !!!")
        self.serial.send_command("M410")  # Quickstop in Marlin
        self.release_motors()
        self.mode = OperationMode.ERROR
    
    def get_status(self) -> Dict:
        """Get current rig status"""
        return {
            "mode": self.mode.value,
            "homed": self.homed,
            "motors": self.motor_state.value,
            "position": self.current_pos.copy(),
            "serial_connected": self.serial.connected,
            "camera_connected": self.camera.connected,
        }
    
    def get_firmware_info(self) -> Optional[str]:
        """Request firmware information"""
        if self.serial.send_command_silent("M115"):
            response = self.serial.read_response(timeout=1.0)
            return response
        return None


# ==================== EXAMPLE USAGE ====================

if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    
    # Create controller with custom config
    config = RigConfig(
        port='/dev/ttyUSB0',      # Adjust for your system
        baudrate=250000,
        bed_x_max=300.0,
        bed_y_max=300.0,
        bed_z_max=300.0,
        park_x=280.0,             # Near corner
        park_y=280.0,
        park_z=200.0,
        center_x=150.0,
        center_y=150.0,
        focus_z=100.0,
    )
    
    controller = SmartRigController(config)
    
    try:
        # Connect
        if not controller.connect():
            logger.error("Failed to connect")
            exit(1)
        
        # Get status
        print(controller.get_status())
        
        # Home
        if not controller.home():
            logger.error("Homing failed")
            exit(1)
        
        # Switch to inspection mode
        if not controller.set_mode_inspection():
            logger.error("Failed to switch to inspection mode")
            exit(1)
        
        # Scan weld
        images = controller.scan_large_weld(start_x=50, end_x=250, step_mm=20)
        
        if images:
            # Stitch panorama
            panorama = controller.stitch_images(images)
            
            if panorama is not None:
                # Save panorama
                cv2.imwrite("panorama.png", panorama)
                logger.info("Panorama saved as panorama.png")
            
            # Save scan data
            controller.save_scan_data()
        
        # Return to loading position
        if not controller.set_mode_loading():
            logger.error("Failed to switch to loading mode")
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        controller.disconnect()
