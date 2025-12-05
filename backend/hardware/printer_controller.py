"""
Two Trees CoreXY Printer Controller Module
Manages serial communication with Marlin firmware 3D printer used as scanning rig

WeldVision X5 - Automated Welding Evaluation System
Platform: RDK X5 (Ubuntu 20.04, ARM64)
"""

import serial
import time
import logging
from typing import Tuple, Optional, Dict, List
from dataclasses import dataclass
from enum import Enum
import threading
import numpy as np

logger = logging.getLogger(__name__)


class PrinterState(Enum):
    """Printer operational states"""
    IDLE = "idle"
    HOMING = "homing"
    MOVING = "moving"
    SCANNING = "scanning"
    ERROR = "error"
    DISCONNECTED = "disconnected"


@dataclass
class PrinterConfig:
    """Printer configuration parameters with triple Z-axis support"""
    port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    timeout: float = 10.0
    
    # Travel limits (mm)
    x_min: float = 0.0
    x_max: float = 235.0
    y_min: float = 0.0
    y_max: float = 235.0
    z_min: float = 0.0
    z_max: float = 250.0
    
    # Triple Z-axis support for bed tilt calibration
    enable_triple_z: bool = True
    z1_min: float = 0.0
    z1_max: float = 250.0
    z2_min: float = 0.0
    z2_max: float = 250.0
    z3_min: float = 0.0
    z3_max: float = 250.0
    
    # Scanning parameters
    feedrate_xy: int = 3000      # mm/min for XY moves
    feedrate_z: int = 800        # mm/min for Z moves
    probe_z_height: float = 10.0 # Height above workpiece for camera
    
    # Nozzle-to-camera offset (calibration)
    camera_offset_x: float = 0.0
    camera_offset_y: float = 0.0
    camera_offset_z: float = 50.0


class PrinterController:
    """
    Controls Two Trees CoreXY 3D printer via Marlin firmware
    
    Used as 3-axis scanning rig for welding inspection
    
    Example:
        controller = PrinterController()
        controller.connect()
        controller.home()
        controller.move_to(100, 100, 10)
        controller.disconnect()
    """
    
    def __init__(self, config: PrinterConfig = None):
        """Initialize printer controller"""
        self.config = config or PrinterConfig()
        self.serial: Optional[serial.Serial] = None
        self.state = PrinterState.DISCONNECTED
        self.current_pos = (0.0, 0.0, 0.0)  # X, Y, Z
        self.is_homed = False
        self.lock = threading.Lock()
        self.response_buffer = []
        self.listening = False
    
    def connect(self) -> bool:
        """
        Connect to printer via serial port
        
        Returns:
            True if connection successful
        """
        try:
            logger.info(f"Connecting to printer at {self.config.port} ({self.config.baud_rate} baud)")
            
            self.serial = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baud_rate,
                timeout=self.config.timeout,
                write_timeout=self.config.timeout
            )
            
            time.sleep(2)  # Wait for printer to respond
            
            # Start listener thread
            self.listening = True
            listener_thread = threading.Thread(target=self._listen_to_printer, daemon=True)
            listener_thread.start()
            
            # Get printer status
            response = self._send_command("M115")  # Get firmware version
            if response:
                logger.info(f"Printer connected: {response}")
                self.state = PrinterState.IDLE
                return True
            else:
                logger.error("No response from printer")
                self.disconnect()
                return False
                
        except serial.SerialException as e:
            logger.error(f"Serial connection failed: {e}")
            self.state = PrinterState.ERROR
            return False
    
    def disconnect(self) -> None:
        """Disconnect from printer"""
        try:
            self.listening = False
            if self.serial and self.serial.is_open:
                self.serial.close()
                logger.info("Printer disconnected")
            self.state = PrinterState.DISCONNECTED
        except Exception as e:
            logger.error(f"Error disconnecting: {e}")
    
    def is_connected(self) -> bool:
        """Check if printer is connected"""
        return self.serial is not None and self.serial.is_open
    
    def _send_command(self, command: str, wait_response: bool = True) -> Optional[str]:
        """
        Send G-code command to printer
        
        Args:
            command: G-code command (e.g., "G28" for home)
            wait_response: Wait for OK response
            
        Returns:
            Response from printer or None if timeout
        """
        if not self.is_connected():
            logger.error("Printer not connected")
            return None
        
        with self.lock:
            try:
                # Clear buffer
                if self.serial.in_waiting:
                    self.serial.read_all()
                
                # Send command
                command_line = command.strip() + "\n"
                logger.debug(f"Sending: {command}")
                self.serial.write(command_line.encode())
                
                if not wait_response:
                    return "QUEUED"
                
                # Wait for OK or error
                timeout_time = time.time() + self.config.timeout
                while time.time() < timeout_time:
                    if self.serial.in_waiting:
                        response = self.serial.readline().decode().strip()
                        if response:
                            logger.debug(f"Response: {response}")
                            if "ok" in response.lower():
                                return response
                            elif "error" in response.lower():
                                logger.error(f"Printer error: {response}")
                                self.state = PrinterState.ERROR
                                return None
                    time.sleep(0.05)
                
                logger.warning(f"Timeout waiting for response to: {command}")
                return None
                
            except Exception as e:
                logger.error(f"Error sending command: {e}")
                self.state = PrinterState.ERROR
                return None
    
    def _listen_to_printer(self) -> None:
        """Listen to printer output continuously (background thread)"""
        while self.listening:
            try:
                if self.is_connected() and self.serial.in_waiting:
                    response = self.serial.readline().decode().strip()
                    if response:
                        self.response_buffer.append(response)
                        if len(self.response_buffer) > 100:
                            self.response_buffer.pop(0)
                time.sleep(0.05)
            except Exception as e:
                logger.debug(f"Listener error: {e}")
    
    def home(self) -> bool:
        """
        Home all axes
        
        Returns:
            True if homing successful
        """
        try:
            logger.info("Homing printer...")
            self.state = PrinterState.HOMING
            
            response = self._send_command("G28")
            if response:
                self.current_pos = (0.0, 0.0, 0.0)
                self.is_homed = True
                self.state = PrinterState.IDLE
                logger.info("Homing complete")
                return True
            else:
                logger.error("Homing failed")
                self.state = PrinterState.ERROR
                return False
                
        except Exception as e:
            logger.error(f"Homing error: {e}")
            self.state = PrinterState.ERROR
            return False
    
    def move_to(self, x: float, y: float, z: float, feed_rate: Optional[int] = None) -> bool:
        """
        Move to absolute position
        
        Args:
            x, y, z: Target position in mm
            feed_rate: Speed in mm/min (optional, uses config if not specified)
            
        Returns:
            True if move successful
        """
        try:
            # Validate bounds
            if not self._validate_position(x, y, z):
                logger.error(f"Position out of bounds: ({x}, {y}, {z})")
                return False
            
            if not self.is_homed:
                logger.warning("Printer not homed, homing first...")
                if not self.home():
                    return False
            
            self.state = PrinterState.MOVING
            
            # Use provided feedrate or estimate based on movement
            if feed_rate is None:
                feed_rate = self.config.feedrate_xy
            
            # Build G-code command
            command = f"G0 X{x:.2f} Y{y:.2f} Z{z:.2f} F{feed_rate}"
            
            response = self._send_command(command)
            if response:
                self.current_pos = (x, y, z)
                self.state = PrinterState.IDLE
                logger.info(f"Moved to ({x:.2f}, {y:.2f}, {z:.2f})")
                return True
            else:
                logger.error(f"Move failed to ({x}, {y}, {z})")
                self.state = PrinterState.ERROR
                return False
                
        except Exception as e:
            logger.error(f"Move error: {e}")
            self.state = PrinterState.ERROR
            return False
    
    def move_relative(self, dx: float, dy: float, dz: float) -> bool:
        """
        Move relative to current position
        
        Args:
            dx, dy, dz: Relative movement in mm
            
        Returns:
            True if move successful
        """
        try:
            new_x = self.current_pos[0] + dx
            new_y = self.current_pos[1] + dy
            new_z = self.current_pos[2] + dz
            return self.move_to(new_x, new_y, new_z)
        except Exception as e:
            logger.error(f"Relative move error: {e}")
            return False
    
    def move_triple_z(self, x: float, y: float, z1: float, z2: float, z3: float) -> bool:
        """
        Move to position with independent triple Z-axis (for bed tilt calibration)
        
        Useful for calibrating RDK Stereo Camera with tilting bed.
        Each Z motor can be positioned independently for precise plane fitting.
        
        Args:
            x, y: XY position in mm
            z1, z2, z3: Independent Z-axis positions in mm
            
        Returns:
            True if move successful
        """
        try:
            if not self.config.enable_triple_z:
                logger.warning("Triple Z-axis not enabled")
                return False
            
            # Validate bounds for all Z axes
            if not (self.config.z1_min <= z1 <= self.config.z1_max):
                logger.error(f"Z1 out of bounds: {z1}")
                return False
            if not (self.config.z2_min <= z2 <= self.config.z2_max):
                logger.error(f"Z2 out of bounds: {z2}")
                return False
            if not (self.config.z3_min <= z3 <= self.config.z3_max):
                logger.error(f"Z3 out of bounds: {z3}")
                return False
            if not (self.config.x_min <= x <= self.config.x_max and
                    self.config.y_min <= y <= self.config.y_max):
                logger.error(f"XY position out of bounds: ({x}, {y})")
                return False
            
            if not self.is_homed:
                logger.warning("Printer not homed, homing first...")
                if not self.home():
                    return False
            
            self.state = PrinterState.MOVING
            
            # Move XY first
            xy_cmd = f"G0 X{x:.2f} Y{y:.2f} F{self.config.feedrate_xy}"
            if not self._send_command(xy_cmd):
                logger.error("XY movement failed")
                self.state = PrinterState.ERROR
                return False
            
            time.sleep(0.5)
            
            # Move Z axes independently
            # Using M400 to wait for moves to complete
            z1_cmd = f"G0 Z{z1:.2f} F{self.config.feedrate_z}"
            z2_cmd = f"G0 E{z2:.2f} F{self.config.feedrate_z}"  # Use E axis for second Z
            z3_cmd = f"G0 D{z3:.2f} F{self.config.feedrate_z}"  # Use D axis for third Z
            
            if not self._send_command(z1_cmd):
                logger.error("Z1 movement failed")
                self.state = PrinterState.ERROR
                return False
            
            if not self._send_command(z2_cmd):
                logger.error("Z2 movement failed")
                self.state = PrinterState.ERROR
                return False
            
            if not self._send_command(z3_cmd):
                logger.error("Z3 movement failed")
                self.state = PrinterState.ERROR
                return False
            
            # Wait for all moves to complete
            self._send_command("M400")
            
            # Update position (store as average Z for main tracking)
            avg_z = (z1 + z2 + z3) / 3.0
            self.current_pos = (x, y, avg_z)
            
            self.state = PrinterState.IDLE
            logger.info(f"Triple Z move to ({x:.2f}, {y:.2f}) Z1={z1:.2f} Z2={z2:.2f} Z3={z3:.2f}")
            return True
            
        except Exception as e:
            logger.error(f"Triple Z move error: {e}")
            self.state = PrinterState.ERROR
            return False
    
    def probe_plane_tilt(self, probe_points: List[Tuple[float, float]]) -> Optional[Dict]:
        """
        Probe multiple points to detect bed tilt plane
        
        Used for camera calibration with tilting bed.
        
        Args:
            probe_points: List of (x, y) positions to probe
            
        Returns:
            Dict with plane equation coefficients {a, b, c} where z = a*x + b*y + c
        """
        try:
            if not probe_points:
                logger.error("No probe points provided")
                return None
            
            logger.info(f"Probing {len(probe_points)} points for plane fitting...")
            
            probe_data = []
            
            # Touch-probe each point
            for i, (x, y) in enumerate(probe_points):
                # Move to position
                if not self.move_to(x, y, self.config.probe_z_height):
                    logger.error(f"Failed to move to probe point {i}")
                    continue
                
                # Probe down (G31 for Z-probe)
                response = self._send_command("G31")
                
                if response:
                    # Extract Z position (depends on firmware response format)
                    try:
                        # Typical Marlin response: "Z Probe: 10.50"
                        z_str = response.split(':')[-1].strip()
                        z = float(z_str)
                        probe_data.append((x, y, z))
                        logger.debug(f"Probe point {i}: ({x}, {y}, {z})")
                    except (ValueError, IndexError):
                        logger.warning(f"Could not parse probe response: {response}")
                
                time.sleep(0.2)
            
            if len(probe_data) < 3:
                logger.error("Need at least 3 probe points for plane fitting")
                return None
            
            # Fit plane to probe data using least squares
            import numpy as np
            points = np.array(probe_data)
            
            # Build system of equations: z = a*x + b*y + c
            A = np.column_stack([points[:, 0], points[:, 1], np.ones(len(points))])
            b = points[:, 2]
            
            # Solve using least squares
            coeffs, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            
            plane_eq = {
                'a': float(coeffs[0]),      # Slope in X
                'b': float(coeffs[1]),      # Slope in Y
                'c': float(coeffs[2]),      # Intercept
                'residual': float(np.mean(residuals)) if len(residuals) > 0 else 0.0,
                'num_points': len(probe_data)
            }
            
            logger.info(f"Plane fitted: z = {plane_eq['a']:.6f}*x + {plane_eq['b']:.6f}*y + {plane_eq['c']:.2f}")
            logger.info(f"Fit error (residual): {plane_eq['residual']:.4f}mm")
            
            return plane_eq
            
        except Exception as e:
            logger.error(f"Plane probing error: {e}")
            return None
    
    def calibrate_bed_tilt(self, calibration_grid: Tuple[int, int] = (3, 3)) -> Optional[Dict]:
        """
        Auto-calibrate bed tilt using triple Z-axis
        
        Probes bed at grid points, fits plane, then levels by adjusting Z motors.
        
        Args:
            calibration_grid: (rows, cols) for probe grid
            
        Returns:
            Dict with calibration results
        """
        try:
            logger.info(f"Starting bed tilt calibration with {calibration_grid[0]}x{calibration_grid[1]} grid")
            
            rows, cols = calibration_grid
            
            # Generate probe grid
            x_points = np.linspace(self.config.x_min + 20, self.config.x_max - 20, cols)
            y_points = np.linspace(self.config.y_min + 20, self.config.y_max - 20, rows)
            
            probe_points = [(x, y) for x in x_points for y in y_points]
            
            # Probe plane
            plane_eq = self.probe_plane_tilt(probe_points)
            if not plane_eq:
                logger.error("Plane fitting failed")
                return None
            
            # Calculate Z offsets for three corner points (triangle for stability)
            corners = [
                (self.config.x_min + 20, self.config.y_min + 20),  # Front-left
                (self.config.x_max - 20, self.config.y_min + 20),  # Front-right
                (self.config.x_min + 20, self.config.y_max - 20),  # Back-left
            ]
            
            z_offsets = []
            for x, y in corners:
                z = plane_eq['a'] * x + plane_eq['b'] * y + plane_eq['c']
                z_offsets.append(z)
            
            logger.info(f"Z offsets at corners: {[f'{z:.2f}' for z in z_offsets]}")
            
            # Move to calibration height using triple Z
            center_x = (self.config.x_min + self.config.x_max) / 2.0
            center_y = (self.config.y_min + self.config.y_max) / 2.0
            
            if not self.move_triple_z(center_x, center_y, z_offsets[0], z_offsets[1], z_offsets[2]):
                logger.error("Failed to move to calibrated position")
                return None
            
            return {
                'plane_equation': plane_eq,
                'z_offsets': z_offsets,
                'calibration_grid': calibration_grid,
                'status': 'success'
            }
            
        except Exception as e:
            logger.error(f"Calibration error: {e}")
            return None
    
    def _validate_position(self, x: float, y: float, z: float) -> bool:
        """Validate position is within bounds"""
        return (self.config.x_min <= x <= self.config.x_max and
                self.config.y_min <= y <= self.config.y_max and
                self.config.z_min <= z <= self.config.z_max)
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current position"""
        return self.current_pos
    
    def set_position(self, x: float, y: float, z: float) -> bool:
        """Manually set position (useful after homing)"""
        if self._validate_position(x, y, z):
            self.current_pos = (x, y, z)
            return True
        return False
    
    def generate_scan_grid(self, 
                          grid_x: Tuple[float, float, float],
                          grid_y: Tuple[float, float, float],
                          z_height: float,
                          spacing: float = 10.0) -> List[Tuple[float, float, float]]:
        """
        Generate scanning grid points
        
        Args:
            grid_x: (min, max, step) for X axis
            grid_y: (min, max, step) for Y axis
            z_height: Z height for scanning
            spacing: Distance between points in mm
            
        Returns:
            List of (x, y, z) scan positions
        """
        scan_points = []
        
        x_min, x_max, x_step = grid_x
        y_min, y_max, y_step = grid_y
        
        x = x_min
        while x <= x_max:
            y = y_min
            while y <= y_max:
                scan_points.append((x, y, z_height))
                y += spacing
            x += spacing
        
        logger.info(f"Generated {len(scan_points)} scan points")
        return scan_points
    
    def emergency_stop(self) -> bool:
        """Emergency stop - move Z to safe height and stop"""
        try:
            logger.warning("EMERGENCY STOP activated")
            self.state = PrinterState.ERROR
            response = self._send_command("M112")  # Emergency stop
            if response:
                self.move_to(self.current_pos[0], self.current_pos[1], 50)  # Move Z up
                return True
            return False
        except Exception as e:
            logger.error(f"Emergency stop error: {e}")
            return False
    
    def get_status(self) -> Dict[str, any]:
        """Get printer status"""
        return {
            'connected': self.is_connected(),
            'state': self.state.value,
            'position': self.current_pos,
            'homed': self.is_homed,
            'last_responses': self.response_buffer[-5:] if self.response_buffer else []
        }
