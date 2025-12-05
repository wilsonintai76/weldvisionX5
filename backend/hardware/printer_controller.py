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
    """Printer configuration parameters"""
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
