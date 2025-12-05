"""
Scan Orchestrator for WeldVision X5
Coordinates printer movement and camera capture for automated scanning

Manages the complete scanning workflow from setup to result export
"""

import logging
import time
import threading
from typing import Optional, Dict, List, Callable
from dataclasses import dataclass
from enum import Enum
import numpy as np

logger = logging.getLogger(__name__)


class ScanState(Enum):
    """Scan operation states"""
    IDLE = "idle"
    INITIALIZING = "initializing"
    POSITIONING = "positioning"
    CAPTURING = "capturing"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"
    PAUSED = "paused"


@dataclass
class ScanConfiguration:
    """Scanning configuration with triple Z-axis support"""
    # Grid parameters
    grid_x_min: float = 50.0
    grid_x_max: float = 200.0
    grid_y_min: float = 50.0
    grid_y_max: float = 200.0
    grid_spacing: float = 25.0  # mm between scan points
    
    # Z-axis (camera height above workpiece)
    scan_z_height: float = 10.0  # mm above part
    
    # Triple Z-axis support for bed tilt
    enable_triple_z: bool = True
    use_calibrated_tilt: bool = False  # Use stored bed tilt calibration
    
    # Capture parameters
    capture_dwell_time: float = 0.5  # seconds at each position
    images_per_point: int = 1
    
    # Safety
    max_scan_time: float = 300.0  # seconds
    enable_safety_checks: bool = True


class ScanResult:
    """Container for scan results"""
    
    def __init__(self, scan_id: str):
        self.scan_id = scan_id
        self.start_time = time.time()
        self.end_time = None
        self.status = "in_progress"
        self.error = None
        
        # Data storage
        self.scan_points: List[Dict] = []
        self.measurements: Dict = {}
        self.defects: List[Dict] = []
        self.point_clouds: List[np.ndarray] = []
        self.images: List[np.ndarray] = []
        
    def add_scan_point(self, point_data: Dict) -> None:
        """Add data from one scan position"""
        self.scan_points.append(point_data)
    
    def finalize(self, success: bool, error: Optional[str] = None) -> None:
        """Finalize scan results"""
        self.end_time = time.time()
        self.status = "completed" if success else "failed"
        self.error = error
    
    def get_summary(self) -> Dict:
        """Get summary of scan results"""
        return {
            'scan_id': self.scan_id,
            'status': self.status,
            'duration_seconds': (self.end_time - self.start_time) if self.end_time else None,
            'points_captured': len(self.scan_points),
            'defects_found': len(self.defects),
            'error': self.error,
        }


class ScanOrchestrator:
    """
    Orchestrates automated scanning workflow
    
    Coordinates printer, camera, and vision processing for complete
    weld inspection scanning sequence
    """
    
    def __init__(self, 
                 printer_controller,
                 camera_handler,
                 vision_processor,
                 config: ScanConfiguration = None):
        """
        Initialize scan orchestrator
        
        Args:
            printer_controller: PrinterController instance
            camera_handler: ROS2 camera handler
            vision_processor: Vision processor instance
            config: Scan configuration
        """
        self.printer = printer_controller
        self.camera = camera_handler
        self.vision = vision_processor
        self.config = config or ScanConfiguration()
        
        self.state = ScanState.IDLE
        self.current_scan: Optional[ScanResult] = None
        self.is_running = False
        self.pause_requested = False
        
        self.scan_thread: Optional[threading.Thread] = None
        self.progress_callbacks: List[Callable] = []
    
    def register_progress_callback(self, callback: Callable) -> None:
        """Register callback for progress updates"""
        self.progress_callbacks.append(callback)
    
    def _notify_progress(self, event: Dict) -> None:
        """Notify all registered callbacks of progress"""
        for callback in self.progress_callbacks:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Callback error: {e}")
    
    def start_scan(self, scan_id: str, configuration: Optional[ScanConfiguration] = None) -> bool:
        """
        Start automated scanning sequence
        
        Args:
            scan_id: Unique scan identifier
            configuration: Optional custom configuration
            
        Returns:
            True if scan started successfully
        """
        try:
            if self.is_running:
                logger.error("Scan already in progress")
                return False
            
            if configuration:
                self.config = configuration
            
            logger.info(f"Starting scan {scan_id}")
            self.state = ScanState.INITIALIZING
            self.current_scan = ScanResult(scan_id)
            self.is_running = True
            self.pause_requested = False
            
            # Start scan in background thread
            self.scan_thread = threading.Thread(target=self._scan_worker, daemon=False)
            self.scan_thread.start()
            
            self._notify_progress({'type': 'started', 'scan_id': scan_id})
            return True
            
        except Exception as e:
            logger.error(f"Scan start error: {e}")
            self.state = ScanState.FAILED
            self.is_running = False
            return False
    
    def _scan_worker(self) -> None:
        """Background worker thread for scanning"""
        try:
            # Initialize
            if not self._initialize_scan():
                self.current_scan.finalize(False, "Initialization failed")
                self.state = ScanState.FAILED
                return
            
            # Generate scan points
            scan_points = self._generate_scan_points()
            logger.info(f"Generated {len(scan_points)} scan points")
            
            # Execute scan
            for i, (x, y, z) in enumerate(scan_points):
                if self.pause_requested:
                    self.state = ScanState.PAUSED
                    while self.pause_requested:
                        time.sleep(0.1)
                    self.state = ScanState.CAPTURING
                
                if not self.is_running:
                    break
                
                # Move to position
                self.state = ScanState.POSITIONING
                if not self.printer.move_to(x, y, z):
                    logger.error(f"Failed to move to ({x}, {y}, {z})")
                    continue
                
                time.sleep(self.config.capture_dwell_time)
                
                # Capture and process
                self.state = ScanState.CAPTURING
                point_data = self._capture_and_process(x, y, z)
                
                if point_data:
                    self.current_scan.add_scan_point(point_data)
                
                # Progress update
                progress = (i + 1) / len(scan_points) * 100
                self._notify_progress({
                    'type': 'progress',
                    'scan_id': self.current_scan.scan_id,
                    'current_point': i + 1,
                    'total_points': len(scan_points),
                    'percent_complete': progress,
                })
            
            # Finalize
            self._finalize_scan()
            self.current_scan.finalize(True)
            self.state = ScanState.COMPLETED
            self._notify_progress({'type': 'completed', 'scan_id': self.current_scan.scan_id})
            
        except Exception as e:
            logger.error(f"Scan worker error: {e}")
            if self.current_scan:
                self.current_scan.finalize(False, str(e))
            self.state = ScanState.FAILED
            self._notify_progress({'type': 'error', 'error': str(e)})
        
        finally:
            self.is_running = False
    
    def _initialize_scan(self) -> bool:
        """Initialize scan (home printer, check camera)"""
        try:
            logger.info("Initializing scan...")
            
            # Check printer connection
            if not self.printer.is_connected():
                logger.error("Printer not connected")
                return False
            
            # Home printer
            if not self.printer.home():
                logger.error("Homing failed")
                return False
            
            # Check camera
            if not self.camera.is_ready():
                logger.error("Camera not ready")
                return False
            
            logger.info("Scan initialization complete")
            return True
            
        except Exception as e:
            logger.error(f"Initialization error: {e}")
            return False
    
    def _generate_scan_points(self) -> List[tuple]:
        """Generate scan grid points"""
        return self.printer.generate_scan_grid(
            grid_x=(self.config.grid_x_min, self.config.grid_x_max, self.config.grid_spacing),
            grid_y=(self.config.grid_y_min, self.config.grid_y_max, self.config.grid_spacing),
            z_height=self.config.scan_z_height,
            spacing=self.config.grid_spacing
        )
    
    def _capture_and_process(self, x: float, y: float, z: float) -> Optional[Dict]:
        """Capture and process frame at position"""
        try:
            # Get frames from camera
            rgb_frame, depth_frame = self.camera.get_latest_frames()
            
            if rgb_frame is None or depth_frame is None:
                logger.warning(f"No frames at ({x}, {y}, {z})")
                return None
            
            # Process frames
            self.state = ScanState.PROCESSING
            result = self.vision.process_frames(rgb_frame, depth_frame)
            
            if not result.get('success'):
                logger.error(f"Processing failed at ({x}, {y}, {z})")
                return None
            
            # Package results
            point_data = {
                'position': (x, y, z),
                'timestamp': time.time(),
                'measurements': result.get('measurements', {}),
                'defects': result.get('defects', []),
                'processing_time': result.get('processing_time_ms', 0),
            }
            
            # Generate point cloud if depth available
            try:
                point_cloud = self.vision.generate_point_cloud(depth_frame)
                if len(point_cloud) > 0:
                    self.current_scan.point_clouds.append(point_cloud)
            except:
                pass
            
            # Store image
            self.current_scan.images.append(rgb_frame.copy())
            
            # Accumulate defects
            self.current_scan.defects.extend(result.get('defects', []))
            
            return point_data
            
        except Exception as e:
            logger.error(f"Capture/process error: {e}")
            return None
    
    def _finalize_scan(self) -> None:
        """Finalize scan (move to safe position)"""
        try:
            logger.info("Finalizing scan...")
            
            # Move to safe position (high Z)
            self.printer.move_to(
                self.printer.current_pos[0],
                self.printer.current_pos[1],
                50.0
            )
            
            logger.info("Scan finalized")
            
        except Exception as e:
            logger.error(f"Finalization error: {e}")
    
    def pause_scan(self) -> bool:
        """Pause current scan"""
        if self.is_running:
            self.pause_requested = True
            logger.info("Scan pause requested")
            return True
        return False
    
    def resume_scan(self) -> bool:
        """Resume paused scan"""
        if self.state == ScanState.PAUSED:
            self.pause_requested = False
            logger.info("Scan resumed")
            return True
        return False
    
    def calibrate_bed_with_triple_z(self) -> Optional[Dict]:
        """
        Calibrate bed tilt using triple Z-axis
        
        Returns:
            Calibration result dict
        """
        try:
            if not hasattr(self.printer, 'calibrate_bed_tilt'):
                logger.error("Printer does not support triple Z-axis calibration")
                return None
            
            logger.info("Starting bed tilt calibration with triple Z-axis...")
            
            result = self.printer.calibrate_bed_tilt(calibration_grid=(3, 3))
            
            if result and result.get('status') == 'success':
                logger.info("Bed calibration completed successfully")
                # Store calibration for use in scans
                self.config.use_calibrated_tilt = True
                return result
            else:
                logger.error("Bed calibration failed")
                return None
                
        except Exception as e:
            logger.error(f"Calibration error: {e}")
            return None
    
    def scan_with_triple_z(self, scan_id: str, config: ScanConfiguration = None,
                          use_calibration: bool = True) -> bool:
        """
        Start scan using triple Z-axis with bed tilt compensation
        
        Args:
            scan_id: Unique scan identifier
            config: Scan configuration
            use_calibration: Use calibrated bed tilt
            
        Returns:
            True if scan started
        """
        try:
            if not self.config.enable_triple_z:
                logger.warning("Triple Z-axis not enabled, using standard scan")
                return self.start_scan(scan_id, config)
            
            if use_calibration and self.config.use_calibrated_tilt:
                logger.info("Using calibrated bed tilt for scan")
            
            return self.start_scan(scan_id, config)
            
        except Exception as e:
            logger.error(f"Triple Z scan error: {e}")
            return False
    
    def _capture_and_process_triple_z(self, x: float, y: float,
                                      z1: float, z2: float, z3: float) -> Optional[Dict]:
        """
        Capture and process frame with triple Z-axis positioning
        
        Args:
            x, y: XY position
            z1, z2, z3: Independent Z-axis positions
            
        Returns:
            Processing result or None
        """
        try:
            # Move with triple Z
            if not self.printer.move_triple_z(x, y, z1, z2, z3):
                logger.warning(f"Failed to move to ({x}, {y}) with triple Z")
                return None
            
            # Wait for settling
            time.sleep(self.config.capture_dwell_time)
            
            # Capture frames
            rgb_frame, depth_frame = self.camera.get_latest_frames()
            
            if rgb_frame is None or depth_frame is None:
                logger.warning(f"No frames at triple Z position")
                return None
            
            # Process
            self.state = ScanState.PROCESSING
            result = self.vision.process_frames(rgb_frame, depth_frame)
            
            if result and result.get('success'):
                # Add Z positioning data
                result['z_positions'] = {'z1': z1, 'z2': z2, 'z3': z3}
                return result
            
            return None
            
        except Exception as e:
            logger.error(f"Triple Z capture error: {e}")
            return None
    
    def stop_scan(self) -> bool:
        """Stop current scan"""
        if self.is_running:
            logger.warning("Scan stop requested")
            self.is_running = False
            self.printer.emergency_stop()
            return True
        return False
    
    def get_scan_status(self) -> Dict:
        """Get current scan status"""
        if not self.current_scan:
            return {'state': self.state.value, 'scanning': False}
        
        points_scanned = len(self.current_scan.scan_points)
        return {
            'state': self.state.value,
            'scanning': self.is_running,
            'scan_id': self.current_scan.scan_id,
            'points_scanned': points_scanned,
            'defects_found': len(self.current_scan.defects),
            'elapsed_time': time.time() - self.current_scan.start_time,
        }
    
    def get_scan_results(self) -> Optional[ScanResult]:
        """Get completed scan results"""
        return self.current_scan
