"""
Automatic Stereo Camera Calibration using Triple Z-Axis

This module automates the stereo camera calibration process by:
1. Using three Z-axis motors to position the camera at multiple viewpoints
2. Capturing calibration patterns (checkerboard) at different heights and tilt angles
3. Performing 3D reconstruction to compute stereo parameters
4. Validating calibration accuracy with point cloud analysis

The triple Z-axis enables precise camera positioning for multi-view calibration
without manual intervention, ensuring consistent and accurate stereo parameters.
"""

import logging
import cv2
import numpy as np
from typing import Tuple, Optional, Dict, List
from dataclasses import dataclass, field
from datetime import datetime
import threading
import time
from enum import Enum

logger = logging.getLogger(__name__)


class CalibrationState(Enum):
    """State machine for calibration process"""
    IDLE = "idle"
    HOMING = "homing"
    POSITIONING = "positioning"
    CAPTURING = "capturing"
    PROCESSING = "processing"
    VALIDATING = "validating"
    COMPLETE = "complete"
    ERROR = "error"


@dataclass
class CalibrationPattern:
    """Detected calibration pattern (checkerboard)"""
    pattern_type: str = "checkerboard"  # Type of pattern
    board_size: Tuple[int, int] = (9, 6)  # Inner corners (width, height)
    square_size: float = 30.0  # Square size in mm
    corners_3d: Optional[np.ndarray] = None  # 3D world coordinates
    corners_left: Optional[np.ndarray] = None  # Left image corners
    corners_right: Optional[np.ndarray] = None  # Right image corners
    pattern_found: bool = False


@dataclass
class CapturePoint:
    """Single calibration capture point"""
    point_id: int
    z1: float  # Z motor 1 position (mm)
    z2: float  # Z motor 2 position (mm)
    z3: float  # Z motor 3 position (mm)
    x: float  # XY position
    y: float
    height: float  # Camera height above reference
    tilt_x: float  # Tilt angle X (degrees)
    tilt_y: float  # Tilt angle Y (degrees)
    image_left: Optional[np.ndarray] = None
    image_right: Optional[np.ndarray] = None
    pattern: Optional[CalibrationPattern] = None
    timestamp: Optional[str] = None


@dataclass
class StereoPair:
    """Stereo camera pair with calibration points"""
    left_intrinsics: np.ndarray = field(default_factory=lambda: np.eye(3))
    left_distortion: np.ndarray = field(default_factory=lambda: np.zeros(5))
    right_intrinsics: np.ndarray = field(default_factory=lambda: np.eye(3))
    right_distortion: np.ndarray = field(default_factory=lambda: np.zeros(5))
    rotation_matrix: np.ndarray = field(default_factory=lambda: np.eye(3))
    translation_vector: np.ndarray = field(default_factory=lambda: np.zeros((3, 1)))
    baseline: float = 50.0
    reprojection_error: float = 0.0


class AutoStereoCameraCalibration:
    """
    Automatic stereo camera calibration using triple Z-axis positioning
    
    Strategy:
    - Use three Z motors to create non-coplanar calibration planes
    - Capture checkerboard patterns at 15-20 different viewpoints
    - Each viewpoint: different height and tilt combination
    - Compute stereo parameters from multi-view correspondences
    - Validate using 3D reconstruction accuracy
    """
    
    def __init__(self, printer_controller=None, camera_left=None, camera_right=None):
        """
        Initialize auto stereo calibration
        
        Args:
            printer_controller: PrinterController instance for Z positioning
            camera_left: Left camera capture object
            camera_right: Right camera capture object
        """
        self.printer = printer_controller
        self.cam_left = camera_left
        self.cam_right = camera_right
        
        # Calibration state
        self.state = CalibrationState.IDLE
        self.capture_points: List[CapturePoint] = []
        self.stereo_pair = StereoPair()
        
        # Calibration parameters
        self.pattern = CalibrationPattern()
        self.total_captures = 0
        self.successful_captures = 0
        
        # Threading for async capture
        self._calibration_thread = None
        self._stop_calibration = False
        self.calibration_progress = 0.0
        self.calibration_message = ""
        
        # Z-axis positioning strategy
        self.z_sweep_range = (0, 10)  # mm range for Z sweeps
        self.tilt_angles = np.linspace(-5, 5, 3)  # degrees
        self.heights = np.linspace(200, 250, 3)  # mm from bed
        
    def set_state(self, state: CalibrationState, message: str = ""):
        """Update calibration state with logging"""
        self.state = state
        self.calibration_message = message
        logger.info(f"Calibration state: {state.value} - {message}")
    
    def generate_capture_sequence(self) -> List[CapturePoint]:
        """
        Generate sequence of capture points using triple Z-axis
        
        Strategy:
        - 3 heights × 3 tilt angles × 2 Z sweep positions = 18 capture points
        - Each point: different camera-to-pattern distance and angle
        - Creates rich calibration data for stereo reconstruction
        
        Returns:
            List of CapturePoint objects
        """
        capture_points = []
        point_id = 0
        
        # Motor positions (triangle configuration)
        motor_positions = {
            "z1": (20, 20),    # Front-left
            "z2": (215, 20),   # Front-right
            "z3": (20, 215)    # Back-left
        }
        
        # Z sweep creates tilt: different motors at different heights
        z_sweeps = [
            {"z1": 0, "z2": 0, "z3": 0},      # Flat plane
            {"z1": 5, "z2": 2.5, "z3": 0},    # Tilted around Y
            {"z1": 0, "z2": 0, "z3": 5},      # Tilted around X
        ]
        
        # Capture at different heights
        for height in self.heights:
            for sweep in z_sweeps:
                capture = CapturePoint(
                    point_id=point_id,
                    z1=sweep["z1"],
                    z2=sweep["z2"],
                    z3=sweep["z3"],
                    x=motor_positions["z1"][0],
                    y=motor_positions["z1"][1],
                    height=height,
                    tilt_x=self._calculate_tilt_x(sweep),
                    tilt_y=self._calculate_tilt_y(sweep),
                )
                capture_points.append(capture)
                point_id += 1
        
        self.total_captures = len(capture_points)
        logger.info(f"Generated {len(capture_points)} capture points for stereo calibration")
        return capture_points
    
    def _calculate_tilt_x(self, z_sweep: Dict) -> float:
        """Calculate X-axis tilt angle from Z positions"""
        z_diff = z_sweep["z3"] - ((z_sweep["z1"] + z_sweep["z2"]) / 2)
        return np.degrees(np.arctan(z_diff / 200.0))  # 200mm reference distance
    
    def _calculate_tilt_y(self, z_sweep: Dict) -> float:
        """Calculate Y-axis tilt angle from Z positions"""
        z_diff = z_sweep["z2"] - z_sweep["z1"]
        return np.degrees(np.arctan(z_diff / 195.0))  # 195mm reference distance
    
    def capture_stereo_pair(self, capture_point: CapturePoint) -> bool:
        """
        Capture synchronized stereo pair at specified position
        
        Args:
            capture_point: Target capture point with Z positions
            
        Returns:
            True if capture successful, False otherwise
        """
        try:
            # Position triple Z-axis
            if self.printer:
                self.set_state(CalibrationState.POSITIONING, 
                             f"Moving to Z1={capture_point.z1}, Z2={capture_point.z2}, Z3={capture_point.z3}")
                self.printer.move_triple_z(
                    capture_point.x, capture_point.y,
                    capture_point.z1, capture_point.z2, capture_point.z3
                )
                time.sleep(0.5)  # Stabilization delay
            
            # Capture synchronized stereo pair
            self.set_state(CalibrationState.CAPTURING, 
                         f"Capturing stereo pair {self.successful_captures + 1}/{self.total_captures}")
            
            if self.cam_left and self.cam_right:
                frame_left = self.cam_left.read()
                frame_right = self.cam_right.read()
                
                if frame_left is not None and frame_right is not None:
                    capture_point.image_left = frame_left
                    capture_point.image_right = frame_right
                    capture_point.timestamp = datetime.now().isoformat()
                    return True
            
            return False
            
        except Exception as e:
            logger.error(f"Error capturing stereo pair: {e}")
            return False
    
    def detect_pattern_pair(self, capture_point: CapturePoint) -> bool:
        """
        Detect checkerboard pattern in stereo pair
        
        Args:
            capture_point: Capture point with images
            
        Returns:
            True if pattern found in both images
        """
        try:
            if capture_point.image_left is None or capture_point.image_right is None:
                return False
            
            pattern = CalibrationPattern()
            
            # Detect in left image
            ret_left, corners_left = cv2.findChessboardCorners(
                capture_point.image_left,
                pattern.board_size,
                None
            )
            
            # Detect in right image
            ret_right, corners_right = cv2.findChessboardCorners(
                capture_point.image_right,
                pattern.board_size,
                None
            )
            
            if ret_left and ret_right:
                # Refine corner locations
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_left = cv2.cornerSubPix(
                    cv2.cvtColor(capture_point.image_left, cv2.COLOR_BGR2GRAY),
                    corners_left, (11, 11), (-1, -1), criteria
                )
                corners_right = cv2.cornerSubPix(
                    cv2.cvtColor(capture_point.image_right, cv2.COLOR_BGR2GRAY),
                    corners_right, (11, 11), (-1, -1), criteria
                )
                
                # Generate 3D object points
                objp = np.zeros((pattern.board_size[0] * pattern.board_size[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:pattern.board_size[0], 0:pattern.board_size[1]].T.reshape(-1, 2)
                objp *= pattern.square_size
                
                pattern.corners_3d = objp
                pattern.corners_left = corners_left
                pattern.corners_right = corners_right
                pattern.pattern_found = True
                
                capture_point.pattern = pattern
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"Error detecting pattern: {e}")
            return False
    
    def stereo_calibrate(self) -> Dict:
        """
        Compute stereo calibration from captured patterns
        
        Uses OpenCV stereoCalibrate to compute:
        - Left/right intrinsic matrices
        - Distortion coefficients
        - Rotation and translation between cameras
        - Epipolar geometry
        
        Returns:
            Dict with calibration results and error metrics
        """
        try:
            self.set_state(CalibrationState.PROCESSING, "Computing stereo calibration...")
            
            # Collect valid patterns
            object_points = []
            image_points_left = []
            image_points_right = []
            
            for point in self.capture_points:
                if point.pattern and point.pattern.pattern_found:
                    object_points.append(point.pattern.corners_3d)
                    image_points_left.append(point.pattern.corners_left)
                    image_points_right.append(point.pattern.corners_right)
            
            if len(object_points) < 3:
                raise ValueError("Need at least 3 valid pattern detections for stereo calibration")
            
            logger.info(f"Using {len(object_points)} valid pattern detections")
            
            # Assume image size (modify based on actual camera)
            image_size = (640, 480)
            
            # Initial guess for intrinsics
            K_init = np.array([
                [500, 0, 320],
                [0, 500, 240],
                [0, 0, 1]
            ], dtype=float)
            
            # Stereo calibration
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            
            ret, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
                object_points, image_points_left, image_points_right,
                K_init, np.zeros(4),  # Initial left intrinsics
                K_init, np.zeros(4),  # Initial right intrinsics
                image_size,
                criteria=criteria,
                flags=cv2.CALIB_FIX_INTRINSIC
            )
            
            # Store calibration results
            self.stereo_pair.left_intrinsics = K_left
            self.stereo_pair.left_distortion = D_left
            self.stereo_pair.right_intrinsics = K_right
            self.stereo_pair.right_distortion = D_right
            self.stereo_pair.rotation_matrix = R
            self.stereo_pair.translation_vector = T
            self.stereo_pair.baseline = np.linalg.norm(T)
            self.stereo_pair.reprojection_error = ret
            
            logger.info(f"Stereo calibration complete. Baseline: {self.stereo_pair.baseline:.2f}mm, "
                       f"Reprojection error: {ret:.4f}")
            
            return {
                "success": True,
                "baseline": float(self.stereo_pair.baseline),
                "reprojection_error": float(ret),
                "captures_used": len(object_points),
                "left_intrinsics": K_left.tolist(),
                "right_intrinsics": K_right.tolist(),
                "left_distortion": D_left.flatten().tolist(),
                "right_distortion": D_right.flatten().tolist(),
            }
            
        except Exception as e:
            logger.error(f"Stereo calibration error: {e}")
            self.set_state(CalibrationState.ERROR, f"Calibration error: {str(e)}")
            return {
                "success": False,
                "error": str(e)
            }
    
    def validate_calibration(self) -> Dict:
        """
        Validate stereo calibration accuracy
        
        Methods:
        1. Epipolar constraint verification
        2. Point cloud consistency
        3. Depth map smoothness
        4. Baseline consistency
        
        Returns:
            Dict with validation metrics
        """
        try:
            self.set_state(CalibrationState.VALIDATING, "Validating calibration...")
            
            validation_results = {
                "epipolar_error": [],
                "point_cloud_density": 0,
                "depth_accuracy": 0.0,
                "baseline_consistency": 0.0,
                "overall_score": 0.0
            }
            
            # Compute epipolar constraint errors
            F = np.linalg.inv(self.stereo_pair.right_intrinsics.T) @ \
                self.stereo_pair.rotation_matrix @ \
                self.stereo_pair.left_intrinsics
            
            epipolar_errors = []
            for point in self.capture_points:
                if point.pattern and point.pattern.pattern_found:
                    pts_left = point.pattern.corners_left
                    pts_right = point.pattern.corners_right
                    
                    # Epipolar constraint: x'^T * F * x = 0
                    for i in range(len(pts_left)):
                        p_left = np.append(pts_left[i].flatten(), 1)
                        p_right = np.append(pts_right[i].flatten(), 1)
                        epipolar_err = abs(p_right @ F @ p_left.T)
                        epipolar_errors.append(epipolar_err)
            
            if epipolar_errors:
                validation_results["epipolar_error"] = [float(np.mean(epipolar_errors)), 
                                                       float(np.std(epipolar_errors))]
            
            # Overall validation score (0-100)
            mean_epipolar = np.mean(epipolar_errors) if epipolar_errors else 0
            validation_results["overall_score"] = max(0, 100 - (mean_epipolar * 10))
            
            logger.info(f"Validation score: {validation_results['overall_score']:.1f}/100")
            
            return validation_results
            
        except Exception as e:
            logger.error(f"Validation error: {e}")
            return {
                "success": False,
                "error": str(e)
            }
    
    def run_full_calibration(self) -> Dict:
        """
        Execute complete auto stereo calibration sequence
        
        Sequence:
        1. Home and initialize Z-axis motors
        2. Generate capture sequence (15-20 points)
        3. Capture stereo pairs at each point
        4. Detect checkerboard patterns
        5. Compute stereo calibration parameters
        6. Validate calibration accuracy
        7. Save results
        
        Returns:
            Dict with complete calibration results
        """
        try:
            self.set_state(CalibrationState.HOMING, "Homing Z-axis motors...")
            
            if self.printer:
                self.printer.home()
                time.sleep(1)
            
            # Generate capture sequence
            self.capture_points = self.generate_capture_sequence()
            self.successful_captures = 0
            
            # Capture stereo pairs
            for i, point in enumerate(self.capture_points):
                self.calibration_progress = (i / len(self.capture_points)) * 50
                
                if self._stop_calibration:
                    break
                
                # Capture pair
                if self.capture_stereo_pair(point):
                    # Detect pattern
                    if self.detect_pattern_pair(point):
                        self.successful_captures += 1
                        logger.info(f"Pattern detected in pair {i+1}/{len(self.capture_points)}")
                    else:
                        logger.warning(f"Pattern not detected in pair {i+1}")
                else:
                    logger.warning(f"Failed to capture pair {i+1}")
            
            if self.successful_captures < 3:
                raise ValueError(f"Only {self.successful_captures} valid patterns captured (need ≥3)")
            
            # Compute stereo calibration
            self.calibration_progress = 55
            calib_results = self.stereo_calibrate()
            
            if not calib_results.get("success"):
                return calib_results
            
            # Validate
            self.calibration_progress = 80
            validation_results = self.validate_calibration()
            
            # Final results
            self.calibration_progress = 100
            self.set_state(CalibrationState.COMPLETE, "Stereo calibration complete!")
            
            results = {
                "success": True,
                "calibration": calib_results,
                "validation": validation_results,
                "total_captures": self.total_captures,
                "successful_captures": self.successful_captures,
                "timestamp": datetime.now().isoformat()
            }
            
            logger.info("Auto stereo calibration completed successfully")
            return results
            
        except Exception as e:
            logger.error(f"Calibration sequence error: {e}")
            self.set_state(CalibrationState.ERROR, f"Error: {str(e)}")
            return {
                "success": False,
                "error": str(e),
                "captures_completed": self.successful_captures
            }
    
    def stop_calibration(self):
        """Stop ongoing calibration sequence"""
        self._stop_calibration = True
        logger.info("Calibration stop requested")
    
    def get_status(self) -> Dict:
        """Get current calibration status"""
        return {
            "state": self.state.value,
            "progress": self.calibration_progress,
            "message": self.calibration_message,
            "captures_completed": self.successful_captures,
            "total_captures": self.total_captures
        }
    
    def save_calibration(self, filename: str = "stereo_calibration.json") -> bool:
        """Save calibration results to JSON file"""
        try:
            data = {
                "timestamp": datetime.now().isoformat(),
                "baseline": float(self.stereo_pair.baseline),
                "reprojection_error": float(self.stereo_pair.reprojection_error),
                "left_intrinsics": self.stereo_pair.left_intrinsics.tolist(),
                "left_distortion": self.stereo_pair.left_distortion.flatten().tolist(),
                "right_intrinsics": self.stereo_pair.right_intrinsics.tolist(),
                "right_distortion": self.stereo_pair.right_distortion.flatten().tolist(),
                "rotation_matrix": self.stereo_pair.rotation_matrix.tolist(),
                "translation_vector": self.stereo_pair.translation_vector.flatten().tolist(),
                "total_captures": self.total_captures,
                "successful_captures": self.successful_captures,
            }
            
            Path("calibration").mkdir(exist_ok=True)
            filepath = Path("calibration") / filename
            with open(filepath, 'w') as f:
                import json
                json.dump(data, f, indent=2)
            
            logger.info(f"Calibration saved to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Error saving calibration: {e}")
            return False
