"""
RDK Stereo Camera Calibration Module with Triple Z-Axis Tilt Support

Handles camera intrinsic calibration, extrinsic calibration, and bed tilt correction
for precise depth measurement on WeldVision X5 system.

Supports three independent Z-axis motors for bed leveling during calibration.
"""

import logging
import numpy as np
from typing import Tuple, Optional, Dict, List
from dataclasses import dataclass, asdict
import json
from pathlib import Path

logger = logging.getLogger(__name__)


@dataclass
class CameraIntrinsics:
    """Camera intrinsic calibration parameters"""
    fx: float = 510.0      # Focal length X (pixels)
    fy: float = 510.0      # Focal length Y (pixels)
    cx: float = 320.0      # Principal point X (pixels)
    cy: float = 240.0      # Principal point Y (pixels)
    k1: float = 0.0        # Radial distortion 1
    k2: float = 0.0        # Radial distortion 2
    p1: float = 0.0        # Tangential distortion 1
    p2: float = 0.0        # Tangential distortion 2
    
    def to_matrix(self) -> np.ndarray:
        """Convert to 3x3 camera matrix"""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
    
    def to_dict(self) -> dict:
        """Convert to dictionary"""
        return asdict(self)


@dataclass
class StereoCalibration:
    """Stereo camera calibration (two cameras)"""
    left: CameraIntrinsics
    right: CameraIntrinsics
    baseline: float = 50.0  # Distance between cameras (mm)
    rotation: np.ndarray = None  # Rotation matrix (3x3)
    translation: np.ndarray = None  # Translation vector (3,)
    
    def __post_init__(self):
        if self.rotation is None:
            self.rotation = np.eye(3)
        if self.translation is None:
            self.translation = np.array([self.baseline, 0, 0])


@dataclass
class BedTiltPlane:
    """Bed tilt plane equation and measurements"""
    a: float  # Slope coefficient X
    b: float  # Slope coefficient Y
    c: float  # Intercept
    residual: float = 0.0  # Fitting error (mm)
    measured_points: int = 0
    z_offsets: List[float] = None  # Z motor offsets
    
    def z_at(self, x: float, y: float) -> float:
        """Calculate Z at given XY position"""
        return self.a * x + self.b * y + self.c
    
    def tilt_angle_xy(self) -> float:
        """Get tilt angle in XY plane (degrees)"""
        magnitude = np.sqrt(self.a**2 + self.b**2)
        return np.degrees(np.arctan(magnitude))
    
    def tilt_angle_x(self) -> float:
        """Get tilt angle around X axis (degrees)"""
        return np.degrees(np.arctan(self.b))
    
    def tilt_angle_y(self) -> float:
        """Get tilt angle around Y axis (degrees)"""
        return np.degrees(np.arctan(self.a))


class CameraCalibration:
    """
    RDK Stereo Camera calibration with triple Z-axis support
    
    Handles:
    1. Intrinsic calibration (camera matrix)
    2. Stereo calibration (baseline, rotation, translation)
    3. Bed tilt detection and correction
    4. Triple Z-axis motor offset calculation for leveling
    """
    
    def __init__(self, calib_dir: str = "calibration"):
        """
        Initialize calibration manager
        
        Args:
            calib_dir: Directory to store calibration files
        """
        self.calib_dir = Path(calib_dir)
        self.calib_dir.mkdir(exist_ok=True)
        
        # Load or create default calibration
        self.intrinsics = self._load_or_create_intrinsics()
        self.stereo = None
        self.bed_tilt = None
    
    def _load_or_create_intrinsics(self) -> StereoCalibration:
        """Load intrinsics from file or create defaults"""
        calib_file = self.calib_dir / "intrinsics.json"
        
        if calib_file.exists():
            try:
                with open(calib_file) as f:
                    data = json.load(f)
                logger.info(f"Loaded intrinsics from {calib_file}")
                
                left = CameraIntrinsics(**data['left'])
                right = CameraIntrinsics(**data['right'])
                baseline = data.get('baseline', 50.0)
                
                return StereoCalibration(left, right, baseline)
            except Exception as e:
                logger.error(f"Failed to load intrinsics: {e}")
        
        # Create defaults for RDK Stereo Camera
        logger.info("Using default RDK Stereo Camera intrinsics")
        left = CameraIntrinsics(fx=510, fy=510, cx=320, cy=240)
        right = CameraIntrinsics(fx=510, fy=510, cx=320, cy=240)
        
        return StereoCalibration(left, right, baseline=50.0)
    
    def save_intrinsics(self) -> bool:
        """Save calibration to file"""
        try:
            calib_file = self.calib_dir / "intrinsics.json"
            
            data = {
                'left': self.intrinsics.left.to_dict(),
                'right': self.intrinsics.right.to_dict(),
                'baseline': self.intrinsics.baseline,
            }
            
            with open(calib_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            logger.info(f"Saved intrinsics to {calib_file}")
            return True
        except Exception as e:
            logger.error(f"Failed to save intrinsics: {e}")
            return False
    
    def calculate_depth(self, left_disparity: np.ndarray) -> np.ndarray:
        """
        Calculate depth from disparity
        
        depth = (baseline * fx) / disparity
        
        Args:
            left_disparity: Disparity map (pixels)
            
        Returns:
            Depth map (mm)
        """
        # Avoid division by zero
        valid = left_disparity > 0
        depth = np.zeros_like(left_disparity, dtype=np.float32)
        
        baseline_mm = self.intrinsics.baseline
        fx = self.intrinsics.left.fx
        
        depth[valid] = (baseline_mm * fx) / left_disparity[valid]
        
        return depth
    
    def undistort_points(self, points: np.ndarray, camera: str = 'left') -> np.ndarray:
        """
        Undistort 2D points using camera model
        
        Args:
            points: Nx2 array of (x, y) image coordinates
            camera: 'left' or 'right'
            
        Returns:
            Undistorted points
        """
        calib = self.intrinsics.left if camera == 'left' else self.intrinsics.right
        
        # Normalize coordinates
        x = (points[:, 0] - calib.cx) / calib.fx
        y = (points[:, 1] - calib.cy) / calib.fy
        
        # Apply distortion correction
        r2 = x**2 + y**2
        distortion = 1 + calib.k1 * r2 + calib.k2 * r2**2
        
        x_undist = x / distortion
        y_undist = y / distortion
        
        # Back to pixel coordinates
        undist_points = np.column_stack([
            x_undist * calib.fx + calib.cx,
            y_undist * calib.fy + calib.cy
        ])
        
        return undist_points
    
    def create_point_cloud(self, rgb_image: np.ndarray, depth_map: np.ndarray,
                          max_distance: float = 1000.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Create 3D point cloud from RGB and depth
        
        Args:
            rgb_image: RGB image (HxWx3)
            depth_map: Depth map in mm (HxW)
            max_distance: Maximum depth to include (mm)
            
        Returns:
            Tuple of (points_3d, colors) where points_3d is Nx3 XYZ in mm
        """
        height, width = depth_map.shape
        
        # Create pixel grid
        x = np.arange(width, dtype=np.float32)
        y = np.arange(height, dtype=np.float32)
        xx, yy = np.meshgrid(x, y)
        
        # Normalize coordinates
        calib = self.intrinsics.left
        x_norm = (xx - calib.cx) / calib.fx
        y_norm = (yy - calib.cy) / calib.fy
        
        # Get valid depth
        valid = (depth_map > 0) & (depth_map < max_distance)
        
        # Calculate 3D points
        z = depth_map[valid]
        x_3d = x_norm[valid] * z
        y_3d = y_norm[valid] * z
        
        points_3d = np.column_stack([x_3d, y_3d, z])
        colors = rgb_image[valid]
        
        return points_3d, colors
    
    def fit_bed_plane(self, probe_measurements: List[Dict]) -> Optional[BedTiltPlane]:
        """
        Fit plane to probe measurements
        
        Args:
            probe_measurements: List of dicts with 'x', 'y', 'z_measured' keys
            
        Returns:
            BedTiltPlane object
        """
        try:
            if len(probe_measurements) < 3:
                logger.error("Need at least 3 measurements for plane fitting")
                return None
            
            # Extract probe data
            data = np.array([[m['x'], m['y'], m['z_measured']] 
                            for m in probe_measurements])
            
            # Fit plane: z = a*x + b*y + c
            A = np.column_stack([data[:, 0], data[:, 1], np.ones(len(data))])
            b = data[:, 2]
            
            coeffs, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            
            plane = BedTiltPlane(
                a=float(coeffs[0]),
                b=float(coeffs[1]),
                c=float(coeffs[2]),
                residual=float(np.mean(residuals)) if len(residuals) > 0 else 0.0,
                measured_points=len(probe_measurements)
            )
            
            logger.info(f"Fitted plane: z = {plane.a:.6f}*x + {plane.b:.6f}*y + {plane.c:.2f}")
            logger.info(f"Tilt angles: X={plane.tilt_angle_x():.2f}°, Y={plane.tilt_angle_y():.2f}°")
            logger.info(f"Plane fit residual: {plane.residual:.4f}mm")
            
            self.bed_tilt = plane
            return plane
            
        except Exception as e:
            logger.error(f"Plane fitting error: {e}")
            return None
    
    def calculate_z_offsets_triple(self, plane: BedTiltPlane,
                                  reference_height: float = 10.0) -> Tuple[float, float, float]:
        """
        Calculate Z motor offsets for triple Z-axis leveling
        
        Three motors positioned at triangle corners for stable leveling.
        
        Args:
            plane: BedTiltPlane from fitting
            reference_height: Target height above workpiece (mm)
            
        Returns:
            Tuple of (z1, z2, z3) offsets in mm
        """
        try:
            # Define three corner positions (triangle for stability)
            # Assuming bed corners at (0,0), (235, 0), (0, 235)
            corners = np.array([
                [20, 20],        # Front-left
                [215, 20],       # Front-right
                [20, 215],       # Back-left
            ])
            
            z_offsets = []
            for x, y in corners:
                # Calculate Z at this corner from fitted plane
                z = plane.z_at(x, y)
                
                # Offset needed to reach reference height
                offset = reference_height - z
                z_offsets.append(offset)
            
            # Normalize so minimum offset is 0
            min_offset = min(z_offsets)
            z_offsets = [z - min_offset for z in z_offsets]
            
            logger.info(f"Z motor offsets: Z1={z_offsets[0]:.2f}mm, "
                       f"Z2={z_offsets[1]:.2f}mm, Z3={z_offsets[2]:.2f}mm")
            
            plane.z_offsets = z_offsets
            return tuple(z_offsets)
            
        except Exception as e:
            logger.error(f"Z offset calculation error: {e}")
            return (0.0, 0.0, 0.0)
    
    def validate_calibration(self, test_points: List[Tuple[float, float, float]]) -> Dict:
        """
        Validate calibration by checking depth accuracy at known distances
        
        Args:
            test_points: List of (x, y, expected_z) in mm
            
        Returns:
            Dict with validation metrics
        """
        try:
            if not self.bed_tilt:
                logger.warning("No bed plane calibration available")
                return {'status': 'no_plane'}
            
            errors = []
            for x, y, expected_z in test_points:
                measured_z = self.bed_tilt.z_at(x, y)
                error = abs(measured_z - expected_z)
                errors.append(error)
            
            result = {
                'status': 'success',
                'mean_error_mm': float(np.mean(errors)),
                'max_error_mm': float(np.max(errors)),
                'std_error_mm': float(np.std(errors)),
                'num_points': len(test_points)
            }
            
            logger.info(f"Calibration validation: "
                       f"Mean error={result['mean_error_mm']:.3f}mm, "
                       f"Max error={result['max_error_mm']:.3f}mm")
            
            return result
            
        except Exception as e:
            logger.error(f"Validation error: {e}")
            return {'status': 'error', 'error': str(e)}
    
    def get_calibration_report(self) -> Dict:
        """Get comprehensive calibration report"""
        report = {
            'intrinsics': {
                'left': self.intrinsics.left.to_dict(),
                'right': self.intrinsics.right.to_dict(),
                'baseline': self.intrinsics.baseline,
            },
            'bed_tilt': {
                'fitted': self.bed_tilt is not None,
            }
        }
        
        if self.bed_tilt:
            report['bed_tilt'].update({
                'coefficients': {'a': self.bed_tilt.a, 'b': self.bed_tilt.b, 'c': self.bed_tilt.c},
                'residual_mm': self.bed_tilt.residual,
                'tilt_angle_x_deg': self.bed_tilt.tilt_angle_x(),
                'tilt_angle_y_deg': self.bed_tilt.tilt_angle_y(),
                'z_offsets': self.bed_tilt.z_offsets,
                'measured_points': self.bed_tilt.measured_points,
            })
        
        return report
