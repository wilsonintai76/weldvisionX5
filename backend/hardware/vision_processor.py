"""
Vision Processor Module for WeldVision X5
OpenCV-based measurement and defect detection for welding inspection

Uses RDK Stereo Camera (RGB + Depth) for comprehensive analysis
"""

import cv2
import numpy as np
import logging
from typing import Tuple, Optional, Dict, List
from dataclasses import dataclass
import time

logger = logging.getLogger(__name__)


@dataclass
class CameraCalibration:
    """Camera calibration parameters"""
    # Intrinsic parameters
    fx: float = 510.0          # Focal length X
    fy: float = 510.0          # Focal length Y
    cx: float = 320.0          # Principal point X
    cy: float = 240.0          # Principal point Y
    
    # Distortion coefficients
    k1: float = -0.05
    k2: float = 0.01
    p1: float = 0.0
    p2: float = 0.0
    
    # Baseline (stereo)
    baseline: float = 50.0     # mm between cameras


class VisionProcessor:
    """
    Processes RGB and depth images from RDK Stereo Camera
    
    Features:
    - Depth map processing
    - Point cloud generation
    - Weld defect detection
    - Height uniformity analysis
    - Gap detection
    """
    
    def __init__(self, calibration: CameraCalibration = None):
        """Initialize vision processor"""
        self.calibration = calibration or CameraCalibration()
        self.last_rgb = None
        self.last_depth = None
        self.processing_time = 0.0
    
    def process_frames(self, 
                       rgb_frame: np.ndarray,
                       depth_frame: np.ndarray) -> Dict[str, any]:
        """
        Process RGB and depth frames
        
        Args:
            rgb_frame: RGB image (H, W, 3)
            depth_frame: Depth map (H, W) in mm
            
        Returns:
            Dictionary with processed results
        """
        start_time = time.time()
        
        try:
            self.last_rgb = rgb_frame.copy()
            self.last_depth = depth_frame.copy()
            
            # Preprocessing
            rgb_processed = self._preprocess_rgb(rgb_frame)
            depth_processed = self._preprocess_depth(depth_frame)
            
            # Segmentation
            weld_mask = self._segment_weld_region(rgb_processed)
            
            # Measurement
            measurements = self._measure_weld(rgb_processed, depth_processed, weld_mask)
            
            # Defect detection
            defects = self._detect_defects(rgb_processed, depth_processed, weld_mask)
            
            self.processing_time = time.time() - start_time
            
            return {
                'success': True,
                'measurements': measurements,
                'defects': defects,
                'processing_time_ms': self.processing_time * 1000,
                'weld_mask': weld_mask,
            }
            
        except Exception as e:
            logger.error(f"Processing error: {e}")
            return {
                'success': False,
                'error': str(e),
                'processing_time_ms': (time.time() - start_time) * 1000,
            }
    
    def _preprocess_rgb(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess RGB frame"""
        try:
            # Denoise
            frame_denoised = cv2.fastNlMeansDenoisingColored(frame, None, h=10, hForColorComponents=10, templateWindowSize=7, searchWindowSize=21)
            
            # Apply CLAHE for contrast enhancement
            lab = cv2.cvtColor(frame_denoised, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            lab = cv2.merge((l, a, b))
            frame_enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
            
            return frame_enhanced
        except Exception as e:
            logger.warning(f"RGB preprocessing error: {e}")
            return frame
    
    def _preprocess_depth(self, depth_map: np.ndarray) -> np.ndarray:
        """Preprocess depth map"""
        try:
            # Remove invalid values (0 = no depth)
            depth_valid = depth_map.copy()
            depth_valid[depth_valid == 0] = np.nan
            
            # Bilateral filtering for smoothing while preserving edges
            depth_filtered = cv2.bilateralFilter(depth_map.astype(np.float32), 9, 75, 75)
            
            return depth_filtered
        except Exception as e:
            logger.warning(f"Depth preprocessing error: {e}")
            return depth_map
    
    def _segment_weld_region(self, rgb_frame: np.ndarray) -> np.ndarray:
        """
        Segment weld region from RGB image
        
        Returns:
            Binary mask (H, W) where weld = 255, background = 0
        """
        try:
            # Convert to HSV for better color segmentation
            hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)
            
            # Define color range for metal weld (silvery-gray)
            # Adjust these ranges based on your specific lighting
            lower_metal = np.array([0, 0, 50])
            upper_metal = np.array([180, 50, 200])
            
            mask = cv2.inRange(hsv, lower_metal, upper_metal)
            
            # Morphological operations to clean mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            return mask
        except Exception as e:
            logger.warning(f"Segmentation error: {e}")
            return np.zeros_like(rgb_frame[:, :, 0])
    
    def _measure_weld(self, 
                      rgb_frame: np.ndarray,
                      depth_frame: np.ndarray,
                      weld_mask: np.ndarray) -> Dict[str, any]:
        """
        Measure weld characteristics
        
        Returns:
            Dictionary with measurements
        """
        try:
            measurements = {}
            
            # Apply mask to depth
            depth_masked = depth_frame.copy()
            depth_masked[weld_mask == 0] = np.nan
            
            # Height statistics
            valid_depths = depth_masked[~np.isnan(depth_masked)]
            if len(valid_depths) > 0:
                measurements['height_mean'] = float(np.nanmean(valid_depths))
                measurements['height_std'] = float(np.nanstd(valid_depths))
                measurements['height_min'] = float(np.nanmin(valid_depths))
                measurements['height_max'] = float(np.nanmax(valid_depths))
                measurements['height_range'] = measurements['height_max'] - measurements['height_min']
            
            # Weld area
            weld_pixels = np.count_nonzero(weld_mask)
            measurements['weld_area_pixels'] = int(weld_pixels)
            
            # Find contours for width/length
            contours, _ = cv2.findContours(weld_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                measurements['weld_width_pixels'] = int(w)
                measurements['weld_length_pixels'] = int(h)
            
            # Uniformity score (1.0 = perfectly uniform)
            if measurements.get('height_std', 0) > 0:
                measurements['uniformity_score'] = max(0.0, 1.0 - (measurements['height_std'] / measurements['height_mean']))
            
            return measurements
            
        except Exception as e:
            logger.error(f"Measurement error: {e}")
            return {'error': str(e)}
    
    def _detect_defects(self,
                        rgb_frame: np.ndarray,
                        depth_frame: np.ndarray,
                        weld_mask: np.ndarray) -> List[Dict[str, any]]:
        """
        Detect weld defects (porosity, gaps, undercuts)
        
        Returns:
            List of detected defects
        """
        defects = []
        
        try:
            # Apply mask
            depth_masked = depth_frame.copy()
            depth_masked[weld_mask == 0] = np.nan
            
            # 1. Porosity detection (local depth discontinuities)
            # Use Laplacian to find edges
            depth_edges = cv2.Laplacian(depth_frame.astype(np.float32), cv2.CV_32F)
            depth_edges = np.abs(depth_edges)
            _, porosity_mask = cv2.threshold(depth_edges, 20, 255, cv2.THRESH_BINARY)
            
            porosity_pixels = np.count_nonzero(porosity_mask & weld_mask.astype(np.uint8))
            if porosity_pixels > 50:  # Threshold
                defects.append({
                    'type': 'porosity',
                    'severity': min(100, (porosity_pixels / 1000) * 100),
                    'location': 'weld_region',
                    'pixel_count': int(porosity_pixels)
                })
            
            # 2. Gap detection (areas with depth < baseline)
            if len(depth_frame[depth_frame > 0]) > 0:
                depth_mean = np.mean(depth_frame[depth_frame > 0])
                gap_threshold = depth_mean - 5  # mm below mean
                gap_mask = (depth_frame < gap_threshold) & (weld_mask > 0)
                gap_pixels = np.count_nonzero(gap_mask)
                
                if gap_pixels > 20:
                    defects.append({
                        'type': 'gap',
                        'severity': min(100, (gap_pixels / 500) * 100),
                        'location': self._find_defect_location(gap_mask),
                        'pixel_count': int(gap_pixels)
                    })
            
            # 3. Surface irregularity (high standard deviation in small regions)
            if np.count_nonzero(~np.isnan(depth_masked)) > 100:
                # Divide into regions and check uniformity
                h, w = depth_frame.shape
                region_size = 32
                
                for y in range(0, h - region_size, region_size):
                    for x in range(0, w - region_size, region_size):
                        region = depth_masked[y:y+region_size, x:x+region_size]
                        valid_region = region[~np.isnan(region)]
                        
                        if len(valid_region) > 20 and np.std(valid_region) > 8:
                            defects.append({
                                'type': 'surface_irregularity',
                                'severity': min(100, np.std(valid_region)),
                                'location': {'x': x, 'y': y},
                                'std_dev': float(np.std(valid_region))
                            })
            
        except Exception as e:
            logger.warning(f"Defect detection error: {e}")
        
        return defects
    
    def _find_defect_location(self, defect_mask: np.ndarray) -> Dict[str, float]:
        """Find centroid of defect"""
        try:
            coords = np.where(defect_mask > 0)
            if len(coords[0]) > 0:
                return {
                    'y': float(np.mean(coords[0])),
                    'x': float(np.mean(coords[1]))
                }
        except:
            pass
        return {'x': 0.0, 'y': 0.0}
    
    def generate_point_cloud(self, depth_frame: np.ndarray) -> np.ndarray:
        """
        Generate 3D point cloud from depth map
        
        Returns:
            Point cloud (N, 3) with X, Y, Z coordinates
        """
        try:
            h, w = depth_frame.shape
            
            # Create pixel coordinate grid
            x_px = np.arange(w)
            y_px = np.arange(h)
            xx, yy = np.meshgrid(x_px, y_px)
            
            # Deproject to 3D
            z = depth_frame
            x = (xx - self.calibration.cx) * z / self.calibration.fx
            y = (yy - self.calibration.cy) * z / self.calibration.fy
            
            # Stack into point cloud
            points = np.stack([x, y, z], axis=-1)
            points = points.reshape(-1, 3)
            
            # Remove invalid points (z = 0)
            valid_points = points[points[:, 2] > 0]
            
            logger.info(f"Generated point cloud with {len(valid_points)} points")
            return valid_points
            
        except Exception as e:
            logger.error(f"Point cloud generation error: {e}")
            return np.array([])
    
    def draw_measurements(self, frame: np.ndarray, measurements: Dict) -> np.ndarray:
        """Draw measurements on frame"""
        try:
            frame_copy = frame.copy()
            h, w = frame_copy.shape[:2]
            
            # Draw text with measurements
            y_offset = 30
            for key, value in measurements.items():
                if isinstance(value, float):
                    text = f"{key}: {value:.2f}"
                else:
                    text = f"{key}: {value}"
                cv2.putText(frame_copy, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                y_offset += 25
            
            return frame_copy
        except Exception as e:
            logger.warning(f"Draw error: {e}")
            return frame
    
    def get_last_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get last processed RGB and depth frames"""
        return self.last_rgb, self.last_depth
