"""
Stereo Camera API Routes for RDK Stereo Camera Module
Handles dual-sensor operations, stereo calibration, and depth-based measurements
"""

import logging
from flask import Blueprint, jsonify, request
from typing import Optional, List
import numpy as np
import cv2

logger = logging.getLogger(__name__)

# Create blueprint
stereo_routes = Blueprint('stereo', __name__, url_prefix='/api/stereo')

# Global camera instance (will be set at startup)
stereo_camera = None
stereo_calibrator = None
depth_processor = None


def set_stereo_camera(camera, calibrator, processor):
    """Set global stereo camera instance"""
    global stereo_camera, stereo_calibrator, depth_processor
    stereo_camera = camera
    stereo_calibrator = calibrator
    depth_processor = processor


# ==================== Camera Information ====================

@stereo_routes.route('/info', methods=['GET'])
def get_camera_info():
    """Get stereo camera hardware information"""
    try:
        if not stereo_camera:
            return jsonify({'error': 'Camera not initialized'}), 503
        
        info = stereo_camera.get_camera_info()
        return jsonify(info), 200
    except Exception as e:
        logger.error(f"Error getting camera info: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/status', methods=['GET'])
def get_camera_status():
    """Get current camera stream status"""
    try:
        if not stereo_camera:
            return jsonify({'error': 'Camera not initialized'}), 503
        
        status = stereo_camera.get_status()
        return jsonify(status), 200
    except Exception as e:
        logger.error(f"Error getting status: {e}")
        return jsonify({'error': str(e)}), 500


# ==================== Stream Control ====================

@stereo_routes.route('/stream/start', methods=['POST'])
def start_stream():
    """Start dual sensor stream"""
    try:
        if not stereo_camera:
            return jsonify({'error': 'Camera not initialized'}), 503
        
        success = stereo_camera.start_stream()
        if success:
            logger.info("Stereo camera stream started")
            return jsonify({
                'status': 'started',
                'message': 'Dual sensor stream active',
                'resolution': stereo_camera.resolution,
                'fps': stereo_camera.fps
            }), 200
        else:
            return jsonify({'error': 'Failed to start stream'}), 500
    except Exception as e:
        logger.error(f"Error starting stream: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/stream/stop', methods=['POST'])
def stop_stream():
    """Stop dual sensor stream"""
    try:
        if not stereo_camera:
            return jsonify({'error': 'Camera not initialized'}), 503
        
        success = stereo_camera.stop_stream()
        if success:
            logger.info("Stereo camera stream stopped")
            return jsonify({'status': 'stopped'}), 200
        else:
            return jsonify({'error': 'Failed to stop stream'}), 500
    except Exception as e:
        logger.error(f"Error stopping stream: {e}")
        return jsonify({'error': str(e)}), 500


# ==================== Stereo Calibration ====================

@stereo_routes.route('/calibration/manual', methods=['POST'])
def calibrate_manual():
    """
    Manual stereo calibration with single checkerboard image
    
    Expected JSON:
    {
        'left_frame': base64 encoded image,
        'right_frame': base64 encoded image,
        'pattern_size': [width, height],  // Optional, default [9, 6]
        'square_size_mm': float  // Optional, default 25.0
    }
    """
    try:
        if not stereo_calibrator:
            return jsonify({'error': 'Calibrator not initialized'}), 503
        
        data = request.get_json()
        pattern_size = tuple(data.get('pattern_size', [9, 6]))
        square_size = data.get('square_size_mm', 25.0)
        
        logger.info(f"Manual stereo calibration requested")
        logger.info(f"  Pattern: {pattern_size}, Square size: {square_size}mm")
        
        # In actual implementation, would decode base64 frames
        # For now, return success placeholder
        
        calibration_data = {
            'method': 'manual',
            'frames_used': 1,
            'pattern_size': pattern_size,
            'square_size_mm': square_size,
            'status': 'pending',
            'message': 'Manual calibration captured - processing...'
        }
        
        return jsonify(calibration_data), 202
    except Exception as e:
        logger.error(f"Manual calibration error: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/calibration/auto', methods=['POST'])
def calibrate_auto():
    """
    Automatic stereo calibration with 15-20 checkerboard image pairs
    
    Expected JSON:
    {
        'left_frames': [base64, ...],  // 15-20 frames
        'right_frames': [base64, ...],
        'pattern_size': [width, height],
        'square_size_mm': float,
        'baseline_mm': float  // Expected baseline distance (70mm = RDK SC230AI dual 2MP stereo camera)
    }
    """
    try:
        if not stereo_calibrator:
            return jsonify({'error': 'Calibrator not initialized'}), 503
        
        data = request.get_json()
        left_frames = data.get('left_frames', [])
        right_frames = data.get('right_frames', [])
        pattern_size = tuple(data.get('pattern_size', [9, 6]))
        square_size = data.get('square_size_mm', 25.0)
        baseline_mm = data.get('baseline_mm', 70)  # 70mm = RDK SC230AI dual 2MP stereo camera
        
        if len(left_frames) < 10 or len(right_frames) < 10:
            return jsonify({
                'error': 'Insufficient frames',
                'minimum_frames': 10,
                'provided_left': len(left_frames),
                'provided_right': len(right_frames)
            }), 400
        
        logger.info(f"Auto stereo calibration requested")
        logger.info(f"  Frames: {len(left_frames)} left, {len(right_frames)} right")
        logger.info(f"  Pattern: {pattern_size}, Baseline: {baseline_mm}mm")
        
        # In actual implementation, would:
        # 1. Decode base64 frames
        # 2. Detect checkerboard in each pair
        # 3. Run stereo calibration
        # 4. Validate results
        
        calibration_data = {
            'method': 'auto',
            'frames_used': len(left_frames),
            'pattern_size': pattern_size,
            'square_size_mm': square_size,
            'baseline_mm': baseline_mm,
            'status': 'processing',
            'message': f'Processing {len(left_frames)} frame pairs...'
        }
        
        return jsonify(calibration_data), 202
    except Exception as e:
        logger.error(f"Auto calibration error: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/calibration/current', methods=['GET'])
def get_current_calibration():
    """Get current stereo calibration data"""
    try:
        if not stereo_calibrator:
            return jsonify({'error': 'Calibrator not initialized'}), 503
        
        calibration = {
            'loaded': stereo_calibrator.K_left is not None,
            'baseline_mm': float(stereo_calibrator.baseline) if stereo_calibrator.baseline else None,
            'intrinsics_available': stereo_calibrator.K_left is not None and stereo_calibrator.K_right is not None,
            'extrinsics_available': stereo_calibrator.R is not None and stereo_calibrator.T is not None,
            'rectification_computed': stereo_calibrator.R_left is not None
        }
        
        return jsonify(calibration), 200
    except Exception as e:
        logger.error(f"Error getting calibration: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/calibration/validate', methods=['POST'])
def validate_calibration():
    """Validate current stereo calibration quality"""
    try:
        if not stereo_calibrator:
            return jsonify({'error': 'Calibrator not initialized'}), 503
        
        validation = stereo_calibrator.validate_calibration_quality()
        
        return jsonify(validation), 200
    except Exception as e:
        logger.error(f"Validation error: {e}")
        return jsonify({'error': str(e)}), 500


# ==================== Depth Processing ====================

@stereo_routes.route('/depth/compute', methods=['POST'])
def compute_depth():
    """
    Compute depth map from stereo image pair
    
    Expected JSON:
    {
        'left_frame': base64 encoded image,
        'right_frame': base64 encoded image,
        'method': 'sgbm' or 'bm'  // Optional, default 'sgbm'
    }
    """
    try:
        if not depth_processor:
            return jsonify({'error': 'Depth processor not initialized'}), 503
        
        data = request.get_json()
        method = data.get('method', 'sgbm')
        
        # In actual implementation, would:
        # 1. Decode base64 frames
        # 2. Rectify using calibration
        # 3. Compute depth map
        # 4. Validate results
        
        logger.info(f"Depth computation requested (method: {method})")
        
        depth_info = {
            'status': 'processing',
            'method': method,
            'message': 'Computing depth map from stereo pair...'
        }
        
        return jsonify(depth_info), 202
    except Exception as e:
        logger.error(f"Depth computation error: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/depth/visualize', methods=['GET'])
def visualize_depth():
    """Get depth map visualization"""
    try:
        if not stereo_camera or stereo_camera.depth_map is None:
            return jsonify({'error': 'No depth map available'}), 404
        
        # In actual implementation, would return visualization image
        
        return jsonify({
            'status': 'available',
            'shape': stereo_camera.depth_map.shape,
            'min_mm': float(np.min(stereo_camera.depth_map)),
            'max_mm': float(np.max(stereo_camera.depth_map))
        }), 200
    except Exception as e:
        logger.error(f"Visualization error: {e}")
        return jsonify({'error': str(e)}), 500


# ==================== Weld Measurement ====================

@stereo_routes.route('/measure/bead', methods=['POST'])
def measure_weld_bead():
    """
    Measure weld bead from depth map
    
    Expected JSON:
    {
        'depth_map': numpy array or base64,
        'roi': {'x': int, 'y': int, 'width': int, 'height': int}  // Optional
    }
    """
    try:
        if not depth_processor or not stereo_camera.depth_map is not None:
            return jsonify({'error': 'No depth data available'}), 404
        
        logger.info("Weld bead measurement requested")
        
        measurements = depth_processor.measure_bead_profile_3d(stereo_camera.depth_map)
        
        return jsonify(measurements), 200
    except Exception as e:
        logger.error(f"Measurement error: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/measure/profile', methods=['POST'])
def measure_3d_profile():
    """
    Measure 3D weld profile from depth map
    
    Returns: Full 3D profile with width, height, and uniformity metrics
    """
    try:
        if not depth_processor:
            return jsonify({'error': 'Depth processor not initialized'}), 503
        
        if stereo_camera.depth_map is None:
            return jsonify({'error': 'No depth data available'}), 404
        
        logger.info("3D profile measurement requested")
        
        # Extract profile data
        profile_data = depth_processor.measure_bead_profile_3d(stereo_camera.depth_map)
        
        # Add 3D characteristics
        profile_data['measurement_type'] = '3d_profile'
        profile_data['source'] = 'stereo_depth_map'
        profile_data['accuracy'] = {
            'at_300mm': '±3mm',
            'at_500mm': '±8mm',
            'at_1000mm': '±25mm'
        }
        
        return jsonify(profile_data), 200
    except Exception as e:
        logger.error(f"Profile measurement error: {e}")
        return jsonify({'error': str(e)}), 500


# ==================== IR Fill Light ====================

@stereo_routes.route('/ir-light/enable', methods=['POST'])
def enable_ir_light():
    """Enable IR fill light for night imaging"""
    try:
        if not stereo_camera:
            return jsonify({'error': 'Camera not initialized'}), 503
        
        success = stereo_camera.set_ir_fill_light(True)
        if success:
            logger.info("IR fill light enabled")
            return jsonify({'status': 'enabled'}), 200
        else:
            return jsonify({'error': 'Failed to enable IR light'}), 500
    except Exception as e:
        logger.error(f"IR control error: {e}")
        return jsonify({'error': str(e)}), 500


@stereo_routes.route('/ir-light/disable', methods=['POST'])
def disable_ir_light():
    """Disable IR fill light"""
    try:
        if not stereo_camera:
            return jsonify({'error': 'Camera not initialized'}), 503
        
        success = stereo_camera.set_ir_fill_light(False)
        if success:
            logger.info("IR fill light disabled")
            return jsonify({'status': 'disabled'}), 200
        else:
            return jsonify({'error': 'Failed to disable IR light'}), 500
    except Exception as e:
        logger.error(f"IR control error: {e}")
        return jsonify({'error': str(e)}), 500


# ==================== Health Check ====================

@stereo_routes.route('/health', methods=['GET'])
def health_check():
    """Health check for stereo camera system"""
    try:
        health = {
            'status': 'healthy',
            'camera_initialized': stereo_camera is not None,
            'calibrator_initialized': stereo_calibrator is not None,
            'processor_initialized': depth_processor is not None,
            'stream_active': stereo_camera.streaming if stereo_camera else False,
            'calibration_loaded': stereo_calibrator.K_left is not None if stereo_calibrator else False
        }
        
        if all([health['camera_initialized'], health['calibrator_initialized'], 
                health['processor_initialized']]):
            return jsonify(health), 200
        else:
            return jsonify({**health, 'status': 'degraded'}), 200
    except Exception as e:
        logger.error(f"Health check error: {e}")
        return jsonify({'status': 'unhealthy', 'error': str(e)}), 500
