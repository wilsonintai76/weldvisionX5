"""
API Routes for Camera Calibration with Triple Z-Axis Support

Endpoints for bed tilt calibration, probe fitting, Z-motor offset calculation,
and automatic stereo camera calibration using triple Z-axis positioning
"""

from flask import Blueprint, request, jsonify
from sqlalchemy.orm import Session
import logging
from typing import Optional
import threading

from hardware.camera_calibration import CameraCalibration, BedTiltPlane
from hardware.printer_controller import PrinterController
from hardware.stereo_calibration_auto import AutoStereoCameraCalibration, CalibrationState

logger = logging.getLogger(__name__)

# Create blueprint
calibration_bp = Blueprint('calibration', __name__, url_prefix='/api/calibration')

# Global instances
calibration: Optional[CameraCalibration] = None
printer_controller: Optional[PrinterController] = None
stereo_calibrator: Optional[AutoStereoCameraCalibration] = None
calibration_thread: Optional[threading.Thread] = None


def initialize_calibration() -> bool:
    """Initialize calibration module"""
    global calibration
    try:
        calibration = CameraCalibration(calib_dir='calibration')
        logger.info("Camera calibration module initialized")
        return True
    except Exception as e:
        logger.error(f"Calibration initialization error: {e}")
        return False


@calibration_bp.route('/intrinsics', methods=['GET'])
def get_intrinsics():
    """Get camera intrinsic calibration"""
    try:
        if calibration is None:
            initialize_calibration()
        
        return jsonify({
            'left': calibration.intrinsics.left.to_dict(),
            'right': calibration.intrinsics.right.to_dict(),
            'baseline': calibration.intrinsics.baseline,
        }), 200
        
    except Exception as e:
        logger.error(f"Get intrinsics error: {e}")
        return {'error': str(e)}, 500


@calibration_bp.route('/intrinsics', methods=['POST'])
def set_intrinsics():
    """Update camera intrinsic parameters"""
    try:
        data = request.json
        
        if calibration is None:
            initialize_calibration()
        
        # Update left camera
        if 'left' in data:
            for key, value in data['left'].items():
                setattr(calibration.intrinsics.left, key, value)
        
        # Update right camera
        if 'right' in data:
            for key, value in data['right'].items():
                setattr(calibration.intrinsics.right, key, value)
        
        # Update baseline
        if 'baseline' in data:
            calibration.intrinsics.baseline = data['baseline']
        
        # Save to file
        if calibration.save_intrinsics():
            return jsonify({'status': 'success', 'message': 'Intrinsics updated'}), 200
        else:
            return {'error': 'Failed to save intrinsics'}, 500
            
    except Exception as e:
        logger.error(f"Set intrinsics error: {e}")
        return {'error': str(e)}, 500


@calibration_bp.route('/bed-tilt/probe', methods=['POST'])
def probe_bed_tilt():
    """
    Probe bed tilt at specified points
    
    Request body:
    {
        "probe_points": [[x1, y1], [x2, y2], ...],
        "use_printer": true,  # Use printer controller to move to points
        "reference_height": 10.0  # Target height above workpiece
    }
    """
    try:
        data = request.json
        probe_points = data.get('probe_points', [])
        use_printer = data.get('use_printer', True)
        reference_height = data.get('reference_height', 10.0)
        
        if not probe_points or len(probe_points) < 3:
            return {'error': 'Need at least 3 probe points'}, 400
        
        logger.info(f"Starting bed tilt probe with {len(probe_points)} points")
        
        if calibration is None:
            initialize_calibration()
        
        # Convert to probe measurements format
        probe_measurements = []
        
        if use_printer and printer_controller:
            # Use printer to auto-probe
            for i, (x, y) in enumerate(probe_points):
                # Move to point
                if not printer_controller.move_to(x, y, reference_height):
                    logger.warning(f"Failed to move to probe point {i}")
                    continue
                
                # Probe (G31 for Z-probe)
                import time
                time.sleep(0.5)  # Dwell
                
                response = printer_controller._send_command("G31")
                
                if response:
                    # Extract Z value from response
                    try:
                        z_str = response.split(':')[-1].strip().split()[0]
                        z = float(z_str)
                        probe_measurements.append({
                            'x': float(x),
                            'y': float(y),
                            'z_measured': z
                        })
                        logger.debug(f"Probe {i}: ({x}, {y}) -> Z={z}")
                    except (ValueError, IndexError):
                        logger.warning(f"Could not parse probe response: {response}")
        else:
            # Use provided measurements
            if 'measurements' in data:
                probe_measurements = data['measurements']
        
        if len(probe_measurements) < 3:
            return {'error': 'Need at least 3 valid probe measurements'}, 400
        
        # Fit plane
        plane = calibration.fit_bed_plane(probe_measurements)
        
        if not plane:
            return {'error': 'Failed to fit bed plane'}, 500
        
        # Calculate Z offsets for triple Z-axis
        z1, z2, z3 = calibration.calculate_z_offsets_triple(plane, reference_height)
        
        return jsonify({
            'plane_equation': {
                'a': plane.a,
                'b': plane.b,
                'c': plane.c,
            },
            'tilt_angles': {
                'tilt_x_degrees': plane.tilt_angle_x(),
                'tilt_y_degrees': plane.tilt_angle_y(),
            },
            'z_offsets': {
                'z1_mm': z1,
                'z2_mm': z2,
                'z3_mm': z3,
            },
            'residual_mm': plane.residual,
            'measured_points': len(probe_measurements),
        }), 200
        
    except Exception as e:
        logger.error(f"Probe bed tilt error: {e}")
        return {'error': str(e)}, 500


@calibration_bp.route('/bed-tilt/calibrate', methods=['POST'])
def calibrate_bed_auto():
    """
    Auto-calibrate bed tilt using printer controller
    
    Request body:
    {
        "grid_rows": 3,
        "grid_cols": 3,
        "reference_height": 10.0
    }
    """
    try:
        if printer_controller is None or not printer_controller.is_connected():
            return {'error': 'Printer not connected'}, 400
        
        data = request.json
        grid_rows = data.get('grid_rows', 3)
        grid_cols = data.get('grid_cols', 3)
        reference_height = data.get('reference_height', 10.0)
        
        logger.info(f"Starting auto bed calibration ({grid_rows}x{grid_cols})")
        
        # Use printer's calibrate method
        result = printer_controller.calibrate_bed_tilt((grid_rows, grid_cols))
        
        if not result or result.get('status') != 'success':
            return {'error': 'Calibration failed'}, 500
        
        # Extract and return results
        plane_eq = result['plane_equation']
        z_offsets = result['z_offsets']
        
        return jsonify({
            'status': 'success',
            'plane_equation': {
                'a': plane_eq['a'],
                'b': plane_eq['b'],
                'c': plane_eq['c'],
                'residual': plane_eq.get('residual', 0.0),
            },
            'z_offsets': {
                'z1_mm': z_offsets[0],
                'z2_mm': z_offsets[1],
                'z3_mm': z_offsets[2],
            },
            'message': 'Bed calibration complete',
        }), 200
        
    except Exception as e:
        logger.error(f"Auto calibration error: {e}")
        return {'error': str(e)}, 500


@calibration_bp.route('/bed-tilt/current', methods=['GET'])
def get_current_tilt():
    """Get current bed tilt calibration"""
    try:
        if calibration is None or calibration.bed_tilt is None:
            return {'error': 'No tilt calibration available'}, 404
        
        plane = calibration.bed_tilt
        
        return jsonify({
            'plane_equation': {
                'a': plane.a,
                'b': plane.b,
                'c': plane.c,
            },
            'tilt_angles': {
                'tilt_x_degrees': plane.tilt_angle_x(),
                'tilt_y_degrees': plane.tilt_angle_y(),
            },
            'z_offsets': plane.z_offsets if plane.z_offsets else [0, 0, 0],
            'residual_mm': plane.residual,
            'measured_points': plane.measured_points,
        }), 200
        
    except Exception as e:
        logger.error(f"Get tilt error: {e}")
        return {'error': str(e)}, 500


@calibration_bp.route('/bed-tilt/apply-triple-z', methods=['POST'])
def apply_triple_z_offsets():
    """
    Apply calculated triple Z-axis offsets to level bed
    
    Request body:
    {
        "z1_mm": 0.0,
        "z2_mm": 2.5,
        "z3_mm": 1.2
    }
    """
    try:
        if printer_controller is None or not printer_controller.is_connected():
            return {'error': 'Printer not connected'}, 400
        
        data = request.json
        z1 = data.get('z1_mm', 0.0)
        z2 = data.get('z2_mm', 0.0)
        z3 = data.get('z3_mm', 0.0)
        
        # Move to center with triple Z offsets
        center_x = (printer_controller.config.x_min + printer_controller.config.x_max) / 2.0
        center_y = (printer_controller.config.y_min + printer_controller.config.y_max) / 2.0
        
        if printer_controller.move_triple_z(center_x, center_y, z1, z2, z3):
            logger.info(f"Applied triple Z offsets: Z1={z1}, Z2={z2}, Z3={z3}")
            return jsonify({
                'status': 'success',
                'message': 'Triple Z offsets applied',
                'position': {'x': center_x, 'y': center_y, 'z1': z1, 'z2': z2, 'z3': z3}
            }), 200
        else:
            return {'error': 'Failed to apply offsets'}, 500
            
    except Exception as e:
        logger.error(f"Apply offsets error: {e}")
        return {'error': str(e)}, 500


@calibration_bp.route('/validate', methods=['POST'])
def validate_calibration():
    """
    Validate calibration accuracy
    
    Request body:
    {
        "test_points": [[x1, y1, z1], [x2, y2, z2], ...]
    }
    """
    try:
        if calibration is None or calibration.bed_tilt is None:
            return {'error': 'No calibration to validate'}, 404
        
        data = request.json
        test_points = data.get('test_points', [])
        
        if not test_points:
            return {'error': 'No test points provided'}, 400
        
        result = calibration.validate_calibration(test_points)
        
        return jsonify(result), 200 if result.get('status') == 'success' else 500
        
    except Exception as e:
        logger.error(f"Validation error: {e}")
        return {'error': str(e)}, 500


@calibration_bp.route('/report', methods=['GET'])
def get_calibration_report():
    """Get comprehensive calibration report"""
    try:
        if calibration is None:
            initialize_calibration()
        
        report = calibration.get_calibration_report()
        
        return jsonify(report), 200


# ==================== AUTOMATIC STEREO CALIBRATION ENDPOINTS ====================


@calibration_bp.route('/stereo/auto/start', methods=['POST'])
def start_auto_stereo_calibration():
    """
    Start automatic stereo camera calibration using triple Z-axis
    
    Initiates full calibration sequence:
    1. Generate 15-20 capture points with varying heights and tilts
    2. Position camera at each point using triple Z motors
    3. Capture stereo pairs and detect checkerboard patterns
    4. Compute stereo parameters (intrinsics, baseline, rotation, translation)
    5. Validate accuracy with epipolar constraints
    
    Returns:
        Dict with calibration job ID and status
    """
    global stereo_calibrator, calibration_thread
    
    try:
        # Initialize stereo calibrator if needed
        if stereo_calibrator is None:
            stereo_calibrator = AutoStereoCameraCalibration(
                printer_controller=printer_controller,
                camera_left=None,  # Inject actual camera objects
                camera_right=None
            )
        
        # Check if calibration already running
        if calibration_thread and calibration_thread.is_alive():
            return jsonify({
                'status': 'running',
                'message': 'Calibration already in progress',
                'progress': stereo_calibrator.get_status()
            }), 400
        
        # Start calibration in background thread
        calibration_thread = threading.Thread(
            target=stereo_calibrator.run_full_calibration,
            daemon=True
        )
        calibration_thread.start()
        
        return jsonify({
            'status': 'started',
            'message': 'Automatic stereo calibration started',
            'job_id': 'stereo_calib_' + str(id(stereo_calibrator))
        }), 202
        
    except Exception as e:
        logger.error(f"Error starting stereo calibration: {e}")
        return jsonify({'error': str(e)}), 500


@calibration_bp.route('/stereo/auto/status', methods=['GET'])
def get_stereo_calibration_status():
    """Get current auto stereo calibration status and progress"""
    try:
        if stereo_calibrator is None:
            return jsonify({
                'status': 'idle',
                'message': 'No calibration in progress'
            }), 200
        
        status = stereo_calibrator.get_status()
        status['calibration_running'] = calibration_thread and calibration_thread.is_alive()
        
        return jsonify(status), 200
        
    except Exception as e:
        logger.error(f"Error getting calibration status: {e}")
        return jsonify({'error': str(e)}), 500


@calibration_bp.route('/stereo/auto/results', methods=['GET'])
def get_stereo_calibration_results():
    """Get automatic stereo calibration results"""
    try:
        if stereo_calibrator is None:
            return jsonify({'error': 'No calibration completed'}), 404
        
        if stereo_calibrator.state != CalibrationState.COMPLETE:
            return jsonify({
                'status': 'not_ready',
                'state': stereo_calibrator.state.value,
                'message': 'Calibration not yet complete'
            }), 202
        
        results = {
            'status': 'complete',
            'baseline': float(stereo_calibrator.stereo_pair.baseline),
            'reprojection_error': float(stereo_calibrator.stereo_pair.reprojection_error),
            'left_intrinsics': stereo_calibrator.stereo_pair.left_intrinsics.tolist(),
            'left_distortion': stereo_calibrator.stereo_pair.left_distortion.flatten().tolist(),
            'right_intrinsics': stereo_calibrator.stereo_pair.right_intrinsics.tolist(),
            'right_distortion': stereo_calibrator.stereo_pair.right_distortion.flatten().tolist(),
            'rotation_matrix': stereo_calibrator.stereo_pair.rotation_matrix.tolist(),
            'translation_vector': stereo_calibrator.stereo_pair.translation_vector.flatten().tolist(),
            'total_captures': stereo_calibrator.total_captures,
            'successful_captures': stereo_calibrator.successful_captures,
            'capture_success_rate': stereo_calibrator.successful_captures / stereo_calibrator.total_captures * 100
        }
        
        return jsonify(results), 200
        
    except Exception as e:
        logger.error(f"Error getting calibration results: {e}")
        return jsonify({'error': str(e)}), 500


@calibration_bp.route('/stereo/auto/stop', methods=['POST'])
def stop_stereo_calibration():
    """Stop ongoing automatic stereo calibration"""
    try:
        if stereo_calibrator is None:
            return jsonify({'message': 'No calibration running'}), 200
        
        stereo_calibrator.stop_calibration()
        
        return jsonify({
            'status': 'stopped',
            'message': 'Calibration stop requested',
            'captures_completed': stereo_calibrator.successful_captures
        }), 200
        
    except Exception as e:
        logger.error(f"Error stopping calibration: {e}")
        return jsonify({'error': str(e)}), 500


@calibration_bp.route('/stereo/auto/save', methods=['POST'])
def save_stereo_calibration():
    """Save completed stereo calibration to file"""
    try:
        if stereo_calibrator is None:
            return jsonify({'error': 'No calibration data available'}), 404
        
        data = request.get_json() or {}
        filename = data.get('filename', 'stereo_calibration.json')
        
        if stereo_calibrator.save_calibration(filename):
            return jsonify({
                'status': 'saved',
                'filename': filename,
                'message': f'Calibration saved to {filename}'
            }), 200
        else:
            return jsonify({'error': 'Failed to save calibration'}), 500
            
    except Exception as e:
        logger.error(f"Error saving calibration: {e}")
        return jsonify({'error': str(e)}), 500


@calibration_bp.route('/stereo/auto/config', methods=['GET', 'POST'])
def stereo_calibration_config():
    """Get/set automatic stereo calibration configuration"""
    try:
        if stereo_calibrator is None:
            stereo_calibrator = AutoStereoCameraCalibration(
                printer_controller=printer_controller
            )
        
        if request.method == 'GET':
            return jsonify({
                'total_captures': stereo_calibrator.total_captures,
                'heights': stereo_calibrator.heights.tolist() if hasattr(stereo_calibrator.heights, 'tolist') else stereo_calibrator.heights,
                'tilt_angles': stereo_calibrator.tilt_angles.tolist() if hasattr(stereo_calibrator.tilt_angles, 'tolist') else stereo_calibrator.tilt_angles,
                'z_sweep_range': list(stereo_calibrator.z_sweep_range),
                'pattern_board_size': stereo_calibrator.pattern.board_size,
                'pattern_square_size': stereo_calibrator.pattern.square_size
            }), 200
        
        elif request.method == 'POST':
            data = request.get_json() or {}
            
            if 'heights' in data:
                import numpy as np
                stereo_calibrator.heights = np.array(data['heights'])
            if 'tilt_angles' in data:
                import numpy as np
                stereo_calibrator.tilt_angles = np.array(data['tilt_angles'])
            if 'z_sweep_range' in data:
                stereo_calibrator.z_sweep_range = tuple(data['z_sweep_range'])
            if 'pattern_board_size' in data:
                stereo_calibrator.pattern.board_size = tuple(data['pattern_board_size'])
            if 'pattern_square_size' in data:
                stereo_calibrator.pattern.square_size = data['pattern_square_size']
            
            return jsonify({
                'status': 'configured',
                'message': 'Stereo calibration configuration updated'
            }), 200
            
    except Exception as e:
        logger.error(f"Error with stereo calibration config: {e}")
        return jsonify({'error': str(e)}), 500
        
    except Exception as e:
        logger.error(f"Report error: {e}")
        return {'error': str(e)}, 500
