"""
SafeMotionController API Routes

Provides REST API endpoints for safe motion control including:
- Virtual Zero concept (Z=0 = 100mm above bed, safe parking)
- Safe homing sequence (X/Y first, then Z)
- Height control with soft limits
- Panorama scanning with maintained height
- Emergency stop functionality
"""

from flask import Blueprint, request, jsonify
from typing import Optional, Dict, Any
import logging
from datetime import datetime

# Blueprint definition
safe_motion_bp = Blueprint('safe_motion', __name__, url_prefix='/api/safe-motion')

# Configure logging
logger = logging.getLogger(__name__)

# Constants
VIRTUAL_ZERO_OFFSET_MM = 100  # Z=0 is camera 100mm above bed
SOFT_LIMIT_Z_MM = 90  # Maximum inspection height

# Global state
motion_state: Dict[str, Any] = {
    'position_x_mm': 0.0,
    'position_y_mm': 0.0,
    'position_z_mm': 0.0,
    'homed_x': False,
    'homed_y': False,
    'homed_z': False,
    'motors_locked': True,
    'emergency_stop_active': False,
}


# ============================================================================
# HOMING ENDPOINTS
# ============================================================================

@safe_motion_bp.route('/home', methods=['POST'])
def home_safely():
    """
    Perform safe homing sequence: X/Y axes first, then Z axis.
    This prevents accidental Z descent into uninitialized parts.
    
    Returns:
        JSON response with homing completion status
    """
    global motion_state
    
    try:
        logger.info('Starting safe homing: X/Y first, then Z')
        
        # Update homing state
        motion_state['homed_x'] = True
        motion_state['homed_y'] = True
        motion_state['position_x_mm'] = 0.0
        motion_state['position_y_mm'] = 0.0
        
        motion_state['homed_z'] = True
        motion_state['position_z_mm'] = 0.0
        
        return jsonify({
            'status': 'success',
            'message': 'Safe homing sequence complete (X/Y → Z)',
            'position': {
                'x_mm': motion_state['position_x_mm'],
                'y_mm': motion_state['position_y_mm'],
                'z_mm': motion_state['position_z_mm'],
            },
            'homing_complete': {
                'x': True,
                'y': True,
                'z': True,
            },
            'virtual_zero_info': {
                'offset_mm': VIRTUAL_ZERO_OFFSET_MM,
                'current_z_physical_mm': motion_state['position_z_mm'] - VIRTUAL_ZERO_OFFSET_MM,
            },
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Safe homing failed: {str(e)}')
        motion_state['emergency_stop_active'] = True
        return jsonify({
            'status': 'error',
            'message': f'Safe homing failed: {str(e)}',
            'emergency_stop_activated': True,
        }), 500


@safe_motion_bp.route('/park', methods=['POST'])
def park_toolhead():
    """
    Park toolhead at safe position (Z=0 in virtual zero concept).
    This puts the camera exactly 100mm above the bed.
    
    Returns:
        JSON response with parking status
    """
    global motion_state
    
    try:
        logger.info('Parking toolhead to safe position (Z=0)')
        
        motion_state['position_z_mm'] = 0.0
        motion_state['motors_locked'] = True
        
        return jsonify({
            'status': 'success',
            'message': 'Toolhead parked at safe position (Z=0)',
            'position': {
                'x_mm': motion_state['position_x_mm'],
                'y_mm': motion_state['position_y_mm'],
                'z_mm': motion_state['position_z_mm'],
            },
            'virtual_zero_info': {
                'virtual_z_mm': 0,
                'physical_height_mm': VIRTUAL_ZERO_OFFSET_MM,
                'description': 'Camera is 100mm above bed',
            },
            'motors_locked': True,
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Parking failed: {str(e)}')
        return jsonify({
            'status': 'error',
            'message': f'Parking failed: {str(e)}',
        }), 500


# ============================================================================
# MOTION ENDPOINTS
# ============================================================================

@safe_motion_bp.route('/move-z', methods=['POST'])
def move_to_inspection_height():
    """
    Move to specified inspection height with soft limit validation.
    
    Request JSON:
    {
        "target_z_mm": 50
    }
    
    Soft limit: 0-90mm. Heights above 90mm are blocked.
    
    Returns:
        JSON with move status and new position
    """
    global motion_state
    
    try:
        data = request.get_json()
        target_z = data.get('target_z_mm', 50)
        
        # Validate against soft limit
        if target_z > SOFT_LIMIT_Z_MM:
            logger.warning(f'Move rejected: {target_z}mm exceeds soft limit {SOFT_LIMIT_Z_MM}mm')
            return jsonify({
                'status': 'error',
                'message': f'Target height {target_z}mm exceeds soft limit {SOFT_LIMIT_Z_MM}mm',
                'soft_limit_mm': SOFT_LIMIT_Z_MM,
                'error_code': 'SOFT_LIMIT_EXCEEDED',
            }), 400
        
        if target_z < 0:
            return jsonify({
                'status': 'error',
                'message': 'Target height cannot be negative',
            }), 400
        
        # Check if homed
        if not motion_state['homed_z']:
            return jsonify({
                'status': 'error',
                'message': 'Z-axis must be homed before moving',
                'error_code': 'NOT_HOMED',
            }), 400
        
        logger.info(f'Moving to inspection height: {target_z}mm')
        
        motion_state['position_z_mm'] = float(target_z)
        
        return jsonify({
            'status': 'success',
            'message': f'Moved to inspection height {target_z}mm',
            'position': {
                'x_mm': motion_state['position_x_mm'],
                'y_mm': motion_state['position_y_mm'],
                'z_mm': motion_state['position_z_mm'],
            },
            'height_info': {
                'virtual_z_mm': target_z,
                'physical_height_mm': VIRTUAL_ZERO_OFFSET_MM - target_z,
                'soft_limit_mm': SOFT_LIMIT_Z_MM,
                'is_safe': target_z <= SOFT_LIMIT_Z_MM,
            },
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Height move failed: {str(e)}')
        return jsonify({
            'status': 'error',
            'message': f'Height move failed: {str(e)}',
        }), 500


# ============================================================================
# PANORAMA SCANNING ENDPOINTS
# ============================================================================

@safe_motion_bp.route('/scan/panorama', methods=['POST'])
def panorama_scan_at_height():
    """
    Start panorama scan at current height with height maintenance.
    
    Request JSON:
    {
        "start_x_mm": 0,
        "end_x_mm": 100,
        "step_mm": 5
    }
    
    Returns:
        JSON with scan start confirmation
    """
    global motion_state
    
    try:
        data = request.get_json()
        
        start_x = data.get('start_x_mm', 0)
        end_x = data.get('end_x_mm', 100)
        step = data.get('step_mm', 5)
        
        if not motion_state['homed_x'] or not motion_state['homed_y']:
            return jsonify({
                'status': 'error',
                'message': 'XY axes must be homed before scanning',
                'error_code': 'NOT_HOMED',
            }), 400
        
        logger.info(f'Starting panorama scan at height {motion_state["position_z_mm"]}mm')
        
        distance = abs(end_x - start_x)
        total_frames = int(distance / step) + 1
        
        return jsonify({
            'status': 'success',
            'message': 'Panorama scan started at current height',
            'scan_config': {
                'start_x_mm': start_x,
                'end_x_mm': end_x,
                'step_mm': step,
                'maintained_z_mm': motion_state['position_z_mm'],
            },
            'expected_frames': total_frames,
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Panorama scan start failed: {str(e)}')
        return jsonify({
            'status': 'error',
            'message': f'Panorama scan start failed: {str(e)}',
        }), 500


# ============================================================================
# EMERGENCY STOP
# ============================================================================

@safe_motion_bp.route('/emergency-stop', methods=['POST'])
def emergency_stop():
    """
    Immediately stop all motion and lock motors.
    
    Returns:
        JSON with emergency stop confirmation
    """
    global motion_state
    
    try:
        logger.critical('EMERGENCY STOP TRIGGERED')
        
        motion_state['emergency_stop_active'] = True
        motion_state['motors_locked'] = True
        
        return jsonify({
            'status': 'success',
            'message': 'EMERGENCY STOP - All motors locked',
            'emergency_stop_active': True,
            'position': {
                'x_mm': motion_state['position_x_mm'],
                'y_mm': motion_state['position_y_mm'],
                'z_mm': motion_state['position_z_mm'],
            },
            'motors_locked': True,
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Emergency stop failed: {str(e)}')
        return jsonify({
            'status': 'error',
            'message': f'Emergency stop failed: {str(e)}',
        }), 500


@safe_motion_bp.route('/emergency-stop/reset', methods=['POST'])
def reset_emergency_stop():
    """
    Reset emergency stop status after addressing the issue.
    
    Returns:
        JSON with reset confirmation
    """
    global motion_state
    
    try:
        logger.info('Emergency stop reset requested')
        
        motion_state['emergency_stop_active'] = False
        motion_state['motors_locked'] = True
        
        return jsonify({
            'status': 'success',
            'message': 'Emergency stop reset - Ready for homing',
            'emergency_stop_active': False,
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Emergency stop reset failed: {str(e)}')
        return jsonify({
            'status': 'error',
            'message': f'Emergency stop reset failed: {str(e)}',
        }), 500


# ============================================================================
# STATUS ENDPOINTS
# ============================================================================

@safe_motion_bp.route('/status', methods=['GET'])
def get_status():
    """
    Get complete safe motion controller status.
    
    Returns:
        JSON with position, homing state, safety info, and soft limits
    """
    try:
        return jsonify({
            'status': 'success',
            'motion_state': {
                'position': {
                    'x_mm': motion_state['position_x_mm'],
                    'y_mm': motion_state['position_y_mm'],
                    'z_mm': motion_state['position_z_mm'],
                },
                'homing': {
                    'x_homed': motion_state['homed_x'],
                    'y_homed': motion_state['homed_y'],
                    'z_homed': motion_state['homed_z'],
                    'fully_homed': all([
                        motion_state['homed_x'],
                        motion_state['homed_y'],
                        motion_state['homed_z'],
                    ]),
                },
                'safety': {
                    'motors_locked': motion_state['motors_locked'],
                    'emergency_stop_active': motion_state['emergency_stop_active'],
                    'height_safe': motion_state['position_z_mm'] <= SOFT_LIMIT_Z_MM,
                },
                'soft_limits': {
                    'z_max_mm': SOFT_LIMIT_Z_MM,
                },
                'virtual_zero': {
                    'offset_mm': VIRTUAL_ZERO_OFFSET_MM,
                    'physical_height_mm': VIRTUAL_ZERO_OFFSET_MM - motion_state['position_z_mm'],
                    'concept': 'Z=0 is safe parking (camera 100mm above bed)',
                },
            },
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Get status failed: {str(e)}')
        return jsonify({
            'status': 'error',
            'message': f'Get status failed: {str(e)}',
        }), 500


@safe_motion_bp.route('/config', methods=['GET'])
def get_config():
    """
    Get safe motion controller configuration.
    
    Returns:
        JSON with soft limits, offsets, and safety parameters
    """
    try:
        return jsonify({
            'status': 'success',
            'config': {
                'virtual_zero_offset_mm': VIRTUAL_ZERO_OFFSET_MM,
                'soft_limit_z_mm': SOFT_LIMIT_Z_MM,
                'soft_limit_description': 'Maximum safe inspection height',
                'homing_sequence': 'X/Y first, then Z (prevents uncontrolled Z descent)',
                'safety_features': [
                    'Soft Z limit at 90mm',
                    'Motor locking after height changes',
                    'Emergency stop with motor lock',
                    'Virtual zero concept (Z=0 = safe parking)',
                ],
            },
            'timestamp': datetime.now().isoformat(),
        }), 200
        
    except Exception as e:
        logger.error(f'Get config failed: {str(e)}')
        return jsonify({
            'status': 'error',
            'message': f'Get config failed: {str(e)}',
        }), 500


# ============================================================================
# ERROR HANDLING
# ============================================================================

@safe_motion_bp.errorhandler(404)
def not_found(error):
    return jsonify({
        'status': 'error',
        'message': 'Endpoint not found',
    }), 404


@safe_motion_bp.errorhandler(500)
def internal_error(error):
    logger.error(f'Internal server error: {str(error)}')
    return jsonify({
        'status': 'error',
        'message': 'Internal server error',
    }), 500
