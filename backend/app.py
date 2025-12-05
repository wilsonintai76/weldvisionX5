import threading
import json
import time
import logging
import os
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent))

from flask import Flask, jsonify, request
from flask_cors import CORS
from flask_sqlalchemy import SQLAlchemy
from system_check import HardwareDetector, DatabaseManager, setup_logging

# Setup logging
logger = setup_logging(logging.INFO)

# Init database manager and check/create database
db_manager = DatabaseManager(db_path='sqlite:///weld_data.db')
try:
    Session = db_manager.initialize()
except Exception as e:
    logger.error(f"Failed to initialize database: {e}")
    raise

session = Session()

# Import database models after Session is ready
from database.models import Student, Scan
from database.scan_models import (
    ScanConfiguration, ScanPoint, Measurement, Defect,
    ScanStatus, DefectType, Base as ScanBase
)
from vision.evaluator import WeldEvaluator
from vision.calibration import Calibrator

# Initialize hardware detector
hardware_detector = HardwareDetector()
hardware_status = hardware_detector.detect_all()

# Log hardware detection results
logger.info("=" * 60)
logger.info("HARDWARE DETECTION RESULTS")
logger.info("=" * 60)
for msg in hardware_status['detection_messages']:
    logger.info(msg)
logger.info("=" * 60)

# Optional ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

app = Flask(__name__)
CORS(app)

# Store hardware status globally for API access
app.config['HARDWARE_STATUS'] = hardware_status
app.config['DB_MANAGER'] = db_manager

# Init Vision Components
calibrator = Calibrator()
evaluator = WeldEvaluator(calibrator)

# Init LED Control
from api.led_routes import init_led_controller, get_led_blueprint
try:
    init_led_controller(app, mode="pwm")
    app.register_blueprint(get_led_blueprint())
    logger.info("LED Controller initialized successfully")
except Exception as e:
    logger.warning(f"LED Controller initialization failed: {e}")

# Init Scan Routes (WeldVision X5)
try:
    from api.scan_routes import scan_bp, initialize_hardware
    app.register_blueprint(scan_bp)
    
    # Initialize hardware for scanning
    initialize_hardware(session)
    logger.info("Scan routes registered and hardware initialized")
except Exception as e:
    logger.warning(f"Scan routes initialization failed: {e}")

# Init Calibration Routes (Triple Z-axis)
try:
    from api.calibration_routes import calibration_bp, initialize_calibration
    app.register_blueprint(calibration_bp)
    
    # Initialize calibration
    initialize_calibration()
    logger.info("Calibration routes registered and module initialized")
except Exception as e:
    logger.warning(f"Calibration routes initialization failed: {e}")

# ROS2 State
current_frame = None
current_depth = None
frame_lock = threading.Lock()
camera_thread = None

# Import optimized camera node for RDK X5
try:
    from api.optimized_camera_node import OptimizedCameraNode
    OPTIMIZED_CAMERA_AVAILABLE = True
except ImportError:
    OPTIMIZED_CAMERA_AVAILABLE = False
    
    # Fallback to original camera node
    class CameraNode(threading.Thread):
        def __init__(self):
            super().__init__()
            self.daemon = True
            self.running = True
            
        def run(self):
            if not ROS_AVAILABLE:
                logger.warning("ROS2 not available, camera thread will not start")
                return
            
            try:
                logger.info("Starting ROS2 camera node (legacy mode)...")
                rclpy.init()
                node = rclpy.create_node('weld_eval_backend')
                bridge = CvBridge()
                
                def img_cb(msg):
                    global current_frame
                    with frame_lock:
                        current_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
                        logger.debug("Frame received")
                        
                def depth_cb(msg):
                    global current_depth
                    with frame_lock:
                        current_depth = bridge.imgmsg_to_cv2(msg, "16UC1")
                        logger.debug("Depth data received")
                    
                node.create_subscription(Image, '/image_raw', img_cb, 10)
                node.create_subscription(Image, '/depth_raw', depth_cb, 10)
                logger.info("ROS2 camera node subscribed to /image_raw and /depth_raw")
                rclpy.spin(node)
            except Exception as e:
                logger.error(f"Camera node error: {e}")
                self.running = False

# Start camera thread if ROS2 is available
if ROS_AVAILABLE and hardware_status['ros2_available']:
    logger.info("Starting camera thread...")
    
    if OPTIMIZED_CAMERA_AVAILABLE:
        logger.info("Using optimized camera node for RDK X5")
        camera_thread = OptimizedCameraNode(
            node_name='weld_eval_backend',
            executor_type='MultiThreadedExecutor',
            num_threads=4
        )
    else:
        logger.info("Using legacy camera node")
        camera_thread = CameraNode()
    
    camera_thread.start()
    logger.info("Camera thread started")
    
    # Register cleanup on shutdown
    def cleanup_camera():
        if camera_thread and camera_thread.running:
            camera_thread.stop() if hasattr(camera_thread, 'stop') else setattr(camera_thread, 'running', False)
            logger.info("Camera thread stopped")
    
    import atexit
    atexit.register(cleanup_camera)
else:
    logger.warning("Camera thread NOT started - ROS2 or camera not available")

@app.route('/api/students', methods=['GET'])
def list_students():
    students = session.query(Student).all()
    return jsonify([{
        'id': s.id, 'name': s.name, 'student_id': s.student_id,
        'class_name': s.class_name, 'level': s.level
    } for s in students])

@app.route('/api/students', methods=['POST'])
def create_student():
    data = request.json
    try:
        new_s = Student(
            name=data['name'], 
            student_id=data['student_id'],
            class_name=data['class_name'],
            level=data.get('level', 'Novice')
        )
        session.add(new_s)
        session.commit()
        return jsonify({'id': new_s.id}), 201
    except Exception as e:
        session.rollback()
        return jsonify({'error': str(e)}), 400

@app.route('/api/students/<int:student_id>', methods=['PUT'])
def update_student(student_id):
    data = request.json
    try:
        student = session.query(Student).filter(Student.id == student_id).first()
        if not student:
            return jsonify({'error': 'Student not found'}), 404
        
        if 'name' in data:
            student.name = data['name']
        if 'student_id' in data:
            student.student_id = data['student_id']
        if 'class_name' in data:
            student.class_name = data['class_name']
        if 'level' in data:
            student.level = data['level']
        
        session.commit()
        return jsonify({
            'id': student.id, 'name': student.name, 'student_id': student.student_id,
            'class_name': student.class_name, 'level': student.level
        })
    except Exception as e:
        session.rollback()
        return jsonify({'error': str(e)}), 400

@app.route('/api/students/<int:student_id>', methods=['DELETE'])
def delete_student(student_id):
    try:
        student = session.query(Student).filter(Student.id == student_id).first()
        if not student:
            return jsonify({'error': 'Student not found'}), 404
        
        session.delete(student)
        session.commit()
        return jsonify({'success': True}), 200
    except Exception as e:
        session.rollback()
        return jsonify({'error': str(e)}), 400

@app.route('/api/scans', methods=['GET'])
def list_scans():
    scans = session.query(Scan).all()
    return jsonify([{
        'id': s.id,
        'student_id': s.student_id,
        'timestamp': s.timestamp.isoformat(),
        'total_score': s.total_score,
        'metrics': {
            'width_val': s.width_val,
            'height_val': s.height_val,
            'uniformity_score': s.uniformity_score,
            'porosity_count': s.porosity_count,
            'spatter_count': s.spatter_count,
            'undercut_detected': s.undercut_detected,
        },
        'defects_json': json.loads(s.defects_json) if s.defects_json else [],
        'image_path': s.image_path,
        'status': s.status
    } for s in scans])

@app.route('/api/scan', methods=['POST'])
def scan():
    data = request.json
    student_id = data.get('student_id')
    rubric = data.get('rubric', {})
    
    # Get frame
    img = None
    depth = None
    
    with frame_lock:
        if current_frame is not None:
            img = current_frame.copy()
            depth = current_depth.copy() if current_depth is not None else None
    
    if img is None:
        # Generate Mock Data if no camera
        import numpy as np
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.ones((480, 640), dtype=np.uint16) * 1000

    result = evaluator.process_scan(img, depth, rubric)
    
    # Persist
    new_scan = Scan(
        student_id=student_id,
        total_score=result['score'],
        width_val=result['metrics']['width_val'],
        height_val=result['metrics']['height_val'],
        uniformity_score=result['metrics']['uniformity_score'],
        porosity_count=result['metrics']['porosity_count'],
        spatter_count=result['metrics']['spatter_count'],
        undercut_detected=result['metrics']['undercut_detected'],
        defects_json=json.dumps(result['defects']),
        status=result['status'],
        image_path="/static/placeholder.jpg"
    )
    session.add(new_scan)
    session.commit()
    
    resp = {
        'id': new_scan.id,
        'student_id': student_id,
        'timestamp': new_scan.timestamp.isoformat(),
        'total_score': result['score'],
        'metrics': result['metrics'],
        'defects_json': result['defects'],
        'image_path': "/static/placeholder.jpg",
        'status': result['status']
    }
    return jsonify(resp), 201

@app.route('/api/rubric', methods=['GET'])
def get_rubric():
    # Return default rubric
    return jsonify({
        'targetWidth': 8.0,
        'widthTolerance': 1.0,
        'targetHeight': 2.0,
        'heightTolerance': 0.5,
        'maxPorosity': 0,
        'maxSpatter': 2
    })

@app.route('/api/rubric', methods=['POST'])
def save_rubric():
    data = request.json
    # In a real implementation, save to database or config file
    # For now, just echo back the rubric
    return jsonify(data), 200

@app.route('/api/calibrate', methods=['POST'])
def calibrate():
    try:
        # In a real implementation, this would capture frames and run calibration
        # For now, return mock calibration data
        matrix = [
            [1024.5, 0, 640.0],
            [0, 1024.5, 360.0],
            [0, 0, 1]
        ]
        dist_coeffs = [-0.102, 0.045, -0.001, 0.002, 0]
        
        return jsonify({
            'matrix': matrix,
            'distCoeffs': dist_coeffs,
            'error': 0.23
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/calibrate/save', methods=['POST'])
def save_calibrate():
    try:
        data = request.json
        if calibrator:
            # Extract matrix and dist_coeffs
            import numpy as np
            matrix = np.array(data.get('matrix', []))
            dist_coeffs = np.array(data.get('distCoeffs', []))
            calibrator.save_config(matrix, dist_coeffs, 0.23)
        return jsonify({'success': True}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/health', methods=['GET'])
def health():
    """System health check including hardware and database status"""
    hardware_info = app.config.get('HARDWARE_STATUS', {})
    db_info = app.config.get('DB_MANAGER').get_db_status() if app.config.get('DB_MANAGER') else {}
    
    return jsonify({
        'status': 'ok',
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'database': {
            'path': db_info.get('db_path'),
            'exists': db_info.get('db_exists'),
            'ready': db_info.get('db_ready'),
        },
        'hardware': {
            'system': hardware_info.get('system', {}),
            'rdk_x5_available': hardware_info.get('rdk_x5_available', False),
            'ros2_available': hardware_info.get('ros2_available', False),
            'camera_available': hardware_info.get('camera_available', False),
            'rdk_camera_present': hardware_info.get('rdk_camera_present', False),
            'ready_for_operation': hardware_info.get('ready_for_operation', False),
        },
        'camera': {
            'frame_available': current_frame is not None,
            'depth_available': current_depth is not None,
            'camera_thread_running': camera_thread is not None and camera_thread.is_alive() if camera_thread else False,
        },
        'messages': hardware_info.get('detection_messages', []) + db_info.get('init_messages', []),
    }), 200

@app.route('/api/system/diagnostics', methods=['GET'])
def system_diagnostics():
    """Detailed system diagnostics for troubleshooting"""
    logger.info("System diagnostics requested")
    
    hardware_info = app.config.get('HARDWARE_STATUS', {})
    db_info = app.config.get('DB_MANAGER').get_db_status() if app.config.get('DB_MANAGER') else {}
    
    # Count database entries
    student_count = session.query(Student).count()
    scan_count = session.query(Scan).count()
    
    diagnostics = {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'system': hardware_info.get('system', {}),
        'database': {
            'path': db_info.get('db_path'),
            'exists': db_info.get('db_exists'),
            'ready': db_info.get('db_ready'),
            'student_count': student_count,
            'scan_count': scan_count,
            'messages': db_info.get('init_messages', []),
        },
        'hardware': {
            'rdk_x5_available': hardware_info.get('rdk_x5_available', False),
            'ros2_available': hardware_info.get('ros2_available', False),
            'camera_available': hardware_info.get('camera_available', False),
            'rdk_camera_present': hardware_info.get('rdk_camera_present', False),
            'ready_for_operation': hardware_info.get('ready_for_operation', False),
            'messages': hardware_info.get('detection_messages', []),
        },
        'camera': {
            'ros2_thread_running': ROS_AVAILABLE and camera_thread is not None and camera_thread.is_alive() if camera_thread else False,
            'frame_available': current_frame is not None,
            'depth_available': current_depth is not None,
        },
        'api': {
            'endpoints': [
                'GET /api/health',
                'GET /api/system/diagnostics',
                'GET /api/ros2/health',
                'GET /api/students',
                'POST /api/students',
                'PUT /api/students/<id>',
                'DELETE /api/students/<id>',
                'GET /api/scans',
                'POST /api/scan',
                'GET /api/rubric',
                'POST /api/rubric',
                'POST /api/calibrate',
                'POST /api/calibrate/save',
            ],
        },
    }
    
    logger.info(f"Diagnostics: Database OK={db_info.get('db_ready')}, "
                f"Hardware OK={hardware_info.get('ready_for_operation')}, "
                f"Students={student_count}, Scans={scan_count}")
    
    return jsonify(diagnostics), 200


@app.route('/api/ros2/health', methods=['GET'])
def ros2_health():
    """ROS2 and camera node health monitoring"""
    try:
        health_data = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'ros2_available': ROS_AVAILABLE and hardware_status.get('ros2_available', False),
            'camera_thread_running': camera_thread is not None and camera_thread.is_alive() if camera_thread else False,
            'camera_mode': 'optimized' if OPTIMIZED_CAMERA_AVAILABLE else 'legacy',
        }
        
        # Add camera node health if using optimized version
        if hasattr(camera_thread, 'get_health_status'):
            try:
                health_data['node_health'] = camera_thread.get_health_status()
            except Exception as e:
                logger.warning(f"Failed to get node health: {e}")
        
        # Add current frame info
        health_data['current_frame'] = {
            'available': current_frame is not None,
            'shape': current_frame.shape if current_frame is not None else None,
        }
        
        health_data['current_depth'] = {
            'available': current_depth is not None,
            'shape': current_depth.shape if current_depth is not None else None,
        }
        
        return jsonify(health_data), 200
        
    except Exception as e:
        logger.error(f"ROS2 health check failed: {e}")
        return jsonify({'error': str(e), 'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')}), 500


if __name__ == '__main__':
    # Get info for startup logging
    db_info = db_manager.get_db_status()
    
    logger.info("=" * 70)
    logger.info("WeldMaster AI - Automated Welding Evaluation System")
    logger.info("=" * 70)
    logger.info(f"Database: {db_info.get('db_path')} - {'EXISTS' if db_info.get('db_exists') else 'CREATED'}")
    logger.info(f"RDK X5: {'DETECTED' if hardware_status['rdk_x5_available'] else 'NOT DETECTED'}")
    logger.info(f"ROS2: {'AVAILABLE' if hardware_status['ros2_available'] else 'NOT AVAILABLE'}")
    logger.info(f"Camera: {'CONNECTED' if hardware_status['camera_available'] else 'NOT CONNECTED'}")
    logger.info(f"Ready for Operation: {'YES' if hardware_status['ready_for_operation'] else 'NO'}")
    logger.info("=" * 70)
    logger.info("Starting Flask API server on http://0.0.0.0:5000")
    logger.info("Health Check: http://localhost:5000/api/health")
    logger.info("Diagnostics: http://localhost:5000/api/system/diagnostics")
    logger.info("ROS2 Health: http://localhost:5000/api/ros2/health")
    logger.info("=" * 70)
    
    app.run(host='0.0.0.0', port=5000, debug=False)