import threading
import json
import time
import logging
import os
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent))
# Add rdk_weld_evaluator src to path for job routes
try:
    project_src_path = Path(__file__).parent.parent / 'rdk_weld_evaluator' / 'src'
    if project_src_path.exists():
        sys.path.insert(0, str(project_src_path))
except Exception:
    pass

from flask import Flask, jsonify, request
from flask_cors import CORS
from flask_sqlalchemy import SQLAlchemy
from system_check import HardwareDetector, DatabaseManager, setup_logging
import base64
import io
from PIL import Image
import yaml

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

# Init SmartRigController Routes (Panorama Scanning)
try:
    from api.smart_rig_routes import smart_rig_bp
    app.register_blueprint(smart_rig_bp)
    logger.info("SmartRigController routes registered")
except Exception as e:
    logger.warning(f"SmartRigController routes initialization failed: {e}")

# Init SafeMotionController Routes (Virtual Zero)
try:
    from api.safe_motion_routes import safe_motion_bp
    app.register_blueprint(safe_motion_bp)
    logger.info("SafeMotionController routes registered")
except Exception as e:
    logger.warning(f"SafeMotionController routes initialization failed: {e}")

# Init Inference Routes (Hybrid edge inference)
try:
    from api.inference_routes import inference_bp
    app.register_blueprint(inference_bp)
    logger.info("Inference routes registered")
except Exception as e:
    logger.warning(f"Inference routes initialization failed: {e}")

# Init Training Routes (Desktop training pipeline)
try:
    from api.training_routes import training_bp
    app.register_blueprint(training_bp)
    logger.info("Training routes registered")
except Exception as e:
    logger.warning(f"Training routes initialization failed: {e}")

# Init Model Management Routes (Model deployment & tracking)
try:
    from api.model_routes import model_bp
    app.register_blueprint(model_bp)
    logger.info("Model management routes registered")
except Exception as e:
    logger.warning(f"Model management routes initialization failed: {e}")

# Init Orchestration Job Routes (Train/Compile/Deploy/Inference control)
try:
    from api.job_routes import jobs_bp
    app.register_blueprint(jobs_bp)
    logger.info("Orchestration job routes registered")
except Exception as e:
    logger.warning(f"Job routes initialization failed: {e}")

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

@app.route('/api/dataset/upload', methods=['POST'])
def dataset_upload():
    try:
        data = request.json
        img_b64 = data.get('imageBase64')
        image_path = data.get('savePath')  # e.g., D:/data/weldsets/setA/images/img_001.jpg
        label_path = data.get('labelPath')  # e.g., D:/data/weldsets/setA/labels/img_001.txt
        labels = data.get('labels', [])  # [{class: 'defect', bbox: [x_center,y_center,w,h]} normalized 0..1]
        if not img_b64 or not image_path:
            return jsonify({'error': 'imageBase64 and savePath required'}), 400

        # Decode and save image
        img_bytes = base64.b64decode(img_b64.split(',')[-1])
        im = Image.open(io.BytesIO(img_bytes)).convert('RGB')
        Path(image_path).parent.mkdir(parents=True, exist_ok=True)
        im.save(image_path)

        # Save YOLO labels only if labelPath provided AND labels exist
        if label_path is not None and labels:
            Path(label_path).parent.mkdir(parents=True, exist_ok=True)
            with open(label_path, 'w', encoding='utf-8') as f:
                for item in labels:
                    cls = item.get('class_id', 0)
                    x, y, w, h = item.get('bbox', [0,0,0,0])
                    f.write(f"{cls} {x} {y} {w} {h}\n")

        return jsonify({'ok': True, 'imagePath': image_path, 'labelPath': label_path}), 200
    except Exception as e:
        logger.error(f"Dataset upload failed: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/dataset/upload_multipart', methods=['POST'])
def dataset_upload_multipart():
    try:
        file = request.files.get('image')
        image_path = request.form.get('savePath')
        label_path = request.form.get('labelPath')
        labels_json = request.form.get('labels')
        labels = []
        if labels_json:
            try:
                labels = json.loads(labels_json)
            except Exception:
                labels = []
        if not file or not image_path:
            return jsonify({'error': 'image file and savePath required'}), 400
        im = Image.open(file.stream).convert('RGB')
        Path(image_path).parent.mkdir(parents=True, exist_ok=True)
        im.save(image_path)
        if label_path is not None and labels:
            Path(label_path).parent.mkdir(parents=True, exist_ok=True)
            with open(label_path, 'w', encoding='utf-8') as f:
                for item in labels:
                    cls = item.get('class_id', 0)
                    x, y, w, h = item.get('bbox', [0,0,0,0])
                    f.write(f"{cls} {x} {y} {w} {h}\n")
        return jsonify({'ok': True, 'imagePath': image_path, 'labelPath': label_path}), 200
    except Exception as e:
        logger.error(f"Dataset multipart upload failed: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/dataset/generate_yaml', methods=['POST'])
def dataset_generate_yaml():
    try:
        data = request.json
        base_dir = data.get('baseDir')  # e.g., D:/data/weldsets/setA
        train_rel = data.get('trainRel', 'images')
        val_rel = data.get('valRel', 'images')
        names = data.get('names', ['good','porosity','undercut'])
        if not base_dir:
            return jsonify({'error': 'baseDir required'}), 400
        cfg = {
            'path': base_dir.replace('\\','/'),
            'train': train_rel,
            'val': val_rel,
            'names': names,
        }
        out_path = Path(base_dir) / 'dataset.yaml'
        with open(out_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
        return jsonify({'ok': True, 'yamlPath': str(out_path)}), 200
    except Exception as e:
        logger.error(f"Generate YAML failed: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/dataset/list', methods=['POST'])
def list_dataset_files():
    data = request.get_json()
    base_path = data.get('base_path')
    image_subdir = data.get('image_subdir', 'images/train')
    label_subdir = data.get('label_subdir', 'labels/train')
    if not base_path:
        return jsonify({'ok': False, 'error': 'base_path required'}), 400
    img_dir = os.path.join(base_path, image_subdir)
    lbl_dir = os.path.join(base_path, label_subdir)
    images = []
    labels_missing = []
    labels = []
    if os.path.isdir(img_dir):
        for fname in os.listdir(img_dir):
            if fname.lower().endswith(('.jpg', '.jpeg', '.png')):
                stem, _ = os.path.splitext(fname)
                images.append(fname)
                lbl_file = os.path.join(lbl_dir, stem + '.txt')
                if os.path.isfile(lbl_file):
                    labels.append(stem + '.txt')
                else:
                    labels_missing.append(stem + '.txt')
    return jsonify({
        'ok': True,
        'image_dir': img_dir,
        'label_dir': lbl_dir,
        'images': images,
        'labels': labels,
        'labels_missing': labels_missing
    })

@app.route('/api/dataset/image/<path:filepath>', methods=['GET'])
def serve_dataset_image(filepath):
    """Serve images from the dataset directory"""
    try:
        from flask import send_file
        import os
        
        # Security: normalize path to prevent directory traversal
        filepath = os.path.normpath(filepath)
        if '..' in filepath:
            return jsonify({'error': 'Invalid path'}), 400
        
        if os.path.exists(filepath) and os.path.isfile(filepath):
            return send_file(filepath, mimetype='image/jpeg')
        else:
            return jsonify({'error': 'Image not found'}), 404
    except Exception as e:
        logger.error(f"Error serving image: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/dataset/delete', methods=['POST'])
def dataset_delete_image():
    """Delete an image and its label file"""
    try:
        import os
        data = request.json
        image_path = data.get('imagePath')
        label_path = data.get('labelPath')
        
        if not image_path:
            return jsonify({'error': 'imagePath required'}), 400
        
        # Security: normalize paths
        image_path = os.path.normpath(image_path)
        if '..' in image_path:
            return jsonify({'error': 'Invalid path'}), 400
        
        # Delete image file
        if os.path.exists(image_path) and os.path.isfile(image_path):
            os.remove(image_path)
            logger.info(f"Deleted image: {image_path}")
        
        # Delete label file if exists
        if label_path:
            label_path = os.path.normpath(label_path)
            if '..' not in label_path and os.path.exists(label_path) and os.path.isfile(label_path):
                os.remove(label_path)
                logger.info(f"Deleted label: {label_path}")
        
        return jsonify({'ok': True, 'message': 'Image deleted'}), 200
    except Exception as e:
        logger.error(f"Error deleting image: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/api/dataset/label/read', methods=['POST'])
def read_label_file():
    """Read label file and return its contents"""
    try:
        data = request.json or {}
        label_path = data.get('labelPath', '')
        
        if not label_path:
            return jsonify({'error': 'labelPath required'}), 400
        
        label_path = os.path.normpath(label_path)
        
        # Security check
        if '..' in label_path:
            return jsonify({'error': 'Invalid path'}), 400
        
        if not os.path.exists(label_path) or not os.path.isfile(label_path):
            return jsonify({'error': 'Label file not found'}), 404
        
        with open(label_path, 'r') as f:
            lines = f.readlines()
        
        # Parse YOLO format: class_id x_center y_center width height
        boxes = []
        for line in lines:
            line = line.strip()
            if line:
                parts = line.split()
                if len(parts) >= 5:
                    boxes.append({
                        'class_id': int(parts[0]),
                        'x': float(parts[1]),
                        'y': float(parts[2]),
                        'w': float(parts[3]),
                        'h': float(parts[4])
                    })
        
        return jsonify({'ok': True, 'boxes': boxes}), 200
    except Exception as e:
        logger.error(f"Error reading label file: {e}")
        return jsonify({'error': str(e)}), 500

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


# RDK Connection Testing Endpoints
@app.route('/api/rdk/test-connection', methods=['POST'])
def test_rdk_connection():
    """Test if RDK device is reachable via network"""
    try:
        data = request.json
        host = data.get('host', '')
        user = data.get('user', 'root')
        
        if not host:
            return jsonify({'error': 'Host is required'}), 400
        
        logger.info(f"Testing connection to RDK at {host}")
        
        # Test 1: Network ping
        import subprocess
        import platform
        
        # Platform-specific ping command
        param = '-n' if platform.system().lower() == 'windows' else '-c'
        ping_cmd = ['ping', param, '1', '-w' if platform.system().lower() == 'windows' else '-W', '1000' if platform.system().lower() == 'windows' else '1', host]
        
        try:
            ping_result = subprocess.run(ping_cmd, capture_output=True, timeout=3)
            reachable = (ping_result.returncode == 0)
        except Exception as e:
            logger.error(f"Ping test failed: {e}")
            reachable = False
        
        # Test 2: SSH connection (if ping succeeds)
        ssh_available = False
        if reachable:
            try:
                ssh_cmd = ['ssh', '-o', 'ConnectTimeout=3', '-o', 'StrictHostKeyChecking=no', 
                          '-o', 'BatchMode=yes', f'{user}@{host}', 'echo OK']
                ssh_result = subprocess.run(ssh_cmd, capture_output=True, timeout=5)
                ssh_available = (ssh_result.returncode == 0)
            except Exception as e:
                logger.error(f"SSH test failed: {e}")
                ssh_available = False
        
        # Test 3: Inference service health check (if SSH works)
        service_healthy = False
        if ssh_available:
            try:
                import requests
                response = requests.get(f'http://{host}:8080/health', timeout=3)
                service_healthy = (response.status_code == 200)
            except Exception as e:
                logger.debug(f"Service health check failed: {e}")
                service_healthy = False
        
        # Determine status message
        if reachable and ssh_available:
            message = f'RDK is ready at {host}'
            status = 'connected'
        elif reachable and not ssh_available:
            message = f'RDK at {host} is reachable but SSH is not available. Check credentials or SSH service.'
            status = 'ssh_failed'
        elif not reachable:
            message = f'Cannot reach RDK at {host}. Check network connection and IP address.'
            status = 'unreachable'
        else:
            message = f'Unknown connection status for {host}'
            status = 'unknown'
        
        logger.info(f"Connection test result: {status} - {message}")
        
        return jsonify({
            'reachable': reachable,
            'sshAvailable': ssh_available,
            'serviceHealthy': service_healthy,
            'connected': reachable and ssh_available,
            'status': status,
            'message': message,
            'host': host,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
        }), 200
        
    except Exception as e:
        logger.error(f"RDK connection test failed: {e}")
        return jsonify({
            'error': str(e),
            'connected': False,
            'message': f'Connection test error: {str(e)}'
        }), 500


@app.route('/api/rdk/status', methods=['GET'])
def get_rdk_status():
    """Get current RDK connection status"""
    # This could be enhanced with cached/persistent status in the future
    return jsonify({
        'statusCheckAvailable': True,
        'message': 'Use POST /api/rdk/test-connection to check RDK connectivity',
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
    }), 200


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
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)
    except Exception as e:
        logger.error(f"Flask server failed to start: {e}")
        import traceback
        logger.error(traceback.format_exc())