"""
API Routes for WeldVision X5 Scanning System

Endpoints for starting scans, monitoring progress, and retrieving results
"""

from flask import Blueprint, request, jsonify
from sqlalchemy.orm import Session
from sqlalchemy.exc import SQLAlchemyError
from datetime import datetime
import logging
import uuid
from typing import Optional

from database.scan_models import (
    Scan, ScanPoint, Measurement, Defect, ScanConfiguration,
    ScanStatus, DefectType, Base
)
from hardware.scan_orchestrator import ScanOrchestrator, ScanConfiguration as OrchestratorConfig
from hardware.printer_controller import PrinterController
from hardware.vision_processor import VisionProcessor
from hardware.ros2_camera_handler import ROS2CameraHandler

logger = logging.getLogger(__name__)

# Create blueprint
scan_bp = Blueprint('scan', __name__, url_prefix='/api/scan')

# Global instances
scan_orchestrator: Optional[ScanOrchestrator] = None
printer_controller: Optional[PrinterController] = None
vision_processor: Optional[VisionProcessor] = None
camera_handler: Optional[ROS2CameraHandler] = None


def initialize_hardware(db_session: Session) -> bool:
    """Initialize hardware components"""
    global scan_orchestrator, printer_controller, vision_processor, camera_handler
    
    try:
        logger.info("Initializing scan hardware...")
        
        # Initialize printer
        printer_controller = PrinterController()
        if not printer_controller.connect():
            logger.error("Failed to connect to printer")
            return False
        logger.info("Printer connected")
        
        # Initialize camera handler
        camera_handler = ROS2CameraHandler()
        if not camera_handler.connect():
            logger.warning("Failed to connect ROS2 camera, using fallback")
        logger.info("Camera handler initialized")
        
        # Initialize vision processor
        vision_processor = VisionProcessor()
        logger.info("Vision processor initialized")
        
        # Initialize orchestrator
        scan_orchestrator = ScanOrchestrator(
            printer_controller,
            vision_processor,
            camera_handler
        )
        logger.info("Scan orchestrator initialized")
        
        return True
        
    except Exception as e:
        logger.error(f"Hardware initialization error: {e}")
        return False


@scan_bp.route('/configurations', methods=['GET'])
def get_configurations():
    """Get all scan configurations"""
    try:
        from app import db
        configs = db.session.query(ScanConfiguration).all()
        return jsonify([cfg.to_dict() for cfg in configs]), 200
    except Exception as e:
        logger.error(f"Get configurations error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/configurations', methods=['POST'])
def create_configuration():
    """Create new scan configuration"""
    try:
        from app import db
        data = request.json
        
        config = ScanConfiguration(
            name=data.get('name', 'Custom Configuration'),
            grid_spacing_mm=data.get('grid_spacing_mm', 25.0),
            grid_overlap_mm=data.get('grid_overlap_mm', 10.0),
            z_height_mm=data.get('z_height_mm', 10.0),
            z_safe_height_mm=data.get('z_safe_height_mm', 50.0),
            dwell_time_ms=data.get('dwell_time_ms', 500),
            capture_count=data.get('capture_count', 3),
            feedrate_xy_mm_min=data.get('feedrate_xy_mm_min', 3000.0),
            feedrate_z_mm_min=data.get('feedrate_z_mm_min', 800.0),
            rgb_brightness_threshold=data.get('rgb_brightness_threshold', 100.0),
            rgb_saturation_threshold=data.get('rgb_saturation_threshold', 50.0),
        )
        
        db.session.add(config)
        db.session.commit()
        
        return jsonify(config.to_dict()), 201
        
    except SQLAlchemyError as e:
        logger.error(f"Database error: {e}")
        return {'error': 'Database error'}, 500
    except Exception as e:
        logger.error(f"Create configuration error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/start', methods=['POST'])
def start_scan():
    """Start a new scan"""
    try:
        from app import db
        
        if scan_orchestrator is None:
            initialize_hardware(db.session)
        
        data = request.json
        scan_id = str(uuid.uuid4())[:8]  # Short ID
        
        # Get or create configuration
        config_id = data.get('configuration_id')
        if config_id:
            db_config = db.session.query(ScanConfiguration).filter_by(id=config_id).first()
            if not db_config:
                return {'error': 'Configuration not found'}, 404
        else:
            db_config = db.session.query(ScanConfiguration).first()
            if not db_config:
                return {'error': 'No configuration available'}, 400
        
        # Create scan record
        scan = Scan(
            scan_id=scan_id,
            student_id=data.get('student_id'),
            configuration_id=db_config.id,
            status=ScanStatus.PENDING,
            workpiece_type=data.get('workpiece_type', 'Unknown'),
            workpiece_material=data.get('workpiece_material', 'Unknown'),
            operator_notes=data.get('operator_notes'),
            grid_x_min=data.get('grid_x_min', 0.0),
            grid_x_max=data.get('grid_x_max', 100.0),
            grid_y_min=data.get('grid_y_min', 0.0),
            grid_y_max=data.get('grid_y_max', 100.0),
            grid_spacing=db_config.grid_spacing_mm,
        )
        
        db.session.add(scan)
        db.session.commit()
        
        # Start orchestration
        orch_config = OrchestratorConfig(
            grid_x_min=scan.grid_x_min,
            grid_x_max=scan.grid_x_max,
            grid_y_min=scan.grid_y_min,
            grid_y_max=scan.grid_y_max,
            grid_spacing=db_config.grid_spacing_mm,
            z_height=db_config.z_height_mm,
            dwell_time_ms=db_config.dwell_time_ms,
        )
        
        # Progress callback to update database
        def on_progress(status_msg: dict):
            try:
                scan_db = db.session.query(Scan).filter_by(scan_id=scan_id).first()
                if scan_db:
                    scan_db.status = ScanStatus.IN_PROGRESS
                    scan_db.started_at = datetime.utcnow()
                    scan_db.completed_points = status_msg.get('completed_points', 0)
                    scan_db.total_points = status_msg.get('total_points', 0)
                    db.session.commit()
            except Exception as e:
                logger.error(f"Progress update error: {e}")
        
        scan_orchestrator.start_scan(scan_id, orch_config, on_progress)
        
        return jsonify({
            'scan_id': scan_id,
            'status': 'started',
            'database_id': scan.id,
        }), 201
        
    except Exception as e:
        logger.error(f"Start scan error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/status/<scan_id>', methods=['GET'])
def get_scan_status(scan_id: str):
    """Get scan status"""
    try:
        from app import db
        
        if scan_orchestrator is None:
            return {'error': 'Orchestrator not initialized'}, 500
        
        # Get orchestrator status
        orch_status = scan_orchestrator.get_scan_status(scan_id)
        
        # Get database status
        scan = db.session.query(Scan).filter_by(scan_id=scan_id).first()
        if not scan:
            return {'error': 'Scan not found'}, 404
        
        return jsonify({
            'scan_id': scan_id,
            'database_id': scan.id,
            'status': scan.status.value,
            'orchestrator_status': orch_status,
            'completed_points': scan.completed_points,
            'total_points': scan.total_points,
            'quality_score': scan.overall_quality_score,
        }), 200
        
    except Exception as e:
        logger.error(f"Get status error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/pause/<scan_id>', methods=['POST'])
def pause_scan(scan_id: str):
    """Pause a scan"""
    try:
        from app import db
        
        if scan_orchestrator is None:
            return {'error': 'Orchestrator not initialized'}, 500
        
        scan_orchestrator.pause_scan(scan_id)
        
        scan = db.session.query(Scan).filter_by(scan_id=scan_id).first()
        if scan:
            scan.status = ScanStatus.PAUSED
            db.session.commit()
        
        return jsonify({'status': 'paused'}), 200
        
    except Exception as e:
        logger.error(f"Pause scan error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/resume/<scan_id>', methods=['POST'])
def resume_scan(scan_id: str):
    """Resume a paused scan"""
    try:
        from app import db
        
        if scan_orchestrator is None:
            return {'error': 'Orchestrator not initialized'}, 500
        
        scan_orchestrator.resume_scan(scan_id)
        
        scan = db.session.query(Scan).filter_by(scan_id=scan_id).first()
        if scan:
            scan.status = ScanStatus.IN_PROGRESS
            db.session.commit()
        
        return jsonify({'status': 'resumed'}), 200
        
    except Exception as e:
        logger.error(f"Resume scan error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/stop/<scan_id>', methods=['POST'])
def stop_scan(scan_id: str):
    """Stop a scan"""
    try:
        from app import db
        
        if scan_orchestrator is None:
            return {'error': 'Orchestrator not initialized'}, 500
        
        scan_orchestrator.stop_scan(scan_id)
        
        scan = db.session.query(Scan).filter_by(scan_id=scan_id).first()
        if scan:
            scan.status = ScanStatus.CANCELLED
            db.session.commit()
        
        return jsonify({'status': 'stopped'}), 200
        
    except Exception as e:
        logger.error(f"Stop scan error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/results/<scan_id>', methods=['GET'])
def get_scan_results(scan_id: str):
    """Get completed scan results"""
    try:
        from app import db
        
        scan = db.session.query(Scan).filter_by(scan_id=scan_id).first()
        if not scan:
            return {'error': 'Scan not found'}, 404
        
        if scan.status != ScanStatus.COMPLETED:
            return {'error': 'Scan not completed'}, 400
        
        return jsonify(scan.to_dict(include_points=True)), 200
        
    except Exception as e:
        logger.error(f"Get results error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/history', methods=['GET'])
def get_scan_history():
    """Get scan history"""
    try:
        from app import db
        
        student_id = request.args.get('student_id', type=int)
        limit = request.args.get('limit', 50, type=int)
        
        query = db.session.query(Scan)
        if student_id:
            query = query.filter_by(student_id=student_id)
        
        scans = query.order_by(Scan.created_at.desc()).limit(limit).all()
        
        return jsonify([scan.to_dict() for scan in scans]), 200
        
    except Exception as e:
        logger.error(f"Get history error: {e}")
        return {'error': str(e)}, 500


@scan_bp.route('/hardware-status', methods=['GET'])
def get_hardware_status():
    """Get hardware status"""
    try:
        status = {
            'printer': {
                'connected': printer_controller.is_connected if printer_controller else False,
                'state': printer_controller.state.name if printer_controller else 'UNKNOWN',
            },
            'camera': {
                'ready': camera_handler.is_ready if camera_handler else False,
                'stats': camera_handler.get_stats() if camera_handler else {},
            },
            'orchestrator': {
                'initialized': scan_orchestrator is not None,
                'active_scans': len(scan_orchestrator.scans) if scan_orchestrator else 0,
            }
        }
        
        return jsonify(status), 200
        
    except Exception as e:
        logger.error(f"Get hardware status error: {e}")
        return {'error': str(e)}, 500
