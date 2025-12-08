"""
Training API Routes - Desktop Model Training
Provides REST endpoints for training pipeline
"""

from flask import Blueprint, request, jsonify, send_file
from models.training_pipeline import TrainingPipeline, WeldDefectDataset
from torch.utils.data import DataLoader
import logging
import threading
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)

training_bp = Blueprint('training', __name__, url_prefix='/api/training')

# Global training pipeline and jobs
training_pipeline = None
training_jobs = {}
training_threads = {}


def init_training_pipeline():
    """Initialize training pipeline"""
    global training_pipeline
    try:
        training_pipeline = TrainingPipeline()
        logger.info("Training pipeline initialized")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize training pipeline: {e}")
        return False


def run_training_job(job_id: str, model_name: str, config: dict):
    """Run training job in background thread"""
    try:
        training_jobs[job_id] = {
            'id': job_id,
            'model': model_name,
            'status': 'running',
            'progress': 0,
            'started_at': datetime.now().isoformat(),
            'epoch': 0,
            'total_epochs': config.get('epochs', 100),
            'loss': 0.0,
            'accuracy': 0.0
        }
        
        logger.info(f"Starting training job {job_id}: {model_name}")
        
        # Load dataset
        frames_dir = config.get('frames_dir', 'backend/data/frames')
        labels_file = config.get('labels_file', 'backend/data/labels.json')
        
        try:
            train_dataset = WeldDefectDataset(frames_dir, labels_file)
            val_dataset = WeldDefectDataset(frames_dir, labels_file)
            
            train_loader = DataLoader(
                train_dataset,
                batch_size=config.get('batch_size', 32),
                shuffle=True
            )
            val_loader = DataLoader(
                val_dataset,
                batch_size=config.get('batch_size', 32)
            )
            
            # Train model
            history = training_pipeline.train_model(
                model_name=model_name,
                train_loader=train_loader,
                val_loader=val_loader,
                epochs=config.get('epochs', 100),
                learning_rate=config.get('learning_rate', 0.001)
            )
            
            # Update job status
            training_jobs[job_id]['status'] = 'completed'
            training_jobs[job_id]['history'] = history
            training_jobs[job_id]['completed_at'] = datetime.now().isoformat()
            
            logger.info(f"Training job {job_id} completed")
        
        except FileNotFoundError:
            training_jobs[job_id]['status'] = 'failed'
            training_jobs[job_id]['error'] = 'Dataset not found'
            logger.error(f"Dataset not found for job {job_id}")
    
    except Exception as e:
        training_jobs[job_id]['status'] = 'failed'
        training_jobs[job_id]['error'] = str(e)
        logger.error(f"Training job {job_id} failed: {e}")


@training_bp.route('/start', methods=['POST'])
def start_training():
    """Start a new training job"""
    try:
        data = request.json
        model_name = data.get('model', 'mobilenet')
        config = data.get('config', {})
        
        if model_name not in ['mobilenet', 'resnet50']:
            return jsonify({'error': 'Unknown model'}), 400
        
        # Create job ID
        job_id = f"job_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Start training in background thread
        thread = threading.Thread(
            target=run_training_job,
            args=(job_id, model_name, config)
        )
        thread.daemon = True
        thread.start()
        training_threads[job_id] = thread
        
        return jsonify({
            'job_id': job_id,
            'model': model_name,
            'status': 'started'
        }), 201
    
    except Exception as e:
        logger.error(f"Start training error: {e}")
        return jsonify({'error': str(e)}), 400


@training_bp.route('/jobs', methods=['GET'])
def list_jobs():
    """List all training jobs"""
    return jsonify({
        'jobs': list(training_jobs.values()),
        'count': len(training_jobs)
    }), 200


@training_bp.route('/jobs/<job_id>', methods=['GET'])
def get_job_status(job_id):
    """Get training job status"""
    if job_id not in training_jobs:
        return jsonify({'error': 'Job not found'}), 404
    
    job = training_jobs[job_id]
    
    # Calculate progress
    if job['total_epochs'] > 0 and 'history' in job:
        job['progress'] = int((len(job['history']['epochs']) / job['total_epochs']) * 100)
    
    return jsonify(job), 200


@training_bp.route('/jobs/<job_id>/cancel', methods=['POST'])
def cancel_job(job_id):
    """Cancel a training job"""
    if job_id not in training_jobs:
        return jsonify({'error': 'Job not found'}), 404
    
    training_jobs[job_id]['status'] = 'cancelled'
    
    return jsonify({'status': 'cancelled'}), 200


@training_bp.route('/models', methods=['GET'])
def list_models():
    """List available trained models"""
    models_dir = Path('backend/models')
    models_list = []
    
    for model_file in models_dir.glob('*_trained.pth'):
        models_list.append({
            'name': model_file.stem.replace('_trained', ''),
            'file': model_file.name,
            'size': model_file.stat().st_size,
            'created': datetime.fromtimestamp(model_file.stat().st_mtime).isoformat()
        })
    
    return jsonify({'models': models_list}), 200


@training_bp.route('/models/<model_name>/download', methods=['GET'])
def download_model(model_name):
    """Download trained model"""
    model_path = Path(f'backend/models/{model_name}_trained.pth')
    
    if not model_path.exists():
        return jsonify({'error': 'Model not found'}), 404
    
    try:
        return send_file(
            model_path,
            as_attachment=True,
            download_name=f'{model_name}_trained.pth'
        )
    except Exception as e:
        logger.error(f"Download model error: {e}")
        return jsonify({'error': str(e)}), 400


@training_bp.route('/export-onnx/<model_name>', methods=['POST'])
def export_onnx(model_name):
    """Export model to ONNX format"""
    try:
        output_path = f'backend/models/{model_name}.onnx'
        training_pipeline.export_to_onnx(model_name, output_path)
        
        return jsonify({
            'status': 'exported',
            'path': output_path
        }), 200
    
    except Exception as e:
        logger.error(f"Export ONNX error: {e}")
        return jsonify({'error': str(e)}), 400


@training_bp.route('/health', methods=['GET'])
def health():
    """Check training pipeline status"""
    if training_pipeline is None:
        return jsonify({
            'status': 'not initialized',
            'message': 'Training pipeline not initialized'
        }), 503
    
    return jsonify({
        'status': 'healthy',
        'device': str(training_pipeline.device),
        'active_jobs': len([j for j in training_jobs.values() if j['status'] == 'running']),
        'total_jobs': len(training_jobs)
    }), 200


def get_training_blueprint():
    """Get training blueprint"""
    return training_bp
