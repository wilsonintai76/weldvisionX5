"""
Model Management API Routes
Manage trained models, versions, and deployments
"""

from flask import Blueprint, request, jsonify, send_file
import logging
from pathlib import Path
from datetime import datetime

logger = logging.getLogger(__name__)

model_bp = Blueprint('models', __name__, url_prefix='/api/models')

# In-memory model registry (could be database)
model_registry = {}


@model_bp.route('/list', methods=['GET'])
def list_models():
    """List all available models"""
    models_dir = Path('backend/models')
    models_list = []
    
    for model_file in models_dir.glob('*_trained.pth'):
        try:
            stat = model_file.stat()
            models_list.append({
                'name': model_file.stem.replace('_trained', ''),
                'file': model_file.name,
                'size': stat.st_size,
                'size_mb': round(stat.st_size / (1024 * 1024), 2),
                'created': datetime.fromtimestamp(stat.st_mtime).isoformat(),
                'type': 'pytorch'
            })
        except Exception as e:
            logger.warning(f"Error reading model {model_file}: {e}")
    
    # Check for ONNX models
    for model_file in models_dir.glob('*.onnx'):
        try:
            stat = model_file.stat()
            models_list.append({
                'name': model_file.stem,
                'file': model_file.name,
                'size': stat.st_size,
                'size_mb': round(stat.st_size / (1024 * 1024), 2),
                'created': datetime.fromtimestamp(stat.st_mtime).isoformat(),
                'type': 'onnx'
            })
        except Exception as e:
            logger.warning(f"Error reading model {model_file}: {e}")
    
    return jsonify({
        'models': models_list,
        'count': len(models_list)
    }), 200


@model_bp.route('/<model_name>/info', methods=['GET'])
def get_model_info(model_name):
    """Get model information"""
    models_dir = Path('backend/models')
    
    # Try to find the model
    model_paths = list(models_dir.glob(f'{model_name}*'))
    
    if not model_paths:
        return jsonify({'error': 'Model not found'}), 404
    
    model_path = model_paths[0]
    
    try:
        stat = model_path.stat()
        return jsonify({
            'name': model_name,
            'file': model_path.name,
            'size': stat.st_size,
            'size_mb': round(stat.st_size / (1024 * 1024), 2),
            'created': datetime.fromtimestamp(stat.st_mtime).isoformat(),
            'type': 'onnx' if model_path.suffix == '.onnx' else 'pytorch'
        }), 200
    
    except Exception as e:
        logger.error(f"Error reading model info: {e}")
        return jsonify({'error': str(e)}), 400


@model_bp.route('/<model_name>/download', methods=['GET'])
def download_model(model_name):
    """Download model file"""
    models_dir = Path('backend/models')
    
    # Try to find the model (prefer pytorch over onnx)
    pytorch_path = models_dir / f'{model_name}_trained.pth'
    onnx_path = models_dir / f'{model_name}.onnx'
    
    model_path = None
    if pytorch_path.exists():
        model_path = pytorch_path
    elif onnx_path.exists():
        model_path = onnx_path
    else:
        return jsonify({'error': 'Model not found'}), 404
    
    try:
        return send_file(
            model_path,
            as_attachment=True,
            download_name=model_path.name
        )
    except Exception as e:
        logger.error(f"Download model error: {e}")
        return jsonify({'error': str(e)}), 400


@model_bp.route('/<model_name>/delete', methods=['DELETE'])
def delete_model(model_name):
    """Delete a model"""
    models_dir = Path('backend/models')
    
    deleted = False
    
    # Delete pytorch model
    pytorch_path = models_dir / f'{model_name}_trained.pth'
    if pytorch_path.exists():
        try:
            pytorch_path.unlink()
            deleted = True
            logger.info(f"Deleted {pytorch_path}")
        except Exception as e:
            logger.error(f"Error deleting {pytorch_path}: {e}")
    
    # Delete ONNX model
    onnx_path = models_dir / f'{model_name}.onnx'
    if onnx_path.exists():
        try:
            onnx_path.unlink()
            deleted = True
            logger.info(f"Deleted {onnx_path}")
        except Exception as e:
            logger.error(f"Error deleting {onnx_path}: {e}")
    
    if not deleted:
        return jsonify({'error': 'Model not found'}), 404
    
    return jsonify({'status': 'deleted'}), 200


@model_bp.route('/<model_name>/metadata', methods=['POST'])
def set_model_metadata(model_name):
    """Set model metadata"""
    try:
        data = request.json
        
        # Store in registry
        model_registry[model_name] = {
            'name': model_name,
            'accuracy': data.get('accuracy'),
            'loss': data.get('loss'),
            'description': data.get('description', ''),
            'tags': data.get('tags', []),
            'updated_at': datetime.now().isoformat()
        }
        
        logger.info(f"Updated metadata for model {model_name}")
        
        return jsonify({
            'status': 'updated',
            'model': model_registry[model_name]
        }), 200
    
    except Exception as e:
        logger.error(f"Set metadata error: {e}")
        return jsonify({'error': str(e)}), 400


@model_bp.route('/<model_name>/metadata', methods=['GET'])
def get_model_metadata(model_name):
    """Get model metadata"""
    if model_name not in model_registry:
        return jsonify({'error': 'Metadata not found'}), 404
    
    return jsonify(model_registry[model_name]), 200


@model_bp.route('/compare', methods=['POST'])
def compare_models():
    """Compare multiple models"""
    try:
        data = request.json
        model_names = data.get('models', [])
        
        comparison = {
            'models': [],
            'comparison_date': datetime.now().isoformat()
        }
        
        models_dir = Path('backend/models')
        
        for model_name in model_names:
            pytorch_path = models_dir / f'{model_name}_trained.pth'
            onnx_path = models_dir / f'{model_name}.onnx'
            
            model_path = pytorch_path if pytorch_path.exists() else onnx_path
            
            if not model_path:
                continue
            
            stat = model_path.stat()
            
            comparison['models'].append({
                'name': model_name,
                'size_mb': round(stat.st_size / (1024 * 1024), 2),
                'created': datetime.fromtimestamp(stat.st_mtime).isoformat(),
                'type': 'onnx' if model_path.suffix == '.onnx' else 'pytorch',
                'metadata': model_registry.get(model_name, {})
            })
        
        return jsonify(comparison), 200
    
    except Exception as e:
        logger.error(f"Compare models error: {e}")
        return jsonify({'error': str(e)}), 400


@model_bp.route('/health', methods=['GET'])
def health():
    """Check models health"""
    models_dir = Path('backend/models')
    
    pytorch_count = len(list(models_dir.glob('*_trained.pth')))
    onnx_count = len(list(models_dir.glob('*.onnx')))
    
    return jsonify({
        'status': 'healthy',
        'pytorch_models': pytorch_count,
        'onnx_models': onnx_count,
        'total_models': pytorch_count + onnx_count
    }), 200


def get_model_blueprint():
    """Get model blueprint"""
    return model_bp
