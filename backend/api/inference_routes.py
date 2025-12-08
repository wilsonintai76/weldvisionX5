"""
Inference API Routes - RDK Edge Inference
Provides REST endpoints for hybrid inference
"""

from flask import Blueprint, request, jsonify
from models.inference import HybridInferenceEngine
import numpy as np
import base64
import cv2
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

inference_bp = Blueprint('inference', __name__, url_prefix='/api/inference')

# Global inference engine
inference_engine = None


def init_inference_engine(use_desktop=False, desktop_url=None):
    """Initialize the hybrid inference engine"""
    global inference_engine
    try:
        inference_engine = HybridInferenceEngine(use_desktop=use_desktop)
        if desktop_url:
            inference_engine.desktop_url = desktop_url
        logger.info("Inference engine initialized successfully")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize inference engine: {e}")
        return False


@inference_bp.route('/predict', methods=['POST'])
def predict():
    """Run inference on RGB + Depth frame"""
    try:
        data = request.json
        
        # Decode frames
        rgb_bytes = base64.b64decode(data.get('rgb', ''))
        depth_bytes = base64.b64decode(data.get('depth', ''))
        
        if not rgb_bytes or not depth_bytes:
            return jsonify({'error': 'Missing RGB or depth data'}), 400
        
        rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((1080, 1920, 3))
        depth = np.frombuffer(depth_bytes, dtype=np.float32).reshape((1080, 1920))
        
        # Run inference
        prediction = inference_engine.predict(rgb, depth)
        prediction['timestamp'] = datetime.now().isoformat()
        
        return jsonify({
            'prediction': prediction,
            'source': 'rdk-hybrid',
            'timestamp': datetime.now().isoformat()
        }), 200
    
    except Exception as e:
        logger.error(f"Inference error: {e}")
        return jsonify({'error': str(e)}), 400


@inference_bp.route('/batch-predict', methods=['POST'])
def batch_predict():
    """Run inference on multiple frames"""
    try:
        data = request.json
        frames = data.get('frames', [])
        
        if not frames:
            return jsonify({'error': 'No frames provided'}), 400
        
        predictions = []
        for frame_data in frames:
            rgb_bytes = base64.b64decode(frame_data.get('rgb', ''))
            depth_bytes = base64.b64decode(frame_data.get('depth', ''))
            
            rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((1080, 1920, 3))
            depth = np.frombuffer(depth_bytes, dtype=np.float32).reshape((1080, 1920))
            
            prediction = inference_engine.predict(rgb, depth)
            prediction['timestamp'] = datetime.now().isoformat()
            predictions.append(prediction)
        
        return jsonify({
            'predictions': predictions,
            'count': len(predictions)
        }), 200
    
    except Exception as e:
        logger.error(f"Batch inference error: {e}")
        return jsonify({'error': str(e)}), 400


@inference_bp.route('/health', methods=['GET'])
def health():
    """Check inference engine status"""
    if inference_engine is None:
        return jsonify({
            'status': 'not initialized',
            'message': 'Inference engine not initialized'
        }), 503
    
    return jsonify({
        'status': 'healthy',
        'engine': 'hybrid (rule-based + mobilenet)',
        'models': ['rule-based', 'mobilenet'],
        'desktop_enabled': inference_engine.use_desktop,
        'desktop_url': inference_engine.desktop_url if inference_engine.use_desktop else None
    }), 200


@inference_bp.route('/config', methods=['GET'])
def get_config():
    """Get inference configuration"""
    if inference_engine is None:
        return jsonify({'error': 'Inference engine not initialized'}), 503
    
    return jsonify({
        'use_desktop': inference_engine.use_desktop,
        'desktop_url': inference_engine.desktop_url,
        'uncertain_threshold': inference_engine.uncertain_threshold
    }), 200


@inference_bp.route('/config', methods=['POST'])
def update_config():
    """Update inference configuration"""
    try:
        data = request.json
        
        if 'use_desktop' in data:
            if data['use_desktop']:
                desktop_url = data.get('desktop_url', 'http://192.168.1.100:5001')
                inference_engine.enable_desktop(desktop_url)
            else:
                inference_engine.disable_desktop()
        
        if 'uncertain_threshold' in data:
            inference_engine.uncertain_threshold = data['uncertain_threshold']
        
        return jsonify({'status': 'updated'}), 200
    
    except Exception as e:
        logger.error(f"Config update error: {e}")
        return jsonify({'error': str(e)}), 400


@inference_bp.route('/enable-desktop', methods=['POST'])
def enable_desktop():
    """Enable desktop refinement"""
    try:
        data = request.json
        desktop_url = data.get('desktop_url', 'http://192.168.1.100:5001')
        
        inference_engine.enable_desktop(desktop_url)
        
        return jsonify({
            'status': 'enabled',
            'desktop_url': desktop_url
        }), 200
    
    except Exception as e:
        logger.error(f"Enable desktop error: {e}")
        return jsonify({'error': str(e)}), 400


@inference_bp.route('/disable-desktop', methods=['POST'])
def disable_desktop():
    """Disable desktop refinement"""
    try:
        inference_engine.disable_desktop()
        
        return jsonify({'status': 'disabled'}), 200
    
    except Exception as e:
        logger.error(f"Disable desktop error: {e}")
        return jsonify({'error': str(e)}), 400


def get_inference_blueprint():
    """Get inference blueprint"""
    return inference_bp
