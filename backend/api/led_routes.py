"""
LED Control API Endpoints
Provides REST API for LED control integration
"""

from flask import Blueprint, request, jsonify
from vision.led_control import create_led_controller, LEDControlMode
import logging

logger = logging.getLogger(__name__)

# Global LED controller instance
led_controller = None

def init_led_controller(app, mode: str = "simulator"):
    """Initialize LED controller for Flask app"""
    global led_controller
    led_controller = create_led_controller(mode=mode)
    led_controller.start_monitoring()
    
    @app.teardown_appcontext
    def cleanup_led(error=None):
        if led_controller:
            led_controller.cleanup()

def get_led_blueprint():
    """Create LED control blueprint"""
    led_bp = Blueprint('led', __name__, url_prefix='/api/led')
    
    @led_bp.route('/status', methods=['GET'])
    def get_led_status():
        """
        Get LED status
        
        Returns:
            {
                "is_on": bool,
                "current_intensity": 0-100,
                "target_intensity": 0-100,
                "mode": "gpio|pwm|relay|simulator",
                "temperature": float,
                "temperature_warning": bool,
                "sensor_available": bool,
                "light_level_lux": float|null
            }
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            status = led_controller.get_status()
            return jsonify({
                "status": "success",
                "data": status
            }), 200
        except Exception as e:
            logger.error(f"Error getting LED status: {e}")
            return jsonify({"error": str(e)}), 500
    
    @led_bp.route('/on', methods=['POST'])
    def led_on():
        """
        Turn LED on with specified intensity
        
        Request body:
            {
                "intensity": 0-100 (optional, default 75)
            }
        
        Returns:
            Success/failure response
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            data = request.get_json() or {}
            intensity = data.get('intensity', 75)
            
            if not isinstance(intensity, int) or intensity < 0 or intensity > 100:
                return jsonify({"error": "Intensity must be 0-100"}), 400
            
            success = led_controller.turn_on(intensity)
            
            return jsonify({
                "status": "success" if success else "failed",
                "message": f"LED turned on at {intensity}%",
                "data": led_controller.get_status()
            }), 200 if success else 500
        
        except Exception as e:
            logger.error(f"Error turning on LED: {e}")
            return jsonify({"error": str(e)}), 500
    
    @led_bp.route('/off', methods=['POST'])
    def led_off():
        """
        Turn LED off
        
        Returns:
            Success/failure response
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            success = led_controller.turn_off()
            
            return jsonify({
                "status": "success" if success else "failed",
                "message": "LED turned off",
                "data": led_controller.get_status()
            }), 200 if success else 500
        
        except Exception as e:
            logger.error(f"Error turning off LED: {e}")
            return jsonify({"error": str(e)}), 500
    
    @led_bp.route('/intensity', methods=['POST'])
    def set_intensity():
        """
        Set LED intensity (0-100%)
        
        Request body:
            {
                "intensity": 0-100
            }
        
        Returns:
            Success/failure response
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            data = request.get_json() or {}
            intensity = data.get('intensity')
            
            if intensity is None:
                return jsonify({"error": "intensity parameter required"}), 400
            
            if not isinstance(intensity, int) or intensity < 0 or intensity > 100:
                return jsonify({"error": "Intensity must be 0-100"}), 400
            
            success = led_controller.set_intensity(intensity)
            
            return jsonify({
                "status": "success" if success else "failed",
                "message": f"Intensity set to {intensity}%",
                "data": led_controller.get_status()
            }), 200 if success else 500
        
        except Exception as e:
            logger.error(f"Error setting intensity: {e}")
            return jsonify({"error": str(e)}), 500
    
    @led_bp.route('/preset', methods=['POST'])
    def set_preset():
        """
        Set LED to preset brightness level
        
        Available presets:
        - "off": 0%
        - "dim": 25%
        - "medium": 50%
        - "bright": 75%
        - "max": 100%
        
        Request body:
            {
                "preset": "off|dim|medium|bright|max"
            }
        
        Returns:
            Success/failure response
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            data = request.get_json() or {}
            preset = data.get('preset')
            
            if not preset:
                return jsonify({"error": "preset parameter required"}), 400
            
            success = led_controller.set_preset(preset)
            
            return jsonify({
                "status": "success" if success else "failed",
                "message": f"LED preset set to {preset}",
                "data": led_controller.get_status()
            }), 200 if success else 500
        
        except Exception as e:
            logger.error(f"Error setting preset: {e}")
            return jsonify({"error": str(e)}), 500
    
    @led_bp.route('/presets', methods=['GET'])
    def get_presets():
        """
        Get available brightness presets
        
        Returns:
            Dictionary of preset names and intensities
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            presets = led_controller.get_presets()
            return jsonify({
                "status": "success",
                "data": presets
            }), 200
        except Exception as e:
            logger.error(f"Error getting presets: {e}")
            return jsonify({"error": str(e)}), 500
    
    @led_bp.route('/light-level', methods=['GET'])
    def get_light_level():
        """
        Get current light level from sensor (if available)
        
        Returns:
            Light level in lux or null if sensor not available
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            light_level = led_controller.get_light_level()
            
            return jsonify({
                "status": "success",
                "sensor_available": led_controller.sensor_available,
                "light_level_lux": light_level,
                "message": "Light sensor not available" if light_level is None else "Light level read successfully"
            }), 200
        except Exception as e:
            logger.error(f"Error getting light level: {e}")
            return jsonify({"error": str(e)}), 500
    
    @led_bp.route('/temperature', methods=['POST'])
    def set_temperature():
        """
        Update LED temperature for monitoring
        
        Request body:
            {
                "temperature": float (Celsius)
            }
        
        Returns:
            Success/failure response
        """
        if not led_controller:
            return jsonify({"error": "LED controller not initialized"}), 500
        
        try:
            data = request.get_json() or {}
            temp = data.get('temperature')
            
            if temp is None:
                return jsonify({"error": "temperature parameter required"}), 400
            
            led_controller.set_temperature(float(temp))
            
            return jsonify({
                "status": "success",
                "message": f"Temperature set to {temp}°C",
                "data": led_controller.get_status()
            }), 200
        
        except Exception as e:
            logger.error(f"Error setting temperature: {e}")
            return jsonify({"error": str(e)}), 500
    
    return led_bp
