"""
LED Control Module for WeldMaster AI
Manages LED ring light control including on/off and intensity adjustment
Supports GPIO-based control on RDK X5 and USB relay fallback
"""

import logging
import json
from enum import Enum
from typing import Dict, Optional, Tuple
import threading
import time

logger = logging.getLogger(__name__)


class LEDControlMode(Enum):
    """LED control modes"""
    GPIO = "gpio"           # Direct GPIO on RDK X5
    PWM = "pwm"             # PWM control for intensity
    RELAY = "relay"         # USB relay control
    SIMULATOR = "simulator" # Software simulation (no hardware)


class LEDController:
    """
    Controls LED ring light brightness and on/off state
    
    Features:
    - On/Off control
    - Intensity adjustment (0-100%)
    - PWM-based brightness control
    - GPIO pin configuration
    - USB relay support
    - Temperature monitoring
    - Brightness profiles
    """
    
    def __init__(self, mode: LEDControlMode = LEDControlMode.SIMULATOR, gpio_pin: int = 17):
        """
        Initialize LED controller
        
        Args:
            mode: Control mode (GPIO, PWM, RELAY, or SIMULATOR)
            gpio_pin: GPIO pin number for LED control (default: 17)
        """
        self.mode = mode
        self.gpio_pin = gpio_pin
        self.is_on = False
        self.current_intensity = 0  # 0-100%
        self.target_intensity = 0
        self.max_intensity = 100
        self.min_intensity = 10  # Minimum usable brightness
        
        # Hardware initialization
        self.gpio = None
        self.pwm = None
        self.relay = None
        self.sensor_available = False
        
        # Temperature monitoring
        self.current_temp = 25.0  # Celsius
        self.max_temp = 60.0  # Shutdown temperature
        self.warning_temp = 55.0
        self.temp_monitoring = True
        
        # Control thread
        self.control_thread = None
        self.running = False
        self.lock = threading.Lock()
        
        # Presets
        self.presets = {
            "off": 0,
            "dim": 25,
            "medium": 50,
            "bright": 75,
            "max": 100
        }
        
        logger.info(f"Initializing LED controller in {mode.value} mode on GPIO pin {gpio_pin}")
        self._initialize_hardware()
    
    def _initialize_hardware(self):
        """Initialize hardware based on selected mode"""
        try:
            if self.mode == LEDControlMode.GPIO or self.mode == LEDControlMode.PWM:
                self._init_gpio()
            elif self.mode == LEDControlMode.RELAY:
                self._init_relay()
            elif self.mode == LEDControlMode.SIMULATOR:
                logger.info("LED controller in simulator mode (no hardware)")
            
            self._check_for_light_sensor()
        except Exception as e:
            logger.warning(f"Hardware initialization failed: {e}. Falling back to simulator mode.")
            self.mode = LEDControlMode.SIMULATOR
    
    def _init_gpio(self):
        """Initialize GPIO for LED control"""
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            self.gpio = GPIO
            
            if self.mode == LEDControlMode.PWM:
                self.pwm = GPIO.PWM(self.gpio_pin, 1000)  # 1kHz frequency
                self.pwm.start(0)
                logger.info(f"GPIO PWM initialized on pin {self.gpio_pin}")
            else:
                logger.info(f"GPIO initialized on pin {self.gpio_pin}")
        except ImportError:
            logger.warning("RPi.GPIO not available - LED GPIO control disabled")
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")
    
    def _init_relay(self):
        """Initialize USB relay control"""
        try:
            # Example using pyusb for relay control
            # You may need to install: pip install pyusb
            import usb.core
            import usb.util
            
            # Find relay device (customize vendor/product IDs for your relay)
            self.relay = usb.core.find(find_all=True, idVendor=0x16c0, idProduct=0x05df)
            if self.relay:
                logger.info("USB relay initialized")
            else:
                logger.warning("USB relay not found")
        except ImportError:
            logger.warning("pyusb not available - USB relay control disabled")
        except Exception as e:
            logger.warning(f"USB relay initialization failed: {e}")
    
    def _check_for_light_sensor(self):
        """
        Check for light intensity sensor availability
        
        Note: Light sensor is OPTIONAL
        - You don't need it to control LED on/off and brightness
        - Useful for: Auto-adjustment, feedback, diagnostics
        - Supported sensors: BH1750, TSL2561, VEML7700
        """
        try:
            # Try to import light sensor libraries
            try:
                import board
                import adafruit_bh1750
                i2c = board.I2C()
                sensor = adafruit_bh1750.Adafruit_BH1750(i2c)
                self.light_sensor = sensor
                self.sensor_available = True
                logger.info("Light intensity sensor (BH1750) detected and ready")
                return
            except:
                pass
            
            # Try TSL2561
            try:
                import adafruit_tsl2561
                i2c = board.I2C()
                sensor = adafruit_tsl2561.Adafruit_TSL2561(i2c)
                self.light_sensor = sensor
                self.sensor_available = True
                logger.info("Light intensity sensor (TSL2561) detected and ready")
                return
            except:
                pass
            
            logger.info("No light sensor detected - brightness control will be software-based only")
            self.sensor_available = False
            
        except Exception as e:
            logger.info(f"Light sensor check failed: {e}")
            self.sensor_available = False
    
    def get_light_level(self) -> Optional[float]:
        """
        Get current light level from sensor (if available)
        
        Returns:
            Light level in lux, or None if sensor not available
        """
        if not self.sensor_available:
            return None
        
        try:
            return self.light_sensor.lux
        except Exception as e:
            logger.warning(f"Failed to read light sensor: {e}")
            return None
    
    def set_intensity(self, intensity: int) -> bool:
        """
        Set LED brightness intensity (0-100%)
        
        Args:
            intensity: Brightness level (0=off, 100=max)
            
        Returns:
            True if successful, False otherwise
        """
        with self.lock:
            # Validate intensity
            if intensity < 0 or intensity > 100:
                logger.error(f"Invalid intensity: {intensity}. Must be 0-100.")
                return False
            
            # Check temperature before setting
            if not self._check_temperature():
                logger.warning("LED not set due to temperature warning")
                return False
            
            self.target_intensity = intensity
            
            if intensity == 0:
                self.is_on = False
            else:
                self.is_on = True
            
            # Apply intensity based on mode
            if self.mode == LEDControlMode.PWM and self.pwm:
                self.pwm.ChangeDutyCycle(intensity)
                self.current_intensity = intensity
                logger.debug(f"LED PWM duty cycle set to {intensity}%")
                return True
            
            elif self.mode == LEDControlMode.GPIO and self.gpio:
                self.gpio.output(self.gpio_pin, intensity > 0)
                self.current_intensity = intensity if intensity > 0 else 0
                logger.debug(f"LED GPIO set to {'ON' if intensity > 0 else 'OFF'}")
                return True
            
            elif self.mode == LEDControlMode.RELAY:
                self._control_relay(intensity > 0)
                self.current_intensity = intensity if intensity > 0 else 0
                return True
            
            elif self.mode == LEDControlMode.SIMULATOR:
                self.current_intensity = intensity
                logger.info(f"[SIMULATOR] LED intensity set to {intensity}%")
                return True
            
            return False
    
    def _control_relay(self, turn_on: bool):
        """Control USB relay"""
        if not self.relay:
            logger.warning("Relay not available")
            return
        
        try:
            # Example relay control command
            # Adjust based on your specific relay model
            cmd = b'\xaa\x11\x01' if turn_on else b'\xaa\x10\x01'
            self.relay.write(0x01, cmd)
            logger.debug(f"Relay {'ON' if turn_on else 'OFF'}")
        except Exception as e:
            logger.error(f"Failed to control relay: {e}")
    
    def turn_on(self, intensity: int = 75) -> bool:
        """
        Turn LED on at specified intensity
        
        Args:
            intensity: Brightness level (default 75%)
            
        Returns:
            True if successful
        """
        return self.set_intensity(intensity)
    
    def turn_off(self) -> bool:
        """
        Turn LED off
        
        Returns:
            True if successful
        """
        return self.set_intensity(0)
    
    def get_status(self) -> Dict:
        """
        Get current LED status
        
        Returns:
            Dictionary with LED status information
        """
        return {
            "is_on": self.is_on,
            "current_intensity": self.current_intensity,
            "target_intensity": self.target_intensity,
            "mode": self.mode.value,
            "gpio_pin": self.gpio_pin,
            "temperature": round(self.current_temp, 1),
            "temperature_warning": self.current_temp > self.warning_temp,
            "sensor_available": self.sensor_available,
            "light_level_lux": self.get_light_level()
        }
    
    def set_preset(self, preset_name: str) -> bool:
        """
        Set LED to predefined brightness level
        
        Available presets:
        - "off": 0%
        - "dim": 25%
        - "medium": 50%
        - "bright": 75%
        - "max": 100%
        
        Args:
            preset_name: Name of preset
            
        Returns:
            True if successful
        """
        if preset_name not in self.presets:
            logger.error(f"Unknown preset: {preset_name}")
            return False
        
        intensity = self.presets[preset_name]
        return self.set_intensity(intensity)
    
    def get_presets(self) -> Dict:
        """Get available brightness presets"""
        return self.presets.copy()
    
    def _check_temperature(self) -> bool:
        """
        Check LED temperature and apply safety measures
        
        Returns:
            False if temperature is critical
        """
        if self.current_temp > self.max_temp:
            logger.error(f"LED temperature critical: {self.current_temp}°C. Shutting down.")
            self.turn_off()
            return False
        
        if self.current_temp > self.warning_temp:
            logger.warning(f"LED temperature warning: {self.current_temp}°C")
        
        return True
    
    def set_temperature(self, temp: float):
        """
        Set current LED temperature (for monitoring)
        
        Args:
            temp: Temperature in Celsius
        """
        with self.lock:
            self.current_temp = temp
            self._check_temperature()
    
    def start_monitoring(self):
        """Start background monitoring thread"""
        if self.running:
            return
        
        self.running = True
        self.control_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.control_thread.start()
        logger.info("LED monitoring thread started")
    
    def stop_monitoring(self):
        """Stop background monitoring thread"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=2)
        logger.info("LED monitoring thread stopped")
    
    def _monitor_loop(self):
        """Background monitoring loop"""
        while self.running:
            try:
                with self.lock:
                    # Smooth intensity transitions
                    if self.current_intensity != self.target_intensity:
                        diff = self.target_intensity - self.current_intensity
                        step = 1 if diff > 0 else -1
                        self.current_intensity += step
                        
                        if self.mode == LEDControlMode.PWM and self.pwm:
                            self.pwm.ChangeDutyCycle(self.current_intensity)
                
                # Check temperature periodically
                self._check_temperature()
                
                time.sleep(0.1)  # Update every 100ms
            except Exception as e:
                logger.error(f"Error in monitoring loop: {e}")
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_monitoring()
        
        try:
            if self.pwm:
                self.pwm.stop()
            if self.gpio:
                self.gpio.cleanup()
        except Exception as e:
            logger.error(f"Cleanup error: {e}")
        
        logger.info("LED controller cleaned up")


# Convenience functions
def create_led_controller(mode: str = "simulator", gpio_pin: int = 17) -> LEDController:
    """Factory function to create LED controller"""
    try:
        mode_enum = LEDControlMode[mode.upper()]
    except KeyError:
        logger.warning(f"Unknown mode {mode}, using simulator")
        mode_enum = LEDControlMode.SIMULATOR
    
    return LEDController(mode=mode_enum, gpio_pin=gpio_pin)
