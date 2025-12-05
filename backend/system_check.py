"""
Hardware and system detection module for WeldMaster AI

Checks for:
- RDK X5 platform availability
- RDK Stereo Camera connectivity
- Database initialization
- System dependencies
"""

import os
import sys
import subprocess
import platform
from typing import Dict, Any
import logging

logger = logging.getLogger(__name__)


class HardwareDetector:
    """Detects and manages hardware availability"""
    
    def __init__(self):
        self.rdk_x5_available = False
        self.camera_available = False
        self.ros2_available = False
        self.rdk_camera_present = False
        self.system_info = {}
        self.detection_messages = []
        
    def detect_all(self) -> Dict[str, Any]:
        """Run all hardware detection checks"""
        logger.info("Starting hardware detection...")
        
        # Basic system info
        self._detect_system_info()
        
        # Platform detection
        self._detect_rdk_x5()
        self._detect_ros2()
        self._detect_camera()
        
        result = {
            'system': self.system_info,
            'rdk_x5_available': self.rdk_x5_available,
            'ros2_available': self.ros2_available,
            'camera_available': self.camera_available,
            'rdk_camera_present': self.rdk_camera_present,
            'detection_messages': self.detection_messages,
            'ready_for_operation': self.camera_available or self.ros2_available,
        }
        
        logger.info(f"Hardware detection complete: {result}")
        return result
    
    def _detect_system_info(self):
        """Detect basic system information"""
        try:
            self.system_info = {
                'platform': platform.system(),
                'platform_release': platform.release(),
                'platform_version': platform.version(),
                'architecture': platform.machine(),
                'python_version': platform.python_version(),
                'hostname': platform.node(),
            }
            logger.debug(f"System info: {self.system_info}")
        except Exception as e:
            logger.warning(f"Failed to get system info: {e}")
    
    def _detect_rdk_x5(self):
        """Detect if running on Horizon Robotics RDK X5"""
        try:
            # Check for RDK X5 specific files/markers
            rdk_markers = [
                '/etc/rdk_version',
                '/opt/horizon/setup_env.sh',
                '/sys/class/gpio',  # GPIO available on embedded systems
            ]
            
            found_markers = []
            for marker in rdk_markers:
                if os.path.exists(marker):
                    found_markers.append(marker)
            
            if found_markers:
                self.rdk_x5_available = True
                msg = f"✅ RDK X5 detected (markers: {', '.join(found_markers)})"
                logger.info(msg)
                self.detection_messages.append(msg)
            else:
                msg = "⚠️ RDK X5 not detected (running on standard Linux/other platform)"
                logger.warning(msg)
                self.detection_messages.append(msg)
                
        except Exception as e:
            msg = f"❌ Error detecting RDK X5: {str(e)}"
            logger.error(msg)
            self.detection_messages.append(msg)
    
    def _detect_ros2(self):
        """Detect ROS2 availability and configuration"""
        try:
            # Try to import ROS2
            import rclpy
            self.ros2_available = True
            
            # Get ROS2 version info
            version = rclpy.__version__ if hasattr(rclpy, '__version__') else 'unknown'
            distro = os.environ.get('ROS_DISTRO', 'unknown')
            
            msg = f"✅ ROS2 available (rclpy: {version}, distro: {distro})"
            logger.info(msg)
            self.detection_messages.append(msg)
            
            # Check for DDS implementation
            rmw_impl = os.environ.get('RMW_IMPLEMENTATION', 'default')
            logger.debug(f"ROS2 DDS Implementation: {rmw_impl}")
            
        except ImportError:
            msg = "⚠️ ROS2 not available (rclpy not installed)"
            logger.warning(msg)
            self.detection_messages.append(msg)
        except Exception as e:
            msg = f"❌ Error detecting ROS2: {str(e)}"
            logger.error(msg)
            self.detection_messages.append(msg)
    
    def _detect_camera(self):
        """Detect camera/depth sensor availability"""
        try:
            import cv2
            
            # Check for RDK Stereo Camera via ROS2
            if self.ros2_available:
                try:
                    import rclpy
                    from sensor_msgs.msg import Image
                    
                    # Try to create a test ROS2 node to check for camera topics
                    rclpy.init(args=None)
                    node = rclpy.create_node('test_camera_detector')
                    
                    # Check for RDK camera topics
                    topics = node.get_topic_names_and_types()
                    camera_topics = [
                        t for t, _ in topics 
                        if 'image' in t.lower() or 'depth' in t.lower()
                    ]
                    
                    node.destroy_node()
                    rclpy.shutdown()
                    
                    if camera_topics:
                        self.rdk_camera_present = True
                        self.camera_available = True
                        msg = f"✅ RDK Stereo Camera detected (topics: {', '.join(camera_topics)})"
                        logger.info(msg)
                        self.detection_messages.append(msg)
                    else:
                        msg = "⚠️ No camera topics found on ROS2"
                        logger.warning(msg)
                        self.detection_messages.append(msg)
                        
                except Exception as e:
                    msg = f"⚠️ ROS2 camera detection failed: {str(e)}"
                    logger.warning(msg)
                    self.detection_messages.append(msg)
            
            # Fallback: Check for USB cameras via OpenCV
            if not self.camera_available:
                camera_index = cv2.CAP_ANY
                cap = cv2.VideoCapture(camera_index)
                
                if cap.isOpened():
                    self.camera_available = True
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        h, w = frame.shape[:2]
                        msg = f"✅ USB Camera detected (resolution: {w}x{h})"
                        logger.info(msg)
                        self.detection_messages.append(msg)
                    else:
                        msg = "⚠️ Camera detected but cannot capture frames"
                        logger.warning(msg)
                        self.detection_messages.append(msg)
                    cap.release()
                else:
                    msg = "⚠️ No camera devices detected"
                    logger.warning(msg)
                    self.detection_messages.append(msg)
                    
        except ImportError:
            msg = "❌ OpenCV not installed (cannot detect camera)"
            logger.error(msg)
            self.detection_messages.append(msg)
        except Exception as e:
            msg = f"❌ Error detecting camera: {str(e)}"
            logger.error(msg)
            self.detection_messages.append(msg)


class DatabaseManager:
    """Manages database initialization and checks"""
    
    def __init__(self, db_path: str = 'sqlite:///weld_data.db'):
        self.db_path = db_path
        self.db_exists = False
        self.db_ready = False
        self.init_messages = []
        
    def initialize(self):
        """Initialize database if it doesn't exist"""
        logger.info(f"Initializing database at: {self.db_path}")
        
        try:
            # Extract actual file path from SQLAlchemy URI
            if self.db_path.startswith('sqlite:///'):
                file_path = self.db_path.replace('sqlite:///', '')
            else:
                file_path = self.db_path
            
            # Check if database file exists
            if os.path.exists(file_path):
                self.db_exists = True
                msg = f"✅ Database already exists: {file_path}"
                logger.info(msg)
                self.init_messages.append(msg)
            else:
                msg = f"ℹ️  Creating new database: {file_path}"
                logger.info(msg)
                self.init_messages.append(msg)
            
            # Import here to avoid circular imports
            from database.models import init_db
            session_factory = init_db(self.db_path)
            self.db_ready = True
            
            msg = "✅ Database initialized and ready"
            logger.info(msg)
            self.init_messages.append(msg)
            
            return session_factory
            
        except Exception as e:
            msg = f"❌ Database initialization failed: {str(e)}"
            logger.error(msg)
            self.init_messages.append(msg)
            raise
    
    def get_db_status(self) -> Dict[str, Any]:
        """Get database status information"""
        return {
            'db_path': self.db_path,
            'db_exists': self.db_exists,
            'db_ready': self.db_ready,
            'init_messages': self.init_messages,
        }


def setup_logging(log_level=logging.INFO):
    """Setup logging configuration"""
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler('weld_evaluator.log'),
        ]
    )
    return logging.getLogger(__name__)
