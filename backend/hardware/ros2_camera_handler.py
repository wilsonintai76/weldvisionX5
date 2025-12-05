"""
ROS2 Camera Handler for WeldVision X5
Handles RDK Stereo Camera input via TROS (Together ROS)

Provides dual RGB and depth streams with efficient buffering
"""

import threading
import logging
import time
from typing import Optional, Tuple
from queue import Queue, Empty
import numpy as np

logger = logging.getLogger(__name__)


class ROS2CameraHandler:
    """
    Handles RDK Stereo Camera via ROS2/TROS
    
    Subscribes to:
    - /image_raw or /camera/rgb/image_raw (RGB stream)
    - /depth_raw or /camera/depth/image_raw (Depth stream)
    
    Provides buffered access to latest frames
    """
    
    def __init__(self, 
                 node_name: str = 'weld_vision_camera',
                 rgb_topic: str = '/image_raw',
                 depth_topic: str = '/depth_raw',
                 buffer_size: int = 5):
        """
        Initialize camera handler
        
        Args:
            node_name: ROS2 node name
            rgb_topic: RGB topic name
            depth_topic: Depth topic name
            buffer_size: Frame buffer size
        """
        self.node_name = node_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.buffer_size = buffer_size
        
        # Buffers
        self.rgb_queue: Queue = Queue(maxsize=buffer_size)
        self.depth_queue: Queue = Queue(maxsize=buffer_size)
        
        # State
        self.is_ready = False
        self.is_running = False
        self.node = None
        self.executor = None
        
        # Statistics
        self.rgb_frame_count = 0
        self.depth_frame_count = 0
        self.last_rgb_time = None
        self.last_depth_time = None
        self.latency_ms = 0.0
    
    def connect(self) -> bool:
        """
        Connect to ROS2 camera topics
        
        Returns:
            True if connection successful
        """
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            
            logger.info(f"Connecting to ROS2 camera topics...")
            logger.info(f"  RGB topic: {self.rgb_topic}")
            logger.info(f"  Depth topic: {self.depth_topic}")
            
            # Initialize ROS2
            if not rclpy.ok():
                rclpy.init()
            
            # Create node
            self.node = rclpy.create_node(self.node_name)
            self.cv_bridge = CvBridge()
            
            # Subscribe to topics
            self.node.create_subscription(
                Image,
                self.rgb_topic,
                self._rgb_callback,
                qos_profile=10
            )
            
            self.node.create_subscription(
                Image,
                self.depth_topic,
                self._depth_callback,
                qos_profile=10
            )
            
            logger.info("ROS2 subscriptions created")
            
            # Start executor thread
            from rclpy.executors import MultiThreadedExecutor
            self.executor = MultiThreadedExecutor(num_threads=2)
            self.executor.add_node(self.node)
            
            self.is_running = True
            executor_thread = threading.Thread(
                target=self._executor_worker,
                daemon=True,
                name="ROS2CameraExecutor"
            )
            executor_thread.start()
            
            # Wait for first frames
            logger.info("Waiting for first frames...")
            timeout = time.time() + 5.0
            while time.time() < timeout:
                if self.rgb_frame_count > 0 and self.depth_frame_count > 0:
                    self.is_ready = True
                    logger.info("Camera ready!")
                    return True
                time.sleep(0.1)
            
            logger.warning("Timeout waiting for frames, but initialized")
            self.is_ready = True
            return True
            
        except ImportError as e:
            logger.error(f"ROS2 import error: {e}")
            logger.info("Falling back to mock camera mode")
            self.is_ready = False
            return False
        except Exception as e:
            logger.error(f"Camera connection error: {e}")
            return False
    
    def _executor_worker(self) -> None:
        """Background executor worker"""
        try:
            while self.is_running:
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            logger.error(f"Executor error: {e}")
        finally:
            logger.info("Executor stopped")
    
    def _rgb_callback(self, msg) -> None:
        """RGB frame callback"""
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Drop oldest if buffer full
            if self.rgb_queue.full():
                try:
                    self.rgb_queue.get_nowait()
                except Empty:
                    pass
            
            self.rgb_queue.put_nowait((frame, time.time()))
            self.rgb_frame_count += 1
            self.last_rgb_time = time.time()
            
            if self.rgb_frame_count % 30 == 0:
                logger.debug(f"RGB frames: {self.rgb_frame_count}")
                
        except Exception as e:
            logger.warning(f"RGB callback error: {e}")
    
    def _depth_callback(self, msg) -> None:
        """Depth frame callback"""
        try:
            # Depth is typically 16UC1 (uint16, 1 channel)
            depth = self.cv_bridge.imgmsg_to_cv2(msg, "16UC1")
            
            # Drop oldest if buffer full
            if self.depth_queue.full():
                try:
                    self.depth_queue.get_nowait()
                except Empty:
                    pass
            
            self.depth_queue.put_nowait((depth, time.time()))
            self.depth_frame_count += 1
            self.last_depth_time = time.time()
            
            if self.depth_frame_count % 30 == 0:
                logger.debug(f"Depth frames: {self.depth_frame_count}")
                
        except Exception as e:
            logger.warning(f"Depth callback error: {e}")
    
    def get_latest_frames(self, 
                         timeout: float = 1.0) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get latest RGB and depth frames
        
        Args:
            timeout: Wait timeout in seconds
            
        Returns:
            Tuple of (rgb_frame, depth_frame) or (None, None) if not available
        """
        try:
            rgb_frame = None
            depth_frame = None
            
            # Drain queues to get latest
            while not self.rgb_queue.empty():
                try:
                    rgb_frame, rgb_time = self.rgb_queue.get_nowait()
                except Empty:
                    break
            
            while not self.depth_queue.empty():
                try:
                    depth_frame, depth_time = self.depth_queue.get_nowait()
                except Empty:
                    break
            
            return rgb_frame, depth_frame
            
        except Exception as e:
            logger.warning(f"Get frames error: {e}")
            return None, None
    
    def is_ready(self) -> bool:
        """Check if camera is ready to capture"""
        return self.is_ready and self.rgb_frame_count > 0 and self.depth_frame_count > 0
    
    def get_stats(self) -> dict:
        """Get camera statistics"""
        return {
            'is_ready': self.is_ready,
            'rgb_frame_count': self.rgb_frame_count,
            'depth_frame_count': self.depth_frame_count,
            'rgb_buffer_size': self.rgb_queue.qsize(),
            'depth_buffer_size': self.depth_queue.qsize(),
            'last_rgb_time': self.last_rgb_time,
            'last_depth_time': self.last_depth_time,
        }
    
    def disconnect(self) -> None:
        """Disconnect from ROS2"""
        try:
            logger.info("Disconnecting from camera...")
            self.is_running = False
            
            if self.executor:
                self.executor.shutdown()
            
            if self.node:
                self.node.destroy_node()
            
            logger.info("Camera disconnected")
            
        except Exception as e:
            logger.error(f"Disconnect error: {e}")
