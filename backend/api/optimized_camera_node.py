"""
Optimized ROS2 Camera Node Handler for RDK X5
Implements best practices for edge device operation
"""

import threading
import logging
import time
from typing import Optional, Tuple
from queue import Queue, Empty

logger = logging.getLogger(__name__)


class OptimizedCameraNode(threading.Thread):
    """
    Optimized ROS2 camera node for RDK X5
    - Proper executor configuration
    - Error recovery
    - Resource cleanup
    - Health monitoring
    """
    
    def __init__(self, node_name: str = 'weld_eval_backend', 
                 executor_type: str = 'MultiThreadedExecutor',
                 num_threads: int = 4):
        super().__init__()
        self.daemon = True
        self.running = True
        self.node_name = node_name
        self.executor_type = executor_type
        self.num_threads = num_threads
        
        # Frame buffers with threading
        self.frame_queue = Queue(maxsize=5)
        self.depth_queue = Queue(maxsize=3)
        
        # Node and executor references
        self.node = None
        self.executor = None
        self.rclpy = None
        
        # Health monitoring
        self.last_frame_time = None
        self.last_depth_time = None
        self.frame_count = 0
        self.depth_count = 0
        self.error_count = 0
        
        # Topic discovery
        self.image_topic = None
        self.depth_topic = None
        
    def discover_camera_topics(self):
        """Discover available camera topics dynamically"""
        try:
            if not self.node:
                return False
                
            topics_and_types = self.node.get_topic_names_and_types()
            
            # Preferred topic names
            image_candidates = [
                '/image_raw',
                '/camera/rgb/image_raw',
                '/usb_cam/image_raw',
                '/front_camera/image_raw',
            ]
            
            depth_candidates = [
                '/depth_raw',
                '/camera/depth/image_raw',
                '/camera/depth_registered/image_raw',
                '/depth_camera/depth/image_raw',
            ]
            
            # Find matching topics
            for topic, types in topics_and_types:
                if not self.image_topic:
                    for candidate in image_candidates:
                        if candidate in topic and 'Image' in str(types):
                            self.image_topic = topic
                            logger.info(f"Discovered image topic: {topic}")
                            break
                
                if not self.depth_topic:
                    for candidate in depth_candidates:
                        if candidate in topic and 'Image' in str(types):
                            self.depth_topic = topic
                            logger.info(f"Discovered depth topic: {topic}")
                            break
            
            return bool(self.image_topic or self.depth_topic)
            
        except Exception as e:
            logger.error(f"Topic discovery failed: {e}")
            return False
    
    def create_subscriptions(self):
        """Create subscriptions with error handling"""
        try:
            if not self.node:
                return False
            
            from sensor_msgs.msg import Image
            
            # Use discovered topics or defaults
            image_topic = self.image_topic or '/image_raw'
            depth_topic = self.depth_topic or '/depth_raw'
            
            # QoS settings optimized for edge device
            from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
            
            qos_image = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            
            qos_depth = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=3,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            
            self.node.create_subscription(
                Image,
                image_topic,
                self._image_callback,
                qos_image
            )
            logger.info(f"Subscribed to {image_topic}")
            
            self.node.create_subscription(
                Image,
                depth_topic,
                self._depth_callback,
                qos_depth
            )
            logger.info(f"Subscribed to {depth_topic}")
            
            return True
            
        except Exception as e:
            logger.error(f"Subscription creation failed: {e}")
            self.error_count += 1
            return False
    
    def _image_callback(self, msg):
        """Process image callback"""
        try:
            from cv_bridge import CvBridge
            
            if not hasattr(self, '_bridge'):
                self._bridge = CvBridge()
            
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Put in queue, drop oldest if full
            try:
                self.frame_queue.put_nowait(frame)
            except:
                try:
                    self.frame_queue.get_nowait()  # Remove oldest
                    self.frame_queue.put_nowait(frame)
                except:
                    pass
            
            self.last_frame_time = time.time()
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                logger.debug(f"Frames received: {self.frame_count}")
                
        except Exception as e:
            logger.error(f"Image callback error: {e}")
            self.error_count += 1
    
    def _depth_callback(self, msg):
        """Process depth callback"""
        try:
            from cv_bridge import CvBridge
            
            if not hasattr(self, '_bridge'):
                self._bridge = CvBridge()
            
            depth = self._bridge.imgmsg_to_cv2(msg, "16UC1")
            
            # Put in queue, drop oldest if full
            try:
                self.depth_queue.put_nowait(depth)
            except:
                try:
                    self.depth_queue.get_nowait()  # Remove oldest
                    self.depth_queue.put_nowait(depth)
                except:
                    pass
            
            self.last_depth_time = time.time()
            self.depth_count += 1
            
            if self.depth_count % 30 == 0:
                logger.debug(f"Depth frames received: {self.depth_count}")
                
        except Exception as e:
            logger.error(f"Depth callback error: {e}")
            self.error_count += 1
    
    def run(self):
        """Main ROS2 node execution loop"""
        try:
            import rclpy
            self.rclpy = rclpy
            
            logger.info("Initializing ROS2...")
            rclpy.init()
            
            logger.info(f"Creating ROS2 node: {self.node_name}")
            self.node = rclpy.create_node(self.node_name)
            
            # Create executor
            if self.executor_type == 'MultiThreadedExecutor':
                from rclpy.executors import MultiThreadedExecutor
                self.executor = MultiThreadedExecutor(num_threads=self.num_threads)
            else:
                from rclpy.executors import SingleThreadedExecutor
                self.executor = SingleThreadedExecutor()
            
            self.executor.add_node(self.node)
            logger.info(f"Using {self.executor_type} with {self.num_threads} threads")
            
            # Discover topics
            logger.info("Discovering camera topics...")
            if not self.discover_camera_topics():
                logger.warning("No camera topics found, using defaults")
            
            # Create subscriptions
            if not self.create_subscriptions():
                logger.error("Failed to create subscriptions")
                return
            
            logger.info("ROS2 camera node ready - spinning...")
            
            # Main spin loop
            while self.running:
                try:
                    self.executor.spin_once(timeout_sec=0.1)
                except Exception as e:
                    logger.error(f"Spin error: {e}")
                    self.error_count += 1
                    if self.error_count > 10:
                        logger.critical("Too many errors, shutting down")
                        break
                    time.sleep(1)
                    
        except Exception as e:
            logger.error(f"ROS2 node fatal error: {e}")
            self.running = False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup ROS2 resources"""
        try:
            logger.info("Cleaning up ROS2 resources...")
            
            if self.executor:
                try:
                    self.executor.shutdown()
                except:
                    pass
            
            if self.node:
                try:
                    self.node.destroy_node()
                except:
                    pass
            
            if self.rclpy:
                try:
                    self.rclpy.shutdown()
                except:
                    pass
            
            logger.info("ROS2 cleanup complete")
            
        except Exception as e:
            logger.error(f"Cleanup error: {e}")
    
    def get_latest_frame(self) -> Optional[any]:
        """Get latest frame from buffer"""
        try:
            return self.frame_queue.get_nowait()
        except Empty:
            return None
    
    def get_latest_depth(self) -> Optional[any]:
        """Get latest depth data from buffer"""
        try:
            return self.depth_queue.get_nowait()
        except Empty:
            return None
    
    def get_health_status(self) -> dict:
        """Get node health status"""
        current_time = time.time()
        frame_latency = current_time - self.last_frame_time if self.last_frame_time else None
        depth_latency = current_time - self.last_depth_time if self.last_depth_time else None
        
        return {
            'running': self.running,
            'frame_count': self.frame_count,
            'depth_count': self.depth_count,
            'error_count': self.error_count,
            'frame_latency_sec': frame_latency,
            'depth_latency_sec': depth_latency,
            'image_topic': self.image_topic,
            'depth_topic': self.depth_topic,
        }
    
    def stop(self):
        """Stop the camera node gracefully"""
        logger.info("Stopping camera node...")
        self.running = False
        self.join(timeout=5)
