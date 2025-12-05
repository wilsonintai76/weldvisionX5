"""
ROS2 Configuration for RDK X5 WeldMaster AI
Optimized settings for Horizon Robotics RDK X5 edge device
"""

import logging

logger = logging.getLogger(__name__)

# ROS2 Node Configuration
ROS2_NODE_CONFIG = {
    'node_name': 'weld_evaluator_backend',
    'namespace': '/weld_master',
    
    # Executor configuration
    'executor_type': 'MultiThreadedExecutor',  # Better for multiple subscriptions
    'num_threads': 4,  # Balanced for RDK X5 performance
    'spin_timeout': 0.1,  # Non-blocking spin timeout (seconds)
    
    # QoS Settings - optimized for edge device
    'image_qos': {
        'history': 'KEEP_LAST',
        'depth': 5,  # Reduced buffer for edge device
        'reliability': 'RELIABLE',
        'durability': 'VOLATILE',
    },
    'depth_qos': {
        'history': 'KEEP_LAST',
        'depth': 3,
        'reliability': 'RELIABLE',
        'durability': 'VOLATILE',
    },
}

# Camera Topics - Primary for RDK Stereo Camera
CAMERA_TOPICS = {
    'primary': {
        'image': '/image_raw',  # RDK stereo camera RGB
        'depth': '/depth_raw',  # RDK stereo camera depth
    },
    'alternative': {
        'image': '/camera/rgb/image_raw',
        'depth': '/camera/depth/image_raw',
    },
    'fallback': {
        'image': '/usb_cam/image_raw',
    }
}

# RDK X5 Hardware Detection Markers
RDK_MARKERS = [
    '/etc/rdk_version',
    '/opt/horizon/setup_env.sh',
    '/sys/class/gpio',
]

# ROS2 Environment Variables for RDK X5
ROS2_ENV = {
    'ROS_DOMAIN_ID': '0',  # Default domain
    'ROS_LOCALHOST_ONLY': '1',  # Local communication only
    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',  # Fast DDS for edge
}

# Thread Configuration
THREADING_CONFIG = {
    'camera_thread_timeout': 30,  # seconds
    'node_spin_rate': 10,  # Hz
    'frame_buffer_size': 5,
    'depth_buffer_size': 3,
}

# Error Recovery
ERROR_RECOVERY = {
    'max_retries': 3,
    'retry_delay': 2,  # seconds
    'node_timeout': 10,  # seconds
    'topic_check_interval': 5,  # seconds
}

# Performance Optimization
PERFORMANCE = {
    'frame_rate_limit': 30,  # FPS cap for edge device
    'image_compression': True,
    'depth_downsampling': True,  # Reduce depth resolution
    'target_image_width': 640,
    'target_image_height': 480,
}


def get_ros2_config():
    """Get ROS2 configuration dictionary"""
    return {
        'node_config': ROS2_NODE_CONFIG,
        'camera_topics': CAMERA_TOPICS,
        'rdk_markers': RDK_MARKERS,
        'threading': THREADING_CONFIG,
        'error_recovery': ERROR_RECOVERY,
        'performance': PERFORMANCE,
    }


def optimize_for_rdk_x5():
    """Print optimization recommendations for RDK X5"""
    recommendations = """
    ========================================
    RDK X5 ROS2 Optimization Recommendations
    ========================================
    
    1. DDS Configuration
       - Use CycloneDDS for lower latency
       - Disable unused transports
       - Set RMW_IMPLEMENTATION to rmw_cyclonedds_cpp
    
    2. Resource Management
       - Set ROS_LOCALHOST_ONLY=1 for local device
       - Limit thread pool size to 4
       - Use KEEP_LAST QoS with depth=5
    
    3. Frame Processing
       - Implement frame buffering
       - Use threading for subscription callbacks
       - Limit frame rate to 30 FPS
    
    4. Network Optimization
       - Run backend on RDK X5 directly
       - Use Unix sockets for local communication
       - Minimize inter-process communication
    
    5. Monitoring
       - Monitor memory usage
       - Track ROS2 node health
       - Log topic latency
       - Implement graceful shutdown
    
    6. Error Handling
       - Implement reconnection logic
       - Validate topic availability
       - Add timeout mechanisms
       - Graceful fallback to simulator mode
    """
    logger.info(recommendations)
    return recommendations
