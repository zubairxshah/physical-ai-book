# Robotics Code Generator Skill

**Skill Type**: Code Generation
**Domain**: Robotics, ROS 2, Computer Vision, Control Systems
**Version**: 1.0.0
**Created**: 2025-12-19

## Purpose

This skill generates production-quality robotics code with proper error handling, logging, and best practices specific to robot systems. It specializes in ROS 2, sensor processing, motion control, and AI model integration for physical robots.

## When to Use This Skill

- Creating ROS 2 nodes (publishers, subscribers, services, actions)
- Implementing robot control algorithms (PID, MPC, trajectory planning)
- Writing computer vision pipelines (camera processing, object detection)
- Integrating AI models (ONNX, TensorRT, PyTorch) with robot systems
- Generating simulation code (Gazebo plugins, Isaac Sim scripts)
- Building sensor interfaces (LiDAR, IMU, depth cameras)

## Input Requirements

```yaml
code_type: enum            # "ros2_node" | "control_algorithm" | "vision_pipeline" | "simulation" | "sensor_interface"
language: enum             # "python" | "cpp"
framework: string          # "rclpy", "rclcpp", "opencv", "pytorch", "isaac_sim"
robot_platform: string     # e.g., "generic", "unitree_g1", "turtlebot", "custom"
hardware: array            # List of sensors/actuators ["camera", "lidar", "imu", "servo"]
complexity: enum           # "basic" | "intermediate" | "production"
include_tests: boolean     # Generate unit tests (default: true)
include_launch: boolean    # Generate launch file for ROS 2 (default: true)
```

## Output Format

### Python ROS 2 Node Structure
```python
#!/usr/bin/env python3
"""
[Module docstring describing purpose and usage]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from typing import Optional, Tuple
import numpy as np

# Message imports
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist


class RobotNode(Node):
    """
    [Class docstring with detailed description]

    Attributes:
        attr1: Description
        attr2: Description
    """

    def __init__(self) -> None:
        super().__init__('robot_node')

        # Parameters
        self.declare_parameter('param_name', default_value)

        # Publishers
        self.publisher_ = self.create_publisher(...)

        # Subscribers
        self.subscriber_ = self.create_subscription(...)

        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Node initialized successfully')

    def timer_callback(self) -> None:
        """Main control loop callback"""
        try:
            # Implementation
            pass
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')

    def cleanup(self) -> None:
        """Cleanup resources before shutdown"""
        self.get_logger().info('Shutting down node')


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ ROS 2 Node Structure
```cpp
/**
 * @file robot_node.hpp
 * @brief [Brief description of node purpose]
 *
 * [Detailed description]
 */

#ifndef ROBOT_NODE_HPP
#define ROBOT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class RobotNode : public rclcpp::Node {
public:
    RobotNode();
    ~RobotNode();

private:
    void timer_callback();
    void cleanup();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double control_frequency_;
};

#endif // ROBOT_NODE_HPP
```

## Code Templates

### 1. ROS 2 Camera Processing Node

```python
#!/usr/bin/env python3
"""
Camera processing node for robot vision pipeline.

Subscribes to camera images, processes them, and publishes results.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraProcessorNode(Node):
    """
    Processes camera images for robot vision tasks.

    Subscribes to: /camera/image_raw
    Publishes to: /vision/processed_image, /vision/detections
    """

    def __init__(self) -> None:
        super().__init__('camera_processor')

        # CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('show_debug', False)
        self.show_debug = self.get_parameter('show_debug').value

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.processed_pub = self.create_publisher(
            Image,
            '/vision/processed_image',
            10
        )

        self.get_logger().info('Camera processor initialized')

    def image_callback(self, msg: Image) -> None:
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image
            processed = self.process_image(cv_image)

            # Convert back to ROS Image
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

    def process_image(self, image: np.ndarray) -> np.ndarray:
        """
        Apply computer vision processing to image.

        Args:
            image: Input BGR image

        Returns:
            Processed BGR image
        """
        # Example: Edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        result = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. PID Controller for Robot Motors

```python
"""
PID controller implementation for robot motor control.
"""

import time
from typing import Tuple
import numpy as np


class PIDController:
    """
    Proportional-Integral-Derivative controller.

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        output_limits: Tuple of (min, max) output values
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limits: Tuple[float, float] = (-1.0, 1.0)
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min, self.output_max = output_limits

        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def compute(self, setpoint: float, measured: float) -> float:
        """
        Compute PID control output.

        Args:
            setpoint: Desired value
            measured: Current measured value

        Returns:
            Control output (clamped to output_limits)
        """
        current_time = time.time()

        # Calculate error
        error = setpoint - measured

        # Calculate dt
        if self.prev_time is None:
            dt = 0.01  # Default
        else:
            dt = current_time - self.prev_time

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(
            self.integral,
            self.output_min / (self.ki + 1e-6),
            self.output_max / (self.ki + 1e-6)
        )
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0

        # Compute total output
        output = p_term + i_term + d_term

        # Clamp output
        output = np.clip(output, self.output_min, self.output_max)

        # Update state
        self.prev_error = error
        self.prev_time = current_time

        return output

    def reset(self) -> None:
        """Reset controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
```

### 3. ROS 2 Launch File

```python
"""
Launch file for robot vision system.

Usage:
    ros2 launch robot_pkg vision_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='false',
        description='Show debug visualizations'
    )

    # Camera node
    camera_node = Node(
        package='robot_pkg',
        executable='camera_node',
        name='camera',
        parameters=[{
            'frame_rate': 30,
            'resolution': [640, 480]
        }],
        output='screen'
    )

    # Vision processor node
    vision_node = Node(
        package='robot_pkg',
        executable='camera_processor',
        name='vision_processor',
        parameters=[{
            'show_debug': LaunchConfiguration('show_debug')
        }],
        output='screen'
    )

    return LaunchDescription([
        show_debug_arg,
        camera_node,
        vision_node
    ])
```

## Code Quality Standards

### Python Standards
- Type hints for all function signatures
- Docstrings following Google or NumPy style
- PEP 8 compliance (use `black` formatter)
- Error handling with specific exceptions
- Logging instead of print statements
- Resource cleanup in finally blocks

### C++ Standards
- Modern C++17/20 features
- Smart pointers (shared_ptr, unique_ptr)
- RAII for resource management
- Doxygen-style documentation
- Const correctness
- No raw pointers or manual memory management

### ROS 2 Specific
- Proper QoS profiles for different message types
- Parameter validation on startup
- Graceful shutdown handling
- Thread-safe operations
- Component-style nodes when applicable
- Lifecycle nodes for critical systems

## Testing Templates

### Python Unit Test
```python
"""
Unit tests for camera processor node.
"""

import unittest
import rclpy
from robot_pkg.camera_processor import CameraProcessorNode


class TestCameraProcessor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = CameraProcessorNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        """Test node initializes correctly"""
        self.assertIsNotNone(self.node.image_sub)
        self.assertIsNotNone(self.node.processed_pub)

    def test_image_processing(self):
        """Test image processing pipeline"""
        import numpy as np

        # Create dummy image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Process
        result = self.node.process_image(test_image)

        # Verify
        self.assertEqual(result.shape, test_image.shape)


if __name__ == '__main__':
    unittest.main()
```

## Common Patterns

### Pattern 1: Sensor Fusion
```python
class SensorFusionNode(Node):
    """Fuse multiple sensor inputs for robust perception"""

    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to multiple sensors
        self.camera_sub = self.create_subscription(...)
        self.lidar_sub = self.create_subscription(...)
        self.imu_sub = self.create_subscription(...)

        # Synchronized callback
        self.sensor_data = {
            'camera': None,
            'lidar': None,
            'imu': None
        }

        # Timer to fuse when all data available
        self.create_timer(0.05, self.fusion_callback)

    def fusion_callback(self):
        """Fuse sensor data when all sources updated"""
        if all(v is not None for v in self.sensor_data.values()):
            fused_result = self.fuse_sensors(self.sensor_data)
            # Publish fused result
```

### Pattern 2: State Machine for Robot Behaviors
```python
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    NAVIGATING = 1
    MANIPULATING = 2
    ERROR = 3


class RobotStateMachine(Node):
    """Manage robot behavior states"""

    def __init__(self):
        super().__init__('state_machine')
        self.state = RobotState.IDLE
        self.create_timer(0.1, self.state_callback)

    def state_callback(self):
        """Execute behavior based on current state"""
        if self.state == RobotState.IDLE:
            self.handle_idle()
        elif self.state == RobotState.NAVIGATING:
            self.handle_navigation()
        # ... other states

    def transition_to(self, new_state: RobotState):
        """Safely transition between states"""
        self.get_logger().info(f'Transitioning: {self.state} -> {new_state}')
        self.state = new_state
```

### Pattern 3: Async Action Server
```python
from rclpy.action import ActionServer
from robot_interfaces.action import Navigate


class NavigationActionServer(Node):
    """Action server for robot navigation"""

    def __init__(self):
        super().__init__('navigation_server')

        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        """Execute navigation to goal"""
        self.get_logger().info('Executing navigation...')

        # Feedback loop
        for progress in range(100):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Navigate.Result()

            # Publish feedback
            feedback = Navigate.Feedback()
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

            await asyncio.sleep(0.1)

        # Success
        goal_handle.succeed()
        result = Navigate.Result()
        result.success = True
        return result
```

## Performance Considerations

### For Real-Time Control
- Use fixed-rate timers instead of callbacks when possible
- Minimize memory allocations in hot paths
- Consider C++ for latency-critical components (<1ms)
- Use appropriate QoS profiles (RELIABLE vs BEST_EFFORT)

### For Computer Vision
- Resize images to minimum required resolution
- Use GPU acceleration (CUDA, TensorRT) when available
- Process every Nth frame if real-time not critical
- Use ROS 2 image transport for compression

### For Machine Learning
- Load models once at initialization, not per inference
- Batch processing when possible
- Use ONNX Runtime or TensorRT for faster inference
- Offload to GPU when available

## Error Handling

```python
def safe_robot_operation(self):
    """Template for safe robot operations"""
    try:
        # Attempt operation
        self.execute_motion()

    except RobotCollisionError as e:
        # Critical safety error
        self.emergency_stop()
        self.get_logger().error(f'Collision detected: {e}')
        self.transition_to(RobotState.ERROR)

    except CommunicationError as e:
        # Recoverable error
        self.get_logger().warning(f'Communication lost: {e}')
        self.retry_connection()

    except Exception as e:
        # Unknown error
        self.get_logger().error(f'Unexpected error: {e}')
        self.safe_shutdown()

    finally:
        # Always cleanup
        self.release_resources()
```

## Changelog

### v1.0.0 (2025-12-19)
- Initial skill creation
- Defined code generation patterns for ROS 2
- Created templates for common robotics tasks
- Established quality and testing standards

## Related Skills

- **Content Generation Skill**: Embeds generated code in book chapters
- **RAG Query Testing Skill**: Tests code example retrieval
- **Technical Reviewer Subagent**: Reviews code for safety and correctness

## Success Metrics

Generated code should achieve:
- ✅ 100% compilation/syntax correctness
- ✅ Proper error handling for robot safety
- ✅ Performance meets real-time requirements
- ✅ Follows ROS 2 and language best practices
- ✅ Includes comprehensive documentation and tests
