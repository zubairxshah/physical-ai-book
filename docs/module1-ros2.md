---
sidebar_position: 15
---

# Module 1: The Robotic Nervous System (ROS 2)

## Introduction to ROS 2

**ROS 2** (Robot Operating System 2) is the middleware backbone of modern robotics. Just as the nervous system coordinates messages between the brain and body, ROS 2 coordinates communication between robot sensors, actuators, and AI algorithms.

### Why ROS 2?

ROS 2 is a complete rewrite of the original ROS, designed for:
- ✅ **Real-time performance** - Critical for safety and responsiveness
- ✅ **Multi-robot systems** - Distributed communication using DDS
- ✅ **Production environments** - Security, reliability, and quality-of-service
- ✅ **Cross-platform support** - Linux, Windows, macOS
- ✅ **Embedded systems** - Runs on resource-constrained devices

### ROS 2 vs ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom TCP/UDP | DDS (Data Distribution Service) |
| Real-time | Limited | Native support |
| Security | None | Built-in encryption |
| Multi-robot | Difficult | Native support |
| Language Support | C++, Python | C++, Python, Rust, Java |
| Lifecycle | Basic | Advanced node lifecycle |

---

## Core Concepts

### 1. Nodes

**Nodes** are the fundamental building blocks - independent processes that perform specific tasks.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2 Node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Node Responsibilities:**
- Sensor data acquisition
- Motor control
- Path planning
- Computer vision processing
- Decision making

### 2. Topics (Publish/Subscribe)

**Topics** enable asynchronous, many-to-many communication.

```python
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # Create publisher
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        # Create timer (30 Hz)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        # ... fill image data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image')

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # Create subscriber
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')
        # Process image...
```

**Common Topics in Humanoid Robotics:**
- `/camera/image_raw` - RGB camera images
- `/depth/image` - Depth sensor data
- `/imu/data` - Inertial measurement unit
- `/joint_states` - Current joint positions/velocities
- `/cmd_vel` - Velocity commands
- `/odometry` - Robot pose estimation

### 3. Services (Request/Response)

**Services** provide synchronous, one-to-one communication for request/response patterns.

```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

# Service Server
class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response

# Service Client
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future
```

**Robotics Service Examples:**
- `/reset_odometry` - Reset robot position
- `/start_mapping` - Begin SLAM mapping
- `/save_map` - Save current map
- `/set_parameters` - Configure robot settings
- `/trigger_grasp` - Execute grasping action

### 4. Actions (Long-Running Tasks)

**Actions** handle long-running operations with feedback and cancellation.

```python
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from example_interfaces.action import Fibonacci

# Action Server
class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Provide feedback during execution
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

# Action Client
class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')
```

**Robotics Action Examples:**
- `/navigate_to_pose` - Navigate to target location
- `/follow_path` - Execute planned trajectory
- `/pick_and_place` - Manipulation task
- `/docking` - Autonomous docking sequence
- `/patrol` - Multi-waypoint patrol mission

---

## Bridging Python Agents to ROS 2 with rclpy

### Integration Architecture

```
┌─────────────────┐
│   AI Agent      │ (LLM, Planning)
│   (Python)      │
└────────┬────────┘
         │
    ┌────▼────┐
    │  rclpy  │ (ROS 2 Python Client)
    └────┬────┘
         │
┌────────▼────────────┐
│  ROS 2 Middleware   │ (DDS)
└────────┬────────────┘
         │
    ┌────▼─────┬──────────┬─────────┐
    │          │          │         │
┌───▼───┐ ┌───▼───┐ ┌────▼───┐ ┌──▼────┐
│Camera │ │Motors │ │Sensors │ │Gripper│
└───────┘ └───────┘ └────────┘ └───────┘
```

### Example: LLM-Controlled Robot

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from openai import OpenAI

class LLMRobotController(Node):
    def __init__(self):
        super().__init__('llm_robot_controller')

        # ROS 2 publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # OpenAI client
        self.llm = OpenAI(api_key="your-key")

        # System prompt
        self.system_prompt = """You are a robot controller.
        Convert natural language commands to robot actions.

        Available actions:
        - move_forward(speed)
        - move_backward(speed)
        - turn_left(angle)
        - turn_right(angle)
        - stop()

        Return JSON: {"action": "action_name", "params": {...}}
        """

    def process_command(self, user_command):
        """Convert natural language to robot action"""
        response = self.llm.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_command}
            ]
        )

        action_json = response.choices[0].message.content
        action = json.loads(action_json)

        # Execute action
        self.execute_action(action)

    def execute_action(self, action):
        """Execute robot action via ROS 2"""
        twist = Twist()

        if action['action'] == 'move_forward':
            twist.linear.x = action['params']['speed']
        elif action['action'] == 'turn_left':
            twist.angular.z = action['params']['angle']
        elif action['action'] == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Executed: {action}')

def main():
    rclpy.init()
    controller = LLMRobotController()

    # Example commands
    controller.process_command("Move forward slowly")
    controller.process_command("Turn 90 degrees to the right")
    controller.process_command("Stop the robot")

    rclpy.shutdown()
```

### Multi-Modal Perception Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionLanguageRobot(Node):
    def __init__(self):
        super().__init__('vision_language_robot')

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Subscribe to voice commands
        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.voice_callback, 10)

        # CV Bridge for image conversion
        self.bridge = CvBridge()
        self.latest_image = None

    def image_callback(self, msg):
        """Store latest image"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def voice_callback(self, msg):
        """Process voice command with visual context"""
        command = msg.data

        if self.latest_image is not None:
            # Encode image to base64 for vision-language model
            _, buffer = cv2.imencode('.jpg', self.latest_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')

            # Send to GPT-4V or similar
            response = self.process_with_vlm(command, image_base64)

            # Execute robot action based on response
            self.execute_response(response)

    def process_with_vlm(self, command, image_base64):
        """Send to vision-language model"""
        # Implementation with OpenAI GPT-4V, Gemini, etc.
        pass
```

---

## URDF: Unified Robot Description Format

**URDF** defines the robot's physical structure, including links, joints, sensors, and visual/collision geometry.

### URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Camera Sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.02"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Right Arm -->
  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.15 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Add legs, sensors, etc. -->

</robot>
```

### Loading URDF in ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET

class URDFController(Node):
    def __init__(self):
        super().__init__('urdf_controller')

        # Load URDF
        self.urdf_path = 'path/to/robot.urdf'
        self.robot_description = self.load_urdf(self.urdf_path)

        # Declare parameter
        self.declare_parameter('robot_description', self.robot_description)

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Parse joints from URDF
        self.joints = self.parse_joints()

    def load_urdf(self, path):
        """Load URDF file"""
        with open(path, 'r') as f:
            return f.read()

    def parse_joints(self):
        """Extract joint information"""
        tree = ET.fromstring(self.robot_description)
        joints = []
        for joint in tree.findall('.//joint'):
            if joint.get('type') in ['revolute', 'prismatic']:
                joints.append(joint.get('name'))
        return joints

    def publish_joint_states(self, positions):
        """Publish joint positions"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints
        msg.position = positions
        self.joint_pub.publish(msg)
```

### Visualizing URDF

```bash
# View URDF in RViz2
ros2 launch urdf_tutorial display.launch.py model:=path/to/robot.urdf

# Check URDF validity
check_urdf robot.urdf

# Convert URDF to PDF visualization
urdf_to_graphiz robot.urdf
```

---

## Practical Example: Humanoid Walker

### Complete ROS 2 Package Structure

```
humanoid_walker/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── humanoid.urdf
├── launch/
│   └── walker.launch.py
├── config/
│   └── params.yaml
└── humanoid_walker/
    ├── __init__.py
    ├── walker_controller.py
    └── gait_planner.py
```

### Walker Controller Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class HumanoidWalkerController(Node):
    def __init__(self):
        super().__init__('humanoid_walker_controller')

        # Parameters
        self.declare_parameter('step_length', 0.1)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('step_frequency', 1.0)

        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # State
        self.current_joints = {}
        self.current_orientation = None
        self.target_velocity = Twist()

        # Gait timer
        self.gait_timer = self.create_timer(0.01, self.gait_control_loop)
        self.phase = 0.0

    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        self.target_velocity = msg

    def imu_callback(self, msg):
        """Monitor robot orientation"""
        self.current_orientation = msg.orientation

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for name, position in zip(msg.name, msg.position):
            self.current_joints[name] = position

    def gait_control_loop(self):
        """Main gait generation loop"""
        if abs(self.target_velocity.linear.x) < 0.01:
            return  # Standing still

        # Increment phase
        freq = self.get_parameter('step_frequency').value
        self.phase += 0.01 * freq * 2 * np.pi
        if self.phase > 2 * np.pi:
            self.phase -= 2 * np.pi

        # Generate joint trajectories
        trajectory = self.generate_walking_trajectory(self.phase)

        # Publish
        self.joint_trajectory_pub.publish(trajectory)

    def generate_walking_trajectory(self, phase):
        """Generate bipedal walking trajectory"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()

        # Joint names for legs
        trajectory.joint_names = [
            'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_pitch', 'right_knee', 'right_ankle'
        ]

        # Compute joint angles using sinusoidal gait pattern
        point = JointTrajectoryPoint()

        step_length = self.get_parameter('step_length').value
        step_height = self.get_parameter('step_height').value

        # Left leg (support phase when phase < π)
        if phase < np.pi:
            left_hip = -0.3 * np.sin(phase)
            left_knee = 0.6 * np.sin(phase)
            left_ankle = -0.3 * np.sin(phase)
        else:
            # Swing phase
            left_hip = 0.5 * np.sin(phase)
            left_knee = 0.8 * np.sin(phase)
            left_ankle = 0.3 * np.sin(phase)

        # Right leg (opposite phase)
        right_phase = phase + np.pi
        if right_phase < np.pi:
            right_hip = -0.3 * np.sin(right_phase)
            right_knee = 0.6 * np.sin(right_phase)
            right_ankle = -0.3 * np.sin(right_phase)
        else:
            right_hip = 0.5 * np.sin(right_phase)
            right_knee = 0.8 * np.sin(right_phase)
            right_ankle = 0.3 * np.sin(right_phase)

        point.positions = [
            left_hip, left_knee, left_ankle,
            right_hip, right_knee, right_ankle
        ]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 10000000  # 10ms

        trajectory.points = [point]
        return trajectory
```

### Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),

        Node(
            package='humanoid_walker',
            executable='walker_controller',
            name='walker_controller',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'step_length': 0.1,
                'step_height': 0.05,
                'step_frequency': 1.0
            }],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': 'path/to/humanoid.urdf'
            }]
        ),
    ])
```

---

## Best Practices

### 1. Node Design Principles

✅ **Single Responsibility** - Each node does one thing well
✅ **Loose Coupling** - Nodes communicate via standard messages
✅ **Configurability** - Use parameters for tuning
✅ **Error Handling** - Gracefully handle failures
✅ **Logging** - Use ROS 2 logging for debugging

### 2. Performance Optimization

```python
# Use QoS profiles appropriately
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

self.subscription = self.create_subscription(
    Image,
    '/camera/image_raw',
    self.callback,
    sensor_qos  # Use appropriate QoS
)
```

### 3. Testing

```python
import unittest
from rclpy.node import Node

class TestWalkerController(unittest.TestCase):
    def test_gait_generation(self):
        # Test gait pattern generation
        pass

    def test_velocity_commands(self):
        # Test command processing
        pass
```

---

## Resources

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/rolling/)
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)

### Community
- [ROS Discourse](https://discourse.ros.org/)
- [ROS 2 GitHub](https://github.com/ros2)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)

### Learning Resources
- The Construct (ROS courses)
- Udemy ROS 2 courses
- Official ROS 2 tutorials

---

## Conclusion

ROS 2 provides the nervous system that enables humanoid robots to coordinate complex behaviors. By mastering nodes, topics, services, and URDF, you can build sophisticated robotic systems that bridge AI agents with physical hardware.

**Next Steps:**
- Set up ROS 2 development environment
- Create your first robot URDF
- Implement sensor integration
- Build multi-node robotic systems
- Move to simulation with Gazebo (Module 2)

---

<div style={{textAlign: 'center', padding: '2rem', background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)', borderRadius: '12px', color: 'white', marginTop: '2rem'}}>
  <h3 style={{color: 'white'}}>Ready for Simulation?</h3>
  <p>Learn how to test your ROS 2 robots in physics simulators</p>
  <a href="/module2-digital-twin" style={{color: 'white', textDecoration: 'underline'}}>Continue to Module 2 →</a>
</div>
