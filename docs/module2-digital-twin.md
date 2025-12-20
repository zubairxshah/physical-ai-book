---
sidebar_position: 16
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Introduction to Digital Twins

A **digital twin** is a virtual replica of a physical robot that mirrors its behavior, physics, and environment in real-time. Digital twins enable:

- 🧪 **Safe Testing** - Validate algorithms before deploying to hardware
- 🔄 **Parallel Development** - Software teams work while hardware is being built
- 📊 **Data Generation** - Create synthetic training data at scale
- 🎯 **Predictive Maintenance** - Monitor and predict robot health
- 🚀 **Rapid Iteration** - Test thousands of scenarios quickly

---

## Gazebo: The Physics-First Simulator

**Gazebo** is the industry-standard robot simulator, tightly integrated with ROS 2. It provides accurate physics simulation, sensor modeling, and plugin extensibility.

### Why Gazebo?

✅ **Accurate Physics** - ODE, Bullet, DART, or Simbody engines
✅ **ROS 2 Native** - Seamless integration with robot middleware
✅ **Sensor Simulation** - Cameras, LiDAR, IMU, depth sensors
✅ **Plugin System** - Extensible with custom behaviors
✅ **Open Source** - Free and community-supported
✅ **Production Ready** - Used by NASA, Boston Dynamics, and more

### Gazebo Architecture

```
┌─────────────────────────────────────┐
│        Gazebo Client (GUI)          │
│   ┌──────────┐    ┌──────────┐     │
│   │  3D View │    │ Plot Tools│     │
│   └──────────┘    └──────────┘     │
└─────────────┬───────────────────────┘
              │
┌─────────────▼───────────────────────┐
│       Gazebo Server (gzserver)      │
│                                     │
│  ┌──────────┐  ┌─────────────┐    │
│  │ Physics  │  │   Sensors   │    │
│  │ Engine   │  │  Rendering  │    │
│  └──────────┘  └─────────────┘    │
│                                     │
│  ┌──────────────────────────────┐  │
│  │       Plugin System          │  │
│  │  - Model plugins             │  │
│  │  - World plugins             │  │
│  │  - Sensor plugins            │  │
│  │  - System plugins            │  │
│  └──────────────────────────────┘  │
└─────────────┬───────────────────────┘
              │
┌─────────────▼───────────────────────┐
│         ROS 2 Integration           │
│  (gz_ros2_control, ros_gz_bridge)   │
└─────────────────────────────────────┘
```

---

## Setting Up Gazebo with ROS 2

### Installation

```bash
# Install Gazebo Harmonic (latest)
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo apt install ros-${ROS_DISTRO}-ros-gz

# Verify installation
gz sim --version
```

### Basic World File

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="humanoid_world">

    <!-- Physics Configuration -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

---

## Simulating Humanoid Robots in Gazebo

### Complete Robot SDF Model

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="simple_humanoid">

    <!-- Base Link -->
    <link name="torso">
      <pose>0 0 1.0 0 0 0</pose>
      <inertial>
        <mass>20.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <iyy>0.5</iyy>
          <izz>0.5</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
        </material>
      </visual>

      <!-- IMU Sensor -->
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </x>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </x>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>

    <!-- Camera Sensor -->
    <link name="camera_link">
      <pose relative_to="torso">0.15 0 0.3 0 0 0</pose>
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>

    <!-- Depth Camera (RealSense-like) -->
    <link name="depth_camera_link">
      <pose relative_to="torso">0.15 0 0.3 0 0 0</pose>
      <sensor name="depth_camera" type="depth_camera">
        <update_rate>20</update_rate>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>10.0</far>
          </clip>
        </camera>
      </sensor>
    </link>

    <!-- LiDAR Sensor -->
    <link name="lidar_link">
      <pose relative_to="torso">0 0 0.3 0 0 0</pose>
      <sensor name="lidar" type="gpu_lidar">
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>640</samples>
              <resolution>1</resolution>
              <min_angle>-1.57</min_angle>
              <max_angle>1.57</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.26</min_angle>
              <max_angle>0.26</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>

    <!-- Joint: Right Hip -->
    <joint name="right_hip_pitch" type="revolute">
      <parent>torso</parent>
      <child>right_thigh</child>
      <pose relative_to="torso">0 -0.1 -0.25 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100</effort>
          <velocity>5.0</velocity>
        </limit>
        <dynamics>
          <damping>1.0</damping>
          <friction>0.1</friction>
        </dynamics>
      </axis>
    </joint>

    <link name="right_thigh">
      <pose relative_to="right_hip_pitch">0 0 -0.2 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
      </visual>
    </link>

    <!-- Add more joints and links for complete humanoid... -->

    <!-- Gazebo ROS 2 Control Plugin -->
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find humanoid_control)/config/ros2_control.yaml</parameters>
    </plugin>

  </model>
</sdf>
```

---

## Physics Simulation Deep Dive

### Gravity and Collision

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties

class PhysicsConfigurator(Node):
    def __init__(self):
        super().__init__('physics_configurator')

        # Service clients
        self.get_physics_client = self.create_client(
            GetPhysicsProperties, '/get_physics_properties')
        self.set_physics_client = self.create_client(
            SetPhysicsProperties, '/set_physics_properties')

    def configure_physics(self):
        """Configure simulation physics"""
        request = SetPhysicsProperties.Request()

        # Gravity (Earth-like)
        request.gravity.z = -9.81

        # Time step
        request.time_step = 0.001  # 1ms
        request.max_update_rate = 1000.0

        # ODE physics parameters
        request.ode_config.auto_disable_bodies = False
        request.ode_config.sor_pgs_precon_iters = 0
        request.ode_config.sor_pgs_iters = 50
        request.ode_config.sor_pgs_w = 1.3
        request.ode_config.sor_pgs_rms_error_tol = 0.0
        request.ode_config.contact_surface_layer = 0.001
        request.ode_config.contact_max_correcting_vel = 100.0
        request.ode_config.cfm = 0.0
        request.ode_config.erp = 0.2
        request.ode_config.max_contacts = 20

        future = self.set_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
```

### Contact and Friction Models

```xml
<!-- Advanced surface properties -->
<collision name="foot_collision">
  <geometry>
    <box>
      <size>0.15 0.08 0.03</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>        <!-- Coefficient of friction -->
        <mu2>1.0</mu2>      <!-- Secondary friction -->
        <slip1>0.0</slip1>  <!-- Slip parameter -->
        <slip2>0.0</slip2>
      </ode>
      <bullet>
        <friction>1.0</friction>
        <friction2>1.0</friction2>
        <rolling_friction>0.1</rolling_friction>
      </bullet>
    </friction>
    <contact>
      <ode>
        <kp>1000000.0</kp>  <!-- Stiffness -->
        <kd>1.0</kd>         <!-- Damping -->
        <max_vel>0.01</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

---

## Unity: High-Fidelity Digital Twins

**Unity** provides photorealistic rendering, advanced physics, and real-time synchronization for human-robot interaction scenarios.

### Why Unity for Robotics?

✅ **Photorealistic Graphics** - High-quality rendering for vision systems
✅ **Human Characters** - Advanced animation and interaction
✅ **VR/AR Support** - Immersive robot testing
✅ **Asset Ecosystem** - Thousands of 3D models available
✅ **ML-Agents** - Built-in reinforcement learning framework
✅ **Real-time Sync** - Mirror physical robot state

### Unity ML-Agents for Humanoids

```csharp
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class HumanoidAgent : Agent
{
    [Header("Body Parts")]
    public Transform torso;
    public Transform head;
    public Transform rightThigh;
    public Transform leftThigh;
    public Transform rightShin;
    public Transform leftShin;

    [Header("Walking Parameters")]
    public float targetWalkingSpeed = 1.0f;
    public Transform target;

    private Rigidbody torsoRb;
    private Vector3 startingPosition;

    public override void Initialize()
    {
        torsoRb = torso.GetComponent<Rigidbody>();
        startingPosition = torso.position;
    }

    public override void OnEpisodeBegin()
    {
        // Reset position
        torso.position = startingPosition;
        torso.rotation = Quaternion.identity;

        // Reset velocities
        torsoRb.velocity = Vector3.zero;
        torsoRb.angularVelocity = Vector3.zero;

        // Randomize target position
        float randomX = Random.Range(-5f, 5f);
        float randomZ = Random.Range(-5f, 5f);
        target.position = new Vector3(randomX, 0, randomZ);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Torso state (10 observations)
        sensor.AddObservation(torso.localPosition);
        sensor.AddObservation(torso.localRotation);
        sensor.AddObservation(torsoRb.velocity);
        sensor.AddObservation(torsoRb.angularVelocity);

        // Joint positions (12 observations)
        sensor.AddObservation(rightThigh.localRotation);
        sensor.AddObservation(leftThigh.localRotation);
        sensor.AddObservation(rightShin.localRotation);
        sensor.AddObservation(leftShin.localRotation);

        // Target direction (3 observations)
        Vector3 dirToTarget = (target.position - torso.position).normalized;
        sensor.AddObservation(dirToTarget);

        // Total: 25 observations
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Continuous actions for joint control
        var continuousActions = actions.ContinuousActions;

        // Apply torques to joints
        ApplyJointTorque(rightThigh, continuousActions[0], continuousActions[1]);
        ApplyJointTorque(leftThigh, continuousActions[2], continuousActions[3]);
        ApplyJointTorque(rightShin, continuousActions[4], continuousActions[5]);
        ApplyJointTorque(leftShin, continuousActions[6], continuousActions[7]);

        // Calculate rewards
        float distanceToTarget = Vector3.Distance(torso.position, target.position);
        float forwardBonus = Vector3.Dot(torso.forward, dirToTarget) * 0.1f;

        // Reward for moving toward target
        AddReward(-0.001f * distanceToTarget);

        // Reward for staying upright
        float uprightBonus = torso.up.y;
        AddReward(uprightBonus * 0.01f);

        // Penalty for falling
        if (torso.position.y < 0.3f)
        {
            AddReward(-1.0f);
            EndEpisode();
        }

        // Success reward
        if (distanceToTarget < 1.0f)
        {
            AddReward(1.0f);
            EndEpisode();
        }
    }

    private void ApplyJointTorque(Transform joint, float torqueX, float torqueZ)
    {
        Rigidbody rb = joint.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.AddRelativeTorque(torqueX * 100f, 0, torqueZ * 100f);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }
}
```

### Unity ROS 2 Integration

```csharp
using UnityEngine;
using ROS2;

public class UnityROS2Bridge : MonoBehaviour
{
    private INode node;
    private IPublisher<geometry_msgs.msg.Twist> cmdVelPublisher;
    private ISubscription<sensor_msgs.msg.Image> imageSubscription;

    void Start()
    {
        // Initialize ROS 2
        ROS2Unity.Init();
        node = ROS2Unity.CreateNode("unity_robot_node");

        // Create publisher for robot commands
        cmdVelPublisher = node.CreatePublisher<geometry_msgs.msg.Twist>("/cmd_vel");

        // Subscribe to camera images
        imageSubscription = node.CreateSubscription<sensor_msgs.msg.Image>(
            "/camera/image_raw",
            msg => OnImageReceived(msg));
    }

    void Update()
    {
        // Send velocity commands
        if (Input.GetKey(KeyCode.W))
        {
            var twist = new geometry_msgs.msg.Twist();
            twist.Linear.X = 1.0;
            cmdVelPublisher.Publish(twist);
        }
    }

    void OnImageReceived(sensor_msgs.msg.Image msg)
    {
        // Process received image
        Debug.Log($"Received image: {msg.Width}x{msg.Height}");
    }

    void OnDestroy()
    {
        node?.Dispose();
        ROS2Unity.Shutdown();
    }
}
```

---

## Digital Twin Synchronization

### Real-Time State Mirroring

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import requests

class DigitalTwinSync(Node):
    def __init__(self):
        super().__init__('digital_twin_sync')

        # Subscribe to physical robot state
        self.joint_state_sub = self.create_subscription(
            JointState, '/robot/joint_states', self.sync_joints, 10)

        # Unity HTTP endpoint
        self.unity_url = "http://localhost:5000/api/robot_state"

    def sync_joints(self, msg):
        """Send joint states to Unity digital twin"""
        payload = {
            'timestamp': self.get_clock().now().to_msg(),
            'joint_names': msg.name,
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort)
        }

        try:
            response = requests.post(self.unity_url, json=payload, timeout=0.01)
            if response.status_code != 200:
                self.get_logger().warn('Failed to sync with digital twin')
        except requests.exceptions.Timeout:
            pass  # Non-blocking
```

---

## Sensor Simulation Best Practices

### Camera Calibration in Simulation

```python
import numpy as np
from sensor_msgs.msg import CameraInfo

class SimulatedCameraCalibration(Node):
    def __init__(self):
        super().__init__('camera_calibration')

        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10)

        # Camera intrinsic parameters
        self.width = 640
        self.height = 480
        self.fx = 554.25  # Focal length x
        self.fy = 554.25  # Focal length y
        self.cx = 320.5   # Principal point x
        self.cy = 240.5   # Principal point y

        self.publish_camera_info()

    def publish_camera_info(self):
        """Publish camera calibration"""
        msg = CameraInfo()
        msg.header.frame_id = 'camera_link'
        msg.width = self.width
        msg.height = self.height

        # Camera matrix [fx 0 cx; 0 fy cy; 0 0 1]
        msg.k = [self.fx, 0.0, self.cx,
                 0.0, self.fy, self.cy,
                 0.0, 0.0, 1.0]

        # Distortion coefficients (pinhole model)
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix (identity for monocular)
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        # Projection matrix
        msg.p = [self.fx, 0.0, self.cx, 0.0,
                 0.0, self.fy, self.cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        self.camera_info_pub.publish(msg)
```

### Adding Realistic Sensor Noise

```python
import numpy as np

class NoisyLidarPublisher(Node):
    def __init__(self):
        super().__init__('noisy_lidar')

        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/raw', self.add_noise, 10)
        self.lidar_pub = self.create_publisher(
            LaserScan, '/lidar/noisy', 10)

    def add_noise(self, msg):
        """Add realistic noise to LiDAR measurements"""
        ranges = np.array(msg.ranges)

        # Gaussian noise proportional to distance
        noise_stddev = 0.01 + 0.001 * ranges
        noise = np.random.normal(0, noise_stddev)
        noisy_ranges = ranges + noise

        # Occasional outliers (5% chance)
        outlier_mask = np.random.random(len(ranges)) < 0.05
        noisy_ranges[outlier_mask] = msg.range_max

        # Clip to valid range
        noisy_ranges = np.clip(noisy_ranges, msg.range_min, msg.range_max)

        msg.ranges = noisy_ranges.tolist()
        self.lidar_pub.publish(msg)
```

---

## Sim-to-Real Transfer

### Domain Randomization

```python
import random

class DomainRandomizer(Node):
    def __init__(self):
        super().__init__('domain_randomizer')

        # Gazebo service clients
        self.set_physics = self.create_client(
            SetPhysicsProperties, '/set_physics_properties')

    def randomize_physics(self):
        """Randomize physics parameters"""
        request = SetPhysicsProperties.Request()

        # Randomize gravity slightly
        request.gravity.z = random.uniform(-10.0, -9.6)

        # Randomize time step
        request.time_step = random.uniform(0.0005, 0.002)

        # Randomize friction
        friction_multiplier = random.uniform(0.8, 1.2)
        # Apply to all surfaces...

        self.set_physics.call_async(request)

    def randomize_lighting(self):
        """Change lighting conditions"""
        # Implement light randomization
        pass

    def randomize_textures(self):
        """Apply random textures to objects"""
        # Implement texture randomization
        pass
```

---

## Conclusion

Digital twins in Gazebo and Unity provide essential tools for developing and validating humanoid robots. By mastering physics simulation, sensor modeling, and real-time synchronization, you can:

- 🚀 Accelerate development cycles
- 🧪 Test safely before deploying to hardware
- 📊 Generate massive training datasets
- 🔄 Maintain synchronized physical-digital systems

**Next Steps:**
- Set up Gazebo Harmonic with ROS 2
- Create a humanoid robot SDF model
- Integrate sensor simulations
- Explore Unity ML-Agents for RL training
- Move to NVIDIA Isaac for GPU acceleration (Module 3)

---

<div style={{textAlign: 'center', padding: '2rem', background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)', borderRadius: '12px', color: 'white', marginTop: '2rem'}}>
  <h3 style={{color: 'white'}}>Ready for GPU-Accelerated Simulation?</h3>
  <p>Learn how NVIDIA Isaac supercharges robot development</p>
  <a href="/module3-nvidia-isaac" style={{color: 'white', textDecoration: 'underline'}}>Continue to Module 3 →</a>
</div>
