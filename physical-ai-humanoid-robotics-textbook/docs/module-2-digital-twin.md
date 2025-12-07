# Module 2: Digital Twin Simulation for Gazebo & Unity

## Table of Contents
1. [Introduction to Digital Twins](#introduction-to-digital-twins)
2. [Setting Up Gazebo Classic/Gazebo Sim](#setting-up-gazebo)
3. [Physics Simulation Fundamentals](#physics-simulation-fundamentals)
4. [Loading Humanoid URDF in Gazebo](#loading-humanoid-urdf-in-gazebo)
5. [Sensor Simulation - Part 1: LiDAR](#sensor-simulation-part-1-lidar)
6. [Sensor Simulation - Part 2: Cameras](#sensor-simulation-part-2-cameras)
7. [Sensor Simulation - Part 3: IMU](#sensor-simulation-part-3-imu)
8. [Building Interactive Environments](#building-interactive-environments)
9. [Unity Robotics Hub Integration](#unity-robotics-hub-integration)
10. [Unity Scene Creation](#unity-scene-creation)
11. [Bridging Unity ↔ ROS 2](#bridging-unity-ros-2)
12. [Practical Lab: Complete Simulation Pipeline](#practical-lab-complete-simulation-pipeline)
13. [Physics Debugging and Tuning](#physics-debugging-and-tuning)
14. [Performance Optimization](#performance-optimization)
15. [Synthetic Data Generation (Preview)](#synthetic-data-generation-preview)
16. [Common Issues and Solutions](#common-issues-and-solutions)
17. [Summary and Next Steps](#summary-and-next-steps)

## Introduction to Digital Twins

A digital twin is a virtual representation of a physical system that enables simulation, analysis, and optimization. In robotics, digital twins allow us to safely test robot behaviors, validate algorithms, and train AI models without the risks associated with physical testing.

This module teaches students to create realistic physics simulations and immersive environments for humanoid robots using Gazebo and Unity. You'll learn to simulate sensors, test robot behavior in controlled environments, and validate designs before real-world deployment.

### Why Simulation Matters in Robotics

Simulation is crucial in robotics for several reasons:

1. **Safety**: Test dangerous maneuvers without physical risk
2. **Cost**: Avoid expensive hardware damage during development
3. **Iteration Speed**: Rapidly test multiple configurations
4. **Data Generation**: Create large datasets for training AI models
5. **Scenario Testing**: Replicate rare or hard-to-reproduce situations

### Gazebo vs Unity: When to Use Which

Gazebo and Unity serve different purposes in the digital twin ecosystem:

- **Gazebo**: Physics-focused simulation with accurate collision detection and dynamics
- **Unity**: Visual-focused simulation with photorealistic rendering capabilities

We'll use both tools in this module to create comprehensive simulation environments.

### Overview of What Students Will Build

By the end of this module, you will have created:
- A Gazebo world with physics properties and obstacles
- A humanoid robot model with LiDAR, depth camera, and IMU sensors
- A Unity environment with photorealistic rendering
- A bridge connecting both environments for synchronized simulation

## Setting Up Gazebo Classic/Gazebo Sim

### Installation (Ubuntu 22.04 + Gazebo 11/Harmonic)

First, ensure ROS 2 Humble Hawksbill is installed. Then install Gazebo:

```bash
sudo apt update && sudo apt install wget
sudo wget -qO - https://packages.osrfoundation.org/gazebo.gpg | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
sudo apt update
sudo apt install gz-harmonic
```

For ROS 2 integration:
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-ros-gz
```

### Understanding Worlds, Models, and Plugins

Gazebo simulations consist of three main components:

1. **Worlds**: Contain the environment, physics properties, and lighting
2. **Models**: Represent physical objects including robots, obstacles, and sensors
3. **Plugins**: Add custom functionality to the simulation

### Creating Your First Empty World

Create a new world file at `module-2/gazebo/worlds/empty.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="empty_world">
    <!-- Include the sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics Engine Configuration -->
    <physics name="ode_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

### Basic World File Structure (SDF Format)

SDF (Simulation Description Format) is an XML-based format that describes simulation environments. Key elements include:

- `<world>`: Contains the entire simulation environment
- `<include>`: References predefined models or assets
- `<physics>`: Configures the physics engine parameters
- `<model>`: Defines custom objects in the simulation
- `<light>`: Specifies light sources in the environment

## Physics Simulation Fundamentals

### Physics Engines: ODE vs Bullet Comparison

Gazebo supports multiple physics engines:

- **ODE (Open Dynamics Engine)**: Default engine, excellent for rigid body simulation
- **Bullet**: Advanced collision detection, better for complex shapes
- **Simbody**: Multibody dynamics specialized engine

For humanoid robots, ODE is typically the best choice due to its stability and performance.

### Gravity, Friction, and Restitution

These parameters control how objects interact:

- **Gravity**: Defines the acceleration due to gravity (default: 0 0 -9.8)
- **Friction**: Resistance to sliding between surfaces (mu values typically 0.5-1.2)
- **Restitution**: "Bounciness" of collisions (0 = no bounce, 1 = perfectly elastic)

### Mass and Inertia Properties

For stable simulation, correct mass and inertia properties are essential:

```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

### Collision Detection and Meshes

Collision detection can use simplified geometries (boxes, spheres, cylinders) or complex meshes. Simplified geometries are faster but less accurate.

### Time Step and Real-Time Factor

- **Time Step**: Duration of each physics simulation step (smaller = more accurate but slower)
- **Real-Time Factor**: Simulation speed relative to real time (1.0 = real-time, >1.0 = faster than real-time)

## Loading Humanoid URDF in Gazebo






















### Gazebo-Specific URDF Tags (`gazebo` blocks)

To make URDF models work in Gazebo, add special `<gazebo>` tags:

```xml
<link name="humanoid_link">
  <!-- Standard URDF elements -->
  <visual>...</visual>
  <collision>...</collision>
  <inertial>...</inertial>
  
  <!-- Gazebo-specific elements -->
  <gazebo reference="humanoid_link">
    <material>Gazebo/Blue</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>
</link>
```

### Adding Visual Materials and Textures

Visual properties in Gazebo are defined in the `<visual>` section of links:

```xml
<visual name="my_visual">
  <geometry>
    <mesh filename="package://my_robot/meshes/part.dae"/>
  </geometry>
  <material name="my_material">
    <script>
      <uri>file://media/materials/scripts/gazebo.material</uri>
      <name>Gazebo/Blue</name>
    </script>
  </material>
</visual>
```

### Joint Controllers and Transmission

To control joints in simulation, define transmissions:

```xml
<transmission name="transmission_joint1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Spawning Robot in World

To spawn a robot in a world, include it in the world file:

```xml
<include>
  <uri>model://humanoid_robot</uri>
  <pose>0 0 1.0 0 0 0</pose>
</include>
```

Or spawn via command line:
```bash
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.urdf -x 0 -y 0 -z 1
```

### Testing Joint Movements from Module 1

After loading the robot, verify joint movements work by commanding joint positions through ROS 2.

## Sensor Simulation - Part 1: LiDAR

### Ray-Based Sensor Principles

LiDAR sensors work by emitting laser rays and measuring the time it takes for reflections to return. In simulation, we model this with multiple ray sensors.

### Configuring LiDAR Plugin (Range, Resolution, FOV)

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>lidar</namespace>
        <argument>~/out:=scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Mounting LiDAR on Humanoid Head/Torso

Position the LiDAR sensor on your robot where a real LiDAR would be mounted, typically on the head or torso for 360-degree coverage.

### Visualizing Point Clouds in RViz

After publishing LiDAR data, visualize it in RViz by adding a LaserScan display for the `/lidar/scan` topic.

### Publishing to ROS 2 Topics

The plugin publishes LaserScan messages to the ROS 2 topic specified in the configuration.

## Sensor Simulation - Part 2: Cameras

### RGB Camera Plugin

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>camera</namespace>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera (RGB-D Simulation)

For RGB-D simulation, use the openni_kinect plugin:

```xml
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>camera</namespace>
      </ros>
      <frame_name>depth_camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Camera Intrinsic Parameters

Camera intrinsics define the relationship between 3D world coordinates and 2D image coordinates. These are typically:
- Focal length (fx, fy)
- Principal point (cx, cy)
- Distortion coefficients

### Image Publishing to ROS 2

Camera plugins publish to topics like `/camera/image_raw` and `/camera/camera_info` following ROS 2 image transport conventions.

### Viewing Camera Feeds

Use image_view to visualize camera feeds:
```bash
ros2 run image_view image_view_raw --ros-args --remap image:=/camera/image_raw
```

## Sensor Simulation - Part 3: IMU

### Inertial Measurement Unit Basics

An IMU combines multiple sensors to measure orientation, angular velocity, and linear acceleration. It typically includes:
- Accelerometer (linear acceleration)
- Gyroscope (angular velocity)
- Magnetometer (orientation relative to magnetic north)

### Noise Models (Gaussian Noise)

Real IMUs have noise that must be simulated for realistic data:

```xml
<gazebo reference="imu_link">
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
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>imu</namespace>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Orientation, Angular Velocity, Linear Acceleration

The IMU outputs three types of data:
- `orientation`: Quaternion representing rotation
- `angular_velocity`: Vector of rotation rates around x, y, z axes
- `linear_acceleration`: Vector of acceleration along x, y, z axes

### IMU for Balance Control

IMU data is crucial for humanoid balance control, providing feedback about the robot's orientation and movement.

### Reading IMU Data in ROS 2 Nodes

Subscribe to IMU data using the sensor_msgs/Imu message type:

```python
import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node

class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
    def imu_callback(self, msg):
        # Process IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        # ... further processing

def main(args=None):
    rclpy.init(args=args)
    imu_reader = ImuReader()
    rclpy.spin(imu_reader)
    imu_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building Interactive Environments

### Adding Obstacles (Boxes, Cylinders, Custom Meshes)

Add static obstacles to your world file:

```xml
<model name="obstacle_box">
  <pose>2 0 0.5 0 0 0</pose>
  <static>true</static>
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
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Red</name>
        </script>
      </material>
    </visual>
    <inertial>
      <mass>1.0</mass>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</model>
```

### Terrain Generation (Heightmaps)

For complex terrain, use heightmaps:

```xml
<model name="terrain">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>file://media/materials/textures/terrain_heightmap.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>file://media/materials/textures/terrain_texture.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

### Dynamic Objects and Actors

Actors can follow predefined paths or respond to simulation events:

```xml
<actor name="walking_person">
  <pose>0 2 0 0 0 0</pose>
  <skin>
    <filename>walking.dae</filename>
    <scale>1.0</scale>
  </skin>
  <animation name="walking">
    <filename>walking.dae</filename>
    <scale>1.0</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <trajectory id="0" type="walking">
      <waypoint>
        <time>0</time>
        <pose>0 2 0 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>5</time>
        <pose>5 2 0 0 0 0</pose>
      </waypoint>
      <waypoint>
        <time>10</time>
        <pose>0 2 0 0 0 0</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

### Lighting and Shadows

Configure lighting for realistic environment rendering:

```xml
<light name="main_light" type="directional">
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
  </attenuation>
  <direction>-0.5 -0.1 -0.9</direction>
</light>
```

### Scene Complexity vs Performance

Balance visual fidelity with simulation performance by:
- Using simplified collision meshes
- Reducing polygon count for distant objects
- Limiting the number of dynamic objects
- Optimizing physics parameters

## Unity Robotics Hub Integration

### Why Unity? Photorealism and HRI

Unity excels at creating photorealistic environments for:
- Human-Robot Interaction (HRI) simulation
- Training computer vision models
- Creating visually appealing user interfaces
- Testing perception algorithms under various lighting conditions

### Installing Unity Editor + Robotics Packages

1. Download and install Unity Hub
2. Install Unity 2022.3 LTS
3. Create a new 3D project
4. In Package Manager, install:
   - ROS TCP Connector
   - URDF Importer

### Setting Up ROS-TCP Connection

The ROS TCP Connector enables communication between ROS 2 and Unity:

1. Add a ROSConnection GameObject to your scene
2. Configure the IP and port settings
3. Register publishers and subscribers in your Unity scripts

### Importing URDF to Unity (URDF-Importer)

The URDF Importer converts ROS URDF files to Unity objects:
1. Go to GameObject > Import URDF
2. Select your URDF file
3. Configure import settings
4. The robot will be created in the scene with appropriate joints and physics

### Articulation Body for Physics

Unity uses Articulation Body components to simulate joint physics, similar to Gazebo's joint constraints.

## Unity Scene Creation

### Creating Realistic Indoor Environment

Create a warehouse-style environment with:
- Floor planes with appropriate materials
- Wall structures using primitive objects or custom meshes
- Obstacle placement to test robot navigation

### Adding Materials and Textures (PBR)

Use physically based rendering (PBR) materials for realistic lighting:
- Albedo map (base color)
- Normal map (surface detail)
- Metallic map (reflectiveness)
- Smoothness map (specular reflection)

### Lighting Setup (Baked vs Real-Time)

Choose between:
- **Baked lighting**: Precomputed for static objects, better performance
- **Real-time lighting**: Dynamic, but more computationally expensive

### Adding Interactive Objects

Create objects that respond to physics:
- Colliders for collision detection
- Rigidbodies for physics simulation
- Scripts for custom behavior

### Human Avatars for HRI Testing

Use Unity's animation system to create human avatars that interact with your robot.

## Bridging Unity ↔ ROS 2

### ROS-TCP Endpoint Setup

Configure the TCP endpoint for communication:
- Set IP address and port
- Establish connection between Unity and ROS
- Handle connection status and errors

### Publishing Unity Sensor Data to ROS

Unity can publish sensor-like data to ROS topics:

```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

// In your sensor script
public class UnityCameraPublisher : MonoBehaviour
{
    public string topicName = "/unity_camera/image_raw";
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    void PublishCameraData(Texture2D image)
    {
        // Convert Unity texture to ROS Image message
        var imageMsg = ImageMsg.CreateImage(image.width, image.height, "rgb8", image.EncodeToPNG());
        ros.Publish(topicName, imageMsg);
    }
}
```

### Receiving ROS Commands in Unity

Subscribe to ROS topics in Unity to control your simulation:

```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

// In your controller script
void Start()
{
    ros = ROSConnection.GetOrCreateInstance();
    ros.Subscribe<Float32Msg>(topicName, MoveRobot);
}

void MoveRobot(Float32Msg msg)
{
    // Process ROS command to move robot in Unity
    transform.position += Vector3.forward * msg.data;
}
```

### Synchronization and Latency Considerations

Ensure both environments stay synchronized:
- Timestamp messages appropriately
- Handle latency in bidirectional communication
- Consider interpolation for smooth motion

### Testing Bidirectional Communication

Create a test to verify data flows in both directions:
- Send commands from ROS to Unity
- Send sensor data from Unity to ROS
- Verify both directions work correctly

## Practical Lab: Complete Simulation Pipeline

### Create Gazebo Warehouse World with Obstacles

Following the structure in `digital_twin_world.world`, create a warehouse environment with:
- Walls forming an enclosed space
- Multiple obstacles of various shapes
- Appropriate lighting and physics parameters

### Load Humanoid with LiDAR, RGB-D, IMU

Use the URDF model created earlier with all sensors properly configured.

### Implement ROS 2 Node to Read All Sensors

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import cv2
from cv_bridge import CvBridge
import numpy as np

class MultiSensorReader(Node):
    def __init__(self):
        super().__init__('multi_sensor_reader')
        self.bridge = CvBridge()
        
        # Subscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10)
        
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.camera_callback,
            10)
            
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
    
    def lidar_callback(self, msg):
        self.get_logger().info(f'Lidar range count: {len(msg.ranges)}')
        
    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info(f'Camera image: {cv_image.shape}')
        
    def imu_callback(self, msg):
        orientation = msg.orientation
        self.get_logger().info(f'IMU orientation: ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})')

def main(args=None):
    rclpy.init(args=args)
    sensor_reader = MultiSensorReader()
    rclpy.spin(sensor_reader)
    sensor_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Move Robot Using Keyboard Teleoperation

Create a simple teleoperation node:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
"""

moveBindings = {
        'i': (1, 0),
        'o': (1, -1),
        'j': (0, 1),
        'l': (0, -1),
        'u': (1, 1),
        ',': (-1, 0),
        '.': (-1, 1),
        'm': (-1, -1),
    }

speedBindings = {
        'q': (1.1, 1.1),
        'z': (.9, .9),
        'w': (1.1, 1),
        'x': (.9, 1),
        'e': (1, 1.1),
        'c': (1, .9),
    }

class TeleopRobot(Node):
    def __init__(self):
        super().__init__('teleop_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = 0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    teleop_node = TeleopRobot()
    
    try:
        print(msg)
        while True:
            key = teleop_node.getKey()
            if key in moveBindings.keys():
                teleop_node.target_linear_velocity = moveBindings[key][0]
                teleop_node.target_angular_velocity = moveBindings[key][1]
            elif key in speedBindings.keys():
                # Handle speed changes
                pass
            else:
                # Stop the robot
                pass
                
            # Publish twist command
            twist = Twist()
            twist.linear.x = teleop_node.target_linear_velocity
            twist.angular.z = teleop_node.target_angular_velocity
            teleop_node.publisher.publish(twist)
            
    except Exception as e:
        print(e)
    finally:
        # Stop the robot
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
```

### Record Sensor Data to Rosbag

```bash
# Record all sensor data
ros2 bag record /lidar/scan /camera/rgb/image_raw /imu/data /robot_description /joint_states
```

### Alternative: Same Setup in Unity

Implement the same robot configuration in Unity with ROS bridge, following the earlier Unity integration steps.

### Compare Performance and Fidelity

Analyze the differences:
- Visual quality
- Physics accuracy
- Sensor data fidelity
- Performance characteristics
- Use cases for each platform

## Physics Debugging and Tuning

### Robot Falls Over: Inertia and Mass Issues

Common causes:
- Center of mass outside support polygon
- Incorrect mass distribution
- Improper inertia tensors

Solutions:
- Verify mass and inertia for each link
- Ensure center of mass is within the robot's base
- Check that all values are physically realistic

### Jittering Joints: Damping and Friction

Symptoms:
- Rapid oscillations at joints
- Unstable movements
- Unnatural-looking motion

Solutions:
- Add damping to joints
- Adjust friction coefficients
- Reduce physics time step
- Increase solver iterations

### Slow Simulation: Mesh Complexity

If simulation runs slowly:
- Simplify collision meshes
- Reduce number of complex objects
- Adjust real-time factor
- Optimize physics parameters

### Sensor Noise Calibration

Calibrate sensor noise to match real-world performance:
- Compare simulated vs real sensor data
- Adjust noise parameters in sensor configurations
- Validate that noise levels are realistic

### Collision Matrix Optimization

For complex models, optimize collision checking:
- Simplify collision geometry
- Adjust collision checking frequency
- Use appropriate collision flags

### Real-Time Factor Tuning

Adjust physics parameters to achieve desired simulation speed:
- Smaller time steps: more accurate but slower
- Larger time steps: faster but potentially unstable
- Balance accuracy and performance needs

## Performance Optimization

### Reducing Mesh Polygon Count

- Use lower-poly versions for collision meshes
- Implement Level of Detail (LOD) systems
- Simplify distant objects

### Level of Detail (LOD) Strategies

Create multiple versions of complex objects:
- High-detail for close-up views
- Low-detail for distant views
- Automatic switching based on distance

### Culling and Occlusion

- Implement frustum culling to avoid rendering objects outside view
- Use occlusion culling for objects blocked by others
- Reduce rendering of non-visible objects

### Physics Update Rates

- Adjust physics update rate based on requirements
- Balance between accuracy and performance
- Consider different rates for different parts of simulation

### Headless Simulation for CI/CD

For automated testing, run Gazebo without GUI:
```bash
gz sim -s -r my_world.world
```

### Parallel Simulation Instances

Run multiple simulation instances for:
- Training data generation
- A/B testing of algorithms
- Performance analysis

## Synthetic Data Generation (Preview)

### Importance for ML Training

Synthetic data from simulation helps train machine learning models when real data is scarce, expensive, or dangerous to collect.

### Randomization Techniques

Domain randomization involves varying:
- Lighting conditions
- Material textures
- Object positions
- Background elements

### Domain Randomization Basics

Create variations to make models more robust:
- Change colors and textures
- Vary environmental conditions
- Add noise and distortions
- Randomize object properties

### Preview of Isaac Sim Capabilities (Module 3)

Isaac Sim builds on these concepts with advanced features for large-scale synthetic data generation.

## Common Issues and Solutions

### Model Spawning Errors

Problem: Robot doesn't appear in simulation
Solution: Check URDF syntax, ensure all mesh files are accessible, verify joint limits

### Sensor Data Not Publishing

Problem: No data on sensor topics
Solution: Verify plugin configurations, check topic names, ensure ROS bridge is active

### Physics Explosions and Instabilities

Problem: Robot parts flying apart or vibrating violently
Solution: Check mass/inertia values, adjust physics parameters, verify joint limits

### Performance Bottlenecks

Problem: Simulation running slowly
Solution: Simplify meshes, reduce physics complexity, optimize rendering

### Unity-ROS Connection Failures

Problem: No communication between Unity and ROS
Solution: Check IP addresses and ports, verify firewall settings, ensure both systems are running

### Version Compatibility Issues

Problem: Different ROS/Gazebo/Unity versions causing incompatibilities
Solution: Use recommended version combinations, keep systems updated consistently

## Summary and Next Steps

### What You've Learned: Simulation Mastery

In this module, you've gained expertise in:
- Creating realistic physics simulations in Gazebo
- Integrating Unity for photorealistic visualization
- Configuring multiple sensors for robot perception
- Establishing bidirectional communication between environments
- Debugging and optimizing simulation performance

### How This Enables Safe Robot Development

These simulation skills allow you to:
- Test robot behaviors safely before physical deployment
- Train AI models on large synthetic datasets
- Validate algorithms under various conditions
- Reduce development costs and risks

### Preview of Module 3: NVIDIA Isaac for Advanced Perception

Module 3 will build on these foundations with NVIDIA Isaac Sim, offering:
- GPU-accelerated physics simulation
- Advanced rendering capabilities
- Large-scale synthetic dataset generation
- Integration with NVIDIA's AI tools

### Recommended Exercises for Practice

1. Create your own custom robot model with sensors
2. Build a complete navigation challenge in simulation
3. Experiment with different physics parameters to see their effects
4. Develop a simple perception pipeline using simulated sensors
5. Create a Unity scene with multiple robots interacting

With these simulation skills, you're well-prepared to tackle complex robotics challenges safely and effectively.