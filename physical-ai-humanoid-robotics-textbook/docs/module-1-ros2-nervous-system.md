# Module 1: The Robotic Nervous System (ROS 2)

## Introduction to ROS 2 Architecture

Welcome to Module 1 of our Physical AI & Humanoid Robotics textbook! In this module, you'll learn the fundamentals of ROS 2 (Robot Operating System 2), the middleware that serves as the "nervous system" for most modern robotics applications. By the end of this module, you'll understand how to create robots that can communicate, coordinate, and control their actions using ROS 2's distributed architecture.

### What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's not an operating system in the traditional sense, but rather a middleware framework that provides services designed for a heterogeneous computer cluster. This includes:

- Hardware abstraction
- Device drivers
- Libraries
- Visualization tools
- Message-passing capabilities
- Package management

ROS 2 is the evolution of the ROS framework, designed to be more robust, secure, and production-ready for commercial applications.

### Why Robotics Needs ROS 2

Robotics applications often involve multiple systems working together: sensors that perceive the environment, actuators that move the robot, algorithms that process information, and interfaces that allow humans to interact. ROS 2 provides a standardized way for these systems to communicate and share data, regardless of their physical location or the programming languages they use.

### Core Concepts: Nodes, Topics, Services, and Actions

ROS 2's architecture is built around several core concepts:

1. **Nodes**: Processes performing computation. Nodes are the online processes that together make up a ROS 2 system.
2. **Topics**: Named buses over which nodes exchange messages. Topics enable asynchronous, many-to-many communication.
3. **Services**: Synchronous, request/response communication patterns between nodes.
4. **Actions**: Communication patterns for long-running tasks with feedback and status updates.

These concepts work together to create a distributed system where different components can be developed, tested, and deployed independently while maintaining clear communication channels.

## Setting Up ROS 2 Environment

Before diving into ROS 2 development, we need to set up your environment. This module uses ROS 2 Humble Hawksbill distribution with Ubuntu 22.04 LTS.

### Installing ROS 2 Humble

Follow these steps to install ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS:

1. **Set up the ROS 2 apt repository**:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Install ROS 2 Humble**:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   ```

3. **Install colcon build tool**:
   ```bash
   sudo apt install -y python3-colcon-common-extensions
   ```

4. **Install ROS 2 dependencies**:
   ```bash
   sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

5. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

6. **Add sourcing to your bashrc to make it permanent**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Creating a Workspace

ROS 2 uses workspaces to organize your projects. Create a new workspace for our work:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

This creates a workspace directory with the necessary structure. The `src` directory is where you'll place your source code packages.

### Package Structure

In ROS 2, functionality is organized into packages. A package typically contains:

- Source code (in various languages)
- Launch files to start multiple nodes
- Configuration files
- Test files
- Documentation
- Metadata files (package.xml, setup files)

## Your First ROS 2 Node

Let's create your first ROS 2 package and nodes. We'll build a simple publisher and subscriber example to understand basic communication.

### Creating a Python Package with rclpy

1. **Navigate to your workspace's src directory**:
   ```bash
   cd ~/ros2_ws/src
   ```

2. **Create a new package**:
   ```bash
   ros2 pkg create --build-type ament_python py_pubsub
   ```

This creates a new Python package named `py_pubsub`. The `--build-type ament_python` flag tells ROS 2 that this is a Python package using the ament build system.

### Publisher Node (Sending Velocity Commands)

Let's create a publisher node that sends velocity commands:

```python
# File: ~/ros2_ws/src/py_pubsub/py_pubsub/publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(String, 'velocity_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Velocity Command: {self.i} m/s'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    rclpy.spin(velocity_publisher)

    # Destroy the node explicitly
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Node (Receiving Sensor Feedback)

Now let's create a subscriber node that receives sensor feedback:

```python
# File: ~/ros2_ws/src/py_pubsub/py_pubsub/subscriber_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_feedback',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Sensor reading received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    sensor_subscriber = SensorSubscriber()

    rclpy.spin(sensor_subscriber)

    # Destroy the node explicitly
    sensor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Building and Testing the Nodes

1. **Make the Python files executable**:
   ```bash
   cd ~/ros2_ws/src/py_pubsub
   chmod +x py_pubsub/publisher_member_function.py
   chmod +x py_pubsub/subscriber_member_function.py
   ```

2. **Update the setup.py file** to include entry points for your nodes:
   ```python
   # File: ~/ros2_ws/src/py_pubsub/setup.py
   import os
   from glob import glob
   from setuptools import setup

   from setuptools import find_packages

   package_name = 'py_pubsub'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='Examples of minimal publisher/subscriber using rclpy',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'velocity_publisher = py_pubsub.publisher_member_function:main',
               'sensor_subscriber = py_pubsub.subscriber_member_function:main',
           ],
       },
   )
   ```

3. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select py_pubsub
   ```

4. **Source your workspace**:
   ```bash
   source install/setup.bash
   ```

5. **Run the publisher**:
   ```bash
   ros2 run py_pubsub velocity_publisher
   ```

6. **In another terminal, run the subscriber**:
   ```bash
   source ~/ros2_ws/install/setup.bash  # Remember to source the workspace
   ros2 run py_pubsub sensor_subscriber
   ```

You should see the publisher sending messages and the subscriber receiving them, demonstrating successful communication between nodes via ROS 2's topic system.

## Understanding URDF

URDF (Universal Robot Description Format) is XML-based format that describes robot models in terms of links, joints, and the connections between them.

### URDF Syntax and Structure

A typical URDF file contains:

- **Links**: Rigid components of the robot (like a chassis, arms, or wheels)
- **Joints**: Connections between links that allow relative motion
- **Visual and Collision Elements**: Describing the appearance and collision properties
- **Materials**: Color and texture information for visualization

### Links and Joints

Links are the rigid components of a robot. Each link must have a unique name and may contain visual and collision elements.

Joints connect links together and define how they can move relative to each other. Common joint types include:
- `revolute`: Rotational joint with limited range
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint
- `fixed`: No movement allowed

### Creating a Simple Humanoid URDF

Here's a basic example of a humanoid arm in URDF:

```xml
<!-- File: ~/ros2_ws/src/py_pubsub/urdf/simple_arm.urdf -->
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Shoulder joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Lower arm link -->
  <link name="lower_arm">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

### Loading in RViz

To visualize this URDF in RViz:

1. **Launch RViz**:
   ```bash
   rviz2
   ```

2. **Add a RobotModel display**:
   - Click "Add" in the bottom-left
   - Select "RobotModel" under "rviz_default_plugins"
   
3. **Set the description topic**:
   - Change the "Robot Description" field to "robot_description" (the default)

4. **Use robot_state_publisher to publish the URDF**:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="`cat ~/ros2_ws/src/py_pubsub/urdf/simple_arm.urdf`"
   ```

## ROS 2 Launch Files

Launch files allow you to start multiple nodes with specific parameters in a coordinated way, making complex systems easier to manage.

### Launch File Syntax

ROS 2 launch files can be written in Python or XML. The Python format is generally preferred as it's more expressive.

### Parameters and Arguments

Launch files support:
- Parameters: Values passed to nodes at startup
- Arguments: Values that can be customized when launching
- Substitutions: Dynamic values computed at runtime

### Multi-node Orchestration

Here's an example launch file that starts both our publisher and subscriber nodes:

```python
# File: ~/ros2_ws/src/py_pubsub/py_pubsub/launch/publisher_subscriber_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='velocity_publisher',
            name='velocity_publisher',
            output='screen'
        ),
        Node(
            package='py_pubsub',
            executable='sensor_subscriber',
            name='sensor_subscriber',
            output='screen'
        )
    ])
```

### Running the Launch File

To run the launch file:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch py_pubsub publisher_subscriber_launch.py
```

This will start both nodes together, which is much more convenient than starting them individually.

## Practical Lab: Humanoid Arm Control

In this practical lab, you'll create a complete working example of controlling a humanoid arm using ROS 2 nodes, URDF, and launch files.

### Goals of the Lab

- Create a ROS 2 package with joint control nodes
- Define a simple humanoid arm model in URDF
- Implement nodes that publish joint commands and read joint states
- Use a launch file to coordinate the complete system

### Getting Started

1. **Create the joint control package**:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python joint_control --dependencies rclpy std_msgs sensor_msgs trajectory_msgs builtin_interfaces
   ```

2. **Create a joint publisher node**:
   ```python
   # File: ~/ros2_ws/src/joint_control/joint_control/joint_publisher.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   import math
   import random


   class JointPublisher(Node):

       def __init__(self):
           super().__init__('joint_publisher')
           self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
           timer_period = 0.1  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.joint_names = [
               'shoulder_joint',
               'elbow_joint'
           ]
           self.i = 0

       def timer_callback(self):
           msg = JointState()
           msg.name = self.joint_names
           msg.position = [
               math.sin(self.i / 10.0),  # Shoulder joint
               math.cos(self.i / 10.0)   # Elbow joint
           ]
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = "base_link"
           self.publisher_.publish(msg)
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)
       joint_publisher = JointPublisher()
       rclpy.spin(joint_publisher)
       joint_publisher.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

3. **Update the setup.py file** for the joint_control package:
   ```python
   # File: ~/ros2_ws/src/joint_control/setup.py
   import os
   from glob import glob
   from setuptools import setup

   from setuptools import find_packages

   package_name = 'joint_control'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='Joint control nodes for ROS 2',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'joint_publisher = joint_control.joint_publisher:main',
           ],
       },
   )
   ```

4. **Build the new package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select joint_control
   source install/setup.bash
   ```

5. **Create a launch file for the complete system**:
   ```python
   # File: ~/ros2_ws/src/joint_control/joint_control/launch/arm_control_launch.py
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node


   def generate_launch_description():
       return LaunchDescription([
           # Joint publisher node
           Node(
               package='joint_control',
               executable='joint_publisher',
               name='joint_publisher',
               output='screen'
           ),
           # Robot state publisher
           Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               parameters=[
                   {'robot_description': open('/home/user/ros2_ws/src/py_pubsub/urdf/simple_arm.urdf', 'r').read()}
               ],
               output='screen'
           )
       ])
   ```

6. **Run the complete system**:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch joint_control arm_control_launch.py
   ```

7. **Visualize in RViz**:
   - Launch RViz in a new terminal: `rviz2`
   - Add a RobotModel display
   - Set the fixed frame to "base_link"

### Expected Results

You should see the simple arm model in RViz moving its joints according to the sine and cosine patterns defined in your joint publisher. This represents a complete ROS 2 system with:

- Custom nodes publishing joint state information
- URDF model describing the robot structure
- Robot state publisher translating joint states to transforms
- RViz visualizing the resulting robot model

## Common Failures and Solutions

When working with ROS 2, you'll encounter common issues. Here are solutions to the most frequent problems:

### Node Communication Problems

**Issue**: Nodes can't communicate with each other
**Solution**:
- Check that both nodes are on the same network (same ROS_DOMAIN_ID)
- Verify topic names match exactly between publisher and subscriber
- Ensure the messaging types are identical between publisher and subscriber
- Use `ros2 topic list` to verify the topic exists
- Use `ros2 topic info /topic_name` to check publishers/subscribers

### URDF Loading Errors

**Issue**: URDF file fails to load or displays incorrectly
**Solution**:
- Verify the XML is well-formed (no unmatched tags)
- Ensure all joint parent/child links exist
- Check that all referenced materials exist
- Make sure the root link is properly defined
- Use `check_urdf` to validate: `check_urdf /path/to/your.urdf`

### Gazebo Integration Issues

**Issue**: Gazebo doesn't work correctly with ROS 2 nodes
**Solution**:
- Make sure you're using Gazebo Garden or Fortress with ROS 2 Humble
- Verify plugin compatibility
- Check that ROS 2 bridges are properly configured
- Ensure Gazebo environment variables are set

### Performance Troubleshooting

**Issue**: Nodes run slowly or with high CPU usage
**Solution**:
- Reduce the frequency of your timers if they're too high
- Check for memory leaks in your nodes
- Optimize the amount of data being published
- Consider using different QoS profiles for your publishers/subscribers

## Summary and Next Steps

In this module, you've learned the fundamentals of ROS 2, the middleware that enables distributed robotic systems to communicate effectively. You now understand:

1. **ROS 2 Architecture**: How nodes, topics, services, and actions create a distributed system for robot software
2. **Environment Setup**: How to install ROS 2 Humble and create a workspace with packages
3. **Node Development**: How to create publisher and subscriber nodes that communicate with each other
4. **URDF Modeling**: How to describe robot structures using URDF format
5. **Launch Systems**: How to coordinate multiple nodes using launch files
6. **Practical Implementation**: How to create a working robotic system with joint control

These concepts form the foundation of the "robotic nervous system" that enables complex robotic behaviors. You now have the building blocks to create sophisticated robotic systems that can perceive their environment, process information, and act on the world around them.

In Module 2, we'll explore how to simulate these robotic systems in physics simulation environments. We'll cover Gazebo and other simulation tools that allow you to test your robotic systems in virtual environments that closely match real-world physics.

Continue your robotics journey by:
1. Extending the simple arm example with additional joints
2. Creating more complex URDF models
3. Experimenting with different QoS profiles for publishers/subscribers
4. Exploring the rich ecosystem of ROS 2 packages available for common robotics tasks