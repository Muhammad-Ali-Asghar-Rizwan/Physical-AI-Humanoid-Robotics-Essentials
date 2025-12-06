# Practical Lab: Humanoid Arm Control

## Goals of the Lab

- Create a ROS 2 package with joint control nodes
- Define a simple humanoid arm model in URDF
- Implement nodes that publish joint commands and read joint states
- Use a launch file to coordinate the complete system

## Getting Started

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

## Expected Results

You should see the simple arm model in RViz moving its joints according to the sine and cosine patterns defined in your joint publisher. This represents a complete ROS 2 system with:

- Custom nodes publishing joint state information
- URDF model describing the robot structure
- Robot state publisher translating joint states to transforms
- RViz visualizing the resulting robot model