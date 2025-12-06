# Quickstart Guide: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-06
**Feature**: Module 1 - The Robotic Nervous System (ROS 2)
**Spec**: specs/001-ros2-control-framework/spec.md

## Overview

This quickstart guide will help you set up your ROS 2 development environment and run your first ROS 2 publisher and subscriber nodes. By the end of this guide, you'll have:

1. Installed ROS 2 Humble Hawksbill
2. Created a simple ROS 2 package
3. Written a publisher node that sends messages
4. Written a subscriber node that receives messages
5. Tested the communication between nodes

## Prerequisites

- Ubuntu 22.04 LTS (recommended) or Windows 11/macOS with ROS 2 support
- Basic Python knowledge
- Terminal/command line familiarity
- 4GB+ RAM, 20GB+ free disk space

## Setup ROS 2 Environment

### For Ubuntu 22.04 LTS (Recommended)

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

### For Windows/macOS

Follow the official ROS 2 Humble installation guide for your platform:
- [Windows Installation Guide](https://docs.ros.org/en/humble/Installation/Windows-Install.html)
- [macOS Installation Guide](https://docs.ros.org/en/humble/Installation/macOS.html)

## Create Your First ROS 2 Package

1. **Create a ROS workspace**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. **Create a new package**:
```bash
cd src
ros2 pkg create --build-type ament_python py_pubsub
```

3. **Navigate to the package**:
```bash
cd py_pubsub
ls -la
```

You should see the basic package structure with directories like `py_pubsub/` (for Python modules) and `test/`.

## Write a Publisher Node

1. **Open the main Python module file**:
```bash
cd ~/ros2_ws/src/py_pubsub/py_pubsub
ls -la
```

2. **Replace the contents of `publisher_member_function.py`** with the publisher code:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Write a Subscriber Node

1. **Create a new file for the subscriber**:
```bash
touch ~/ros2_ws/src/py_pubsub/py_pubsub/subscriber_member_function.py
```

2. **Add the subscriber code** to the file:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Make Python Files Executable and Update Setup

1. **Make the Python files executable**:
```bash
cd ~/ros2_ws/src/py_pubsub
chmod +x py_pubsub/publisher_member_function.py
chmod +x py_pubsub/subscriber_member_function.py
```

2. **Edit the `setup.py` file** to include the entry points:
```python
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
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
```

## Build and Run Your Nodes

1. **Build the package**:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash  # only if new terminal
colcon build --packages-select py_pubsub
```

2. **Source your workspace**:
```bash
source install/setup.bash
```

3. **Open a new terminal and run the publisher**:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub talker
```

4. **In another terminal, run the subscriber**:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub listener
```

5. **You should see the publisher sending messages and the subscriber receiving them**:
```
[INFO] [1611186610.123456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1611186610.623456789] [minimal_subscriber]: I heard: "Hello World: 0"
```

## Visualize with RViz2

1. **Launch RViz2**:
```bash
rviz2
```

2. **In RViz2, set the Fixed Frame** to `odom` or `base_link` depending on your URDF model.

3. **Add displays** for robot model visualization:
   - Add by clicking "Add" in the bottom-left
   - Select "RobotModel" under "rviz_default_plugins"

4. **Load a URDF model** to visualize:
   - Create a URDF file with a simple robot model
   - Point the RobotModel display to your URDF file location

## Troubleshooting Common Issues

### Problem: Command not found
**Solution**: Make sure you've sourced your ROS environment:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Problem: Package not found when running
**Solution**: 
1. Check that the package builds successfully: `colcon build`
2. Ensure you sourced the workspace after building: `source install/setup.bash`

### Problem: Nodes not communicating
**Solution**:
1. Verify both nodes are on the same topic: `ros2 topic list`
2. Check topic types match: `ros2 topic info /topic`
3. Confirm both nodes are running in the same ROS domain

### Problem: Python import errors
**Solution**: Make sure the package was built with colcon and you've sourced the workspace.

## Next Steps

After completing this quickstart, you understand:
- How to create a ROS 2 package
- How to write publisher and subscriber nodes
- How to build and run ROS 2 nodes
- How to visualize robots with RViz2

Continue with Module 1 content to learn about:
- Advanced ROS 2 concepts (services, actions)
- Creating URDF robot descriptions
- Using launch files to coordinate multiple nodes
- Debugging ROS 2 communication issues