# ROS 2 Launch Files

## Launch File Syntax

ROS 2 launch files allow you to start multiple nodes with specific parameters in a coordinated way, making complex systems easier to manage. They can be written in Python or XML. The Python format is generally preferred as it's more expressive.

## Parameters and Arguments

Launch files support:
- Parameters: Values passed to nodes at startup
- Arguments: Values that can be customized when launching
- Substitutions: Dynamic values computed at runtime

## Multi-node Orchestration

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

## Running the Launch File

To run the launch file:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch py_pubsub publisher_subscriber_launch.py
```

This will start both nodes together, which is much more convenient than starting them individually.