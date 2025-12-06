# Your First ROS 2 Node

## Creating a Python Package with rclpy

Let's create your first ROS 2 package and nodes. We'll build a simple publisher and subscriber example to understand basic communication.

### Creating a New Package

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