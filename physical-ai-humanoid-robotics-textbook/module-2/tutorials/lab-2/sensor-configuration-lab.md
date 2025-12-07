# Lab 2: Sensor Configuration and Integration

## Objective
Students will configure virtual sensors (LiDAR, depth camera, IMU) on the humanoid robot and verify data publication to ROS 2 topics.

## Prerequisites
- Completed Lab 1: Gazebo Setup
- Basic understanding of ROS 2 topics and messages

## Tasks

### Task 1: Configure LiDAR Sensor
1. Navigate to the URDF directory:
   ```bash
   cd module-2/urdf
   ```

2. Update your humanoid URDF to include a LiDAR sensor on the robot's head:
   ```xml
   <!-- Add this to your head link definition -->
   <gazebo reference="head">
     <sensor name="lidar_head" type="ray">
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
       <plugin name="lidar_head_controller" filename="libgazebo_ros_ray_sensor.so">
         <ros>
           <namespace>lidar</namespace>
           <argument>~/out:=scan</argument>
         </ros>
         <output_type>sensor_msgs/LaserScan</output_type>
         <frame_name>head</frame_name>
       </plugin>
     </sensor>
   </gazebo>
   ```

3. Launch the updated robot in Gazebo:
   ```bash
   gz sim digital_twin_world.world
   ```

4. Verify the LiDAR is publishing data:
   ```bash
   ros2 topic echo /lidar/scan
   ```

### Task 2: Configure Depth Camera
1. Add a depth camera sensor to your URDF in the torso section:
   ```xml
   <gazebo reference="torso">
     <sensor name="depth_camera" type="depth">
       <always_on>true</always_on>
       <update_rate>30</update_rate>
       <camera name="head">
         <horizontal_fov>1.047</horizontal_fov>
         <image>
           <format>R8G8B8</format>
           <width>640</width>
           <height>480</height>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
       </camera>
       <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
         <ros>
           <namespace>camera</namespace>
           <remapping>~/depth/image_raw:=/camera/depth/image_raw</remapping>
           <remapping>~/depth/points:=/camera/depth/points</remapping>
           <remapping>~/rgb/image_raw:=/camera/rgb/image_raw</remapping>
           <remapping>~/rgb/camera_info:=/camera/rgb/camera_info</remapping>
         </ros>
         <frame_name>torso</frame_name>
       </plugin>
     </sensor>
   </gazebo>
   ```

2. Launch the updated robot in Gazebo and verify the camera topics:
   ```bash
   ros2 topic list | grep camera
   ros2 topic echo /camera/rgb/image_raw
   ```

### Task 3: Configure IMU Sensor
1. You already have the IMU sensor configured in your `imu_config.xacro` file from earlier.

2. Verify the IMU is publishing data:
   ```bash
   ros2 topic echo /imu/data
   ```

3. Test with a simple subscriber to see the data:
   ```python
   import rclpy
   from sensor_msgs.msg import Imu
   from rclpy.node import Node

   class ImuTester(Node):
       def __init__(self):
           super().__init__('imu_tester')
           self.subscription = self.create_subscription(
               Imu,
               '/imu/data',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'IMU Orientation: x={msg.orientation.x}, y={msg.orientation.y}, z={msg.orientation.z}, w={msg.orientation.w}')

   def main(args=None):
       rclpy.init(args=args)
       imu_tester = ImuTester()
       rclpy.spin(imu_tester)
       imu_tester.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Task 4: Visualize All Sensor Data
1. Launch RViz to visualize all sensor data:
   ```bash
   rviz2
   ```

2. Add displays for:
   - LaserScan (topic: `/lidar/scan`)
   - Image (topic: `/camera/rgb/image_raw`)
   - RobotModel (topic: `/robot_description`)

3. Verify that all sensors are publishing data correctly and are visualized in RViz

4. Record sensor data to a rosbag:
   ```bash
   ros2 bag record /lidar/scan /camera/rgb/image_raw /imu/data
   ```

## Lab Report
Create a lab report with the following sections:
1. Configuration steps for each sensor
2. Verification results for each sensor topic
3. Screenshots of sensor data in RViz
4. Any issues encountered and solutions
5. Analysis of sensor data quality

## Assessment Criteria
- Successfully configure LiDAR, depth camera, and IMU sensors
- All sensors publish data to correct ROS 2 topics
- Data is visualizable in RViz
- Lab report with complete documentation and analysis