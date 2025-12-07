# Lab 3: Unity Integration and ROS 2 Bridge

## Objective
Students will set up Unity with the Robotics Hub package, import the humanoid robot, and establish bidirectional communication with ROS 2.

## Prerequisites
- Unity 2022.3 LTS installed
- Completed Module 1 (ROS 2 basics)
- Completed Module 2 Labs 1 and 2

## Tasks

### Task 1: Set Up Unity Environment
1. Install the Unity Robotics Hub package:
   - Open Unity Hub and create a new 3D project
   - In the Package Manager (Window > Package Manager), install:
     - ROS TCP Connector (from Unity Robotics Hub)
     - URDF Importer (from Unity Robotics Hub)

2. Create a new scene:
   - File > New Scene
   - Save as `digital_twin_scene.unity` in the scenes directory

3. Configure the ROS TCP Connector:
   - Create an empty GameObject and name it "ROS Connection"
   - Add the ROSConnection component to this GameObject
   - Set the IP address to `127.0.0.1` and port to `10000`

### Task 2: Import Humanoid Robot to Unity
1. Use the URDF Importer to import your humanoid URDF:
   - Go to GameObject > Import URDF
   - Set the URDF path to your `humanoid.urdf.xacro` file
   - Import with these settings:
     - Convex collision: enabled
     - Rigid body per link: enabled
     - Use articulation body: enabled

2. Adjust the robot's position in the scene:
   - Move the robot to (0, 0, 0) or desired starting location
   - Ensure the robot is upright and properly scaled

3. Add physics materials to improve stability:
   - Create a physics material with static and dynamic friction ~0.7
   - Apply to ground plane and robot feet colliders

### Task 3: Implement Unity-ROS 2 Bridge
1. Create a new C# script named `UnityROS2Bridge.cs` in the scripts directory
2. Implement the bridge functionality to:
   - Publish sensor data from Unity to ROS 2 topics
   - Subscribe to ROS 2 topics to control the Unity model
   - Synchronize robot states between Unity and ROS 2

3. Attach the script to the robot model in Unity
4. Configure the script parameters:
   - IMU topic: `/imu/data`
   - Camera topic: `/camera/rgb/image_raw`
   - LiDAR topic: `/scan` (if simulating in Unity)

### Task 4: Test Bidirectional Communication
1. Launch your Gazebo simulation:
   ```bash
   gz sim digital_twin_world.world
   ```

2. Start your ROS nodes that publish sensor data
3. Launch the Unity scene
4. Verify that:
   - Sensor data from Gazebo appears in Unity
   - Robot movements in ROS 2 are reflected in Unity
   - Data flows correctly in both directions

5. Create a simple test script to verify communication:
   ```csharp
   using UnityEngine;
   using Unity.Robotics.ROSTCPConnector;
   using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

   public class CommunicationTester : MonoBehaviour
   {
       public string testTopic = "/unity/test_message";
       private ROSConnection ros;

       void Start()
       {
           ros = ROSConnection.GetOrCreateInstance();
           ros.RegisterPublisher<StringMsg>(testTopic);
           
           // Send a test message every 5 seconds
           InvokeRepeating("SendTestMessage", 2.0f, 5.0f);
       }

       void SendTestMessage()
       {
           ros.Publish(testTopic, new StringMsg($"Test message from Unity at time {Time.time}"));
           Debug.Log("Sent test message to ROS");
       }
   }
   ```

### Task 5: Visualize Unity Scene
1. Set up cameras in Unity:
   - Main camera for general view
   - Additional cameras attached to robot (for sensor simulation)
   - Camera angles that match your ROS 2 camera configuration

2. Add lighting and materials:
   - Use physically based rendering (PBR) materials
   - Set up lighting that matches Gazebo environment
   - Ensure visual consistency with the intended application

## Lab Report
Create a lab report with the following sections:
1. Unity setup process and package installation
2. URDF import procedure and any configuration changes needed
3. ROS-TCP connection testing and verification
4. Sensor data synchronization between environments
5. Screenshots of Unity scene with humanoid robot
6. Any challenges encountered and how they were resolved

## Assessment Criteria
- Successfully import URDF model into Unity
- Establish ROS-TCP communication
- Demonstrate bidirectional data flow
- Create a visually appealing Unity scene
- Complete lab report with proper documentation