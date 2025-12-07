using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;

public class UnityROS2Bridge : MonoBehaviour
{
    [Header("ROS 2 Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("IMU Settings")]
    public string imuTopicName = "/imu/data";
    public Transform imuTransform; // The transform where the IMU is attached

    [Header("LiDAR Settings")]
    public string lidarTopicName = "/scan";
    
    [Header("Camera Settings")]
    public string cameraTopicName = "/camera/rgb/image_raw";
    
    private ROSConnection ros;
    private ImuMsg currentImuData;

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(imuTopicName);
        
        // Initialize IMU data
        currentImuData = new ImuMsg();
        currentImuData.orientation = new Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry.QuaternionMsg();
        currentImuData.angular_velocity = new Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry.Vector3Msg();
        currentImuData.linear_acceleration = new Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry.Vector3Msg();
        
        // Set header information
        currentImuData.header = new Unity.Robotics.ROSTCPConnector.MessageTypes.Std.HeaderMsg();
        currentImuData.header.frame_id = "imu_link";
    }

    // Update is called once per frame
    void Update()
    {
        // Simulate publishing IMU data
        PublishIMUData();
    }

    private void PublishIMUData()
    {
        // Update header timestamp
        currentImuData.header.stamp = new TimeMsg(
            (uint)(Time.timeSinceLevelLoad),
            (uint)((Time.timeSinceLevelLoad - Mathf.Floor(Time.timeSinceLevelLoad)) * 1e9f)
        );

        // Get simulated IMU data based on the transform's orientation and movement
        var orientation = imuTransform.rotation;
        currentImuData.orientation.x = orientation.x;
        currentImuData.orientation.y = orientation.y;
        currentImuData.orientation.z = orientation.z;
        currentImuData.orientation.w = orientation.w;

        // Simulate angular velocity (currently zero, but could be based on rotation changes)
        currentImuData.angular_velocity.x = 0.0f;
        currentImuData.angular_velocity.y = 0.0f;
        currentImuData.angular_velocity.z = 0.0f;

        // Simulate linear acceleration (could include gravity)
        Vector3 worldAcceleration = Physics.gravity;
        currentImuData.linear_acceleration.x = worldAcceleration.x;
        currentImuData.linear_acceleration.y = worldAcceleration.y;
        currentImuData.linear_acceleration.z = worldAcceleration.z;

        // Publish the message
        ros.Publish(imuTopicName, currentImuData);
    }
}