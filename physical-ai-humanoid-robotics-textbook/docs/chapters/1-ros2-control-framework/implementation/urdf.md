# Understanding URDF

## URDF Syntax and Structure

URDF (Universal Robot Description Format) is XML-based format that describes robot models in terms of links, joints, and the connections between them.

A typical URDF file contains:

- **Links**: Rigid components of the robot (like a chassis, arms, or wheels)
- **Joints**: Connections between links that allow relative motion
- **Visual and Collision Elements**: Describing the appearance and collision properties
- **Materials**: Color and texture information for visualization

## Links and Joints

Links are the rigid components of a robot. Each link must have a unique name and may contain visual and collision elements.

Joints connect links together and define how they can move relative to each other. Common joint types include:
- `revolute`: Rotational joint with limited range
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint
- `fixed`: No movement allowed

## Creating a Simple Humanoid URDF

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

## Loading in RViz

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