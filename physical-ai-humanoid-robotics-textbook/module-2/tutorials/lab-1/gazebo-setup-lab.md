# Lab 1: Gazebo Setup and Basic World Creation

## Objective
Students will learn to create a basic Gazebo simulation environment with physics properties and test a humanoid robot model.

## Prerequisites
- Completed Module 1 (ROS 2 basics)
- Gazebo and ROS 2 Humble installed

## Tasks

### Task 1: Create Basic Gazebo World
1. Navigate to the gazebo worlds directory:
   ```bash
   cd module-2/gazebo/worlds
   ```

2. Create a new world file named `basic_warehouse.world` based on the template you created:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="basic_warehouse">
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

       <!-- Optional: Add a simple building model -->
       <include>
         <uri>model://cafe</uri>
       </include>
     </world>
   </sdf>
   ```

3. Launch Gazebo with your new world:
   ```bash
   gz sim basic_warehouse.world
   ```

4. Verify that:
   - The world loads without errors
   - Physics properties are applied correctly
   - Gravity is working (drop an object to test)

### Task 2: Import and Test Humanoid URDF
1. Navigate to the URDF directory:
   ```bash
   cd module-2/urdf
   ```

2. Create a launch file for your robot:
   ```xml
   <launch>
     <!-- Load the URDF into the parameter server -->
     <param name="robot_description" command="xacro $(find-pkg-share module_2_description)/urdf/humanoid.urdf.xacro" />
     
     <!-- Spawn the robot in Gazebo -->
     <node pkg="gazebo_ros" exec="spawn_entity.py" 
           args="-topic robot_description -entity humanoid_robot" />
   </launch>
   ```

3. Launch the robot in your world:
   ```bash
   ros2 launch <package_name> spawn_humanoid.launch.py
   ```

4. Verify that:
   - The robot spawns correctly in the world
   - All joints are visible and moveable
   - The robot responds to gravity appropriately

### Task 3: Test Robot Stability
1. Run the simulation for at least 60 seconds
2. Monitor the robot's stability:
   - Does it remain upright?
   - Do any joints exhibit excessive movement?
   - Does the center of mass remain stable?

3. Record your observations in a text file

## Lab Report
Create a lab report with the following sections:
1. World creation steps and any challenges encountered
2. Robot spawning results
3. Stability test results and observations
4. Any modifications made to improve stability

## Assessment Criteria
- Successfully create and load a Gazebo world
- Successfully spawn the humanoid robot
- Robot maintains stability for 60+ seconds
- Lab report with observations and potential improvements