# Lab Exercise 1: Setting Up Your Gazebo Simulation Environment

## Objective
Students will learn to set up a basic Gazebo simulation environment with physics properties and run a simple simulation.

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Gazebo Garden installed
- Basic understanding of Linux command line

## Tasks

### Task 1: Verify Installation
1. Open a terminal and source your ROS environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Verify Gazebo installation:
   ```bash
   gz --version
   ```

3. Check that required packages are installed:
   ```bash
   apt list --installed | grep ros-humble-gazebo
   ```

### Task 2: Create Your First World
1. Navigate to the module-2 directory:
   ```bash
   cd ~/digital-twin-sim/module-2/gazebo/worlds
   ```

2. Create a simple world file named `my_first_world.sdf` with the following content:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="my_world">
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
     </world>
   </sdf>
   ```

### Task 3: Run Your World
1. Launch Gazebo with your world:
   ```bash
   gz sim -r my_first_world.sdf
   ```

2. Observe the basic world with ground plane and lighting.

3. Add a simple box using the Gazebo GUI:
   - Click on the "Insert" tab
   - Select a box model
   - Place it in the world
   - Observe how it falls due to gravity

### Task 4: Modify Physics Properties
1. Create a second world file `physics_world.sdf` with different gravity:
   ```xml
   <gravity>0 0 -4.9</gravity>  <!-- Half the normal gravity -->
   ```

2. Run this world and observe the difference in how objects fall.

### Task 5: Performance Testing
1. Add multiple objects to your world and observe the real-time factor (RTF) in the Gazebo window
2. The RTF should remain close to 1.0 for real-time performance

## Deliverables
- A screenshot of your Gazebo world with added objects
- Document the RTF observed in Task 5
- Describe how the physics changes affected object behavior

## Assessment Rubric
- [ ] Successfully launched Gazebo with custom world file
- [ ] Demonstrated understanding of physics properties
- [ ] Achieved RTF of ≥0.8 with multiple objects in scene
- [ ] Documented observations clearly

## Next Steps
After completing this lab, proceed to Lab Exercise 2: Configuring Robots and Sensors in Simulation.