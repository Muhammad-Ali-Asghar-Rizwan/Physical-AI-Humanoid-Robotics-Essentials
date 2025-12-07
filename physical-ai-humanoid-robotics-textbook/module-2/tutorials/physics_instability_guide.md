# Physics Instability Debugging Guide

## Common Issues and Solutions

### 1. Robot Falling Over or Unstable Joints

**Problem**: Humanoid robot falls over shortly after simulation starts
**Causes**:
- Incorrect mass distribution
- Improper joint limits or stiffness
- Physics step size too large
- Inadequate solver iterations

**Solutions**:
- Verify inertial properties are set correctly for each link
- Check that center of mass is within support polygon of feet
- Reduce physics `max_step_size` in world file (try 0.001 or smaller)
- Increase ODE solver iterations in physics configuration (try 20-50 iterations)

### 2. Penetrating or "Phantom" Collisions

**Problem**: Robot parts pass through each other or environment
**Causes**:
- Collision meshes don't match visual meshes
- Physics parameters too lenient
- Time step too large for fast movements

**Solutions**:
- Verify collision geometry matches visual geometry
- Decrease `max_step_size` in physics configuration
- Adjust ERP (Error Reduction Parameter) and CFM (Constraint Force Mixing) values
- Try different physics engine (Bullet vs ODE)

### 3. Excessive Jittering or Vibrations

**Problem**: Robot exhibits rapid oscillations at joints
**Causes**:
- High joint stiffness
- Insufficient damping
- Inconsistent mass/inertia values

**Solutions**:
- Add damping to joints (especially for humanoid models)
- Verify mass and inertia matrices are physically realistic
- Try reducing solver iterations for more compliance

### 4. NaN Values in Simulation

**Problem**: Joints report NaN (Not a Number) values
**Causes**:
- Division by zero in calculations
- Invalid initial conditions
- Numerical instability

**Solutions**:
- Check for zero mass or zero volume geometry
- Verify initial joint positions are within limits
- Review custom plugins for mathematical errors

## Debugging Techniques

### 1. Visual Debugging
```bash
# Enable contact visualization in Gazebo
gz sim -r digital_twin_world.world --verbose

# In Gazebo GUI, enable View -> Contacts
```

### 2. Logging and Monitoring
```bash
# Monitor joint states
ros2 topic echo /joint_states --field position

# Check IMU and sensor data
ros2 topic echo /imu/data
```

### 3. Physics Parameter Tuning
Start with the following conservative values:
```xml
<physics name="ode_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>1e-5</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Systematic Debugging Process

1. **Verify Model Integrity**:
   - Check URDF in RViz before simulation
   - Ensure joints have proper limits and dynamics
   - Validate mass and inertia properties

2. **Test in Isolation**:
   - Test single links without joints
   - Add joints one by one to identify problematic joints
   - Validate sensor placement separately

3. **Adjust Physics Parameters**:
   - Start with conservative time steps (0.001)
   - Increase solver iterations if experiencing instability
   - Fine-tune ERP and CFM for contact behavior

4. **Validate Simulation Results**:
   - Run for 60+ seconds to ensure stability
   - Monitor ROS 2 topics for consistent data
   - Check frame rates to ensure performance targets are met

## Student Troubleshooting Checklist

- [ ] URDF loads correctly in RViz
- [ ] Robot appears correctly in Gazebo
- [ ] Joint limits are properly defined
- [ ] Mass and inertia are specified for all links
- [ ] Center of mass is within support polygon
- [ ] Physics time step is small enough (0.001 or smaller)
- [ ] Solver iterations are sufficient (try 20+)
- [ ] Simulation runs for 60+ seconds without falling
- [ ] Sensor topics are publishing data correctly
- [ ] Unity scene renders at 30+ FPS with interactive elements