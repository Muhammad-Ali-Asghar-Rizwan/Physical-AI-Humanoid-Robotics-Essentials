# Physics Engine Comparison and Configuration Guide

## Available Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different characteristics and performance profiles:

### ODE (Open Dynamics Engine)
- Default physics engine for Gazebo
- Excellent for rigid body simulation
- Good performance with many objects
- Well-tested and stable
- Handles contacts and joints reliably

### Bullet Physics
- Advanced collision detection
- Better for complex shapes and interactions
- More accurate contact simulation
- Slightly higher computational requirements

### Simbody
- Multibody dynamics specialized engine
- Excellent for articulated systems
- Complex joint types supported
- Good for biomechanical simulations

## Configuration Examples

### ODE Configuration (Recommended for humanoid robots)
```xml
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
```

### Bullet Configuration
```xml
<physics name="bullet_physics" type="bullet">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iterations>10</iterations>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
    </constraints>
  </bullet>
</physics>
```

### Simbody Configuration
```xml
<physics name="simbody_physics" type="simbody">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <simbody>
    <min_step_size>1e-6</min_step_size>
    <accuracy>0.001</accuracy>
    <max_transient_velocity>1.0</max_transient_velocity>
  </simbody>
</physics>
```

## Performance Comparison

| Engine | Performance | Accuracy | Stability | Use Case |
|--------|-------------|----------|-----------|----------|
| ODE | High | Good | Excellent | General simulation, humanoid robots |
| Bullet | Medium | Excellent | Good | Complex contacts, precision |
| Simbody | Medium | Excellent | Good | Articulated systems |

## Recommendations for Humanoid Robots

For humanoid robot simulation, we recommend using the ODE physics engine with the following parameters:
- Lower `max_step_size` for better stability (0.001 or smaller)
- Appropriate `real_time_update_rate` for desired simulation speed
- Adjust `iters` in the solver based on required stability vs performance balance
- For humanoid balance, consider friction coefficients between 0.8-1.2

## Testing Physics Configurations

### Stability Test
1. Load a humanoid model in the simulation
2. Run the simulation for at least 60 seconds
3. Monitor for joint instabilities or unexpected movements
4. Adjust parameters as needed

### Performance Test
1. Monitor simulation speed factor (should be close to 1.0)
2. Check CPU usage during simulation
3. Adjust `max_step_size` and solver iterations to balance stability and performance