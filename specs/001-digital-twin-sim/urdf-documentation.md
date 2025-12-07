# Humanoid URDF Structure Guide

## Overview
This guide explains the structure of the humanoid robot model defined in the URDF file. URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS.

## URDF Components

### Links
Links represent rigid bodies of the robot. Each link has:
- Visual properties (shape, color, material)
- Collision properties (shape for physics simulation)
- Inertial properties (mass, center of mass, inertia matrix)

### Joints
Joints connect links and define how they move relative to each other. Types include:
- Fixed: No movement between links
- Revolute: Rotational movement around a single axis
- Continuous: Unlimited rotational movement
- Prismatic: Linear sliding movement

## Humanoid Robot Structure

### Torso Assembly
- base_link: Lower body section with balancing support
- torso: Main body section with arms connected
- head: Top section with sensor mounting capability

### Limbs
- Arms: Upper and lower segments with joints for movement
- Legs: Upper and lower segments with joints for walking/standing
- Feet: End effectors for ground contact and stability

## Joint Constraints
- Joint limits: Define the range of motion for revolute joints
- Safety: Prevents self-collision and damage to the physical robot
- Control: Provides reference points for robot controllers

## Verification Steps
1. Load the URDF in RViz to check visual representation
2. Verify joint connectivity and movement range
3. Test in Gazebo simulation to ensure physics behave correctly
4. Check that joint controllers can properly command movements

## Best Practices for Humanoid Models
1. Maintain proper mass distribution for stability
2. Configure appropriate inertial properties for physics simulation
3. Set realistic joint limits based on human anatomy or robot design
4. Include appropriate collision geometry to prevent self-intersection
5. Use fixed joints to connect non-moving parts like feet