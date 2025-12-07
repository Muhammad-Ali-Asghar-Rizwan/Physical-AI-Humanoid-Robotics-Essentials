# Module 2: Digital Twin Simulation for Gazebo & Unity

This module teaches students to create realistic physics simulations and immersive environments for humanoid robots using Gazebo and Unity. Students will learn to simulate sensors, test robot behavior in controlled environments, and validate designs before real-world deployment.

## Directory Structure

```
module-2/
├── gazebo/           # Gazebo simulation files
│   ├── worlds/       # World files (.world)
│   ├── models/       # 3D models
│   └── plugins/      # Custom Gazebo plugins
├── unity/            # Unity project files
│   ├── scenes/       # Unity scenes
│   ├── prefabs/      # Reusable Unity objects
│   └── scripts/      # Unity C# scripts
├── urdf/             # Robot description files
├── ros2_packages/    # ROS 2 packages
└── tutorials/        # Lab exercises and guides
    ├── lab-1/        # Lab 1: Gazebo Setup
    ├── lab-2/        # Lab 2: Sensor Configuration
    └── lab-3/        # Lab 3: Unity Integration
```

## Learning Outcomes

By completing this module, students will be able to:

1. Set up Gazebo simulation environments with custom physics properties
2. Import and simulate URDF humanoid models with proper joint constraints
3. Configure virtual sensors (LiDAR, depth cameras, IMUs) for robot perception
4. Create photorealistic Unity environments for enhanced visualization and HRI
5. Implement Unity-ROS 2 bridge for bidirectional communication
6. Validate robot stability and collision handling in simulation
7. Generate synthetic training data from simulation runs
8. Debug physics instabilities in humanoid robot simulations

## Prerequisites

- Completion of Module 1 (ROS 2 basics)
- Basic understanding of robot kinematics and dynamics
- Familiarity with Linux command line

## Getting Started

1. Complete the lab exercises in the `tutorials/` directory in order
2. Follow along with the documentation in the main textbook
3. Practice with the example files provided in each subdirectory
4. Experiment with different physics parameters and sensor configurations

## Key Concepts

- **Digital Twins**: Virtual representations of physical systems
- **Physics Simulation**: Accurate modeling of real-world physics
- **Sensor Simulation**: Replicating real sensor behavior in virtual environments
- **ROS 2 Integration**: Connecting simulation environments to ROS 2
- **Unity Bridge**: Connecting photorealistic visualization with robotics

## Tools Used

- Gazebo (Harmonic) for physics simulation
- Unity 2022.3 LTS for visualization
- ROS 2 Humble Hawksbill for robot communication
- RViz for data visualization
- Docusaurus for documentation

This module serves as a bridge between basic ROS 2 concepts learned in Module 1 and advanced perception systems in Module 3.