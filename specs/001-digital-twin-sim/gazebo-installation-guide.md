# Gazebo Installation Guide for ROS 2

## Compatible Versions

- **ROS 2 Distribution**: Humble Hawksbill (LTS)
- **Gazebo Version**: Garden (recommended) or Fortress
- **Ubuntu Version**: 22.04 LTS (recommended for stability)

## Installation Requirements

### System Requirements
- Ubuntu 22.04 LTS or newer
- At least 8GB RAM (16GB recommended)
- At least 20GB free disk space
- GPU with OpenGL 2.1+ support
- Multi-core processor (4+ cores recommended)

### Dependencies
- ROS 2 Humble Hawksbill properly installed
- Colcon build system
- Python 3.8 or higher
- Git
- CMake 3.16 or higher

## Installation Process

### Step 1: Add ROS 2 Gazebo Repository
```bash
sudo apt update && sudo apt install wget
sudo wget -qO - https://packages.osrfoundation.org/gazebo.gpg | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
sudo apt update
```

### Step 2: Install Gazebo
```bash
# For Gazebo Garden (recommended)
sudo apt install gz-garden

# Verify installation
gz --version
```

### Step 3: Install ROS 2 Gazebo Packages
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-ros-gz
```

### Step 4: Set Up Environment
```bash
# Add to ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Required Plugins and Configurations

### Commonly Used Plugins
- libgazebo_ros_factory.so - For spawning models via ROS
- libgazebo_ros_p3d.so - For 3D position and velocity
- libgazebo_ros_imu.so - For IMU sensor simulation
- libgazebo_ros_laser.so - For laser range finder simulation
- libgazebo_ros_camera.so - For camera sensor simulation

### Configuration Files
- ~/.gazebo/gazebo.conf - Gazebo configuration
- ~/.gazebo/models - Custom models directory
- ~/.gazebo/worlds - Custom worlds directory

## Troubleshooting Common Issues
- If Gazebo crashes on startup: Check GPU drivers and OpenGL support
- If physics simulation is unstable: Adjust time step in world file
- If models don't appear: Verify model paths and permissions