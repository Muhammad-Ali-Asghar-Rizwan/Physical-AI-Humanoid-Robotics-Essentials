# Setting Up ROS 2 Environment

## Installing ROS 2 Humble

Before diving into ROS 2 development, we need to set up your environment. This module uses ROS 2 Humble Hawksbill distribution with Ubuntu 22.04 LTS.

### Installing ROS 2 Humble on Ubuntu 22.04 LTS

Follow these steps to install ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS:

1. **Set up the ROS 2 apt repository**:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. **Install ROS 2 Humble**:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   ```

3. **Install colcon build tool**:
   ```bash
   sudo apt install -y python3-colcon-common-extensions
   ```

4. **Install ROS 2 dependencies**:
   ```bash
   sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

5. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

6. **Add sourcing to your bashrc to make it permanent**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Creating a Workspace

ROS 2 uses workspaces to organize your projects. Create a new workspace for our work:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

This creates a workspace directory with the necessary structure. The `src` directory is where you'll place your source code packages.

### Package Structure

In ROS 2, functionality is organized into packages. A package typically contains:

- Source code (in various languages)
- Launch files to start multiple nodes
- Configuration files
- Test files
- Documentation
- Metadata files (package.xml, setup files)

The `src` directory at the root of your workspace is where you'll place your source code packages. You can create multiple packages in this directory, which will all be built together when you run the build command.