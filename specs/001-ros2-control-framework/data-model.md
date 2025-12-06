# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-06
**Feature**: Module 1 - The Robotic Nervous System (ROS 2)
**Spec**: specs/001-ros2-control-framework/spec.md

## Key Entities

### ROS 2 Package
- **Description**: A collection of nodes, libraries, and data that function as a single unit within ROS 2
- **Attributes**: 
  - package_name: string (alphanumeric with underscores, lowercase)
  - version: string (semantic versioning format)
  - maintainer: string (contact information)
  - license: string (e.g., MIT, Apache 2.0)
  - dependencies: list of strings (other ROS packages required)
- **Relationships**: Contains multiple Nodes, may import other Packages
- **Validations**: 
  - Package name must follow ROS naming conventions
  - Must include necessary configuration files (package.xml, setup.py)

### Node
- **Description**: A process that performs computation in ROS 2, communicating with other nodes through topics, services, and actions
- **Attributes**: 
  - node_name: string (unique within the ROS graph)
  - namespace: string (optional, provides logical grouping)
  - lifecycle_state: enum (unconfigured, inactive, active, finalized)
  - topics_published: list of Topic references
  - topics_subscribed: list of Topic references
- **Relationships**: Belongs to a Package, communicates via Topics/Services/Actions
- **Validations**: 
  - Node name must be unique in the namespace
  - Must properly initialize and shutdown resources

### Topic
- **Description**: A unidirectional communication channel where publishers send messages to subscribers
- **Attributes**: 
  - topic_name: string (with proper namespace)
  - message_type: string (e.g., std_msgs/msg/String)
  - qos_profile: QoS object (quality of service settings)
  - publishers: list of Node references
  - subscribers: list of Node references
- **Relationships**: Nodes publish to or subscribe from Topics
- **Validations**: 
  - Topic names must follow ROS naming conventions
  - Message types must be valid ROS message definitions
  - QoS settings must be compatible between publishers and subscribers

### Service
- **Description**: A synchronous request/response communication pattern between nodes
- **Attributes**: 
  - service_name: string (with proper namespace)
  - service_type: string (e.g., std_srvs/srv/SetBool)
  - clients: list of Node references
  - server: Node reference
- **Relationships**: Nodes can be clients or servers for Services
- **Validations**: 
  - Service names must follow ROS naming conventions
  - Service types must be valid ROS service definitions

### Action
- **Description**: A communication pattern for long-running tasks with feedback
- **Attributes**: 
  - action_name: string (with proper namespace)
  - action_type: string (e.g., example_interfaces/action/Fibonacci)
  - clients: list of Node references
  - servers: list of Node references
- **Relationships**: Nodes can be clients or servers for Actions
- **Validations**: 
  - Action names must follow ROS naming conventions
  - Action types must be valid ROS action definitions

### URDF Model
- **Description**: Universal Robot Description Format, an XML format to describe robot models including links, joints, and transforms
- **Attributes**: 
  - model_name: string (name of the robot model)
  - links: list of Link objects
  - joints: list of Joint objects
  - materials: list of Material objects
  - gazebo_extensions: list of Gazebo-specific extensions
- **Relationships**: Used by simulation and visualization tools
- **Validations**: 
  - Must have a single root link
  - Joint connections must form a valid kinematic chain
  - All referenced materials and meshes must exist

### Link
- **Description**: A rigid component of a robot in URDF
- **Attributes**: 
  - link_name: string (unique within the URDF model)
  - visual: Visual object (for visualization)
  - collision: Collision object (for physics simulation)
  - inertial: Inertial object (mass, center of mass, etc.)
- **Relationships**: Part of a URDF Model, connected via Joints
- **Validations**: 
  - Name must be unique within the model
  - Must have valid geometry definition

### Joint
- **Description**: Connection between two links in URDF
- **Attributes**: 
  - joint_name: string (unique within the URDF model)
  - joint_type: enum (revolute, continuous, prismatic, fixed, etc.)
  - parent_link: string (name of parent link)
  - child_link: string (name of child link)
  - origin: transform (position and orientation of joint)
  - axis: vector (axis of rotation/translation)
- **Relationships**: Part of a URDF Model, connects Links
- **Validations**: 
  - Name must be unique within the model
  - Joint must connect exactly two links
  - Joint type must be valid and consistent with limits

### Launch File
- **Description**: An XML or Python file that starts multiple nodes with specific parameters in a coordinated way
- **Attributes**: 
  - launch_file_name: string (name of the launch file)
  - nodes: list of Node configurations
  - parameters: list of Parameter objects
  - remappings: list of Topic remapping rules
  - conditions: list of conditional launch rules
- **Relationships**: References and launches Nodes with specific configurations
- **Validations**: 
  - Must have valid syntax for XML or Python format
  - Referenced nodes and parameters must exist
  - Conditions must be properly formatted

### Parameter
- **Description**: Configuration values that can be set for nodes at runtime
- **Attributes**: 
  - param_name: string (name of the parameter)
  - param_value: any (the parameter value)
  - param_type: enum (integer, double, string, boolean, list)
  - node_reference: Node (which node the parameter applies to)
  - description: string (documentation of the parameter's purpose)
- **Relationships**: Associated with Nodes, used in Launch Files
- **Validations**: 
  - Parameter name must follow ROS naming conventions
  - Value type must match declared parameter type
  - Must be valid according to the node's parameter definitions

## State Transitions

### Node Lifecycle States
- unconfigured → inactive (via configure transition)
- inactive → active (via activate transition)
- active → inactive (via deactivate transition)
- inactive → unconfigured (via cleanup transition)
- any state → finalized (via shutdown)

### Launch File Execution States
- defined → loading (when launch system starts)
- loading → ready (when all dependencies resolved)
- ready → running (when nodes are launched)
- running → shutting_down (when shutdown initiated)
- shutting_down → complete (when all nodes terminated)