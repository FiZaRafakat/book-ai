# Data Model: ROS 2 Robotics Module

## Overview
This document defines the key entities and concepts for the ROS 2 Robotics Module. Since this is a documentation project, the "data model" represents the conceptual entities that students will learn about.

## Core ROS 2 Entities

### ROS 2 Node
- **Definition**: A process that performs computation in the ROS 2 system; fundamental building block that communicates with other nodes
- **Attributes**:
  - node_name: String identifier for the node
  - namespace: Optional grouping mechanism
  - lifecycle: Created, Active, Inactive, Finalized states
- **Relationships**: Communicates with other nodes via topics, services, and actions
- **Validation**: Must have a unique name within its namespace

### ROS 2 Topic
- **Definition**: A named bus over which nodes exchange messages in a publish-subscribe pattern
- **Attributes**:
  - topic_name: String identifier for the topic
  - message_type: Defines the structure of messages sent on the topic
  - qos_profile: Quality of Service settings (reliability, durability, etc.)
- **Relationships**: Publishers send messages to topics, subscribers receive messages from topics
- **Validation**: Message types must be consistent across all publishers and subscribers

### ROS 2 Service
- **Definition**: A request-response communication pattern between nodes for synchronous operations
- **Attributes**:
  - service_name: String identifier for the service
  - request_type: Structure of the request message
  - response_type: Structure of the response message
  - qos_profile: Quality of Service settings
- **Relationships**: Service clients send requests, service servers process requests and send responses
- **Validation**: Request and response types must match between client and server

### ROS 2 Action
- **Definition**: A goal-oriented communication pattern with feedback and status updates
- **Attributes**:
  - action_name: String identifier for the action
  - goal_type: Structure of the goal message
  - feedback_type: Structure of the feedback message
  - result_type: Structure of the result message
- **Relationships**: Action clients send goals to action servers, receive feedback and results
- **Validation**: All message types must be consistent between client and server

### URDF Model
- **Definition**: XML-based format describing robot structure including links, joints, and their physical properties
- **Attributes**:
  - links: Collection of rigid body elements
  - joints: Collection of connections between links
  - materials: Visual and physical properties
  - transmissions: Mapping between joints and actuators
- **Relationships**: Links connected by joints form the robot kinematic chain
- **Validation**: Must be well-formed XML, joints must connect valid links

## Link Entity (URDF Component)
- **Definition**: A rigid body element in a robot description
- **Attributes**:
  - name: Unique identifier for the link
  - visual: Geometry and material properties for visualization
  - collision: Geometry properties for collision detection
  - inertial: Mass, center of mass, and inertia properties
- **Relationships**: Connected to other links via joints

## Joint Entity (URDF Component)
- **Definition**: Connection between two links in a robot description
- **Attributes**:
  - name: Unique identifier for the joint
  - type: Revolute, Continuous, Prismatic, Fixed, etc.
  - parent: Parent link in the kinematic chain
  - child: Child link in the kinematic chain
  - axis: Axis of motion for the joint
  - limits: Range of motion and effort/torque limits
- **Relationships**: Connects parent and child links in the robot structure

## Frame Entity (URDF Component)
- **Definition**: Coordinate system attached to a link for spatial relationships
- **Attributes**:
  - frame_id: Identifier for the coordinate frame
  - position: 3D position relative to parent frame
  - orientation: 3D orientation relative to parent frame
- **Relationships**: Establishes spatial relationship between different parts of the robot

## Chapter Content Structure
### Chapter 1: ROS 2 Basics for Physical AI
- Covers Node entity
- Covers Topic entity
- Covers Service entity
- Covers Action entity
- Explains ROS 2 as a distributed robotic nervous system

### Chapter 2: Python Control with rclpy
- Explains implementation of Node using rclpy
- Shows Publisher/Subscriber patterns with Topic
- Demonstrates Service Client/Server implementations
- Connects AI agents to ROS controllers

### Chapter 3: Humanoid Structure with URDF
- Explains URDF Model entity
- Details Link and Joint entities
- Explains Frame concept
- Shows reading and modifying URDF for humanoids

## Validation Rules
- All entities must be explained at introductory level
- Examples must use Python and rclpy as specified
- Content must avoid ROS 1, Gazebo, Isaac, Unity, or hardware drivers
- Humanoid focus must be maintained throughout