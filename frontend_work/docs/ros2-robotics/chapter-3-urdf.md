---
sidebar_position: 3
---

# Chapter 3: Humanoid Structure with URDF

## Introduction to URDF

Welcome to Chapter 3 of the ROS 2 Robotics Module! In this chapter, we'll explore URDF (Unified Robot Description Format), which is the standard way to describe robot models in ROS. URDF is particularly important for humanoid robots as it allows us to define their complex structure with multiple links and joints.

URDF is an XML-based format that describes robot models, including their physical and visual properties. It's used extensively in ROS for simulation, visualization, and understanding robot kinematics. For humanoid robots, URDF becomes essential as it defines the complex structure with multiple degrees of freedom required for human-like movement.

## What is URDF?

URDF stands for Unified Robot Description Format. It's an XML-based format that allows you to describe a robot's structure, including:

- **Links**: Rigid bodies that make up the robot
- **Joints**: Connections between links that allow movement
- **Visual properties**: How the robot appears in simulation and visualization
- **Collision properties**: How the robot interacts with its environment
- **Inertial properties**: Physical characteristics for simulation

For humanoid robots, URDF provides a standardized way to describe the complex kinematic structure that mimics the human body with limbs, joints, and degrees of freedom.

## URDF in the Context of Humanoid Robots

Humanoid robots have a specific structure that mimics the human form, typically including:

- A torso/chest
- A head with possible degrees of freedom
- Two arms with shoulders, elbows, and wrists
- Two legs with hips, knees, and ankles
- Possible hands with fingers

URDF allows us to define these components and their relationships precisely, enabling accurate simulation and control of humanoid robots.

## URDF Model Structure and XML Format

The URDF model is based on XML and follows a hierarchical structure that represents the robot as a collection of rigid bodies (links) connected by joints. This structure forms a kinematic tree, with one link designated as the base (root) and all other links connected through joints.

### Root Structure

Every URDF file starts with the robot tag:

```xml
<?xml version="1.0"?>
<robot name="robot_name" version="1.0">
  <!-- Robot description content -->
</robot>
```

The robot tag can include optional attributes like `version` and must have a unique `name`.

### Link Elements

Links represent rigid bodies in the robot. Each link can have multiple sub-elements:

```xml
<link name="link_name">
  <!-- Visual properties for display -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
      <!-- or <cylinder radius="0.5" length="1"/> -->
      <!-- or <sphere radius="0.5"/> -->
      <!-- or <mesh filename="mesh.stl"/> -->
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joint Elements

Joints connect links and define how they can move relative to each other:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>

  <!-- For joints with limits (revolute, prismatic) -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>

  <!-- For continuous joints -->
  <!-- No limits needed -->

  <!-- For fixed joints -->
  <!-- No motion allowed -->
</joint>
```

### Common XML Elements and Attributes

- `name`: Unique identifier for links, joints, and other elements
- `type`: Defines the type of joint or geometry
- `xyz`: Position coordinates (x, y, z)
- `rpy`: Rotation coordinates (roll, pitch, yaw) in radians
- `size`: Dimensions for box geometry
- `radius` and `length`: Dimensions for cylinder geometry
- `radius`: Dimension for sphere geometry
- `filename`: Path to mesh file
- `rgba`: Color values (red, green, blue, alpha)
- `value`: Numeric values for mass, limits, etc.

### Complete Simple URDF Example

Here's a complete example of a simple robot arm with 2 links and 1 joint:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Link Entity in URDF

A Link represents a rigid body in the robot model. It's the fundamental building block that defines the physical structure of the robot. In humanoid robots, links correspond to body parts like the torso, head, arms, legs, hands, and feet.

### Link Attributes and Properties

Each link in URDF can contain several important properties that define its physical and visual characteristics:

#### 1. Visual Properties
The visual section defines how the link appears in simulation and visualization:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Shape definition -->
  </geometry>
  <material name="material_name">
    <color rgba="0.8 0.2 0.2 1.0"/>
  </material>
</visual>
```

- `origin`: Position and orientation of the visual element relative to the link frame
- `geometry`: Defines the shape (box, cylinder, sphere, or mesh)
- `material`: Defines the color and appearance properties

#### 2. Collision Properties
The collision section defines how the link interacts with other objects in simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Shape definition -->
  </geometry>
</collision>
```

Collision geometry can be simpler than visual geometry for computational efficiency.

#### 3. Inertial Properties
The inertial section defines the physical properties for dynamics simulation:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

- `mass`: The mass of the link in kilograms
- `inertia`: The 3x3 inertia matrix values (symmetric, so only 6 unique values)

### Link Relationships in Humanoid Robots

In humanoid robots, links are organized in a hierarchical structure that mimics the human body:

```
base_link (usually torso)
├── head_link
├── upper_arm_left_link
│   └── lower_arm_left_link
│       └── hand_left_link
├── upper_arm_right_link
│   └── lower_arm_right_link
│       └── hand_right_link
├── upper_leg_left_link
│   └── lower_leg_left_link
│       └── foot_left_link
└── upper_leg_right_link
    └── lower_leg_right_link
        └── foot_right_link
```

### Example: Humanoid Torso Link

Here's an example of how a humanoid torso link might be defined:

```xml
<link name="torso">
  <visual>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 1.0"/>
    </geometry>
    <material name="light_grey">
      <color rgba="0.7 0.7 0.7 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 1.0"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.2"/>
  </inertial>
</link>
```

### Common Link Naming Conventions

For humanoid robots, common naming conventions include:

- `base_link` or `torso`: The main body
- `head`: The head link
- `upper_arm_left`, `lower_arm_left`, `hand_left`: Left arm components
- `upper_arm_right`, `lower_arm_right`, `hand_right`: Right arm components
- `upper_leg_left`, `lower_leg_left`, `foot_left`: Left leg components
- `upper_leg_right`, `lower_leg_right`, `foot_right`: Right leg components

## Joint Entity in URDF

Joints define the connection between two links and specify how they can move relative to each other. In humanoid robots, joints are crucial for enabling the complex movements that mimic human motion, such as arm swinging, leg bending, and head turning.

### Joint Types

URDF supports several joint types, each allowing different types of motion:

#### 1. Revolute Joints
Revolute joints allow rotation around a single axis, with limits on the rotation range:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.5" effort="50" velocity="2"/>
</joint>
```

**Use case in humanoid robots**: Elbow, knee, wrist, and finger joints.

#### 2. Continuous Joints
Continuous joints allow unlimited rotation around a single axis:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base"/>
  <child link="rotating_part"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

**Use case in humanoid robots**: Waist rotation, head rotation (pan).

#### 3. Prismatic Joints
Prismatic joints allow linear translation along a single axis:

```xml
<joint name="slider_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0.0" upper="0.5" effort="100" velocity="1"/>
</joint>
```

**Use case in humanoid robots**: Less common, but could be used for telescoping mechanisms.

#### 4. Fixed Joints
Fixed joints connect two links rigidly with no relative motion:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```

**Use case in humanoid robots**: Connecting sensors to links, or when two parts are permanently attached.

#### 5. Floating Joints
Floating joints allow motion in all 6 degrees of freedom:

```xml
<joint name="floating_joint" type="floating">
  <parent link="world"/>
  <child link="free_body"/>
</joint>
```

**Use case in humanoid robots**: Rarely used for main body joints, but could be used for simulating free-floating objects.

#### 6. Planar Joints
Planar joints allow motion in a plane and rotation around the normal to that plane:

```xml
<joint name="planar_joint" type="planar">
  <parent link="base"/>
  <child link="platform"/>
  <limit effort="100" velocity="1"/>
</joint>
```

**Use case in humanoid robots**: Rarely used for main body joints.

### Joint Attributes and Properties

Each joint in URDF has several important properties:

- `name`: A unique identifier for the joint
- `type`: The type of joint (revolute, continuous, prismatic, fixed, floating, planar)
- `parent`: The parent link in the kinematic tree
- `child`: The child link in the kinematic tree
- `origin`: Position and orientation of the joint relative to the parent link
- `axis`: The axis of motion for the joint (for revolute, continuous, and prismatic joints)
- `limit`: For joints with limits, specifies the range of motion, maximum effort, and maximum velocity

### Joint Relationships in Humanoid Robots

In humanoid robots, joints connect the links to form a kinematic structure that enables human-like movement:

```
torso -> (revolute) -> head (for neck movement)
torso -> (revolute) -> upper_arm_left (for shoulder)
upper_arm_left -> (revolute) -> lower_arm_left (for elbow)
lower_arm_left -> (revolute) -> hand_left (for wrist)

torso -> (revolute) -> upper_arm_right (for shoulder)
upper_arm_right -> (revolute) -> lower_arm_right (for elbow)
lower_arm_right -> (revolute) -> hand_right (for wrist)

torso -> (fixed) -> upper_leg_left (for hip - simplified)
upper_leg_left -> (revolute) -> lower_leg_left (for knee)
lower_leg_left -> (revolute) -> foot_left (for ankle)

torso -> (fixed) -> upper_leg_right (for hip - simplified)
upper_leg_right -> (revolute) -> lower_leg_right (for knee)
lower_leg_right -> (revolute) -> foot_right (for ankle)
```

### Example: Humanoid Shoulder Joint

Here's an example of how a humanoid shoulder joint might be defined:

```xml
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm_left"/>
  <origin xyz="0.1 0.2 0.5" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
</joint>

<joint name="left_shoulder_yaw" type="revolute">
  <parent link="upper_arm_left"/>
  <child link="shoulder_helper"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
</joint>

<joint name="left_shoulder_roll" type="revolute">
  <parent link="shoulder_helper"/>
  <child link="lower_arm_left"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="2"/>
</joint>
```

## Frame Concept and Coordinate Systems

Frames and coordinate systems are fundamental concepts in robotics that allow us to define positions, orientations, and transformations in 3D space. In URDF, each link has its own coordinate frame, and joints define transformations between these frames.

### Coordinate System Convention

ROS and URDF follow the right-hand rule for coordinate systems:

- **X-axis**: Points forward (in the direction of the robot's forward movement)
- **Y-axis**: Points to the left (perpendicular to X, following right-hand rule)
- **Z-axis**: Points upward (perpendicular to X and Y, following right-hand rule)

This is consistent with the ROS standard coordinate frame convention.

### Frames in URDF

Every link in a URDF model has an associated coordinate frame. The origin of this frame is defined relative to the parent link's frame by the joint that connects them. The frame's orientation is determined by the joint's axis of rotation or translation.

#### Origin Element

The `<origin>` element in both links and joints defines the position and orientation of a frame relative to its parent:

```xml
<origin xyz="1.0 2.0 3.0" rpy="0.1 0.2 0.3"/>
```

- `xyz`: Position in meters (x, y, z coordinates)
- `rpy`: Orientation in radians (roll, pitch, yaw angles)

### Transformations Between Frames

Joints define transformations between the coordinate frames of connected links. For example, if a joint connects `link1` to `link2`, the joint's origin and axis define how `link2`'s frame is positioned and oriented relative to `link1`'s frame.

### Frame Naming and TF

In ROS, the transform framework (TF) is used to keep track of all coordinate frames in a robot system. Each link in URDF automatically gets a frame with the same name as the link. For example, a link named `upper_arm_left` will have a corresponding frame named `upper_arm_left`.

### Example: Frame Relationships in a Humanoid Arm

Let's consider how frames work in a simple humanoid arm:

```
Frame: torso
├── Position: World origin
└── Joint: left_shoulder
    ├── Transformation: [0.1, 0.2, 0.5] [0, 0, 0]
    └── Frame: upper_arm_left
        └── Joint: left_elbow
            ├── Transformation: [0, 0, -0.3] [0, 0, 0]
            └── Frame: lower_arm_left
```

In this example:
- The `torso` frame is positioned at the world origin
- The `upper_arm_left` frame is positioned at [0.1, 0.2, 0.5] relative to the `torso` frame
- The `lower_arm_left` frame is positioned at [0, 0, -0.3] relative to the `upper_arm_left` frame

### Working with Frames in ROS

When working with URDF models in ROS, you can use the TF2 library to query transformations between any two frames in the robot:

```python
import rclpy
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class FrameTransformer:
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self, target_frame, source_frame):
        """Get transformation from source_frame to target_frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            return transform
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return None
```

### Coordinate System Considerations for Humanoid Robots

When designing URDF models for humanoid robots, consider these coordinate system principles:

1. **Consistency**: Use consistent coordinate frame orientations across all links
2. **Anatomical alignment**: Align frames with natural body axes when possible
3. **Joint axes**: Ensure joint axes align with the intended range of motion
4. **Visualization**: Consider how frames will appear in visualization tools

For example, in a humanoid torso, you might align the Z-axis with the spine, X-axis with the shoulder-to-shoulder direction, and Y-axis with the hip-to-hip direction.

## Reading URDF Files for Humanoid Robots

Reading and interpreting URDF files is a crucial skill for working with humanoid robots. This section will guide you through the process of understanding existing URDF models and extracting useful information from them.

### Understanding the Robot Structure

When reading a URDF file, start by identifying the main components:

1. **Robot name**: Check the `<robot name="...">` tag to understand what robot model you're working with
2. **Links**: Look for all `<link>` elements to identify the robot's rigid bodies
3. **Joints**: Find all `<joint>` elements to understand how links are connected
4. **Materials**: Note any `<material>` definitions for visualization

### Example: Analyzing a Humanoid URDF

Let's look at a portion of a typical humanoid robot URDF and how to read it:

```xml
<robot name="simple_humanoid">
  <!-- Torso (base link) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin_color">
        <color rgba="0.8 0.6 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left upper arm -->
  <link name="upper_arm_left">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm_left"/>
    <origin xyz="0.2 0.15 0.7" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>
</robot>
```

### Reading This URDF Step by Step:

1. **Robot Identification**: The robot is named "simple_humanoid"

2. **Link Analysis**:
   - `torso`: The main body, with dimensions 0.3m x 0.2m x 1.0m, positioned with its center at (0, 0, 0.5) relative to its own frame
   - `head`: A spherical head with 0.1m radius
   - `upper_arm_left`: A cylindrical left upper arm with 0.05m radius and 0.3m length

3. **Joint Analysis**:
   - `neck_joint`: Connects torso to head, allowing rotation around the Y-axis (pitch movement), positioned at (0, 0, 1.0) relative to the torso frame
   - `left_shoulder_joint`: Connects torso to left upper arm, allowing rotation around the X-axis (shoulder pitch), positioned at (0.2, 0.15, 0.7) relative to the torso frame

4. **Physical Properties**:
   - The torso has a mass of 10.0 kg
   - The head has a mass of 2.0 kg
   - The left upper arm has a mass of 1.0 kg

### Tools for Reading URDF Files

Several ROS tools can help you visualize and understand URDF files:

#### 1. Using `check_urdf`
```bash
# Check if a URDF file is valid
check_urdf /path/to/robot.urdf
```

This command will:
- Validate the XML syntax
- Display the kinematic tree
- List all links and joints
- Show joint properties

#### 2. Using `rviz2` for Visualization
```bash
# Launch RViz2 with robot state publisher
ros2 run rviz2 rviz2
```

In RViz2, you can:
- Add a RobotModel display
- Set the robot description to your URDF file
- Visualize the robot in 3D
- See the coordinate frames

#### 3. Using `robot_state_publisher`
```bash
# Publish robot state for visualization
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'
```

### Programmatic Reading of URDF

You can also read and analyze URDF files programmatically using Python:

```python
import xml.etree.ElementTree as ET

def analyze_urdf(urdf_path):
    """Analyze a URDF file and extract key information"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    print(f"Robot name: {root.attrib['name']}")

    # Count links and joints
    links = root.findall('link')
    joints = root.findall('joint')

    print(f"Number of links: {len(links)}")
    print(f"Number of joints: {len(joints)}")

    # Extract link information
    print("\nLinks:")
    for link in links:
        name = link.attrib['name']
        print(f"  - {name}")

        # Check for visual, collision, and inertial elements
        if link.find('visual') is not None:
            print(f"    Visual: Present")
        if link.find('collision') is not None:
            print(f"    Collision: Present")
        if link.find('inertial') is not None:
            print(f"    Inertial: Present")

    # Extract joint information
    print("\nJoints:")
    for joint in joints:
        name = joint.attrib['name']
        joint_type = joint.attrib['type']
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']

        print(f"  - {name}: {parent} -> {child} ({joint_type})")

        # Check for limits if they exist
        limit = joint.find('limit')
        if limit is not None:
            lower = limit.attrib.get('lower', 'N/A')
            upper = limit.attrib.get('upper', 'N/A')
            print(f"    Limits: {lower} to {upper}")

# Usage
# analyze_urdf('path/to/your/humanoid.urdf')
```

## Modifying URDF for Humanoid Robots

Modifying URDF files is an important skill for customizing humanoid robots to meet specific requirements. This section covers common modifications and best practices for making changes to existing URDF models.

### Common URDF Modifications

#### 1. Changing Physical Dimensions

To modify the physical dimensions of a link, update the geometry section:

**Before:**
```xml
<link name="upper_arm_left">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
</link>
```

**After (longer and thicker arm):**
```xml
<link name="upper_arm_left">
  <visual>
    <geometry>
      <cylinder radius="0.07" length="0.4"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.07" length="0.4"/>
    </geometry>
  </collision>
</link>
```

#### 2. Adjusting Joint Limits

Modify the `<limit>` element within a joint to change the range of motion:

**Before:**
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_left"/>
  <child link="lower_arm_left"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
</joint>
```

**After (wider range of motion):**
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_left"/>
  <child link="lower_arm_left"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="50" velocity="2"/>
</joint>
```

#### 3. Adding New Links and Joints

To add a new component like a hand to the arm:

```xml
<!-- Add the hand link -->
<link name="hand_left">
  <visual>
    <geometry>
      <box size="0.1 0.08 0.15"/>
    </geometry>
    <material name="skin_color">
      <color rgba="0.8 0.6 0.4 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.08 0.15"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.3"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<!-- Connect the hand to the lower arm -->
<joint name="wrist_joint" type="revolute">
  <parent link="lower_arm_left"/>
  <child link="hand_left"/>
  <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
</joint>
```

#### 4. Updating Inertial Properties

When changing dimensions or adding components, update the mass and inertia values:

```xml
<!-- After adding the hand, update the lower arm's properties -->
<link name="lower_arm_left">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </collision>
  <!-- Updated mass to account for the attached hand -->
  <inertial>
    <mass value="0.8"/>  <!-- Increased from 0.5 -->
    <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
  </inertial>
</link>
```

### Best Practices for URDF Modification

#### 1. Backup Original Files
Always keep a backup of the original URDF file before making modifications:

```bash
cp original_robot.urdf original_robot.urdf.bak
```

#### 2. Validate After Each Change
Use the `check_urdf` tool after making modifications:

```bash
check_urdf modified_robot.urdf
```

#### 3. Update Related Files
When modifying URDF files, also update:
- Launch files that reference the URDF
- Configuration files for controllers
- Any scripts that depend on specific link or joint names

#### 4. Maintain Consistent Naming
Follow the same naming conventions throughout your URDF:

```xml
<!-- If you have left_arm_joint, name the right one right_arm_joint -->
<joint name="left_arm_joint" type="revolute">...</joint>
<joint name="right_arm_joint" type="revolute">...</joint>
```

### Advanced Modifications

#### Using Xacro for Parameterization

For more complex modifications, consider using Xacro (XML Macros), which allows parameterization:

**robot.xacro:**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Define parameters -->
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.05" />
  <xacro:property name="arm_mass" value="1.0" />

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side parent_link position">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="${arm_radius}" length="${arm_length}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${arm_radius}" length="${arm_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${arm_mass}"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${position}" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:arm side="left" parent_link="torso" position="0.2 0.15 0.7"/>
  <xacro:arm side="right" parent_link="torso" position="0.2 -0.15 0.7"/>

</robot>
```

This approach allows you to easily modify parameters like arm length by changing a single value.

#### Adding Transmission Elements

For simulation and real robot control, you may need to add transmission elements:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Testing Modified URDF Files

After making modifications, test your URDF:

1. **Validate the syntax:**
   ```bash
   check_urdf your_modified_robot.urdf
   ```

2. **Visualize in RViz:**
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat your_modified_robot.urdf)'
   ros2 run rviz2 rviz2
   ```

3. **Check kinematics:**
   ```bash
   ros2 run tf2_tools view_frames
   ```

## Practical Examples of Humanoid URDF Files

In this section, we'll look at practical examples of complete humanoid URDF files that demonstrate real-world applications. These examples will help you understand how all the concepts we've covered come together in a complete robot model.

### Example 1: Simple Humanoid Robot

Here's a complete example of a basic humanoid robot with torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Torso (base link) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="upper_arm_left">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="lower_arm_left">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="hand_left">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm_left"/>
    <origin xyz="0.2 0.15 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Left Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="upper_arm_left"/>
    <child link="lower_arm_left"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="15" velocity="1"/>
  </joint>

  <!-- Left Wrist Joint -->
  <joint name="left_wrist_joint" type="revolute">
    <parent link="lower_arm_left"/>
    <child link="hand_left"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left, mirrored) -->
  <link name="upper_arm_right">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="lower_arm_right">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="hand_right">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Shoulder Joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm_right"/>
    <origin xyz="0.2 -0.15 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Right Elbow Joint -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="upper_arm_right"/>
    <child link="lower_arm_right"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="15" velocity="1"/>
  </joint>

  <!-- Right Wrist Joint -->
  <joint name="right_wrist_joint" type="revolute">
    <parent link="lower_arm_right"/>
    <child link="hand_right"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="upper_leg_left">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="lower_leg_left">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="foot_left">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Hip Joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_leg_left"/>
    <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="1"/>
  </joint>

  <!-- Left Knee Joint -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="upper_leg_left"/>
    <child link="lower_leg_left"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="30" velocity="1"/>
  </joint>

  <!-- Left Ankle Joint -->
  <joint name="left_ankle_joint" type="revolute">
    <parent link="lower_leg_left"/>
    <child link="foot_left"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

  <!-- Right Leg (similar to left, mirrored) -->
  <link name="upper_leg_right">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="lower_leg_right">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="foot_right">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Hip Joint -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_leg_right"/>
    <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="30" velocity="1"/>
  </joint>

  <!-- Right Knee Joint -->
  <joint name="right_knee_joint" type="revolute">
    <parent link="upper_leg_right"/>
    <child link="lower_leg_right"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="30" velocity="1"/>
  </joint>

  <!-- Right Ankle Joint -->
  <joint name="right_ankle_joint" type="revolute">
    <parent link="lower_leg_right"/>
    <child link="foot_right"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>

</robot>
```

### Example 2: Xacro-Based Humanoid (More Maintainable)

Here's the same robot using Xacro for better maintainability:

```xml
<?xml version="1.0"?>
<robot name="xacro_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Material definitions -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <!-- Macro for creating a limb -->
  <xacro:macro name="limb" params="side parent_link position shoulder_axis elbow_axis">
    <!-- Upper part -->
    <link name="upper_${side}">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Lower part -->
    <link name="lower_${side}">
      <visual>
        <geometry>
          <cylinder radius="0.04" length="0.25"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.04" length="0.25"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <origin xyz="0 0 -0.125" rpy="0 0 0"/>
        <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- End effector -->
    <link name="${side}">
      <visual>
        <geometry>
          <box size="0.1 0.08 0.15"/>
        </geometry>
        <material name="orange"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.1 0.08 0.15"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.3"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Joints -->
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="upper_${side}"/>
      <origin xyz="${position}" rpy="0 0 0"/>
      <axis xyz="${shoulder_axis}"/>
      <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
    </joint>

    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="upper_${side}"/>
      <child link="lower_${side}"/>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <axis xyz="${elbow_axis}"/>
      <limit lower="0" upper="2.0" effort="15" velocity="1"/>
    </joint>

    <joint name="${side}_wrist_joint" type="revolute">
      <parent link="lower_${side}"/>
      <child link="${side}"/>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Create arms using macro -->
  <xacro:limb side="arm_left"
              parent_link="torso"
              position="0.2 0.15 0.7"
              shoulder_axis="0 1 0"
              elbow_axis="0 1 0"/>

  <xacro:limb side="arm_right"
              parent_link="torso"
              position="0.2 -0.15 0.7"
              shoulder_axis="0 1 0"
              elbow_axis="0 1 0"/>

  <!-- Macro for creating a leg -->
  <xacro:macro name="leg" params="side parent_link position hip_axis knee_axis">
    <!-- Upper part -->
    <link name="upper_leg_${side}">
      <visual>
        <geometry>
          <cylinder radius="0.06" length="0.4"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.06" length="0.4"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="2.0"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>

    <!-- Lower part -->
    <link name="lower_leg_${side}">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.4"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.4"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Foot -->
    <link name="foot_${side}">
      <visual>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <!-- Joints -->
    <joint name="${side}_hip_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="upper_leg_${side}"/>
      <origin xyz="${position}" rpy="0 0 0"/>
      <axis xyz="${hip_axis}"/>
      <limit lower="-0.5" upper="0.5" effort="30" velocity="1"/>
    </joint>

    <joint name="${side}_knee_joint" type="revolute">
      <parent link="upper_leg_${side}"/>
      <child link="lower_leg_${side}"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="${knee_axis}"/>
      <limit lower="0" upper="2.0" effort="30" velocity="1"/>
    </joint>

    <joint name="${side}_ankle_joint" type="revolute">
      <parent link="lower_leg_${side}"/>
      <child link="foot_${side}"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Create legs using macro -->
  <xacro:leg side="left"
             parent_link="torso"
             position="0 0.1 -0.1"
             hip_axis="0 1 0"
             knee_axis="0 1 0"/>

  <xacro:leg side="right"
             parent_link="torso"
             position="0 -0.1 -0.1"
             hip_axis="0 1 0"
             knee_axis="0 1 0"/>

</robot>
```

These examples demonstrate:
- Complete humanoid structure with torso, head, arms, and legs
- Proper use of visual, collision, and inertial properties
- Consistent naming conventions
- Appropriate joint types and limits
- Use of macros in Xacro for code reuse and maintainability

## Exercises for Identifying Links and Joints in URDF Files

This section provides exercises to help you practice identifying and understanding the components of URDF files for humanoid robots.

### Exercise 1: Link Identification

Consider the following URDF snippet:

```xml
<link name="upper_arm_left">
  <visual>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <material name="grey">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
  </inertial>
</link>
```

**Questions:**
1. What is the name of this link?
2. What shape is used for the visual representation?
3. What are the dimensions of this link?
4. What is the mass of this link?
5. Which body part does this link likely represent in a humanoid robot?

### Exercise 2: Joint Analysis

Analyze this joint definition:

```xml
<joint name="left_elbow_joint" type="revolute">
  <parent link="upper_arm_left"/>
  <child link="lower_arm_left"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.0" effort="15" velocity="1"/>
</joint>
```

**Questions:**
1. What type of joint is this?
2. What are the parent and child links?
3. What is the axis of rotation?
4. What is the range of motion for this joint?
5. What human joint does this represent?

### Exercise 3: Kinematic Chain Recognition

Consider this sequence of URDF elements:

```xml
<link name="torso">...</link>
<link name="upper_arm_left">...</link>
<link name="lower_arm_left">...</link>
<link name="hand_left">...</link>

<joint name="left_shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm_left"/>
  ...
</joint>

<joint name="left_elbow_joint" type="revolute">
  <parent link="upper_arm_left"/>
  <child link="lower_arm_left"/>
  ...
</joint>

<joint name="left_wrist_joint" type="revolute">
  <parent link="lower_arm_left"/>
  <child link="hand_left"/>
  ...
</joint>
```

**Questions:**
1. Draw the kinematic chain represented by these links and joints.
2. What part of the humanoid body does this represent?
3. How many degrees of freedom does this chain have?
4. What is the base link of this chain?
5. What is the end-effector link of this chain?

### Exercise 4: Complete Robot Analysis

Using the complete humanoid URDF examples from the previous section:

1. **Count the total number of links** in the robot model.
2. **Count the total number of joints** in the robot model.
3. **Identify the base link** of the robot (the root of the kinematic tree).
4. **List all the different joint types** used in the model.
5. **Find the heaviest link** in the robot and identify which body part it represents.
6. **Identify the joint with the largest range of motion**.
7. **Trace the kinematic chain** from the torso to the left hand.
8. **Trace the kinematic chain** from the torso to the right foot.

### Exercise 5: URDF Debugging

The following URDF snippet contains several errors. Identify and correct them:

```xml
<robot name="debug_robot">
  <link name="link1">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>

  <link name="link2">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="link1"/>
    <child link="link3"/>  <!-- Error? -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <joint name="joint2" type="prismatic">
    <parent link="link_unknown"/>  <!-- Error? -->
    <child link="link2"/>
  </joint>
</robot>
```

**Questions:**
1. Identify the errors in this URDF.
2. Explain why these errors would cause problems.
3. Correct the errors.

### Exercise 6: Frame and Coordinate System Analysis

Consider a humanoid robot with the following torso link positioned at the world origin:

```xml
<link name="torso">
  <visual>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.3 1.0"/>
    </geometry>
  </visual>
</link>

<joint name="neck_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 1.0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

**Questions:**
1. Where is the origin of the torso frame located relative to the box geometry?
2. Where is the neck joint located in the torso's coordinate system?
3. If the neck joint rotates 0.5 radians around the Y-axis, how would that affect the head's orientation?
4. What direction does the X-axis point in the torso's frame?
5. What direction does the Z-axis point in the head's frame when the neck joint angle is 0?

### Exercise 7: Modification Challenge

Take the simple humanoid URDF from the examples and:

1. **Add a new link**: Add a camera link to the head with appropriate visual and inertial properties.
2. **Add a new joint**: Connect the camera to the head with a fixed joint positioned appropriately.
3. **Modify existing joint limits**: Increase the range of motion for one of the arm joints.
4. **Change a link dimension**: Modify the size of one of the arm links.
5. **Verify your changes**: Describe how you would validate that your modifications are correct.

### Answers to Selected Exercises

**Exercise 1:**
1. `upper_arm_left`
2. Cylinder
3. Radius 0.05m, length 0.3m
4. 1.0 kg
5. Left upper arm of a humanoid robot

**Exercise 2:**
1. Revolute joint
2. Parent: `upper_arm_left`, Child: `lower_arm_left`
3. Y-axis (0 1 0)
4. From 0 to 2.0 radians (0° to ~114°)
5. Left elbow joint

**Exercise 3:**
1. Torso → left_shoulder_joint → upper_arm_left → left_elbow_joint → lower_arm_left → left_wrist_joint → hand_left
2. Left arm of a humanoid robot
3. Three degrees of freedom (one per joint)
4. torso
5. hand_left

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF files
- Identify links, joints, and frames in URDF for humanoid robots
- Read and interpret existing URDF files
- Create and modify URDF files for humanoid robots
- Understand the relationship between URDF and robot kinematics