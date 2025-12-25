---
sidebar_label: URDF for Humanoids
---

# URDF for Humanoids: Describing Robot Structure and Kinematics

In this section, we'll explore Unified Robot Description Format (URDF), the standard way to describe robot structure in ROS 2. For humanoid robots, URDF is essential for defining the complex kinematic chains that enable natural movement and interaction.

## Learning Objectives

By the end of this section, you will:
- Understand the structure and components of URDF files
- Know how to describe complex humanoid kinematics using URDF
- Be able to create URDF models for humanoid robots
- Understand the relationship between URDF and robot simulation
- Apply URDF concepts to Physical AI system integration

## What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical structure of a robot, including:
- Links (rigid bodies)
- Joints (connections between links)
- Visual and collision properties
- Inertial properties
- Sensors and actuators

For humanoid robots, URDF is crucial because it enables:
- Kinematic analysis for movement planning
- Dynamic simulation for testing
- Visualization in tools like RViz
- Integration with motion planning algorithms

## URDF Structure

A URDF file consists of several key elements:

### Links
Links represent rigid bodies in the robot. Each link has:
- Visual properties (for display)
- Collision properties (for physics simulation)
- Inertial properties (for dynamics)

### Joints
Joints connect links and define their relative motion. Joint types include:
- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint
- **Fixed**: No motion between links
- **Floating**: 6-DOF motion (rarely used)

## Basic URDF Example

Here's a simple URDF for a humanoid leg:

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Hip joint -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Thigh link -->
  <link name="thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Knee joint -->
  <joint name="knee_joint" type="revolute">
    <parent link="thigh"/>
    <child link="shank"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="1"/>
  </joint>

  <!-- Shank link -->
  <link name="shank">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## URDF for Complex Humanoid Robots

Humanoid robots have complex kinematic structures that require careful URDF design:

### Kinematic Chains
Humanoid robots typically have multiple kinematic chains:
- **Leg chains**: For locomotion and balance
- **Arm chains**: For manipulation and interaction
- **Head chain**: For vision and interaction
- **Spine**: For posture and flexibility

### Fixed vs. Moving Joints
- Use **fixed joints** for non-moving parts (torso, head)
- Use **revolute joints** for articulated parts (joints)
- Consider **continuous joints** for rotating parts (wheels)

## URDF Best Practices for Humanoids

### Naming Conventions
Use consistent naming for humanoid robots:
- `l_arm_*` for left arm components
- `r_arm_*` for right arm components
- `l_leg_*` for left leg components
- `r_leg_*` for right leg components

### Mass and Inertia Properties
Accurate mass and inertia properties are crucial for:
- Physics simulation
- Balance control
- Motion planning
- Safety considerations

### Joint Limits and Safety
Always define appropriate joint limits:
- Prevent damage to physical robots
- Ensure realistic human-like motion
- Support safety in simulation

## URDF and Physical AI Integration

### Forward Kinematics
URDF enables forward kinematics calculations to determine:
- End-effector positions
- Reachable workspace
- Collision detection

### Inverse Kinematics
URDF provides the kinematic structure needed for:
- Motion planning
- Trajectory generation
- Balance control

### Simulation Integration
URDF models are used in simulation environments:
- Gazebo for physics simulation
- RViz for visualization
- MoveIt! for motion planning

## Xacro: URDF Macros

For complex humanoid robots, Xacro (XML Macros) helps manage URDF complexity:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Macro for creating a joint -->
  <xacro:macro name="simple_joint" params="name parent child joint_type axis xyz rpy lower upper">
    <joint name="${name}" type="${joint_type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${axis}"/>
      <limit lower="${lower}" upper="${upper}" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_joint name="hip_joint"
                      parent="base_link"
                      child="thigh"
                      joint_type="revolute"
                      axis="0 1 0"
                      xyz="0 0 -0.05"
                      rpy="0 0 0"
                      lower="-1.57"
                      upper="1.57"/>

</robot>
```

## URDF Tools and Validation

### Checking URDF
Use ROS tools to validate your URDF:
```bash
check_urdf /path/to/robot.urdf
```

### Visualizing URDF
Visualize your robot model:
```bash
urdf_to_graphiz /path/to/robot.urdf
```

### Debugging URDF
Common issues to check:
- Joint parent-child relationships
- Proper mass and inertia values
- Joint limits and types
- Collision and visual properties

## Diagram: Humanoid Robot URDF Structure
```
                    torso
                      |
        ┌─────────────┼─────────────┐
        |             |             |
      head         l_arm         r_arm
        |             |             |
        |          shoulder       shoulder
        |             |             |
        |          l_elbow       r_elbow
        |             |             |
        |            l_hand       r_hand
        |
    ┌───┴───┐
    |       |
  l_leg   r_leg
    |       |
  l_hip   r_hip
    |       |
  l_knee  r_knee
    |       |
  l_ankle r_ankle
    |       |
  l_foot  r_foot
```

## URDF in Physical AI Systems

URDF serves as the foundation for several Physical AI capabilities:

### Perception Integration
- Camera positions and fields of view
- Sensor mounting points
- Field of view visualization

### Motion Planning
- Collision checking
- Kinematic constraints
- Reachable workspace analysis

### Control Systems
- Joint mapping
- Kinematic chain definitions
- Balance control parameters

## Key Takeaways

- URDF describes robot structure using links and joints
- Proper URDF is essential for humanoid robot simulation and control
- Use Xacro to manage complex URDF models
- Accurate mass and inertia properties are crucial for physical simulation
- URDF enables integration with motion planning and control systems
- Follow naming conventions for consistency

## Exercises

1. Create a simple URDF model for a humanoid arm
2. Use Xacro to create a parametric humanoid leg model
3. Validate your URDF using ROS tools and visualize it

---

*Next: [Module 2: Digital Twin Introduction](../module-2-digital-twin/intro.md)*