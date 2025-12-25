---
sidebar_label: Gazebo Simulation
---

# Gazebo Simulation: Physics-Based Robot Simulation

In this section, we'll explore Gazebo, the premier physics-based simulation environment for robotics. Gazebo is essential for Physical AI development as it provides realistic physics simulation that closely matches real-world robot behavior.

## Learning Objectives

By the end of this section, you will:
- Understand Gazebo's architecture and core components
- Know how to create and configure simulation environments for humanoid robots
- Be able to launch and control robots in Gazebo
- Understand physics parameters and their impact on simulation accuracy
- Learn how to integrate Gazebo with ROS 2 for Physical AI development

## Introduction to Gazebo

Gazebo is a 3D dynamic simulator that provides realistic physics simulation for robotics applications. It's widely used in the robotics community for:
- Robot development and testing
- Algorithm validation
- Training of AI systems
- Competition preparation

For Physical AI and humanoid robots, Gazebo is particularly valuable because it:
- Models complex physics interactions
- Simulates multiple sensors accurately
- Handles contact dynamics for balance and manipulation
- Integrates seamlessly with ROS 2

## Gazebo Architecture

Gazebo's architecture consists of several key components:

### Physics Engine
Gazebo supports multiple physics engines:
- **ODE (Open Dynamics Engine)**: Default, good for general simulation
- **Bullet**: Fast, suitable for real-time applications
- **Simbody**: Accurate for complex articulated systems
- **DART**: Advanced contact modeling

For humanoid robots, ODE and DART are often preferred due to their handling of complex kinematic chains.

### Rendering Engine
Gazebo uses OGRE for 3D rendering, providing:
- Realistic visual simulation
- Multiple camera viewpoints
- Lighting and material effects
- Real-time visualization

### Sensor Simulation
Gazebo includes realistic models for various sensors:
- Cameras (RGB, depth, stereo)
- LiDAR and 2D/3D laser scanners
- IMU (Inertial Measurement Units)
- Force/Torque sensors
- GPS and magnetometers

## Setting Up Gazebo with ROS 2

### Installation
Gazebo comes with ROS 2 distributions, but you may need to install additional packages:
```bash
sudo apt install ros-<distro>-gazebo-ros-pkgs
sudo apt install ros-<distro>-gazebo-plugins
```

### Basic Launch
To launch Gazebo with ROS 2 integration:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

## Creating Simulation Environments

### World Files
Gazebo environments are defined in SDF (Simulation Description Format) world files:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Launching Custom Worlds
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/world.sdf
```

## Spawning Robots in Gazebo

### Robot Description Parameter
Robots are typically loaded using the robot_description parameter:

```xml
<!-- Launch file example -->
<launch>
  <arg name="model" default="my_robot"/>
  <param name="robot_description" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
  <node pkg="gazebo_ros" exec="spawn_entity.py" args="-topic robot_description -entity my_robot"/>
</launch>
```

### Gazebo Plugins
To make robots functional in simulation, you need Gazebo plugins:

```xml
<!-- Joint state publisher plugin -->
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <topic_name>joint_states</topic_name>
    <update_rate>30</update_rate>
  </plugin>
</gazebo>

<!-- Joint position controller -->
<gazebo>
  <plugin name="joint_position_controller" filename="libgazebo_ros_joint_position.so">
    <robot_namespace>/my_robot</robot_namespace>
    <command_topic>position_commands</command_topic>
    <state_topic>joint_states</state_topic>
  </plugin>
</gazebo>
```

## Physics Parameters for Humanoid Robots

### Contact Parameters
For humanoid robots, proper contact modeling is crucial for balance:

```xml
<gazebo reference="foot_link">
  <collision>
    <surface>
      <contact>
        <ode>
          <mu>1.0</mu>  <!-- Friction coefficient -->
          <mu2>1.0</mu2>
          <kp>1e+10</kp>  <!-- Contact stiffness -->
          <kd>1e+06</kd>  <!-- Contact damping -->
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

### Inertial Parameters
Accurate inertial properties are essential:
- Mass values should match real robot
- Center of mass should be correctly positioned
- Moments of inertia should be realistic

## Sensor Simulation in Gazebo

### Camera Simulation
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
      <topic_name>image_raw</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Simulation
```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
    </imu>
  </sensor>
</gazebo>
```

## Control Systems in Gazebo

### PID Controllers
For accurate simulation, tune PID controllers:

```yaml
# Controller configuration
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: joint_trajectory_controller/JointTrajectoryController

position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
    gains:
      joint1: {p: 100.0, i: 0.01, d: 10.0}
      joint2: {p: 100.0, i: 0.01, d: 10.0}
      joint3: {p: 100.0, i: 0.01, d: 10.0}
```

## Gazebo and Physical AI Integration

### Perception Pipeline
Gazebo enables testing of perception pipelines:
- Camera data for object detection
- LiDAR for mapping and navigation
- IMU for state estimation
- Force sensors for contact detection

### Behavior Testing
- Test walking gaits in various environments
- Validate manipulation skills
- Evaluate human-robot interaction scenarios
- Assess navigation algorithms

### Reinforcement Learning
Gazebo provides the environment for training RL agents:
- Physics-accurate simulation
- Fast execution for training
- Safe exploration of behaviors
- Transfer to real robots

## Best Practices for Humanoid Robot Simulation

### Model Accuracy
- Use accurate URDF models with proper masses and inertias
- Include realistic joint limits and friction
- Model flexible components appropriately

### Simulation Parameters
- Choose appropriate physics engine for your robot
- Tune contact parameters for stable simulation
- Use appropriate update rates for control systems

### Validation
- Compare simulation results with real robot data
- Validate sensor models against real sensors
- Test controllers on both simulated and real robots

## Troubleshooting Common Issues

### Simulation Instability
- Check mass and inertia values
- Adjust contact parameters
- Verify joint limits and friction

### Performance Issues
- Reduce model complexity if needed
- Adjust physics update rate
- Optimize sensor parameters

### Control Problems
- Verify controller parameters
- Check joint state publication
- Ensure proper ROS 2 integration

## Diagram: Gazebo Integration with ROS 2
```
┌─────────────┐    Publish    ┌──────────────────┐
│   Robot     │ ────────────► │   Gazebo        │
│ Controller  │ ◄──────────── │   Simulator     │
└─────────────┘   Subscribe   └──────────────────┘
       │                              │
       └──────────────────────────────┘
                ROS 2 Interface
```

## Key Takeaways

- Gazebo provides realistic physics simulation for robot development
- Proper physics parameters are crucial for humanoid robot simulation
- Sensor simulation enables complete perception pipeline testing
- Integration with ROS 2 allows for comprehensive Physical AI development
- Validation against real robots is essential for successful transfer

## Exercises

1. Create a simple world file with obstacles for robot navigation
2. Set up a basic humanoid robot model in Gazebo with joint controllers
3. Implement sensor simulation for a humanoid robot in Gazebo
4. Compare simulation performance with different physics engines

---

*Next: [Unity for Human-Robot Interaction](./unity-hri.md)*