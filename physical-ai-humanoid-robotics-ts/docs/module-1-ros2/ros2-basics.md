---
sidebar_label: ROS 2 Basics
---

# ROS 2 Basics: The Foundation of Robot Communication

In this section, we'll explore the fundamental concepts that make ROS 2 the backbone of modern robotics. Understanding these basics is crucial for developing Physical AI systems that can interact with the physical world through humanoid robots.

## Learning Objectives

By the end of this section, you will:
- Understand the core architecture of ROS 2
- Know how to work with packages, nodes, and the ROS 2 workspace
- Be familiar with the main ROS 2 command-line tools
- Understand Quality of Service (QoS) settings and their importance
- Know how to set up a basic ROS 2 development environment

## The ROS 2 Architecture

ROS 2 is built on a distributed system architecture that enables multiple processes (nodes) to communicate with each other through a publish-subscribe model. This architecture is essential for Physical AI systems that need to process sensor data, make decisions, and control actuators in real-time.

### DDS: The Communication Layer

At its core, ROS 2 uses Data Distribution Service (DDS) as its communication middleware. DDS provides:
- **Reliable message delivery** between nodes
- **Quality of Service (QoS) policies** that define how messages are handled
- **Discovery mechanisms** that allow nodes to find each other automatically
- **Language and platform independence** for maximum flexibility

For Physical AI systems, DDS is crucial because it ensures that sensor data reaches processing nodes reliably and that control commands are delivered with appropriate timing guarantees.

### The Client Library

ROS 2 provides client libraries (like rclcpp for C++ and rclpy for Python) that wrap the DDS functionality with a more user-friendly interface. These libraries handle the complexity of DDS while providing the essential ROS 2 concepts.

## ROS 2 Packages

In ROS 2, code is organized into packages, which are the basic units of organization. A package typically contains:
- Source code (C++ or Python)
- Configuration files
- Launch files
- Documentation
- Dependencies

For humanoid robots, you might have packages for:
- Joint control
- Sensor processing
- Path planning
- Human-robot interaction
- AI perception

### Package Structure

A typical ROS 2 package includes:
```
package_name/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata and dependencies
├── src/                    # Source code files
├── include/package_name/   # Header files (for C++)
├── launch/                 # Launch files for starting multiple nodes
├── config/                 # Configuration files
└── test/                   # Unit tests
```

## Working with the ROS 2 Workspace

A ROS 2 workspace is a directory that contains multiple packages. The typical structure is:

```
workspace_name/
├── src/                    # Source code packages
├── build/                  # Build artifacts
├── install/                # Installed packages
└── log/                    # Log files
```

### Creating and Building a Workspace

To create a new ROS 2 workspace:

```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
colcon build
source install/setup.bash
```

The `colcon build` command compiles all packages in the `src` directory, and sourcing the setup file makes the packages available in your current terminal session.

## Essential ROS 2 Commands

ROS 2 provides a rich set of command-line tools for managing and debugging your robot systems:

### ros2 run
Run a specific executable from a package:
```bash
ros2 run package_name executable_name
```

### ros2 node
List and inspect running nodes:
```bash
ros2 node list          # List all active nodes
ros2 node info <node>   # Get detailed information about a node
```

### ros2 topic
Inspect and interact with topics:
```bash
ros2 topic list         # List all active topics
ros2 topic echo <topic> # Print messages from a topic
ros2 topic pub <topic> <type> <data> # Publish data to a topic
```

### ros2 service
Work with services:
```bash
ros2 service list       # List all active services
ros2 service call <service> <type> <request> # Call a service
```

## Quality of Service (QoS) in ROS 2

QoS settings are crucial for Physical AI systems because they determine how messages are handled in terms of reliability, durability, and performance. For humanoid robots, the right QoS settings can mean the difference between stable operation and dangerous failures.

### Reliability Policy
- **Reliable**: All messages are guaranteed to be delivered (important for critical control commands)
- **Best effort**: Messages may be dropped (acceptable for high-frequency sensor data)

### Durability Policy
- **Transient local**: Late-joining subscribers receive the most recent message
- **Volatile**: Messages are only delivered to currently active subscribers

For humanoid robots, you might use reliable delivery for joint position commands but best-effort for high-frequency camera images.

## Launch Files

Launch files allow you to start multiple nodes with a single command, which is essential for complex humanoid robots with many subsystems. Launch files are typically written in Python:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_controller',
            executable='controller_node',
            name='joint_controller'
        ),
        Node(
            package='sensor_processor',
            executable='camera_node',
            name='camera_processor'
        )
    ])
```

## ROS 2 and Physical AI Integration

For Physical AI systems, ROS 2 serves as the integration layer between:
- AI perception algorithms and sensor data
- Decision-making systems and actuator control
- Simulation environments and real robots
- Human interaction systems and robot behavior

The modular architecture of ROS 2 allows different teams to work on different aspects of Physical AI while maintaining compatibility and communication between components.

## Diagram: ROS 2 Architecture Overview
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Perception    │    │ Decision-Making │    │   Actuation     │
│     Node        │◄──►│     Node        │◄──►│     Node        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         ▲                       ▲                       ▲
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │   DDS Layer     │
                    │ (Communication) │
                    └─────────────────┘
```

## Key Takeaways

- ROS 2 provides the communication infrastructure for robot systems
- The distributed architecture enables modularity and scalability
- Quality of Service settings are crucial for Physical AI reliability
- Packages organize code into manageable units
- Launch files simplify the startup of complex robot systems

## Exercises

1. Create a simple ROS 2 workspace and build it
2. Use `ros2 topic list` to see what topics are available on your system
3. Research the different QoS policies and consider when you might use each one for a humanoid robot

---

*Next: [Nodes, Topics, Services, and Actions](./nodes-topics-services.md)*