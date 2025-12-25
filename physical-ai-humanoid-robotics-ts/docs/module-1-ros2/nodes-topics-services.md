---
sidebar_label: Nodes, Topics, Services, and Actions
---

# Nodes, Topics, Services, and Actions: The Communication Primitives

In this section, we'll dive deep into the fundamental communication mechanisms that enable Physical AI systems to interact with the physical world through humanoid robots. These primitives form the backbone of all robot communication in ROS 2.

## Learning Objectives

By the end of this section, you will:
- Understand the four main communication patterns in ROS 2
- Know when to use each communication type for Physical AI applications
- Be able to implement nodes that communicate using topics, services, and actions
- Understand the timing and reliability characteristics of each communication type
- Apply these concepts to humanoid robot control and perception

## Nodes: The Building Blocks of Robot Systems

A node is a process that performs computation in ROS 2. In the context of Physical AI and humanoid robots, nodes represent individual components of the robot system that can communicate with each other.

### Node Characteristics

- **Process-based**: Each node runs as a separate process
- **Single-threaded by default**: Nodes process callbacks sequentially
- **Namespaced**: Nodes have unique names within the ROS graph
- **Communicative**: Nodes interact with other nodes through topics, services, and actions

For humanoid robots, you might have nodes for:
- Joint controllers
- Sensor data processing
- Path planning
- Vision processing
- Speech recognition
- Behavior management

### Creating a Node

Here's a basic example of a node in Python:

```python
import rclpy
from rclpy.node import Node

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.get_logger().info('Joint Controller Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = JointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publish-Subscribe Communication

Topics enable asynchronous, one-way communication between nodes using a publish-subscribe pattern. This is the most common communication pattern in ROS 2 and is essential for Physical AI systems.

### Topic Characteristics

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Many-to-many**: Multiple publishers can publish to a topic, multiple subscribers can subscribe
- **Real-time friendly**: Low-latency communication suitable for sensor data and control commands
- **Typed**: All messages on a topic must be of the same type

### When to Use Topics

Topics are ideal for:
- **Sensor data**: Camera images, LiDAR scans, IMU readings
- **Control commands**: Joint positions, velocities, efforts
- **State information**: Robot pose, battery level, system status
- **Broadcasting**: Information that multiple nodes need simultaneously

### Topic Example for Humanoid Robots

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [0.0, 0.1, -0.05]  # Example positions
        self.publisher.publish(msg)
```

### Quality of Service for Topics

For Physical AI applications, QoS settings are crucial:
- **Joint control**: Reliable delivery with low latency
- **Sensor data**: Best effort for high-frequency data like cameras
- **State information**: Transient local for late-joining nodes

## Services: Request-Response Communication

Services provide synchronous, bidirectional communication with request-response semantics. This is useful when you need to guarantee that a specific request has been processed and receive a response.

### Service Characteristics

- **Synchronous**: The client waits for a response
- **Request-response**: One request, one response pattern
- **Blocking**: The client blocks until the response is received
- **Reliable**: Requests and responses are guaranteed to be delivered

### When to Use Services

Services are appropriate for:
- **Configuration**: Setting robot parameters
- **Calibration**: Requesting sensor calibration
- **One-time commands**: Execute a specific action and wait for completion
- **Query operations**: Request specific information with a response

### Service Example for Humanoid Robots

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class RobotEnableService(Node):
    def __init__(self):
        super().__init__('robot_enable_service')
        self.srv = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_robot_callback
        )
        self.robot_enabled = False

    def enable_robot_callback(self, request, response):
        self.robot_enabled = request.data
        response.success = True
        response.message = f'Robot enabled: {self.robot_enabled}'
        return response
```

## Actions: Goal-Oriented Communication

Actions provide a more complex communication pattern for long-running tasks that require feedback and status updates. This is particularly important for Physical AI systems that need to perform complex, multi-step tasks.

### Action Characteristics

- **Goal-oriented**: Designed for tasks with a clear beginning and end
- **Feedback**: Continuous feedback during task execution
- **Preemption**: Goals can be canceled or replaced
- **Status tracking**: Real-time status of task execution

### When to Use Actions

Actions are ideal for:
- **Navigation**: Moving to a specific location with progress feedback
- **Manipulation**: Grasping objects with status updates
- **Complex behaviors**: Walking, dancing, or other multi-step actions
- **Trajectory execution**: Following a complex path with monitoring

### Action Example for Humanoid Robots

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('walking_action_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'walk_trajectory',
            self.execute_trajectory
        )

    def execute_trajectory(self, goal_handle):
        # Execute the walking trajectory
        feedback_msg = FollowJointTrajectory.Feedback()

        for i in range(100):  # Simulate walking steps
            if goal_handle.is_canceling():
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # Update feedback
            feedback_msg.actual.positions = [i * 0.01] * len(feedback_msg.actual.positions)
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate walking
            time.sleep(0.1)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result
```

## Communication Patterns in Physical AI Systems

### Sensor-Processing Pattern

Sensors publish data to topics, which are consumed by processing nodes:

```
[Camera Node] ──► [Image Processing Node] ──► [Object Detection Node]
[IMU Node]    ──► [State Estimation Node]
```

### Control Pattern

Planning nodes publish commands to controllers:

```
[Path Planner] ──► [Trajectory Generator] ──► [Joint Controller]
```

### Service-Based Configuration

Configuration services allow runtime parameter adjustment:

```
[GUI Node] ──► [Set Parameters Service] ──► [Controller Node]
```

## Best Practices for Physical AI Communication

### Timing Considerations

- **High-frequency sensors**: Use best-effort QoS with appropriate queue sizes
- **Control commands**: Use reliable QoS with low latency requirements
- **State information**: Consider durability settings for late-joining nodes

### Error Handling

- Implement timeouts for service calls
- Handle connection failures gracefully
- Monitor topic liveness for critical systems
- Use appropriate retry mechanisms

### Safety in Communication

For humanoid robots interacting with humans:
- Implement safety topics that can override control commands
- Use reliable communication for safety-critical systems
- Design fail-safe behaviors when communication fails

## Diagram: Communication Pattern Example
```
┌─────────────┐    Topic     ┌──────────────────┐
│ Camera Node │ ──────────►  │ Perception Node  │
└─────────────┘              └──────────────────┘
                                    │
                              Topic │
                                    ▼
                          ┌──────────────────┐    Service
                          │ Behavior Planner │ ───────────► [Configuration Service]
                          └──────────────────┘
                                    │
                              Topic │
                                    ▼
                          ┌──────────────────┐
                          │ Motion Planner   │
                          └──────────────────┘
                                    │
                              Action│
                                    ▼
                          ┌──────────────────┐
                          │ Controller Node  │
                          └──────────────────┘
                                    │
                              Topic │
                                    ▼
                         ┌──────────────────┐
                         │ Hardware Drivers │
                         └──────────────────┘
```

## Key Takeaways

- Nodes are the basic computational units in ROS 2
- Topics provide asynchronous, many-to-many communication
- Services offer synchronous request-response communication
- Actions enable goal-oriented communication with feedback
- Choose the right communication pattern based on timing and reliability needs
- Consider safety implications when designing communication patterns

## Exercises

1. Create a simple publisher and subscriber for joint state messages
2. Implement a service that sets a robot's operational mode
3. Design an action server for a simple humanoid walking behavior

---

*Next: [URDF for Humanoids](./urdf-humanoids.md)*