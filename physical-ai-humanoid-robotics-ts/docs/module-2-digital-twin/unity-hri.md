---
sidebar_label: Unity for Human-Robot Interaction
---

# Unity for Human-Robot Interaction: Visual and Interaction Simulation

In this section, we'll explore Unity's role in Physical AI development, focusing on creating realistic environments for human-robot interaction simulation. Unity excels at high-fidelity visual rendering and complex interaction scenarios that complement physics-based simulation.

## Learning Objectives

By the end of this section, you will:
- Understand Unity's capabilities for robot simulation and HRI
- Know how to set up Unity for robotics applications
- Be familiar with Unity Robotics tools and packages
- Understand how to simulate human behavior in Unity
- Learn about VR/AR integration for immersive HRI testing
- Appreciate Unity's role in the Physical AI ecosystem

## Introduction to Unity for Robotics

Unity is a powerful 3D development platform that has gained significant traction in robotics for creating high-fidelity simulation environments. Unlike physics-focused simulators like Gazebo, Unity excels at:

- **Visual realism**: High-quality graphics and rendering
- **Human behavior simulation**: Realistic human models and behaviors
- **Interaction design**: Complex user interfaces and interactions
- **VR/AR capabilities**: Immersive testing environments
- **Cross-platform deployment**: Runs on multiple platforms

For Physical AI and humanoid robots, Unity provides:

- **Realistic human models**: For testing HRI scenarios
- **Complex environments**: Detailed indoor/outdoor scenes
- **Social interaction simulation**: Multi-human scenarios
- **Immersive testing**: VR environments for teleoperation

## Unity Robotics Ecosystem

### Unity Robotics Hub
The Unity Robotics Hub provides essential tools:
- **ROS#**: Bridge between Unity and ROS/ROS 2
- **Unity Perception**: Tools for synthetic data generation
- **ML-Agents**: Machine learning framework for Unity
- **XR packages**: Virtual and augmented reality support

### ROS# Integration
ROS# enables communication between Unity and ROS 2:
- Publish/subscribe to ROS topics
- Call ROS services
- Execute ROS actions
- Bridge Unity messages to ROS messages

## Setting Up Unity for Robotics

### Installation Requirements
- Unity Hub (latest LTS version)
- Unity Editor (2021.3 LTS or newer recommended)
- Unity Robotics packages
- ROS 2 installation on the same machine

### Basic Setup
1. Create a new 3D project in Unity
2. Import Unity Robotics packages via Package Manager
3. Configure ROS# connection settings
4. Set up robot models and environments

### ROS# Configuration
```csharp
// Example ROS# connection in Unity
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect("127.0.0.1", 10000); // Connect to ROS 2 bridge
    }

    void Update()
    {
        // Send robot position to ROS
        ros.Send("robot_position", new PositionMsg(transform.position));
    }
}
```

## Creating Robot Models in Unity

### Importing Robot Models
Unity supports various 3D model formats:
- **FBX**: Most common for Unity
- **OBJ**: Simple geometry import
- **URDF**: Direct import from ROS (with URDF Importer)

### Robot Animation and Control
Unity's animation system can be used for:
- Predefined robot motions
- Inverse kinematics for manipulation
- Smooth joint movement visualization

### Physics Integration
While Unity has its own physics engine:
- Use Unity physics for visual interactions
- Keep Gazebo for accurate robot physics
- Synchronize state between simulators

## Human Behavior Simulation

### Character Controllers
Unity provides robust character controllers:
- **NavMesh**: Pathfinding for human agents
- **Animation controllers**: Realistic human movement
- **Behavior trees**: Complex human decision-making

### Social Interaction Scenarios
Create realistic HRI scenarios:
- **Personal space**: Humans maintaining appropriate distance
- **Gestures**: Human body language and communication
- **Attention**: Eye contact and attention direction
- **Emotions**: Facial expressions and emotional responses

### Example Human Agent
```csharp
using UnityEngine;
using UnityEngine.AI;

public class HumanAgent : MonoBehaviour
{
    NavMeshAgent agent;
    Animator animator;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        // Example: Human moves toward a target
        if (target != null)
        {
            agent.SetDestination(target.position);

            // Play walking animation based on speed
            float speed = agent.velocity.magnitude;
            animator.SetFloat("Speed", speed);
        }
    }
}
```

## Environment Design for HRI

### Realistic Environments
Create environments that mirror real-world settings:
- **Home environments**: Living rooms, kitchens, bedrooms
- **Workplace settings**: Offices, factories, laboratories
- **Public spaces**: Stores, hospitals, airports

### Interactive Elements
Design environments with interactive elements:
- **Doors**: That open/close based on robot approach
- **Furniture**: That can be moved or navigated around
- **Objects**: That can be manipulated by robots
- **Lighting**: Dynamic lighting conditions

### Environmental Complexity
Consider various environmental factors:
- **Clutter**: Objects that affect navigation
- **Dynamic obstacles**: Moving humans and objects
- **Lighting conditions**: Different times of day
- **Weather effects**: Indoor/outdoor considerations

## VR/AR Integration for HRI

### VR for Teleoperation
Virtual reality enables:
- **Immersive teleoperation**: Direct robot control
- **Presence testing**: How humans perceive robot presence
- **Safety training**: Safe testing of dangerous scenarios
- **User experience testing**: How humans interact with robots

### AR for Mixed Reality
Augmented reality allows:
- **Robot visualization**: Overlay robot information in real world
- **Path planning visualization**: See robot planned paths
- **Sensor data overlay**: Visualize robot perception
- **Collaborative scenarios**: Humans and robots in shared space

### VR/AR Setup
Unity provides packages for VR/AR development:
- **XR Interaction Toolkit**: For VR controller interaction
- **OpenXR**: Standardized XR support
- **Oculus Integration**: For Oculus devices
- **AR Foundation**: For augmented reality

## Unity Perception Package

### Synthetic Data Generation
Unity Perception generates:
- **Labeled training data**: For AI model training
- **Sensor simulation**: Camera, LiDAR, IMU data
- **Domain randomization**: Varying visual conditions
- **Ground truth data**: Perfect annotations for training

### Perception Tools
- **Bounding Box Labeler**: Object detection training data
- **Semantic Segmentation**: Pixel-level labeling
- **Instance Segmentation**: Object instance labeling
- **Depth Sensor Simulation**: Depth camera data

### Example: Synthetic Dataset Generation
```csharp
using Unity.Perception.GroundTruth;
using Unity.Perception.Randomization;

public class DatasetGenerator : MonoBehaviour
{
    void Start()
    {
        // Configure randomization parameters
        ConfigureRandomization();

        // Start data collection
        StartDataCollection();
    }

    void ConfigureRandomization()
    {
        // Randomize lighting, textures, object positions
        // This creates diverse training data
    }

    void StartDataCollection()
    {
        // Collect synthetic sensor data with ground truth
    }
}
```

## Human-Robot Interaction Scenarios

### Service Robot Scenarios
Design scenarios for service robots:
- **Restaurant service**: Taking orders, delivering food
- **Hospital assistance**: Guiding patients, delivering supplies
- **Home assistance**: Cleaning, companionship, monitoring

### Industrial Collaboration
Scenarios for industrial robots:
- **Cobot workflows**: Human-robot collaboration
- **Safety protocols**: Emergency stop, collision avoidance
- **Task coordination**: Synchronized work patterns

### Social Robotics
Social interaction scenarios:
- **Companionship**: Elderly care, child interaction
- **Education**: Teaching and learning interactions
- **Entertainment**: Gaming, storytelling, performance

## Integration with ROS 2

### Message Types
Unity can handle various ROS message types:
- **Sensor messages**: Camera, IMU, LiDAR data
- **Control messages**: Joint commands, navigation goals
- **State messages**: Robot pose, joint states
- **Custom messages**: Application-specific data

### Bridge Configuration
Set up ROS# bridge for efficient communication:
- **Connection management**: Handle network interruptions
- **Message serialization**: Optimize data transfer
- **Timing synchronization**: Align simulation and ROS time

### Example: Camera Data Bridge
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraBridge : MonoBehaviour
{
    public Camera unityCamera;
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    void Update()
    {
        // Capture camera data and send to ROS
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = unityCamera.targetTexture;
        unityCamera.Render();

        Texture2D image = new Texture2D(unityCamera.targetTexture.width,
                                       unityCamera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, unityCamera.targetTexture.width,
                                 unityCamera.targetTexture.height), 0, 0);
        image.Apply();

        // Convert to ROS format and send
        // (Implementation details for converting Unity texture to ROS image)

        RenderTexture.active = currentRT;
    }
}
```

## Best Practices for HRI Simulation

### Realistic Human Behavior
- Model natural human movement patterns
- Include realistic reaction times
- Simulate human errors and unpredictability
- Consider cultural differences in interaction

### Performance Optimization
- Use Level of Detail (LOD) for complex scenes
- Optimize rendering for real-time performance
- Balance visual quality with simulation speed
- Consider hardware limitations

### Validation and Verification
- Compare simulation results with real-world data
- Validate human behavior models
- Test edge cases and unexpected interactions
- Ensure safety in simulated scenarios

## Challenges and Limitations

### Visual vs. Physical Fidelity
- Unity prioritizes visual quality over physical accuracy
- Use in combination with physics simulators
- Validate critical physical interactions elsewhere

### Computational Requirements
- High-fidelity graphics require powerful hardware
- VR/AR applications need high frame rates
- Balance quality with performance requirements

### Human Behavior Modeling
- Modeling realistic human behavior is complex
- Humans are unpredictable and diverse
- Requires extensive validation against real data

## Diagram: Unity in Physical AI Ecosystem
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Real Robot    │    │   Unity HRI     │    │   Gazebo        │
│                 │    │   Simulation    │    │   Physics       │
│  Sensors        │◄──►│  Visual         │    │   Simulation    │
│  Actuators      │    │  Interaction    │◄──►│  Accurate       │
│  Controllers    │    │  VR/AR          │    │  Physics        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         ▲                       ▲                       ▲
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │   ROS 2 Bridge  │
                    │   (ROS#)        │
                    └─────────────────┘
```

## Industry Applications

Unity is used in various robotics applications:
- **Toyota**: HSR robot interaction testing
- **Boston Dynamics**: Visualization and teleoperation
- **Amazon Robotics**: Warehouse interaction scenarios
- **Healthcare Robotics**: Hospital navigation and interaction

## Key Takeaways

- Unity excels at visual fidelity and human behavior simulation
- Essential for testing human-robot interaction scenarios
- VR/AR capabilities enable immersive testing
- Should be used in conjunction with physics simulators
- Critical for social robotics and service robot development

## Exercises

1. Create a simple Unity scene with a robot and human agent
2. Implement basic navigation for the human agent using NavMesh
3. Set up ROS# connection and send simple messages between Unity and ROS 2
4. Design a simple HRI scenario with Unity Perception for data generation

---

*Next: [Sensor Simulation and Validation](./sensors.md)*