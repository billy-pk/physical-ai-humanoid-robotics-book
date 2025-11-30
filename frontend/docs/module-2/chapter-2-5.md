---
sidebar_position: 6
title: 2.5 Unity Integration for Photorealistic Rendering
---

# Chapter 2.5: Unity Integration for Photorealistic Rendering

While Gazebo provides accurate physics, Unity offers photorealistic rendering with ray-traced lighting, realistic materials, and massive asset libraries. Combining both enables physics-accurate simulation with visual realism—essential for training vision-based AI models and validating humanoid robot behaviors in realistic environments.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Set up** Unity-ROS 2 bridge for bidirectional communication
- **Import** robot models into Unity with proper materials and lighting
- **Configure** ray-traced rendering for photorealistic visuals
- **Generate** synthetic training data (images, depth maps) for ML models
- **Integrate** Unity scenes with Gazebo physics simulation

## Prerequisites

- **Unity 2022.3 LTS** installed (free, download from unity.com)
- **ROS 2 Humble** configured (Module 1)
- **Gazebo** experience (Chapters 2.1 to 2.4)
- **Basic Unity** knowledge (scenes, GameObjects, materials) - optional but helpful

## Part 1: Unity-ROS 2 Integration Architecture

### Why Unity + Gazebo?

**Gazebo strengths**:
- Accurate physics simulation
- Native ROS 2 integration
- Real-time sensor models

**Unity strengths**:
- Photorealistic rendering (ray tracing, global illumination)
- Massive asset library (environments, objects, textures)
- ML training data generation
- Cross-platform (Windows, macOS, Linux)

**Together**: Physics accuracy + visual realism = production-ready simulation pipeline

### Integration Approaches

| Approach | Use Case | Complexity |
|----------|----------|------------|
| **Unity-ROS 2 Bridge** | Real-time sync, bidirectional | Medium |
| **Export from Gazebo** | One-time model transfer | Low |
| **Dual Simulation** | Physics in Gazebo, rendering in Unity | High |

**This chapter uses Unity-ROS 2 Bridge** (most flexible).

### ROS 2 Unity Packages

**Key packages**:
- **`ros2-for-unity`**: Core ROS 2 communication
- **`ros_tcp_connector`**: TCP bridge between Unity and ROS 2
- **`Unity Robotics Hub`**: Official Unity robotics tools

## Part 2: Hands-On Tutorial

### Project: Create Photorealistic Humanoid Scene in Unity

**Goal**: Import humanoid robot into Unity, set up ROS 2 communication, and generate synthetic training images.

**Tools**: Unity 2022.3 LTS, ROS 2 Humble, ROS-TCP-Connector

### Step 1: Install Unity and ROS 2 Bridge

**Install Unity 2022.3 LTS**:
1. Download from [unity.com/download](https://unity.com/download)
2. Install Unity Hub
3. Install Unity Editor 2022.3 LTS
4. Create new 3D project: "HumanoidSimulation"

**Install ROS-TCP-Connector** (Unity package):
1. In Unity: `Window → Package Manager`
2. Click `+ → Add package from git URL`
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Click `Add`

**Install ROS 2 dependencies** (Ubuntu):
```bash
# Install ROS 2 Unity bridge
sudo apt install ros-humble-rosbridge-suite

# Install ROS-TCP endpoint (Python)
pip3 install ros-tcp-endpoint
```

### Step 2: Set Up ROS 2 Connection in Unity

**Create ROS connection script**: `Assets/Scripts/ROSConnection.cs`

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using sensor_msgs.msg;

public class ROSConnection : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/humanoid/camera/image_raw";
    
    void Start()
    {
        // Connect to ROS 2
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect("localhost", 10000);  // Default ROS-TCP port
        
        Debug.Log("Connected to ROS 2");
    }
    
    void OnDestroy()
    {
        if (ros != null)
        {
            ros.Disconnect();
        }
    }
}
```

**Create ROS-TCP endpoint** (Python): `ros_tcp_endpoint.py`

```python
#!/usr/bin/env python3
"""
ROS-TCP Endpoint for Unity-ROS 2 communication
ROS 2 Humble | Python 3.10+
"""
from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist

class UnityROSBridge(Node):
    def __init__(self):
        super().__init__('unity_ros_bridge')
        self.get_logger().info('Unity ROS Bridge started')

def main():
    rclpy.init()
    node = UnityROSBridge()
    
    # Create TCP server
    tcp_server = TcpServer(node, 'localhost', 10000)
    
    # Register publishers (Unity → ROS 2)
    tcp_server.register_publisher('/humanoid/joint_states', JointState)
    
    # Register subscribers (ROS 2 → Unity)
    tcp_server.register_subscriber('/humanoid/cmd_vel', Twist)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run ROS-TCP endpoint**:
```bash
python3 ros_tcp_endpoint.py
```

### Step 3: Import Robot Model into Unity

**Option A: Export from Gazebo** (simplest):
```bash
# Export URDF as OBJ/STL (use mesh export tools)
# Or use Blender to convert URDF meshes
```

**Option B: Create in Unity** (more control):
1. Create GameObject hierarchy:
   - `Humanoid` (root)
     - `Torso`
       - `Head`
       - `LeftArm`
       - `RightArm`
       - `LeftLeg`
       - `RightLeg`

2. Add primitive shapes (Cubes, Cylinders) matching URDF dimensions

3. Add materials:
   - Create materials in `Assets/Materials/`
   - Assign colors/textures to each link

**Example Unity script** (`Assets/Scripts/RobotController.cs`):

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using sensor_msgs.msg;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;
    private GameObject torso, head, leftLeg, rightLeg;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        
        // Get robot parts
        torso = GameObject.Find("Torso");
        head = GameObject.Find("Head");
        leftLeg = GameObject.Find("LeftLeg");
        rightLeg = GameObject.Find("RightLeg");
        
        // Subscribe to joint states
        ros.Subscribe<JointState>("/humanoid/joint_states", UpdateJoints);
    }
    
    void UpdateJoints(JointState jointState)
    {
        // Update joint angles based on ROS 2 messages
        // Example: leftLeg.transform.localRotation = Quaternion.Euler(0, jointState.position[0], 0);
    }
}
```

### Step 4: Configure Photorealistic Rendering

**Enable HDRP** (High Definition Render Pipeline):
1. `Edit → Project Settings → Graphics`
2. Set `Scriptable Render Pipeline Settings` to HDRP asset
3. Create HDRP asset: `Assets → Create → Rendering → HDRP Asset`

**Configure ray tracing** (if GPU supports):
1. `Edit → Project Settings → Graphics → HDRP Global Settings`
2. Enable `Ray Tracing`
3. Set `Ray Tracing Quality` to High

**Add lighting**:
1. Create `Directional Light` (sun)
2. Set intensity: 3.0
3. Enable shadows: `Soft Shadows`
4. Add `HDRI Sky` for realistic environment

**Configure camera**:
1. Create `Camera` GameObject
2. Set position: `(0, 1.6, -2)` (humanoid eye height)
3. Set field of view: 60°
4. Enable HDR
5. Add post-processing: Bloom, Color Grading

### Step 5: Generate Synthetic Training Data

**Create data collection script**: `Assets/Scripts/DataCollector.cs`

```csharp
using UnityEngine;
using System.IO;
using System.Collections;

public class DataCollector : MonoBehaviour
{
    public Camera renderCamera;
    public string outputPath = "Assets/TrainingData/";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public int frameRate = 30;
    
    private int frameCount = 0;
    private float timer = 0f;
    
    void Start()
    {
        // Create output directory
        Directory.CreateDirectory(outputPath);
        Directory.CreateDirectory(outputPath + "rgb/");
        Directory.CreateDirectory(outputPath + "depth/");
        Directory.CreateDirectory(outputPath + "labels/");
    }
    
    void Update()
    {
        timer += Time.deltaTime;
        
        if (timer >= 1f / frameRate)
        {
            CaptureFrame();
            timer = 0f;
        }
    }
    
    void CaptureFrame()
    {
        // Render RGB image
        RenderTexture rgbRT = new RenderTexture(imageWidth, imageHeight, 24);
        renderCamera.targetTexture = rgbRT;
        renderCamera.Render();
        
        RenderTexture.active = rgbRT;
        Texture2D rgbImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        rgbImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        rgbImage.Apply();
        
        // Save RGB image
        byte[] rgbBytes = rgbImage.EncodeToPNG();
        File.WriteAllBytes($"{outputPath}rgb/frame_{frameCount:06d}.png", rgbBytes);
        
        // Render depth image (if depth camera configured)
        // Similar process for depth map
        
        frameCount++;
    }
}
```

**Use case**: Generate thousands of training images for object detection models.

### Step 6: Sync Unity with Gazebo (Optional)

**Dual simulation approach**:
1. **Gazebo**: Runs physics simulation, publishes joint states
2. **Unity**: Receives joint states, renders visually
3. **ROS 2**: Bridges communication between both

**Unity script** (subscribe to Gazebo joint states):

```csharp
using Unity.Robotics.ROSTCPConnector;
using sensor_msgs.msg;

public class GazeboSync : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointState>("/humanoid/joint_states", UpdateRobotPose);
    }
    
    void UpdateRobotPose(JointState jointState)
    {
        // Update Unity robot pose based on Gazebo joint states
        // This syncs Unity visualization with Gazebo physics
    }
}
```

### Step 7: Debugging Common Issues

#### Issue 1: Unity Can't Connect to ROS 2
**Symptoms**: Connection timeout, no messages

**Solutions**:
```bash
# Verify ROS-TCP endpoint running
ps aux | grep ros_tcp_endpoint

# Check firewall
sudo ufw allow 10000/tcp

# Verify ROS 2 running
ros2 node list
```

#### Issue 2: Robot Model Not Visible
**Symptoms**: Empty scene, no robot

**Solutions**:
- Check camera position/orientation
- Verify materials assigned
- Check lighting (add Directional Light)
- Verify GameObject hierarchy

#### Issue 3: Low Frame Rate
**Symptoms**: Laggy rendering

**Solutions**:
- Reduce render resolution
- Disable ray tracing (if enabled)
- Simplify materials/textures
- Use LOD (Level of Detail) for distant objects

## Part 3: Advanced Topics (Optional)

### Domain Randomization

**Randomize** lighting, textures, object positions for robust ML training:

```csharp
void RandomizeEnvironment()
{
    // Random lighting
    Light sun = GameObject.Find("Directional Light").GetComponent<Light>();
    sun.intensity = Random.Range(0.5f, 3.0f);
    sun.color = Random.ColorHSV();
    
    // Random object positions
    foreach (GameObject obj in sceneObjects)
    {
        obj.transform.position += Random.insideUnitSphere * 0.5f;
    }
    
    // Random textures
    // Apply random materials to objects
}
```

### ROS 2 Action Integration

**Use ROS 2 Actions** for long-running tasks:

```csharp
using Unity.Robotics.ROSTCPConnector;
using nav2_msgs.action;

public class NavigationAction : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // Send navigation goal via ROS 2 Action
    }
}
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Visual validation**: Capstone will use Unity for photorealistic testing
- **Training data**: Generate synthetic images for vision model training
- **Presentation**: Professional Unity renders for demo videos
- **Multi-modal**: Combine Gazebo physics with Unity visuals

Understanding Unity integration now enables photorealistic capstone demonstrations.

## Summary

You learned:
- ✅ Set up **Unity-ROS 2 bridge** for bidirectional communication
- ✅ Imported **robot models** into Unity with materials and lighting
- ✅ Configured **photorealistic rendering** (HDRP, ray tracing)
- ✅ Generated **synthetic training data** (RGB images, depth maps)
- ✅ Integrated **Unity with Gazebo** for dual simulation

**Next steps**: Module 3 (NVIDIA Isaac Sim) will build on Gazebo physics and Unity rendering for advanced simulation workflows.

---

## Exercises

### Exercise 1: Unity-ROS 2 Communication (Required)

**Objective**: Establish bidirectional communication between Unity and ROS 2.

**Tasks**:
1. Set up ROS-TCP-Connector in Unity
2. Create ROS connection script
3. Publish joint states from Unity to ROS 2
4. Subscribe to velocity commands in Unity from ROS 2
5. Verify communication with `ros2 topic echo/list`

**Acceptance Criteria**:
- [ ] Unity connects to ROS 2 successfully
- [ ] Joint states published to ROS 2 topic
- [ ] Velocity commands received in Unity
- [ ] No connection errors

**Estimated Time**: 90 minutes

### Exercise 2: Photorealistic Scene (Required)

**Objective**: Create realistic environment in Unity.

**Tasks**:
1. Import or create humanoid robot model
2. Add realistic materials (metal, plastic, rubber)
3. Configure HDRP lighting (directional light, sky)
4. Add environment objects (tables, chairs, objects)
5. Capture rendered images

**Acceptance Criteria**:
- [ ] Scene looks photorealistic
- [ ] Robot model visible and properly lit
- [ ] Shadows and reflections working
- [ ] Images saved successfully

**Estimated Time**: 120 minutes

### Exercise 3: Synthetic Data Generation (Challenge)

**Objective**: Generate dataset of synthetic images for ML training.

**Tasks**:
1. Create data collection script
2. Randomize camera positions/orientations
3. Randomize lighting and object positions
4. Generate 100+ images with labels
5. Export dataset in standard format (COCO, YOLO)

**Requirements**:
- 100+ RGB images
- Corresponding depth maps (optional)
- Bounding box labels (if object detection)
- Metadata file (JSON/YAML)

**Estimated Time**: 180 minutes

---

## Additional Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) - Official Unity robotics tools
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) - Unity-ROS bridge
- [Unity HDRP Documentation](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@latest/) - High Definition Render Pipeline
- [Synthetic Data Generation](https://unity.com/solutions/automotive-transportation/manufacturing/synthetic-data) - Unity synthetic data tools

---

**Next**: [Module 3: The AI-Robot Brain (NVIDIA Isaac) →](../module-3/intro.md)
