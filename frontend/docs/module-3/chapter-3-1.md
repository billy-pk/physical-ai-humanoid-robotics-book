---
sidebar_position: 2
title: 3.1 Isaac Sim Setup & Photorealistic Simulation
---

# Chapter 3.1: Isaac Sim Setup & Photorealistic Simulation

NVIDIA Isaac Sim provides photorealistic, GPU-accelerated simulation for robotics AI. Built on Omniverse and USD (Universal Scene Description), it enables training vision models, testing navigation algorithms, and validating behaviors in visually realistic environments before hardware deployment.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Install** Isaac Sim 2023.1.1 on Ubuntu 22.04 with GPU support
- **Understand** Omniverse and USD scene description format
- **Create** photorealistic scenes with ray-traced lighting and materials
- **Import** robot models and configure physics simulation
- **Launch** Isaac Sim with ROS 2 integration and verify communication

## Prerequisites

- **Ubuntu 22.04 LTS** installed (native recommended for GPU support)
- **NVIDIA GPU** with CUDA support (RTX 2060 or better, 8GB+ VRAM)
- **NVIDIA drivers** installed (version 525+)
- **CUDA Toolkit** 11.8+ installed
- **ROS 2 Humble** configured (Module 1)
- **Basic Python** knowledge (classes, imports)

## Part 1: Isaac Sim Architecture Fundamentals

### What is Isaac Sim?

**Isaac Sim** is NVIDIA's photorealistic robotics simulator that:
- Provides **ray-traced rendering** matching real-world visuals
- Accelerates **perception and VSLAM** using GPU compute
- Generates **synthetic training data** for ML models
- Integrates with **Omniverse** for collaborative workflows
- Supports **ROS 2** via Isaac ROS packages

**Why Isaac Sim for humanoid robots?**
- **Visual Realism**: Train vision models on photorealistic data
- **GPU Acceleration**: Real-time VSLAM and perception
- **Synthetic Data**: Generate millions of labeled images
- **Production Ready**: Used by NVIDIA, Boston Dynamics, Tesla

### Omniverse and USD

**Omniverse** is NVIDIA's collaboration platform:
- **USD (Universal Scene Description)**: Open-source format for 3D scenes
- **Live Sync**: Multiple users edit scenes simultaneously
- **Connectors**: Integrate with Blender, Maya, Unreal Engine
- **Extensions**: Isaac Sim runs as Omniverse extension

**USD concepts**:
- **Stage**: Root container for scene graph
- **Prims**: Scene elements (meshes, lights, cameras)
- **Layers**: Compositing system (like Photoshop layers)
- **References**: Link external USD files

### Isaac Sim vs. Gazebo

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Rendering** | Basic OpenGL | Ray-traced (RTX) |
| **Physics** | ODE/Bullet (CPU) | PhysX (GPU-accelerated) |
| **Perception** | Basic sensors | GPU-accelerated VSLAM |
| **Data Generation** | Manual scripts | Built-in pipelines |
| **Performance** | CPU-bound | GPU-accelerated |
| **Use Case** | Physics validation | AI training, visuals |

**Best practice**: Use Gazebo for physics development, Isaac Sim for AI and photorealistic testing.

## Part 2: Hands-On Tutorial

### Project: Install Isaac Sim and Create Photorealistic Scene

**Goal**: Install Isaac Sim 2023.1.1, create a photorealistic scene with humanoid robot, and verify ROS 2 integration.

**Tools**: Ubuntu 22.04, NVIDIA GPU, Isaac Sim 2023.1.1, ROS 2 Humble

### Step 1: Install NVIDIA Drivers and CUDA

**Check GPU**:
```bash
lspci | grep -i nvidia
# Should show: NVIDIA Corporation [GPU model]

# Check current driver
nvidia-smi
# Should show driver version and GPU info
```

**Install NVIDIA drivers** (if not installed):
```bash
# Add NVIDIA PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver (usually latest)
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot

# Verify after reboot
nvidia-smi
```

**Install CUDA Toolkit 11.8**:
```bash
# Download CUDA 11.8 installer
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run

# Run installer
sudo sh cuda_11.8.0_520.61.05_linux.run

# Add to PATH (add to ~/.bashrc)
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH

# Verify
nvcc --version
```

### Step 2: Install Omniverse Launcher

**Download Omniverse Launcher**:
1. Visit: [https://www.nvidia.com/en-us/omniverse/download/](https://www.nvidia.com/en-us/omniverse/download/)
2. Download Linux installer (`.AppImage`)
3. Make executable:
```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

**First-time setup**:
1. Create NVIDIA account (free)
2. Sign in to Omniverse Launcher
3. Accept terms and conditions

### Step 3: Install Isaac Sim

**Via Omniverse Launcher**:
1. Open **Exchange** tab
2. Search for **"Isaac Sim"**
3. Click **Install** (requires ~15GB disk space)
4. Select version **2023.1.1** (or latest stable)

**Installation time**: ~30 to 60 minutes (downloads ~10GB)

**Verify installation**:
```bash
# Isaac Sim installs to: ~/.nvidia-omniverse/omniverse-launcher/IsaacSim-[version]/
# Check installation
ls ~/.nvidia-omniverse/omniverse-launcher/IsaacSim-*/
```

### Step 4: Launch Isaac Sim

**Via Omniverse Launcher**:
1. Go to **Library** tab
2. Find **Isaac Sim**
3. Click **Launch**

**Via command line**:
```bash
# Find Isaac Sim executable
find ~/.nvidia-omniverse -name "isaac-sim.sh" -type f

# Launch (example path)
~/.nvidia-omniverse/omniverse-launcher/IsaacSim-2023.1.1/isaac-sim.sh
```

**Expected Result**:
- Isaac Sim window opens
- Shows default scene (empty stage)
- GPU usage visible in `nvidia-smi`

### Step 5: Create First Scene

**In Isaac Sim**:
1. **File → New** (creates empty USD stage)
2. **Add → Lights → Dome Light** (for environment lighting)
3. **Add → Geometry → Cube** (test object)
4. **Add → Camera** (for rendering)

**Save scene**:
1. **File → Save As**
2. Save as `my_first_scene.usd` in `~/isaac_sim_scenes/`

**Viewport controls**:
- **Left-click + drag**: Rotate camera
- **Middle-click + drag**: Pan camera
- **Right-click + drag**: Zoom
- **F**: Focus on selected object

### Step 6: Add Humanoid Robot Model

**Option A: Import URDF** (from Module 1):
1. **Isaac Utils → Workflows → Import Robot**
2. Select URDF file: `~/gazebo_ws/src/gazebo_worlds/models/simple_humanoid/model.urdf`
3. Click **Import**
4. Robot appears in scene

**Option B: Use Pre-built Models**:
1. **Content → Isaac → Assets → Robots**
2. Browse available robots (e.g., "Carter", "Franka")
3. Drag robot into scene

**Configure robot physics**:
1. Select robot root prim
2. **Physics → Apply Physics** → **Rigid Body**
3. Set **Mass**: 50 kg (for humanoid)
4. Set **Center of Mass**: (0, 0, 0.5)

### Step 7: Configure Photorealistic Rendering

**Enable RTX Ray Tracing**:
1. **Window → Rendering → RTX Settings**
2. Enable **Ray Tracing**
3. Set **Ray Tracing Quality**: High
4. Enable **Denoising**: On

**Configure Materials**:
1. Select robot link
2. **Material → Create Material** → **OmniPBR** (Physically Based Rendering)
3. Set properties:
   - **Base Color**: (0.2, 0.5, 1.0) - Blue
   - **Metallic**: 0.0 (non-metallic)
   - **Roughness**: 0.3 (slightly glossy)
   - **Normal Map**: (optional, for surface detail)

**Add Environment**:
1. **Content → Isaac → Environments**
2. Choose environment (e.g., "Warehouse", "Office")
3. Drag into scene

**Configure Lighting**:
1. Select **Dome Light**
2. Set **Intensity**: 3.0
3. Enable **Shadows**: Soft Shadows
4. Set **Color Temperature**: 5500K (daylight)

### Step 8: Set Up ROS 2 Integration

**Install Isaac ROS Bridge**:
```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_bridge.git

# Install dependencies
cd ~/isaac_ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

**Enable ROS 2 in Isaac Sim**:
1. **Window → Extensions**
2. Search for **"ROS2"**
3. Enable **"omni.isaac.ros2_bridge"**
4. Restart Isaac Sim

**Verify ROS 2 connection**:
```bash
# Terminal: Check ROS 2 topics
source ~/isaac_ros_ws/install/setup.bash
ros2 topic list

# Should see Isaac Sim topics:
# /clock
# /tf
# /joint_states
```

### Step 9: Create Complete Scene

**Scene structure** (`humanoid_scene.usd`):
```
Stage Root
├── /World
    ├── /ground_plane (Physics-enabled ground)
    ├── /humanoid_robot (URDF import)
    │   ├── /torso
    │   ├── /head (with camera)
    │   ├── /left_leg
    │   └── /right_leg
    ├── /environment (Warehouse/Office)
    ├── /lights (Dome Light, Directional Light)
    └── /cameras (Render camera, ROS camera)
```

**Save scene**:
1. **File → Save As**
2. Save as `humanoid_scene.usd`
3. This becomes your main simulation scene

### Step 10: Debugging Common Issues

#### Issue 1: "No GPU detected" or "CUDA not available"
**Symptoms**: Isaac Sim won't launch or shows CPU-only mode

**Solutions**:
```bash
# Verify GPU
nvidia-smi

# Verify CUDA
nvcc --version

# Check CUDA in Isaac Sim
# Window → Extensions → omni.isaac.core → Check CUDA status
```

#### Issue 2: "Isaac Sim crashes on launch"
**Symptoms**: Application closes immediately

**Solutions**:
```bash
# Check system requirements
# Minimum: RTX 2060, 8GB VRAM, 16GB RAM

# Check logs
cat ~/.nvidia-omniverse/omniverse-launcher/IsaacSim-*/logs/*.log

# Try launching with verbose output
~/.nvidia-omniverse/omniverse-launcher/IsaacSim-*/isaac-sim.sh --verbose
```

#### Issue 3: "URDF import fails"
**Symptoms**: Robot doesn't appear or shows errors

**Solutions**:
```bash
# Check URDF validity
check_urdf model.urdf

# Verify mesh paths are absolute or package-relative
# Use: package://package_name/path/to/mesh.dae

# Check Isaac Sim console for import errors
# Window → Console (shows detailed error messages)
```

#### Issue 4: "ROS 2 topics not appearing"
**Symptoms**: No `/clock` or `/joint_states` topics

**Solutions**:
```bash
# Verify ROS 2 bridge extension enabled
# Window → Extensions → omni.isaac.ros2_bridge → Enabled

# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID
# Default is 0, ensure Isaac Sim uses same domain

# Restart Isaac Sim after enabling extension
```

#### Issue 5: "Low frame rate" or "Laggy rendering"
**Symptoms**: Simulation runs slowly

**Solutions**:
1. **Reduce ray tracing quality**: RTX Settings → Quality → Medium
2. **Disable denoising**: Can improve performance
3. **Reduce resolution**: Window → Rendering → Resolution → 1280×720
4. **Simplify scene**: Remove unnecessary objects
5. **Check GPU usage**: `nvidia-smi` should show high GPU utilization

## Part 3: Advanced Topics (Optional)

### USD Composition

**Layers and references**:
```python
# In Isaac Sim Python script
from pxr import Usd, UsdGeom

# Create stage
stage = Usd.Stage.CreateNew("scene.usd")

# Reference external USD file
robot_prim = stage.DefinePrim("/World/Robot")
robot_prim.GetReferences().AddReference("robot_model.usd")

# Override in current layer
robot_prim.GetAttribute("xformOp:translate").Set((0, 0, 1))
```

**Use case**: Modular scene composition, team collaboration

### Python Scripting in Isaac Sim

**Create Python script**: `scripts/setup_scene.py`

```python
#!/usr/bin/env python3
"""
Setup Isaac Sim scene programmatically
Isaac Sim 2023.1.1 | Python 3.10+
"""
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
ground_plane = world.scene.add_default_ground_plane()

# Add robot (from URDF)
robot_urdf_path = "/path/to/humanoid.urdf"
robot = world.scene.add(
    Robot(
        prim_path="/World/Robot",
        name="humanoid",
        usd_path=robot_urdf_path,
        position=np.array([0, 0, 1.0])
    )
)

# Reset world
world.reset()

# Run simulation
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

**Run script**:
```bash
~/.nvidia-omniverse/omniverse-launcher/IsaacSim-*/isaac-sim.sh --script setup_scene.py
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Photorealistic testing**: Capstone will use Isaac Sim for visual validation
- **Scene building**: Complex environments with obstacles and objects
- **Robot modeling**: Humanoid model with sensors and actuators
- **ROS 2 integration**: Foundation for VSLAM and navigation (Chapters 3.3 to 3.4)

Understanding Isaac Sim setup now is essential for building the capstone simulation environment.

## Summary

You learned:
- ✅ Installed **Isaac Sim 2023.1.1** on Ubuntu 22.04 with GPU support
- ✅ Created **photorealistic scenes** with ray-traced lighting
- ✅ Imported **robot models** (URDF) into Isaac Sim
- ✅ Configured **ROS 2 integration** via Isaac ROS bridge
- ✅ Debugged **common installation and launch issues**

**Next steps**: In Chapter 3.2, you'll set up synthetic data generation pipelines for ML training.

---

## Exercises

### Exercise 1: Scene Creation (Required)

**Objective**: Build a complete photorealistic scene in Isaac Sim.

**Tasks**:
1. Create new USD stage
2. Add ground plane with physics
3. Import humanoid robot model (URDF)
4. Add environment (warehouse or office)
5. Configure lighting (dome light + directional light)
6. Add camera for rendering
7. Save scene as `my_scene.usd`

**Acceptance Criteria**:
- [ ] Scene loads without errors
- [ ] Robot visible and properly lit
- [ ] Ray-traced shadows working
- [ ] Physics simulation stable
- [ ] Scene file saved successfully

**Estimated Time**: 90 minutes

### Exercise 2: ROS 2 Integration (Required)

**Objective**: Verify ROS 2 communication with Isaac Sim.

**Tasks**:
1. Enable ROS 2 bridge extension
2. Launch Isaac Sim with robot
3. Verify `/clock` topic publishing
4. Verify `/joint_states` topic (if robot has joints)
5. Publish joint commands from ROS 2 and verify robot moves

**Commands**:
```bash
# Check topics
ros2 topic list

# Echo joint states
ros2 topic echo /joint_states

# Publish joint command (example)
ros2 topic pub /joint_commands sensor_msgs/JointState "{name: ['left_knee'], position: [1.57]}"
```

**Acceptance Criteria**:
- [ ] ROS 2 topics visible
- [ ] `/clock` publishing simulation time
- [ ] Joint commands affect robot pose
- [ ] No connection errors

**Estimated Time**: 60 minutes

### Exercise 3: Material Customization (Challenge)

**Objective**: Create realistic materials for robot parts.

**Tasks**:
1. Research material properties (metal, plastic, rubber)
2. Create custom OmniPBR materials:
   - Metal torso (high metallic, low roughness)
   - Plastic limbs (low metallic, medium roughness)
   - Rubber feet (low metallic, high roughness)
3. Apply materials to robot links
4. Compare visual appearance with/without ray tracing

**Questions**:
- How does metallic value affect appearance?
- What roughness value gives realistic plastic look?
- Why use ray tracing for material rendering?

**Estimated Time**: 90 minutes

---

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html) - Official reference
- [Omniverse USD Documentation](https://openusd.org/release/index.html) - USD format specification
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS) - ROS 2 integration packages
- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/) - Platform overview

---

**Next**: [Chapter 3.2: Synthetic Data Generation for Training →](chapter-3 to 2.md)
