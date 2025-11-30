---
sidebar_position: 2
title: 2.1 Gazebo Fundamentals & Setup
---

# Chapter 2.1: Gazebo Fundamentals & Setup

Gazebo is the industry-standard physics simulator for robotics. It provides accurate dynamics, realistic sensor models, and seamless ROS 2 integration—making it essential for developing humanoid robots before deploying to expensive hardware.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Install** Gazebo Classic or Fortress on Ubuntu 22.04 with ROS 2 integration
- **Understand** Gazebo's architecture (worlds, models, plugins)
- **Launch** Gazebo with ROS 2 bridge and verify communication
- **Create** custom world files with physics, lighting, and objects
- **Debug** common installation and launch issues

## Prerequisites

- **Ubuntu 22.04 LTS** installed (native or VM with 4GB+ RAM)
- **ROS 2 Humble** installed and configured (Module 1, Chapter 1.1)
- **Basic Linux terminal skills**: cd, ls, sudo, nano/vim
- **Understanding of ROS 2 topics** (from Module 1)

## Part 1: Gazebo Architecture Fundamentals

### What is Gazebo?

**Gazebo** is a 3D physics simulator that:
- Simulates **rigid body dynamics** (gravity, collisions, friction)
- Models **sensors** (cameras, LiDAR, IMUs) with realistic noise
- Provides **ROS 2 integration** via `gazebo_ros` packages
- Supports **real-time** and **faster-than-real-time** simulation

**Why Gazebo for humanoid robots?**
- **Cost**: Test algorithms without $160K+ hardware
- **Safety**: Debug falls, crashes, and failures safely
- **Speed**: Run 10x-100x faster than real-time
- **Reproducibility**: Exact same conditions every run
- **Multi-robot**: Test coordination of multiple humanoids

### Gazebo Versions

| Version | Status | ROS 2 Support | Recommended For |
|---------|--------|---------------|------------------|
| **Gazebo Classic 11** | Stable, widely used | ROS 2 Humble/Iron | Production projects, tutorials |
| **Gazebo Fortress** | Latest, actively developed | ROS 2 Humble/Iron | New projects, advanced features |
| **Gazebo Garden** | Future release | ROS 2 Jazzy+ | Experimental |

**This course uses Gazebo Classic 11** (stable, well-documented) or **Gazebo Fortress** (latest features).

### Key Concepts

#### 1. Worlds
A **world file** (`.world`) defines:
- **Physics engine** (ODE, Bullet, DART)
- **Gravity** and environment properties
- **Models** (robots, objects, obstacles)
- **Lighting** (sun, spotlights)
- **Plugins** (sensor drivers, controllers)

**Example world structure**:
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
    </physics>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- Your robot model here -->
  </world>
</sdf>
```

#### 2. Models
**Models** are reusable robot/object descriptions (URDF or SDF format).

**Model locations**:
- **System models**: `/usr/share/gazebo-11/models/` (installed with Gazebo)
- **User models**: `~/.gazebo/models/` (your custom models)
- **Workspace models**: `~/gazebo_ws/src/[package]/models/` (project-specific)

**Example**: `model://ground_plane` refers to `/usr/share/gazebo-11/models/ground_plane/`

#### 3. Plugins
**Plugins** are shared libraries that extend Gazebo functionality:
- **Sensor plugins**: Camera, LiDAR, IMU drivers
- **Model plugins**: Joint controllers, physics modifiers
- **World plugins**: Environment effects

**ROS 2 integration**: `libgazebo_ros_*.so` plugins publish/subscribe to ROS 2 topics.

### Gazebo-ROS 2 Bridge

The `gazebo_ros` package provides:
- **Launch files**: Start Gazebo with ROS 2 integration
- **Plugins**: Convert Gazebo data ↔ ROS 2 messages
- **Services**: Spawn/delete models, pause/reset simulation

**Key topics**:
- `/clock`: Simulation time (for time synchronization)
- `/model_states`: Positions/velocities of all models
- `/gazebo/reset_world`: Reset simulation service

## Part 2: Hands-On Tutorial

### Project: Install Gazebo and Create Your First World

**Goal**: Install Gazebo Classic 11, launch it with ROS 2 integration, and create a custom world file.

**Tools**: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11

### Step 1: Install Gazebo Classic 11

```bash
# Update package lists
sudo apt update

# Install Gazebo Classic 11
sudo apt install gazebo11 libgazebo11-dev

# Install ROS 2 Gazebo integration packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

# Verify installation
gazebo --version
# Should output: gazebo11 11.x.x
```

**Installation time**: ~5 to 10 minutes (downloads ~500MB)

### Step 2: Test Basic Launch

**Terminal 1: Launch Gazebo (empty world)**:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch Gazebo with ROS 2 bridge
ros2 launch gazebo_ros gazebo.launch.py
```

**Expected Output**:
- Gazebo GUI window opens
- Empty world with ground plane and sun
- Terminal shows: `[INFO] [gazebo]: Gazebo multi-robot simulator, version 11.x.x`

**Terminal 2: Verify ROS 2 Integration**:
```bash
source /opt/ros/humble/setup.bash

# List ROS 2 topics
ros2 topic list

# You should see:
# /clock
# /model_states
# /parameter_events
# /rosout
```

**Check simulation time**:
```bash
ros2 topic echo /clock
```

**Expected Output**:
```
clock:
  sec: 0
  nanosec: 0
---
clock:
  sec: 0
  nanosec: 10000000
---
```

### Step 3: Create Custom World File

**Create workspace**:
```bash
mkdir -p ~/gazebo_ws/src
cd ~/gazebo_ws/src

# Create ROS 2 package for Gazebo worlds
ros2 pkg create --build-type ament_cmake gazebo_worlds --dependencies gazebo_ros
cd gazebo_worlds
mkdir -p worlds
```

**Create world file**: `worlds/my_first_world.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_first_world">
    
    <!-- Physics Engine -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.4</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add a simple box -->
    <model name="test_box">
      <pose>2 0 0.5 0 0 0</pose>
      <static>false</static>
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
            <script>
              <name>Gazebo/Red</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <iyy>0.083</iyy>
            <izz>0.083</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

**Explanation**:
- **`<physics>`**: ODE engine with gravity (-9.81 m/s² in Z direction)
- **`<max_step_size>`**: Physics timestep (0.001s = 1000 Hz)
- **`<real_time_factor>`**: 1.0 = real-time, 2.0 = 2x speed
- **`<model>`**: Box at position (2, 0, 0.5) with mass and inertia

### Step 4: Launch Custom World

**Create launch file**: `launch/my_world.launch.py`

```python
#!/usr/bin/env python3
"""
Launch file for custom Gazebo world
ROS 2 Humble | Gazebo Classic 11
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package path
    pkg_share = FindPackageShare('gazebo_worlds').find('gazebo_worlds')
    world_path = os.path.join(pkg_share, 'worlds', 'my_first_world.world')

    return LaunchDescription([
        # Launch Gazebo with custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'
        ),
        
        # Spawn ROS 2 bridge node (optional, for advanced features)
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen'
        )
    ])
```

**Build and launch**:
```bash
cd ~/gazebo_ws
colcon build --packages-select gazebo_worlds
source install/setup.bash

# Launch custom world
ros2 launch gazebo_worlds my_world.launch.py
```

**Expected Result**:
- Gazebo opens with your custom world
- Red box visible at position (2, 0, 0.5)
- Box falls due to gravity and lands on ground plane

### Step 5: Add Models from Gazebo Library

Gazebo comes with pre-built models. Add them to your world:

**Modify `my_first_world.world`** (add before `</world>`):

```xml
    <!-- Add a table -->
    <include>
      <uri>model://table</uri>
      <pose>0 2 0 0 0 0</pose>
      <name>table_1</name>
    </include>

    <!-- Add a coke can -->
    <include>
      <uri>model://coke_can</uri>
      <pose>0 2 0.75 0 0 0</pose>
      <name>coke_1</name>
    </include>
```

**Reload world** (or restart Gazebo):
- The table and coke can appear
- Coke can falls onto table due to gravity

### Step 6: Introspection Tools

**List all models in simulation**:
```bash
ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList
```

**Get model state**:
```bash
ros2 topic echo /model_states
```

**Expected Output**:
```
name:
- 'ground_plane'
- 'sun'
- 'test_box'
- 'table_1'
- 'coke_1'
pose:
  - position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  # ... more poses
```

**Spawn model via service**:
```bash
ros2 service call /gazebo/spawn_entity gazebo_msgs/srv/SpawnEntity \
  "{name: 'my_robot', xml: '<?xml version=\"1.0\"?><sdf version=\"1.6\"><model name=\"my_robot\"><link name=\"link\"><visual name=\"visual\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>', initial_pose: {position: {x: 0.0, y: 0.0, z: 1.0}}}"
```

### Step 7: Debugging Common Issues

#### Issue 1: "gazebo: command not found"
**Cause**: Gazebo not installed or not in PATH

**Solution**:
```bash
sudo apt install gazebo11
which gazebo
# Should output: /usr/bin/gazebo
```

#### Issue 2: "Unable to find model://ground_plane"
**Cause**: Gazebo model paths not configured

**Solution**:
```bash
# Check model paths
echo $GAZEBO_MODEL_PATH

# Add default path if missing
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH

# Make permanent (add to ~/.bashrc)
echo 'export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
```

#### Issue 3: "ROS 2 topics not appearing"
**Cause**: `gazebo_ros` packages not installed or not sourced

**Solution**:
```bash
# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs

# Source ROS 2
source /opt/ros/humble/setup.bash

# Verify topics
ros2 topic list | grep gazebo
```

#### Issue 4: "Gazebo GUI not opening" (headless mode)
**Cause**: Running on server without display

**Solution**:
```bash
# Launch with GUI disabled (for headless systems)
gazebo --verbose --headless my_world.world

# Or set display variable
export DISPLAY=:0
gazebo my_world.world
```

#### Issue 5: "Physics simulation unstable" (objects jittering/penetrating)
**Cause**: Timestep too large or solver settings incorrect

**Solution**:
```xml
<!-- In world file, reduce timestep -->
<max_step_size>0.0001</max_step_size>
<real_time_update_rate>10000</real_time_update_rate>

<!-- Or increase solver iterations -->
<iters>50</iters>
```

## Part 3: Advanced Topics (Optional)

### Gazebo Fortress (Latest Version)

**Install Gazebo Fortress** (alternative to Classic):
```bash
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main"
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo apt update
sudo apt install gz-fortress
```

**Key differences**:
- **SDFormat 1.8+**: More features than SDF 1.6
- **Better performance**: Optimized physics engine
- **New plugins**: Enhanced sensor models

**Launch Fortress**:
```bash
gz sim my_world.sdf
```

### Performance Tuning

**For faster simulation**:
```xml
<physics>
  <max_step_size>0.002</max_step_size>  <!-- Larger timestep = faster -->
  <real_time_factor>2.0</real_time_factor>  <!-- 2x real-time speed -->
</physics>
```

**For higher accuracy**:
```xml
<physics>
  <max_step_size>0.0001</max_step_size>  <!-- Smaller timestep = more accurate -->
  <real_time_factor>0.5</real_time_factor>  <!-- Slower but more precise -->
  <ode>
    <solver>
      <iters>100</iters>  <!-- More iterations = better convergence -->
    </solver>
  </ode>
</physics>
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **World building**: Capstone will use a complex Gazebo world with:
  - Multiple rooms and obstacles
  - Objects to manipulate (cups, books, tools)
  - Dynamic obstacles (moving people, doors)
  
- **Model management**: The humanoid robot model will be:
  - Defined in URDF/SDF (from Module 1)
  - Spawned in Gazebo with sensors
  - Controlled via ROS 2 topics
  
- **Simulation workflow**: Develop → Test in Gazebo → Validate → Deploy to hardware

Understanding world files and Gazebo architecture now is essential for building the capstone simulation environment.

## Summary

You learned:
- ✅ Gazebo provides physics-accurate simulation for robotics
- ✅ Installed **Gazebo Classic 11** on Ubuntu 22.04
- ✅ Created custom world files with physics and models
- ✅ Launched Gazebo with ROS 2 integration
- ✅ Debugged common installation and launch issues

**Next steps**: In Chapter 2.2, you'll configure physics engines for realistic humanoid dynamics (gravity, collisions, friction).

---

## Exercises

### Exercise 1: Custom World Creation (Required)

**Objective**: Build a world file with multiple objects and verify physics.

**Tasks**:
1. Create a new world file `exercise_world.world`
2. Add 3 different objects (box, sphere, cylinder) at different heights
3. Configure gravity and physics timestep
4. Launch world and observe objects falling
5. Verify objects land correctly (no penetration)

**Acceptance Criteria**:
- [ ] World file loads without errors
- [ ] All 3 objects visible in Gazebo
- [ ] Objects fall due to gravity
- [ ] No penetration through ground plane
- [ ] Physics simulation stable (no jittering)

**Estimated Time**: 45 minutes

### Exercise 2: Model Spawning Service (Required)

**Objective**: Use ROS 2 services to spawn models dynamically.

**Tasks**:
1. Launch empty Gazebo world
2. Use `ros2 service call` to spawn a box model
3. Spawn 3 boxes at different positions
4. Verify models appear in `/model_states` topic
5. Delete a model using `/gazebo/delete_model` service

**Commands**:
```bash
# Spawn box
ros2 service call /gazebo/spawn_entity gazebo_msgs/srv/SpawnEntity \
  "{name: 'box1', xml: '<?xml version=\"1.0\"?><sdf version=\"1.6\"><model name=\"box1\"><link name=\"link\"><visual name=\"visual\"><geometry><box><size>1 1 1</size></box></geometry></visual><collision name=\"collision\"><geometry><box><size>1 1 1</size></box></geometry></collision><inertial><mass>1.0</mass><inertia><ixx>0.083</ixx><iyy>0.083</iyy><izz>0.083</izz></inertia></inertial></link></model></sdf>', initial_pose: {position: {x: 0.0, y: 0.0, z: 2.0}}}"

# Delete model
ros2 service call /gazebo/delete_model gazebo_msgs/srv/DeleteModel "{model_name: 'box1'}"
```

**Acceptance Criteria**:
- [ ] Successfully spawned 3 boxes via service calls
- [ ] Boxes appear at correct positions
- [ ] Models listed in `/model_states`
- [ ] Successfully deleted a model

**Estimated Time**: 30 minutes

### Exercise 3: World File Analysis (Challenge)

**Objective**: Analyze and modify an existing Gazebo world file.

**Tasks**:
1. Find an example world file (e.g., `/usr/share/gazebo-11/worlds/empty.world`)
2. Copy it to your workspace
3. Modify physics parameters (gravity, timestep, solver)
4. Add 5+ models from Gazebo library
5. Compare simulation behavior with different physics settings

**Questions**:
- What happens if you set `real_time_factor` to 10.0?
- How does `max_step_size` affect simulation accuracy?
- What is the purpose of `<sor>` (Successive Over-Relaxation) parameter?

**Estimated Time**: 60 minutes

---

## Additional Resources

- [Gazebo Classic Documentation](https://gazebosim.org/docs) - Official reference
- [Gazebo Tutorials](https://gazebosim.org/docs/latest/tutorials) - Step-by-step guides
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/gazebo_ros_pkgs) - GitHub repository
- [SDFormat Specification](http://sdformat.org/) - World/model file format
- [Gazebo Model Database](https://app.gazebosim.org/) - Browse and download models

---

**Next**: [Chapter 2.2: Physics Simulation (Gravity, Collisions, Friction) →](chapter-2 to 2.md)
