---
sidebar_position: 5
title: 2.4 URDF/SDF Robot Description
---

# Chapter 2.4: URDF/SDF Robot Description

URDF (Unified Robot Description Format) and SDF (Simulation Description Format) define robot structure, joints, links, sensors, and visual properties. This chapter covers converting URDF to SDF, optimizing models for Gazebo, and best practices for humanoid robot descriptions.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Convert** URDF models to SDF format for Gazebo
- **Optimize** robot models for simulation performance
- **Define** collision geometries separate from visual geometries
- **Add** sensors and actuators to robot descriptions
- **Debug** model loading and visualization issues

## Prerequisites

- **URDF modeling** experience (Module 1, Chapter 1.3)
- **Gazebo Classic 11** installed (Chapter 2.1)
- **ROS 2 Humble** configured
- **Understanding** of XML syntax

## Part 1: URDF vs. SDF

### Format Comparison

| Aspect | URDF | SDF |
|--------|------|-----|
| **Purpose** | Robot description (ROS) | Simulation description (Gazebo) |
| **Scope** | Single robot | World + robots + physics |
| **Sensors** | Limited support | Full sensor modeling |
| **Physics** | Basic | Advanced (friction, collisions) |
| **Plugins** | Via `<gazebo>` tags | Native support |

**When to use each**:
- **URDF**: ROS 2 packages, RViz2 visualization, robot_state_publisher
- **SDF**: Gazebo simulation, complex physics, sensor plugins

**Best practice**: Start with URDF (ROS 2 compatibility), convert to SDF for Gazebo.

### Conversion Process

**URDF → SDF conversion**:
1. Gazebo automatically converts URDF to SDF on load
2. Manual conversion: `gz sdf -k model.urdf > model.sdf`
3. SDF provides more features (advanced physics, sensors)

## Part 2: Hands-On Tutorial

### Project: Optimize Humanoid Model for Gazebo

**Goal**: Convert URDF to SDF, optimize collision geometries, and improve simulation performance.

**Tools**: Gazebo Classic 11, `gz sdf` tool, URDF

### Step 1: Convert URDF to SDF

**Install SDF tools**:
```bash
sudo apt install libsdformat-dev gz-sim
```

**Convert URDF to SDF**:
```bash
cd ~/gazebo_ws/src/gazebo_worlds/models/simple_humanoid

# Convert URDF to SDF
gz sdf -k model.urdf > model.sdf

# View SDF file
cat model.sdf
```

**Key differences in SDF**:
- **`<model>`** instead of `<robot>`
- **`<link>`** structure similar but with more physics options
- **`<joint>`** includes damping and friction
- **`<sensor>`** defined directly (not via Gazebo tags)

### Step 2: Optimize Collision Geometries

**Problem**: Complex visual meshes slow down collision detection.

**Solution**: Use simple collision geometries (boxes, spheres, cylinders).

**Original URDF** (complex visual):
```xml
<link name="torso">
  <visual name="torso_visual">
    <geometry>
      <mesh filename="package://my_robot/meshes/torso.dae"/>
    </geometry>
  </visual>
  <collision name="torso_collision">
    <geometry>
      <mesh filename="package://my_robot/meshes/torso.dae"/>  <!-- Slow! -->
    </geometry>
  </collision>
</link>
```

**Optimized URDF** (simple collision):
```xml
<link name="torso">
  <visual name="torso_visual">
    <geometry>
      <mesh filename="package://my_robot/meshes/torso.dae"/>  <!-- Detailed visual -->
    </geometry>
  </visual>
  <collision name="torso_collision">
    <geometry>
      <box size="0.3 0.2 0.4"/>  <!-- Simple box for collision -->
    </geometry>
    <pose>0 0 0 0 0 0</pose>  <!-- Match visual center -->
  </collision>
</link>
```

**Performance improvement**: 10 to 100x faster collision detection!

### Step 3: Add Inertial Properties

**Complete inertial definition** (for accurate physics):

```xml
<link name="thigh">
  <inertial>
    <mass>2.0</mass>
    <pose>0 0 -0.15 0 0 0</pose>  <!-- Center of mass offset -->
    <inertia>
      <!-- For cylinder: Ixx = Iyy = (1/12)*m*(3*r² + h²) -->
      <!-- For cylinder: Izz = (1/2)*m*r² -->
      <ixx>0.02</ixx>   <!-- Rotation about X-axis -->
      <iyy>0.02</iyy>   <!-- Rotation about Y-axis -->
      <izz>0.001</izz>  <!-- Rotation about Z-axis (smaller for long cylinder) -->
      <ixy>0.0</ixy>    <!-- Cross terms (0 for symmetric objects) -->
      <ixz>0.0</ixz>
      <iyz>0.0</iyz>
    </inertia>
  </inertial>
</link>
```

**Inertia formulas** (common shapes):

| Shape | Ixx, Iyy | Izz |
|-------|----------|-----|
| **Box** (a×b×c) | (1/12)*m*(b²+c²) | (1/12)*m*(a²+b²) |
| **Cylinder** (r, h) | (1/12)*m*(3*r²+h²) | (1/2)*m*r² |
| **Sphere** (r) | (2/5)*m*r² | (2/5)*m*r² |

### Step 4: Define Joint Limits and Dynamics

**Complete joint definition**:

```xml
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shank"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  
  <!-- Joint limits -->
  <limit lower="0" upper="3.14159" effort="100" velocity="10"/>
  
  <!-- Joint dynamics (damping, friction) -->
  <dynamics damping="1.0" friction="0.1"/>
  
  <!-- Safety limits (soft stops) -->
  <safety_controller soft_lower_limit="0.1" soft_upper_limit="3.0" k_position="100" k_velocity="10"/>
</joint>
```

**Key parameters**:
- **`lower/upper`**: Hard limits (radians)
- **`effort`**: Maximum torque (N⋅m)
- **`velocity`**: Maximum angular velocity (rad/s)
- **`damping`**: Resistance to motion (reduces oscillation)
- **`friction`**: Static friction in joint

### Step 5: Add Material Properties

**Define materials** (for visual appearance):

```xml
<!-- Define material -->
<material name="blue_plastic">
  <color rgba="0 0.5 1 1"/>  <!-- RGBA: Red, Green, Blue, Alpha -->
</material>

<!-- Use material in visual -->
<link name="torso">
  <visual name="torso_visual">
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
    <material name="blue_plastic"/>
  </visual>
</link>
```

**Common materials**:
- **Metal**: `rgba="0.7 0.7 0.7 1"` (gray)
- **Plastic**: `rgba="0.2 0.2 0.8 1"` (blue)
- **Rubber**: `rgba="0.1 0.1 0.1 1"` (black)

### Step 6: Create Complete SDF Model

**File**: `models/simple_humanoid/model.sdf`

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_humanoid">
    
    <!-- Base Link (Torso) -->
    <link name="torso">
      <pose>0 0 1.0 0 0 0</pose>  <!-- Initial pose -->
      
      <!-- Visual -->
      <visual name="torso_visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.4</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Blue</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      
      <!-- Collision (optimized: simple box) -->
      <collision name="torso_collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.4</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      
      <!-- Inertial -->
      <inertial>
        <mass>10.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.2</ixx>
          <iyy>0.15</iyy>
          <izz>0.15</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Joints (same as URDF) -->
    <joint name="left_hip" type="revolute">
      <parent>torso</parent>
      <child>left_thigh</child>
      <pose>0.1 0 -0.2 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100</effort>
          <velocity>10</velocity>
        </limit>
        <dynamics>
          <damping>1.0</damping>
          <friction>0.1</friction>
        </dynamics>
      </axis>
    </joint>

    <!-- Add other links and joints (similar structure) -->
    
    <!-- Sensors (defined directly in SDF) -->
    <link name="head">
      <!-- ... head link definition ... -->
      
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/humanoid</namespace>
          </ros>
          <frame_name>head_camera_frame</frame_name>
        </plugin>
      </sensor>
    </link>

  </model>
</sdf>
```

### Step 7: Spawn SDF Model in Gazebo

**Launch file** (`launch/humanoid_sdf.launch.py`):

```python
#!/usr/bin/env python3
"""
Launch humanoid SDF model in Gazebo
ROS 2 Humble | Gazebo Classic 11
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_worlds').find('gazebo_worlds')
    world_path = os.path.join(pkg_share, 'worlds', 'humanoid_physics.world')
    sdf_path = os.path.join(pkg_share, 'models', 'simple_humanoid', 'model.sdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'
        ),
        
        # Spawn SDF model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-file', sdf_path, '-x', '0', '-y', '0', '-z', '1.0'],
            output='screen'
        )
    ])
```

**Launch**:
```bash
ros2 launch gazebo_worlds humanoid_sdf.launch.py
```

### Step 8: Debugging Model Issues

#### Issue 1: Model Not Loading
**Symptoms**: Gazebo shows error, model doesn't appear

**Solutions**:
```bash
# Check SDF validity
gz sdf -k model.sdf

# Check for XML syntax errors
xmllint --noout model.sdf

# Verify model path
ls -la ~/.gazebo/models/simple_humanoid/
```

#### Issue 2: Collision Detection Not Working
**Symptoms**: Objects pass through each other

**Solutions**:
```xml
<!-- Ensure collision geometry defined -->
<collision name="link_collision">
  <geometry>
    <box size="0.3 0.2 0.4"/>
  </geometry>
</collision>

<!-- Check collision pose matches visual -->
<pose>0 0 0 0 0 0</pose>
```

#### Issue 3: Robot Unstable (Falling Over)
**Symptoms**: Robot tips over immediately

**Solutions**:
```xml
<!-- Check center of mass -->
<inertial>
  <pose>0 0 0 0 0 0</pose>  <!-- Should be at geometric center -->
</inertial>

<!-- Increase base width (wider stance) -->
<link name="foot">
  <geometry>
    <box size="0.2 0.1 0.02"/>  <!-- Wider foot = more stable -->
  </geometry>
</link>
```

#### Issue 4: Joints Not Moving
**Symptoms**: Joints don't respond to commands

**Solutions**:
```xml
<!-- Verify joint type -->
<joint type="revolute">  <!-- Not 'fixed' -->

<!-- Check joint limits -->
<limit lower="-3.14" upper="3.14"/>

<!-- Verify parent/child links exist -->
<parent>torso</parent>
<child>left_thigh</child>
```

## Part 3: Advanced Topics (Optional)

### Model Database Integration

**Add model to Gazebo database**:
```bash
# Copy model to user directory
cp -r ~/gazebo_ws/src/gazebo_worlds/models/simple_humanoid ~/.gazebo/models/

# Verify
gz model --list | grep humanoid
```

**Use model in world file**:
```xml
<include>
  <uri>model://simple_humanoid</uri>
  <pose>0 0 1.0 0 0 0</pose>
  <name>humanoid_1</name>
</include>
```

### Performance Optimization

**Tips for faster simulation**:
1. **Simple collision geometries**: Boxes/spheres instead of meshes
2. **Reduce sensor update rates**: 10 Hz instead of 30 Hz
3. **Disable unnecessary visuals**: Use `<visual>` only when needed
4. **Optimize physics timestep**: Larger timestep = faster (but less accurate)

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Model optimization**: Capstone humanoid will have optimized collision geometries for real-time simulation
- **Sensor integration**: All sensors (camera, LiDAR, IMU) defined in SDF
- **Joint control**: Proper joint limits and dynamics for walking and manipulation
- **Performance**: Optimized model enables faster-than-real-time simulation for training

Understanding URDF/SDF optimization now is essential for building the capstone robot model.

## Summary

You learned:
- ✅ Converted **URDF to SDF** format for Gazebo
- ✅ Optimized **collision geometries** for performance
- ✅ Defined **complete inertial properties** for accurate physics
- ✅ Added **joint dynamics** (damping, friction) for stability
- ✅ Debugged **model loading and visualization** issues

**Next steps**: In Chapter 2.5, you'll integrate Unity for photorealistic rendering and synthetic data generation.

---

## Exercises

### Exercise 1: URDF to SDF Conversion (Required)

**Objective**: Convert Module 1 URDF model to optimized SDF.

**Tasks**:
1. Take URDF from Module 1 (Chapter 1.3)
2. Convert to SDF using `gz sdf`
3. Optimize collision geometries (replace meshes with primitives)
4. Add joint dynamics (damping, friction)
5. Verify model loads correctly in Gazebo

**Acceptance Criteria**:
- [ ] SDF file valid (no errors)
- [ ] All collision geometries optimized
- [ ] Model spawns correctly in Gazebo
- [ ] Physics simulation stable

**Estimated Time**: 90 minutes

### Exercise 2: Model Performance Comparison (Required)

**Objective**: Measure simulation performance with different collision geometries.

**Tasks**:
1. Create two versions of same model:
   - Version A: Mesh collision geometries
   - Version B: Primitive collision geometries
2. Measure simulation FPS for each
3. Compare CPU usage
4. Document performance improvement

**Metrics**:
- Simulation FPS
- CPU usage (%)
- Memory usage (MB)

**Estimated Time**: 60 minutes

### Exercise 3: Complex Humanoid Model (Challenge)

**Objective**: Create detailed humanoid model with arms and hands.

**Tasks**:
1. Add arms (shoulder, elbow, wrist joints)
2. Add hands (fingers with joints)
3. Optimize all collision geometries
4. Define proper inertial properties
5. Test in Gazebo with physics

**Requirements**:
- 20+ joints total
- All links have collision/inertial properties
- Model stable in simulation

**Estimated Time**: 180 minutes

---

## Additional Resources

- [URDF Specification](http://wiki.ros.org/urdf/XML) - URDF format reference
- [SDFormat Specification](http://sdformat.org/) - SDF format reference
- [Gazebo Model Database](https://app.gazebosim.org/) - Browse models
- [Inertia Calculator](https://www.omnicalculator.com/physics/moment-of-inertia) - Calculate inertia values

---

**Next**: [Chapter 2.5: Unity Integration for Photorealistic Rendering →](chapter-2 to 5.md)
