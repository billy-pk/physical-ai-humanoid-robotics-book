---
sidebar_position: 3
title: 2.2 Physics Simulation (Gravity, Collisions, Friction)
---

# Chapter 2.2: Physics Simulation (Gravity, Collisions, Friction)

Realistic physics simulation is critical for humanoid robots. Walking, balancing, and manipulation all depend on accurate modeling of gravity, collisions, and friction. This chapter covers configuring Gazebo's physics engines to simulate realistic humanoid dynamics.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Configure** physics engines (ODE, Bullet) for humanoid dynamics
- **Model** gravity, friction coefficients, and collision properties
- **Simulate** humanoid walking and balance dynamics
- **Debug** physics artifacts (penetration, jitter, instability)
- **Tune** solver parameters for stability vs. performance tradeoffs

## Prerequisites

- **Gazebo Classic 11** installed (Chapter 2.1)
- **ROS 2 Humble** configured
- **Basic understanding** of physics (forces, torques, Newton's laws)
- **URDF modeling** experience (from Module 1)

## Part 1: Physics Engine Fundamentals

### Physics Engines in Gazebo

Gazebo supports multiple physics engines:

| Engine | Strengths | Use Cases |
|--------|-----------|-----------|
| **ODE** (Open Dynamics Engine) | Fast, stable, default | General robotics, humanoids |
| **Bullet** | Accurate collisions, soft bodies | Manipulation, grasping |
| **DART** | Advanced constraints | Complex mechanisms |
| **Simbody** | Biomechanics | Human motion simulation |

**This course uses ODE** (default, most widely used).

### Key Physics Parameters

#### 1. Gravity
**Gravity** accelerates objects downward (typically -9.81 m/s² in Z direction).

**Configuration**:
```xml
<physics type="ode">
  <gravity>0 0 -9.81</gravity>  <!-- X, Y, Z components -->
</physics>
```

**Why it matters for humanoids**:
- Walking requires gravity to generate ground reaction forces
- Balance control depends on gravitational torque
- Falling dynamics must be realistic for safety testing

#### 2. Collision Detection
**Collision detection** prevents objects from penetrating each other.

**Collision geometries**:
- **Box**: Fast, good for simple shapes
- **Sphere**: Very fast, good for round objects
- **Mesh**: Accurate but slower (use for complex shapes)
- **Cylinder**: Good for limbs, links

**Example**:
```xml
<link name="foot">
  <collision name="foot_collision">
    <geometry>
      <box>
        <size>0.15 0.08 0.02</size>  <!-- Length, width, height -->
      </box>
    </geometry>
    <pose>0 0 -0.01 0 0 0</pose>  <!-- Slightly below visual -->
  </collision>
</link>
```

**Best practice**: Collision geometry should be **simpler** than visual geometry (for performance).

#### 3. Friction
**Friction** resists sliding motion. Critical for walking and manipulation.

**Friction coefficients**:
- **Static friction** (`mu1`): Force to start sliding
- **Dynamic friction** (`mu2`): Force to keep sliding
- **Typical values**:
  - Rubber on concrete: `mu1=1.0`, `mu2=0.8`
  - Metal on metal: `mu1=0.5`, `mu2=0.3`
  - Ice: `mu1=0.1`, `mu2=0.05`

**Configuration**:
```xml
<collision name="foot_collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>        <!-- Static friction -->
        <mu2>0.8</mu2>      <!-- Dynamic friction -->
        <fdir1>0 0 0</fdir1> <!-- Friction direction (anisotropic) -->
        <slip1>0.0</slip1>   <!-- Slip in direction 1 -->
        <slip2>0.0</slip2>   <!-- Slip in direction 2 -->
      </ode>
    </friction>
  </surface>
</collision>
```

#### 4. Inertia
**Inertia** determines how objects resist rotational acceleration.

**For humanoid links**:
- **Mass**: Total weight of link
- **Center of mass**: Where mass is concentrated
- **Inertia tensor**: Resistance to rotation about each axis

**Example (humanoid leg link)**:
```xml
<link name="thigh">
  <inertial>
    <mass>2.5</mass>  <!-- kg -->
    <pose>0 0 0.15 0 0 0</pose>  <!-- Center of mass offset -->
    <inertia>
      <ixx>0.05</ixx>  <!-- Rotation about X-axis -->
      <iyy>0.05</iyy>  <!-- Rotation about Y-axis -->
      <izz>0.01</izz>  <!-- Rotation about Z-axis (smaller for cylindrical link) -->
      <ixy>0.0</ixy>   <!-- Cross terms (usually 0 for symmetric objects) -->
      <ixz>0.0</ixz>
      <iyz>0.0</iyz>
    </inertia>
  </inertial>
</link>
```

**Why inertia matters**:
- Walking dynamics depend on leg inertia
- Balance control requires accurate center of mass
- Manipulation forces depend on object inertia

### Solver Parameters

**ODE solver settings** control accuracy vs. performance:

```xml
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>      <!-- 'quick' or 'world' -->
      <iters>10</iters>       <!-- Solver iterations (more = more accurate, slower) -->
      <sor>1.4</sor>          <!-- Successive Over-Relaxation (1.0 to 1.9) -->
      <use_dynamic_moi_rescaling>true</use_dynamic_moi_rescaling>
    </solver>
    <constraints>
      <cfm>0.00001</cfm>      <!-- Constraint Force Mixing (smaller = stiffer) -->
      <erp>0.2</erp>          <!-- Error Reduction Parameter (0 to 1, higher = faster correction) -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Tuning guide**:
- **More iterations** (`iters=50`): More accurate, slower
- **Higher ERP** (`erp=0.8`): Faster constraint correction, may cause jitter
- **Lower CFM** (`cfm=0.000001`): Stiffer contacts, more stable

## Part 2: Hands-On Tutorial

### Project: Simulate Humanoid Walking Dynamics

**Goal**: Create a simple humanoid model with realistic physics and simulate walking dynamics.

**Tools**: Gazebo Classic 11, ROS 2 Humble, URDF

### Step 1: Create Simple Humanoid URDF

**File**: `models/simple_humanoid/model.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  
  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual name="torso_visual">
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision name="torso_collision">
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.15" iyz="0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <link name="left_thigh">
    <visual name="thigh_visual">
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision name="thigh_collision">
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_shank">
    <visual name="shank_visual">
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision name="shank_collision">
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_foot">
    <visual name="foot_visual">
      <geometry>
        <box size="0.15 0.08 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision name="foot_collision">
      <geometry>
        <box size="0.15 0.08 0.02"/>
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
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.01" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.1 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shank"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="10"/>
  </joint>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shank"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="5"/>
  </joint>

  <!-- Right Leg (mirror of left) -->
  <link name="right_thigh">
    <visual name="thigh_visual">
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision name="thigh_collision">
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <link name="right_shank">
    <visual name="shank_visual">
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision name="shank_collision">
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <link name="right_foot">
    <visual name="foot_visual">
      <geometry>
        <box size="0.15 0.08 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision name="foot_collision">
      <geometry>
        <box size="0.15 0.08 0.02"/>
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
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.01" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="-0.1 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shank"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="10"/>
  </joint>

  <joint name="right_ankle" type="revolute">
    <parent link="right_shank"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="5"/>
  </joint>

</robot>
```

**Key features**:
- **Realistic masses**: Torso (10 kg), thighs (2 kg each), shanks (1.5 kg), feet (0.5 kg)
- **Proper inertia**: Cylindrical links have higher inertia about length axis
- **Friction on feet**: `mu=1.0` prevents sliding
- **Joint limits**: Realistic ranges for hip, knee, ankle

### Step 2: Create Gazebo World with Physics

**File**: `worlds/humanoid_physics.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_physics">
    
    <!-- Physics Configuration -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>  <!-- Higher for stability -->
          <sor>1.4</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground Plane with Friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
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
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Step 3: Launch Humanoid in Gazebo

**File**: `launch/humanoid_physics.launch.py`

```python
#!/usr/bin/env python3
"""
Launch humanoid robot in Gazebo with physics simulation
ROS 2 Humble | Gazebo Classic 11
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('gazebo_worlds').find('gazebo_worlds')
    world_path = os.path.join(pkg_share, 'worlds', 'humanoid_physics.world')
    urdf_path = os.path.join(pkg_share, 'models', 'simple_humanoid', 'model.urdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'
        ),
        
        # Spawn humanoid robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '1.0'],
            output='screen'
        ),
        
        # Robot state publisher (for RViz2 visualization)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        )
    ])
```

**Build and launch**:
```bash
cd ~/gazebo_ws
colcon build --packages-select gazebo_worlds
source install/setup.bash

ros2 launch gazebo_worlds humanoid_physics.launch.py
```

**Expected Result**:
- Humanoid spawns at height z=1.0 m
- Robot falls due to gravity
- Feet contact ground with friction
- Robot should stabilize (not penetrate ground)

### Step 4: Test Physics Parameters

**Modify friction** (in URDF foot collision):
```xml
<friction>
  <ode>
    <mu>0.1</mu>  <!-- Low friction (like ice) -->
    <mu2>0.05</mu2>
  </ode>
</friction>
```

**Result**: Robot slides when landing (unstable).

**Modify gravity** (in world file):
```xml
<gravity>0 0 -4.9</gravity>  <!-- Half gravity (like moon) -->
```

**Result**: Robot falls slower, easier to balance.

**Modify solver iterations**:
```xml
<iters>10</iters>  <!-- Lower iterations -->
```

**Result**: May see jittering or penetration (less stable).

### Step 5: Debugging Physics Issues

#### Issue 1: Objects Penetrating Ground
**Symptoms**: Robot sinks into ground plane

**Solutions**:
```xml
<!-- Increase contact surface layer -->
<contact_surface_layer>0.01</contact_surface_layer>

<!-- Increase solver iterations -->
<iters>100</iters>

<!-- Reduce timestep for more accuracy -->
<max_step_size>0.0001</max_step_size>
```

#### Issue 2: Jittering/Shaking
**Symptoms**: Robot vibrates when stationary

**Solutions**:
```xml
<!-- Reduce ERP (slower correction) -->
<erp>0.1</erp>

<!-- Increase CFM (softer contacts) -->
<cfm>0.0001</cfm>

<!-- Increase damping in joints -->
<joint name="left_hip" type="revolute">
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

#### Issue 3: Unrealistic Bouncing
**Symptoms**: Objects bounce too much on contact

**Solutions**:
```xml
<!-- Add restitution (bounciness) control -->
<collision name="foot_collision">
  <surface>
    <bounce>
      <restitution_coefficient>0.0</restitution_coefficient>  <!-- 0 = no bounce -->
    </bounce>
  </surface>
</collision>
```

## Part 3: Advanced Topics (Optional)

### Bullet Physics Engine

**Switch to Bullet** (alternative to ODE):
```xml
<physics type="bullet">
  <gravity>0 0 -9.81</gravity>
  <bullet>
    <solver>
      <type>SequentialImpulse</type>
      <iters>50</iters>
      <sor>1.0</sor>
    </solver>
    <constraints>
      <cfm>0.00001</cfm>
      <erp>0.2</erp>
    </constraints>
  </bullet>
</physics>
```

**When to use Bullet**:
- More accurate collision detection
- Better for soft bodies
- Slightly slower than ODE

### Contact Forces Visualization

**Enable contact visualization**:
```bash
# In Gazebo GUI: View → Contacts
# Or via ROS 2:
ros2 topic echo /gazebo/default/physics/contacts
```

**Useful for debugging**:
- See which links are in contact
- Verify contact forces
- Debug collision geometries

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Walking dynamics**: Capstone humanoid will walk using physics-accurate simulation
- **Balance control**: Balance algorithms depend on accurate center of mass and inertia
- **Manipulation**: Grasping objects requires realistic friction and contact forces
- **Fall recovery**: Testing fall scenarios safely requires accurate physics

Understanding physics parameters now is essential for tuning the capstone simulation.

## Summary

You learned:
- ✅ Configured **physics engines** (ODE) for humanoid dynamics
- ✅ Modeled **gravity, friction, and collisions** in URDF
- ✅ Created **realistic humanoid model** with proper masses and inertia
- ✅ Simulated **walking dynamics** and verified physics accuracy
- ✅ Debugged **common physics artifacts** (penetration, jitter)

**Next steps**: In Chapter 2.3, you'll add sensors (cameras, LiDAR, IMUs) to your humanoid model.

---

## Exercises

### Exercise 1: Friction Experiment (Required)

**Objective**: Understand how friction affects humanoid stability.

**Tasks**:
1. Launch humanoid with default friction (`mu=1.0`)
2. Record if robot stabilizes after falling
3. Change friction to `mu=0.1` (low friction)
4. Observe robot sliding/unstable behavior
5. Change friction to `mu=2.0` (high friction)
6. Compare stability across friction values

**Questions**:
- What friction value gives best stability?
- Why does low friction cause sliding?
- How does friction affect walking?

**Acceptance Criteria**:
- [ ] Tested 3 different friction values
- [ ] Documented behavior for each value
- [ ] Explained relationship between friction and stability

**Estimated Time**: 45 minutes

### Exercise 2: Inertia Tuning (Required)

**Objective**: Understand how inertia affects robot dynamics.

**Tasks**:
1. Modify thigh link inertia (increase `izz` by 10x)
2. Launch robot and observe falling behavior
3. Compare to original inertia values
4. Modify center of mass position (move upward)
5. Observe effect on balance

**Questions**:
- How does higher inertia affect rotation?
- Why does center of mass position matter?
- What inertia values are realistic for humanoid links?

**Estimated Time**: 60 minutes

### Exercise 3: Physics Solver Tuning (Challenge)

**Objective**: Optimize solver parameters for stability vs. performance.

**Tasks**:
1. Test with `iters=10` (low accuracy)
2. Test with `iters=100` (high accuracy)
3. Measure simulation speed (FPS) for each
4. Find minimum iterations for stable simulation
5. Document performance vs. accuracy tradeoff

**Metrics**:
- Simulation FPS (higher = faster)
- Physics stability (no jitter/penetration)
- CPU usage

**Estimated Time**: 90 minutes

---

## Additional Resources

- [ODE Documentation](https://www.ode.org/) - Physics engine reference
- [Gazebo Physics Tutorial](https://gazebosim.org/docs/latest/physics_configuration) - Configuration guide
- [URDF Physics](http://wiki.ros.org/urdf/XML/link) - Inertia and collision modeling
- [Friction Coefficients](https://www.engineeringtoolbox.com/friction-coefficients-d_778.html) - Real-world values

---

**Next**: [Chapter 2.3: Sensor Simulation (LiDAR, Cameras, IMUs) →](chapter-2 to 3.md)
