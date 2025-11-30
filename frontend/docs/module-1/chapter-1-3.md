---
sidebar_position: 4
title: 1.3 URDF for Humanoid Robots
---

# Chapter 1.3: URDF for Humanoid Robots

URDF (Unified Robot Description Format) is the standard XML-based format for describing robot kinematics, dynamics, and sensors in ROS. Every humanoid robot—from academic research platforms to Boston Dynamics' Atlas—uses URDF (or its derivatives) to define structure.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Define** robot structure using URDF (links, joints, sensors)
- **Model** different joint types (revolute, prismatic, continuous, fixed)
- **Add** collision geometries and visual meshes
- **Attach** sensors (cameras, LiDAR, IMUs) to robot links
- **Visualize** URDF models in RViz2 with joint state controls

## Prerequisites

- **ROS 2 Humble installed** with RViz2
- **Basic XML syntax**: tags, attributes, nesting
- **Understanding of coordinate frames**: X-forward, Y-left, Z-up (REP-103)
- **3D geometry basics**: rotations, translations, Euler angles

## Part 1: URDF Fundamentals

### What is URDF?

URDF is an XML format that describes:
1. **Links**: Rigid bodies (torso, upper_arm, forearm, hand)
2. **Joints**: Connections between links (shoulder, elbow, wrist)
3. **Sensors**: Cameras, LiDAR, IMUs attached to links
4. **Visual**: How the robot looks (meshes, colors)
5. **Collision**: Simplified geometry for physics simulation

**Why URDF matters**:
- **Kinematics**: Calculate end-effector position from joint angles (forward kinematics)
- **Simulation**: Gazebo and Isaac Sim use URDF for physics
- **Visualization**: RViz2 displays robot models
- **Planning**: MoveIt 2 uses URDF for motion planning

### URDF Structure

```xml
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>
  
  <link name="upper_arm">...</link>
  
  <!-- Joints connect links -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
</robot>
```

### Key Concepts

#### Links
A **link** represents a rigid body with:
- **Visual**: Appearance (mesh file or primitive shapes)
- **Collision**: Simplified geometry for collision detection
- **Inertial**: Mass and moment of inertia for dynamics

#### Joints
A **joint** connects two links. Types:

| Joint Type | Description | Humanoid Example |
|------------|-------------|------------------|
| **revolute** | Rotates around axis, with limits | Elbow (0° to 150°) |
| **continuous** | Rotates 360° without limits | Wheel axle |
| **prismatic** | Slides along axis | Elevator mechanism |
| **fixed** | No movement | Sensor mount |
| **floating** | 6-DOF (position + orientation) | Base link (for mobile humanoids) |
| **planar** | Moves in XY plane | Rarely used |

#### Coordinate Frames (REP-103 Standard)

ROS follows **right-hand rule**:
- **X-axis**: Forward (red)
- **Y-axis**: Left (green)
- **Z-axis**: Up (blue)

**Rotations** (RPY - Roll, Pitch, Yaw):
- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis
- **Yaw**: Rotation around Z-axis

## Part 2: Hands-On Tutorial

### Project 1: Simple Two-Link Robot Arm

**Goal**: Create a URDF for a 2-DOF robot arm and visualize in RViz2.

**File**: \`simple_arm.urdf\`

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  
  <!-- Base Link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Upper Arm Link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0"
               iyy="0.004" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Shoulder Joint (connects base to upper arm) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Rotates around Y-axis (pitch) -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- Forearm Link -->
  <link name="forearm">
    <visual>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.04 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>
  
  <!-- Elbow Joint (connects upper arm to forearm) -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Rotates around Y-axis (pitch) -->
    <limit lower="0.0" upper="2.6" effort="50" velocity="1.0"/>
  </joint>
  
</robot>
```

**Explanation**:
- **Lines 6 to 27**: \`base_link\` - gray cylinder base (0.1m radius, 0.05m height)
- **Lines 29 to 50**: \`upper_arm\` - blue box (0.3m long)
- **Lines 52 to 58**: \`shoulder_joint\` - revolute joint rotating around Y-axis (-90° to 90°)
- **Lines 60 to 81**: \`forearm\` - red box (0.25m long)
- **Lines 83 to 89**: \`elbow_joint\` - revolute joint (0° to 150°)

### Visualize in RViz2

**Step 1: Install joint_state_publisher GUI**
```bash
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-xacro
```

**Step 2: Launch RViz2 with joint state publisher**
```bash
# Terminal 1: Publish robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_arm.urdf)"

# Terminal 2: Publish joint states (manual control)
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: Visualize in RViz2
rviz2
```

**Step 3: Configure RViz2**
1. Click "Add" → "RobotModel"
2. In RobotModel settings, set "Description Topic" to \`/robot_description\`
3. Change "Fixed Frame" to \`base_link\`
4. You should see the 2-link arm!

**Step 4: Control joints**
- In the \`joint_state_publisher_gui\` window, drag sliders to move shoulder and elbow
- RViz2 updates in real-time

---

### Project 2: Humanoid Torso with Arms

**Goal**: Model a simplified humanoid upper body with torso, shoulders, and arms.

**File**: \`humanoid_torso.urdf\`

```xml
<?xml version="1.0"?>
<robot name="humanoid_torso">
  
  <!-- Torso (Base) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.4 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.4 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0"
               iyy="0.3" iyz="0.0" izz="0.25"/>
    </inertial>
  </link>
  
  <!-- Left Shoulder -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Left Shoulder Joint (pitch) -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0 0.25 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Left Shoulder to Upper Arm (roll for rotation) -->
  <joint name="left_shoulder_roll" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Roll axis -->
    <limit lower="-2.0" upper="2.0" effort="50" velocity="1.0"/>
  </joint>
  
  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="light_blue">
        <color rgba="0.4 0.4 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.7"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>
  
  <!-- Left Elbow Joint -->
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.6" effort="50" velocity="1.0"/>
  </joint>
  
  <!-- Mirror for right arm (abbreviated for space) -->
  <!-- In practice, duplicate left arm structure with mirrored Y positions -->
  
</robot>
```

**Key Features**:
- **Torso**: White box as base link
- **Shoulder joint**: 2-DOF (pitch + roll for realistic arm motion)
- **Upper arm**: Blue cylinder
- **Forearm**: Light blue cylinder
- **Realistic joint limits**: Based on human anatomy

**Visualize**:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat humanoid_torso.urdf)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2
```

---

### Project 3: Adding Sensors (Camera)

**Goal**: Attach a camera sensor to the robot's head.

```xml
<!-- Head Link -->
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.12"/>
    </geometry>
    <material name="skin">
      <color rgba="0.9 0.75 0.65 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.12"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0"
             iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>

<!-- Neck Joint -->
<joint name="neck" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.4" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Yaw (head turns left/right) -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>

<!-- Camera Sensor (attached to head) -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
             iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<!-- Camera Mount (fixed to head) -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.12 0 0" rpy="0 0 0"/>  <!-- 12cm in front of head center -->
</joint>

<!-- Gazebo Camera Plugin (for simulation) -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>/camera/image_raw</imageTopicName>
      <cameraInfoTopicName>/camera/camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

**Explanation**:
- **Lines 1 to 22**: Head link (sphere)
- **Lines 24 to 30**: Neck joint (rotates head left/right)
- **Lines 32 to 50**: Camera link (small box)
- **Lines 52 to 56**: Fixed joint mounting camera to head
- **Lines 58 to 80**: Gazebo camera plugin (for simulation in Module 2)

---

### Step 3: Debugging Common Issues

#### Issue 1: RViz2 shows "No transform from [link] to [fixed frame]"
**Cause**: \`fixed_frame\` set to non-existent link

**Solution**:
- In RViz2, change "Fixed Frame" to \`base_link\` or \`torso\`
- Run \`ros2 run tf2_tools view_frames\` to see available frames

#### Issue 2: Joint doesn't move in joint_state_publisher_gui
**Cause**: Joint type is \`fixed\` instead of \`revolute\`

**Solution**:
```xml
<!-- Wrong: -->
<joint name="shoulder" type="fixed">

<!-- Correct: -->
<joint name="shoulder" type="revolute">
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

#### Issue 3: "Unable to parse robot model" error
**Cause**: XML syntax error (missing closing tag, typo)

**Solution**:
```bash
# Validate URDF syntax
check_urdf simple_arm.urdf

# If check_urdf not found:
sudo apt install liburdfdom-tools
```

#### Issue 4: Robot appears inside-out or distorted
**Cause**: Incorrect inertia matrix or negative mass

**Solution**:
- Mass must be > 0
- Inertia diagonal elements (ixx, iyy, izz) must be > 0
- For primitive shapes, use standard formulas:
  - **Box**: \`ixx = (m/12) * (h² + d²)\`
  - **Cylinder**: \`ixx = (m/12) * (3r² + h²)\`
  - **Sphere**: \`ixx = iyy = izz = (2/5) * m * r²\`

## Part 3: Advanced Topics (Optional)

### Using Xacro for Macros

URDF is verbose. **Xacro** (XML Macros) adds:
- Variables
- Math expressions
- Macros (reusable templates)

**Example**: Define arm as macro, instantiate for left and right

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <!-- Properties (variables) -->
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="arm_radius" value="0.04"/>
  
  <!-- Macro for arm -->
  <xacro:macro name="arm" params="side reflect">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder radius="${arm_radius}" length="${arm_length}"/>
        </geometry>
      </visual>
    </link>
    
    <joint name="${side}_shoulder" type="revolute">
      <origin xyz="0 ${reflect * 0.25} 0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    </joint>
  </xacro:macro>
  
  <!-- Instantiate for both arms -->
  <xacro:arm side="left" reflect="1"/>
  <xacro:arm side="right" reflect="-1"/>
  
</robot>
```

**Convert xacro to URDF**:
```bash
xacro humanoid.xacro > humanoid.urdf
```

### Loading Meshes

For realistic appearance, use STL or DAE mesh files:

```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot_description/meshes/upper_arm.stl" scale="1.0 1.0 1.0"/>
  </geometry>
</visual>
```

**Mesh file sources**:
- CAD software (Fusion 360, SolidWorks) → export as STL
- 3D scanning of real robots
- Open-source repositories (GrabCAD, Thingiverse)

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Humanoid model**: Week 13 capstone uses a complete URDF with:
  - Torso, head, arms, legs (20+ links, 18+ DOF)
  - Camera sensor for object detection
  - LiDAR for navigation
  - IMU for balance control

- **Simulation**: URDF loaded into Gazebo (Module 2) and Isaac Sim (Module 3)
  ```bash
  # Spawn in Gazebo
  ros2 launch gazebo_ros spawn_entity.py -entity humanoid -file humanoid.urdf
  ```

- **Motion planning**: MoveIt 2 uses URDF for inverse kinematics
  - "Move right hand to (x, y, z)" → MoveIt calculates joint angles

Understanding URDF is essential for any robot project in ROS 2.

## Summary

You learned:
- ✅ Defined robot structure using **URDF** (links, joints, sensors)
- ✅ Modeled **revolute**, **fixed**, and **continuous** joints
- ✅ Added **visual**, **collision**, and **inertial** properties
- ✅ Attached **sensors** (camera) to robot links
- ✅ Visualized URDF in **RViz2** with interactive joint control

**Next steps**: In Chapter 1.4, you'll package URDF files in ROS 2 packages and create launch files to automate visualization.

---

## Exercises

### Exercise 1: 3-Link Leg (Required)

**Objective**: Model a simple humanoid leg with hip, knee, and ankle joints.

**Tasks**:
1. Create \`simple_leg.urdf\`
2. Define links: \`hip\`, \`thigh\`, \`shin\`, \`foot\`
3. Joints:
   - \`hip_joint\`: revolute, pitch (flexion/extension)
   - \`knee_joint\`: revolute, pitch (0° to 150°)
   - \`ankle_joint\`: revolute, pitch (-30° to 45°)
4. Visualize in RViz2

**Acceptance Criteria**:
- [ ] Leg has 3 movable joints
- [ ] Joint limits prevent unnatural poses
- [ ] RViz2 displays correctly with sliders

**Estimated Time**: 60 minutes

### Exercise 2: Add IMU Sensor (Challenge)

**Objective**: Attach an IMU (Inertial Measurement Unit) to the torso.

**Tasks**:
1. Add \`imu_link\` (small box, 2cm × 2cm × 1cm)
2. Fixed joint mounting IMU to \`torso\` at origin
3. Add Gazebo IMU plugin:

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <update_rate>100.0</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Acceptance Criteria**:
- [ ] IMU link visible in RViz2
- [ ] URDF validates with \`check_urdf\`

**Estimated Time**: 45 minutes

### Exercise 3: Complete Humanoid Skeleton (Advanced)

**Objective**: Build a full humanoid URDF with arms, legs, torso, and head.

**Requirements**:
- **18 DOF total**:
  - Neck: 1 DOF (yaw)
  - Arms: 2 × 3 DOF (shoulder pitch/roll, elbow)
  - Torso: 1 DOF (waist rotation)
  - Legs: 2 × 5 DOF (hip pitch/roll/yaw, knee, ankle)

**Hints**:
- Use \`xacro\` macros to avoid duplication
- Start with torso, add head, then mirror arms and legs

**Acceptance Criteria**:
- [ ] All 18 joints controllable in \`joint_state_publisher_gui\`
- [ ] Realistic joint limits based on human anatomy
- [ ] Clean visualization in RViz2

**Estimated Time**: 3 to 4 hours (spread over multiple days)

---

## Additional Resources

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) - Official ROS 2 tutorials
- [Xacro Documentation](http://wiki.ros.org/xacro) - XML macros for URDF
- [REP-103: Coordinate Frames](https://www.ros.org/reps/rep-0103.html) - Standard frames for mobile robots
- [URDF Visual Studio Code Extension](https://marketplace.visualstudio.com/items?itemName=smilerobotics.urdf) - Syntax highlighting and validation
- [Online URDF Validator](http://wiki.ros.org/urdf/Tutorials/Using%20urdf%20with%20robot_state_publisher) - Debug XML syntax

---

**Previous**: [← Chapter 1.2: Python Integration with rclpy](chapter-1 to 2.md) | **Next**: [Chapter 1.4: Package Development & Launch Files →](chapter-1 to 4.md)
