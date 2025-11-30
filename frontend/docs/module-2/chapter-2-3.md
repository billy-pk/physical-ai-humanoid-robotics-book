---
sidebar_position: 4
title: 2.3 Sensor Simulation (LiDAR, Cameras, IMUs)
---

# Chapter 2.3: Sensor Simulation (LiDAR, Cameras, IMUs)

Sensors are the eyes and ears of humanoid robots. Gazebo provides realistic sensor models that publish ROS 2 topics, enabling you to develop perception algorithms in simulation before deploying to hardware. This chapter covers camera, LiDAR, and IMU sensor plugins.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Add** camera plugins to robot models publishing ROS 2 Image topics
- **Configure** LiDAR sensors for SLAM and obstacle detection
- **Simulate** IMU data for balance control and orientation estimation
- **Calibrate** sensor noise models to match real hardware
- **Process** sensor data in ROS 2 nodes for perception pipelines

## Prerequisites

- **Gazebo Classic 11** with ROS 2 integration (Chapters 2.1 to 2.2)
- **ROS 2 Humble** with `rclpy` experience (Module 1)
- **Understanding** of ROS 2 topics and messages (Module 1)
- **Basic image processing** concepts (pixels, frames, resolution)

## Part 1: Sensor Plugin Architecture

### How Gazebo Sensors Work

**Gazebo sensors** are plugins that:
1. **Simulate** physical sensor behavior (optics, noise, latency)
2. **Publish** ROS 2 topics with sensor data
3. **Configure** via SDF/URDF parameters (resolution, frame rate, noise)

**Sensor types**:
- **Camera**: RGB images, depth images, stereo pairs
- **LiDAR**: 2D/3D laser scans for SLAM
- **IMU**: Accelerometer, gyroscope, magnetometer
- **Force/Torque**: Contact sensors for manipulation
- **GPS**: Localization (for outdoor robots)

### ROS 2 Message Types

| Sensor | Gazebo Plugin | ROS 2 Message Type | Package |
|--------|---------------|---------------------|---------|
| **Camera** | `libgazebo_ros_camera.so` | `sensor_msgs/Image` | sensor_msgs |
| **Depth Camera** | `libgazebo_ros_depth_camera.so` | `sensor_msgs/Image` (16-bit) | sensor_msgs |
| **LiDAR** | `libgazebo_ros_ray_sensor.so` | `sensor_msgs/LaserScan` | sensor_msgs |
| **IMU** | `libgazebo_ros_imu_sensor.so` | `sensor_msgs/Imu` | sensor_msgs |

### Sensor Noise Models

**Real sensors have noise**. Gazebo models:
- **Gaussian noise**: Random variations (like camera sensor noise)
- **Bias**: Systematic offset (like IMU drift)
- **Quantization**: Discrete steps (like digital sensors)

**Why noise matters**:
- **Realistic testing**: Algorithms must work with noisy data
- **Robustness**: Test failure modes (low light, sensor failures)
- **Calibration**: Match simulation to real sensor characteristics

## Part 2: Hands-On Tutorial

### Project: Add Sensors to Humanoid Robot

**Goal**: Add camera, LiDAR, and IMU sensors to the humanoid model and verify ROS 2 topic publishing.

**Tools**: Gazebo Classic 11, ROS 2 Humble, URDF/SDF

### Step 1: Add Camera Sensor to Head

**Modify URDF** (`models/simple_humanoid/model.urdf`):

Add head link and camera:
```xml
<!-- Head Link -->
<link name="head">
  <visual name="head_visual">
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision name="head_collision">
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<!-- Head Joint -->
<joint name="head_joint" type="fixed">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>

<!-- Camera Sensor (Gazebo Plugin) -->
<gazebo reference="head">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>  <!-- 30 Hz -->
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.05</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
      </ros>
      <camera_name>head_camera</camera_name>
      <frame_name>head_camera_frame</frame_name>
      <hack_baseline>0.07</hack_baseline>
      <min_depth>0.05</min_depth>
      <max_depth>100</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

**Key parameters**:
- **`update_rate`**: Frames per second (30 Hz = real-time video)
- **`horizontal_fov`**: Field of view in radians (1.047 ≈ 60°)
- **`width/height`**: Image resolution (640×480 = VGA)
- **`frame_name`**: TF frame for camera (for coordinate transforms)

### Step 2: Add LiDAR Sensor to Torso

**Add LiDAR** (after camera sensor):

```xml
<!-- LiDAR Sensor (Gazebo Plugin) -->
<gazebo reference="torso">
  <sensor name="lidar" type="ray">
    <pose>0 0 0.2 0 0 0</pose>  <!-- Position on torso -->
    <visualize>true</visualize>  <!-- Show rays in Gazebo -->
    <update_rate>10</update_rate>  <!-- 10 Hz -->
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>  <!-- 360 points per scan -->
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180° -->
          <max_angle>3.14159</max_angle>   <!-- +180° -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>   <!-- 10 cm minimum -->
        <max>10.0</max>  <!-- 10 m maximum -->
        <resolution>0.01</resolution>  <!-- 1 cm resolution -->
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>scan:=lidar/scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Key parameters**:
- **`samples`**: Number of laser rays (360 = 1° resolution)
- **`min/max_angle`**: Scanning range (-180° to +180° = full circle)
- **`min/max` range**: Detection distance (0.1 to 10 m)
- **`visualize`**: Show laser rays in Gazebo GUI (helpful for debugging)

### Step 3: Add IMU Sensor to Torso

**Add IMU** (after LiDAR sensor):

```xml
<!-- IMU Sensor (Gazebo Plugin) -->
<gazebo reference="torso">
  <sensor name="imu" type="imu">
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>imu:=imu/data</remapping>
      </ros>
      <frame_name>imu_frame</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>  <!-- Gyroscope noise -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- Accelerometer noise -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

**Key parameters**:
- **`update_rate`**: IMU frequency (100 Hz typical for balance control)
- **`noise`**: Gaussian noise model (matches real IMU characteristics)
- **`stddev`**: Standard deviation (higher = more noise)
- **`initial_orientation_as_reference`**: Use starting pose as reference frame

### Step 4: Launch Robot and Verify Sensors

**Launch file** (`launch/humanoid_sensors.launch.py`):

```python
#!/usr/bin/env python3
"""
Launch humanoid with sensors in Gazebo
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
    urdf_path = os.path.join(pkg_share, 'models', 'simple_humanoid', 'model.urdf')

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'
        ),
        
        # Spawn humanoid
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '1.0'],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        )
    ])
```

**Launch and verify**:
```bash
cd ~/gazebo_ws
colcon build --packages-select gazebo_worlds
source install/setup.bash

ros2 launch gazebo_worlds humanoid_sensors.launch.py
```

**Terminal 2: Check ROS 2 topics**:
```bash
source /opt/ros/humble/setup.bash

# List all topics
ros2 topic list

# You should see:
# /humanoid/camera/image_raw
# /humanoid/lidar/scan
# /humanoid/imu/data
# /clock
# /model_states
```

**Verify camera**:
```bash
# Check camera topic info
ros2 topic info /humanoid/camera/image_raw

# Echo camera messages (header info)
ros2 topic echo /humanoid/camera/image_raw --no-arr | head -20
```

**Verify LiDAR**:
```bash
# Check LiDAR topic
ros2 topic info /humanoid/lidar/scan

# Echo LiDAR scan
ros2 topic echo /humanoid/lidar/scan --no-arr | head -30
```

**Verify IMU**:
```bash
# Check IMU topic
ros2 topic info /humanoid/imu/data

# Echo IMU data
ros2 topic echo /humanoid/imu/data
```

**Expected Output** (IMU):
```
header:
  stamp:
    sec: 0
    nanosec: 123456789
  frame_id: imu_frame
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
angular_velocity:
  x: 0.001
  y: -0.002
  z: 0.0005
linear_acceleration:
  x: 0.01
  y: -0.02
  z: -9.81  <!-- Gravity in Z direction -->
```

### Step 5: Visualize Sensor Data in RViz2

**Launch RViz2**:
```bash
rviz2
```

**Configure displays**:
1. **Add Image display**:
   - Topic: `/humanoid/camera/image_raw`
   - You should see camera feed from robot's perspective

2. **Add LaserScan display**:
   - Topic: `/humanoid/lidar/scan`
   - You should see 2D laser scan visualization

3. **Add TF display**:
   - Shows coordinate frames (camera_frame, lidar_frame, imu_frame)

**Expected Result**:
- Camera shows robot's view of Gazebo world
- LiDAR shows distance measurements as colored points
- IMU data visible in terminal (orientation, angular velocity, acceleration)

### Step 6: Process Sensor Data in ROS 2 Node

**Create sensor processor node**: `src/sensor_processor.py`

```python
#!/usr/bin/env python3
"""
Process sensor data from humanoid robot
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
import cv2
from cv_bridge import CvBridge
import numpy as np

class SensorProcessor(Node):
    """
    Processes camera, LiDAR, and IMU data from humanoid robot
    """
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid/lidar/scan',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        self.get_logger().info('Sensor processor node started')
    
    def camera_callback(self, msg):
        """Process camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            
            # Example: Detect edges
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            
            # Log image info
            self.get_logger().info(
                f'Camera: {width}x{height}, edges detected: {np.sum(edges > 0)} pixels'
            )
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')
    
    def lidar_callback(self, msg):
        """Process LiDAR scans"""
        # Get ranges (distances)
        ranges = np.array(msg.ranges)
        
        # Filter invalid readings (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            min_dist = np.min(valid_ranges)
            max_dist = np.max(valid_ranges)
            avg_dist = np.mean(valid_ranges)
            
            self.get_logger().info(
                f'LiDAR: min={min_dist:.2f}m, max={max_dist:.2f}m, avg={avg_dist:.2f}m'
            )
            
            # Detect obstacles (within 1 meter)
            obstacles = valid_ranges[valid_ranges < 1.0]
            if len(obstacles) > 0:
                self.get_logger().warn(f'Obstacle detected! {len(obstacles)} points < 1m')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract linear acceleration (includes gravity)
        accel = msg.linear_acceleration
        accel_magnitude = np.sqrt(
            accel.x**2 + accel.y**2 + accel.z**2
        )
        
        # Extract angular velocity
        gyro = msg.angular_velocity
        
        # Log IMU data
        self.get_logger().info(
            f'IMU: accel_mag={accel_magnitude:.2f} m/s², '
            f'gyro=({gyro.x:.3f}, {gyro.y:.3f}, {gyro.z:.3f}) rad/s'
        )
        
        # Detect fall (high acceleration)
        if accel_magnitude > 15.0:
            self.get_logger().warn('High acceleration detected - possible fall!')

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Add dependencies** (`package.xml`):
```xml
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>opencv-python</depend>
```

**Install OpenCV**:
```bash
sudo apt install ros-humble-cv-bridge python3-opencv
```

**Run sensor processor**:
```bash
cd ~/gazebo_ws
colcon build --packages-select gazebo_worlds
source install/setup.bash

# Terminal 1: Launch Gazebo + robot
ros2 launch gazebo_worlds humanoid_sensors.launch.py

# Terminal 2: Run sensor processor
ros2 run gazebo_worlds sensor_processor
```

**Expected Output**:
```
[INFO] [sensor_processor]: Sensor processor node started
[INFO] [sensor_processor]: Camera: 640x480, edges detected: 15234 pixels
[INFO] [sensor_processor]: LiDAR: min=0.15m, max=8.5m, avg=4.2m
[INFO] [sensor_processor]: IMU: accel_mag=9.81 m/s², gyro=(0.001, -0.002, 0.0005) rad/s
```

### Step 7: Debugging Common Sensor Issues

#### Issue 1: Camera Not Publishing
**Symptoms**: No `/humanoid/camera/image_raw` topic

**Solutions**:
```bash
# Check if plugin loaded
gazebo --verbose  # Look for plugin loading messages

# Verify plugin filename
# Should be: libgazebo_ros_camera.so

# Check ROS namespace
ros2 topic list | grep camera
```

#### Issue 2: LiDAR Shows No Data
**Symptoms**: Empty scan ranges

**Solutions**:
```xml
<!-- Ensure visualize=true for debugging -->
<visualize>true</visualize>

<!-- Check range limits -->
<min>0.1</min>
<max>10.0</max>

<!-- Verify sensor pose (should face forward) -->
<pose>0 0 0.2 0 0 0</pose>
```

#### Issue 3: IMU Shows Zero Values
**Symptoms**: All IMU readings are 0.0

**Solutions**:
```xml
<!-- Ensure always_on=true -->
<always_on>true</always_on>

<!-- Check update_rate -->
<update_rate>100</update_rate>

<!-- Verify sensor reference -->
<frame_name>imu_frame</frame_name>
```

#### Issue 4: Sensor Data Too Noisy
**Symptoms**: Unrealistic noise levels

**Solutions**:
```xml
<!-- Reduce noise standard deviation -->
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.001</stddev>  <!-- Lower = less noise -->
</noise>
```

## Part 3: Advanced Topics (Optional)

### Depth Camera (RGB-D)

**Add depth camera** (for 3D perception):
```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.05</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_depth_camera.so">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>depth/image_raw:=depth/image_raw</remapping>
    </ros>
    <output_type>sensor_msgs/Image</output_type>
    <frame_name>depth_camera_frame</frame_name>
  </plugin>
</sensor>
```

**Use case**: Object manipulation, 3D mapping, obstacle avoidance

### Stereo Camera Pair

**Add stereo cameras** (for depth estimation):
```xml
<!-- Left camera -->
<sensor name="left_camera" type="camera">
  <pose>-0.05 0 0 0 0 0</pose>  <!-- Left eye -->
  <!-- ... camera config ... -->
</sensor>

<!-- Right camera -->
<sensor name="right_camera" type="camera">
  <pose>0.05 0 0 0 0 0</pose>  <!-- Right eye -->
  <!-- ... camera config ... -->
</sensor>
```

**Use case**: Stereo vision for 3D perception

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Vision pipeline**: Capstone will use camera for object detection and manipulation
- **SLAM**: LiDAR data for mapping and localization
- **Balance control**: IMU data for maintaining upright posture
- **Multi-modal perception**: Combining camera + LiDAR + IMU for robust navigation

Understanding sensor simulation now is essential for developing the capstone perception system.

## Summary

You learned:
- ✅ Added **camera, LiDAR, and IMU sensors** to humanoid model
- ✅ Configured **sensor plugins** publishing ROS 2 topics
- ✅ Verified **sensor data** using ROS 2 introspection tools
- ✅ Processed **sensor data** in Python ROS 2 nodes
- ✅ Debugged **common sensor issues** (missing topics, noise)

**Next steps**: In Chapter 2.4, you'll convert URDF to SDF format and optimize robot models for Gazebo.

---

## Exercises

### Exercise 1: Camera Calibration (Required)

**Objective**: Configure camera parameters to match real hardware.

**Tasks**:
1. Research specs for a real camera (e.g., Intel RealSense D435)
2. Configure Gazebo camera to match:
   - Resolution (e.g., 1920×1080)
   - Field of view (e.g., 87°)
   - Frame rate (e.g., 30 Hz)
3. Verify image dimensions match specification
4. Test with different resolutions (VGA, HD, 4K)

**Acceptance Criteria**:
- [ ] Camera resolution matches specification
- [ ] Field of view calculated correctly
- [ ] Frame rate verified with `ros2 topic hz`

**Estimated Time**: 60 minutes

### Exercise 2: LiDAR Obstacle Detection (Required)

**Objective**: Process LiDAR data to detect obstacles.

**Tasks**:
1. Create ROS 2 node subscribing to `/humanoid/lidar/scan`
2. Detect obstacles within 2 meters
3. Calculate obstacle angle (relative to robot)
4. Publish obstacle locations to new topic
5. Visualize obstacles in RViz2

**Acceptance Criteria**:
- [ ] Node detects obstacles correctly
- [ ] Obstacle angles calculated accurately
- [ ] Visualization shows detected obstacles

**Estimated Time**: 90 minutes

### Exercise 3: IMU Balance Detection (Challenge)

**Objective**: Use IMU data to detect robot balance state.

**Tasks**:
1. Subscribe to `/humanoid/imu/data`
2. Calculate tilt angle from linear acceleration
3. Detect if robot is falling (high angular velocity)
4. Implement fall detection algorithm
5. Publish balance status topic

**Hints**:
- Tilt angle: `atan2(accel.x, accel.z)`
- Fall threshold: Angular velocity > 2.0 rad/s

**Estimated Time**: 120 minutes

---

## Additional Resources

- [Gazebo Sensors](https://gazebosim.org/docs/latest/sensors) - Sensor documentation
- [ROS 2 Sensor Messages](https://docs.ros2.org/humble/api/sensor_msgs/) - Message types
- [CV Bridge](http://wiki.ros.org/cv_bridge) - ROS-OpenCV integration
- [IMU Calibration](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-sources/sensor-error-sources) - Real-world noise models

---

**Next**: [Chapter 2.4: URDF/SDF Robot Description →](chapter-2 to 4.md)
