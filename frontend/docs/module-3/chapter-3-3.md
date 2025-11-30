---
sidebar_position: 4
title: 3.3 Isaac ROS - Hardware-Accelerated VSLAM
---

# Chapter 3.3: Isaac ROS - Hardware-Accelerated VSLAM

Visual SLAM (Simultaneous Localization and Mapping) enables humanoid robots to build maps and localize themselves using camera data. Isaac ROS provides GPU-accelerated VSLAM algorithms that run at real-time speeds—essential for autonomous navigation in GPS-denied environments.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Install** and configure Isaac ROS packages for VSLAM
- **Set up** stereo VSLAM using GPU-accelerated algorithms
- **Integrate** VSLAM with ROS 2 navigation stack
- **Visualize** VSLAM output (maps, trajectories) in RViz2
- **Debug** VSLAM performance and accuracy issues

## Prerequisites

- **Isaac Sim** or **stereo cameras** (physical or simulated)
- **NVIDIA GPU** with CUDA support (RTX 2060 or better)
- **ROS 2 Humble** configured
- **Understanding** of SLAM concepts (mapping, localization, loop closure)
- **Basic computer vision** knowledge (features, descriptors, matching)

## Part 1: VSLAM Fundamentals

### What is VSLAM?

**VSLAM (Visual SLAM)** uses camera images to:
- **Build maps** of the environment (3D structure)
- **Localize** the robot within the map (6DOF pose)
- **Track** camera motion in real-time

**Why VSLAM for humanoids?**
- **GPS-denied**: Works indoors and in urban canyons
- **Visual**: Uses cameras (cheaper than LiDAR)
- **Rich information**: Provides 3D structure for manipulation
- **Real-time**: GPU acceleration enables real-time performance

### VSLAM Pipeline

**Typical VSLAM pipeline**:
1. **Feature Detection**: Find keypoints (corners, edges) in images
2. **Feature Matching**: Match features between frames
3. **Motion Estimation**: Estimate camera pose from matches
4. **Map Building**: Add new 3D points to map
5. **Loop Closure**: Detect revisited locations
6. **Optimization**: Refine map and poses (bundle adjustment)

### Isaac ROS VSLAM Packages

| Package | Function | GPU Acceleration |
|---------|----------|------------------|
| **isaac_ros_visual_slam** | Main VSLAM node | Yes (CUDA) |
| **isaac_ros_stereo_image_proc** | Stereo rectification | Yes |
| **isaac_ros_image_proc** | Image preprocessing | Yes |
| **isaac_ros_nitros** | Zero-copy messaging | Yes |

**Key advantage**: GPU acceleration enables real-time performance (30+ FPS).

## Part 2: Hands-On Tutorial

### Project: Set Up Stereo VSLAM System

**Goal**: Configure Isaac ROS VSLAM with stereo cameras and visualize localization output.

**Tools**: Isaac ROS, ROS 2 Humble, NVIDIA GPU, stereo cameras (or Isaac Sim)

### Step 1: Install Isaac ROS Packages

**Create workspace**:
```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
```

**Clone Isaac ROS repositories**:
```bash
# Core packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_proc.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_stereo_image_proc.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Install dependencies
cd ~/isaac_ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Build workspace**:
```bash
cd ~/isaac_ros_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Verify installation**:
```bash
ros2 pkg list | grep isaac
# Should show: isaac_ros_visual_slam, isaac_ros_stereo_image_proc, etc.
```

### Step 2: Set Up Stereo Cameras

**Option A: Physical Stereo Camera** (e.g., Intel RealSense D435):
```bash
# Install RealSense ROS 2 driver
sudo apt install ros-humble-realsense2-camera

# Launch camera
ros2 launch realsense2_camera rs_launch.py
```

**Option B: Isaac Sim Stereo Cameras**:
```python
# In Isaac Sim Python script
from omni.isaac.sensor import Camera

# Left camera
left_camera = Camera(
    prim_path="/World/LeftCamera",
    position=[-0.05, 0, 1.6],  # Left eye
    resolution=(1280, 720),
    frequency=30
)

# Right camera
right_camera = Camera(
    prim_path="/World/RightCamera",
    position=[0.05, 0, 1.6],  # Right eye
    resolution=(1280, 720),
    frequency=30
)
```

**Verify stereo topics**:
```bash
ros2 topic list | grep camera
# Should see:
# /left/camera/image_raw
# /right/camera/image_raw
# /left/camera/camera_info
# /right/camera/camera_info
```

### Step 3: Configure Stereo Image Processing

**Create launch file**: `launch/stereo_vslam.launch.py`

```python
#!/usr/bin/env python3
"""
Launch stereo VSLAM system
ROS 2 Humble | Isaac ROS
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Stereo image proc (rectification)
        Node(
            package='isaac_ros_stereo_image_proc',
            executable='isaac_ros_stereo_image_proc',
            name='stereo_image_proc',
            parameters=[{
                'left_camera_namespace': '/left',
                'right_camera_namespace': '/right',
                'left_camera_name': 'camera',
                'right_camera_name': 'camera',
            }],
            remappings=[
                ('left/image_rect', '/left/image_rect'),
                ('right/image_rect', '/right/image_rect'),
                ('left/camera_info', '/left/camera_info'),
                ('right/camera_info', '/right/camera_info'),
            ]
        ),
        
        # Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_rectified_pose': True,
                'denoise_input_images': False,
                'rectified_images': True,
                'enable_imu': False,  # Set True if IMU available
                'enable_slam_visualization': True,
                'enable_landmarks_view': True,
                'enable_observations_view': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'input_left_camera_frame': 'left_camera_frame',
                'input_right_camera_frame': 'right_camera_frame',
            }],
            remappings=[
                ('stereo_camera/left/image', '/left/image_rect'),
                ('stereo_camera/right/image', '/right/image_rect'),
                ('stereo_camera/left/camera_info', '/left/camera_info'),
                ('stereo_camera/right/camera_info', '/right/camera_info'),
            ]
        )
    ])
```

**Launch**:
```bash
source ~/isaac_ros_ws/install/setup.bash
ros2 launch your_package stereo_vslam.launch.py
```

### Step 4: Verify VSLAM Output

**Check VSLAM topics**:
```bash
ros2 topic list | grep vslam
# Should see:
# /visual_slam/tracking/odometry
# /visual_slam/tracking/pose
# /visual_slam/tracking/status
# /visual_slam/map
```

**Echo odometry**:
```bash
ros2 topic echo /visual_slam/tracking/odometry
```

**Expected Output**:
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: 0.123
      y: 0.456
      z: 0.789
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
twist:
  linear:
    x: 0.1
    y: 0.0
    z: 0.0
  angular:
    z: 0.05
```

**Check VSLAM status**:
```bash
ros2 topic echo /visual_slam/tracking/status
```

**Status meanings**:
- **TRACKING**: VSLAM is tracking successfully
- **LOST**: Tracking lost (insufficient features)
- **INITIALIZING**: Still initializing (first few frames)

### Step 5: Visualize in RViz2

**Launch RViz2**:
```bash
rviz2
```

**Configure displays**:
1. **Add TF**: Shows coordinate frames (map → odom → base_link)
2. **Add Odometry**: Topic `/visual_slam/tracking/odometry`
3. **Add Map**: Topic `/visual_slam/map` (if available)
4. **Add Camera**: Topic `/left/image_rect` (for visual feedback)

**Expected visualization**:
- **TF tree**: Shows robot pose in map frame
- **Odometry arrow**: Shows robot position and orientation
- **Trajectory**: Path robot has traveled (if enabled)

### Step 6: Integrate with Robot

**Publish VSLAM pose to robot**:
```python
#!/usr/bin/env python3
"""
Bridge VSLAM pose to robot state
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class VSLAMPoseBridge(Node):
    def __init__(self):
        super().__init__('vslam_pose_bridge')
        
        # Subscribe to VSLAM odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )
        
        # Publish TF transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
    def odom_callback(self, msg):
        """Publish VSLAM pose as TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = VSLAMPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 7: Debugging Common Issues

#### Issue 1: "VSLAM status: LOST"
**Symptoms**: Tracking fails immediately or after few frames

**Solutions**:
```bash
# Check camera topics are publishing
ros2 topic hz /left/image_rect
ros2 topic hz /right/image_rect

# Verify camera info is correct
ros2 topic echo /left/camera_info

# Check image quality (should have features)
ros2 run rqt_image_view rqt_image_view /left/image_rect
```

**Common causes**:
- **Low texture**: Scene has no features (blank walls)
- **Motion blur**: Camera moving too fast
- **Poor lighting**: Images too dark/bright
- **Calibration**: Incorrect camera intrinsics

#### Issue 2: "VSLAM not publishing odometry"
**Symptoms**: No `/visual_slam/tracking/odometry` topic

**Solutions**:
```bash
# Check node is running
ros2 node list | grep visual_slam

# Check node logs
ros2 node info /visual_slam

# Verify GPU is available
nvidia-smi
```

#### Issue 3: "Poor localization accuracy"
**Symptoms**: Robot position drifts over time

**Solutions**:
```python
# Increase feature detection
# In launch file parameters:
'enable_rectified_pose': True,
'denoise_input_images': True,  # Enable denoising

# Use IMU if available (reduces drift)
'enable_imu': True,
```

**Calibration**:
- **Stereo calibration**: Ensure cameras are properly calibrated
- **Baseline**: Correct stereo baseline distance
- **Intrinsics**: Accurate camera matrix and distortion

#### Issue 4: "High CPU/GPU usage"
**Symptoms**: System lagging, high resource usage

**Solutions**:
```python
# Reduce image resolution
# In camera configuration:
resolution=(640, 480)  # Instead of (1280, 720)

# Reduce frame rate
frequency=15  # Instead of 30 Hz
```

## Part 3: Advanced Topics (Optional)

### IMU Integration

**Add IMU to VSLAM** (reduces drift):
```python
# In launch file
Node(
    package='isaac_ros_visual_slam',
    executable='isaac_ros_visual_slam',
    parameters=[{
        'enable_imu': True,  # Enable IMU fusion
    }],
    remappings=[
        # ... (camera topics)
        ('imu', '/imu/data'),  # IMU topic
    ]
)
```

**Benefits**:
- **Reduced drift**: IMU provides motion priors
- **Faster initialization**: IMU helps with initial motion
- **Robustness**: Works better in low-texture environments

### Loop Closure Detection

**Enable loop closure** (for map consistency):
```python
parameters=[{
    'enable_loop_closure': True,
    'loop_closure_search_radius': 5.0,  # meters
    'loop_closure_min_inliers': 50,
}]
```

**Benefits**:
- **Map consistency**: Corrects accumulated drift
- **Global optimization**: Refines entire map
- **Relocalization**: Can relocalize after tracking loss

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Localization**: Capstone will use VSLAM for real-time pose estimation
- **Mapping**: Build maps of indoor environments for navigation
- **Robustness**: GPU acceleration enables real-time performance
- **Integration**: VSLAM provides pose for Nav2 navigation (Chapter 3.4)

Understanding VSLAM now is essential for the capstone navigation system.

## Summary

You learned:
- ✅ Installed **Isaac ROS packages** for VSLAM
- ✅ Configured **stereo VSLAM** with GPU acceleration
- ✅ Integrated **VSLAM with ROS 2** navigation stack
- ✅ Visualized **VSLAM output** (odometry, maps) in RViz2
- ✅ Debugged **VSLAM performance** and accuracy issues

**Next steps**: In Chapter 3.4, you'll configure Nav2 navigation stack for bipedal humanoid path planning.

---

## Exercises

### Exercise 1: Basic VSLAM Setup (Required)

**Objective**: Set up stereo VSLAM and verify tracking.

**Tasks**:
1. Install Isaac ROS packages
2. Configure stereo cameras (physical or simulated)
3. Launch VSLAM node
4. Verify odometry publishing
5. Visualize trajectory in RViz2

**Acceptance Criteria**:
- [ ] VSLAM node running without errors
- [ ] Odometry topic publishing at 30+ Hz
- [ ] Status shows "TRACKING"
- [ ] Trajectory visible in RViz2

**Estimated Time**: 90 minutes

### Exercise 2: VSLAM Accuracy Test (Required)

**Objective**: Measure VSLAM localization accuracy.

**Tasks**:
1. Move robot in known pattern (square, circle)
2. Record VSLAM odometry
3. Compare estimated path to ground truth
4. Calculate position error (RMSE)
5. Document accuracy results

**Metrics**:
- Position error (meters)
- Orientation error (degrees)
- Drift rate (meters per minute)

**Estimated Time**: 120 minutes

### Exercise 3: IMU Integration (Challenge)

**Objective**: Integrate IMU with VSLAM for improved accuracy.

**Tasks**:
1. Set up IMU sensor (physical or simulated)
2. Configure VSLAM to use IMU
3. Compare accuracy with/without IMU
4. Measure drift reduction
5. Document improvements

**Requirements**:
- IMU publishing `/imu/data` topic
- VSLAM configured with `enable_imu: True`
- Accuracy comparison report

**Estimated Time**: 180 minutes

---

## Additional Resources

- [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) - GitHub repository
- [VSLAM Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/) - Official docs
- [SLAM Tutorial](https://www.youtube.com/watch?v=U6vr3iNrwRA) - Visual SLAM overview
- [Stereo Vision](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html) - Stereo depth estimation

---

**Next**: [Chapter 3.4: Nav2 Navigation for Bipedal Humanoids →](chapter-3 to 4.md)
