---
sidebar_position: 4
title: 0.3 Sensor Systems for Humanoid Robots
---

# Chapter 0.3: Sensor Systems for Humanoid Robots

Humanoid robots need "senses" to perceive the world—just like humans have eyes, ears, and balance systems. This chapter covers the sensor systems that enable humanoid robots to see, hear, feel, and maintain balance. You'll learn about cameras, LiDAR, IMUs, force/torque sensors, and how to simulate them in PyBullet.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Understand** different sensor types used in humanoid robots
- **Explain** how vision sensors (cameras, depth cameras) work
- **Describe** LiDAR for 3D mapping and obstacle detection
- **Understand** IMUs for balance and orientation tracking
- **Implement** basic sensor simulation in PyBullet
- **Appreciate** sensor fusion for robust perception

## Prerequisites

- **Chapters 0.1 to 0.2** completed (Physical AI concepts, humanoid landscape)
- **Basic Python** knowledge
- **PyBullet** installed (from Chapter 0.1)

## Part 1: Sensor Overview

### Why Sensors Matter

**Without sensors**, a robot is **blind and deaf**:
- Can't see obstacles → will collide
- Can't hear commands → can't respond
- Can't feel objects → can't grasp properly
- Can't sense balance → will fall over

**With sensors**, a robot can:
- **Perceive** the environment
- **Navigate** safely
- **Manipulate** objects
- **Interact** with humans
- **Maintain** balance

### Sensor Categories

| Category | Purpose | Examples |
|----------|---------|----------|
| **Vision** | See the world | RGB cameras, depth cameras, stereo vision |
| **Range** | Measure distances | LiDAR, ultrasonic sensors, depth cameras |
| **Motion** | Track movement | IMU, encoders, gyroscopes |
| **Force/Torque** | Feel forces | Force sensors, tactile sensors |
| **Audio** | Hear sounds | Microphones, microphone arrays |

**Humanoid robots** typically use **all of these** for robust perception.

## Part 2: Vision Sensors

### RGB Cameras

**What they do**: Capture color images (like human eyes)

**How they work**:
- **Lens**: Focuses light onto sensor
- **Sensor**: Converts light to electrical signals
- **Processor**: Converts signals to digital image (RGB pixels)

**Specifications**:
- **Resolution**: 640x480 to 4K+ (more pixels = more detail)
- **Frame rate**: 30 to 60 FPS (frames per second)
- **Field of view**: 60 to 120° (how wide the view)

**Use cases**:
- Object detection and recognition
- Human detection and tracking
- Visual navigation (following paths, reading signs)
- Gesture recognition

**Limitations**:
- **Lighting**: Poor performance in dark or bright conditions
- **Occlusion**: Can't see behind objects
- **2D only**: No depth information (unless stereo)

### Depth Cameras

**What they do**: Measure distance to objects (like human depth perception)

**How they work**:
- **Structured light**: Projects pattern, measures distortion (Intel RealSense)
- **Time-of-flight**: Measures time for light to bounce back
- **Stereo vision**: Two cameras, triangulate distance

**Output**: Depth map (each pixel = distance in meters)

**Use cases**:
- 3D mapping and SLAM
- Obstacle avoidance
- Object manipulation (knowing how far to reach)
- Human pose estimation

**Specifications**:
- **Range**: 0.5m to 10m+ (depending on technology)
- **Accuracy**: ±1 to 5cm (varies with distance)
- **Resolution**: 640x480 to 1080p

**Limitations**:
- **Reflective surfaces**: Poor performance (glass, mirrors)
- **Outdoor lighting**: Struggles in bright sunlight
- **Range limits**: Can't see beyond maximum range

### Stereo Vision

**What they do**: Use two cameras to compute depth (like human eyes)

**How it works**:
1. Two cameras capture images from slightly different positions
2. Find matching points in both images
3. Compute depth using triangulation

**Advantages**:
- **Passive**: No special lighting needed
- **Works outdoors**: Better than structured light in sunlight
- **Color + depth**: RGB images with depth

**Disadvantages**:
- **Texture required**: Needs features to match (fails on blank walls)
- **Computational**: More processing than single camera
- **Calibration**: Cameras must be precisely aligned

## Part 3: LiDAR (Light Detection and Ranging)

### What is LiDAR?

**LiDAR** uses lasers to measure distances in 360° around the robot.

**How it works**:
1. **Laser emitter**: Sends out laser pulses
2. **Reflection**: Light bounces off objects
3. **Receiver**: Detects reflected light
4. **Time measurement**: Calculate distance from time-of-flight
5. **Rotation**: Spins to scan 360°

**Output**: Point cloud (3D points representing surfaces)

### LiDAR Specifications

**Range**: 10m to 200m+ (depending on model)
**Angular resolution**: 0.1° to 1° (how fine the scan)
**Scan rate**: 10 to 40 Hz (scans per second)
**Points per scan**: 1,000 to 1,000,000+ points

### Use Cases

**Mapping**:
- Create 3D maps of environments
- SLAM (Simultaneous Localization and Mapping)

**Navigation**:
- Obstacle detection and avoidance
- Path planning in unknown environments

**Object Detection**:
- Identify objects by shape
- Track moving objects

### Limitations

**Cost**: Expensive ($1,000-$100,000+)
**Size**: Large and heavy (challenging for humanoids)
**Power**: High power consumption
**Outdoors**: Can struggle in rain, fog, bright sunlight

**Note**: Some humanoids use **solid-state LiDAR** (smaller, cheaper, but limited field of view).

## Part 4: IMU (Inertial Measurement Unit)

### What is an IMU?

**IMU** measures motion and orientation (like human inner ear/balance system).

**Components**:
- **Accelerometer**: Measures linear acceleration (gravity + motion)
- **Gyroscope**: Measures angular velocity (rotation)
- **Magnetometer**: Measures magnetic field (compass, optional)

### How IMUs Work

**Accelerometer**:
- Uses tiny masses that move when accelerated
- Measures force on masses → acceleration
- **Gravity**: Always measures 9.81 m/s² downward (when stationary)

**Gyroscope**:
- Uses spinning masses or MEMS (Micro-Electro-Mechanical Systems)
- Measures rotation rate (degrees/second)
- **Integration**: Can compute orientation over time

**Magnetometer**:
- Measures Earth's magnetic field
- Provides absolute heading (north direction)
- **Calibration**: Needs calibration for accuracy

### Use Cases

**Balance Control**:
- Detect tilt and rotation
- Maintain upright posture
- Prevent falls

**Navigation**:
- Dead reckoning (estimate position from motion)
- Orientation tracking
- Step counting (for walking robots)

**Motion Tracking**:
- Measure movement during manipulation
- Detect impacts or collisions
- Monitor robot health

### Limitations

**Drift**: Orientation estimate drifts over time (gyroscope integration error)
**Noise**: Sensor measurements are noisy
**Calibration**: Requires calibration for accuracy
**Magnetic interference**: Magnetometer affected by metal objects

**Solution**: **Sensor fusion** (combine IMU with other sensors like cameras).

## Part 5: Force/Torque Sensors

### What are Force/Torque Sensors?

**Force sensors** measure forces applied to the robot (like human touch).

**Types**:
- **Force sensors**: Measure linear forces (push/pull)
- **Torque sensors**: Measure rotational forces (twist)
- **Tactile sensors**: Measure contact pressure distribution

### Use Cases

**Manipulation**:
- **Grasping**: Know how hard to grip (don't crush objects)
- **Contact detection**: Know when touching objects
- **Force control**: Apply specific forces

**Balance**:
- **Foot contact**: Detect when foot touches ground
- **Weight distribution**: Measure forces on each foot
- **Stability**: Detect if robot is tipping

**Safety**:
- **Collision detection**: Detect unexpected contact
- **Force limits**: Stop if force exceeds threshold
- **Human interaction**: Gentle touch detection

### Specifications

**Range**: 0.1N to 10,000N+ (depending on application)
**Accuracy**: ±1 to 5% of full scale
**Update rate**: 1 to 10 kHz (very fast)

**Placement**:
- **Wrists**: Measure manipulation forces
- **Ankles**: Measure walking forces
- **Fingers**: Measure grasping forces

## Part 6: Sensor Fusion

### Why Combine Sensors?

**Single sensor limitations**:
- **Camera**: No depth (unless stereo/depth camera)
- **LiDAR**: No color information
- **IMU**: Drifts over time
- **Force sensor**: Only measures contact

**Sensor fusion** combines multiple sensors for:
- **Robustness**: If one sensor fails, others compensate
- **Accuracy**: Multiple measurements improve accuracy
- **Completeness**: Get both color and depth, position and orientation

### Common Fusion Strategies

**Vision + LiDAR**:
- **Vision**: Color, texture, object recognition
- **LiDAR**: Accurate 3D structure
- **Combined**: Colored 3D point cloud

**IMU + Vision**:
- **IMU**: Fast motion tracking (high frequency)
- **Vision**: Accurate position (low frequency, no drift)
- **Combined**: Smooth, accurate motion estimate

**Force + Vision**:
- **Vision**: See object before contact
- **Force**: Feel object during contact
- **Combined**: Robust manipulation

## Part 7: Hands-On Tutorial

### Project: Simulate Sensors in PyBullet

**Goal**: Add sensors to a robot simulation and visualize sensor data.

**Tools**: PyBullet, Python 3.10+, NumPy, OpenCV (optional)

### Step 1: Camera Simulation

**Create file**: `camera_sensor.py`

```python
#!/usr/bin/env python3
"""
Camera sensor simulation in PyBullet
Demonstrates RGB and depth camera rendering
"""
import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time

# Connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load ground
planeId = p.loadURDF("plane.urdf")

# Create a simple scene with objects
boxId = p.loadURDF("cube.urdf", [2, 0, 0.5], [0, 0, 0, 1])
sphereId = p.loadURDF("sphere.urdf", [-2, 0, 0.5], [0, 0, 0, 1])

# Camera parameters
camera_pos = [0, 0, 2]  # Camera position
camera_target = [0, 0, 0]  # Where camera looks
camera_up = [0, 1, 0]  # Up direction

# Camera view matrix
view_matrix = p.computeViewMatrix(
    cameraEyePosition=camera_pos,
    cameraTargetPosition=camera_target,
    cameraUpVector=camera_up
)

# Camera projection matrix
aspect = 640.0 / 480.0  # Width/height
fov = 60  # Field of view (degrees)
near = 0.01  # Near plane
far = 10  # Far plane

projection_matrix = p.computeProjectionMatrixFOV(
    fov, aspect, near, far
)

print("Camera sensor simulation started!")
print("Capturing RGB and depth images...")

# Capture images
for i in range(100):
    p.stepSimulation()
    
    # Render camera view
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=640,
        height=480,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix
    )
    
    # Convert RGB image (4 channels: RGBA)
    rgb_array = np.array(rgbImg)
    rgb_array = rgb_array[:, :, :3]  # Remove alpha channel
    rgb_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)  # OpenCV uses BGR
    
    # Convert depth image
    depth_array = np.array(depthImg)
    
    # Normalize depth for visualization (0 to 255)
    depth_normalized = (depth_array * 255 / far).astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLOR_MAP_INFERNO)
    
    # Display images (every 10 steps)
    if i % 10 == 0:
        cv2.imshow("RGB Camera", rgb_array)
        cv2.imshow("Depth Camera", depth_colormap)
        cv2.waitKey(1)
        
        print(f"Step {i}: Captured RGB ({rgb_array.shape}) and depth ({depth_array.shape})")
    
    time.sleep(1./240.)

cv2.destroyAllWindows()
p.disconnect()
print("Camera simulation complete!")
```

**Run simulation**:
```bash
python3 camera_sensor.py
```

**Expected Output**:
- Two windows showing RGB and depth camera views
- RGB shows colored objects
- Depth shows distance (closer = brighter)

### Step 2: IMU Simulation

**Create file**: `imu_sensor.py`

```python
#!/usr/bin/env python3
"""
IMU sensor simulation in PyBullet
Demonstrates accelerometer and gyroscope measurements
"""
import pybullet as p
import pybullet_data
import numpy as np
import time

# Connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load ground
planeId = p.loadURDF("plane.urdf")

# Create a box that will tilt
boxId = p.loadURDF("cube.urdf", [0, 0, 1], [0, 0, 0, 1])

# Simulate IMU on the box
print("IMU sensor simulation started!")
print("Measuring acceleration and angular velocity...")

previous_linear_velocity = np.array([0, 0, 0])
previous_angular_velocity = np.array([0, 0, 0])

for i in range(1000):
    p.stepSimulation()
    
    # Get box state
    pos, orient = p.getBasePositionAndOrientation(boxId)
    linear_velocity, angular_velocity = p.getBaseVelocity(boxId)
    
    # Convert to numpy arrays
    linear_velocity = np.array(linear_velocity)
    angular_velocity = np.array(angular_velocity)
    
    # Simulate accelerometer (measures linear acceleration + gravity)
    # In real IMU: acceleration = linear_acceleration + gravity (in sensor frame)
    # For simplicity, we'll compute linear acceleration from velocity change
    dt = 1./240.  # Simulation timestep
    linear_acceleration = (linear_velocity - previous_linear_velocity) / dt
    
    # Add gravity (in world frame, pointing down in Z)
    gravity_world = np.array([0, 0, -9.81])
    
    # Transform gravity to box frame (simplified - would use rotation matrix)
    # For now, assume box is upright (gravity = [0, 0, -9.81] in box frame)
    # In reality, need to rotate gravity vector by box orientation
    accelerometer_reading = linear_acceleration + gravity_world
    
    # Gyroscope reading (angular velocity in rad/s)
    gyroscope_reading = angular_velocity
    
    # Print readings every 50 steps
    if i % 50 == 0:
        print(f"\nStep {i}:")
        print(f"  Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        print(f"  Accelerometer: ({accelerometer_reading[0]:.2f}, {accelerometer_reading[1]:.2f}, {accelerometer_reading[2]:.2f}) m/s²")
        print(f"  Gyroscope: ({gyroscope_reading[0]:.2f}, {gyroscope_reading[1]:.2f}, {gyroscope_reading[2]:.2f}) rad/s")
    
    previous_linear_velocity = linear_velocity
    previous_angular_velocity = angular_velocity
    
    time.sleep(1./240.)

p.disconnect()
print("IMU simulation complete!")
```

**Run simulation**:
```bash
python3 imu_sensor.py
```

**Expected Output**:
- Box falls and tilts
- Accelerometer readings show gravity + motion
- Gyroscope readings show rotation

### Step 3: LiDAR Simulation

**Create file**: `lidar_sensor.py`

```python
#!/usr/bin/env python3
"""
LiDAR sensor simulation in PyBullet
Demonstrates ray casting for distance measurement
"""
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time

# Connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load ground
planeId = p.loadURDF("plane.urdf")

# Create objects to detect
boxId = p.loadURDF("cube.urdf", [2, 0, 0.5], [0, 0, 0, 1])
sphereId = p.loadURDF("sphere.urdf", [-2, 0, 0.5], [0, 0, 0, 1])

# LiDAR parameters
lidar_pos = [0, 0, 1]  # LiDAR position
num_rays = 360  # Number of rays (360° scan)
max_range = 10  # Maximum range (meters)
ray_length = max_range

print("LiDAR sensor simulation started!")
print(f"Scanning {num_rays} rays in 360°...")

# Perform LiDAR scan
ray_from = []
ray_to = []
ray_results = []

for i in range(num_rays):
    angle = 2 * np.pi * i / num_rays  # Angle in radians
    
    # Compute ray direction
    ray_dir = [np.cos(angle), np.sin(angle), 0]  # Horizontal scan
    ray_end = [
        lidar_pos[0] + ray_length * ray_dir[0],
        lidar_pos[1] + ray_length * ray_dir[1],
        lidar_pos[2] + ray_length * ray_dir[2]
    ]
    
    ray_from.append(lidar_pos)
    ray_to.append(ray_end)

# Cast rays
ray_results = p.rayTestBatch(ray_from, ray_to)

# Extract distances
distances = []
for result in ray_results:
    if result[0] != -1:  # Hit something
        hit_pos = result[3]  # Hit position
        distance = np.linalg.norm(np.array(hit_pos) - np.array(lidar_pos))
        distances.append(distance)
    else:  # No hit (max range)
        distances.append(max_range)

# Visualize LiDAR scan
angles = np.linspace(0, 2*np.pi, num_rays)
plt.figure(figsize=(10, 10))
plt.polar(angles, distances)
plt.title("LiDAR Scan (360°)")
plt.xlabel("Distance (m)")
plt.show()

print(f"LiDAR scan complete!")
print(f"Detected {sum(1 for d in distances if d < max_range)} hits")
print(f"Average distance: {np.mean([d for d in distances if d < max_range]):.2f} m")

p.disconnect()
print("LiDAR simulation complete!")
```

**Run simulation**:
```bash
python3 lidar_sensor.py
```

**Expected Output**:
- Polar plot showing 360° LiDAR scan
- Detects objects at different distances
- Visual representation of environment

### Step 4: Sensor Fusion Example

**Create file**: `sensor_fusion.py`

```python
#!/usr/bin/env python3
"""
Simple sensor fusion example
Combines camera and IMU for better state estimation
"""
import pybullet as p
import pybullet_data
import numpy as np
import time

# Connect to physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("cube.urdf", [0, 0, 1], [0, 0, 0, 1])

print("Sensor fusion simulation started!")
print("Combining camera and IMU measurements...")

# Camera (low frequency, accurate position)
camera_update_rate = 10  # Hz
camera_last_update = 0

# IMU (high frequency, drifts)
imu_update_rate = 240  # Hz

# State estimate (fused)
estimated_pos = np.array([0, 0, 1])
estimated_vel = np.array([0, 0, 0])

for i in range(1000):
    p.stepSimulation()
    t = i / 240.0  # Time in seconds
    
    # Get true state (for comparison)
    true_pos, _ = p.getBasePositionAndOrientation(boxId)
    true_vel, _ = p.getBaseVelocity(boxId)
    true_pos = np.array(true_pos)
    true_vel = np.array(true_vel)
    
    # IMU measurement (high frequency)
    if i % (240 // imu_update_rate) == 0:
        # Update velocity estimate from IMU
        # (In reality, would integrate accelerometer)
        estimated_vel = true_vel + np.random.normal(0, 0.01, 3)  # Add noise
    
    # Camera measurement (low frequency, accurate)
    if i % (240 // camera_update_rate) == 0:
        # Reset position estimate from camera
        estimated_pos = true_pos + np.random.normal(0, 0.001, 3)  # Very accurate
        camera_last_update = t
    
    # Fuse estimates (simple: use camera when available, IMU otherwise)
    # Update position from velocity
    dt = 1./240.
    if t - camera_last_update < 1./camera_update_rate:
        # Recent camera update, trust it
        pass  # Keep camera position
    else:
        # No recent camera update, use IMU
        estimated_pos = estimated_pos + estimated_vel * dt
    
    # Print comparison every 100 steps
    if i % 100 == 0:
        error = np.linalg.norm(estimated_pos - true_pos)
        print(f"Step {i}: True pos = {true_pos}, Estimated pos = {estimated_pos}, Error = {error:.3f} m")
    
    time.sleep(1./240.)

p.disconnect()
print("Sensor fusion simulation complete!")
```

**What this demonstrates**:
- **Camera**: Accurate but slow (low frequency)
- **IMU**: Fast but drifts (high frequency)
- **Fusion**: Combines both for best estimate

## Part 8: Debugging Common Issues

### Issue 1: "Camera image is black"
**Symptoms**: Camera returns all zeros or black image

**Solutions**:
```python
# Check camera position and target
# Ensure objects are in view
# Verify camera is not inside an object
# Check lighting (some simulators need light sources)
```

### Issue 2: "IMU readings are noisy"
**Symptoms**: Accelerometer/gyroscope values jump around

**Solutions**:
```python
# Add filtering (moving average, Kalman filter)
# Reduce simulation timestep (more frequent updates)
# Add noise model (real sensors are noisy)
```

### Issue 3: "LiDAR doesn't detect objects"
**Symptoms**: All rays return max range

**Solutions**:
```python
# Check ray origin and direction
# Verify objects are within range
# Check collision detection is enabled
# Ensure objects have collision geometry
```

## Summary

You learned:
- ✅ **Vision sensors**: RGB cameras, depth cameras, stereo vision for seeing the world
- ✅ **LiDAR**: 3D mapping and obstacle detection using lasers
- ✅ **IMU**: Balance, orientation, and motion tracking
- ✅ **Force/torque sensors**: Tactile feedback for manipulation
- ✅ **Sensor fusion**: Combining multiple sensors for robust perception
- ✅ **Simulation**: How to simulate sensors in PyBullet

**Next steps**: Complete the Module 0 project—build a robot simulation with sensors!

---

## Exercises

### Exercise 1: Camera Calibration (Required)

**Objective**: Understand camera parameters and calibration.

**Tasks**:
1. Create simulation with camera
2. Experiment with different:
   - Field of view (FOV)
   - Resolution
   - Camera positions
3. Observe how parameters affect image
4. Document findings

**Acceptance Criteria**:
- [ ] Camera simulation working
- [ ] Different FOVs tested
- [ ] Different resolutions tested
- [ ] Observations documented

**Estimated Time**: 45 minutes

### Exercise 2: IMU Drift Analysis (Required)

**Objective**: Understand IMU drift and its impact.

**Tasks**:
1. Create simulation with IMU
2. Track orientation over time
3. Measure drift (error accumulation)
4. Compare with ground truth
5. Document drift rate

**Acceptance Criteria**:
- [ ] IMU simulation working
- [ ] Orientation tracked over time
- [ ] Drift measured and documented
- [ ] Comparison with ground truth

**Estimated Time**: 60 minutes

### Exercise 3: Sensor Fusion Implementation (Challenge)

**Objective**: Implement simple sensor fusion.

**Tasks**:
1. Create simulation with camera and IMU
2. Implement fusion algorithm (weighted average or Kalman filter)
3. Compare fused estimate to individual sensors
4. Measure improvement in accuracy
5. Document fusion approach

**Requirements**:
- Camera and IMU simulated
- Fusion algorithm implemented
- Accuracy improvement demonstrated
- Approach documented

**Estimated Time**: 120 minutes

---

## Additional Resources

- [PyBullet Camera API](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit) - Camera documentation
- [Intel RealSense](https://www.intelrealsense.com/) - Depth camera technology
- [Velodyne LiDAR](https://velodynelidar.com/) - LiDAR specifications
- [Sensor Fusion Tutorial](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) - Kalman filtering

---

**Next**: [Module 0 Project: Basic Robot Simulation →](intro.md#assessment)

**Return to**: [Module 0 Introduction →](intro.md)
