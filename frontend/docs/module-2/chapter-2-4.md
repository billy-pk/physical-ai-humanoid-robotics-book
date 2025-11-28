---
sidebar_position: 5
title: 2.4 Visual SLAM Basics
---

# Chapter 2.4: Visual SLAM Basics

How does a robot know where it is? How does it build a map of an unknown environment? **SLAM (Simultaneous Localization and Mapping)** solves both problems at once using cameras.

## Learning Outcomes

- **Understand** the SLAM problem and why it's challenging
- **Explain** visual odometry and visual SLAM concepts
- **Apply** ORB-SLAM for camera-based localization
- **Recognize** when to use visual vs. LIDAR SLAM

## The SLAM Problem

**SLAM**: Simultaneously estimate:
1. **Robot's pose** (position + orientation) over time
2. **Map** of the environment

**The chicken-and-egg problem**:
- To localize, you need a map
- To build a map, you need to know your position

SLAM solves both together!

---

## Visual Odometry (VO)

**Visual Odometry** = estimating robot motion from camera images.

### How It Works

1. **Extract features** from consecutive frames (ORB, SIFT)
2. **Match features** between frames
3. **Estimate camera motion** from feature correspondences
4. **Integrate motion** to get trajectory

### Example with OpenCV

```python
import cv2
import numpy as np

# Feature detector
orb = cv2.ORB_create(nfeatures=2000)

# Feature matcher
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Initialize
cap = cv2.VideoCapture('robot_trajectory.mp4')
ret, prev_frame = cap.read()
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# Detect keypoints in first frame
prev_kp, prev_des = orb.detectAndCompute(prev_gray, None)

# Camera intrinsics (example values - get from calibration)
K = np.array([[500, 0, 320],
              [0, 500, 240],
              [0, 0, 1]], dtype=np.float32)

# Initialize pose
pose = np.eye(4)  # 4x4 transformation matrix

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect features in current frame
    curr_kp, curr_des = orb.detectAndCompute(gray, None)

    # Match features
    matches = bf.match(prev_des, curr_des)
    matches = sorted(matches, key=lambda x: x.distance)

    # Get matched keypoint coordinates
    pts1 = np.float32([prev_kp[m.queryIdx].pt for m in matches[:100]])
    pts2 = np.float32([curr_kp[m.trainIdx].pt for m in matches[:100]])

    # Estimate essential matrix
    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC,
                                   prob=0.999, threshold=1.0)

    # Recover pose (R, t)
    _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K)

    # Update pose
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.squeeze()
    pose = pose @ T

    # Print current position
    position = pose[:3, 3]
    print(f"Position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

    # Update for next iteration
    prev_kp, prev_des = curr_kp, curr_des

cap.release()
```

**Limitations of pure VO**:
- ❌ Drift accumulates over time (no loop closure)
- ❌ No map built
- ❌ Can't re-localize if lost

This is where full SLAM comes in!

---

## Visual SLAM

Visual SLAM extends VO by:
- **Building a map** of 3D landmarks
- **Loop closure detection** (recognize revisited places)
- **Global optimization** (correct accumulated drift)

### Popular Visual SLAM Systems

| System | Type | Strengths | Use Case |
|--------|------|-----------|----------|
| **ORB-SLAM3** | Feature-based | Fast, accurate, robust | General robotics |
| **LSD-SLAM** | Direct (no features) | Dense maps | Dense reconstruction |
| **RTAB-Map** | RGB-D | Works with depth cameras | Indoor robots |
| **SVO** | Semi-direct | Very fast | Drones, micro-robots |

---

## ORB-SLAM3: The Gold Standard

**ORB-SLAM3** is the most widely used visual SLAM system.

### Key Features

- ✅ Works with **monocular**, **stereo**, or **RGB-D** cameras
- ✅ Real-time performance (30+ FPS)
- ✅ Loop closure and global optimization
- ✅ Re-localization (recover from tracking loss)
- ✅ Multi-map support

### Installation

```bash
# Dependencies
sudo apt install libopencv-dev libeigen3-dev libpangolin-dev

# Clone and build
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

### Running ORB-SLAM3

```bash
# Monocular example (TUM dataset)
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/TUM1.yaml \
    /path/to/dataset
```

**Outputs**:
- Real-time 3D map visualization
- Trajectory (camera poses over time)
- Keyframe database

### Using with ROS

```bash
# Build ROS wrapper
cd Examples/ROS/ORB_SLAM3
mkdir build && cd build
cmake .. -DROS_BUILD_TYPE=Release
make

# Run
rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml
```

Subscribe to camera topic:
```bash
rostopic echo /camera/image_raw
```

---

## Understanding SLAM Output

### 1. Trajectory

List of camera poses over time:
```
timestamp, x, y, z, qx, qy, qz, qw
1678901234.5, 0.0, 0.0, 0.0, 0, 0, 0, 1
1678901234.6, 0.05, 0.01, 0.02, 0.01, 0, 0, 0.999
...
```

### 2. Map Points

3D landmarks (features) in the environment:
```
x, y, z
1.23, 0.45, 2.10
0.98, -0.12, 1.87
...
```

### 3. Keyframes

Subset of frames used for mapping (not every frame is kept).

---

## Visual-Inertial SLAM

Combine camera with IMU for better performance.

**Why add IMU?**
- ✅ Provides scale (monocular SLAM has scale ambiguity)
- ✅ Better performance in fast motion
- ✅ Robust to visual degradation (blur, low texture)

**Popular VI-SLAM systems**:
- ORB-SLAM3 (supports IMU)
- VINS-Mono
- Kimera

### Example: VINS-Mono with ROS

```bash
# Clone and build
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd VINS-Mono
catkin_make

# Run
roslaunch vins_estimator euroc.launch
roslaunch vins_estimator vins_rviz.launch

# Play dataset
rosbag play MH_01_easy.bag
```

**Visualization in RViz**:
- Green line: Estimated trajectory
- Red points: Map landmarks
- Blue frustums: Keyframe poses

---

## When to Use Visual SLAM vs. LIDAR SLAM

| Factor | Visual SLAM | LIDAR SLAM |
|--------|-------------|------------|
| **Cost** | Low (camera ~$50) | High (LIDAR $500-$5000) |
| **Indoor** | Good (if textured) | Excellent |
| **Outdoor** | Excellent | Good |
| **Lighting** | Sensitive | Unaffected |
| **Speed** | Medium (30-60 Hz) | Fast (10-20 Hz) |
| **Map density** | Sparse features | Dense 3D points |
| **Best for** | Drones, wheeled robots | Autonomous vehicles |

**Humanoid robots often use both**:
- Visual SLAM for rich environment understanding
- LIDAR for reliable obstacle avoidance

---

## Practical Tips

### 1. Camera Selection

**For visual SLAM**:
- **Global shutter** (not rolling shutter) — prevents motion distortion
- **Wide FOV** (>90°) — tracks more features
- **High frame rate** (60+ FPS) — better for fast motion

### 2. Calibration is Critical

Poor calibration → poor SLAM performance.

Calibrate:
- Intrinsic parameters (focal length, distortion)
- Stereo baseline (for stereo cameras)
- IMU-camera extrinsics (for VI-SLAM)

Tools:
- [Kalibr](https://github.com/ethz-asl/kalibr) (ROS-based)
- OpenCV calibration

### 3. Environment Matters

**SLAM works best with**:
- ✅ Rich visual features (posters, furniture, trees)
- ✅ Good lighting
- ✅ Moderate motion (not too fast)

**SLAM struggles with**:
- ❌ Blank walls
- ❌ Repetitive patterns (tiles, bricks)
- ❌ Motion blur
- ❌ Reflective/transparent surfaces

---

## Exercises

### 1. Visual Odometry Experiment
Download the [TUM RGB-D dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset). Implement simple visual odometry using ORB features. Plot the estimated trajectory vs. ground truth. What's the drift after 1 minute?

### 2. ORB-SLAM3 Installation
Install ORB-SLAM3 and run it on a sample dataset. Observe:
- How many map points are created?
- How many keyframes?
- Does loop closure occur?
- What's the average tracking FPS?

### 3. Visual vs. LIDAR Research
Find a recent paper comparing visual SLAM and LIDAR SLAM. Summarize:
- Which performed better indoors?
- Which performed better outdoors?
- What were the failure cases for each?

### 4. DIY Camera Trajectory
Record a video walking around a room with your phone (keep it steady!). Process it with ORB-SLAM3 monocular mode. Can it reconstruct the room? Where does it fail?

---

## Key Takeaways

✅ **SLAM** solves localization and mapping simultaneously
✅ **Visual odometry** estimates motion from camera but drifts over time
✅ **Visual SLAM** builds maps, detects loop closures, and corrects drift
✅ **ORB-SLAM3** is the state-of-the-art for most robotics applications
✅ **Visual-Inertial SLAM** (camera + IMU) is more robust
✅ SLAM requires **calibrated cameras** and **feature-rich environments**

---

## Further Reading

- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898) (Campos et al., 2021)
- [Visual SLAM Tutorial](https://www.youtube.com/watch?v=U6vr3iNrwRA) (Cyrill Stachniss lectures)
- *Introduction to Visual SLAM* by Xiang Gao (free book)
- [OpenVSLAM](https://github.com/xdspacelab/openvslam) (alternative to ORB-SLAM)

---

**Previous**: [← Chapter 2.3: 3D Vision and Depth](chapter-2-3.md)

---

🎉 **Module 2 Complete!** You now understand computer vision from classical techniques to deep learning, 3D perception, and visual SLAM. Ready to make robots move intelligently in Module 3!
