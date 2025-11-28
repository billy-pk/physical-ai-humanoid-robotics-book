---
sidebar_position: 4
title: 2.3 3D Vision and Depth Perception
---

# Chapter 2.3: 3D Vision and Depth Perception

Robots need to understand 3D space to navigate, grasp objects, and avoid obstacles. This chapter covers depth sensing technologies and 3D data processing for robotics.

## Learning Outcomes

- **Understand** depth sensing technologies (stereo, ToF, structured light)
- **Process** depth images and point clouds
- **Apply** 3D vision for navigation and manipulation
- **Use** libraries (OpenCV, Open3D) for 3D data

## Why 3D Vision?

2D images lack depth—you can't tell if an object is far away or just small.

**3D vision enables**:
- **Navigation**: Detect obstacles, measure distances
- **Grasping**: Know exact 3D position/orientation of objects
- **Mapping**: Build 3D maps of environments
- **Safety**: Detect humans and collision risks

---

## Depth Sensing Technologies

### 1. Stereo Vision

Uses two cameras (like human eyes) to compute depth via triangulation.

**How it works**:
1. Capture images from left and right cameras
2. Find corresponding points in both images (matching)
3. Compute **disparity** (pixel difference)
4. Convert disparity to depth: `depth = (focal_length × baseline) / disparity`

**Pros**:
- ✅ Passive (no active illumination)
- ✅ Works outdoors in sunlight
- ✅ Inexpensive (just two cameras)

**Cons**:
- ❌ Computationally expensive (matching)
- ❌ Fails on textureless surfaces
- ❌ Requires precise calibration

### 2. Time-of-Flight (ToF)

Measures time for light pulse to return.

**Examples**: Microsoft Kinect Azure, PMD sensors

**Pros**:
- ✅ Fast, dense depth maps
- ✅ Works on textureless surfaces

**Cons**:
- ❌ Limited range (~5m indoors)
- ❌ Fails in bright sunlight
- ❌ More expensive

### 3. Structured Light

Projects known pattern, measures deformation.

**Examples**: Intel RealSense D435, Kinect v1

**Pros**:
- ✅ High accuracy at close range
- ✅ Dense depth maps

**Cons**:
- ❌ Interference from multiple devices
- ❌ Doesn't work outdoors (sunlight)
- ❌ Limited range (~3-5m)

---

## Using Depth Cameras: Intel RealSense

### Installation

```bash
pip install pyrealsense2 opencv-python numpy
```

### Basic Depth Capture

```python
import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap to depth (for visualization)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )

        # Stack images horizontally
        images = np.hstack((color_image, depth_colormap))

        cv2.imshow('RealSense', images)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
```

### Get 3D Coordinates from Pixel

```python
# Get intrinsic parameters
depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

# Pixel coordinates (example: center of image)
pixel_x, pixel_y = 320, 240

# Get depth at this pixel (in millimeters)
depth_value = depth_frame.get_distance(pixel_x, pixel_y)  # meters

# Convert to 3D point
point_3d = rs.rs2_deproject_pixel_to_point(
    depth_intrin,
    [pixel_x, pixel_y],
    depth_value
)

print(f"3D point: x={point_3d[0]:.3f}m, y={point_3d[1]:.3f}m, z={point_3d[2]:.3f}m")
```

---

## Point Clouds

**Point cloud** = collection of 3D points representing a scene.

### Creating Point Clouds from Depth

```python
import open3d as o3d

def depth_to_pointcloud(depth_image, color_image, intrinsics):
    """
    Convert depth + color images to point cloud.

    Args:
        depth_image: HxW depth map (meters)
        color_image: HxWx3 RGB image
        intrinsics: Camera intrinsic parameters

    Returns:
        Open3D point cloud
    """
    h, w = depth_image.shape
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy

    # Generate pixel coordinates
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # Compute 3D points
    z = depth_image
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # Stack into Nx3 array
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)

    # Get colors
    colors = color_image.reshape(-1, 3) / 255.0

    # Filter invalid points (z == 0)
    valid = points[:, 2] > 0
    points = points[valid]
    colors = colors[valid]

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

# Visualize
o3d.visualization.draw_geometries([pcd])
```

### Point Cloud Processing

```python
import open3d as o3d

# Load point cloud (example: from PLY file)
pcd = o3d.io.read_point_cloud("scene.ply")

# Downsample (reduce points for speed)
pcd_down = pcd.voxel_down_sample(voxel_size=0.02)  # 2cm voxels

# Remove outliers
pcd_clean, ind = pcd_down.remove_statistical_outlier(
    nb_neighbors=20,
    std_ratio=2.0
)

# Estimate normals (useful for surface reconstruction)
pcd_clean.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)

# Segment plane (e.g., find ground/table)
plane_model, inliers = pcd_clean.segment_plane(
    distance_threshold=0.01,
    ransac_n=3,
    num_iterations=1000
)

# Separate plane and objects
plane_cloud = pcd_clean.select_by_index(inliers)
object_cloud = pcd_clean.select_by_index(inliers, invert=True)

# Color code (plane = red, objects = original colors)
plane_cloud.paint_uniform_color([1, 0, 0])

o3d.visualization.draw_geometries([plane_cloud, object_cloud])
```

---

## 3D Object Detection

Combine YOLO (2D) with depth to get 3D bounding boxes.

```python
from ultralytics import YOLO
import pyrealsense2 as rs

model = YOLO('yolov8n.pt')

# ... (RealSense initialization code)

while True:
    # Get frames
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())

    # Run YOLO detection
    results = model(color_image, conf=0.5)

    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            label = model.names[cls]

            # Get depth at bounding box center
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Get 3D position
            depth = depth_frame.get_distance(center_x, center_y)

            if depth > 0:
                # Deproject to 3D
                intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                point_3d = rs.rs2_deproject_pixel_to_point(
                    intrin, [center_x, center_y], depth
                )

                # Draw on image
                cv2.rectangle(color_image,
                             (int(x1), int(y1)), (int(x2), int(y2)),
                             (0, 255, 0), 2)
                text = f"{label} ({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})m"
                cv2.putText(color_image, text, (int(x1), int(y1)-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('3D Detection', color_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

**Output**: Object label + 3D position in robot's coordinate frame.

---

## Stereo Vision with OpenCV

If you have two calibrated cameras but no depth camera:

```python
import cv2
import numpy as np

# Load stereo calibration parameters (from calibration process)
stereo_map_left_x = np.load('stereo_map_left_x.npy')
stereo_map_left_y = np.load('stereo_map_left_y.npy')
stereo_map_right_x = np.load('stereo_map_right_x.npy')
stereo_map_right_y = np.load('stereo_map_right_y.npy')

# Stereo matcher
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16*6,  # Must be divisible by 16
    blockSize=11,
    P1=8 * 3 * 11**2,
    P2=32 * 3 * 11**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

# Capture from left and right cameras
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(1)

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not ret_left or not ret_right:
        break

    # Rectify images
    frame_left_rect = cv2.remap(frame_left, stereo_map_left_x,
                                stereo_map_left_y, cv2.INTER_LINEAR)
    frame_right_rect = cv2.remap(frame_right, stereo_map_right_x,
                                 stereo_map_right_y, cv2.INTER_LINEAR)

    # Convert to grayscale
    gray_left = cv2.cvtColor(frame_left_rect, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right_rect, cv2.COLOR_BGR2GRAY)

    # Compute disparity
    disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    # Visualize
    disparity_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    disparity_colormap = cv2.applyColorMap(disparity_vis, cv2.COLORMAP_JET)

    cv2.imshow('Disparity', disparity_colormap)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## Exercises

### 1. Depth Camera Exploration
If you have access to a depth camera (RealSense, Kinect, or smartphone with LiDAR), capture depth images of different surfaces:
- Smooth wall
- Textured carpet
- Reflective surface (mirror, metal)

Which surfaces produce clean depth maps? Which fail?

### 2. Point Cloud Manipulation
Load a point cloud (use Open3D sample data or capture your own). Apply:
- Downsampling (different voxel sizes)
- Outlier removal
- Plane segmentation

Visualize results at each step. How does voxel size affect detail vs. speed?

### 3. 3D Object Localization
Combine YOLO with a depth camera to detect and localize objects in 3D. Create a simple application that:
1. Detects a specific object class (e.g., "cup")
2. Prints its 3D coordinates
3. Draws an arrow pointing to it in the visualization

### 4. Stereo Calibration Research
Research stereo camera calibration. What parameters need to be determined? Why is calibration critical for accurate depth?

---

## Key Takeaways

✅ **3D vision** enables robots to navigate and manipulate in real environments
✅ **Depth cameras** (ToF, structured light) are easiest for robotics
✅ **Stereo vision** is passive and works outdoors, but computationally heavy
✅ **Point clouds** represent 3D scenes for mapping and object detection
✅ **Combining 2D detection + depth** = full 3D object localization
✅ Use **Open3D** for point cloud processing, **RealSense SDK** for depth cameras

---

## Further Reading

- [Intel RealSense Documentation](https://www.intelrealsense.com/sdk-2/)
- [Open3D Tutorials](http://www.open3d.org/docs/release/tutorial/index.html)
- *Multiple View Geometry in Computer Vision* by Hartley & Zisserman (stereo vision theory)

---

**Previous**: [← Chapter 2.2: Deep Learning for Vision](chapter-2-2.md) | **Next**: [Chapter 2.4: Visual SLAM Basics →](chapter-2-4.md)

3D perception unlocked! Next, we'll see how robots use vision to localize themselves and build maps simultaneously.
