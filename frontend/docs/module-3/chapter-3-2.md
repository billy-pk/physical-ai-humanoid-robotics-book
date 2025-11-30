---
sidebar_position: 3
title: 3.2 Synthetic Data Generation for Training
---

# Chapter 3.2: Synthetic Data Generation for Training

Training vision-based AI models requires thousands of labeled images. Isaac Sim provides built-in tools for generating synthetic data with perfect annotations—enabling rapid dataset creation without manual labeling. This chapter covers data collection pipelines, domain randomization, and dataset export formats.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Set up** data collection pipelines in Isaac Sim for ML training
- **Generate** RGB, depth, and segmentation images with annotations
- **Implement** domain randomization for robust model training
- **Export** datasets in COCO, YOLO, and custom formats
- **Validate** synthetic data quality and diversity

## Prerequisites

- **Isaac Sim 2023.1.1** installed and configured (Chapter 3.1)
- **Python 3.10+** with numpy, opencv-python
- **Basic ML concepts**: datasets, annotations, training/validation splits
- **Understanding** of image formats (PNG, JPEG) and coordinate systems

## Part 1: Synthetic Data Fundamentals

### Why Synthetic Data?

**Challenges with real-world data**:
- **Labeling cost**: Manual annotation is expensive ($0.10-$1.00 per image)
- **Data scarcity**: Rare scenarios (accidents, edge cases) hard to capture
- **Privacy**: Real images may contain sensitive information
- **Variability**: Hard to control lighting, weather, object positions

**Advantages of synthetic data**:
- **Perfect labels**: Automatic, pixel-perfect annotations
- **Infinite variety**: Generate millions of images quickly
- **Controlled conditions**: Test specific scenarios (lighting, weather)
- **Cost-effective**: Generate 10,000 images in hours vs. weeks of manual labeling

### Data Types for Humanoid Robots

| Data Type | Use Case | Format |
|-----------|----------|--------|
| **RGB Images** | Object detection, classification | PNG/JPEG (H×W×3) |
| **Depth Maps** | 3D perception, manipulation | PNG 16-bit (H×W×1) |
| **Segmentation Masks** | Instance segmentation | PNG (H×W×1, class IDs) |
| **Bounding Boxes** | Object detection | JSON (COCO/YOLO format) |
| **Pose Annotations** | Keypoint detection | JSON (COCO format) |
| **Point Clouds** | 3D SLAM, manipulation | PLY/PCD (N×3) |

**This chapter focuses on**: RGB images, depth maps, segmentation masks, and bounding boxes.

### Domain Randomization

**Domain randomization** varies scene parameters to improve model robustness:

- **Lighting**: Intensity, color temperature, shadows
- **Textures**: Material properties, surface patterns
- **Object positions**: Random placement, orientations
- **Camera angles**: Varying viewpoints, distances
- **Backgrounds**: Different environments, clutter
- **Weather**: Fog, rain, snow (if outdoor)

**Goal**: Train models that generalize to real-world variations.

## Part 2: Hands-On Tutorial

### Project: Generate Synthetic Dataset for Object Detection

**Goal**: Create a data collection pipeline that generates 500+ labeled images of humanoid robot interacting with objects.

**Tools**: Isaac Sim 2023.1.1, Python 3.10+, OpenCV

### Step 1: Set Up Data Collection Script

**Create Python script**: `scripts/collect_synthetic_data.py`

```python
#!/usr/bin/env python3
"""
Synthetic data collection pipeline for Isaac Sim
Isaac Sim 2023.1.1 | Python 3.10+
"""
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim in headless mode (faster)
simulation_app = SimulationApp({
    "headless": False,  # Set True for batch processing
    "width": 1920,
    "height": 1080
})

import numpy as np
import cv2
import json
import os
from pathlib import Path
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import random

# Configuration
OUTPUT_DIR = Path("~/synthetic_data")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

RGB_DIR = OUTPUT_DIR / "rgb"
DEPTH_DIR = OUTPUT_DIR / "depth"
SEGMENTATION_DIR = OUTPUT_DIR / "segmentation"
ANNOTATIONS_DIR = OUTPUT_DIR / "annotations"

for dir_path in [RGB_DIR, DEPTH_DIR, SEGMENTATION_DIR, ANNOTATIONS_DIR]:
    dir_path.mkdir(exist_ok=True)

NUM_IMAGES = 500
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080

class DataCollector:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.frame_count = 0
        self.annotations = []
        
        # Setup scene
        self.setup_scene()
        
        # Setup camera
        self.setup_camera()
        
    def setup_scene(self):
        """Initialize scene with robot and objects"""
        # Add ground plane
        self.world.scene.add_default_ground_plane()
        
        # Add robot (from URDF)
        robot_urdf_path = "/path/to/humanoid.urdf"
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/Robot",
                name="humanoid",
                usd_path=None,  # Will load URDF
                position=np.array([0, 0, 1.0])
            )
        )
        
        # Add objects to detect (cubes, spheres, etc.)
        self.objects = []
        # ... (object creation code)
        
    def setup_camera(self):
        """Create camera sensor"""
        self.camera = self.world.scene.add(
            Camera(
                prim_path="/World/Camera",
                name="camera",
                position=np.array([0, -2, 1.6]),  # Eye height
                resolution=(IMAGE_WIDTH, IMAGE_HEIGHT),
                frequency=30
            )
        )
        
    def randomize_scene(self):
        """Apply domain randomization"""
        # Randomize lighting
        light_intensity = random.uniform(0.5, 3.0)
        # ... (light randomization)
        
        # Randomize object positions
        for obj in self.objects:
            obj.set_world_pose(
                position=np.array([
                    random.uniform(-2, 2),
                    random.uniform(-2, 2),
                    random.uniform(0.5, 1.5)
                ])
            )
        
        # Randomize robot pose
        # ... (joint angle randomization)
        
        # Randomize camera angle
        camera_angle = random.uniform(-30, 30)  # degrees
        # ... (camera pose update)
        
    def capture_frame(self):
        """Capture RGB, depth, and segmentation images"""
        # Step simulation
        self.world.step(render=True)
        
        # Get camera data
        rgb_data = self.camera.get_rgba()  # H×W×4 (RGBA)
        depth_data = self.camera.get_current_frame()["distance_to_image_plane"]  # H×W
        segmentation_data = self.camera.get_current_frame()["semantic_segmentation"]  # H×W
        
        # Convert to standard formats
        rgb_image = (rgb_data[:, :, :3] * 255).astype(np.uint8)  # Remove alpha, scale to 0 to 255
        depth_image = (depth_data * 1000).astype(np.uint16)  # Convert to mm, 16-bit
        seg_image = segmentation_data.astype(np.uint8)  # Class IDs
        
        # Save images
        frame_id = f"{self.frame_count:06d}"
        cv2.imwrite(str(RGB_DIR / f"{frame_id}.png"), cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(DEPTH_DIR / f"{frame_id}.png"), depth_image)
        cv2.imwrite(str(SEGMENTATION_DIR / f"{frame_id}.png"), seg_image)
        
        # Generate annotations (bounding boxes)
        annotations = self.generate_annotations(seg_image)
        
        return annotations
    
    def generate_annotations(self, seg_image):
        """Generate bounding box annotations from segmentation"""
        annotations = []
        
        # Find unique object IDs (excluding background=0)
        unique_ids = np.unique(seg_image)
        unique_ids = unique_ids[unique_ids > 0]
        
        for obj_id in unique_ids:
            # Find bounding box
            mask = (seg_image == obj_id)
            rows = np.any(mask, axis=1)
            cols = np.any(mask, axis=0)
            
            if np.any(rows) and np.any(cols):
                y_min, y_max = np.where(rows)[0][[0, -1]]
                x_min, x_max = np.where(cols)[0][[0, -1]]
                
                # COCO format: [x_min, y_min, width, height]
                bbox = [int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)]
                area = bbox[2] * bbox[3]
                
                annotations.append({
                    "id": len(annotations),
                    "image_id": self.frame_count,
                    "category_id": int(obj_id),
                    "bbox": bbox,
                    "area": area,
                    "iscrowd": 0
                })
        
        return annotations
    
    def collect_dataset(self):
        """Main data collection loop"""
        print(f"Starting data collection: {NUM_IMAGES} images")
        
        for i in range(NUM_IMAGES):
            # Randomize scene
            self.randomize_scene()
            
            # Capture frame
            annotations = self.capture_frame()
            self.annotations.extend(annotations)
            
            self.frame_count += 1
            
            if (i + 1) % 50 == 0:
                print(f"Collected {i + 1}/{NUM_IMAGES} images")
        
        # Save annotations (COCO format)
        self.save_coco_annotations()
        
        print(f"Data collection complete: {self.frame_count} images")
    
    def save_coco_annotations(self):
        """Save annotations in COCO format"""
        coco_data = {
            "images": [
                {
                    "id": i,
                    "width": IMAGE_WIDTH,
                    "height": IMAGE_HEIGHT,
                    "file_name": f"{i:06d}.png"
                }
                for i in range(self.frame_count)
            ],
            "annotations": self.annotations,
            "categories": [
                {"id": 1, "name": "robot"},
                {"id": 2, "name": "object"},
                # ... (add more categories)
            ]
        }
        
        with open(ANNOTATIONS_DIR / "annotations.json", "w") as f:
            json.dump(coco_data, f, indent=2)

if __name__ == "__main__":
    collector = DataCollector()
    collector.world.reset()
    collector.collect_dataset()
    simulation_app.close()
```

### Step 2: Configure Domain Randomization

**Enhanced randomization** (`scripts/domain_randomization.py`):

```python
class DomainRandomizer:
    def __init__(self, world):
        self.world = world
        
    def randomize_lighting(self):
        """Randomize scene lighting"""
        # Get dome light
        light_prim = self.world.stage.GetPrimAtPath("/World/DomeLight")
        
        # Random intensity
        intensity = random.uniform(0.5, 3.0)
        light_prim.GetAttribute("intensity").Set(intensity)
        
        # Random color temperature
        color_temp = random.uniform(3000, 7000)  # Kelvin
        # Convert to RGB (simplified)
        rgb = self.kelvin_to_rgb(color_temp)
        light_prim.GetAttribute("color").Set(rgb)
        
    def randomize_materials(self):
        """Randomize object materials"""
        for obj in self.objects:
            material = obj.get_applied_material()
            
            # Random base color
            base_color = [
                random.uniform(0, 1),
                random.uniform(0, 1),
                random.uniform(0, 1)
            ]
            material.set_attribute("base_color", base_color)
            
            # Random roughness
            roughness = random.uniform(0.1, 0.9)
            material.set_attribute("roughness", roughness)
    
    def randomize_camera(self):
        """Randomize camera pose"""
        # Random position (around robot)
        angle = random.uniform(0, 2 * np.pi)
        distance = random.uniform(1.5, 3.0)
        height = random.uniform(1.2, 2.0)
        
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)
        z = height
        
        # Random orientation (look at robot)
        # ... (camera pose calculation)
    
    def kelvin_to_rgb(self, kelvin):
        """Convert color temperature to RGB"""
        # Simplified conversion (use proper algorithm in production)
        if kelvin < 4000:
            # Warm (reddish)
            return [1.0, 0.7, 0.5]
        elif kelvin < 6000:
            # Neutral (white)
            return [1.0, 1.0, 1.0]
        else:
            # Cool (bluish)
            return [0.8, 0.9, 1.0]
```

### Step 3: Run Data Collection

**Execute script**:
```bash
cd ~/isaac_sim_ws
python3 scripts/collect_synthetic_data.py
```

**Expected Output**:
```
Starting data collection: 500 images
Collected 50/500 images
Collected 100/500 images
...
Collected 500/500 images
Data collection complete: 500 images
```

**Verify output**:
```bash
# Check directories
ls ~/synthetic_data/rgb/ | wc -l  # Should show 500
ls ~/synthetic_data/depth/ | wc -l
ls ~/synthetic_data/segmentation/ | wc -l

# View sample image
eog ~/synthetic_data/rgb/000000.png
```

### Step 4: Export to YOLO Format

**Convert COCO to YOLO** (`scripts/coco_to_yolo.py`):

```python
#!/usr/bin/env python3
"""
Convert COCO annotations to YOLO format
"""
import json
from pathlib import Path

def coco_to_yolo(coco_file, output_dir, image_width, image_height):
    """Convert COCO JSON to YOLO txt files"""
    with open(coco_file, 'r') as f:
        coco_data = json.load(f)
    
    # Create mapping: image_id -> annotations
    image_annotations = {}
    for ann in coco_data['annotations']:
        image_id = ann['image_id']
        if image_id not in image_annotations:
            image_annotations[image_id] = []
        image_annotations[image_id].append(ann)
    
    # Create category mapping
    category_map = {cat['id']: idx for idx, cat in enumerate(coco_data['categories'])}
    
    # Convert each image's annotations
    for image_info in coco_data['images']:
        image_id = image_info['id']
        file_name = image_info['file_name'].replace('.png', '.txt')
        
        yolo_file = output_dir / file_name
        with open(yolo_file, 'w') as f:
            if image_id in image_annotations:
                for ann in image_annotations[image_id]:
                    # YOLO format: class_id center_x center_y width height (normalized 0 to 1)
                    bbox = ann['bbox']  # [x_min, y_min, width, height]
                    center_x = (bbox[0] + bbox[2] / 2) / image_width
                    center_y = (bbox[1] + bbox[3] / 2) / image_height
                    width = bbox[2] / image_width
                    height = bbox[3] / image_height
                    
                    class_id = category_map[ann['category_id']]
                    
                    f.write(f"{class_id} {center_x} {center_y} {width} {height}\n")

# Usage
coco_file = Path("~/synthetic_data/annotations/annotations.json")
yolo_dir = Path("~/synthetic_data/yolo_labels")
yolo_dir.mkdir(exist_ok=True)

coco_to_yolo(coco_file, yolo_dir, 1920, 1080)
```

### Step 5: Validate Dataset Quality

**Create validation script** (`scripts/validate_dataset.py`):

```python
#!/usr/bin/env python3
"""
Validate synthetic dataset quality
"""
import cv2
import numpy as np
from pathlib import Path
import json

def validate_dataset(data_dir):
    """Check dataset for common issues"""
    rgb_dir = Path(data_dir) / "rgb"
    depth_dir = Path(data_dir) / "depth"
    seg_dir = Path(data_dir) / "segmentation"
    ann_file = Path(data_dir) / "annotations" / "annotations.json"
    
    issues = []
    
    # Check image counts match
    rgb_count = len(list(rgb_dir.glob("*.png")))
    depth_count = len(list(depth_dir.glob("*.png")))
    seg_count = len(list(seg_dir.glob("*.png")))
    
    if rgb_count != depth_count or rgb_count != seg_count:
        issues.append(f"Image count mismatch: RGB={rgb_count}, Depth={depth_count}, Seg={seg_count}")
    
    # Check annotations
    with open(ann_file, 'r') as f:
        annotations = json.load(f)
    
    if len(annotations['images']) != rgb_count:
        issues.append(f"Annotation count mismatch: {len(annotations['images'])} vs {rgb_count}")
    
    # Check image quality (sample check)
    sample_image = cv2.imread(str(rgb_dir / "000000.png"))
    if sample_image is None:
        issues.append("Sample image failed to load")
    else:
        # Check for blank images
        if np.mean(sample_image) < 10:
            issues.append("Sample image appears blank (too dark)")
    
    # Check annotation coverage
    images_with_annotations = set(ann['image_id'] for ann in annotations['annotations'])
    if len(images_with_annotations) < rgb_count * 0.9:  # 90% should have annotations
        issues.append(f"Low annotation coverage: {len(images_with_annotations)}/{rgb_count}")
    
    return issues

# Run validation
data_dir = Path("~/synthetic_data")
issues = validate_dataset(data_dir)

if issues:
    print("Dataset validation issues:")
    for issue in issues:
        print(f"  - {issue}")
else:
    print("Dataset validation passed!")
```

### Step 6: Debugging Common Issues

#### Issue 1: "No images generated" or "Empty directories"
**Symptoms**: Script runs but no files created

**Solutions**:
```python
# Check output directory permissions
import os
print(os.access(OUTPUT_DIR, os.W_OK))  # Should be True

# Verify camera is capturing
rgb_data = self.camera.get_rgba()
print(f"RGB shape: {rgb_data.shape}")  # Should be (H, W, 4)

# Check file paths
print(f"Saving to: {RGB_DIR / '000000.png'}")
```

#### Issue 2: "Segmentation mask all zeros"
**Symptoms**: Segmentation images are blank

**Solutions**:
```python
# Enable semantic segmentation in camera
self.camera.enable_semantic_segmentation()

# Assign semantic labels to objects
from omni.isaac.core.utils.prims import set_targets
set_targets(prim_path="/World/Robot", semantic_label="robot")
set_targets(prim_path="/World/Objects", semantic_label="object")
```

#### Issue 3: "Annotations missing bounding boxes"
**Symptoms**: JSON file has empty annotations array

**Solutions**:
```python
# Verify segmentation data
seg_data = self.camera.get_current_frame()["semantic_segmentation"]
print(f"Segmentation unique values: {np.unique(seg_data)}")

# Check object IDs match category mapping
# Ensure objects have semantic labels assigned
```

#### Issue 4: "Domain randomization not working"
**Symptoms**: All images look similar

**Solutions**:
```python
# Verify randomization is called
print(f"Random seed: {random.getstate()}")

# Check randomization ranges
light_intensity = random.uniform(0.5, 3.0)
print(f"Light intensity: {light_intensity}")  # Should vary

# Reset random seed if needed
random.seed()  # Use system time
```

## Part 3: Advanced Topics (Optional)

### Multi-View Data Collection

**Capture from multiple cameras**:
```python
# Create multiple cameras
cameras = [
    Camera(prim_path="/World/Camera_Front", position=[0, -2, 1.6]),
    Camera(prim_path="/World/Camera_Left", position=[-2, 0, 1.6]),
    Camera(prim_path="/World/Camera_Right", position=[2, 0, 1.6]),
]

# Capture from all cameras
for cam in cameras:
    rgb = cam.get_rgba()
    # Save with camera identifier
```

### Active Learning Pipeline

**Select diverse samples**:
```python
def select_diverse_samples(images, n_samples=100):
    """Select diverse images using clustering"""
    from sklearn.cluster import KMeans
    
    # Extract features (e.g., color histogram)
    features = [extract_features(img) for img in images]
    
    # Cluster
    kmeans = KMeans(n_clusters=n_samples)
    kmeans.fit(features)
    
    # Select samples closest to cluster centers
    # ... (sample selection logic)
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Training data**: Capstone vision models will use synthetic data from Isaac Sim
- **Domain adaptation**: Randomization ensures models generalize to real-world
- **Rapid iteration**: Generate new datasets quickly as requirements change
- **Perfect labels**: Automatic annotations enable supervised learning

Understanding synthetic data generation now is essential for training the capstone perception system.

## Summary

You learned:
- ✅ Set up **data collection pipelines** in Isaac Sim
- ✅ Generated **RGB, depth, and segmentation images** with annotations
- ✅ Implemented **domain randomization** for robust training
- ✅ Exported **datasets in COCO and YOLO formats**
- ✅ Validated **synthetic data quality** and diversity

**Next steps**: In Chapter 3.3, you'll set up hardware-accelerated VSLAM using Isaac ROS.

---

## Exercises

### Exercise 1: Basic Data Collection (Required)

**Objective**: Generate 100 labeled images with bounding boxes.

**Tasks**:
1. Create data collection script
2. Set up scene with robot and 3 objects
3. Configure camera for RGB + segmentation
4. Collect 100 images with annotations
5. Export to COCO format
6. Validate dataset quality

**Acceptance Criteria**:
- [ ] 100 RGB images generated
- [ ] 100 segmentation masks generated
- [ ] COCO annotations file created
- [ ] All images have at least one annotation
- [ ] Dataset validation passes

**Estimated Time**: 120 minutes

### Exercise 2: Domain Randomization (Required)

**Objective**: Implement comprehensive domain randomization.

**Tasks**:
1. Randomize lighting (intensity, color)
2. Randomize object positions and orientations
3. Randomize camera angles
4. Randomize materials (colors, textures)
5. Collect 200 images with randomization
6. Compare diversity (visual inspection)

**Acceptance Criteria**:
- [ ] Lighting varies across images
- [ ] Object positions randomized
- [ ] Camera angles vary
- [ ] Visual diversity confirmed

**Estimated Time**: 90 minutes

### Exercise 3: Multi-Format Export (Challenge)

**Objective**: Export dataset in multiple formats.

**Tasks**:
1. Export to COCO format (already done)
2. Convert to YOLO format
3. Create custom format (e.g., TensorFlow TFRecord)
4. Generate dataset statistics (class distribution, image sizes)
5. Create dataset README with format descriptions

**Requirements**:
- COCO JSON file
- YOLO txt files (one per image)
- Custom format (your choice)
- Statistics report (JSON/Markdown)

**Estimated Time**: 180 minutes

---

## Additional Resources

- [Isaac Sim Data Collection](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_adding_sensors.html) - Official tutorial
- [COCO Format](https://cocodataset.org/#format-data) - Annotation format specification
- [YOLO Format](https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data) - YOLO annotation guide
- [Domain Randomization](https://arxiv.org/abs/1703.06907) - Research paper

---

**Next**: [Chapter 3.3: Isaac ROS - Hardware-Accelerated VSLAM →](chapter-3 to 3.md)
