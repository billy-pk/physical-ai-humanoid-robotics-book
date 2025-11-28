---
sidebar_position: 3
title: 2.2 Deep Learning for Vision
---

# Chapter 2.2: Deep Learning for Vision

Deep learning transformed computer vision, enabling robots to recognize objects with human-level accuracy. This chapter covers Convolutional Neural Networks (CNNs) and modern object detection for robotics.

## Learning Outcomes

- **Understand** CNN architecture and how it processes images
- **Apply** pre-trained models for object detection
- **Implement** transfer learning for robot-specific tasks
- **Use** YOLO for real-time detection in robotics

## Why Deep Learning for Vision?

**Classical CV limitations**:
- Hand-crafted features (edges, corners) don't generalize well
- Breaks down with lighting changes, occlusions, viewpoint shifts
- Requires expert tuning for each task

**Deep learning advantages**:
- ✅ Learns features automatically from data
- ✅ Robust to variations (lighting, angles, backgrounds)
- ✅ Achieves superhuman accuracy on many tasks
- ✅ Transfer learning: pre-trained models work out-of-the-box

---

## Convolutional Neural Networks (CNNs)

CNNs are specialized neural networks for processing grid-like data (images).

### Key Components

**1. Convolutional Layers**
- Apply filters (kernels) to detect patterns
- Early layers: edges, textures
- Deep layers: object parts, full objects

**2. Pooling Layers**
- Reduce spatial dimensions (downsampling)
- Max pooling: take maximum value in region
- Makes network invariant to small translations

**3. Fully Connected Layers**
- Final layers for classification
- Combine features to make predictions

### CNN Architecture Example

```
Input Image (224×224×3)
     ↓
Conv Layer 1 (32 filters, 3×3) → ReLU → MaxPool
     ↓
Conv Layer 2 (64 filters, 3×3) → ReLU → MaxPool
     ↓
Conv Layer 3 (128 filters, 3×3) → ReLU → MaxPool
     ↓
Flatten → Fully Connected (512) → ReLU → Dropout
     ↓
Output (1000 classes)
```

---

## Object Detection with Pre-Trained Models

Use **pre-trained models** (trained on ImageNet, COCO) instead of training from scratch.

### Example: Image Classification with ResNet

```python
import torch
import torchvision.transforms as transforms
from torchvision.models import resnet50
from PIL import Image
import requests

# Load pre-trained ResNet50
model = resnet50(pretrained=True)
model.eval()

# Image preprocessing (required by ResNet)
preprocess = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )
])

# Load and preprocess image
img = Image.open('robot_view.jpg')
input_tensor = preprocess(img).unsqueeze(0)  # Add batch dimension

# Inference
with torch.no_grad():
    output = model(input_tensor)

# Get predicted class
probabilities = torch.nn.functional.softmax(output[0], dim=0)
top5_prob, top5_idx = torch.topk(probabilities, 5)

# Load ImageNet labels
labels_url = "https://raw.githubusercontent.com/anishathalye/imagenet-simple-labels/master/imagenet-simple-labels.json"
labels = requests.get(labels_url).json()

# Print predictions
for i in range(5):
    print(f"{labels[top5_idx[i]]}: {top5_prob[i].item()*100:.2f}%")
```

**Output example**:
```
coffee mug: 87.34%
cup: 8.21%
espresso: 2.15%
...
```

---

## Real-Time Object Detection: YOLO

**YOLO (You Only Look Once)** is the go-to detector for robotics—fast and accurate.

### Why YOLO for Robots?

| Model | Speed | Accuracy | Use Case |
|-------|-------|----------|----------|
| **Faster R-CNN** | ~5 FPS | Very High | Offline analysis |
| **SSD** | ~20 FPS | Medium | Moderate real-time |
| **YOLOv5/v8** | ~60+ FPS | High | **Robotics (real-time)** |

### Using YOLOv8 (Ultralytics)

```bash
# Install
pip install ultralytics opencv-python
```

```python
from ultralytics import YOLO
import cv2

# Load pre-trained model
model = YOLO('yolov8n.pt')  # 'n' = nano (fastest), 's', 'm', 'l', 'x' (most accurate)

# Method 1: Detect in single image
results = model('robot_workspace.jpg')

# Visualize
annotated = results[0].plot()
cv2.imshow('Detections', annotated)
cv2.waitKey(0)

# Method 2: Real-time webcam detection
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run detection
    results = model(frame, conf=0.5)  # confidence threshold

    # Get bounding boxes
    for result in results:
        boxes = result.boxes
        for box in boxes:
            # Extract box coordinates
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].cpu().numpy()
            cls = int(box.cls[0].cpu().numpy())
            label = model.names[cls]

            # Draw on frame
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                         (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}',
                       (int(x1), int(y1)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('YOLO Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### Getting 3D Position from Detection

```python
# After detecting object in image, estimate 3D position
# (requires depth camera or stereo vision)

def pixel_to_3d(u, v, depth, camera_matrix):
    """
    Convert pixel coordinates to 3D point.

    Args:
        u, v: Pixel coordinates
        depth: Depth value at (u, v) in meters
        camera_matrix: 3x3 camera intrinsic matrix

    Returns:
        (x, y, z) in camera frame
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return x, y, z

# Example usage
bbox_center_u = (x1 + x2) / 2
bbox_center_v = (y1 + y2) / 2
depth_at_center = depth_image[int(bbox_center_v), int(bbox_center_u)]

x, y, z = pixel_to_3d(bbox_center_u, bbox_center_v, depth_at_center, K)
print(f"Object at 3D position: ({x:.2f}, {y:.2f}, {z:.2f}) meters")
```

---

## Transfer Learning for Custom Objects

Train a detector for robot-specific objects (tools, parts, etc.).

### Process

1. **Collect images** of your objects (~100-500 per class)
2. **Annotate** bounding boxes (use tools like LabelImg, Roboflow)
3. **Fine-tune** pre-trained YOLO on your dataset

### Fine-Tuning YOLOv8

```python
from ultralytics import YOLO

# Load pre-trained model
model = YOLO('yolov8n.pt')

# Train on custom dataset
results = model.train(
    data='custom_data.yaml',  # Dataset config
    epochs=50,
    imgsz=640,
    batch=16,
    name='robot_tools_detector'
)

# Use trained model
model = YOLO('runs/detect/robot_tools_detector/weights/best.pt')
results = model('test_image.jpg')
```

**Dataset config** (`custom_data.yaml`):
```yaml
train: /path/to/train/images
val: /path/to/val/images

nc: 3  # Number of classes
names: ['wrench', 'screwdriver', 'bolt']
```

**Why this works**:
- Pre-trained model already knows edges, shapes, textures
- Fine-tuning adapts it to your specific objects
- Needs far less data than training from scratch

---

## Semantic Segmentation

Classify every pixel (not just bounding boxes).

**Use cases**:
- Terrain classification (road, grass, obstacles)
- Grasp point detection (object vs. background)
- Scene understanding

### Example with DeepLabV3

```python
import torchvision
import torch

# Load pre-trained segmentation model
model = torchvision.models.segmentation.deeplabv3_resnet50(pretrained=True)
model.eval()

# Preprocess image
input_tensor = preprocess(img).unsqueeze(0)

# Inference
with torch.no_grad():
    output = model(input_tensor)['out'][0]

# Get class for each pixel
output_predictions = output.argmax(0).byte().cpu().numpy()

# Visualize (color-code each class)
from matplotlib import pyplot as plt
plt.imshow(output_predictions)
plt.show()
```

---

## Practical Tips for Robotics

### 1. Model Selection

**For real-time robotics (>30 FPS)**:
- YOLOv8n (nano) or YOLOv8s (small)
- MobileNetV3 + SSD

**For accuracy-critical tasks**:
- YOLOv8l/x
- Faster R-CNN with ResNet101

### 2. Hardware Acceleration

```python
# Use GPU if available
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model.to(device)

# For edge devices (Jetson, Raspberry Pi)
# Use TensorRT or ONNX optimization
```

### 3. Confidence Thresholding

```python
# Adjust confidence based on application
results = model(frame, conf=0.7)  # High conf = fewer false positives

# For safety-critical: higher threshold (0.8-0.9)
# For exploration: lower threshold (0.3-0.5)
```

---

## Exercises

### 1. Pre-Trained Model Exploration
Download YOLOv8 and run it on 5 different images. Compare detection quality with different model sizes (yolov8n vs yolov8m). What's the FPS difference?

### 2. Custom Object Detection
Create a small dataset (20-30 images) of an object in your environment (coffee mug, book, etc.). Annotate bounding boxes using [Roboflow](https://roboflow.com/). Fine-tune YOLOv8 and test it.

### 3. 3D Position Estimation
Combine YOLO detection with a depth camera (Intel RealSense or Kinect). For each detected object, compute its 3D position relative to the robot. Print coordinates.

### 4. Segmentation vs Detection
Compare outputs of YOLOv8 (bounding boxes) vs DeepLabV3 (segmentation) on the same image. Which is better for:
- Counting objects?
- Identifying traversable terrain?
- Precise grasping?

---

## Key Takeaways

✅ **CNNs** automatically learn hierarchical visual features
✅ **Pre-trained models** (ImageNet, COCO) work well out-of-the-box
✅ **YOLO** is the best choice for real-time robotic vision (60+ FPS)
✅ **Transfer learning** enables custom detectors with limited data
✅ **Segmentation** provides pixel-level understanding for complex tasks
✅ Always use **GPU** for deep learning inference on robots

---

## Further Reading

- [PyTorch Vision Tutorials](https://pytorch.org/tutorials/beginner/transfer_learning_tutorial.html)
- [Ultralytics YOLO Docs](https://docs.ultralytics.com/)
- *Deep Learning for Computer Vision* (Stanford CS231n course notes)

---

**Previous**: [← Chapter 2.1: CV Fundamentals](chapter-2-1.md) | **Next**: [Chapter 2.3: 3D Vision and Depth →](chapter-2-3.md)

With 2D detection mastered, let's explore how robots perceive depth and understand 3D space!
