---
sidebar_position: 2
title: 2.1 Computer Vision Fundamentals
---

# Chapter 2.1: Computer Vision Fundamentals

Computer vision enables robots to extract meaningful information from images. This chapter covers essential image processing techniques and classical computer vision algorithms used in robotics.

## Learning Outcomes

- **Understand** image representation (pixels, channels, color spaces)
- **Apply** filtering techniques (blur, edge detection)
- **Extract** features (corners, keypoints) for object recognition
- **Implement** basic CV pipelines using OpenCV

## Image Representation

### Digital Images

Images are 2D arrays of pixels. Each pixel stores intensity values.

**Grayscale image**: Single channel (0-255)
```python
import cv2
import numpy as np

# Create 100x100 black image
img = np.zeros((100, 100), dtype=np.uint8)

# Set some pixels to white (255)
img[40:60, 40:60] = 255

cv2.imshow('Image', img)
cv2.waitKey(0)
```

**Color image (RGB)**: Three channels (Red, Green, Blue)
```python
# Color image: height × width × 3
color_img = np.zeros((100, 100, 3), dtype=np.uint8)
color_img[:, :] = [0, 255, 0]  # Green image
```

### Color Spaces

| Color Space | Channels | Use Case |
|-------------|----------|----------|
| **RGB** | Red, Green, Blue | Display, general processing |
| **BGR** | Blue, Green, Red | OpenCV default |
| **HSV** | Hue, Saturation, Value | Color-based segmentation |
| **Grayscale** | Intensity only | Edge detection, faster processing |

**Convert between color spaces**:
```python
# Load image
img = cv2.imread('robot_view.jpg')

# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Convert to HSV (for color detection)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
```

---

## Image Filtering

Filters modify images by applying operations on pixel neighborhoods.

### Blurring (Smoothing)

Reduces noise by averaging neighboring pixels.

```python
# Gaussian blur (most common)
blurred = cv2.GaussianBlur(img, (5, 5), sigmaX=1.5)

# Kernel size must be odd: (3,3), (5,5), (7,7)
# Larger kernel = more blur
```

**Use case**: Reduce camera noise before edge detection

### Edge Detection

Finds boundaries where intensity changes sharply.

**Canny Edge Detector** (most popular):
```python
# Convert to grayscale first
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur
blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)

# Canny edge detection
edges = cv2.Canny(blurred, threshold1=50, threshold2=150)

cv2.imshow('Edges', edges)
cv2.waitKey(0)
```

**Parameters**:
- `threshold1`: Lower threshold (weak edges)
- `threshold2`: Upper threshold (strong edges)
- Adjust based on lighting conditions

---

## Feature Detection

Features are distinctive image points used for object recognition and tracking.

### Corner Detection (Harris)

Corners are points where intensity changes in multiple directions.

```python
# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = np.float32(gray)

# Harris corner detection
corners = cv2.cornerHarris(gray, blockSize=2, ksize=3, k=0.04)

# Threshold to mark corners
img[corners > 0.01 * corners.max()] = [0, 0, 255]  # Red dots

cv2.imshow('Corners', img)
cv2.waitKey(0)
```

### Keypoint Detection (ORB)

**ORB (Oriented FAST and Rotated BRIEF)** detects keypoints and computes descriptors.

```python
# Create ORB detector
orb = cv2.ORB_create(nfeatures=500)

# Detect keypoints and compute descriptors
keypoints, descriptors = orb.detectAndCompute(gray, None)

# Draw keypoints on image
img_keypoints = cv2.drawKeypoints(
    img, keypoints, None,
    color=(0, 255, 0),
    flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
)

cv2.imshow('ORB Keypoints', img_keypoints)
cv2.waitKey(0)
```

**Keypoints** = location + orientation + scale
**Descriptors** = binary vectors describing local appearance

---

## Feature Matching

Match keypoints between two images (useful for object recognition).

```python
# Detect keypoints in both images
orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# Create matcher
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors
matches = bf.match(des1, des2)

# Sort by distance (best matches first)
matches = sorted(matches, key=lambda x: x.distance)

# Draw top 50 matches
img_matches = cv2.drawMatches(
    img1, kp1, img2, kp2, matches[:50], None,
    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
)

cv2.imshow('Matches', img_matches)
cv2.waitKey(0)
```

**Applications**:
- Object detection in cluttered scenes
- Template matching
- Visual odometry (camera-based robot localization)

---

## Practical Example: Object Detection via Color

Detect a red ball using color thresholding in HSV space.

```python
import cv2
import numpy as np

# Capture from camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define red color range in HSV
    # Red wraps around in HSV (0-10 and 170-180)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    # Find contours
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # Draw bounding boxes around detected objects
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Filter small noise
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, 'Red Object', (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('Detection', frame)
    cv2.imshow('Mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

**Use case**: Humanoid robot detecting colored objects for manipulation tasks.

---

## OpenCV Basics

### Installation
```bash
pip install opencv-python opencv-contrib-python numpy
```

### Essential Functions

**Read/Write**:
```python
img = cv2.imread('path/to/image.jpg')
cv2.imwrite('output.jpg', img)
```

**Display**:
```python
cv2.imshow('Window Name', img)
cv2.waitKey(0)  # Wait for keypress
cv2.destroyAllWindows()
```

**Resize**:
```python
resized = cv2.resize(img, (640, 480))  # (width, height)
```

**Draw shapes**:
```python
# Rectangle
cv2.rectangle(img, (x1, y1), (x2, y2), color=(0,255,0), thickness=2)

# Circle
cv2.circle(img, (cx, cy), radius=30, color=(255,0,0), thickness=-1)

# Text
cv2.putText(img, 'Hello', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
```

---

## Exercises

### 1. Color Space Exploration
Load an image and convert it to RGB, HSV, and grayscale. Display all three side-by-side. Which color space makes it easiest to detect a specific colored object?

### 2. Edge Detection Tuning
Take a photo with your phone/webcam. Apply Canny edge detection with different threshold values:
- (50, 150)
- (100, 200)
- (30, 100)

Compare results. Which works best? Why?

### 3. Feature Matching Challenge
Take two photos of the same object from different angles. Use ORB feature matching to find correspondences. How many good matches do you get? What happens if you rotate or scale the object significantly?

### 4. Color-Based Object Tracker
Modify the red ball detection code to:
1. Track a blue object instead (find HSV range)
2. Display the center coordinates of the detected object
3. Draw a trail showing object movement over time

**Hint**: Store previous positions in a list and draw lines between them.

---

## Key Takeaways

✅ Images are 2D/3D arrays of pixel intensity values
✅ **Color spaces** (RGB, HSV) affect how we process images
✅ **Filtering** (blur, edges) extracts structure from images
✅ **Features** (corners, keypoints) enable object recognition
✅ **OpenCV** is the standard library for computer vision in robotics
✅ Classical CV still useful for real-time robotics (fast, lightweight)

---

## Further Reading

- [OpenCV Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) - Official documentation
- *Learning OpenCV* by Bradski & Kaehler - Comprehensive textbook
- [PyImageSearch](https://pyimagesearch.com/) - Practical CV tutorials

---

**Previous**: [← Module 2 Introduction](intro.md) | **Next**: [Chapter 2.2: Deep Learning for Vision →](chapter-2-2.md)

Classical computer vision provides fast, interpretable methods. Next, we'll explore how deep learning revolutionized visual perception!
