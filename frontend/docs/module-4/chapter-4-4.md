---
sidebar_position: 5
title: 4.4 Multi-Modal Integration (Speech + Vision + Gesture)
---

# Chapter 4.4: Multi-Modal Integration (Speech + Vision + Gesture)

Human-robot interaction is most natural when robots understand multiple input modalities: speech, vision, and gesture. Combining these modalities enables robust understanding even when individual modalities fail or are ambiguous. This chapter covers fusing multi-modal inputs for unified perception-action pipelines.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Combine** voice commands with visual perception for object identification
- **Implement** gesture recognition for human-robot interaction
- **Fuse** multi-modal inputs for robust understanding
- **Create** unified perception-action pipeline
- **Handle** modality conflicts and ambiguity resolution

## Prerequisites

- **Chapters 4.1 to 4.3** completed (Whisper, LLM planning, action execution)
- **Vision system** (cameras, object detection) from Module 2/3
- **ROS 2 Humble** configured
- **Python 3.10+** with computer vision libraries (OpenCV, mediapipe)
- **Basic ML concepts**: Object detection, pose estimation

## Part 1: Multi-Modal Fusion Fundamentals

### Why Multi-Modal?

**Single modality limitations**:
- **Speech only**: "Pick up the cup" → Which cup? (ambiguous)
- **Vision only**: Sees cup but doesn't know what human wants
- **Gesture only**: Pointing gesture → What action? (unclear)

**Multi-modal advantages**:
- **Disambiguation**: Speech + vision → "Pick up the red cup" + vision identifies red cup
- **Robustness**: If one modality fails, others compensate
- **Rich context**: Multiple cues improve understanding
- **Natural interaction**: Humans use speech + gesture + vision together

### Fusion Strategies

| Strategy | Approach | Use Case |
|----------|----------|----------|
| **Early Fusion** | Combine raw inputs | When modalities are complementary |
| **Late Fusion** | Combine processed outputs | When modalities are independent |
| **Attention-Based** | Weight modalities dynamically | When reliability varies |
| **Hierarchical** | Use one modality to guide another | When one modality is primary |

**This chapter uses**: Late fusion (combine processed outputs) and attention-based weighting.

### Modality Types

| Modality | Input | Processing | Output |
|----------|-------|------------|--------|
| **Speech** | Audio | Whisper → Text | Command text |
| **Vision** | Images | Object detection → Bounding boxes | Object list with poses |
| **Gesture** | Images/IMU | Pose estimation → Gesture classification | Gesture type (point, wave, etc.) |

## Part 2: Hands-On Tutorial

### Project: Multi-Modal Command Understanding

**Goal**: Combine speech, vision, and gesture recognition to understand human commands robustly.

**Tools**: Whisper, OpenCV, MediaPipe, ROS 2 Humble, Python 3.10+

### Step 1: Set Up Vision System

**Install computer vision libraries**:
```bash
pip3 install opencv-python mediapipe numpy
sudo apt install ros-humble-vision-msgs
```

**Create vision node**: `voice_commands/vision_node.py`

```python
#!/usr/bin/env python3
"""
Vision processing node for object detection
ROS 2 Humble | Python 3.10+ | OpenCV
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    """
    Processes camera images and detects objects
    Publishes object detections for multi-modal fusion
    """
    def __init__(self):
        super().__init__('vision_node')
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for detections
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/vision/detections',
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Simple color-based object detection (would use YOLO/SSD in production)
        self.color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255]),
            'green': ([50, 50, 50], [70, 255, 255]),
        }
        
        self.get_logger().info('Vision node started')
    
    def image_callback(self, msg):
        """Process image and detect objects"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect objects (simplified - would use ML model in production)
            detections = self.detect_objects(cv_image)
            
            # Publish detections
            if detections:
                detections_msg = self.create_detections_msg(msg.header, detections)
                self.detections_pub.publish(detections_msg)
                
        except Exception as e:
            self.get_logger().error(f'Vision processing error: {e}')
    
    def detect_objects(self, image):
        """Detect objects in image (simplified color-based detection)"""
        detections = []
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect colored objects
        for color_name, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    center_x = x + w / 2
                    center_y = y + h / 2
                    
                    detections.append({
                        'class': f'{color_name}_cup',  # Simplified class name
                        'confidence': 0.8,
                        'bbox': [x, y, w, h],
                        'center': [center_x, center_y]
                    })
        
        return detections
    
    def create_detections_msg(self, header, detections):
        """Create ROS 2 detection message"""
        detections_msg = Detection2DArray()
        detections_msg.header = header
        
        for det in detections:
            detection = vision_msgs.msg.Detection2D()
            detection.bbox.center.x = det['center'][0]
            detection.bbox.center.y = det['center'][1]
            detection.bbox.size_x = det['bbox'][2]
            detection.bbox.size_y = det['bbox'][3]
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = det['class']
            hypothesis.score = det['confidence']
            detection.results.append(hypothesis)
            
            detections_msg.detections.append(detection)
        
        return detections_msg

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Set Up Gesture Recognition

**Create gesture recognition node**: `voice_commands/gesture_node.py`

```python
#!/usr/bin/env python3
"""
Gesture recognition node using MediaPipe
ROS 2 Humble | Python 3.10+ | MediaPipe
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class GestureNode(Node):
    """
    Recognizes hand gestures from camera images
    """
    def __init__(self):
        super().__init__('gesture_node')
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for gestures
        self.gesture_pub = self.create_publisher(String, '/gestures/recognized', 10)
        
        # MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.bridge = CvBridge()
        
        # Gesture definitions (simplified)
        self.gestures = {
            'point': self.is_pointing_gesture,
            'wave': self.is_waving_gesture,
            'grasp': self.is_grasping_gesture,
        }
        
        self.get_logger().info('Gesture node started')
    
    def image_callback(self, msg):
        """Process image and recognize gestures"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Convert BGR to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = self.hands.process(rgb_image)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Classify gesture
                    gesture = self.classify_gesture(hand_landmarks)
                    
                    if gesture:
                        self.get_logger().info(f'Gesture detected: {gesture}')
                        
                        # Publish gesture
                        msg = String()
                        msg.data = gesture
                        self.gesture_pub.publish(msg)
                        
        except Exception as e:
            self.get_logger().error(f'Gesture recognition error: {e}')
    
    def classify_gesture(self, landmarks):
        """Classify gesture from hand landmarks"""
        # Get key points
        thumb_tip = landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        
        # Simple gesture classification
        if self.is_pointing_gesture(landmarks):
            return 'point'
        elif self.is_waving_gesture(landmarks):
            return 'wave'
        elif self.is_grasping_gesture(landmarks):
            return 'grasp'
        
        return None
    
    def is_pointing_gesture(self, landmarks):
        """Check if pointing gesture (index finger extended, others curled)"""
        # Simplified: Check if index finger is extended and others are not
        index_tip = landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_pip = landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        middle_tip = landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        middle_pip = landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
        
        # Index finger extended (tip above PIP)
        index_extended = index_tip.y < index_pip.y
        # Middle finger curled (tip below PIP)
        middle_curled = middle_tip.y > middle_pip.y
        
        return index_extended and middle_curled
    
    def is_waving_gesture(self, landmarks):
        """Check if waving gesture (all fingers extended, moving)"""
        # Simplified: All fingers extended
        fingers_extended = all(
            landmarks.landmark[i].y < landmarks.landmark[i-2].y
            for i in [4, 8, 12, 16, 20]  # Finger tips
        )
        return fingers_extended
    
    def is_grasping_gesture(self, landmarks):
        """Check if grasping gesture (fingers curled)"""
        # Simplified: Fingers curled
        fingers_curled = all(
            landmarks.landmark[i].y > landmarks.landmark[i-2].y
            for i in [8, 12, 16, 20]  # Finger tips (excluding thumb)
        )
        return fingers_curled

def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create Multi-Modal Fusion Node

**Create fusion node**: `voice_commands/multimodal_fusion.py`

```python
#!/usr/bin/env python3
"""
Multi-modal fusion node combining speech, vision, and gesture
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import json
from collections import deque

class MultimodalFusion(Node):
    """
    Fuses speech, vision, and gesture inputs for robust command understanding
    """
    def __init__(self):
        super().__init__('multimodal_fusion')
        
        # Subscribers
        self.speech_sub = self.create_subscription(
            String,
            '/voice_commands/text',
            self.speech_callback,
            10
        )
        
        self.vision_sub = self.create_subscription(
            Detection2DArray,
            '/vision/detections',
            self.vision_callback,
            10
        )
        
        self.gesture_sub = self.create_subscription(
            String,
            '/gestures/recognized',
            self.gesture_callback,
            10
        )
        
        # Publisher for fused commands
        self.fused_cmd_pub = self.create_publisher(String, '/voice_commands/fused_command', 10)
        
        # Modality buffers (store recent inputs)
        self.speech_buffer = deque(maxlen=5)
        self.vision_buffer = deque(maxlen=10)
        self.gesture_buffer = deque(maxlen=5)
        
        # Fusion parameters
        self.declare_parameter('fusion_timeout', 2.0)  # Seconds to wait for modalities
        self.fusion_timeout = self.get_parameter('fusion_timeout').value
        
        self.get_logger().info('Multi-modal fusion node started')
    
    def speech_callback(self, msg):
        """Handle speech input"""
        text = msg.data.strip()
        if text:
            self.speech_buffer.append({
                'text': text,
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
            self.get_logger().info(f'Speech received: {text}')
            self.attempt_fusion()
    
    def vision_callback(self, msg):
        """Handle vision input"""
        detections = []
        for det in msg.detections:
            if det.results:
                obj_class = det.results[0].id
                confidence = det.results[0].score
                center_x = det.bbox.center.x
                center_y = det.bbox.center.y
                
                detections.append({
                    'class': obj_class,
                    'confidence': confidence,
                    'center': [center_x, center_y]
                })
        
        if detections:
            self.vision_buffer.append({
                'detections': detections,
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
            self.get_logger().info(f'Vision: {len(detections)} objects detected')
            self.attempt_fusion()
    
    def gesture_callback(self, msg):
        """Handle gesture input"""
        gesture = msg.data
        if gesture:
            self.gesture_buffer.append({
                'gesture': gesture,
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
            self.get_logger().info(f'Gesture: {gesture}')
            self.attempt_fusion()
    
    def attempt_fusion(self):
        """Attempt to fuse modalities when new input arrives"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Get recent inputs within timeout window
        recent_speech = [
            s for s in self.speech_buffer
            if current_time - s['timestamp'] < self.fusion_timeout
        ]
        recent_vision = [
            v for v in self.vision_buffer
            if current_time - v['timestamp'] < self.fusion_timeout
        ]
        recent_gesture = [
            g for g in self.gesture_buffer
            if current_time - g['timestamp'] < self.fusion_timeout
        ]
        
        # Fuse if we have speech (primary modality)
        if recent_speech:
            speech_text = recent_speech[-1]['text']  # Most recent
            
            # Extract object references from speech
            objects_mentioned = self.extract_objects_from_speech(speech_text)
            
            # Match with vision detections
            matched_objects = self.match_speech_to_vision(objects_mentioned, recent_vision)
            
            # Incorporate gesture (e.g., pointing)
            gesture_context = recent_gesture[-1]['gesture'] if recent_gesture else None
            
            # Create fused command
            fused_command = self.create_fused_command(
                speech_text,
                matched_objects,
                gesture_context
            )
            
            # Publish fused command
            msg = String()
            msg.data = json.dumps(fused_command)
            self.fused_cmd_pub.publish(msg)
            
            self.get_logger().info(f'Fused command: {fused_command}')
    
    def extract_objects_from_speech(self, text):
        """Extract object mentions from speech"""
        objects = []
        text_lower = text.lower()
        
        # Simple keyword matching (would use NER in production)
        color_keywords = ['red', 'blue', 'green', 'yellow']
        object_keywords = ['cup', 'book', 'bottle', 'object', 'thing']
        
        for color in color_keywords:
            if color in text_lower:
                for obj in object_keywords:
                    if obj in text_lower:
                        objects.append(f'{color}_{obj}')
        
        return objects
    
    def match_speech_to_vision(self, objects_mentioned, vision_data):
        """Match speech objects to vision detections"""
        matched = []
        
        if not vision_data:
            return matched
        
        # Get most recent vision data
        latest_vision = vision_data[-1]['detections']
        
        for obj_mentioned in objects_mentioned:
            # Find matching detection
            for det in latest_vision:
                if obj_mentioned in det['class']:
                    matched.append({
                        'object_id': det['class'],
                        'confidence': det['confidence'],
                        'center': det['center']
                    })
                    break
        
        return matched
    
    def create_fused_command(self, speech_text, matched_objects, gesture):
        """Create fused command with all modalities"""
        command = {
            'speech': speech_text,
            'objects': matched_objects,
            'gesture': gesture,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        # If gesture is pointing, add pointing direction
        if gesture == 'point':
            # Would extract pointing direction from gesture
            command['pointing_direction'] = 'forward'  # Placeholder
        
        return command

def main(args=None):
    rclpy.init(args=args)
    node = MultimodalFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Update LLM Planner for Multi-Modal Input

**Modify LLM planner** to use fused commands:
```python
# In LLMPlanner.__init__
self.fused_cmd_sub = self.create_subscription(
    String,
    '/voice_commands/fused_command',
    self.fused_command_callback,
    10
)

def fused_command_callback(self, msg):
    """Handle fused multi-modal command"""
    fused_cmd = json.loads(msg.data)
    speech_text = fused_cmd.get('speech')
    objects = fused_cmd.get('objects', [])
    gesture = fused_cmd.get('gesture')
    
    # Enhance prompt with vision and gesture context
    enhanced_prompt = f"""Command: {speech_text}

Visual context:
"""
    for obj in objects:
        enhanced_prompt += f"- {obj['object_id']} detected at position {obj['center']}\n"
    
    if gesture:
        enhanced_prompt += f"\nGesture: {gesture}\n"
    
    enhanced_prompt += "\nGenerate action plan considering the visual context and gesture."
    
    # Generate plan with enhanced context
    plan = self.generate_plan_with_prompt(enhanced_prompt)
    # ... (publish plan)
```

### Step 5: Test Multi-Modal Pipeline

**Launch complete system**:
```bash
# Terminal 1: Vision node
ros2 run voice_commands vision_node

# Terminal 2: Gesture node
ros2 run voice_commands gesture_node

# Terminal 3: Multi-modal fusion
ros2 run voice_commands multimodal_fusion

# Terminal 4: LLM planner (updated)
ros2 run voice_commands llm_planner

# Terminal 5: Action executor
ros2 run voice_commands action_executor
```

**Test scenarios**:
1. **Speech + Vision**: Say "Pick up the red cup" while camera sees red cup
2. **Speech + Gesture**: Say "Go there" while pointing
3. **All modalities**: Say "Pick up that cup" + point + camera sees cup

**Monitor fusion**:
```bash
ros2 topic echo /voice_commands/fused_command
```

### Step 6: Debugging Common Issues

#### Issue 1: "Modalities not synchronizing"
**Symptoms**: Speech and vision arrive at different times

**Solutions**:
```python
# Increase fusion timeout
fusion_timeout = 3.0  # Wait longer for modalities

# Use timestamps to align modalities
# Match modalities within time window
```

#### Issue 2: "Object matching fails"
**Symptoms**: Speech mentions "red cup" but vision doesn't match

**Solutions**:
```python
# Improve object extraction from speech
# Use more robust matching (fuzzy matching, synonyms)
# Handle color variations ("red" vs "crimson")

# Add fallback: If no match, use most confident detection
```

#### Issue 3: "Gesture recognition inaccurate"
**Symptoms**: Wrong gestures detected

**Solutions**:
```python
# Improve gesture classification
# Add temporal smoothing (require gesture for N frames)
# Use more sophisticated gesture models
```

## Part 3: Advanced Topics (Optional)

### Attention-Based Fusion

**Weight modalities dynamically**:
```python
def fuse_with_attention(self, speech, vision, gesture):
    """Fuse modalities with attention weights"""
    # Calculate reliability scores
    speech_reliability = self.calculate_speech_reliability(speech)
    vision_reliability = self.calculate_vision_reliability(vision)
    gesture_reliability = self.calculate_gesture_reliability(gesture)
    
    # Normalize weights
    total = speech_reliability + vision_reliability + gesture_reliability
    speech_weight = speech_reliability / total
    vision_weight = vision_reliability / total
    gesture_weight = gesture_reliability / total
    
    # Weighted fusion
    # ...
```

### Temporal Fusion

**Fuse over time windows**:
```python
# Maintain history of inputs
# Fuse across time window (e.g., last 2 seconds)
# Handle temporal relationships (gesture before speech, etc.)
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Robust understanding**: Multi-modal fusion enables robust command understanding
- **Natural interaction**: Supports natural human communication patterns
- **Error recovery**: If one modality fails, others compensate
- **Rich context**: Multiple cues improve action planning accuracy

Understanding multi-modal fusion now is essential for the capstone natural interaction system.

## Summary

You learned:
- ✅ Combined **voice commands with visual perception** for object identification
- ✅ Implemented **gesture recognition** for human-robot interaction
- ✅ Fused **multi-modal inputs** for robust understanding
- ✅ Created **unified perception-action pipeline**
- ✅ Handled **modality conflicts** and ambiguity resolution

**Next steps**: In Chapter 4.5, you'll learn humanoid kinematics and balance control for executing manipulation and navigation actions.

---

## Exercises

### Exercise 1: Basic Multi-Modal Fusion (Required)

**Objective**: Combine speech and vision for object disambiguation.

**Tasks**:
1. Set up vision node detecting objects
2. Create fusion node combining speech and vision
3. Test with ambiguous commands:
   - "Pick up the cup" (multiple cups visible)
   - "Pick up the red cup" (vision identifies red cup)
4. Verify fusion improves command understanding
5. Document fusion improvements

**Acceptance Criteria**:
- [ ] Vision node detecting objects
- [ ] Fusion node combining modalities
- [ ] Ambiguous commands resolved correctly
- [ ] Fusion improves accuracy

**Estimated Time**: 120 minutes

### Exercise 2: Gesture Integration (Required)

**Objective**: Add gesture recognition to multi-modal system.

**Tasks**:
1. Set up gesture recognition node
2. Integrate gestures into fusion pipeline
3. Test scenarios:
   - Pointing gesture + "Go there"
   - Waving gesture + "Come here"
4. Verify gesture enhances commands
5. Document gesture impact

**Acceptance Criteria**:
- [ ] Gesture node recognizing gestures
- [ ] Gestures integrated into fusion
- [ ] Gesture-enhanced commands work correctly

**Estimated Time**: 180 minutes

### Exercise 3: Attention-Based Fusion (Challenge)

**Objective**: Implement dynamic modality weighting.

**Tasks**:
1. Calculate reliability scores for each modality
2. Implement attention-based fusion
3. Test with varying reliability:
   - High noise speech (low reliability)
   - Poor lighting vision (low reliability)
   - Clear gesture (high reliability)
4. Verify attention weights adapt correctly
5. Compare to fixed-weight fusion

**Requirements**:
- Reliability calculation
- Attention-based weighting
- Performance comparison

**Estimated Time**: 240 minutes

---

## Additional Resources

- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands.html) - Gesture recognition
- [OpenCV Object Detection](https://docs.opencv.org/4.x/d9/db7/tutorial_py_table_of_contents_contours.html) - Vision processing
- [Multi-Modal Fusion](https://arxiv.org/abs/2209.03499) - Research paper
- [ROS 2 Vision Messages](https://github.com/ros-perception/vision_msgs) - Vision message types

---

**Next**: [Chapter 4.5: Humanoid Kinematics & Balance Control →](chapter-4 to 5.md)
