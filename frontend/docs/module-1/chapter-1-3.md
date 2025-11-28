---
sidebar_position: 3
title: 1.3 Sensors and Actuators
---

# Chapter 1.3: Sensors and Actuators

Robots need eyes to see, ears to hear, and muscles to move. This chapter explores the **sensors** that enable robots to perceive their environment and the **actuators** that allow them to act upon it. These are the interface between the digital brain (AI) and the physical world.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Classify** major sensor types: proprioceptive vs. exteroceptive
- **Understand** how cameras, LIDAR, IMUs, and force sensors work
- **Explain** actuator types: electric motors, hydraulics, pneumatics
- **Compare** sensor and actuator characteristics (range, accuracy, bandwidth)
- **Apply** sensor fusion concepts to combine multiple data sources
- **Design** sensor/actuator systems for specific robotic applications

## Sensors: Perceiving the World

**Sensors** convert physical phenomena (light, force, acceleration) into electrical signals that robots can process.

### Sensor Classification

**1. Proprioceptive Sensors** (Internal State)
- Measure robot's own configuration and motion
- Examples: encoders, IMUs, force/torque sensors in joints

**2. Exteroceptive Sensors** (External Environment)
- Measure external world properties
- Examples: cameras, LIDAR, microphones, tactile sensors

| Sensor Type | Category | Measures | Typical Use |
|-------------|----------|----------|-------------|
| **Encoder** | Proprioceptive | Joint position/velocity | Motor control, odometry |
| **IMU** | Proprioceptive | Acceleration, angular velocity | Balance, orientation |
| **Camera** | Exteroceptive | Light intensity (RGB) | Object detection, navigation |
| **LIDAR** | Exteroceptive | Distance (time-of-flight) | 3D mapping, obstacle avoidance |
| **Force/Torque** | Proprioceptive | Contact forces | Manipulation, collision detection |
| **Microphone** | Exteroceptive | Sound waves | Voice commands, audio localization |

---

## Vision Sensors: Cameras

**Cameras** are the most common sensors in robotics, providing rich visual information.

### Camera Types

**1. RGB Cameras** (Standard Color)
- Capture red, green, blue channels
- Resolution: 640×480 (VGA) to 4K+
- Frame rate: 30-120 fps typical
- **Pros**: Rich color info, inexpensive, mature algorithms
- **Cons**: No depth info, sensitive to lighting

**2. Depth Cameras** (RGB-D)
- Provide color + per-pixel distance
- Technologies:
  - **Structured light**: Project pattern, measure distortion (Intel RealSense)
  - **Time-of-flight (ToF)**: Measure light travel time (Microsoft Kinect)
  - **Stereo**: Use two cameras like human eyes

**3. Event Cameras** (DVS - Dynamic Vision Sensor)
- Each pixel independently detects brightness changes
- **Pros**: Ultra-low latency (&lt;1ms), high dynamic range
- **Cons**: Novel technology, fewer algorithms available

### Camera Parameters

**Intrinsic Parameters**:
- **Focal length** (f): Distance from lens to sensor
- **Principal point** (cₓ, cᵧ): Optical center in image
- **Lens distortion**: Radial and tangential distortion coefficients

**Camera Calibration** determines these parameters for accurate 3D reconstruction.

### Example: Reading Camera in Python

```python
import cv2

# Open camera (0 = default camera)
cap = cv2.VideoCapture(0)

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    # Capture frame
    ret, frame = cap.read()

    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display
    cv2.imshow('Robot Camera View', gray)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## LIDAR: Laser Distance Measurement

**LIDAR** (Light Detection and Ranging) uses laser pulses to measure distances, creating precise 3D point clouds.

### How LIDAR Works

1. **Emit** laser pulse
2. **Measure** time for reflection to return
3. **Calculate** distance: d = (c × t) / 2
   - c = speed of light (3×10⁸ m/s)
   - t = round-trip time

### LIDAR Types

**1D LIDAR**: Single beam (distance sensor)
**2D LIDAR**: Scanning in plane (e.g., Hokuyo, SICK) — common in mobile robots
**3D LIDAR**: Full 3D scanning (e.g., Velodyne, Ouster) — autonomous vehicles

### LIDAR Specifications

| Specification | Typical Range | Example |
|---------------|---------------|---------|
| **Range** | 0.1m - 100m | Indoor: 10m, Outdoor: 50m+ |
| **Angular resolution** | 0.1° - 1° | Velodyne: 0.4° horizontal |
| **Scan rate** | 5 - 20 Hz | 10 Hz = 10 scans/second |
| **Accuracy** | ±2cm - ±5cm | High-end: ±2cm |

### Applications in Humanoid Robots

- **Navigation**: Avoid obstacles, map environment
- **Localization**: Determine position in known map (SLAM)
- **Object detection**: Identify objects by shape
- **Terrain analysis**: Detect stairs, slopes, uneven ground

---

## IMU: Inertial Measurement Unit

**IMUs** measure acceleration and rotational velocity, crucial for balance and orientation.

### IMU Components

**1. Accelerometer** (3-axis)
- Measures linear acceleration in x, y, z
- Detects gravity (when stationary)
- Used for: orientation estimation, fall detection

**2. Gyroscope** (3-axis)
- Measures angular velocity (rotation rate)
- Used for: orientation tracking, stabilization

**3. Magnetometer** (3-axis, optional)
- Measures magnetic field (compass)
- Provides absolute heading (north reference)
- **Challenge**: Susceptible to magnetic interference

### IMU Data Fusion

Raw IMU data is noisy. **Sensor fusion** combines accelerometer and gyroscope:

**Complementary Filter** (simple approach):
```python
import numpy as np

class ComplementaryFilter:
    """
    Fuse accelerometer and gyroscope for tilt angle estimation.
    """
    def __init__(self, alpha=0.98, dt=0.01):
        """
        Args:
            alpha: Filter coefficient (0-1, higher = trust gyro more)
            dt: Time step (seconds)
        """
        self.alpha = alpha
        self.dt = dt
        self.angle = 0.0  # Current estimated angle

    def update(self, accel_angle, gyro_rate):
        """
        Update angle estimate.

        Args:
            accel_angle: Angle from accelerometer (degrees)
            gyro_rate: Angular velocity from gyroscope (deg/s)

        Returns:
            Estimated angle (degrees)
        """
        # Integrate gyroscope (high-frequency, drifts over time)
        gyro_angle = self.angle + gyro_rate * self.dt

        # Combine with accelerometer (low-frequency, stable long-term)
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle

        return self.angle

# Example usage
filter = ComplementaryFilter(alpha=0.98, dt=0.01)

# Simulate IMU readings
accel_angle = 45.0  # Tilt from accelerometer
gyro_rate = 2.0     # Rotating at 2 deg/s

for _ in range(100):
    estimated_angle = filter.update(accel_angle, gyro_rate)
    print(f"Estimated angle: {estimated_angle:.2f}°")
```

**Advanced Fusion**: Kalman Filter, Madgwick Filter (quaternion-based)

### Humanoid Robot Applications

- **Balance control**: Detect tipping, adjust posture
- **Fall detection**: Trigger protective response
- **Orientation estimation**: Know which way is "up"
- **Gait analysis**: Monitor walking patterns

---

## Force and Torque Sensors

**Force/Torque (F/T) sensors** measure contact forces, essential for manipulation and safe interaction.

### Where They're Used

- **Wrist**: Measure forces during grasping/manipulation
- **Feet**: Measure ground reaction forces (ZMP estimation)
- **Joints**: Detect collisions, measure load

### Sensing Principles

**Strain Gauges**: Measure deformation of material under load
**Capacitive**: Measure capacitance change under pressure
**Piezoelectric**: Generate voltage when compressed

### 6-Axis F/T Sensor

Measures:
- **Forces**: Fₓ, Fᵧ, Fz (3 axes)
- **Torques**: τₓ, τᵧ, τz (3 axes)

**Applications**:
```python
# Example: Detect collision from force spike
def detect_collision(force_reading, threshold=50.0):
    """
    Detect collision based on force magnitude.

    Args:
        force_reading: (Fx, Fy, Fz) in Newtons
        threshold: Force threshold for collision (N)

    Returns:
        True if collision detected
    """
    force_magnitude = np.linalg.norm(force_reading)
    return force_magnitude > threshold

# Simulated force reading
force = np.array([5.0, 10.0, 45.0])  # Mostly vertical

if detect_collision(force, threshold=30.0):
    print("Collision detected! Emergency stop.")
else:
    print("Normal operation.")
```

---

## Actuators: Making Robots Move

**Actuators** convert electrical energy into mechanical motion. The choice of actuator determines robot capabilities, cost, and complexity.

### Actuator Comparison

| Type | Power-to-Weight | Speed | Precision | Cost | Common Use |
|------|-----------------|-------|-----------|------|------------|
| **DC Motor** | Medium | High | High | Low | Small robots, joints |
| **Servo Motor** | Medium | Medium | Very High | Medium | Precise positioning |
| **Stepper Motor** | Low | Low | High | Low | 3D printers, slow motion |
| **Brushless DC (BLDC)** | High | Very High | High | Medium | Drones, high-performance robots |
| **Hydraulic** | Very High | Medium | Medium | High | Heavy-duty (construction, Atlas) |
| **Pneumatic** | Medium | High | Low | Medium | Grippers, soft robotics |

---

## Electric Motors

**Electric motors** are the most common actuators in humanoid robots.

### DC Motor (Brushed)

**Pros**:
- Simple to control (voltage → speed)
- Inexpensive
- Good torque-to-weight ratio

**Cons**:
- Brushes wear out over time
- Less efficient than brushless
- Electrical noise

**Control Example**:
```python
import RPi.GPIO as GPIO

class DCMotorController:
    """
    Simple DC motor controller using PWM.
    """
    def __init__(self, enable_pin, in1_pin, in2_pin):
        GPIO.setmode(GPIO.BCM)
        self.enable_pin = enable_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin

        GPIO.setup(enable_pin, GPIO.OUT)
        GPIO.setup(in1_pin, GPIO.OUT)
        GPIO.setup(in2_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(enable_pin, 1000)  # 1kHz frequency
        self.pwm.start(0)

    def set_speed(self, speed):
        """
        Set motor speed.

        Args:
            speed: -100 to 100 (negative = reverse)
        """
        if speed > 0:
            GPIO.output(self.in1_pin, GPIO.HIGH)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.pwm.ChangeDutyCycle(abs(speed))
        elif speed < 0:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(abs(speed))
        else:
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.LOW)
            self.pwm.ChangeDutyCycle(0)

    def stop(self):
        self.set_speed(0)
        self.pwm.stop()
        GPIO.cleanup()
```

### Servo Motor

**Servos** combine motor + encoder + controller for precise position control.

**Types**:
- **Standard servo**: Limited rotation (0-180°)
- **Continuous rotation servo**: Full 360° rotation
- **Digital servo**: Faster response, more precise

**Control**: PWM signal encodes desired angle
```
Pulse width: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
```

**Example with Python**:
```python
import time
import pigpio  # Raspberry Pi GPIO library

class ServoController:
    """
    Control hobby servo motor (0-180°).
    """
    def __init__(self, pin, min_pulse=500, max_pulse=2500):
        self.pi = pigpio.pi()
        self.pin = pin
        self.min_pulse = min_pulse  # microseconds for 0°
        self.max_pulse = max_pulse  # microseconds for 180°

    def set_angle(self, angle):
        """
        Set servo angle.

        Args:
            angle: 0-180 degrees
        """
        angle = max(0, min(180, angle))  # Clamp to valid range
        pulse_width = self.min_pulse + (angle / 180.0) * (self.max_pulse - self.min_pulse)
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)

    def cleanup(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.pi.stop()

# Usage
servo = ServoController(pin=18)
servo.set_angle(90)  # Move to center
time.sleep(1)
servo.set_angle(0)   # Move to 0°
servo.cleanup()
```

### Brushless DC Motor (BLDC)

**Advantages**:
- High efficiency (85-90%)
- Long lifespan (no brushes)
- High power-to-weight ratio
- Quiet operation

**Disadvantages**:
- Requires electronic speed controller (ESC)
- More complex control (requires rotor position feedback)

**Applications**: Quadcopters, high-performance humanoid joints

---

## Hydraulic and Pneumatic Actuators

### Hydraulic Actuators

**Principle**: Pressurized fluid (oil) drives piston

**Pros**:
- Extremely high force (Boston Dynamics Atlas uses hydraulics)
- High power density
- Natural compliance (shock absorption)

**Cons**:
- Heavy (pump, reservoir, valves)
- Messy (oil leaks)
- Complex maintenance
- Expensive

**Use cases**: Heavy-duty humanoids, construction robots

### Pneumatic Actuators

**Principle**: Compressed air drives piston or inflates chamber

**Pros**:
- Safe (air is compressible → inherent compliance)
- Fast actuation
- Clean (no fluids)

**Cons**:
- Low precision (air compressibility)
- Requires air compressor
- Noisy

**Use cases**: Grippers, soft robotics, prosthetics

---

## Sensor Fusion: Combining Multiple Sensors

No single sensor is perfect. **Sensor fusion** combines multiple sensors to overcome individual limitations.

### Why Sensor Fusion?

| Sensor | Strengths | Weaknesses |
|--------|-----------|------------|
| **Camera** | Rich visual info, object recognition | No depth, lighting-dependent |
| **LIDAR** | Accurate depth, works in dark | Expensive, no color/texture |
| **IMU** | High-frequency orientation | Drifts over time |
| **GPS** | Absolute position outdoors | Inaccurate indoors, slow updates |

**Fusion examples**:
- **Camera + LIDAR**: Color point clouds, semantic segmentation with depth
- **IMU + Encoder**: Accurate odometry (wheel slippage detection)
- **Camera + IMU**: Visual-inertial odometry (VIO) for drones

### Kalman Filter: The Classic Fusion Algorithm

**Kalman filter** optimally combines noisy sensor measurements with predictions.

**Concept**:
1. **Predict** next state using motion model
2. **Update** prediction with new sensor measurement
3. **Weight** prediction vs. measurement based on uncertainty

**Example: Fusing GPS and IMU for position**

```python
import numpy as np

class SimpleKalmanFilter:
    """
    1D Kalman filter for position estimation.
    """
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance  # Model uncertainty
        self.measurement_variance = measurement_variance  # Sensor noise

        self.position = 0.0
        self.variance = 1.0

    def predict(self, motion):
        """
        Predict next position based on motion.

        Args:
            motion: Expected movement (e.g., from IMU integration)
        """
        self.position += motion
        self.variance += self.process_variance

    def update(self, measurement):
        """
        Update estimate with new sensor measurement.

        Args:
            measurement: Sensor reading (e.g., from GPS)
        """
        # Kalman gain (how much to trust measurement vs. prediction)
        kalman_gain = self.variance / (self.variance + self.measurement_variance)

        # Update position
        self.position += kalman_gain * (measurement - self.position)

        # Update variance (uncertainty)
        self.variance = (1 - kalman_gain) * self.variance

    def get_state(self):
        return self.position, self.variance

# Example usage
kf = SimpleKalmanFilter(process_variance=0.1, measurement_variance=1.0)

# Simulate sensor readings
for t in range(10):
    # IMU predicts 1m movement
    kf.predict(motion=1.0)

    # GPS measures position (with noise)
    gps_reading = t + np.random.normal(0, 0.5)
    kf.update(gps_reading)

    pos, var = kf.get_state()
    print(f"Time {t}: Position = {pos:.2f}m, Uncertainty = {var:.3f}")
```

---

## Designing a Sensor Suite for Humanoid Robots

When selecting sensors for a humanoid robot, consider:

### Design Checklist

**1. Task Requirements**
- What must the robot sense? (objects, terrain, humans)
- Required range, accuracy, update rate?

**2. Environmental Constraints**
- Indoor vs. outdoor (lighting, weather)
- Structured vs. unstructured environment
- Presence of humans (safety sensors needed)

**3. Computational Resources**
- Processing power for sensor data (cameras are data-heavy)
- Real-time requirements (latency budgets)

**4. Cost and Power**
- Budget constraints
- Battery life impact

**5. Redundancy**
- Critical sensors should have backups
- Diverse sensor modalities (don't rely on vision alone)

### Example: Warehouse Humanoid Robot

**Task**: Navigate warehouse, pick items, avoid humans

**Sensor Suite**:
- **2× RGB cameras** (stereo pair): Object recognition, depth estimation
- **1× 2D LIDAR**: Floor-level obstacle detection, navigation
- **1× IMU**: Balance, orientation
- **2× Force sensors** (wrists): Grasp feedback
- **4× Bumper switches** (body): Emergency collision detection
- **1× Microphone array**: Voice commands

**Rationale**:
- Stereo cameras: Sufficient for indoors, cheaper than LIDAR
- 2D LIDAR: Reliable navigation, lower cost than 3D
- Force sensors: Essential for manipulation
- Redundancy: Vision + LIDAR + bumpers for safety

---

## Exercises

### 1. Sensor Selection
For each scenario, select the most appropriate sensor and justify:
- Detecting if a door is open or closed from 5m away
- Measuring precise angle of robot's elbow joint
- Detecting when robot's hand touches an object
- Determining if robot is tilted (about to fall)

### 2. Camera Resolution Trade-off
A robot has limited processing power (can analyze 10 million pixels/second).
- Option A: 1280×720 at 30 fps
- Option B: 1920×1080 at 15 fps

Which would you choose for:
a) Fast-moving object tracking
b) Detailed object recognition
c) Indoor navigation

### 3. IMU Integration
Given accelerometer reading: (0.5, 0.2, 9.6) m/s² when robot is stationary, calculate the tilt angle from vertical. (Hint: Gravity is 9.81 m/s²)

### 4. Motor Sizing
A robot arm link has:
- Mass: 1.5 kg
- Length: 0.4 m (center of mass at 0.2 m from joint)
- Maximum angular acceleration: 10 rad/s²

Calculate the required motor torque (ignoring gravity for simplicity).
Formula: τ = I·α, where I = m·r² for point mass

### 5. Code Challenge
Implement a **median filter** to remove noise spikes from LIDAR data:
```python
def median_filter(distances, window_size=5):
    """
    Apply median filter to distance measurements.

    Args:
        distances: List of distance readings
        window_size: Size of median window (odd number)

    Returns:
        Filtered distances
    """
    # Your code here
    pass
```

### 6. Sensor Fusion Design
Design a sensor fusion scheme for estimating a humanoid robot's position in a building. You have:
- GPS (accurate outdoors, unreliable indoors)
- IMU (high-frequency, drifts)
- LIDAR (for localization via map matching)
- Wheel encoders (measure distance traveled)

Describe which sensors to use in which situations and how to combine them.

### 7. Research Task
Find specifications for a real humanoid robot (e.g., NAO, Pepper, Atlas, Optimus). List:
- All sensors it uses
- All actuator types
- Estimated cost (if available)
- One limitation in its sensor/actuator design

---

## Key Takeaways

✅ **Proprioceptive sensors** measure internal state; **exteroceptive sensors** measure environment
✅ **Cameras** provide rich visual data but lack depth; **LIDAR** provides accurate 3D geometry
✅ **IMUs** are essential for balance and orientation in mobile robots
✅ **Force sensors** enable safe, compliant manipulation
✅ **Electric motors** (DC, servo, BLDC) are most common; **hydraulics** provide extreme power
✅ **Sensor fusion** combines multiple sensors to overcome individual limitations
✅ Sensor/actuator selection involves **task requirements, cost, power, and redundancy**

---

## Further Reading

- **Books**:
  - *Probabilistic Robotics* by Thrun, Burgard, Fox (sensor models, Kalman filtering)
  - *Robotics: Modelling, Planning and Control* by Siciliano et al. (actuators, control)

- **Papers**:
  - "LIDAR-based 3D Object Perception" (survey of LIDAR processing)
  - "A Comparison of Kalman Filtering Approaches for Sensor Fusion" (fusion techniques)

- **Tutorials**:
  - OpenCV documentation (camera calibration, image processing)
  - ROS sensor integration tutorials (LIDAR, IMU, cameras)

- **Datasheets**:
  - Intel RealSense D435 (depth camera)
  - Velodyne VLP-16 (3D LIDAR)
  - Bosch BMI088 (IMU)
  - Dynamixel servos (common in research robots)

---

**Previous**: [← Chapter 1.2: Robotics Fundamentals](chapter-1-2.md) | **Next**: [Chapter 1.4: ROS and Simulation →](chapter-1-4.md)

With sensors and actuators understood, we're ready to learn the tools roboticists use to build and test systems: ROS and simulation!
