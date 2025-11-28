---
sidebar_position: 2
title: 1.2 Robotics Fundamentals
---

# Chapter 1.2: Robotics Fundamentals

Now that we understand AI fundamentals, let's explore the physical side of Physical AI: **robotics**. This chapter covers core robotics concepts—kinematics, dynamics, control theory—essential for understanding how humanoid robots move and interact with the world.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Understand** robot anatomy: links, joints, degrees of freedom (DOF)
- **Define** forward and inverse kinematics and their applications
- **Explain** robot dynamics: forces, torques, and equations of motion
- **Describe** control system fundamentals (PID, feedforward, feedback)
- **Apply** basic kinematic calculations for simple robot arms
- **Recognize** challenges unique to bipedal humanoid robots

## Robot Anatomy: Links, Joints, and Degrees of Freedom

A robot's physical structure determines what motions it can perform. Understanding this anatomy is foundational to robotics.

### Basic Components

**Links (Rigid Bodies)**
- Physical segments of the robot (e.g., upper arm, forearm, thigh)
- Connected by joints
- Have mass, inertia, and geometry

**Joints (Connections)**
- Allow relative motion between links
- Types:
  - **Revolute (R)**: Rotation around an axis (like human elbow)
  - **Prismatic (P)**: Linear sliding motion (like a drawer)
  - **Spherical**: 3-DOF ball-and-socket (like human shoulder)
  - **Fixed**: No motion (welded connection)

**Degrees of Freedom (DOF)**
- Number of independent ways a robot can move
- Each joint contributes DOF:
  - Revolute joint: 1 DOF
  - Prismatic joint: 1 DOF
  - Spherical joint: 3 DOF

### Example: Human Arm vs. Robot Arm

| Segment | Human | Typical Robot Arm |
|---------|-------|-------------------|
| **Shoulder** | 3 DOF (spherical) | 3 DOF (3 revolute joints) |
| **Elbow** | 1 DOF (hinge) | 1 DOF (revolute) |
| **Wrist** | 3 DOF (complex) | 3 DOF (3 revolute joints) |
| **Total DOF** | 7 DOF per arm | 7 DOF per arm |

**Humanoid robots** typically have:
- **Total**: 25-40 DOF for full body
- **Legs**: 6 DOF per leg (hip: 3, knee: 1, ankle: 2)
- **Arms**: 7 DOF per arm
- **Torso**: 1-3 DOF
- **Head**: 2-3 DOF (pan, tilt, sometimes roll)
- **Hands**: 5-20 DOF depending on dexterity

### Task Space vs. Joint Space

- **Joint Space**: Describes robot configuration using joint angles (θ₁, θ₂, ..., θₙ)
- **Task Space (Cartesian Space)**: Describes end-effector position and orientation in 3D space (x, y, z, roll, pitch, yaw)

**Example**:
```
Joint Space: [shoulder: 45°, elbow: 90°, wrist: 30°]
Task Space: hand position at (0.5m, 0.3m, 1.2m)
```

## Kinematics: The Geometry of Motion

**Kinematics** studies motion without considering forces. For robots, it's about relating joint angles to end-effector position.

### Forward Kinematics (FK)

**Definition**: Given joint angles, compute end-effector position and orientation.

**Formula**: Position = f(joint angles)

**Example - 2-Link Planar Robot Arm**:

```
Link 1 length: L₁ = 1.0m, angle: θ₁
Link 2 length: L₂ = 0.8m, angle: θ₂

End-effector position:
x = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂)
y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂)

If θ₁ = 30°, θ₂ = 45°:
x = 1.0·cos(30°) + 0.8·cos(75°) ≈ 1.073m
y = 1.0·sin(30°) + 0.8·sin(75°) ≈ 1.273m
```

**Code Example**:

```python
import numpy as np

def forward_kinematics_2link(theta1, theta2, L1=1.0, L2=0.8):
    """
    Compute end-effector position for 2-link planar arm.

    Args:
        theta1: First joint angle (radians)
        theta2: Second joint angle (radians)
        L1: Length of first link (meters)
        L2: Length of second link (meters)

    Returns:
        (x, y) position of end-effector
    """
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

# Example usage
theta1 = np.deg2rad(30)  # Convert degrees to radians
theta2 = np.deg2rad(45)
x, y = forward_kinematics_2link(theta1, theta2)
print(f"End-effector position: ({x:.3f}m, {y:.3f}m)")
# Output: End-effector position: (1.073m, 1.273m)
```

### Inverse Kinematics (IK)

**Definition**: Given desired end-effector position, compute required joint angles.

**Formula**: Joint angles = f⁻¹(desired position)

**Challenge**: Much harder than FK!
- May have **multiple solutions** (elbow up vs. elbow down)
- May have **no solution** (target out of reach)
- **Computationally expensive** for complex robots

**Example - 2-Link Planar Robot (Analytical Solution)**:

```python
def inverse_kinematics_2link(x, y, L1=1.0, L2=0.8, elbow_up=True):
    """
    Compute joint angles for desired end-effector position.

    Args:
        x, y: Desired end-effector position
        L1, L2: Link lengths
        elbow_up: If True, use elbow-up solution

    Returns:
        (theta1, theta2) joint angles in radians

    Raises:
        ValueError: If position is unreachable
    """
    # Check if position is reachable
    distance = np.sqrt(x**2 + y**2)
    if distance > L1 + L2 or distance < abs(L1 - L2):
        raise ValueError("Position unreachable!")

    # Law of cosines for theta2
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)  # Numerical stability

    if elbow_up:
        theta2 = np.arccos(cos_theta2)
    else:
        theta2 = -np.arccos(cos_theta2)

    # Solve for theta1
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

# Example usage
target_x, target_y = 1.5, 0.5
theta1, theta2 = inverse_kinematics_2link(target_x, target_y)
print(f"Joint angles: θ₁={np.rad2deg(theta1):.1f}°, θ₂={np.rad2deg(theta2):.1f}°")

# Verify with forward kinematics
x_check, y_check = forward_kinematics_2link(theta1, theta2)
print(f"Verification: ({x_check:.3f}, {y_check:.3f}) ≈ ({target_x}, {target_y})")
```

**For Humanoid Robots**:
- IK is **critical** for task execution (reach to a point, place foot)
- Usually solved **numerically** (optimization, Jacobian methods)
- Real-time requirements demand fast IK solvers

## Dynamics: Forces and Motion

While kinematics ignores forces, **dynamics** explicitly models them using Newton's laws.

### Key Concepts

**Newton's Second Law**: F = ma (Force = mass × acceleration)

For rotational motion: **τ = I·α** (Torque = inertia × angular acceleration)

**Robot Dynamics Equation** (Lagrangian formulation):

```
τ = M(q)·q̈ + C(q, q̇)·q̇ + G(q)

Where:
τ = Joint torques (what motors must produce)
M(q) = Mass/inertia matrix (depends on configuration q)
C(q, q̇) = Coriolis and centrifugal terms (velocity-dependent)
G(q) = Gravity terms
q = Joint positions, q̇ = velocities, q̈ = accelerations
```

**Forward Dynamics**: Given torques → compute accelerations
**Inverse Dynamics**: Given desired motion → compute required torques

### Why Dynamics Matter

For humanoid robots, understanding dynamics is essential for:

1. **Motor Selection**: Knowing required torques helps choose appropriate actuators
2. **Control**: Feedforward control uses inverse dynamics to compensate for gravity and inertia
3. **Simulation**: Predicting how robot will move under applied forces
4. **Energy Efficiency**: Optimal trajectories minimize energy consumption

### Example: Gravity Compensation

```python
def gravity_torque_single_joint(mass, length, theta):
    """
    Calculate torque needed to hold a link against gravity.

    Args:
        mass: Link mass (kg)
        length: Distance from joint to center of mass (m)
        theta: Joint angle from horizontal (radians)

    Returns:
        Required torque (N·m)
    """
    g = 9.81  # Gravity (m/s²)
    torque = mass * g * length * np.cos(theta)
    return torque

# Example: Humanoid arm link
# Mass: 2kg, center of mass 0.3m from shoulder, angle 45° from horizontal
arm_mass = 2.0
arm_length = 0.3
arm_angle = np.deg2rad(45)

torque_needed = gravity_torque_single_joint(arm_mass, arm_length, arm_angle)
print(f"Torque to hold arm: {torque_needed:.2f} N·m")
# Output: ~4.16 N·m
```

This simple calculation shows why robot joints need **continuous torque** just to hold position against gravity!

## Control Systems: Making Robots Move Accurately

**Control systems** ensure robots follow desired trajectories despite disturbances, model errors, and uncertainties.

### Control Loop Architecture

```
Desired State → [Controller] → Motor Commands → [Robot] → Actual State
                      ↑                                      ↓
                      └────────── Feedback (Sensors) ───────┘
```

### PID Control

**PID (Proportional-Integral-Derivative)** is the most common control algorithm in robotics.

**Formula**:
```
u(t) = Kₚ·e(t) + Kᵢ·∫e(t)dt + Kₐ·de(t)/dt

Where:
u(t) = Control output (motor command)
e(t) = Error (desired - actual)
Kₚ = Proportional gain (reacts to current error)
Kᵢ = Integral gain (eliminates steady-state error)
Kₐ = Derivative gain (dampens oscillations)
```

**Components**:
- **P (Proportional)**: Larger error → larger correction (fast response, but may overshoot)
- **I (Integral)**: Accumulates error over time (eliminates bias, but can cause instability)
- **D (Derivative)**: Responds to rate of error change (reduces overshoot, adds damping)

**Code Example**:

```python
class PIDController:
    """Simple PID controller for joint position control."""

    def __init__(self, Kp, Ki, Kd, dt):
        """
        Initialize PID controller.

        Args:
            Kp, Ki, Kd: PID gains
            dt: Time step (seconds)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measured_value):
        """
        Compute control output.

        Args:
            setpoint: Desired value
            measured_value: Current value

        Returns:
            Control output
        """
        # Error
        error = setpoint - measured_value

        # Proportional term
        P = self.Kp * error

        # Integral term (accumulated error)
        self.integral += error * self.dt
        I = self.Ki * self.integral

        # Derivative term (rate of change)
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative

        # Total control output
        output = P + I + D

        # Update state
        self.prev_error = error

        return output

# Example: Control joint to 90°
controller = PIDController(Kp=10.0, Ki=0.5, Kd=2.0, dt=0.01)

current_angle = 0.0  # Starting at 0°
target_angle = 90.0

for step in range(100):
    # Compute control command
    torque = controller.compute(target_angle, current_angle)

    # Simulate robot response (simplified)
    current_angle += torque * 0.01  # Simple integration

    if step % 20 == 0:
        print(f"Step {step}: Angle = {current_angle:.2f}°, Torque = {torque:.2f}")
```

### Challenges in Humanoid Robot Control

1. **High Dimensionality**: 25+ DOF requires coordinated control
2. **Underactuation**: Fewer actuators than DOF in some configurations (floating base)
3. **Contact Dynamics**: Interaction with ground, objects (hybrid dynamics)
4. **Balance**: Must maintain center of mass over support polygon
5. **Compliance**: Too stiff → fragile, too soft → imprecise

**Modern Approaches**:
- **Model Predictive Control (MPC)**: Optimize over future horizon
- **Whole-Body Control**: Coordinate all joints simultaneously
- **Reinforcement Learning**: Learn control policies from data
- **Impedance Control**: Control force vs. position tradeoff

## Bipedal Locomotion: Walking on Two Legs

Bipedal walking is one of the hardest problems in humanoid robotics.

### The Balance Challenge

**Static Stability**: Center of Mass (CoM) stays within support polygon
- Slow, stable
- Used by early robots (ASIMO)

**Dynamic Stability**: CoM can move outside support polygon during motion
- Fast, natural
- Requires prediction and control (Boston Dynamics Atlas)

**Zero Moment Point (ZMP)**:
- Point on ground where net moment from contact forces is zero
- If ZMP stays inside support polygon → stable walking
- Widely used stability criterion

### Gait Phases

Humanoid walking involves cyclical phases:

1. **Double Support**: Both feet on ground
2. **Single Support**: One foot on ground, other swinging
3. **Heel Strike**: Swinging foot contacts ground
4. **Toe Off**: Stance foot lifts off

### Walking Controllers

**Traditional Approach**:
1. Plan footstep locations
2. Generate CoM trajectory (ZMP-based)
3. Inverse kinematics for joint trajectories
4. PID tracking control

**Modern Learning Approach**:
1. Train in simulation with RL
2. Learn robust policies that handle disturbances
3. Transfer to real robot (sim-to-real)

---

## Exercises

### 1. Degrees of Freedom
Count the DOF for:
- A simple gripper (2 fingers, 1 joint each)
- A quadruped robot leg (4 legs, 3 joints per leg)
- A humanoid torso (waist: 3 DOF, neck: 2 DOF)

### 2. Forward Kinematics Calculation
Using the 2-link planar arm formula, compute end-effector position for:
- L₁ = 1.2m, L₂ = 0.9m
- θ₁ = 60°, θ₂ = -30°

(Show your work step-by-step)

### 3. Inverse Kinematics Challenge
Is the position (2.5m, 0.5m) reachable for a 2-link arm with L₁ = 1.0m, L₂ = 0.8m? Explain why or why not.

### 4. PID Tuning
Describe what happens in a PID controller if:
- Kₚ is too high
- Kᵢ is too high
- Kₐ is too high

### 5. Code Challenge
Modify the PID controller code to:
1. Add **integral windup protection** (limit integral term to ±100)
2. Add a `reset()` method to clear integral and derivative terms
3. Test with a step response and plot angle vs. time

### 6. Walking Stability
Research the **inverted pendulum model** used in humanoid walking. Explain in 3-4 sentences how it relates to balance control.

### 7. Design Exercise
You're designing a humanoid robot arm for assembly tasks. Decide:
- How many DOF? (justify)
- Joint types and locations
- Expected payload (weight it can carry)
- Workspace (reachable volume)

---

## Key Takeaways

✅ Robots consist of **links** connected by **joints**, with DOF determining motion capability
✅ **Forward kinematics** computes end-effector position from joint angles (easy)
✅ **Inverse kinematics** computes joint angles for desired position (hard, multiple solutions)
✅ **Dynamics** models forces and torques required for motion
✅ **PID control** is fundamental for tracking desired trajectories
✅ **Bipedal walking** requires sophisticated balance and coordination strategies
✅ Modern humanoid control combines **model-based** and **learning-based** approaches

---

## Further Reading

- **Books**:
  - *Introduction to Robotics: Mechanics and Control* by John J. Craig (classic kinematics/dynamics)
  - *Modern Robotics* by Lynch and Park (geometric approach)
  - *Humanoid Robots* by Kajita et al. (bipedal walking focus)

- **Online Resources**:
  - MIT OpenCourseWare: Robotics (6.832)
  - Coursera: Robotics Specialization (UPenn)
  - ROS (Robot Operating System) tutorials

- **Papers**:
  - "Biped Walking Pattern Generation by using Preview Control of ZMP" (Kajita, 2003)
  - "Learning Agile and Dynamic Motor Skills for Legged Robots" (Hwangbo, 2019)

- **Videos**:
  - Boston Dynamics Atlas parkour demonstrations
  - Honda ASIMO walking demos

---

**Previous**: [← Chapter 1.1: AI Fundamentals](chapter-1-1.md) | **Next**: [Chapter 1.3: Sensors and Actuators →](chapter-1-3.md)

Understanding kinematics and control is the foundation—next, we'll explore the sensors and actuators that enable robots to perceive and act!
