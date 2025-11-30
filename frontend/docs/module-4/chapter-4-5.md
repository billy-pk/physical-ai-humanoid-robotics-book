---
sidebar_position: 6
title: 4.5 Humanoid Kinematics & Balance Control
---

# Chapter 4.5: Humanoid Kinematics & Balance Control

Humanoid robots require sophisticated control algorithms to maintain balance while walking, manipulating objects, and navigating. This chapter covers forward/inverse kinematics, balance control (ZMP, LIPM), walking gaits, and integration with ROS 2 control stack.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Understand** forward and inverse kinematics for humanoid robots
- **Implement** balance control algorithms (ZMP, Linear Inverted Pendulum Model)
- **Configure** walking gaits and manipulation poses
- **Integrate** kinematics and balance with ROS 2 control stack
- **Debug** balance and walking issues

## Prerequisites

- **ROS 2 Humble** configured (Module 1)
- **URDF modeling** experience (Module 1, Chapter 1.3)
- **Basic robotics** concepts: coordinate frames, transformations, Jacobians
- **Mathematics**: Linear algebra, trigonometry, calculus basics
- **Python 3.10+** with numpy, scipy

## Part 1: Humanoid Kinematics Fundamentals

### Forward vs. Inverse Kinematics

**Forward Kinematics (FK)**:
- **Input**: Joint angles (θ₁, θ₂, ..., θₙ)
- **Output**: End-effector pose (position + orientation)
- **Question**: "Where is the hand given joint angles?"

**Inverse Kinematics (IK)**:
- **Input**: Desired end-effector pose
- **Output**: Joint angles to achieve pose
- **Question**: "What joint angles put the hand here?"

**For humanoids**:
- **Walking**: IK for foot placement, FK for center of mass
- **Manipulation**: IK for hand positioning, FK for reachability
- **Balance**: FK for center of mass calculation

### Humanoid Structure

**Typical humanoid**:
- **Torso**: Base link (6 DOF: x, y, z, roll, pitch, yaw)
- **Legs**: 6 DOF each (hip 3 + knee 1 + ankle 2)
- **Arms**: 7 DOF each (shoulder 3 + elbow 1 + wrist 3)
- **Head**: 2 DOF (pan + tilt)

**Total**: ~30+ degrees of freedom

### Balance Control

**Why balance matters**:
- **Stability**: Prevent falling
- **Walking**: Maintain balance while moving
- **Manipulation**: Don't tip over when reaching

**Balance methods**:
- **ZMP (Zero Moment Point)**: Point where net moment is zero
- **LIPM (Linear Inverted Pendulum Model)**: Simplified dynamics model
- **MPC (Model Predictive Control)**: Optimal control with preview

## Part 2: Hands-On Tutorial

### Project: Balance Control for Humanoid Robot

**Goal**: Implement basic balance control and walking gait for humanoid robot.

**Tools**: ROS 2 Humble, Python 3.10+, numpy, scipy, PyKDL (kinematics library)

### Step 1: Install Kinematics Libraries

**Install PyKDL** (ROS kinematics library):
```bash
sudo apt install ros-humble-kdl-parser-py ros-humble-orocos-kdl
pip3 install PyKDL
```

**Install scipy** (for optimization):
```bash
pip3 install scipy
```

### Step 2: Forward Kinematics Implementation

**Create FK node**: `voice_commands/kinematics_node.py`

```python
#!/usr/bin/env python3
"""
Forward and inverse kinematics for humanoid robot
ROS 2 Humble | Python 3.10+ | PyKDL
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation

class KinematicsNode(Node):
    """
    Calculates forward and inverse kinematics for humanoid
    """
    def __init__(self):
        super().__init__('kinematics_node')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publisher for end-effector poses
        self.left_hand_pub = self.create_publisher(PoseStamped, '/kinematics/left_hand_pose', 10)
        self.right_hand_pub = self.create_publisher(PoseStamped, '/kinematics/right_hand_pose', 10)
        self.left_foot_pub = self.create_publisher(PoseStamped, '/kinematics/left_foot_pose', 10)
        self.right_foot_pub = self.create_publisher(PoseStamped, '/kinematics/right_foot_pose', 10)
        self.com_pub = self.create_publisher(Point, '/kinematics/center_of_mass', 10)
        
        # Robot parameters (from URDF)
        self.link_lengths = {
            'torso_height': 0.4,
            'thigh_length': 0.3,
            'shank_length': 0.3,
            'foot_length': 0.15,
            'upper_arm_length': 0.25,
            'forearm_length': 0.25,
        }
        
        self.get_logger().info('Kinematics node started')
    
    def joint_callback(self, msg):
        """Calculate FK from joint states"""
        # Extract joint angles
        joint_angles = {}
        for i, name in enumerate(msg.name):
            joint_angles[name] = msg.position[i]
        
        # Calculate end-effector poses
        left_foot_pose = self.calculate_left_foot_pose(joint_angles)
        right_foot_pose = self.calculate_right_foot_pose(joint_angles)
        left_hand_pose = self.calculate_left_hand_pose(joint_angles)
        right_hand_pose = self.calculate_right_hand_pose(joint_angles)
        com_position = self.calculate_center_of_mass(joint_angles)
        
        # Publish poses
        self.publish_pose(self.left_foot_pub, left_foot_pose, 'base_link')
        self.publish_pose(self.right_foot_pub, right_foot_pose, 'base_link')
        self.publish_pose(self.left_hand_pub, left_hand_pose, 'base_link')
        self.publish_pose(self.right_hand_pub, right_hand_pose, 'base_link')
        
        # Publish center of mass
        com_msg = Point()
        com_msg.x = com_position[0]
        com_msg.y = com_position[1]
        com_msg.z = com_position[2]
        self.com_pub.publish(com_msg)
    
    def calculate_left_foot_pose(self, joint_angles):
        """Calculate left foot pose using FK"""
        # Simplified FK for leg (would use proper DH parameters in production)
        hip_yaw = joint_angles.get('left_hip_yaw', 0.0)
        hip_pitch = joint_angles.get('left_hip_pitch', 0.0)
        hip_roll = joint_angles.get('left_hip_roll', 0.0)
        knee = joint_angles.get('left_knee', 0.0)
        ankle_pitch = joint_angles.get('left_ankle_pitch', 0.0)
        ankle_roll = joint_angles.get('left_ankle_roll', 0.0)
        
        # Forward kinematics (simplified)
        # Position relative to hip
        thigh_len = self.link_lengths['thigh_length']
        shank_len = self.link_lengths['shank_length']
        
        # Calculate foot position (simplified 2D in sagittal plane)
        x = thigh_len * np.sin(hip_pitch) + shank_len * np.sin(hip_pitch + knee)
        z = -thigh_len * np.cos(hip_pitch) - shank_len * np.cos(hip_pitch + knee)
        y = 0.1  # Left leg offset
        
        # Apply hip yaw and roll
        # ... (full 3D transformation)
        
        return np.array([x, y, z])
    
    def calculate_right_foot_pose(self, joint_angles):
        """Calculate right foot pose (similar to left)"""
        # Similar to left foot, with y = -0.1
        return np.array([0.0, -0.1, -0.6])  # Placeholder
    
    def calculate_left_hand_pose(self, joint_angles):
        """Calculate left hand pose"""
        # Similar FK for arm
        return np.array([0.0, 0.2, 0.8])  # Placeholder
    
    def calculate_right_hand_pose(self, joint_angles):
        """Calculate right hand pose"""
        return np.array([0.0, -0.2, 0.8])  # Placeholder
    
    def calculate_center_of_mass(self, joint_angles):
        """Calculate center of mass position"""
        # Weighted average of link centers
        # Simplified: Assume CoM at torso center
        return np.array([0.0, 0.0, 0.5])  # Placeholder
    
    def publish_pose(self, publisher, position, frame_id):
        """Publish pose message"""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0  # Default orientation
        publisher.publish(pose)
```

### Step 3: Inverse Kinematics Implementation

**Create IK solver**: `voice_commands/ik_solver.py`

```python
#!/usr/bin/env python3
"""
Inverse kinematics solver for humanoid
ROS 2 Humble | Python 3.10+ | scipy
"""
import numpy as np
from scipy.optimize import minimize

class IKSolver:
    """
    Solves inverse kinematics for humanoid limbs
    """
    def __init__(self, link_lengths):
        self.link_lengths = link_lengths
    
    def solve_leg_ik(self, target_foot_pose, initial_angles=None):
        """
        Solve IK for leg to achieve target foot pose
        Returns joint angles: [hip_yaw, hip_pitch, hip_roll, knee, ankle_pitch, ankle_roll]
        """
        if initial_angles is None:
            initial_angles = np.zeros(6)
        
        def objective(joint_angles):
            """Minimize distance between current and target pose"""
            current_pose = self.forward_kinematics_leg(joint_angles)
            error = np.linalg.norm(current_pose - target_foot_pose)
            return error
        
        # Joint limits
        bounds = [
            (-1.57, 1.57),   # hip_yaw
            (-1.57, 1.57),   # hip_pitch
            (-0.5, 0.5),     # hip_roll
            (0, 3.14),       # knee
            (-0.5, 0.5),     # ankle_pitch
            (-0.5, 0.5),     # ankle_roll
        ]
        
        result = minimize(objective, initial_angles, bounds=bounds, method='L-BFGS-B')
        
        if result.success:
            return result.x
        else:
            return None
    
    def forward_kinematics_leg(self, joint_angles):
        """Calculate foot pose from joint angles (simplified)"""
        hip_pitch = joint_angles[1]
        knee = joint_angles[3]
        
        thigh_len = self.link_lengths['thigh_length']
        shank_len = self.link_lengths['shank_length']
        
        x = thigh_len * np.sin(hip_pitch) + shank_len * np.sin(hip_pitch + knee)
        z = -thigh_len * np.cos(hip_pitch) - shank_len * np.cos(hip_pitch + knee)
        
        return np.array([x, 0, z])
```

### Step 4: Balance Control (ZMP)

**Create balance controller**: `voice_commands/balance_controller.py`

```python
#!/usr/bin/env python3
"""
Balance control using ZMP (Zero Moment Point)
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import numpy as np

class BalanceController(Node):
    """
    Maintains humanoid balance using ZMP control
    """
    def __init__(self):
        super().__init__('balance_controller')
        
        # Subscribe to center of mass
        self.com_sub = self.create_subscription(
            Point,
            '/kinematics/center_of_mass',
            self.com_callback,
            10
        )
        
        # Subscribe to foot contact forces (would come from sensors)
        # For now: Assume both feet on ground
        
        # Publisher for balance corrections
        self.balance_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/balance/commands',
            10
        )
        
        # ZMP parameters
        self.support_polygon = np.array([
            [-0.1, -0.05],  # Left foot corners
            [0.1, -0.05],
            [0.1, 0.05],
            [-0.1, 0.05]
        ])
        
        self.desired_zmp = np.array([0.0, 0.0])  # Center of support polygon
        
        self.get_logger().info('Balance controller started')
    
    def com_callback(self, msg):
        """Calculate ZMP and generate balance corrections"""
        com_position = np.array([msg.x, msg.y, msg.z])
        
        # Calculate current ZMP (simplified: assume CoM projects to ground)
        current_zmp = np.array([com_position[0], com_position[1]])
        
        # Calculate ZMP error
        zmp_error = current_zmp - self.desired_zmp
        
        # Generate balance correction (adjust joint angles to shift CoM)
        correction = self.calculate_balance_correction(zmp_error)
        
        # Publish correction commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = correction.tolist()
        self.balance_cmd_pub.publish(cmd_msg)
    
    def calculate_balance_correction(self, zmp_error):
        """Calculate joint angle corrections to maintain balance"""
        # Simplified: Adjust ankle angles to shift CoM
        # In production: Use full-body IK or MPC
        
        max_correction = 0.1  # Radians
        
        # Proportional control
        kp = 0.5
        correction_x = -kp * zmp_error[0]  # Negative for stability
        correction_y = -kp * zmp_error[1]
        
        # Limit corrections
        correction_x = np.clip(correction_x, -max_correction, max_correction)
        correction_y = np.clip(correction_y, -max_correction, max_correction)
        
        # Return corrections for ankle joints
        return np.array([
            correction_x,  # Left ankle pitch
            correction_y,  # Left ankle roll
            correction_x,  # Right ankle pitch
            correction_y,  # Right ankle roll
        ])
    
    def is_zmp_in_support_polygon(self, zmp):
        """Check if ZMP is within support polygon"""
        # Simplified point-in-polygon check
        # In production: Use proper polygon containment algorithm
        return True  # Placeholder
```

### Step 5: Walking Gait Generator

**Create gait generator**: `voice_commands/gait_generator.py`

```python
#!/usr/bin/env python3
"""
Walking gait generator for humanoid
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class GaitGenerator(Node):
    """
    Generates walking gait patterns for humanoid
    """
    def __init__(self):
        super().__init__('gait_generator')
        
        # Gait parameters
        self.step_length = 0.2  # meters
        self.step_height = 0.05  # meters
        self.step_duration = 1.0  # seconds
        self.stance_phase_ratio = 0.6  # 60% stance, 40% swing
        
        # State
        self.phase = 0.0  # 0 to 1 (gait cycle)
        self.left_foot_swing = False
        self.right_foot_swing = False
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/gait/joint_commands',
            10
        )
        
        # Timer for gait cycle
        self.gait_timer = self.create_timer(0.01, self.update_gait)  # 100 Hz
        
        self.get_logger().info('Gait generator started')
    
    def update_gait(self):
        """Update gait cycle and generate joint commands"""
        # Update phase
        dt = 0.01  # Timer period
        phase_increment = dt / self.step_duration
        self.phase += phase_increment
        
        if self.phase >= 1.0:
            self.phase = 0.0
            # Switch swing leg
            self.left_foot_swing = not self.left_foot_swing
            self.right_foot_swing = not self.right_foot_swing
        
        # Generate joint angles for current phase
        joint_angles = self.generate_gait_angles()
        
        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_angles.tolist()
        self.joint_cmd_pub.publish(cmd_msg)
    
    def generate_gait_angles(self):
        """Generate joint angles for current gait phase"""
        # Simplified gait: sinusoidal trajectories
        # In production: Use optimized trajectories
        
        # Base angles (standing pose)
        left_hip_pitch = 0.0
        left_knee = 0.0
        left_ankle_pitch = 0.0
        
        right_hip_pitch = 0.0
        right_knee = 0.0
        right_ankle_pitch = 0.0
        
        # Add gait motion
        if self.left_foot_swing:
            # Left foot swinging
            swing_phase = self.phase / self.stance_phase_ratio if self.phase < self.stance_phase_ratio else (self.phase - self.stance_phase_ratio) / (1 - self.stance_phase_ratio)
            
            left_hip_pitch = np.sin(swing_phase * np.pi) * 0.3  # Lift leg
            left_knee = np.sin(swing_phase * np.pi) * 0.5  # Bend knee
            left_ankle_pitch = -np.sin(swing_phase * np.pi) * 0.2  # Point toe
        
        if self.right_foot_swing:
            # Right foot swinging (similar)
            swing_phase = self.phase / self.stance_phase_ratio if self.phase < self.stance_phase_ratio else (self.phase - self.stance_phase_ratio) / (1 - self.stance_phase_ratio)
            
            right_hip_pitch = np.sin(swing_phase * np.pi) * 0.3
            right_knee = np.sin(swing_phase * np.pi) * 0.5
            right_ankle_pitch = -np.sin(swing_phase * np.pi) * 0.2
        
        # Return all joint angles
        return np.array([
            left_hip_pitch, left_knee, left_ankle_pitch,
            right_hip_pitch, right_knee, right_ankle_pitch,
            # ... (add other joints)
        ])
    
    def start_walking(self, direction='forward', speed=0.5):
        """Start walking in specified direction"""
        self.walking = True
        self.direction = direction
        self.speed = speed
        self.get_logger().info(f'Starting to walk {direction} at speed {speed}')
    
    def stop_walking(self):
        """Stop walking and return to standing pose"""
        self.walking = False
        self.get_logger().info('Stopping walk')
```

### Step 6: Integration with ROS 2 Control

**Create control interface**: `voice_commands/control_interface.py`

```python
#!/usr/bin/env python3
"""
Control interface for humanoid robot
Integrates kinematics, balance, and gait
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

class ControlInterface(Node):
    """
    Unified control interface for humanoid
    """
    def __init__(self):
        super().__init__('control_interface')
        
        # Subscribe to action goals
        self.action_goal_sub = self.create_subscription(
            PoseStamped,
            '/control/goal_pose',
            self.goal_callback,
            10
        )
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )
        
        # Internal components (would instantiate actual classes)
        # self.ik_solver = IKSolver(...)
        # self.balance_controller = BalanceController(...)
        # self.gait_generator = GaitGenerator(...)
        
        self.get_logger().info('Control interface started')
    
    def goal_callback(self, msg):
        """Handle control goal (e.g., move hand to pose)"""
        target_pose = msg.pose
        
        # Solve IK to achieve pose
        # joint_angles = self.ik_solver.solve_arm_ik(target_pose)
        
        # Apply balance corrections
        # balance_correction = self.balance_controller.get_correction()
        
        # Combine and publish joint commands
        # final_angles = joint_angles + balance_correction
        # self.publish_joint_commands(final_angles)
        
        self.get_logger().info('Control goal received')
    
    def publish_joint_commands(self, joint_angles):
        """Publish joint angle commands"""
        msg = Float64MultiArray()
        msg.data = joint_angles.tolist()
        self.joint_cmd_pub.publish(msg)
```

### Step 7: Debugging Common Issues

#### Issue 1: "IK solution not found"
**Symptoms**: IK solver fails to find solution

**Solutions**:
```python
# Check if target is within workspace
# Increase joint limits if needed
# Use better initial guess
# Try multiple IK solvers (analytical vs. numerical)
```

#### Issue 2: "Robot falls over" or "Unstable balance"
**Symptoms**: Robot tips over during motion

**Solutions**:
```python
# Increase support polygon (wider stance)
# Improve ZMP control gains
# Add damping to balance controller
# Slow down motion (reduce speed)
```

#### Issue 3: "Walking gait jerky" or "Unnatural motion"
**Symptoms**: Robot movement is not smooth

**Solutions**:
```python
# Smooth joint trajectories (use splines)
# Optimize gait parameters (step length, duration)
# Add acceleration limits
# Use motion planning (trajectory optimization)
```

## Part 3: Advanced Topics (Optional)

### Model Predictive Control (MPC)

**Advanced balance control**:
```python
# Use MPC for optimal balance control
# Preview future ZMP trajectory
# Optimize joint trajectories
# Handle constraints (joint limits, torque limits)
```

### Whole-Body Control

**Coordinate all joints**:
```python
# Solve IK for entire body simultaneously
# Consider balance constraints
# Optimize for energy efficiency
# Handle redundancy (multiple solutions)
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Manipulation**: IK enables precise hand positioning for grasping
- **Navigation**: Walking gaits enable locomotion
- **Balance**: Balance control prevents falls during actions
- **Integration**: Connects high-level actions to low-level control

Understanding kinematics and balance now is essential for the capstone robot control.

## Summary

You learned:
- ✅ Understood **forward and inverse kinematics** for humanoid robots
- ✅ Implemented **balance control algorithms** (ZMP) for stability
- ✅ Configured **walking gaits** for locomotion
- ✅ Integrated **kinematics and balance** with ROS 2 control
- ✅ Debugged **balance and walking issues**

**Next steps**: Complete the capstone project integrating all modules into a complete autonomous humanoid system.

---

## Exercises

### Exercise 1: Forward Kinematics (Required)

**Objective**: Calculate end-effector poses from joint angles.

**Tasks**:
1. Implement FK for one leg
2. Test with known joint angles
3. Verify foot pose matches expected position
4. Extend to arm FK
5. Visualize poses in RViz2

**Acceptance Criteria**:
- [ ] FK implemented for leg
- [ ] Calculated poses match expected values
- [ ] Poses visualized in RViz2
- [ ] FK extended to arm

**Estimated Time**: 120 minutes

### Exercise 2: Inverse Kinematics (Required)

**Objective**: Solve IK to achieve target end-effector poses.

**Tasks**:
1. Implement IK solver for leg
2. Test with target foot poses
3. Verify solution achieves target
4. Handle joint limits
5. Test with unreachable targets (should fail gracefully)

**Acceptance Criteria**:
- [ ] IK solver implemented
- [ ] Solutions achieve target poses
- [ ] Joint limits respected
- [ ] Unreachable targets handled

**Estimated Time**: 180 minutes

### Exercise 3: Balance Control (Challenge)

**Objective**: Implement ZMP-based balance control.

**Tasks**:
1. Calculate ZMP from center of mass
2. Implement ZMP controller
3. Test balance corrections
4. Integrate with walking gait
5. Validate balance during walking

**Requirements**:
- ZMP calculation
- Balance controller
- Integration with gait
- Balance validation

**Estimated Time**: 240 minutes

---

## Additional Resources

- [Humanoid Robotics](https://www.springer.com/series/15556) - Textbook on humanoid control
- [ZMP Control](https://ieeexplore.ieee.org/document/1232809) - Research paper
- [PyKDL Documentation](https://www.orocos.org/kdl.html) - Kinematics library
- [ROS 2 Control](https://control.ros.org/) - ROS 2 control framework

---

**Next**: [Capstone Project: Autonomous Humanoid System →](capstone.md)
