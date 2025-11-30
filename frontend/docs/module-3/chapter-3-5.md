---
sidebar_position: 6
title: 3.5 Sim-to-Real Transfer Workflows
---

# Chapter 3.5: Sim-to-Real Transfer Workflows

The ultimate test of simulation is deployment to physical hardware. Sim-to-real transfer validates that algorithms developed in Isaac Sim and Gazebo work on real robots. This chapter covers calibration, validation, and debugging the inevitable gaps between simulation and reality.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Validate** simulation algorithms on physical humanoid hardware
- **Calibrate** sensors and actuators for real-world deployment
- **Debug** sim-to-real gaps (lighting, friction, sensor noise)
- **Deploy** Isaac Sim-trained models to Unitree G1 or similar hardware
- **Document** sim-to-real differences and mitigation strategies

## Prerequisites

- **Completed Chapters 3.1 to 3.4** (Isaac Sim, VSLAM, Nav2)
- **Access to physical robot** (Unitree G1, Boston Dynamics Spot, or similar) - OR detailed simulation validation plan
- **Understanding** of sensor calibration and system identification
- **Basic hardware** debugging skills (multimeter, oscilloscope optional)

## Part 1: Sim-to-Real Transfer Fundamentals

### The Sim-to-Real Gap

**Why simulation differs from reality**:
- **Physics**: Simplified models (friction, contact dynamics)
- **Sensors**: Perfect vs. noisy, calibrated vs. uncalibrated
- **Actuators**: Ideal vs. real motors (backlash, saturation)
- **Environment**: Controlled vs. unpredictable (lighting, obstacles)
- **Timing**: Deterministic vs. variable latency

**Common gaps**:
- **Lighting**: Simulation has perfect lighting, reality has shadows/glare
- **Friction**: Simulation friction coefficients don't match real surfaces
- **Sensor noise**: Simulation sensors are noiseless, real sensors have noise
- **Actuator dynamics**: Simulation motors are ideal, real motors have delays
- **Communication**: Simulation has no latency, real systems have network delays

### Transfer Strategies

| Strategy | Approach | Use Case |
|----------|----------|----------|
| **Domain Randomization** | Vary simulation parameters | Robust models (Chapter 3.2) |
| **Calibration** | Match sim to real parameters | Sensor/actuator tuning |
| **Progressive Transfer** | Start simple, add complexity | Incremental validation |
| **Hybrid Simulation** | Combine sim and real data | Best of both worlds |

**This chapter focuses on**: Calibration and progressive transfer.

## Part 2: Hands-On Tutorial

### Project: Validate Navigation on Physical Hardware

**Goal**: Deploy Nav2 navigation system (from Chapter 3.4) to physical humanoid and validate sim-to-real transfer.

**Tools**: Physical robot (Unitree G1 or similar), ROS 2 Humble, Nav2, calibration tools

### Step 1: Pre-Deployment Checklist

**Simulation validation**:
- [ ] Navigation works in Isaac Sim/Gazebo
- [ ] VSLAM provides accurate localization
- [ ] Path planning handles obstacles correctly
- [ ] Recovery behaviors work (spin, backup)
- [ ] All ROS 2 topics publishing correctly

**Hardware preparation**:
- [ ] Robot fully charged
- [ ] Safety checks completed (emergency stop tested)
- [ ] Network connectivity verified
- [ ] ROS 2 installed on robot computer
- [ ] Sensors calibrated (cameras, LiDAR, IMU)

**Environment preparation**:
- [ ] Test area cleared of obstacles
- [ ] Markers placed for ground truth validation
- [ ] Lighting conditions documented
- [ ] Safety personnel notified

### Step 2: Sensor Calibration

**Camera calibration** (for VSLAM):
```bash
# Install calibration tools
sudo apt install ros-humble-camera-calibration

# Calibrate stereo cameras
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.024 \
  left:=/left/image_raw right:=/right/image_raw \
  left_camera:=/left right_camera:=/right

# Save calibration file
# Copy to: ~/.ros/camera_info/left.yaml and right.yaml
```

**LiDAR calibration** (for costmaps):
```bash
# Check LiDAR data
ros2 topic echo /scan

# Verify range and angle limits match specification
# Adjust in Nav2 costmap config if needed
```

**IMU calibration** (for VSLAM fusion):
```python
#!/usr/bin/env python3
"""
IMU calibration script
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.accel_samples = []
        self.gyro_samples = []
        
    def imu_callback(self, msg):
        """Collect IMU samples for calibration"""
        # Collect samples when robot is stationary
        self.accel_samples.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.gyro_samples.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        if len(self.accel_samples) >= 1000:
            self.calculate_bias()
    
    def calculate_bias(self):
        """Calculate IMU bias (offset)"""
        accel_bias = np.mean(self.accel_samples, axis=0)
        gyro_bias = np.mean(self.gyro_samples, axis=0)
        
        # Expected: accel should be [0, 0, -9.81] when stationary (gravity)
        # Expected: gyro should be [0, 0, 0] when stationary
        
        self.get_logger().info(f"Accelerometer bias: {accel_bias}")
        self.get_logger().info(f"Gyroscope bias: {gyro_bias}")
        
        # Save to calibration file
        # Apply bias correction in IMU driver

def main(args=None):
    rclpy.init(args=args)
    node = IMUCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Deploy to Hardware

**Transfer code to robot**:
```bash
# Option 1: SSH to robot
scp -r ~/isaac_ros_ws robot@robot-ip:~/isaac_ros_ws

# Option 2: Git repository (recommended)
git push origin main
# On robot: git pull

# Option 3: ROS 2 package installation
# Build debian packages and install on robot
```

**Launch on robot**:
```bash
# SSH to robot
ssh robot@robot-ip

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Launch navigation
ros2 launch humanoid_nav2 humanoid_nav2.launch.py
```

**Verify topics**:
```bash
# On robot or monitoring computer
ros2 topic list

# Should see same topics as simulation:
# /scan
# /odom
# /map
# /plan
# /cmd_vel
```

### Step 4: Validate Navigation

**Test 1: Basic Navigation**:
```bash
# Set initial pose
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Send goal (1 meter forward)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**Observe**:
- Does robot plan path correctly?
- Does robot execute path smoothly?
- Does robot reach goal within tolerance?
- Any oscillations or stuck behavior?

**Test 2: Obstacle Avoidance**:
```bash
# Place obstacle in path
# Send goal beyond obstacle
# Verify robot avoids obstacle
```

**Test 3: Recovery Behaviors**:
```bash
# Place robot in corner (stuck situation)
# Verify recovery behaviors activate (spin, backup)
```

### Step 5: Debug Sim-to-Real Gaps

#### Gap 1: VSLAM Tracking Lost
**Symptoms**: VSLAM status shows "LOST" frequently

**Causes**:
- **Lighting**: Real lighting differs from simulation
- **Texture**: Real surfaces have less texture
- **Motion blur**: Real camera motion causes blur

**Solutions**:
```python
# Increase feature detection threshold
# In VSLAM config:
'min_features': 100,  # Increase from default

# Reduce motion blur
# Slow down robot movement
max_vel_x: 0.3  # Instead of 0.5

# Improve lighting
# Add lights or use camera with better low-light performance
```

#### Gap 2: Navigation Oscillations
**Symptoms**: Robot moves back and forth, doesn't reach goal

**Causes**:
- **Sensor noise**: Real sensors noisier than simulation
- **Actuator delays**: Real motors have response delays
- **Friction**: Real friction differs from simulation

**Solutions**:
```yaml
# Increase goal tolerance
xy_goal_tolerance: 0.5  # Larger tolerance

# Reduce controller aggressiveness
max_vel_x: 0.3  # Slower
acc_lim_x: 0.3  # Lower acceleration

# Add filtering to sensor data
# Filter LiDAR scans before costmap
```

#### Gap 3: Costmap Inaccuracies
**Symptoms**: Costmap shows obstacles where none exist (or misses obstacles)

**Causes**:
- **Sensor calibration**: Incorrect sensor parameters
- **TF errors**: Incorrect coordinate transforms
- **Noise**: Sensor noise interpreted as obstacles

**Solutions**:
```bash
# Verify TF tree
ros2 run tf2_tools view_frames
# Check: map → odom → base_link → sensor_frame

# Recalibrate sensors
# Run calibration procedures

# Adjust costmap parameters
obstacle_max_range: 3.0  # Reduce range to minimize noise
inflation_radius: 0.6    # Increase safety margin
```

#### Gap 4: Path Planning Failures
**Symptoms**: "No path found" errors on real robot

**Causes**:
- **Map differences**: Real environment differs from simulation
- **Costmap inflation**: Too conservative (blocks all paths)
- **Goal placement**: Goal in obstacle or unreachable

**Solutions**:
```yaml
# Reduce inflation radius
inflation_radius: 0.4  # Smaller safety margin

# Allow unknown space
allow_unknown: true

# Increase goal tolerance
tolerance: 1.0  # Larger tolerance
```

### Step 6: Document Sim-to-Real Differences

**Create validation report**: `docs/sim_to_real_validation.md`

```markdown
# Sim-to-Real Validation Report

## Test Environment
- **Robot**: Unitree G1
- **Date**: 2024-XX-XX
- **Location**: Test lab
- **Lighting**: Fluorescent (500 lux)

## Test Results

### Navigation Accuracy
| Metric | Simulation | Real Hardware | Gap |
|--------|------------|---------------|-----|
| Position Error (RMSE) | 0.05 m | 0.15 m | +0.10 m |
| Goal Success Rate | 100% | 85% | -15% |
| Path Smoothness | 2.3 turns/m | 3.1 turns/m | +0.8 turns/m |

### VSLAM Performance
| Metric | Simulation | Real Hardware | Gap |
|--------|------------|---------------|-----|
| Tracking Success | 100% | 75% | -25% |
| Localization Error | 0.02 m | 0.08 m | +0.06 m |
| Frame Rate | 30 FPS | 25 FPS | -5 FPS |

## Identified Gaps

### 1. Lighting Differences
**Issue**: Real lighting causes VSLAM tracking loss
**Impact**: High (frequent re-initialization)
**Mitigation**: Added lighting, improved camera settings

### 2. Sensor Noise
**Issue**: Real sensors noisier than simulation
**Impact**: Medium (costmap inaccuracies)
**Mitigation**: Added filtering, increased inflation radius

### 3. Friction Differences
**Issue**: Real friction lower than simulation
**Impact**: Low (slight path deviations)
**Mitigation**: Adjusted velocity limits

## Recommendations

1. **Improve lighting** in test environment
2. **Add sensor filtering** to reduce noise
3. **Increase goal tolerance** for robustness
4. **Test in multiple environments** (indoor, outdoor, different lighting)

## Next Steps

- [ ] Deploy to production environment
- [ ] Long-term testing (100+ navigation runs)
- [ ] Performance monitoring
- [ ] Continuous calibration updates
```

## Part 3: Advanced Topics (Optional)

### Hybrid Simulation

**Combine sim and real data**:
```python
# Use real sensor data in simulation
# Replace simulated camera with real camera feed
# Keep physics simulation, use real perception
```

**Benefits**:
- Real sensor data (no sim-to-real gap for perception)
- Safe physics testing (no hardware damage)
- Faster iteration (no need to reset real robot)

### Continuous Calibration

**Auto-calibration system**:
```python
# Monitor sensor drift over time
# Automatically recalibrate when drift detected
# Update calibration parameters dynamically
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Validation**: Capstone algorithms validated on physical hardware
- **Calibration**: Sensors and actuators calibrated for real-world
- **Documentation**: Sim-to-real gaps documented and mitigated
- **Deployment**: Ready for production deployment

Understanding sim-to-real transfer now ensures the capstone works on real hardware.

## Summary

You learned:
- ✅ Validated **simulation algorithms** on physical hardware
- ✅ Calibrated **sensors and actuators** for real-world deployment
- ✅ Debugged **sim-to-real gaps** (lighting, friction, sensor noise)
- ✅ Deployed **Isaac Sim-trained models** to physical robot
- ✅ Documented **sim-to-real differences** and mitigation strategies

**Next steps**: Module 4 (Vision-Language-Action) will integrate voice commands, LLM planning, and multi-modal interaction for the final capstone project.

---

## Exercises

### Exercise 1: Sensor Calibration (Required)

**Objective**: Calibrate sensors for real-world deployment.

**Tasks**:
1. Calibrate stereo cameras (if available)
2. Calibrate IMU (measure bias)
3. Verify LiDAR data quality
4. Document calibration parameters
5. Compare calibrated vs. uncalibrated performance

**Acceptance Criteria**:
- [ ] Camera calibration file created
- [ ] IMU bias measured and documented
- [ ] Sensor data quality verified
- [ ] Calibration report created

**Estimated Time**: 120 minutes

### Exercise 2: Navigation Validation (Required)

**Objective**: Test Nav2 navigation on physical hardware (or detailed simulation plan).

**Tasks**:
1. Deploy Nav2 to robot (or create deployment plan)
2. Test basic navigation (forward, backward, turn)
3. Test obstacle avoidance
4. Measure navigation accuracy
5. Document sim-to-real differences

**If no hardware available**:
- Create detailed deployment plan
- Document expected sim-to-real gaps
- Propose mitigation strategies
- Create validation test procedures

**Acceptance Criteria**:
- [ ] Navigation tested (or plan created)
- [ ] Accuracy metrics documented
- [ ] Sim-to-real gaps identified
- [ ] Mitigation strategies proposed

**Estimated Time**: 180 minutes

### Exercise 3: Sim-to-Real Gap Analysis (Challenge)

**Objective**: Comprehensive analysis of simulation vs. reality differences.

**Tasks**:
1. Run identical tests in simulation and real hardware
2. Measure quantitative differences (position error, success rate)
3. Identify qualitative differences (behavior, appearance)
4. Propose improvements to simulation models
5. Create comprehensive validation report

**Metrics**:
- Position accuracy (RMSE)
- Success rate (%)
- Path smoothness
- Computation time
- Resource usage

**Estimated Time**: 240 minutes

---

## Additional Resources

- [ROS 2 Robot Calibration](http://wiki.ros.org/calibration) - Calibration tools
- [Sim-to-Real Transfer](https://arxiv.org/abs/1703.06907) - Research paper
- [Unitree G1 Documentation](https://www.unitree.com/) - Hardware specifications
- [Boston Dynamics Spot SDK](https://dev.bostondynamics.com/) - Example deployment

---

**Next**: [Module 4: Vision-Language-Action (VLA) →](../module-4/intro.md)
