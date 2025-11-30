---
sidebar_position: 6
title: 1.5 Parameter Management & Best Practices
---

# Chapter 1.5: Parameter Management & Best Practices

Production robotics systems require runtime configurability, robust communication policies, and predictable lifecycle management. This chapter covers parameters, Quality of Service (QoS) profiles, lifecycle nodes, and best practices for building reliable ROS 2 systems.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Configure** nodes with YAML parameter files for runtime customization
- **Implement** parameter callbacks to respond to configuration changes
- **Select** appropriate QoS profiles for different communication patterns
- **Understand** lifecycle nodes for safety-critical humanoid systems
- **Apply** ROS 2 best practices for maintainable, scalable robot software

## Prerequisites

- **Completed Chapters 1.1 to 1.4**
- **Understanding of callbacks** (from Chapter 1.2)
- **Experience with YAML** syntax (basic key-value pairs)

## Part 1: ROS 2 Parameters

### What are Parameters?

**Parameters** are configuration values that nodes can read and modify at runtime without recompiling code.

**Example use cases**:
- **Camera node**: Resolution (640×480 vs. 1920×1080), frame rate (30 Hz vs. 60 Hz)
- **Motor controller**: PID gains (Kp, Ki, Kd)
- **Object detector**: Confidence threshold (0.5 vs. 0.8)

**Why parameters over hardcoded values?**
- **Reusability**: Same node works for different robots by changing config
- **Tuning**: Adjust PID gains without recompiling
- **Multi-robot**: Run same package with different configs on different robots

### Parameter Types

ROS 2 supports these parameter types:
- \`bool\`: \`True\` / \`False\`
- \`int\`: \`42\`, \`-10\`
- \`double\`: \`3.14159\`, \`0.5\`
- \`string\`: \`"left_camera"\`, \`"/dev/ttyUSB0"\`
- \`byte_array\`, \`bool_array\`, \`int_array\`, \`double_array\`, \`string_array\`

## Part 2: Hands-On Tutorial

### Project 1: Node with Parameters

**Goal**: Create a motor controller node with configurable PID gains.

**File**: \`motor_controller.py\`

```python
#!/usr/bin/env python3
"""
Motor Controller with Parameters
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorController(Node):
    """
    Motor controller with PID parameters.
    Parameters:
    - kp (double): Proportional gain
    - ki (double): Integral gain
    - kd (double): Derivative gain
    - target_position (double): Desired motor position
    """
    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters with default values
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('target_position', 0.0)
        self.declare_parameter('control_rate', 50.0)  # Hz
        
        # Read parameter values
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.target = self.get_parameter('target_position').get_parameter_value().double_value
        rate = self.get_parameter('control_rate').get_parameter_value().double_value
        
        # Publisher
        self.pub = self.create_publisher(Float64, '/motor/command', 10)
        
        # Timer based on control_rate parameter
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        
        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.current_position = 0.0  # Simulated
        
        self.get_logger().info(
            f'Motor Controller started: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, '
            f'Target={self.target}, Rate={rate} Hz'
        )
    
    def control_loop(self):
        """PID control loop."""
        # Calculate error
        error = self.target - self.current_position
        
        # PID terms
        P = self.kp * error
        self.integral += error * (1.0 / 50.0)  # dt
        I = self.ki * self.integral
        derivative = (error - self.prev_error) * 50.0
        D = self.kd * derivative
        
        # Control output
        control = P + I + D
        
        # Simulate motor response
        self.current_position += control * 0.01
        
        # Publish command
        msg = Float64()
        msg.data = control
        self.pub.publish(msg)
        
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation**:
- **Lines 22 to 26**: Declare parameters with defaults
- **Lines 28 to 32**: Read parameter values
- **Line 38**: Timer rate calculated from parameter

**Run with default parameters**:
```bash
ros2 run my_package motor_controller
```

**Run with custom parameters** (command line):
```bash
ros2 run my_package motor_controller --ros-args \
  -p kp:=2.5 \
  -p ki:=0.5 \
  -p kd:=0.1 \
  -p target_position:=1.57 \
  -p control_rate:=100.0
```

**Get parameter value** (while node is running):
```bash
ros2 param get /motor_controller kp
# Output: Double value is: 1.0
```

**Set parameter at runtime**:
```bash
ros2 param set /motor_controller kp 3.0
# Output: Set parameter successful
```

**List all parameters**:
```bash
ros2 param list /motor_controller
# Output:
# kd
# ki
# kp
# target_position
# control_rate
```

---

### Project 2: Parameter File (YAML)

**Goal**: Load parameters from a YAML file.

**File**: \`config/motor_params.yaml\`

```yaml
motor_controller:
  ros__parameters:
    kp: 2.5
    ki: 0.3
    kd: 0.08
    target_position: 1.57  # 90 degrees
    control_rate: 100.0
    
    # Additional parameters
    max_integral: 10.0
    output_limit: 5.0
```

**Structure**:
- Top level: Node name (\`motor_controller\`)
- \`ros__parameters\`: Required key
- Parameters as key-value pairs

**Run with parameter file**:
```bash
ros2 run my_package motor_controller --ros-args \
  --params-file config/motor_params.yaml
```

**Launch file with parameters**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_package')
    param_file = os.path.join(pkg_dir, 'config', 'motor_params.yaml')
    
    return LaunchDescription([
        Node(
            package='my_package',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[param_file]  # Load YAML file
        ),
    ])
```

---

### Project 3: Parameter Callbacks

**Goal**: Respond to parameter changes at runtime.

```python
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters with descriptors
        kp_descriptor = ParameterDescriptor(
            description='Proportional gain for PID controller',
            read_only=False,  # Can be changed at runtime
            dynamic_typing=False
        )
        self.declare_parameter('kp', 1.0, kp_descriptor)
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Motor controller with parameter callback ready')
    
    def parameter_callback(self, params):
        """
        Called when parameters are modified at runtime.
        
        Args:
            params: List of parameters that changed
        
        Returns:
            SetParametersResult: Success or failure
        """
        result = SetParametersResult(successful=True)
        
        for param in params:
            if param.name == 'kp':
                new_kp = param.value
                
                # Validate parameter
                if new_kp < 0.0:
                    self.get_logger().error(f'Invalid Kp: {new_kp}. Must be >= 0')
                    result.successful = False
                    result.reason = 'Kp must be non-negative'
                else:
                    self.kp = new_kp
                    self.get_logger().info(f'Updated Kp to {self.kp}')
            
            elif param.name == 'ki':
                new_ki = param.value
                if new_ki < 0.0:
                    result.successful = False
                    result.reason = 'Ki must be non-negative'
                else:
                    self.ki = new_ki
                    self.get_logger().info(f'Updated Ki to {self.ki}')
        
        return result
```

**Test callback**:
```bash
# Terminal 1: Run node
ros2 run my_package motor_controller

# Terminal 2: Change parameter
ros2 param set /motor_controller kp 5.0
# Node logs: [INFO] Updated Kp to 5.0

# Try invalid value
ros2 param set /motor_controller kp -1.0
# Node logs: [ERROR] Invalid Kp: -1.0. Must be >= 0
# Terminal output: Setting parameter failed: Kp must be non-negative
```

---

## Part 3: Quality of Service (QoS)

### What is QoS?

**Quality of Service** profiles define communication reliability, durability, and history settings.

**Common QoS profiles**:

| Profile | Reliability | Durability | History | Use Case |
|---------|-------------|------------|---------|----------|
| **Sensor Data** | Best Effort | Volatile | Keep Last 1 | Camera images (OK to drop frames) |
| **Default** | Reliable | Volatile | Keep Last 10 | General commands |
| **Services** | Reliable | Volatile | Keep Last 10 | Request/reply |
| **System Default** | Reliable | Transient Local | Keep Last 10 | Configuration topics |

### QoS Settings

**Reliability**:
- **Reliable**: Guarantees delivery (retransmits lost packets)
- **Best Effort**: Allows dropped messages (lower latency)

**Durability**:
- **Volatile**: Late joiners miss old messages
- **Transient Local**: Late joiners receive last N messages

**History**:
- **Keep Last N**: Store last N messages
- **Keep All**: Store all messages (until memory limit)

### Using QoS in Code

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Custom QoS for camera images (best effort, low latency)
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # Keep only latest frame
)

# Create publisher with custom QoS
self.publisher_ = self.create_publisher(
    Image,
    '/camera/image_raw',
    qos_profile
)

# Create subscriber with matching QoS
self.subscription = self.create_subscription(
    Image,
    '/camera/image_raw',
    self.image_callback,
    qos_profile
)
```

**Predefined QoS profiles**:
```python
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default

# Use sensor data profile (best effort)
self.publisher_ = self.create_publisher(Image, '/camera', qos_profile_sensor_data)
```

**QoS Mismatch**:
If publisher and subscriber have incompatible QoS, communication fails silently.

**Debug QoS issues**:
```bash
# Check QoS settings of a topic
ros2 topic info -v /camera/image_raw
```

---

## Part 4: Lifecycle Nodes (Optional Advanced)

### What are Lifecycle Nodes?

**Lifecycle nodes** provide deterministic state management for safety-critical systems.

**States**:
1. **Unconfigured**: Node created but not ready
2. **Inactive**: Configured but not processing data
3. **Active**: Fully operational
4. **Finalized**: Shutting down

**Use case**: Humanoid robot startup sequence
1. Initialize hardware (Unconfigured → Inactive)
2. Run self-tests and calibration (Inactive)
3. Start control loops (Inactive → Active)
4. Emergency stop (Active → Inactive)
5. Shutdown (Inactive → Finalized)

**Example**:
```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class SafetyController(LifecycleNode):
    def __init__(self):
        super().__init__('safety_controller')
    
    def on_configure(self, state):
        """Called when transitioning to Inactive."""
        self.get_logger().info('Configuring hardware...')
        # Initialize sensors, motors
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        """Called when transitioning to Active."""
        self.get_logger().info('Starting control loops...')
        # Start timers, enable motors
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        """Called when transitioning to Inactive (emergency stop)."""
        self.get_logger().warn('Emergency stop activated!')
        # Stop motors, disable control
        return TransitionCallbackReturn.SUCCESS
```

**Control lifecycle**:
```bash
# Configure node
ros2 lifecycle set /safety_controller configure

# Activate node
ros2 lifecycle set /safety_controller activate

# Emergency stop (deactivate)
ros2 lifecycle set /safety_controller deactivate
```

---

## Part 5: ROS 2 Best Practices

### 1. Node Design

**Single Responsibility Principle**:
- ❌ Bad: One node handles camera, object detection, AND navigation
- ✅ Good: Separate nodes for camera driver, detector, planner

**Keep nodes small and focused** (~200 to 500 lines per node).

### 2. Naming Conventions

**Nodes**: \`snake_case\`
```
/camera_driver
/object_detector
/motion_planner
```

**Topics**: \`/namespace/description\`
```
/camera/image_raw
/robot/joint_states
/nav/goal
```

**Parameters**: \`snake_case\`
```
max_velocity
pid_gains
camera_resolution
```

### 3. Error Handling

**Always validate inputs**:
```python
def joint_command_callback(self, msg):
    if len(msg.position) != self.num_joints:
        self.get_logger().error(
            f'Expected {self.num_joints} joints, got {len(msg.position)}'
        )
        return
    
    # Process command
```

**Use try-except for external calls**:
```python
try:
    response = self.client.call_async(request)
except Exception as e:
    self.get_logger().error(f'Service call failed: {e}')
```

### 4. Logging Levels

Use appropriate log levels:
```python
self.get_logger().debug('Detailed diagnostic: joint angle = 1.234')  # Verbose
self.get_logger().info('Node started successfully')                   # Normal
self.get_logger().warn('Battery low: 15% remaining')                  # Warning
self.get_logger().error('Sensor read failed: timeout')                # Error
self.get_logger().fatal('Critical hardware failure, shutting down')   # Fatal
```

**Set log level**:
```bash
ros2 run my_pkg my_node --ros-args --log-level DEBUG
```

### 5. Performance Tips

**Use timers, not \`while True\` loops**:
```python
# ❌ Bad: Blocks executor
def __init__(self):
    while True:
        self.publish_data()
        time.sleep(0.1)

# ✅ Good: Uses ROS 2 timer
def __init__(self):
    self.create_timer(0.1, self.publish_data)
```

**Avoid blocking operations in callbacks**:
```python
# ❌ Bad: Blocks other callbacks
def callback(self, msg):
    result = self.expensive_computation(msg)  # Takes 5 seconds

# ✅ Good: Offload to thread
def callback(self, msg):
    threading.Thread(target=self.expensive_computation, args=(msg,)).start()
```

### 6. Testing

**Write unit tests** for nodes:
```python
import unittest
from my_package.motor_controller import MotorController

class TestMotorController(unittest.TestCase):
    def test_pid_calculation(self):
        node = MotorController()
        # Test PID logic
        self.assertAlmostEqual(node.kp, 1.0)
```

**Run tests**:
```bash
colcon test --packages-select my_package
colcon test-result --verbose
```

---

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Parameters**: Configure LLM model, Nav2 settings, object detection threshold via YAML
  ```yaml
  llm_planner:
    ros__parameters:
      model: "gpt-4"
      temperature: 0.7
      max_tokens: 500
  ```

- **QoS**: Use sensor_data QoS for camera (30 Hz), reliable QoS for navigation goals
- **Lifecycle**: Safety controller ensures humanoid shuts down gracefully on emergency stop
- **Best practices**: Modular nodes (voice, planning, navigation, vision) communicate via well-defined interfaces

## Summary

You learned:
- ✅ Configured nodes with **YAML parameter files**
- ✅ Implemented **parameter callbacks** for runtime changes
- ✅ Selected **QoS profiles** for different communication patterns
- ✅ Understood **lifecycle nodes** for safety-critical systems
- ✅ Applied **ROS 2 best practices** for maintainable code

**Next steps**: Module 1 complete! In Module 2, you'll simulate humanoid robots in Gazebo with physics and sensors.

---

## Exercises

### Exercise 1: Configurable Publisher (Required)

**Objective**: Create a publisher with configurable topic name and publish rate.

**Tasks**:
1. Create node with parameters:
   - \`topic_name\` (string, default: \`/data\`)
   - \`publish_rate\` (double, default: 10.0 Hz)
2. Publish incrementing counter to configured topic
3. Create YAML file with custom settings
4. Test with parameter file

**Acceptance Criteria**:
- [ ] Parameters loaded from YAML
- [ ] \`ros2 param set\` changes publish rate
- [ ] Node publishes to custom topic name

**Estimated Time**: 45 minutes

### Exercise 2: QoS Experiments (Required)

**Objective**: Understand QoS compatibility.

**Tasks**:
1. Create publisher with \`BEST_EFFORT\` reliability
2. Create subscriber with \`RELIABLE\` reliability
3. Run both—observe no communication
4. Fix by matching QoS profiles
5. Document findings

**Acceptance Criteria**:
- [ ] Demonstrated QoS mismatch (no communication)
- [ ] Fixed with matching profiles
- [ ] Explained why mismatch failed

**Estimated Time**: 30 minutes

### Exercise 3: Complete Module 1 Project (Capstone)

**Objective**: Build the Module 1 assessment project.

**Requirements** (from intro.md):
1. ROS 2 package controlling 6+ humanoid joints
2. Publish joint commands, subscribe to joint states
3. Service to execute pre-defined poses ("wave", "sit", "stand")
4. Launch file starting all nodes
5. RViz2 visualization
6. Demo video (2 to 3 minutes)

**Hints**:
- Use URDF from Chapter 1.3
- Publisher/subscriber from Chapter 1.2
- Service from Chapter 1.2
- Launch file from Chapter 1.4
- Parameters from this chapter

**Acceptance Criteria**:
- [ ] All requirements met (see intro.md grading rubric)
- [ ] GitHub repository with code
- [ ] Demo video uploaded

**Estimated Time**: 4 to 6 hours (module project)

---

## Additional Resources

- [ROS 2 Parameters Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html) - Official guide
- [QoS Policies](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html) - Complete QoS reference
- [Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html) - Design document
- [ROS 2 Best Practices](https://docs.ros.org/en/humble/Contributing/Developer-Guide.html) - Coding standards

---

**Previous**: [← Chapter 1.4: Package Development & Launch Files](chapter-1 to 4.md) | **Next**: [Module 2: The Digital Twin (Gazebo & Unity) →](../module-2/intro.md)

**Congratulations!** You've completed Module 1. You now understand ROS 2's architecture, can write Python nodes, model robots with URDF, package code, and manage parameters. Module 2 will teach you to simulate humanoids in Gazebo and Unity!
