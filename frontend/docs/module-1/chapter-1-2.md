---
sidebar_position: 3
title: 1.2 Python Integration with rclpy
---

# Chapter 1.2: Python Integration with rclpy

Python is the primary language for rapid robotics development. The \`rclpy\` library provides a Pythonic interface to ROS 2, enabling you to build nodes, publish data, subscribe to topics, and create services—all with clean, readable code.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Implement** publisher nodes that send sensor data to topics
- **Create** subscriber nodes that process incoming messages
- **Configure** timers for periodic control loops (e.g., 10 Hz, 50 Hz)
- **Build** service servers and clients for request/reply patterns
- **Debug** Python nodes using ROS 2 logging and introspection tools

## Prerequisites

- **ROS 2 Humble installed** (Chapter 1.1)
- **Python 3.10+** (check with \`python3 --version\`)
- **Basic Python**: classes, methods, \`__init__\`, decorators
- **Understanding of ROS 2 concepts**: nodes, topics, services (Chapter 1.1)

## Part 1: rclpy Fundamentals

### The rclpy Node Lifecycle

Every ROS 2 Python node follows this pattern:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize publishers, subscribers, timers, services
    
def main(args=None):
    rclpy.init(args=args)           # Initialize ROS 2 context
    node = MyNode()                  # Create node instance
    rclpy.spin(node)                 # Keep node alive and process callbacks
    node.destroy_node()              # Cleanup
    rclpy.shutdown()                 # Shutdown ROS 2 context

if __name__ == '__main__':
    main()
```

**Key functions**:
- **\`rclpy.init()\`**: Initializes ROS 2 communication
- **\`rclpy.spin()\`**: Blocks and processes callbacks (subscriptions, timers, services)
- **\`node.destroy_node()\`**: Cleans up resources
- **\`rclpy.shutdown()\`**: Closes ROS 2 context

### Message Types

ROS 2 messages are strongly typed. Common message packages:

| Package | Use Case | Example Types |
|---------|----------|---------------|
| **std_msgs** | Simple data | \`String\`, \`Int32\`, \`Float64\`, \`Bool\` |
| **sensor_msgs** | Sensor data | \`Image\`, \`LaserScan\`, \`Imu\`, \`JointState\` |
| **geometry_msgs** | Positions/velocities | \`Pose\`, \`Twist\`, \`PoseStamped\` |
| **nav_msgs** | Navigation | \`Odometry\`, \`Path\` |

**Import syntax**:
```python
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
```

## Part 2: Hands-On Tutorial

### Project 1: Simple Publisher Node

**Goal**: Create a node that publishes joint angle commands at 10 Hz.

**File**: \`simple_publisher.py\`

```python
#!/usr/bin/env python3
"""
Simple Publisher Node
Publishes joint angle commands to /joint_commands topic
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class JointCommandPublisher(Node):
    """
    Publishes sinusoidal joint commands for demonstration.
    Topic: /joint_commands
    Type: std_msgs/msg/Float64
    Rate: 10 Hz
    """
    def __init__(self):
        super().__init__('joint_command_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(
            Float64,                    # Message type
            '/joint_commands',          # Topic name
            10                          # Queue size (buffer last 10 messages)
        )
        
        # Create timer that calls timer_callback every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Counter for generating sinusoidal motion
        self.counter = 0
        
        self.get_logger().info('Joint Command Publisher started')
    
    def timer_callback(self):
        """
        Called every 0.1 seconds (10 Hz).
        Publishes sinusoidal joint angle.
        """
        msg = Float64()
        
        # Generate sinusoidal motion: angle = sin(counter * 0.1)
        # Range: -1 to 1 radians
        msg.data = math.sin(self.counter * 0.1)
        
        # Publish message
        self.publisher_.publish(msg)
        
        # Log every 10th message (once per second)
        if self.counter % 10 == 0:
            self.get_logger().info(f'Publishing joint command: {msg.data:.3f} rad')
        
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    
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
- **Lines 21 to 26**: Create publisher with topic name, message type, and queue size
- **Lines 28 to 29**: Timer triggers \`timer_callback()\` every 0.1 seconds
- **Lines 40 to 42**: Generate sinusoidal joint motion
- **Lines 44 to 45**: Publish message to topic
- **Queue size**: If subscribers are slow, keep last 10 messages in buffer

**Run the node**:
```bash
# Make executable
chmod +x simple_publisher.py

# Run directly (for testing)
python3 simple_publisher.py

# Or use ros2 run (after packaging in Chapter 1.4)
# ros2 run my_package simple_publisher
```

**Verify it's working**:
```bash
# Terminal 2: Echo the topic
ros2 topic echo /joint_commands

# Terminal 3: Check publish rate
ros2 topic hz /joint_commands
```

Expected output from \`ros2 topic hz\`:
```
average rate: 10.002
        min: 0.099s max: 0.101s std dev: 0.00050s window: 100
```

---

### Project 2: Simple Subscriber Node

**Goal**: Create a node that subscribes to joint commands and logs them.

**File**: \`simple_subscriber.py\`

```python
#!/usr/bin/env python3
"""
Simple Subscriber Node
Subscribes to joint commands and processes them
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommandSubscriber(Node):
    """
    Subscribes to /joint_commands and logs received values.
    Topic: /joint_commands
    Type: std_msgs/msg/Float64
    """
    def __init__(self):
        super().__init__('joint_command_subscriber')
        
        # Create subscription
        self.subscription = self.create_subscription(
            Float64,                        # Message type
            '/joint_commands',              # Topic name
            self.listener_callback,         # Callback function
            10                              # Queue size
        )
        
        self.get_logger().info('Joint Command Subscriber started')
    
    def listener_callback(self, msg):
        """
        Called whenever a message arrives on /joint_commands.
        
        Args:
            msg (Float64): Received joint angle in radians
        """
        # Convert radians to degrees for display
        angle_degrees = msg.data * (180.0 / 3.14159)
        
        self.get_logger().info(
            f'Received joint command: {msg.data:.3f} rad ({angle_degrees:.1f}°)'
        )

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandSubscriber()
    
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
- **Lines 20 to 25**: Create subscription with topic, message type, callback, and queue size
- **Lines 29 to 40**: Callback triggered automatically when messages arrive
- **Line 37**: Convert radians to degrees for human-readable output
- **Callback execution**: Runs in background while \`rclpy.spin()\` processes events

**Run both nodes together**:
```bash
# Terminal 1: Publisher
python3 simple_publisher.py

# Terminal 2: Subscriber
python3 simple_subscriber.py
```

**Expected output** (subscriber terminal):
```
[INFO] [1698765432.123] [joint_command_subscriber]: Joint Command Subscriber started
[INFO] [1698765432.224] [joint_command_subscriber]: Received joint command: 0.995 rad (57.0°)
[INFO] [1698765432.324] [joint_command_subscriber]: Received joint command: 0.985 rad (56.4°)
```

---

### Project 3: Service Server and Client

**Goal**: Create a service that calculates battery runtime based on current draw.

#### Service Server (Battery Monitor)

**File**: \`battery_service_server.py\`

```python
#!/usr/bin/env python3
"""
Battery Service Server
Provides battery runtime estimation service
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # We'll use this as template
from std_srvs.srv import Trigger

# For real robot, create custom service type:
# BatteryQuery.srv:
#   float64 current_draw  # Amps
#   ---
#   float64 runtime       # Hours
#   string status

class BatteryMonitor(Node):
    """
    Service that estimates battery runtime.
    Service: /battery/estimate_runtime
    Type: Trigger (simplified for demo)
    """
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Create service
        self.srv = self.create_service(
            Trigger,                              # Service type
            '/battery/get_status',                # Service name
            self.battery_status_callback          # Callback function
        )
        
        # Simulate battery state
        self.battery_percent = 85.0  # 85% charged
        
        self.get_logger().info('Battery Monitor Service ready')
    
    def battery_status_callback(self, request, response):
        """
        Called when a client requests battery status.
        
        Args:
            request (Trigger.Request): Empty request
            response (Trigger.Response): Contains success (bool) and message (string)
        
        Returns:
            Trigger.Response: Battery status message
        """
        # Simulate battery drain
        self.battery_percent -= 0.5
        if self.battery_percent < 0:
            self.battery_percent = 100.0  # Reset for demo
        
        # Prepare response
        response.success = True
        response.message = f'Battery: {self.battery_percent:.1f}% remaining'
        
        self.get_logger().info(f'Service called. {response.message}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client (Mission Planner)

**File**: \`battery_service_client.py\`

```python
#!/usr/bin/env python3
"""
Battery Service Client
Calls battery service to check status
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MissionPlanner(Node):
    """
    Client that queries battery status before starting missions.
    """
    def __init__(self):
        super().__init__('mission_planner')
        
        # Create client
        self.client = self.create_client(Trigger, '/battery/get_status')
        
        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for battery service...')
        
        self.get_logger().info('Mission Planner ready')
    
    def check_battery(self):
        """
        Calls the battery service and returns the response.
        
        Returns:
            Trigger.Response: Battery status
        """
        request = Trigger.Request()  # Empty request for Trigger service
        
        # Call service (blocks until response)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Battery status: {response.message}')
            return response
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    
    # Check battery 5 times
    for i in range(5):
        node.get_logger().info(f'--- Mission {i+1} ---')
        response = node.check_battery()
        
        if response and response.success:
            battery_percent = float(response.message.split(':')[1].split('%')[0])
            
            if battery_percent > 20.0:
                node.get_logger().info('✓ Battery OK, starting mission')
            else:
                node.get_logger().warn('⚠ Low battery, aborting mission')
        
        # Wait 1 second between checks
        import time
        time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run service and client**:
```bash
# Terminal 1: Start service server
python3 battery_service_server.py

# Terminal 2: Run client
python3 battery_service_client.py
```

**Expected Output** (client terminal):
```
[INFO] [mission_planner]: Mission Planner ready
[INFO] [mission_planner]: --- Mission 1 ---
[INFO] [mission_planner]: Battery status: Battery: 84.5% remaining
[INFO] [mission_planner]: ✓ Battery OK, starting mission
[INFO] [mission_planner]: --- Mission 2 ---
[INFO] [mission_planner]: Battery status: Battery: 84.0% remaining
[INFO] [mission_planner]: ✓ Battery OK, starting mission
```

---

### Step 4: Debugging Common Issues

#### Issue 1: "ModuleNotFoundError: No module named 'rclpy'"
**Cause**: ROS 2 environment not sourced

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Or add to ~/.bashrc
```

#### Issue 2: Node runs but doesn't publish/subscribe
**Cause**: Topic name mismatch (typo in topic name)

**Solution**:
```bash
# Check active topics
ros2 topic list

# Verify topic name matches exactly
ros2 topic info /joint_commands
```

#### Issue 3: "Message type 'X' not found"
**Cause**: Missing ROS 2 message package

**Solution**:
```bash
# Install common message packages
sudo apt install ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs
```

#### Issue 4: Subscriber callback never called
**Cause**: QoS profile mismatch between publisher and subscriber

**Solution**:
```python
# Use compatible QoS settings (more details in Chapter 1.5)
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
self.publisher_ = self.create_publisher(Float64, '/topic', qos)
self.subscription = self.create_subscription(Float64, '/topic', callback, qos)
```

## Part 3: Advanced Topics (Optional)

### Multi-threaded Execution

For complex nodes with multiple callbacks, use executors:

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node1)
executor.add_node(node2)

try:
    executor.spin()
finally:
    executor.shutdown()
```

**Use case**: Process camera images in one thread while controlling motors in another.

### Logging Levels

ROS 2 supports 5 logging levels:

```python
self.get_logger().debug('Detailed diagnostic info')
self.get_logger().info('Normal operation')
self.get_logger().warn('Warning, degraded performance')
self.get_logger().error('Error occurred, function failed')
self.get_logger().fatal('Critical error, node shutting down')
```

**Set logging level**:
```bash
ros2 run my_package my_node --ros-args --log-level DEBUG
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Voice input node**: Subscriber to \`/audio\` topic, publishes transcribed text
  ```python
  self.subscription = self.create_subscription(Audio, '/audio', self.process_audio, 10)
  self.text_pub = self.create_publisher(String, '/voice_command', 10)
  ```

- **LLM planning node**: Subscribes to \`/voice_command\`, publishes navigation goals
  ```python
  self.create_subscription(String, '/voice_command', self.plan_task, 10)
  self.goal_pub = self.create_publisher(PoseStamped, '/nav2/goal', 10)
  ```

- **Service for object detection**: "Is the red ball visible?"
  ```python
  self.create_service(Trigger, '/vision/detect_object', self.detect_callback)
  ```

Understanding publishers, subscribers, and services is essential for orchestrating the capstone's multi-node architecture.

## Summary

You learned:
- ✅ Created **publisher nodes** that send messages to topics at fixed rates
- ✅ Implemented **subscriber nodes** with callbacks to process incoming data
- ✅ Built **service servers** and **clients** for request/reply patterns
- ✅ Used **timers** for periodic control loops (10 Hz, 50 Hz)
- ✅ Debugged common \`rclpy\` issues with logging and introspection tools

**Next steps**: In Chapter 1.3, you'll model humanoid robots using URDF, the standard format for robot descriptions.

---

## Exercises

### Exercise 1: Velocity Command Publisher (Required)

**Objective**: Create a node that publishes velocity commands for a mobile robot.

**Tasks**:
1. Create \`velocity_publisher.py\`
2. Publish \`geometry_msgs/msg/Twist\` messages to \`/cmd_vel\`
3. Linear velocity: 0.5 m/s forward
4. Angular velocity: 0.1 rad/s (turning left)
5. Publish at 20 Hz

**Starter code**:
```python
from geometry_msgs.msg import Twist

msg = Twist()
msg.linear.x = 0.5   # m/s forward
msg.angular.z = 0.1  # rad/s counterclockwise
```

**Acceptance Criteria**:
- [ ] Node publishes at 20 Hz (verify with \`ros2 topic hz /cmd_vel\`)
- [ ] Twist message has correct linear and angular values
- [ ] Code includes logging statements

**Estimated Time**: 30 minutes

### Exercise 2: Odometry Subscriber (Required)

**Objective**: Subscribe to odometry data and extract robot position.

**Tasks**:
1. Create \`odom_subscriber.py\`
2. Subscribe to \`/odom\` topic (type: \`nav_msgs/msg/Odometry\`)
3. Extract robot's (x, y) position from \`msg.pose.pose.position\`
4. Log position every 1 second

**Install nav_msgs**:
```bash
sudo apt install ros-humble-nav-msgs
```

**Hint**:
```python
from nav_msgs.msg import Odometry

def callback(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    self.get_logger().info(f'Position: x={x:.2f}, y={y:.2f}')
```

**Acceptance Criteria**:
- [ ] Successfully subscribes to \`/odom\`
- [ ] Logs (x, y) position
- [ ] No errors when no publisher exists (node should wait gracefully)

**Estimated Time**: 30 minutes

### Exercise 3: Two-Node Communication (Challenge)

**Objective**: Create two nodes that communicate bidirectionally.

**Tasks**:
1. **Node A** (\`temperature_sensor.py\`):
   - Publishes random temperature (20 to 30°C) to \`/temperature\` at 5 Hz
   - Subscribes to \`/fan_speed\` and logs received values

2. **Node B** (\`fan_controller.py\`):
   - Subscribes to \`/temperature\`
   - If temperature > 25°C, publish fan_speed = 100 to \`/fan_speed\`
   - If temperature ≤ 25°C, publish fan_speed = 0

**Hints**:
```python
import random
temp = random.uniform(20.0, 30.0)
```

**Acceptance Criteria**:
- [ ] Both nodes run simultaneously
- [ ] Fan speed changes based on temperature
- [ ] Visualize with \`rqt_graph\` showing bidirectional communication

**Estimated Time**: 60 minutes

---

## Additional Resources

- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/) - Complete reference
- [ROS 2 Python Examples](https://github.com/ros2/examples/tree/humble/rclpy) - Official example nodes
- [Understanding Publishers and Subscribers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) - Official tutorial
- [Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) - Request/reply patterns

---

**Previous**: [← Chapter 1.1: ROS 2 Architecture](chapter-1 to 1.md) | **Next**: [Chapter 1.3: URDF for Humanoid Robots →](chapter-1 to 3.md)
