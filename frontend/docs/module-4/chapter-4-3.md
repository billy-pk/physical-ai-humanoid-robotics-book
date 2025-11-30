---
sidebar_position: 4
title: 4.3 Natural Language to ROS 2 Actions
---

# Chapter 4.3: Natural Language to ROS 2 Actions

LLM-generated plans must be executed by the robot. This chapter covers mapping natural language actions to ROS 2 Action goals, implementing action executors, handling preconditions and postconditions, and creating feedback mechanisms for robust task execution.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Map** natural language actions to ROS 2 Action goals
- **Implement** action executor coordinating multiple behaviors
- **Handle** action preconditions and postconditions
- **Create** feedback mechanisms for action completion
- **Debug** action execution failures and recovery

## Prerequisites

- **Chapter 4.2** completed (LLM planning)
- **ROS 2 Actions** understanding (Module 1, Chapter 1.1)
- **ROS 2 Humble** configured
- **Python 3.10+** with rclpy experience
- **Understanding** of state machines and action coordination

## Part 1: Action Mapping Fundamentals

### Natural Language → ROS 2 Actions

**Mapping challenge**: LLM generates abstract actions ("pick_up", "navigate_to"), but ROS 2 Actions require specific goals (coordinates, object IDs, poses).

**Mapping strategies**:
1. **Direct mapping**: Simple actions map directly (stop → emergency stop action)
2. **Parameter extraction**: Extract parameters from LLM output (location → coordinates)
3. **Query resolution**: Resolve ambiguous references ("the cup" → object ID via vision)
4. **Precondition checking**: Verify action can execute (object exists, robot can reach)

### ROS 2 Action Types

| Action Type | Use Case | Example |
|-------------|----------|---------|
| **NavigateToPose** | Navigation | Move to (x, y, theta) |
| **FollowPath** | Path following | Follow planned path |
| **PickObject** | Manipulation | Grasp object at pose |
| **PlaceObject** | Manipulation | Place object at pose |
| **LookAt** | Perception | Point camera at target |
| **Speak** | Communication | Text-to-speech output |

**Custom actions**: Define your own for robot-specific behaviors.

### Action Execution Flow

**Typical flow**:
1. **Receive plan**: JSON array of actions from LLM
2. **Validate preconditions**: Check if action can execute
3. **Map to ROS 2 Action**: Convert abstract action → Action goal
4. **Send goal**: Call ROS 2 Action client
5. **Monitor execution**: Track progress and handle feedback
6. **Check postconditions**: Verify action completed successfully
7. **Handle errors**: Replan if action fails

## Part 2: Hands-On Tutorial

### Project: Action Executor for Humanoid Robot

**Goal**: Create action executor that takes LLM-generated plans and executes them via ROS 2 Actions.

**Tools**: ROS 2 Humble, rclpy, action clients, Python 3.10+

### Step 1: Define Custom Actions

**Create action package**: `humanoid_actions`

```bash
cd ~/isaac_ros_ws/src
ros2 pkg create --build-type ament_cmake humanoid_actions --dependencies rclcpp action_msgs geometry_msgs
cd humanoid_actions
mkdir action
```

**Define PickObject action**: `action/PickObject.action`

```
# Goal: Object to pick up
string object_id
geometry_msgs/PoseStamped object_pose

---
# Result: Success status
bool success
string message

---
# Feedback: Current status
string status
float32 progress  # 0.0 to 1.0
```

**Define PlaceObject action**: `action/PlaceObject.action`

```
# Goal: Where to place object
string object_id
geometry_msgs/PoseStamped target_pose

---
# Result
bool success
string message

---
# Feedback
string status
float32 progress
```

**Update `CMakeLists.txt`**:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickObject.action"
  "action/PlaceObject.action"
  DEPENDENCIES geometry_msgs action_msgs
)
```

**Build actions**:
```bash
cd ~/isaac_ros_ws
colcon build --packages-select humanoid_actions
source install/setup.bash
```

### Step 2: Create Action Executor Node

**Create executor**: `voice_commands/action_executor.py`

```python
#!/usr/bin/env python3
"""
Action executor for humanoid robot
Executes LLM-generated plans via ROS 2 Actions
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import json
import math

# Import custom actions (after building)
try:
    from humanoid_actions.action import PickObject, PlaceObject
    CUSTOM_ACTIONS_AVAILABLE = True
except ImportError:
    CUSTOM_ACTIONS_AVAILABLE = False
    print("Warning: Custom actions not available, using placeholders")

class ActionExecutor(Node):
    """
    Executes LLM-generated action plans via ROS 2 Actions
    """
    def __init__(self):
        super().__init__('action_executor')
        
        # Subscribe to plans
        self.plan_sub = self.create_subscription(
            String,
            '/voice_commands/plan',
            self.plan_callback,
            10
        )
        
        # Publishers for status
        self.status_pub = self.create_publisher(String, '/voice_commands/execution_status', 10)
        self.complete_pub = self.create_publisher(String, '/voice_commands/action_complete', 10)
        self.error_pub = self.create_publisher(String, '/voice_commands/action_error', 10)
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        if CUSTOM_ACTIONS_AVAILABLE:
            self.pick_client = ActionClient(self, PickObject, 'pick_object')
            self.place_client = ActionClient(self, PlaceObject, 'place_object')
        
        # Execution state
        self.current_plan = []
        self.current_action_index = 0
        self.executing = False
        
        # Location mapping (simplified - would use VSLAM/vision in production)
        self.location_map = {
            "kitchen": {"x": 3.0, "y": 2.0, "theta": 0.0},
            "living_room": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "table": {"x": 2.0, "y": 1.0, "theta": 0.0},
            "shelf": {"x": 1.0, "y": 3.0, "theta": 1.57},
        }
        
        self.get_logger().info('Action executor started')
    
    def plan_callback(self, msg):
        """Receive plan and start execution"""
        try:
            plan = json.loads(msg.data)
            if isinstance(plan, list) and len(plan) > 0:
                self.current_plan = plan
                self.current_action_index = 0
                self.executing = True
                self.get_logger().info(f'Received plan with {len(plan)} actions')
                self.execute_next_action()
            else:
                self.get_logger().error('Invalid plan format')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse plan: {e}')
    
    def execute_next_action(self):
        """Execute next action in plan"""
        if not self.executing or self.current_action_index >= len(self.current_plan):
            self.executing = False
            self.publish_status("Plan complete")
            return
        
        action = self.current_plan[self.current_action_index]
        action_name = action.get('action')
        parameters = action.get('parameters', {})
        
        self.get_logger().info(f'Executing action {self.current_action_index + 1}/{len(self.current_plan)}: {action_name}')
        self.publish_status(f"Executing: {action_name}")
        
        # Map action to ROS 2 Action
        if action_name == 'navigate_to':
            self.execute_navigate(parameters)
        elif action_name == 'pick_up':
            self.execute_pick_up(parameters)
        elif action_name == 'place':
            self.execute_place(parameters)
        elif action_name == 'look_at':
            self.execute_look_at(parameters)
        elif action_name == 'speak':
            self.execute_speak(parameters)
        elif action_name == 'wait':
            self.execute_wait(parameters)
        else:
            self.get_logger().error(f'Unknown action: {action_name}')
            self.handle_action_error(action_name, f"Unknown action: {action_name}")
    
    def execute_navigate(self, parameters):
        """Execute navigation action"""
        location = parameters.get('location')
        
        if location not in self.location_map:
            self.get_logger().error(f'Unknown location: {location}')
            self.handle_action_error('navigate_to', f"Unknown location: {location}")
            return
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            self.handle_action_error('navigate_to', "Action server not available")
            return
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        loc = self.location_map[location]
        goal_msg.pose.pose.position.x = loc['x']
        goal_msg.pose.pose.position.y = loc['y']
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = self.euler_to_quaternion(0, 0, loc['theta'])
        goal_msg.pose.pose.orientation = q
        
        # Send goal
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def execute_pick_up(self, parameters):
        """Execute pick up action"""
        object_id = parameters.get('object')
        
        if not CUSTOM_ACTIONS_AVAILABLE:
            self.get_logger().warn('PickObject action not available, simulating...')
            # Simulate action
            self.get_clock().create_timer(2.0, lambda: self.action_complete('pick_up', {"object": object_id}))
            return
        
        if not self.pick_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('PickObject action server not available')
            self.handle_action_error('pick_up', "Action server not available")
            return
        
        # Get object pose (would query vision system in production)
        object_pose = self.get_object_pose(object_id)
        if not object_pose:
            self.handle_action_error('pick_up', f"Object not found: {object_id}")
            return
        
        # Create goal
        goal_msg = PickObject.Goal()
        goal_msg.object_id = object_id
        goal_msg.object_pose = object_pose
        
        # Send goal
        self.send_goal_future = self.pick_client.send_goal_async(
            goal_msg,
            feedback_callback=self.pick_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.pick_goal_response_callback)
    
    def execute_place(self, parameters):
        """Execute place action"""
        object_id = parameters.get('object')
        location = parameters.get('location')
        
        if not CUSTOM_ACTIONS_AVAILABLE:
            self.get_logger().warn('PlaceObject action not available, simulating...')
            self.get_clock().create_timer(2.0, lambda: self.action_complete('place', {"object": object_id, "location": location}))
            return
        
        if not self.place_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('PlaceObject action server not available')
            self.handle_action_error('place', "Action server not available")
            return
        
        # Get target pose
        if location in self.location_map:
            target_pose = self.create_pose_stamped(location)
        else:
            self.handle_action_error('place', f"Unknown location: {location}")
            return
        
        # Create goal
        goal_msg = PlaceObject.Goal()
        goal_msg.object_id = object_id
        goal_msg.target_pose = target_pose
        
        # Send goal
        self.send_goal_future = self.place_client.send_goal_async(
            goal_msg,
            feedback_callback=self.place_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.place_goal_response_callback)
    
    def execute_look_at(self, parameters):
        """Execute look at action (simplified)"""
        object_id = parameters.get('object')
        self.get_logger().info(f'Looking at: {object_id}')
        # In production: Control camera/head to look at object
        # For now: Simulate
        self.action_complete('look_at', {"object": object_id})
    
    def execute_speak(self, parameters):
        """Execute speak action"""
        text = parameters.get('text', '')
        self.get_logger().info(f'Robot says: {text}')
        # In production: Use TTS (text-to-speech) system
        # For now: Log message
        self.action_complete('speak', {"text": text})
    
    def execute_wait(self, parameters):
        """Execute wait action"""
        duration = parameters.get('duration', 1.0)
        self.get_logger().info(f'Waiting for {duration} seconds')
        # Create timer for wait
        self.get_clock().create_timer(duration, lambda: self.action_complete('wait', {"duration": duration}))
    
    # Action response callbacks
    def nav_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.handle_action_error('navigate_to', "Goal rejected")
            return
        
        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation progress: {feedback.current_pose.pose.position}')
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        if result:
            self.get_logger().info('Navigation completed successfully')
            self.action_complete('navigate_to', {"location": "target"})
        else:
            self.handle_action_error('navigate_to', "Navigation failed")
    
    def pick_goal_response_callback(self, future):
        """Handle pick goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.handle_action_error('pick_up', "Goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.pick_result_callback)
    
    def pick_feedback_callback(self, feedback_msg):
        """Handle pick feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Pick progress: {feedback.progress:.2f}')
    
    def pick_result_callback(self, future):
        """Handle pick result"""
        result = future.result().result
        if result.success:
            self.action_complete('pick_up', {"object": "target"})
        else:
            self.handle_action_error('pick_up', result.message)
    
    def place_goal_response_callback(self, future):
        """Handle place goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.handle_action_error('place', "Goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.place_result_callback)
    
    def place_feedback_callback(self, feedback_msg):
        """Handle place feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Place progress: {feedback.progress:.2f}')
    
    def place_result_callback(self, future):
        """Handle place result"""
        result = future.result().result
        if result.success:
            self.action_complete('place', {"object": "target", "location": "target"})
        else:
            self.handle_action_error('place', result.message)
    
    def action_complete(self, action_name, result_data):
        """Handle action completion"""
        self.get_logger().info(f'Action completed: {action_name}')
        
        # Publish completion
        complete_msg = String()
        complete_msg.data = json.dumps({
            "action": action_name,
            "result": result_data
        })
        self.complete_pub.publish(complete_msg)
        
        # Move to next action
        self.current_action_index += 1
        self.execute_next_action()
    
    def handle_action_error(self, action_name, error_message):
        """Handle action failure"""
        self.get_logger().error(f'Action failed: {action_name} - {error_message}')
        
        # Publish error
        error_msg = String()
        error_msg.data = json.dumps({
            "action": action_name,
            "error": error_message,
            "original_command": "unknown"  # Would track original command
        })
        self.error_pub.publish(error_msg)
        
        # Stop execution (or trigger replanning)
        self.executing = False
        self.publish_status("Execution failed")
    
    def publish_status(self, status):
        """Publish execution status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    # Helper functions
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        q = Quaternion()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        return q
    
    def get_object_pose(self, object_id):
        """Get object pose (would query vision system)"""
        # Placeholder - in production: Query vision/VSLAM for object pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 2.0
        pose.pose.position.y = 1.0
        pose.pose.position.z = 0.5
        pose.pose.orientation.w = 1.0
        return pose
    
    def create_pose_stamped(self, location):
        """Create pose from location name"""
        if location not in self.location_map:
            return None
        loc = self.location_map[location]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = loc['x']
        pose.pose.position.y = loc['y']
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.euler_to_quaternion(0, 0, loc['theta'])
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Add to setup.py**:
```python
entry_points={
    'console_scripts': [
        'whisper_node = voice_commands.whisper_node:main',
        'command_processor = voice_commands.command_processor:main',
        'llm_planner = voice_commands.llm_planner:main',
        'action_executor = voice_commands.action_executor:main',
    ],
},
```

### Step 3: Test Action Execution

**Launch complete pipeline**:
```bash
# Terminal 1: Whisper
ros2 run voice_commands whisper_node

# Terminal 2: Command processor
ros2 run voice_commands command_processor

# Terminal 3: LLM planner
ros2 run voice_commands llm_planner

# Terminal 4: Action executor
ros2 run voice_commands action_executor

# Terminal 5: Nav2 (if available)
ros2 launch nav2_bringup navigation_launch.py
```

**Test end-to-end**:
1. Say: "Pick up the red cup and place it on the table"
2. Watch execution:
   - Whisper transcribes
   - LLM generates plan
   - Action executor executes actions
   - Status updates published

**Monitor topics**:
```bash
ros2 topic echo /voice_commands/execution_status
ros2 topic echo /voice_commands/action_complete
ros2 topic echo /voice_commands/action_error
```

### Step 4: Precondition Checking

**Add precondition validation**: `voice_commands/precondition_checker.py`

```python
#!/usr/bin/env python3
"""
Check action preconditions before execution
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PreconditionChecker(Node):
    def __init__(self):
        super().__init__('precondition_checker')
        
        # Subscribe to plans
        self.plan_sub = self.create_subscription(
            String,
            '/voice_commands/plan',
            self.plan_callback,
            10
        )
        
        # Publisher for validated plans
        self.validated_plan_pub = self.create_publisher(String, '/voice_commands/validated_plan', 10)
        
        # Robot state (would subscribe to actual state topics)
        self.robot_state = {
            "location": "living_room",
            "held_object": None,
            "battery_level": 100.0,
        }
    
    def plan_callback(self, msg):
        """Validate plan preconditions"""
        plan = json.loads(msg.data)
        validated_plan = []
        
        for action in plan:
            action_name = action.get('action')
            parameters = action.get('parameters', {})
            
            # Check preconditions
            if self.check_preconditions(action_name, parameters):
                validated_plan.append(action)
            else:
                self.get_logger().warn(f'Action failed precondition check: {action_name}')
                # Could request replanning here
        
        # Publish validated plan
        if validated_plan:
            msg = String()
            msg.data = json.dumps(validated_plan)
            self.validated_plan_pub.publish(msg)
    
    def check_preconditions(self, action_name, parameters):
        """Check if action preconditions are met"""
        if action_name == 'pick_up':
            # Precondition: Robot must be near object
            object_id = parameters.get('object')
            # Check if object exists and is reachable
            return self.object_exists(object_id) and self.object_reachable(object_id)
        
        elif action_name == 'place':
            # Precondition: Robot must be holding object
            object_id = parameters.get('object')
            return self.robot_state['held_object'] == object_id
        
        elif action_name == 'navigate_to':
            # Precondition: Battery sufficient
            return self.robot_state['battery_level'] > 20.0
        
        return True  # Default: allow action
    
    def object_exists(self, object_id):
        """Check if object exists (would query vision system)"""
        # Placeholder
        return True
    
    def object_reachable(self, object_id):
        """Check if object is within reach (would check kinematics)"""
        # Placeholder
        return True
```

### Step 5: Debugging Common Issues

#### Issue 1: "Action server not available"
**Symptoms**: Action client can't connect to server

**Solutions**:
```bash
# Check if action server is running
ros2 action list

# Verify action server node
ros2 node list | grep [action_server_name]

# Check action server is in same ROS domain
echo $ROS_DOMAIN_ID
```

#### Issue 2: "Action goal rejected"
**Symptoms**: Action server rejects goal

**Solutions**:
```python
# Check goal format
# Verify goal message structure matches action definition
# Check goal parameters are valid (e.g., pose in valid range)

# Add goal validation
def validate_goal(self, goal_msg):
    # Check pose is in map bounds
    # Check object exists
    # Check robot can reach
    pass
```

#### Issue 3: "Action execution hangs"
**Symptoms**: Action never completes

**Solutions**:
```python
# Add timeout
import asyncio

async def execute_with_timeout(self, action_func, timeout=30.0):
    try:
        await asyncio.wait_for(action_func(), timeout=timeout)
    except asyncio.TimeoutError:
        self.get_logger().error('Action timeout')
        self.handle_action_error(action_name, "Timeout")
```

#### Issue 4: "Actions execute out of order"
**Symptoms**: Actions don't wait for previous action to complete

**Solutions**:
```python
# Ensure sequential execution
# Don't call execute_next_action() until current action completes
# Use callbacks properly (don't call next action in send_goal, wait for result)
```

## Part 3: Advanced Topics (Optional)

### Parallel Action Execution

**Execute independent actions in parallel**:
```python
# Some actions can run in parallel (e.g., look_at while navigating)
# Use asyncio or threading for parallel execution
# But ensure dependencies are respected
```

### Action Retry Logic

**Retry failed actions**:
```python
def execute_with_retry(self, action_func, max_retries=3):
    for attempt in range(max_retries):
        try:
            result = action_func()
            if result.success:
                return result
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            self.get_logger().warn(f'Action failed, retrying ({attempt + 1}/{max_retries})')
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Action execution**: Capstone will execute LLM plans via ROS 2 Actions
- **Robustness**: Precondition checking and error handling ensure reliable execution
- **Feedback**: Action completion feedback enables replanning
- **Integration**: Connects LLM planning to robot control

Understanding action execution now is essential for the capstone system.

## Summary

You learned:
- ✅ Mapped **natural language actions** to ROS 2 Action goals
- ✅ Implemented **action executor** coordinating multiple behaviors
- ✅ Handled **preconditions and postconditions** for robust execution
- ✅ Created **feedback mechanisms** for action completion
- ✅ Debugged **action execution failures** and recovery

**Next steps**: In Chapter 4.4, you'll integrate multi-modal inputs (speech + vision + gesture) for richer human-robot interaction.

---

## Exercises

### Exercise 1: Basic Action Execution (Required)

**Objective**: Execute simple action plans via ROS 2 Actions.

**Tasks**:
1. Create action executor node
2. Implement navigation action execution
3. Test with simple plan: `[{"action": "navigate_to", "parameters": {"location": "kitchen"}}]`
4. Verify action completes successfully
5. Monitor action feedback and results

**Acceptance Criteria**:
- [ ] Action executor node running
- [ ] Navigation action executes correctly
- [ ] Action completion published
- [ ] Feedback received during execution

**Estimated Time**: 120 minutes

### Exercise 2: Precondition Checking (Required)

**Objective**: Validate action preconditions before execution.

**Tasks**:
1. Create precondition checker node
2. Define preconditions for each action type
3. Validate plans before execution
4. Test with invalid plans (should be filtered)
5. Document precondition rules

**Acceptance Criteria**:
- [ ] Precondition checker implemented
- [ ] Invalid actions filtered out
- [ ] Valid plans pass through
- [ ] Precondition rules documented

**Estimated Time**: 90 minutes

### Exercise 3: Error Recovery (Challenge)

**Objective**: Implement robust error handling and recovery.

**Tasks**:
1. Detect action failures
2. Implement retry logic (3 attempts)
3. Trigger replanning on persistent failures
4. Test with simulated failures
5. Document recovery strategies

**Requirements**:
- Retry logic implemented
- Replanning triggered on failure
- Recovery strategies documented

**Estimated Time**: 180 minutes

---

## Additional Resources

- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) - Action implementation
- [Nav2 Actions](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2action_plugin.html) - Navigation actions
- [Action Client API](https://docs.ros2.org/humble/api/rclpy/api/rclpy.action.html) - rclpy action client

---

**Next**: [Chapter 4.4: Multi-Modal Integration (Speech + Vision + Gesture) →](chapter-4 to 4.md)
