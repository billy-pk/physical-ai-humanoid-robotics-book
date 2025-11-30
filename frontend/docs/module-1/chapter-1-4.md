---
sidebar_position: 5
title: 1.4 Package Development & Launch Files
---

# Chapter 1.4: Package Development & Launch Files

ROS 2 packages are the fundamental unit of organization—they contain nodes, launch files, configuration files, and robot descriptions. Launch files automate the startup of multi-node systems, eliminating the need to manually run dozens of terminals.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Create** ROS 2 Python packages using \`ros2 pkg create\`
- **Structure** packages with proper \`package.xml\` and \`setup.py\` configuration
- **Build** workspaces with \`colcon\` and manage dependencies
- **Write** launch files in Python to coordinate multiple nodes
- **Deploy** complete robot systems with a single command

## Prerequisites

- **ROS 2 Humble installed**
- **Completed Chapter 1.2** (Python nodes with rclpy)
- **Completed Chapter 1.3** (URDF files)
- **Basic understanding** of Python packaging

## Part 1: ROS 2 Package Structure

### What is a ROS 2 Package?

A **package** is a directory containing:
- **Source code**: Python nodes (\`.py\` files)
- **Configuration**: \`package.xml\`, \`setup.py\`, \`setup.cfg\`
- **Launch files**: \`.launch.py\` files
- **Resources**: URDF files, meshes, config YAML files

**Example package structure**:
```
my_humanoid_control/
├── package.xml           # Package metadata and dependencies
├── setup.py              # Python package installation config
├── setup.cfg             # Entry points for executables
├── resource/             # Package marker files
├── my_humanoid_control/  # Python module directory
│   ├── __init__.py
│   ├── joint_controller.py
│   └── sensor_processor.py
├── launch/               # Launch files
│   └── robot_bringup.launch.py
├── urdf/                 # Robot descriptions
│   └── humanoid.urdf
└── config/               # Parameter files
    └── controller_params.yaml
```

### Package Types

| Type | Language | Build System | Use Case |
|------|----------|--------------|----------|
| **Python** | Python | setuptools | Most rapid development |
| **CMake** | C++ | CMake | Performance-critical nodes |
| **Hybrid** | C++ + Python | CMake + setuptools | Large projects |

This chapter focuses on **Python packages** (most common for beginners).

## Part 2: Hands-On Tutorial

### Project 1: Create Your First ROS 2 Package

**Goal**: Create a package for humanoid joint control with publisher/subscriber nodes.

**Step 1: Create workspace**
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Step 2: Create package**
```bash
ros2 pkg create --build-type ament_python humanoid_control \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs

# Output:
# going to create a new package
# package name: humanoid_control
# destination directory: /home/user/ros2_ws/src
# package format: 3
# version: 0.0.0
# ...
```

**Explanation**:
- \`--build-type ament_python\`: Python package
- \`--dependencies\`: Auto-add dependencies to \`package.xml\`

**Step 3: Examine generated files**
```bash
cd humanoid_control
ls -la
```

Output:
```
humanoid_control/
├── humanoid_control/
│   └── __init__.py
├── package.xml
├── resource/
│   └── humanoid_control
├── setup.cfg
├── setup.py
└── test/
```

---

### Project 2: Add Nodes to Package

**Step 1: Create joint controller node**

**File**: \`humanoid_control/humanoid_control/joint_controller.py\`

```python
#!/usr/bin/env python3
"""
Humanoid Joint Controller Node
Publishes joint commands for humanoid arms
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class HumanoidJointController(Node):
    """
    Publishes joint commands to /joint_commands topic.
    Simulates sinusoidal arm motion.
    """
    def __init__(self):
        super().__init__('humanoid_joint_controller')
        
        # Publisher for joint states
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # Timer for 50 Hz control loop
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # Joint names
        self.joint_names = [
            'left_shoulder',
            'left_elbow',
            'right_shoulder',
            'right_elbow'
        ]
        
        self.counter = 0
        self.get_logger().info('Humanoid Joint Controller started (50 Hz)')
    
    def control_loop(self):
        """Generate and publish joint commands."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Generate sinusoidal motion
        t = self.counter * 0.02  # Time in seconds
        msg.position = [
            0.5 * math.sin(t),                # Left shoulder
            0.8 * math.sin(t * 1.5),          # Left elbow
            0.5 * math.sin(t + math.pi),      # Right shoulder (out of phase)
            0.8 * math.sin(t * 1.5 + math.pi) # Right elbow
        ]
        
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidJointController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 2: Create sensor processor node**

**File**: \`humanoid_control/humanoid_control/sensor_processor.py\`

```python
#!/usr/bin/env python3
"""
Sensor Processor Node
Processes joint state feedback
ROS 2 Humble | Python 3.10+
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SensorProcessor(Node):
    """
    Subscribes to /joint_commands and logs joint states.
    """
    def __init__(self):
        super().__init__('sensor_processor')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Sensor Processor started')
    
    def joint_state_callback(self, msg):
        """Process joint state messages."""
        if len(msg.position) > 0:
            # Log first joint position (left shoulder)
            self.get_logger().info(
                f'{msg.name[0]}: {msg.position[0]:.3f} rad',
                throttle_duration_sec=1.0  # Log max once per second
            )

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 3: Register nodes in setup.py**

Edit \`setup.py\` to add entry points:

```python
from setuptools import find_packages, setup

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/robot_bringup.launch.py']),
        # Install URDF files
        ('share/' + package_name + '/urdf', ['urdf/simple_arm.urdf']),
        # Install config files
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Humanoid robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = humanoid_control.joint_controller:main',
            'sensor_processor = humanoid_control.sensor_processor:main',
        ],
    },
)
```

**Key sections**:
- **Lines 15 to 20**: Install additional files (launch, URDF, config)
- **Lines 28 to 31**: Entry points create executable commands

**Step 4: Build the package**
```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_control

# Output:
# Starting >>> humanoid_control
# Finished <<< humanoid_control [1.23s]
#
# Summary: 1 package finished [1.45s]
```

**Step 5: Source the workspace**
```bash
source ~/ros2_ws/install/setup.bash

# Add to ~/.bashrc for automatic sourcing:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Step 6: Run the nodes**
```bash
# Terminal 1: Joint controller
ros2 run humanoid_control joint_controller

# Terminal 2: Sensor processor
ros2 run humanoid_control sensor_processor
```

**Expected Output** (Terminal 2):
```
[INFO] [sensor_processor]: Sensor Processor started
[INFO] [sensor_processor]: left_shoulder: 0.041 rad
[INFO] [sensor_processor]: left_shoulder: 0.082 rad
```

---

### Project 3: Create Launch File

**Goal**: Start both nodes with a single command using a launch file.

**Step 1: Create launch directory**
```bash
cd ~/ros2_ws/src/humanoid_control
mkdir -p launch
```

**Step 2: Write launch file**

**File**: \`launch/robot_bringup.launch.py\`

```python
#!/usr/bin/env python3
"""
Robot Bringup Launch File
Starts joint controller and sensor processor nodes
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for humanoid control system.
    
    Starts:
    - joint_controller: Publishes joint commands
    - sensor_processor: Processes joint feedback
    """
    return LaunchDescription([
        # Joint Controller Node
        Node(
            package='humanoid_control',
            executable='joint_controller',
            name='joint_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Sensor Processor Node
        Node(
            package='humanoid_control',
            executable='sensor_processor',
            name='sensor_processor',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
```

**Explanation**:
- **Lines 6 to 7**: Import launch system modules
- **Lines 19 to 28**: Define first node with package, executable, output mode
- **Lines 30 to 38**: Define second node
- **output='screen'**: Display logs in terminal (vs. log files)

**Step 3: Rebuild with launch file**
```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_control
source install/setup.bash
```

**Step 4: Run launch file**
```bash
ros2 launch humanoid_control robot_bringup.launch.py
```

**Expected Output**:
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [joint_controller]: Humanoid Joint Controller started (50 Hz)
[INFO] [sensor_processor]: Sensor Processor started
[INFO] [sensor_processor]: left_shoulder: 0.041 rad
[INFO] [sensor_processor]: left_shoulder: 0.123 rad
```

**Stop all nodes**: \`Ctrl+C\` once

---

### Project 4: Advanced Launch File with RViz2

**Goal**: Launch nodes + URDF visualization in RViz2 automatically.

**File**: \`launch/robot_visualization.launch.py\`

```python
#!/usr/bin/env python3
"""
Robot Visualization Launch File
Launches controller, state publisher, and RViz2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('humanoid_control')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_arm.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Robot State Publisher (publishes robot transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Joint State Publisher GUI (manual joint control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'robot_view.rviz')]
        ),
        
        # Joint Controller
        Node(
            package='humanoid_control',
            executable='joint_controller',
            name='joint_controller',
            output='screen'
        ),
    ])
```

**Explanation**:
- **Lines 14 to 21**: Load URDF file from package
- **Lines 25 to 29**: Declare launch argument (can override from command line)
- **Lines 31 to 41**: \`robot_state_publisher\` converts URDF → TF transforms
- **Lines 43 to 49**: GUI for manual joint control
- **Lines 51 to 57**: RViz2 with custom config file

**Run**:
```bash
ros2 launch humanoid_control robot_visualization.launch.py
```

---

### Step 3: Debugging Common Issues

#### Issue 1: "Package 'humanoid_control' not found"
**Cause**: Workspace not sourced after build

**Solution**:
```bash
cd ~/ros2_ws
source install/setup.bash
```

#### Issue 2: "No executable found"
**Cause**: Entry point not defined in \`setup.py\`

**Solution**:
```python
# In setup.py, add:
entry_points={
    'console_scripts': [
        'joint_controller = humanoid_control.joint_controller:main',
    ],
},
```
Then rebuild: \`colcon build --packages-select humanoid_control\`

#### Issue 3: Launch file not found
**Cause**: Launch file not installed in \`setup.py\`

**Solution**:
```python
# In setup.py data_files:
('share/' + package_name + '/launch', ['launch/robot_bringup.launch.py']),
```
Rebuild after modifying \`setup.py\`.

#### Issue 4: "colcon: command not found"
**Cause**: colcon not installed

**Solution**:
```bash
sudo apt install python3-colcon-common-extensions
```

## Part 3: Advanced Topics (Optional)

### Multi-Package Workspaces

Real projects have multiple packages:
```
ros2_ws/
├── src/
│   ├── humanoid_control/      # Control nodes
│   ├── humanoid_description/  # URDF files
│   ├── humanoid_navigation/   # Nav2 config
│   └── humanoid_perception/   # Vision nodes
```

**Build all packages**:
```bash
colcon build  # Builds all packages in src/
```

**Build specific package**:
```bash
colcon build --packages-select humanoid_control
```

### Launch File Composition

Include other launch files:
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
    ])
)
```

### Conditional Node Launching

Start nodes based on conditions:
```python
from launch.conditions import IfCondition

Node(
    package='humanoid_control',
    executable='joint_controller',
    condition=IfCondition(LaunchConfiguration('use_controller'))
)
```

Run with:
```bash
ros2 launch my_pkg robot.launch.py use_controller:=true
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Capstone workspace** will have 5+ packages:
  ```
  capstone_ws/
  ├── humanoid_description/  # URDF models
  ├── humanoid_bringup/      # Launch files
  ├── voice_control/         # Whisper node
  ├── llm_planner/           # GPT-4 planning
  └── navigation/            # Nav2 config
  ```

- **Master launch file** starts entire system:
  ```bash
  ros2 launch humanoid_bringup full_system.launch.py
  ```
  This single command launches:
  - Voice input node (Whisper)
  - LLM planner (GPT-4)
  - Navigation stack (Nav2)
  - Object detection (YOLO)
  - Joint controllers
  - Gazebo/Isaac Sim

Understanding package structure and launch files is critical for managing complex multi-node systems.

## Summary

You learned:
- ✅ Created **ROS 2 Python packages** with proper structure
- ✅ Configured **\`package.xml\`** and **\`setup.py\`** with dependencies and entry points
- ✅ Built packages with **colcon** and sourced workspaces
- ✅ Wrote **launch files** to start multiple nodes with one command
- ✅ Integrated URDF, nodes, and visualization in automated launch sequences

**Next steps**: In Chapter 1.5, you'll learn parameter management, QoS profiles, and best practices for production-ready ROS 2 systems.

---

## Exercises

### Exercise 1: Create Custom Package (Required)

**Objective**: Create a package for a temperature monitoring system.

**Tasks**:
1. Create package \`temperature_monitor\`
2. Add node \`temp_publisher.py\` (publishes random temps 20 to 30°C to \`/temperature\`)
3. Add node \`temp_analyzer.py\` (subscribes, logs if > 25°C)
4. Create launch file starting both nodes
5. Build and test

**Acceptance Criteria**:
- [ ] Package builds without errors
- [ ] \`ros2 run temperature_monitor temp_publisher\` works
- [ ] Launch file starts both nodes
- [ ] \`ros2 topic echo /temperature\` shows data

**Estimated Time**: 60 minutes

### Exercise 2: Multi-Robot Launch (Challenge)

**Objective**: Launch multiple instances of the same node with different namespaces.

**Tasks**:
1. Modify launch file to start 3 joint controllers
2. Each controller publishes to:
   - \`/robot1/joint_commands\`
   - \`/robot2/joint_commands\`
   - \`/robot3/joint_commands\`

**Hint**:
```python
Node(
    package='humanoid_control',
    executable='joint_controller',
    name='joint_controller',
    namespace='robot1',  # This adds /robot1 prefix
    output='screen'
)
```

**Acceptance Criteria**:
- [ ] Launch file starts 3 controllers
- [ ] \`ros2 topic list\` shows 3 separate topics
- [ ] Each topic publishes independently

**Estimated Time**: 45 minutes

### Exercise 3: Package with Dependencies (Advanced)

**Objective**: Create a package that depends on another package.

**Tasks**:
1. Create package \`robot_description\` (contains URDF files only)
2. Create package \`robot_control\` (depends on \`robot_description\`)
3. Launch file in \`robot_control\` loads URDF from \`robot_description\`

**Hints**:
- Add dependency in \`package.xml\`:
  ```xml
  <depend>robot_description</depend>
  ```
- Use \`get_package_share_directory('robot_description')\` in launch file

**Acceptance Criteria**:
- [ ] Both packages build
- [ ] \`robot_control\` can access URDF from \`robot_description\`
- [ ] Launch file works

**Estimated Time**: 90 minutes

---

## Additional Resources

- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) - Official guide
- [Launch File Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) - Complete launch system reference
- [colcon Documentation](https://colcon.readthedocs.io/) - Build system details
- [ament_cmake vs. ament_python](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html) - When to use C++ vs. Python

---

**Previous**: [← Chapter 1.3: URDF for Humanoid Robots](chapter-1 to 3.md) | **Next**: [Chapter 1.5: Parameter Management & Best Practices →](chapter-1 to 5.md)
