---
sidebar_position: 5
title: 3.4 Nav2 Navigation for Bipedal Humanoids
---

# Chapter 3.4: Nav2 Navigation for Bipedal Humanoids

Nav2 (Navigation 2) is the ROS 2 navigation framework that enables robots to plan paths, avoid obstacles, and reach goals autonomously. Configuring Nav2 for bipedal humanoids requires special considerations: wider turning radius, balance constraints, and dynamic obstacle avoidance.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Install** and configure Nav2 navigation stack for ROS 2
- **Set up** costmaps (global and local) for humanoid constraints
- **Configure** path planners (A*, Theta*) for bipedal robots
- **Implement** obstacle avoidance with humanoid-specific parameters
- **Test** navigation in simulation and validate path planning

## Prerequisites

- **ROS 2 Humble** configured (Module 1)
- **VSLAM** or **localization** system (Chapter 3.3 or alternative)
- **Laser scan** or **costmap** data source
- **Understanding** of path planning algorithms (A*, Dijkstra)
- **Basic knowledge** of costmaps and occupancy grids

## Part 1: Nav2 Architecture Fundamentals

### What is Nav2?

**Nav2** provides:
- **Global path planning**: Find path from start to goal
- **Local path planning**: Avoid obstacles while following global path
- **Costmaps**: Represent obstacles and costs in 2D grid
- **Recovery behaviors**: Handle stuck situations (rotate, backup)
- **Plugins**: Modular architecture for custom planners/controllers

**Why Nav2 for humanoids?**
- **Industry standard**: Used by Boston Dynamics, Fetch Robotics, Clearpath
- **Modular**: Customize for bipedal constraints
- **Real-time**: Efficient algorithms for dynamic environments
- **ROS 2 native**: Seamless integration with ROS 2 ecosystem

### Nav2 Components

| Component | Function | Humanoid Considerations |
|-----------|----------|------------------------|
| **Planner Server** | Global path planning | Wider turning radius |
| **Controller Server** | Local path following | Balance constraints |
| **Recovery Server** | Handle failures | Bipedal recovery (sit, stand) |
| **BT Navigator** | Behavior tree orchestration | Custom behaviors for humanoids |
| **Costmap 2D** | Obstacle representation | Humanoid footprint |

### Key Concepts

#### 1. Costmaps
**Costmaps** are 2D grids representing:
- **Obstacles**: High cost (255) = impassable
- **Free space**: Low cost (0) = traversable
- **Gradients**: Intermediate costs for navigation preferences

**Types**:
- **Global costmap**: Entire environment (for global planning)
- **Local costmap**: Surrounding area (for obstacle avoidance)

#### 2. Path Planners
**Planners** find paths through costmaps:

| Algorithm | Use Case | Humanoid Suitability |
|-----------|----------|----------------------|
| **A*** | Optimal paths | Good (configurable) |
| **Theta*** | Smooth paths | Excellent (fewer turns) |
| **NavFn** | Fast planning | Good (legacy) |
| **Smac Planner** | Hybrid A* | Excellent (smooth) |

**For humanoids**: Theta* or Smac Planner (smooth, fewer turns).

#### 3. Controllers
**Controllers** execute paths:

| Controller | Behavior | Humanoid Suitability |
|------------|----------|----------------------|
| **DWB** | Dynamic window | Good (configurable) |
| **TEB** | Timed Elastic Band | Excellent (smooth) |
| **MPPI** | Model Predictive | Excellent (balance-aware) |

**For humanoids**: TEB or MPPI (smooth, balance-aware).

## Part 2: Hands-On Tutorial

### Project: Configure Nav2 for Humanoid Navigation

**Goal**: Set up Nav2 navigation stack with humanoid-specific parameters and test path planning in simulation.

**Tools**: ROS 2 Humble, Nav2, VSLAM (from Chapter 3.3), Isaac Sim or Gazebo

### Step 1: Install Nav2

**Install Nav2 packages**:
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Verify installation
ros2 pkg list | grep nav2
```

**Key packages**:
- `nav2_bringup`: Launch files and configurations
- `nav2_planner`: Path planning algorithms
- `nav2_controller`: Path following controllers
- `nav2_recoveries`: Recovery behaviors
- `nav2_costmap_2d`: Costmap implementation

### Step 2: Create Nav2 Configuration

**Create package**:
```bash
cd ~/isaac_ros_ws/src
ros2 pkg create --build-type ament_cmake humanoid_nav2 --dependencies nav2_bringup nav2_planner nav2_controller
cd humanoid_nav2
mkdir -p config launch
```

**Global costmap config**: `config/global_costmap.yaml`

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3  # Humanoid radius (wider than wheeled robots)
      resolution: 0.05  # 5 cm per pixel
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0  # Humanoid height
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5  # Larger for humanoid safety
```

**Local costmap config**: `config/local_costmap.yaml`

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0  # Higher for dynamic obstacles
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          obstacle_max_range: 3.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5
```

**Planner config**: `config/planner.yaml`

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5  # Goal tolerance (meters)
      use_astar: false  # Use Dijkstra (smoother)
      allow_unknown: true
```

**Controller config**: `config/controller.yaml`

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    # Progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    # Goal checker
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    
    # Controller (DWB)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5  # Slower for humanoid stability
      max_vel_y: 0.0
      max_vel_theta: 0.5
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 0.5
      decel_lim_x: 0.5
      decel_lim_y: 0.0
      decel_lim_theta: 0.5
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.1
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      # Humanoid-specific: wider footprint
      footprint: "[[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]"
```

**Recovery config**: `config/recovery.yaml`

```yaml
recovery_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
    backup:
      plugin: "nav2_recoveries::BackUp"
    wait:
      plugin: "nav2_recoveries::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: True
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

**BT Navigator config**: `config/bt_navigator.yaml`

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
```

### Step 3: Create Launch File

**Launch file**: `launch/humanoid_nav2.launch.py`

```python
#!/usr/bin/env python3
"""
Launch Nav2 for humanoid robot
ROS 2 Humble | Nav2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('humanoid_nav2').find('humanoid_nav2')
    
    # Config file paths
    nav2_config = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    global_costmap = PathJoinSubstitution([pkg_share, 'config', 'global_costmap.yaml'])
    local_costmap = PathJoinSubstitution([pkg_share, 'config', 'local_costmap.yaml'])
    planner = PathJoinSubstitution([pkg_share, 'config', 'planner.yaml'])
    controller = PathJoinSubstitution([pkg_share, 'config', 'controller.yaml'])
    recovery = PathJoinSubstitution([pkg_share, 'config', 'recovery.yaml'])
    bt_navigator = PathJoinSubstitution([pkg_share, 'config', 'bt_navigator.yaml'])
    
    return LaunchDescription([
        # Nav2 lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'local_costmap',
                    'global_costmap'
                ]
            }]
        ),
        
        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner, {'use_sim_time': True}]
        ),
        
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller, {'use_sim_time': True}]
        ),
        
        # Recovery server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery, {'use_sim_time': True}]
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator, {'use_sim_time': True}]
        ),
        
        # Global costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[global_costmap, {'use_sim_time': True}]
        ),
        
        # Local costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[local_costmap, {'use_sim_time': True}]
        ),
    ])
```

### Step 4: Test Navigation

**Launch Nav2**:
```bash
cd ~/isaac_ros_ws
colcon build --packages-select humanoid_nav2
source install/setup.bash

ros2 launch humanoid_nav2 humanoid_nav2.launch.py
```

**Send navigation goal**:
```bash
# Set initial pose (required before navigation)
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Visualize in RViz2**:
```bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Add displays**:
1. **Map**: Topic `/map` (if using static map)
2. **Global Costmap**: Topic `/global_costmap/costmap`
3. **Local Costmap**: Topic `/local_costmap/costmap`
4. **Path**: Topic `/plan` (global path)
5. **TF**: Coordinate frames

### Step 5: Debugging Common Issues

#### Issue 1: "No path found"
**Symptoms**: Planner returns failure

**Solutions**:
```bash
# Check costmap is populated
ros2 topic echo /global_costmap/costmap --no-arr | head -20

# Verify robot pose is set
ros2 topic echo /amcl_pose  # If using AMCL
# Or check VSLAM odometry: /visual_slam/tracking/odometry

# Check goal is in free space
# Visualize costmap in RViz2
```

**Common causes**:
- **Goal in obstacle**: Goal position has high cost
- **No path exists**: Obstacles block all paths
- **Costmap empty**: No sensor data or map

#### Issue 2: "Robot oscillating" or "stuck"
**Symptoms**: Robot moves back and forth, doesn't reach goal

**Solutions**:
```yaml
# Increase goal tolerance
xy_goal_tolerance: 0.5  # Larger tolerance

# Reduce controller frequency
controller_frequency: 10.0  # Lower frequency

# Increase progress check radius
required_movement_radius: 1.0  # Larger radius
```

#### Issue 3: "Costmap not updating"
**Symptoms**: Costmap shows old obstacles

**Solutions**:
```bash
# Check sensor topics
ros2 topic hz /scan  # Should be publishing

# Verify costmap update frequency
# In config: update_frequency: 5.0

# Check costmap plugins loaded
ros2 param get /local_costmap plugins
```

## Part 3: Advanced Topics (Optional)

### Custom Planner for Humanoids

**Create smooth planner** (fewer turns):
```python
# Custom planner plugin
class HumanoidPlanner:
    def create_plan(self, start, goal):
        # Use Theta* or Smac Planner for smooth paths
        # Penalize sharp turns
        # Consider balance constraints
        pass
```

### Dynamic Obstacle Avoidance

**Configure for dynamic obstacles**:
```yaml
obstacle_layer:
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True  # Clear old obstacles
    marking: True   # Mark new obstacles
    obstacle_max_range: 5.0
```

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Path planning**: Capstone will use Nav2 for navigation
- **Obstacle avoidance**: Dynamic obstacle avoidance for safe navigation
- **Goal execution**: Navigate to manipulation locations
- **Recovery**: Handle stuck situations (rotate, backup)

Understanding Nav2 configuration now is essential for the capstone navigation system.

## Summary

You learned:
- ✅ Installed **Nav2 navigation stack** for ROS 2
- ✅ Configured **costmaps** (global and local) for humanoid constraints
- ✅ Set up **path planners** (A*, Theta*) for bipedal robots
- ✅ Implemented **obstacle avoidance** with humanoid-specific parameters
- ✅ Tested **navigation** in simulation and validated path planning

**Next steps**: In Chapter 3.5, you'll validate simulation algorithms on physical hardware and debug sim-to-real gaps.

---

## Exercises

### Exercise 1: Basic Nav2 Setup (Required)

**Objective**: Configure Nav2 and test basic navigation.

**Tasks**:
1. Install Nav2 packages
2. Create configuration files (costmaps, planner, controller)
3. Launch Nav2 stack
4. Set initial pose
5. Send navigation goal
6. Verify robot reaches goal

**Acceptance Criteria**:
- [ ] Nav2 launches without errors
- [ ] Costmaps visible in RViz2
- [ ] Global path planned successfully
- [ ] Robot executes path
- [ ] Goal reached within tolerance

**Estimated Time**: 120 minutes

### Exercise 2: Humanoid-Specific Tuning (Required)

**Objective**: Optimize Nav2 parameters for bipedal robot.

**Tasks**:
1. Measure humanoid turning radius
2. Configure wider footprint in costmap
3. Adjust velocity limits (slower for stability)
4. Test navigation with different parameters
5. Compare performance (path smoothness, time to goal)

**Metrics**:
- Path smoothness (number of turns)
- Time to reach goal
- Success rate (goal reached vs. failed)

**Estimated Time**: 90 minutes

### Exercise 3: Dynamic Obstacle Avoidance (Challenge)

**Objective**: Test Nav2 with moving obstacles.

**Tasks**:
1. Add moving objects to simulation
2. Configure dynamic obstacle layer
3. Test navigation with static obstacles
4. Test navigation with moving obstacles
5. Measure avoidance performance

**Requirements**:
- Moving obstacles in simulation
- Dynamic obstacle layer configured
- Performance metrics documented

**Estimated Time**: 180 minutes

---

## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/) - Official Nav2 guide
- [Nav2 Tutorials](https://navigation.ros.org/getting_started/index.html) - Step-by-step tutorials
- [Costmap 2D](https://navigation.ros.org/configuration/packages/costmap-plugins/costmap-plugins.html) - Costmap configuration
- [Nav2 GitHub](https://github.com/ros-planning/navigation2) - Source code

---

**Next**: [Chapter 3.5: Sim-to-Real Transfer Workflows →](chapter-3 to 5.md)
