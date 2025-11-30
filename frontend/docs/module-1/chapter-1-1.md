---
sidebar_position: 2
title: 1.1 ROS 2 Architecture & Core Concepts
---

# Chapter 1.1: ROS 2 Architecture & Core Concepts

ROS 2 is the middleware layer that enables humanoid robots to coordinate hundreds of sensors, motors, and algorithms in real-time. Understanding its architecture is fundamental to building autonomous Physical AI systems.

## Learning Outcomes

By the end of this chapter, you will be able to:

- **Understand** the ROS 2 computational graph (nodes, topics, services, actions)
- **Install** ROS 2 Humble on Ubuntu 22.04 and configure the environment
- **Implement** basic publisher and subscriber nodes using command-line tools
- **Debug** communication between nodes using ROS 2 introspection tools
- **Explain** how DDS middleware enables distributed robot systems

## Prerequisites

- **Ubuntu 22.04 LTS** installed (native or VM with 4GB+ RAM)
- **Basic Linux terminal skills**: cd, ls, sudo, nano/vim
- **Python 3.10+** (comes with Ubuntu 22.04)
- **Git** for version control

## Part 1: ROS 2 Architecture Fundamentals

### The ROS 2 Computational Graph

ROS 2 applications consist of **nodes** communicating through three primary patterns:

| Communication Pattern | Use Case | Example |
|-----------------------|----------|---------|
| **Topics** (Pub/Sub) | Continuous sensor data streaming | Camera publishes images at 30 Hz |
| **Services** (Request/Reply) | One-time computations | "What's the current battery level?" |
| **Actions** (Goal-Based) | Long-running tasks with feedback | "Navigate to (x, y) and report progress" |

### Key Concepts

#### 1. Nodes
A **node** is a single executable process performing a specific task.

**Example nodes in a humanoid robot**:
- \`/camera_driver\`: Publishes RGB images
- \`/object_detector\`: Subscribes to images, publishes detected objects
- \`/motion_planner\`: Plans joint trajectories to grasp objects
- \`/joint_controller\`: Commands motors based on planned trajectories

**Why multiple nodes?**
- **Modularity**: Replace object_detector with a better model without touching other code
- **Fault isolation**: If camera_driver crashes, planner keeps running
- **Distributed processing**: Run camera_driver on Jetson, planner on workstation

#### 2. Topics (Publish/Subscribe)
**Topics** are named buses where nodes publish and subscribe to messages.

**Key Properties**:
- **Many-to-many**: Multiple publishers and subscribers per topic
- **Asynchronous**: Publishers don't wait for subscribers
- **Typed**: Each topic has a message type (e.g., \`sensor_msgs/Image\`)

**Example**:
```
Topic: /camera/image_raw
Type: sensor_msgs/msg/Image
Publishers: /camera_driver
Subscribers: /object_detector, /face_recognizer, /slam_node
```

#### 3. Services (Request/Reply)
**Services** provide synchronous request/reply interactions.

**Example**:
```
Service: /get_battery_level
Type: std_srvs/srv/Trigger
Server: /battery_monitor
Clients: /mission_planner, /safety_monitor
```

**Use case**: Mission planner calls service before starting a task: "Do we have enough battery for this 5-minute navigation?"

#### 4. Actions (Goal-Based)
**Actions** are for long-running tasks requiring feedback and cancellation.

**Example**:
```
Action: /navigate_to_pose
Type: nav2_msgs/action/NavigateToPose
Server: /nav2_controller
Client: /mission_planner
```

**Workflow**:
1. Client sends goal: "Navigate to (x=5.0, y=3.2)"
2. Server sends feedback: "60% complete, ETA 12 seconds"
3. Client can cancel if needed
4. Server sends result: "Goal reached" or "Failed: obstacle"

### DDS Middleware Layer

ROS 2 uses **DDS (Data Distribution Service)** as its communication layer.

**Benefits for humanoid robots**:
- **QoS (Quality of Service)**: Configure reliability/latency tradeoffs
  - Camera images: "Best effort" (OK to drop frames for low latency)
  - Safety commands: "Reliable" (never drop emergency stop messages)
- **Discovery**: Nodes automatically find each other on the network
- **Security**: Encrypt communication (critical for commercial robots)
- **Multi-robot**: Multiple humanoids share data on the same network

## Part 2: Hands-On Tutorial

### Project: Install ROS 2 and Communicate Between Nodes

**Goal**: Install ROS 2 Humble, create publisher and subscriber nodes using CLI tools, and verify communication.

**Tools**: Ubuntu 22.04, ROS 2 Humble

### Step 1: Install ROS 2 Humble

```bash
# Ensure UTF-8 locale
locale  # check if UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes RViz2, demos, tutorials)
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

**Installation time**: ~10 to 15 minutes (downloads ~1GB)

### Step 2: Configure Environment

```bash
# Source ROS 2 setup file (add to ~/.bashrc to make permanent)
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --help

# You should see:
# usage: ros2 [-h] [--use-python-default-buffering] Call \`ros2 <command> -h\` for more detailed usage. ...
```

**Add to \`~/.bashrc\` for automatic sourcing**:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Test with Example Nodes

**Terminal 1: Run a publisher (talker node)**:
```bash
ros2 run demo_nodes_cpp talker
```

**Expected Output**:
```
[INFO] [1698765432.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1698765433.123456789] [talker]: Publishing: 'Hello World: 2'
[INFO] [1698765434.123456789] [talker]: Publishing: 'Hello World: 3'
```

**Terminal 2: Run a subscriber (listener node)**:
```bash
ros2 run demo_nodes_py listener
```

**Expected Output**:
```
[INFO] [1698765434.234567890] [listener]: I heard: 'Hello World: 3'
[INFO] [1698765435.234567890] [listener]: I heard: 'Hello World: 4'
```

**What's happening?**
1. \`talker\` publishes messages to topic \`/chatter\` every 1 second
2. \`listener\` subscribes to \`/chatter\` and prints received messages
3. DDS middleware automatically discovers nodes and routes messages

### Step 4: Introspection Tools

**List all active nodes**:
```bash
ros2 node list
```
Output:
```
/talker
/listener
```

**List all topics**:
```bash
ros2 topic list
```
Output:
```
/chatter
/parameter_events
/rosout
```

**Get topic info**:
```bash
ros2 topic info /chatter
```
Output:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

**Echo topic messages** (see live data):
```bash
ros2 topic echo /chatter
```
Output:
```
data: 'Hello World: 10'
---
data: 'Hello World: 11'
---
```

**Check message type definition**:
```bash
ros2 interface show std_msgs/msg/String
```
Output:
```
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data
```

**Publish from command line** (no code needed):
```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from terminal'"
```

### Step 5: Visualize with RQt Graph

RQt provides a graphical view of the ROS 2 computational graph.

```bash
# Install rqt (if not already installed)
sudo apt install ros-humble-rqt*

# Run rqt_graph
rqt_graph
```

**What you'll see**:
- Nodes as ovals (\`/talker\`, \`/listener\`)
- Topics as rectangles (\`/chatter\`)
- Arrows showing publish/subscribe relationships

### Step 6: Debugging Common Issues

#### Issue 1: "ros2: command not found"
**Cause**: ROS 2 setup file not sourced

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Or add to ~/.bashrc:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Issue 2: "Package 'demo_nodes_cpp' not found"
**Cause**: Desktop install missing or incomplete

**Solution**:
```bash
sudo apt install ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py
```

#### Issue 3: Nodes running but not communicating
**Cause**: DDS discovery issues or different ROS_DOMAIN_ID

**Solution**:
```bash
# Check domain ID (default is 0)
echo $ROS_DOMAIN_ID

# If using multiple ROS 2 systems, set unique domain IDs
export ROS_DOMAIN_ID=42  # Use same ID for all nodes that should communicate
```

#### Issue 4: "Unable to initialize transports" error
**Cause**: Firewall blocking DDS multicast

**Solution**:
```bash
# Disable firewall (temporary, for development only)
sudo ufw disable

# Or add rule to allow ROS 2 traffic
sudo ufw allow from 224.0.0.0/4
```

## Part 3: Advanced Topics (Optional)

### Quality of Service (QoS) Profiles

ROS 2 allows fine-grained control over message delivery guarantees.

**Common QoS profiles**:
- **Reliable**: Guaranteed delivery (for safety commands)
- **Best Effort**: Drop old messages (for sensor streams)
- **Transient Local**: Late joiners receive last message
- **Volatile**: Late joiners get nothing

**Example use case**:
- Camera publishes at 30 Hz with "Best Effort" (low latency, OK to drop frames)
- Emergency stop uses "Reliable" (never drop critical commands)

**Set QoS via CLI**:
```bash
ros2 topic pub --qos-reliability reliable /emergency_stop std_msgs/msg/Bool "data: true"
```

### ROS 2 vs. ROS 1 Architecture

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| **Master** | Required (single point of failure) | None (peer-to-peer discovery) |
| **Middleware** | Custom (TCP/UDP) | DDS (industry standard) |
| **Language Support** | C++, Python | C++, Python, Rust, Ada, more |
| **Real-time** | Not supported | Real-time capable with DDS-RT |
| **Security** | None | DDS encryption & authentication |

## Integration with Capstone

**How this chapter contributes** to the Week 13 autonomous humanoid:

- **Node architecture**: The capstone will have 10+ nodes:
  - \`/whisper_listener\` (voice input)
  - \`/llm_planner\` (task decomposition)
  - \`/nav2_controller\` (navigation)
  - \`/object_detector\` (vision)
  - \`/gripper_controller\` (manipulation)
  
- **Topic communication**: Nodes exchange data asynchronously:
  - Voice → LLM → Navigation goals → Motor commands
  
- **Service calls**: "Get current robot pose", "Check if object reachable"
  
- **Actions**: Long-running navigation and manipulation tasks with feedback

Understanding topics, services, and actions now is essential for orchestrating complex multi-node systems.

## Summary

You learned:
- ✅ ROS 2 uses **nodes** communicating via **topics**, **services**, and **actions**
- ✅ **DDS middleware** enables distributed, real-time robot systems
- ✅ Installed **ROS 2 Humble** on Ubuntu 22.04
- ✅ Ran example nodes and verified communication with introspection tools
- ✅ Debugged common installation and communication issues

**Next steps**: In Chapter 1.2, you'll write custom publisher/subscriber nodes in Python using \`rclpy\`.

---

## Exercises

### Exercise 1: Multi-Publisher Experiment (Required)

**Objective**: Understand many-to-many topic communication.

**Tasks**:
1. Open **three terminals**
2. Terminal 1: Run \`ros2 run demo_nodes_cpp talker\`
3. Terminal 2: Run \`ros2 run demo_nodes_py talker\` (yes, same topic!)
4. Terminal 3: Run \`ros2 topic echo /chatter\`

**Questions**:
- How many messages per second do you see?
- Run \`ros2 topic info /chatter\` — how many publishers are listed?
- Stop one talker — what happens to the message rate?

**Acceptance Criteria**:
- [ ] Successfully ran multiple publishers on same topic
- [ ] Observed interleaved messages from both publishers
- [ ] Explained behavior in 2 to 3 sentences

**Estimated Time**: 15 minutes

### Exercise 2: Custom Topic Publishing (Required)

**Objective**: Publish data to a topic using the command line.

**Tasks**:
1. Create a new topic \`/robot/joint_position\` of type \`std_msgs/msg/Float64\`
2. Publish a value (e.g., 1.57 radians for 90 degrees)
3. Echo the topic in another terminal to verify

**Commands**:
```bash
# Terminal 1: Publish
ros2 topic pub /robot/joint_position std_msgs/msg/Float64 "data: 1.57" --rate 1

# Terminal 2: Echo
ros2 topic echo /robot/joint_position
```

**Acceptance Criteria**:
- [ ] Topic publishes at 1 Hz
- [ ] Echo shows correct value
- [ ] Explained what \`--rate 1\` does

**Estimated Time**: 20 minutes

### Exercise 3: Investigate Message Types (Challenge)

**Objective**: Explore ROS 2 message types for sensors.

**Tasks**:
1. Find the message type for **IMU data** (Inertial Measurement Unit)
2. Display the message definition
3. Identify which fields store **angular velocity** and **linear acceleration**

**Hints**:
```bash
# List all message types
ros2 interface list | grep Imu

# Show message definition
ros2 interface show sensor_msgs/msg/Imu
```

**Questions**:
- What is the full message type name?
- Which coordinate frame is the IMU data expressed in?
- Why does the message have \`orientation_covariance\` fields?

**Estimated Time**: 30 minutes

---

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) - Official reference
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html) - Detailed architecture guide
- [DDS Specification](https://www.omg.org/spec/DDS/) - Underlying middleware protocol
- [ROS 2 Design](https://design.ros2.org/) - Why ROS 2 was redesigned from ROS 1
- [QoS Policies](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html) - Reliability and durability settings

---

**Next**: [Chapter 1.2: Python Integration with rclpy →](chapter-1 to 2.md)
