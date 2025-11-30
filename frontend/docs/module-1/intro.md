---
sidebar_position: 1
title: Module 1 Introduction
---

# Module 1: The Robotic Nervous System (ROS 2)

**Duration**: Weeks 3 to 5 (3 weeks)
**Focus**: Mastering the middleware that powers modern humanoid robots

## What You'll Build

By the end of this module, you will have created:
- **A complete ROS 2 workspace** with multiple packages for humanoid control
- **Publisher/subscriber nodes** for sensor data processing and motor commands
- **Service/client systems** for complex robot behaviors (pick, place, navigate)
- **A URDF humanoid model** with joints, links, and sensor definitions
- **Launch files** for coordinating multi-node robot systems

**Module Project**: A ROS 2 package that controls a simulated humanoid's joints using topic-based commands and visualizes the robot in RViz2.

## Module Overview

ROS 2 (Robot Operating System 2) is the **nervous system** of modern robotics. Just as your nervous system coordinates millions of neurons to enable movement, perception, and thought, ROS 2 coordinates software components (nodes) that control sensors, motors, planning, and perception.

**Why ROS 2 matters for Physical AI**:
- **Industry Standard**: Used by Boston Dynamics (Spot), Agility Robotics (Digit), Tesla (Optimus development), and 90% of robotics companies
- **Distributed Architecture**: Run perception on GPU, planning on CPU, control on real-time systems—all communicating seamlessly
- **Hardware Abstraction**: Write code once, deploy on TurtleBot, Unitree Go2, or custom humanoids without changes
- **Ecosystem**: 3,000+ packages for navigation (Nav2), manipulation (MoveIt 2), perception (perception_pcl), and more

Unlike ROS 1, ROS 2 is built for **production robotics**:
- Real-time capable (for safety-critical humanoid balance control)
- Multi-robot communication (coordinate humanoid teams)
- Security (DDS encryption for commercial deployments)
- Cross-platform (Linux, Windows, macOS, embedded systems)

### Learning Path

**Chapter 1.1: ROS 2 Architecture & Core Concepts**
- Understand nodes, topics, services, and actions
- Learn the publish/subscribe communication pattern
- Explore the DDS middleware layer
- Install ROS 2 Humble on Ubuntu 22.04

**Chapter 1.2: Python Integration with rclpy**
- Build publisher and subscriber nodes with \`rclpy\`
- Implement timers for periodic control loops
- Create service servers and clients for complex behaviors
- Handle callbacks and multi-threaded execution

**Chapter 1.3: URDF for Humanoid Robots**
- Define robot structure with URDF (Unified Robot Description Format)
- Model joints (revolute, prismatic, fixed) and links
- Add sensors (cameras, LiDAR, IMUs) to robot models
- Visualize humanoids in RViz2

**Chapter 1.4: Package Development & Launch Files**
- Create ROS 2 packages with \`colcon\`
- Structure code with Python modules and entry points
- Write launch files to coordinate multiple nodes
- Build and source workspaces

**Chapter 1.5: Parameter Management & Best Practices**
- Configure nodes with YAML parameter files
- Implement parameter callbacks for runtime changes
- Understand Quality of Service (QoS) profiles
- Explore lifecycle nodes for safety-critical systems

## Tools & Technologies

You will use:
- **ROS 2 Humble**: LTS release (Long-Term Support until 2027) - [Install Guide](https://docs.ros.org/en/humble/Installation.html)
- **Ubuntu 22.04 LTS**: Official ROS 2 Humble platform
- **Python 3.10+**: Primary programming language with \`rclpy\` client library
- **colcon**: Build system for ROS 2 packages
- **RViz2**: 3D visualization for robot models and sensor data
- **Gazebo Classic**: Robot simulator (covered in Module 2, introduced here)

Installation guides provided in Chapter 1.1.

## Prerequisites

From Weeks 1 to 2 (Foundations of Physical AI):
- **Python proficiency**: Functions, classes, decorators, async/await
- **Linux command line**: Navigation, file operations, environment variables
- **Basic robotics concepts**: Coordinate frames, transformations, sensors
- **Physics fundamentals**: Forces, torques, kinematics

**Don't worry if you're rusty**—we review key concepts as needed!

## Week-by-Week Timeline

**Week 3: ROS 2 Fundamentals**
- Chapter 1.1: ROS 2 Architecture & Core Concepts
- Chapter 1.2: Python Integration with rclpy

**Week 4: Robot Modeling**
- Chapter 1.3: URDF for Humanoid Robots
- Chapter 1.4: Package Development & Launch Files

**Week 5: Advanced ROS 2 & Module Project**
- Chapter 1.5: Parameter Management & Best Practices
- **Module 1 Project**: Humanoid joint controller with RViz2 visualization

## Assessment (20% of final grade)

**Project**: ROS 2 Humanoid Joint Controller

**Requirements**:
1. **Functional**:
   - Publish joint commands to control 6+ humanoid joints (arms, legs, torso)
   - Subscribe to joint state feedback and visualize in RViz2
   - Implement a service to execute pre-defined poses ("wave", "sit", "stand")
   - Use launch files to start all nodes with one command

2. **Technical**:
   - ROS 2 package with proper structure (\`package.xml\`, \`setup.py\`)
   - Python nodes using \`rclpy\` (no shell scripts)
   - URDF file defining humanoid structure
   - Launch file coordinating 3+ nodes
   - README with setup and usage instructions

**Deliverables**:
- **GitHub Repository**:
  - \`/src\`: Python nodes
  - \`/launch\`: Launch files
  - \`/urdf\`: Robot description
  - \`/config\`: Parameter YAML files
  - \`/README.md\`: Documentation
  - \`/demo_video.mp4\`: Screen recording (2 to 3 minutes)

**Video Demo Must Show**:
1. Running the launch file
2. Humanoid model in RViz2
3. Executing at least 3 different poses via service calls
4. Terminal output showing topic communication

**Grading Rubric**:

| Criterion | Excellent (90 to 100%) | Good (75 to 89%) | Needs Work (less than 75%) |
|-----------|---------------------|---------------|-------------------|
| **Functionality** | All joints move smoothly, services work flawlessly, RViz2 visualization perfect | Minor bugs, most features working | Missing features, frequent errors |
| **Code Quality** | Clean, well-documented, follows ROS 2 style guide | Readable but sparse comments | Hard to understand, poor structure |
| **Documentation** | Complete setup guide, clear explanations, usage examples | Basic instructions, missing some details | Incomplete or confusing |
| **Demo** | Professional video, showcases all features, clear audio/visual | Shows main features, acceptable quality | Unclear or missing key features |

**Submission**:
Submit via course LMS by **end of Week 5**.
Late penalty: -10% per day (max 3 days late).

---

## Real-World Applications

**What you'll be able to build after this module**:

**Humanoid Control Systems**:
- Joint-level control for walking, manipulation, and balance
- Sensor fusion (IMU + cameras + force sensors) through ROS 2 topics
- Emergency stop systems using service calls

**Multi-Robot Coordination**:
- Deploy multiple humanoids working together in a warehouse
- Share perception data (detected objects, people) between robots
- Centralized task allocation via ROS 2 services

**Sim-to-Real Transfer**:
- Develop in Gazebo simulation (Module 2)
- Deploy identical code to physical Unitree G1 or Boston Dynamics Spot
- Use ROS 2's hardware abstraction to switch between simulation and reality

---

## Success Stories: What Students Built

**Week 3 Milestone**: First publisher/subscriber nodes exchanging sensor data

**Week 4 Milestone**: Custom humanoid URDF visualized in RViz2 with moving joints

**Week 5 Milestone**: Complete package controlling simulated humanoid—ready for Gazebo integration in Module 2!

---

## Why ROS 2 Over ROS 1?

If you've used ROS 1 (Melodic, Noetic), here's what's different:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Middleware** | Custom TCP/UDP | DDS (industry standard) |
| **Real-time** | Not supported | Real-time capable |
| **Security** | None | DDS encryption |
| **Multi-robot** | Difficult | Native support |
| **Lifecycle** | Basic | Advanced lifecycle nodes |
| **Build System** | catkin | colcon |
| **Python Client** | rospy | rclpy |

**Bottom line**: ROS 2 is production-ready for commercial humanoid robots. ROS 1 is legacy (end-of-life in 2025).

---

## Getting Help

**Stuck on ROS 2 errors?**
- Check **Chapter X.X Debugging Sections** (every chapter includes 3 to 4 common issues)
- [ROS 2 Answers](https://answers.ros.org/) - Community Q&A
- [ROS 2 Documentation](https://docs.ros.org/en/humble/) - Official reference
- **AI Book Assistant** (bottom-right corner) - Trained on this course content

**Office Hours**: See course schedule for TA support

---

## Ready to Start?

This module is hands-on and code-heavy. You'll spend more time in the terminal than reading theory. By Week 5, you'll have a portfolio-worthy ROS 2 project demonstrating skills that robotics companies actively seek.

**Let's build the nervous system for humanoid robots.**

---

**Next**: [Chapter 1.1: ROS 2 Architecture & Core Concepts →](chapter-1 to 1.md)
