---
sidebar_position: 1
title: Module 2 Introduction
---

# Module 2: The Digital Twin (Gazebo & Unity)

**Duration**: Weeks 6 to 7 (2 weeks)
**Focus**: Building realistic physics simulations and photorealistic environments for humanoid robots

## What You'll Build

By the end of this module, you will have created:
- **A complete Gazebo world** with physics, lighting, and obstacles
- **A humanoid robot model** (URDF/SDF) with sensors and actuators
- **Sensor plugins** for cameras, LiDAR, and IMUs publishing ROS 2 topics
- **A Unity photorealistic scene** integrated with ROS 2 for visual fidelity
- **Sim-to-real validation** workflows comparing simulation to physical hardware

**Module Project**: A Gazebo simulation of a humanoid robot navigating a cluttered environment using sensor data (camera + LiDAR) and executing manipulation tasks, with Unity rendering for photorealistic visualization.

## Module Overview

**Simulation is the foundation of modern robotics**. Before deploying code to expensive humanoid hardware (Unitree G1: $160K, Boston Dynamics Atlas: $2M+), engineers develop, test, and debug in simulation. Gazebo provides **physics-accurate simulation** for dynamics, collisions, and sensors. Unity adds **photorealistic rendering** for vision-based AI training and validation.

**Why simulation matters for Physical AI**:
- **Cost**: Test algorithms thousands of times without hardware wear
- **Safety**: Debug crashes, falls, and failures in simulation first
- **Speed**: Run simulations faster than real-time (10x, 100x speedup)
- **Reproducibility**: Exact same conditions for every test
- **Data Generation**: Generate millions of training images for vision models
- **Multi-Robot**: Test coordination of 10+ humanoids simultaneously

**Gazebo vs. Unity**:
- **Gazebo Classic/Fortress**: Physics engine (ODE/Bullet), sensor simulation, ROS 2 integration
- **Unity**: Photorealistic rendering, ray-traced lighting, synthetic data for ML training
- **Together**: Physics accuracy + visual realism = production-ready simulation

### Learning Path

**Chapter 2.1: Gazebo Fundamentals & Setup**
- Install Gazebo Classic/Fortress on Ubuntu 22.04
- Understand world files, models, and plugins
- Launch Gazebo with ROS 2 integration
- Create your first simulated world

**Chapter 2.2: Physics Simulation (Gravity, Collisions, Friction)**
- Configure physics engines (ODE, Bullet)
- Model gravity, friction coefficients, and collision properties
- Simulate humanoid walking dynamics
- Debug physics artifacts (penetration, jitter)

**Chapter 2.3: Sensor Simulation (LiDAR, Cameras, IMUs)**
- Add camera plugins publishing ROS 2 Image topics
- Configure LiDAR sensors for SLAM
- Simulate IMU data for balance control
- Calibrate sensor noise models

**Chapter 2.4: URDF/SDF Robot Description**
- Convert Module 1 URDF to SDF format
- Define joints, links, and collision geometries
- Add sensors to robot models
- Visualize robots in Gazebo

**Chapter 2.5: Unity Integration for Photorealistic Rendering**
- Set up Unity-ROS 2 bridge
- Import robot models into Unity
- Configure ray-traced lighting and materials
- Generate synthetic training data

## Tools & Technologies

You will use:
- **Gazebo Classic 11** or **Gazebo Fortress**: Physics simulation engine - [Install Guide](https://gazebosim.org/docs)
- **ROS 2 Humble**: Integration layer (from Module 1)
- **Unity 2022.3 LTS**: Photorealistic rendering - [Download](https://unity.com/download)
- **URDF/SDF**: Robot description formats
- **RViz2**: Visualization (from Module 1)

Installation guides provided in Chapter 2.1.

## Prerequisites

From Module 1 (Weeks 3 to 5):
- **ROS 2 Humble** installed and configured
- **URDF modeling** experience (joints, links, sensors)
- **Python with rclpy** for ROS 2 nodes
- **colcon** build system and launch files
- **Basic Linux** command line proficiency

**Don't worry if you're rusty**—we review key ROS 2 concepts as needed!

## Week-by-Week Timeline

**Week 6: Gazebo Fundamentals**
- Chapter 2.1: Gazebo Fundamentals & Setup
- Chapter 2.2: Physics Simulation (Gravity, Collisions, Friction)
- Chapter 2.3: Sensor Simulation (LiDAR, Cameras, IMUs)

**Week 7: Advanced Simulation & Module Project**
- Chapter 2.4: URDF/SDF Robot Description
- Chapter 2.5: Unity Integration for Photorealistic Rendering
- **Module 2 Project**: Humanoid navigation and manipulation in Gazebo

## Assessment (20% of final grade)

**Project**: Gazebo Humanoid Simulation with Sensor Integration

**Requirements**:
1. **Functional**:
   - Humanoid robot model (URDF/SDF) with 6+ joints
   - Gazebo world with obstacles and physics
   - Camera plugin publishing ROS 2 Image topics
   - LiDAR plugin publishing ROS 2 LaserScan topics
   - Robot navigates from start to goal using sensor data
   - Manipulation task (pick/place object) demonstrated

2. **Technical**:
   - Complete Gazebo world file (`.world`)
   - Robot SDF/URDF with sensors defined
   - ROS 2 launch file starting Gazebo + robot
   - Python node subscribing to sensor topics
   - Unity scene (optional) showing photorealistic rendering
   - README with setup and usage instructions

**Deliverables**:
- **GitHub Repository**:
  - `/worlds`: Gazebo world files
  - `/models`: Robot URDF/SDF models
  - `/launch`: ROS 2 launch files
  - `/src`: Python nodes for sensor processing
  - `/unity`: Unity project (if included)
  - `/README.md`: Documentation
  - `/demo_video.mp4`: Screen recording (3 to 5 minutes)

**Video Demo Must Show**:
1. Launching Gazebo with robot model
2. Robot sensors publishing ROS 2 topics (verify with `ros2 topic echo`)
3. Robot navigating through obstacles
4. Manipulation task execution
5. Unity rendering (if implemented)

**Grading Rubric**:

| Criterion | Excellent (90 to 100%) | Good (75 to 89%) | Needs Work (less than 75%) |
|-----------|---------------------|---------------|-------------------|
| **Functionality** | All sensors working, smooth navigation, manipulation successful | Minor bugs, most features working | Missing features, frequent errors |
| **Physics Accuracy** | Realistic dynamics, no penetration, stable simulation | Mostly realistic, minor artifacts | Unrealistic behavior, frequent crashes |
| **Code Quality** | Clean, well-documented, follows ROS 2/Gazebo conventions | Readable but sparse comments | Hard to understand, poor structure |
| **Documentation** | Complete setup guide, clear explanations, usage examples | Basic instructions, missing some details | Incomplete or confusing |
| **Demo** | Professional video, showcases all features, clear audio/visual | Shows main features, acceptable quality | Unclear or missing key features |

**Submission**:
Submit via course LMS by **end of Week 7**.
Late penalty: -10% per day (max 3 days late).

---

## Real-World Applications

**What you'll be able to build after this module**:

**Sim-to-Real Transfer**:
- Develop walking gaits in Gazebo, deploy to Unitree G1
- Train vision models on Unity synthetic data, test on real cameras
- Validate manipulation strategies before hardware deployment

**Multi-Robot Coordination**:
- Simulate warehouse with 10+ humanoids
- Test collision avoidance and task allocation
- Validate communication protocols

**Safety-Critical Testing**:
- Test emergency stop systems thousands of times
- Validate fall recovery algorithms
- Debug balance control without hardware damage

---

## Success Stories: What Students Built

**Week 6 Milestone**: First Gazebo world with physics, robot model loading successfully

**Week 7 Milestone**: Complete simulation with sensors, robot navigating and manipulating objects—ready for Isaac Sim integration in Module 3!

---

## Why Gazebo + Unity?

**Gazebo strengths**:
- Industry-standard physics (used by NASA, DARPA, Boston Dynamics)
- Native ROS 2 integration
- Accurate sensor models (noise, latency)
- Real-time capable

**Unity strengths**:
- Photorealistic rendering (ray tracing, global illumination)
- Massive asset library (environments, objects)
- ML training data generation
- Cross-platform (Windows, macOS, Linux)

**Together**: Physics accuracy + visual realism = production-ready simulation pipeline

---

## Getting Help

**Stuck on Gazebo errors?**
- Check **Chapter X.X Debugging Sections** (every chapter includes 3 to 4 common issues)
- [Gazebo Answers](https://answers.gazebosim.org/) - Community Q&A
- [Gazebo Documentation](https://gazebosim.org/docs) - Official reference
- **AI Book Assistant** (bottom-right corner) - Trained on this course content

**Office Hours**: See course schedule for TA support

---

## Ready to Start?

This module bridges ROS 2 (Module 1) with advanced simulation (Module 3: Isaac Sim). You'll build the digital twin that enables safe, fast, and reproducible robotics development.

**Let's build realistic simulations for humanoid robots.**

---

**Next**: [Chapter 2.1: Gazebo Fundamentals & Setup →](chapter-2 to 1.md)
