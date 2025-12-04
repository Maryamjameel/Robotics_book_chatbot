---
id: lesson-01-gazebo
title: "Lesson 1: Physics Simulation with Gazebo"
---

## 2.1: Physics Simulation Foundations: Gazebo

**Gazebo** is an open-source 3D robot simulator that integrates with ROS 2. It provides realistic physics simulation, sensor simulation with configurable noise models, and plugin-based extensibility.

### 2.1.1: Gazebo Architecture

Gazebo consists of three primary components:

**Server (gzserver)**: Runs physics simulation, manages world state, updates sensors, and processes plugin computations. The server runs independently of visualization.

**Client (gzclient)**: Provides a 3D graphical interface for visualization and interactive control. Multiple clients can connect to a single server.

**Plugins**: Extend Gazebo by implementing custom sensors, controllers, and physics behaviors.

The physics engine options include:
- **ODE (Open Dynamics Engine)**: Accurate, widely used, stable for many robotic systems
- **Bullet Physics**: Good collision detection, faster computation
- **DART**: Supports complex articulated systems, contact-rich dynamics

### 2.1.2-2.1.5: Simulation Configuration and Physics

Gazebo uses SDF (Simulation Description Format) for world configuration, includes realistic sensor noise models, integrates with ROS 2 via plugins, and implements rigid body dynamics with numerical integration at typical 1000 Hz frequency.

Key features:
- Camera, depth, and LiDAR sensor simulation
- Configurable noise and distortion models
- Direct ROS 2 topic integration via gazebo_ros plugins
- Accurate physics with gravity, friction, and contact dynamics
