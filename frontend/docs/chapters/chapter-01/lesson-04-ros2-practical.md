---
id: lesson-04-ros2-practical
title: "Lesson 4: ROS 2 Practical Implementation"
---

## 1.3.4: Building ROS 2 Packages with Python

A ROS 2 **package** is a directory containing your nodes, launch files, and configuration.

**Package Structure:**

```
learning_ros2/
├── setup.py              (Package metadata and dependencies)
├── setup.cfg             (Configuration)
├── resource/             (Resources)
├── learning_ros2/        (Python source code)
│   └── __init__.py
├── launch/               (Launch files)
│   └── hello_world.launch.yaml
├── nodes/                (Node executables)
│   ├── publisher_node.py
│   └── subscriber_node.py
└── urdf/                 (Robot descriptions)
    └── robot.urdf
```

**Launch Files** orchestrate multiple nodes with parameters. This YAML launch file starts a publisher and subscriber together:

```yaml
version: 2

launch:
  # Publisher node
  - node:
      package: "learning_ros2"
      executable: "publisher_node"
      name: "publisher"
      output: "screen"
      parameters:
        - publish_frequency: 1.0

  # Subscriber node
  - node:
      package: "learning_ros2"
      executable: "subscriber_node"
      name: "subscriber"
      output: "screen"
```

**Running from a Launch File:**

```bash
ros2 launch learning_ros2 hello_world.launch.yaml
```

### 1.3.5: URDF: Robot Structure Description

**URDF (Unified Robot Description Format)** is an XML format for describing robot structure: links (rigid bodies), joints (connections between links), and geometry (visual appearance and collision shapes).

A complete working URDF example is provided in Code Example 4 (the 2-link arm). A URDF file contains:

**Links** represent rigid bodies with properties like mass, inertia (moment of inertia), visual appearance (geometry and color), and collision shape for physics simulation.

**Joints** connect links and define how they can move relative to each other. Revolute joints allow rotation around an axis (like a hinge), while prismatic joints allow sliding motion. Joints specify:
- Parent and child links they connect
- The axis of motion (X, Y, or Z)
- Motion limits (minimum and maximum angles or distances)
- Dynamics properties like damping and friction

### 1.3.6: Launch Files and Multi-Node Systems

A complete ROS 2 system typically requires multiple coordinating nodes. A **launch file** is a script that:
1. Starts multiple nodes
2. Loads parameters for each node
3. Remaps topic names
4. Coordinates startup and shutdown

This approach enables complex systems to be started with a single command rather than manually launching each node in separate terminals.
