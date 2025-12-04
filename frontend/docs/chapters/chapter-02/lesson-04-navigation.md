---
id: lesson-04-navigation
title: "Lesson 4: Autonomous Navigation with Nav2"
---

## 2.5: Autonomous Navigation with Nav2

**Nav2 (Navigation 2)** is the standard ROS 2 navigation stack combining perception, planning, and control for autonomous navigation.

### 2.5.1: Nav2 Architecture

Nav2 consists of:

**Costmap Layers**: Represent navigable space
- Static layer: Known obstacles and walls
- Obstacle layer: Dynamic obstacles from sensors
- Inflation layer: Expand obstacles for clearance

**Path Planner**: Generates collision-free paths
- Global planner: Complete path from start to goal
- Local planner: Real-time obstacle avoidance

**Behavior Trees**: Coordinate navigation behaviors
- Execute sequences: Move to goal, stop, recover
- Reusable behaviors: Enable complex missions

### 2.5.2: Behavior Trees for Robot Control

Behavior trees provide hierarchical task decomposition:
- **Composites**: Sequence, selector, parallel
- **Decorators**: Modify child behavior (repeat, timeout)
- **Actions**: Primitive behaviors (move, turn, grasp)
- **Conditions**: Check state (goal reached, obstacle detected)

Trees enable structured, reusable robot control more intuitive than state machines.

### 2.5.3-2.5.4: Nav2 Parameters and Configuration

Nav2 requires tuning:
- Costmap parameters (resolution, extent, update frequency)
- Planner parameters (tolerance, grid alignment)
- Controller parameters (look-ahead distance, max velocity)
- Behavior tree structure

Configuration files define these parameters, enabling rapid experimentation without code changes.
