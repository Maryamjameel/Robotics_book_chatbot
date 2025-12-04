---
id: lesson-01-kinematics
title: "Lesson 1: Humanoid Arm Kinematics"
---

## 3.1: Humanoid Arm Kinematics

Robotic manipulators like humanoid arms must precisely position and orient their end-effectors (hands) in 3D space. **Kinematics** is the mathematical framework for understanding robot motion without considering forces or dynamics.

### 3.1.1: Denavit-Hartenberg Parameters

The **Denavit-Hartenberg (DH) convention** provides a systematic way to describe robot geometry using 4 parameters per joint:

- **a** (link length): Distance along joint axis
- **α** (link twist): Rotation around joint axis
- **d** (joint offset): Distance along previous frame
- **θ** (joint angle): Rotation angle (variable for revolute joints)

These parameters form transformation matrices that describe the spatial relationship between robot joints.

### 3.1.2: Forward Kinematics

**Forward kinematics** computes the end-effector position and orientation from joint angles. For a 7-DOF manipulator:

$$\mathbf{T}_0^7 = \mathbf{A}_1(\theta_1) \mathbf{A}_2(\theta_2) \cdots \mathbf{A}_7(\theta_7)$$

where each $\mathbf{A}_i$ is a 4×4 transformation matrix.

### 3.1.3: Jacobian Matrix

The **Jacobian** relates joint velocities to end-effector velocities:

$$\mathbf{v} = \mathbf{J}(\theta) \dot{\theta}$$

where:
- $\mathbf{v}$ is the end-effector linear and angular velocity (6×1)
- $\mathbf{J}$ is the Jacobian matrix (6×7)
- $\dot{\theta}$ is joint angular velocities (7×1)

The Jacobian is essential for inverse kinematics, singularity analysis, and force control.

### 3.1.4: Inverse Kinematics

**Inverse kinematics (IK)** solves the inverse problem: given desired end-effector pose, find joint angles. For redundant manipulators (more than 6 DOF), infinite solutions exist.

**Methods:**
- **Analytical IK**: Closed-form solutions (fast, but limited to specific geometries)
- **Numerical IK**: Iterative methods like damped least-squares (DLS) or pseudo-inverse

**DLS Algorithm:**

$$\Delta\theta = \mathbf{J}^T(\mathbf{J}\mathbf{J}^T + \lambda^2\mathbf{I})^{-1} \mathbf{e}$$

where:
- $\mathbf{e}$ is the pose error (position and orientation difference)
- $\lambda$ is the damping factor (prevents singularities)
- The pseudo-inverse $\mathbf{J}^\dagger$ includes null-space projection for redundancy exploitation

### 3.1.5: Workspace Analysis

The **workspace** is the set of poses reachable by the end-effector. For humanoid arms with 7 DOF and typical dimensions:
- Reachable workspace: ~1.5 meters (including torso mobility)
- Dexterous workspace: ~0.8 meters (can approach from multiple directions)
- Singularities: Certain configurations lose DOF due to colinear joints

Understanding workspace limits is critical for task planning and ensuring physical feasibility of grasping and manipulation tasks.
