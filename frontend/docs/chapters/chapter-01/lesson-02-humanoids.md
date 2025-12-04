---
id: lesson-02-humanoids
title: "Lesson 2: The Humanoid Robotics Landscape"
---

## 1.2: The Humanoid Robotics Landscape

### 1.2.1: Current State of Humanoid Robotics

The field of humanoid robotics is entering rapid development. As of late 2024, several systems stand out:

**Boston Dynamics Atlas**

Boston Dynamics Atlas represents the current state-of-the-art in dynamic humanoid robotics:

- **Height**: Approximately 1.7 meters (human-height)
- **Weight**: Approximately 80 kg
- **Degrees of Freedom**: 28 (6 in each arm, 6 in each leg, others in torso and head)
- **Sensors**: Multiple LIDAR units, stereo cameras, inertial measurement units, force-torque sensors
- **Key Achievements**: Dynamic parkour (running, jumping, backflips), complex manipulation, obstacle navigation
- **Status**: Research platform; not commercially available
- **Notable Capability**: Can navigate stairs, climb ladders, and perform acrobatic feats

Atlas demonstrates that bipedal humanoid systems can achieve dynamic, athletic performance.

**Tesla Humanoid (Optimus)**

Tesla has committed significant resources to developing a humanoid robot:

- **Height**: Approximately 1.73 meters (designed to match average human height)
- **Weight**: Target approximately 57 kg
- **Degrees of Freedom**: 40+ (more dexterous hands than Atlas)
- **Actuators**: Tesla-designed actuators optimized for repetitive manufacturing tasks
- **Development Status**: Prototype demonstrations as of 2024
- **Key Differentiator**: Manufacturing focus and cost optimization
- **Applications**: Manufacturing, repetitive tasks, consumer/home use
- **Design Philosophy**: Bipedal form optimized for human environments

Optimus represents the commercial ambition for humanoids.

**Unitree G1**

Unitree Robotics has released the G1, a commercial humanoid robot:

- **Height**: Approximately 1.6 meters
- **Weight**: Approximately 35 kg
- **Degrees of Freedom**: 23 (fewer than Atlas but sufficient for many tasks)
- **Current Status**: Commercial product available as of 2024
- **Price Range**: Approximately $150,000-$250,000 USD
- **Accessibility**: Marketed to universities, research institutions, and robotics companies
- **Availability**: International shipping available; several universities worldwide have deployed G1

The G1 represents a middle ground: more capable than low-cost research platforms, more affordable than fully custom systems.

### 1.2.2: Sensor Systems

Modern humanoid robots employ multiple complementary sensor types:

**Cameras and Vision Systems**
- **RGB Cameras**: Standard color images for object recognition, navigation, manipulation
- **Depth Cameras (RGB-D)**: Combine color with depth information for 3D understanding
- **Stereo Vision**: Two cameras providing depth through triangulation, enabling 3D scene reconstruction
- **LIDAR**: Laser scanning for precise 3D environment mapping and obstacle detection

**Inertial Measurement Units (IMUs)**
- **Accelerometers**: Measure linear acceleration (including gravity; 9.81 m/s² downward at rest)
- **Gyroscopes**: Measure angular velocity (rotation rates around 3 axes)
- **Magnetometers**: Optional; measure magnetic field (enables compass-like orientation)
- **Output**: 9D measurement (3D acceleration, 3D angular velocity, optional 3D magnetic field)

**Force-Torque Sensors (6-Axis Sensors)**

Force-torque sensors measure forces and torques at a point:

- **Output**: 6D measurement: 3 force axes (Fx, Fy, Fz) and 3 torque axes (τx, τy, τz)
- **Integration points**: Often placed in wrist (to measure hand contact forces), ankle (to measure ground reaction forces), or spine (to measure load)
- **Frequency**: 100-1000 Hz (high frequency enables real-time force feedback control)
- **Strengths**: Direct measurement of interaction forces, enables tactile feedback control
- **Limitations**: Cost, mechanical complexity, requires careful mounting

**Joint Encoders and Proprioception**
- **Position Encoders**: Measure joint angles (how much the joint has rotated)
- **Velocity Estimation**: Measure rate of change of joint angles (angular velocity)
- **Critical for**: Knowing robot configuration, inverse kinematics, feedback control

### 1.2.3: Degrees of Freedom and Applications

The number of **degrees of freedom (DOF)** determines what tasks a robot can perform.

**DOF Analysis:**

For a robotic manipulator (robot arm), DOF is roughly the number of joints. A 3-DOF arm can reach points in 3D space but with constraints. A 6-DOF arm can reach any 3D point with any orientation—six degrees of freedom gives control of position (x, y, z) and orientation (roll, pitch, yaw).

For humanoid robots, total DOF includes:
- **Positioning**: 3 DOF to reach the object (x, y, z)
- **Orientation**: 3 DOF to orient the tool/gripper (roll, pitch, yaw)
- **Mobility**: Additional DOF for the body/legs to approach the object

This is why humanoid robots like Atlas (28 DOF) can perform complex tasks: they have sufficient degrees of freedom for manipulation (arms), locomotion (legs), and body reconfiguration (torso).

**Tradeoffs:**
- **More DOF**: Greater task flexibility, better adaptability, more complex control
- **Fewer DOF**: Simpler control, lower cost, faster computation, better energy efficiency
