---
id: lesson-02-locomotion
title: "Lesson 2: Bipedal Locomotion Control"
---

## 3.2: Bipedal Locomotion

Walking on two legs requires dynamic balance, precise timing, and closed-loop control. Unlike quadrupeds, bipedal robots have a smaller base of support and must actively prevent tipping.

### 3.2.1: Zero Moment Point (ZMP) Theory

The **Zero Moment Point (ZMP)** is the point on the ground where the net moment (torque) is zero. For stable walking:

$$\text{ZMP must remain within the support polygon (feet contact area)}$$

If ZMP exits the foot boundary, the robot will tip over. ZMP location relates to the robot's center of mass (COM):

$$x_{ZMP} = x_{COM} - \frac{z_{COM}}{g} \ddot{x}_{COM}$$

where $z_{COM}$ is COM height and $g$ is gravity.

### 3.2.2: Inverted Pendulum Model

The **linear inverted pendulum (LIP) model** simplifies walking dynamics. At constant COM height $z_{COM} = \bar{z}$:

$$\ddot{x}_{COM} = \frac{g}{\bar{z}}(x_{COM} - x_{ZMP})$$

This decouples the vertical and horizontal dynamics, enabling efficient trajectory planning.

### 3.2.3: Gait Patterns

Different walking speeds use different gait patterns:

| Gait | Speed Range | Characteristics |
|------|-------------|-----------------|
| Static Walk | 0.3-0.5 m/s | Both feet on ground; COM always within support |
| Dynamic Walk | 0.5-1.5 m/s | Single support phase; ZMP trajectory optimized |
| Running | >1.5 m/s | Flight phase; both feet leave ground |

Each gait requires different feedback control and constraint handling.

### 3.2.4: Balance Recovery

When ZMP approaches the edge, the robot must recover by:

1. **Step placement**: Move foot to expand support polygon
2. **COM adjustment**: Shift center of mass back toward stability
3. **Ankle torque**: Apply corrective torque through ankle actuators

Real-time implementations use fast reaction times (~100 ms) to prevent falling.

### 3.2.5: Implementation Challenges

- **Sensor latency**: 50-100 ms delay in IMU and force feedback
- **Model uncertainty**: Friction, actuator limits, shoe deformation
- **Real-time constraints**: Must run control loops at 100-1000 Hz
- **Energy efficiency**: Minimize power consumption for long operation
