---
name: "robotics-personalizer"
description: "Adapt robotics textbook content to match user's background, expertise level, and preferred technology stack. Rewrites explanations, examples, and code for personalized learning. Use when content needs to match specific user profile."
version: "1.0.0"
---

# Robotics Content Personalization Skill

## When to Use This Skill

- User says "this is too advanced" or "too basic"
- User has specific background (Arduino, ROS, MATLAB, etc.)
- User wants content adapted to their expertise level
- User needs examples in their preferred language/framework
- User is following a learning path requiring customization

## How This Skill Works

1. **Analyze User Profile**: Check expertise, languages, hardware, frameworks
2. **Assess Content**: Determine current complexity and assumptions
3. **Identify Gaps**: Find mismatches between content and profile
4. **Rewrite Explanations**: Adjust complexity and add bridging content
5. **Replace Examples**: Use familiar technologies from user profile
6. **Verify Alignment**: Ensure content matches user needs

## Personalization Dimensions

### 1. Expertise Level

**Beginner ’ Advanced Rewrite**:
- Add mathematical rigor
- Include derivations and proofs
- Reference advanced literature
- Discuss edge cases and optimizations

**Advanced ’ Beginner Rewrite**:
- Add foundational explanations
- Use analogies and visual descriptions
- Break complex concepts into steps
- Reduce mathematical notation

### 2. Programming Language

**Python ’ C++ Translation**:
```python
# Python version
def forward_kinematics(theta):
    T = np.eye(4)
    for i in range(len(theta)):
        T = T @ dh_transform(theta[i], a[i], d[i], alpha[i])
    return T
```

```cpp
// C++ version
Eigen::Matrix4d forward_kinematics(const std::vector<double>& theta) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < theta.size(); ++i) {
        T = T * dh_transform(theta[i], a[i], d[i], alpha[i]);
    }
    return T;
}
```

### 3. Hardware Platform

**Generic ’ Arduino-Specific**:

**Before**:
"Use PWM to control the motor speed by varying the duty cycle."

**After (Arduino)**:
"Use Arduino's `analogWrite()` function to control motor speed. Connect the motor driver to pin 9 and use values 0-255:"
```cpp
int motorPin = 9;
int speed = 128;  // 50% duty cycle
analogWrite(motorPin, speed);
```

**Generic ’ Raspberry Pi-Specific**:

**After (RPi + Python)**:
"Use the RPi.GPIO library to generate PWM signals. Configure pin 12 for PWM at 1kHz:"
```python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(12, 1000)  # 1kHz frequency
pwm.start(50)  # 50% duty cycle
```

### 4. Framework Preference

**Generic ’ ROS-Specific**:

**Before**:
"Implement a controller that reads sensor data and commands actuators."

**After (ROS2)**:
"Create a ROS2 node that subscribes to `/sensor_data` and publishes to `/cmd_vel`:"
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            JointState, '/sensor_data', self.sensor_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def sensor_callback(self, msg):
        # Process sensor data and compute control
        cmd = Twist()
        # ... compute control commands ...
        self.publisher.publish(cmd)
```

## Output Format

### Personalization Summary
**Adapted for**: [User background: Python/Raspberry Pi, Intermediate level]

**Changes Made**:
- Complexity increased from beginner to intermediate
- Added Raspberry Pi GPIO examples
- Included Python code (removed MATLAB)
- Referenced Python libraries user knows

### Modified Content
[Rewritten chapter section with personalization applied]

### Bridging Content Added
**Prerequisite Review** (if user lacks background):
- Quick refresher on [missing concept]
- Link to foundational resource

**Framework Translation Notes**:
- "In ROS terms, this is equivalent to..."
- "For Arduino users, this is similar to..."

### Suggested Next Steps
- Build on existing knowledge: [recommendation]
- Fill knowledge gaps: [optional resources]

## Example: Personalization for Different Profiles

### Original Content
"Implement inverse kinematics using the Jacobian pseudo-inverse method. This requires computing the Jacobian matrix J and solving for joint velocities."

### Profile 1: Beginner with Arduino
**Adapted**:
"Inverse kinematics lets you specify where you want the robot's hand to go, and the algorithm figures out what angles each servo motor needs.

For Arduino robot arms, you can use a simplified version:
1. Measure where the hand is now (forward kinematics)
2. Calculate how far off you are from the target
3. Move each servo a small amount in the direction that reduces the error
4. Repeat until close enough

**Arduino Code**:
```cpp
#include <Servo.h>

void moveToTarget(float targetX, float targetY) {
  // Simple iterative approach for 2-DOF arm
  for (int iter = 0; iter < 100; iter++) {
    // Calculate current position
    float currentX = getX();
    float currentY = getY();

    // Calculate error
    float errorX = targetX - currentX;
    float errorY = targetY - currentY;

    // Small adjustment (1 degree increments)
    if (abs(errorX) > 0.01 || abs(errorY) > 0.01) {
      servo1.write(servo1.read() + adjustAngle1);
      servo2.write(servo2.read() + adjustAngle2);
    } else {
      break;  // Close enough!
    }
    delay(20);
  }
}
```

This avoids complex math - the Arduino just tries small movements until it gets close!"

### Profile 2: Advanced with ROS2 & C++
**Adapted**:
"Implement the Jacobian pseudo-inverse IK solver using Eigen for real-time performance. For a 6-DOF manipulator, we compute:

$$\Delta \mathbf{q} = J^+(\mathbf{q}) \cdot \Delta \mathbf{x}$$

where $J^+ = J^T(JJ^T + \lambda I)^{-1}$ is the damped pseudo-inverse (» = 0.01 for numerical stability near singularities).

**ROS2 Controller**:
```cpp
#include <Eigen/Dense>
#include "rclpy/rclpy.hpp"
#include "control_msgs/msg/joint_trajectory.hpp"

class IKController : public rclpy::Node {
private:
  Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q) {
    // Analytic Jacobian for your specific robot
    // Use chain rule: J = f/q
    Eigen::MatrixXd J(6, q.size());
    // ... compute Jacobian ...
    return J;
  }

  Eigen::VectorXd dampedPseudoInverse(const Eigen::MatrixXd& J, double lambda = 0.01) {
    Eigen::MatrixXd J_T = J.transpose();
    Eigen::MatrixXd A = J * J_T + lambda * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    return J_T * A.inverse();
  }

public:
  void solveIK(const Eigen::Vector3d& target_pos, const Eigen::Quaterniond& target_orient) {
    // Iterative Jacobian-based IK with singularity handling
    // See [Siciliano 2009] for algorithmic details
  }
};
```

**Performance**: Achieves < 1ms solve time for 6-DOF arms on modern CPUs. For 50+ DOF humanoids, consider sparse Jacobian methods or hierarchical task decomposition."

## User Profile Structure

Expected profile data:
```json
{
  "expertise": "beginner|intermediate|advanced",
  "languages": ["Python", "C++", "MATLAB"],
  "hardware": ["Arduino", "Raspberry Pi", "NVIDIA Jetson"],
  "frameworks": ["ROS2", "PyTorch", "OpenCV"],
  "learning_style": "hands-on|theoretical|visual"
}
```

## Personalization Best Practices

1. **Preserve Accuracy**: Never sacrifice technical correctness
2. **Maintain Learning Objectives**: Ensure adapted content still teaches core concepts
3. **Bridge Knowledge Gaps**: Add prerequisite explanations when needed
4. **Use Familiar Tools**: Reference libraries/hardware user knows
5. **Respect Expertise**: Don't over-explain to advanced users or overwhelm beginners
