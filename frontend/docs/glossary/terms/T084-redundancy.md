---
id: T084
term: Redundancy
acronym:
categories:
  - Kinematics & Dynamics
  - Control & Learning
related_terms:
  - Degrees of Freedom
  - Kinematics
  - Inverse Kinematics
  - Null Space
chapter_introduced: 3
section_reference: 3.4
usage_example: The 7-DOF robot possesses kinematic redundancy, allowing it to reach a point in multiple configurations and avoid obstacles.
---
In robotics, redundancy occurs when a manipulator has more degrees of freedom (DOF) than strictly necessary to achieve a desired task. For instance, reaching a point in 3D space requires a minimum of 3 DOFs for position, and 3 more for orientation, totaling 6. A robot with 7 or more DOFs is considered kinematically redundant. This excess of freedom provides significant advantages, such as obstacle avoidance, singularity avoidance, optimizing joint limits, or minimizing energy consumption, as there are infinite joint configurations that can achieve the same end-effector pose.

### Key Concepts
- **Excess DOF**: More joints than required for a task.
- **Obstacle Avoidance**: Ability to navigate around obstructions.
- **Singularity Avoidance**: Can choose configurations to stay away from singular points.
- **Null Space**: The set of joint motions that do not change the end-effector pose.

### Related Terms
- Degrees of Freedom
- Kinematics
- Inverse Kinematics
- Null Space

### See Also
- [Kinematic Redundancy on Wikipedia](https://en.wikipedia.org/wiki/Kinematic_redundancy)
- Robotics textbook chapter on Redundant Manipulators
