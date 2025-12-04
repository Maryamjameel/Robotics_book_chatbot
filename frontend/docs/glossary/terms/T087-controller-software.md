---
id: T087
term: Controller (Software)
acronym:
categories:
  - Control & Learning
  - Software Concepts
related_terms:
  - Feedback Control
  - PID Controller
  - Control Loop
  - State Estimation
chapter_introduced: 3
section_reference: 3.6
usage_example: The robot's joint-level controller uses a PID algorithm to maintain desired positions and velocities.
---
In robotics, a software controller is a program or algorithm that processes sensor feedback and generates commands for the robot's actuators to achieve a desired behavior or trajectory. It is the "brain" of the control system, implementing control laws (e.g., PID, fuzzy logic, model predictive control) to minimize the error between the desired state and the actual state of the robot. Software controllers can operate at various levels, from low-level joint control to high-level task planning, and are essential for precise, stable, and intelligent robot operation.

### Key Concepts
- **Control Algorithm**: The logic (e.g., PID) used to derive commands.
- **Sensor Feedback**: Input data used to assess the robot's current state.
- **Actuator Commands**: Output signals sent to motors or other actuators.
- **Error Minimization**: The primary goal of a controller, reducing deviation from target.

### Related Terms
- Feedback Control
- PID Controller
- Control Loop
- State Estimation

### See Also
- [Control System on Wikipedia](https://en.wikipedia.org/wiki/Control_system)
- Robotics textbook chapter on Robot Control Architectures
