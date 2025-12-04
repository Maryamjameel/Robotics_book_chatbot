---
id: T083
term: Workspace
acronym:
categories:
  - Kinematics & Dynamics
related_terms:
  - Joint Space
  - Task Space
  - Degrees of Freedom
  - Robot Configuration
chapter_introduced: 3
section_reference: 3.3
usage_example: The robot's spherical workspace limits the positions its end-effector can reach.
---
The workspace of a robot refers to the total volume of space that its end-effector can reach. It is a critical characteristic in robot design and task planning. The workspace is typically defined by the robot's kinematics, including its link lengths, joint limits, and degrees of freedom. There are two primary types: the *reachable workspace*, which includes all points the end-effector can reach in at least one orientation, and the *dexterous workspace*, which includes all points the end-effector can reach in all orientations.

### Key Concepts
- **Reachable Workspace**: All points the end-effector can touch.
- **Dexterous Workspace**: All points the end-effector can touch with any orientation.
- **Joint Limits**: Physical constraints on the range of motion of each joint.
- **End-Effector**: The tool or gripper attached to the robot's final link.

### Related Terms
- Joint Space
- Task Space
- Degrees of Freedom
- Robot Configuration

### See Also
- [Robot Workspace on Wikipedia](https://en.wikipedia.org/wiki/Robot_workspace)
- Robotics textbook chapter on Kinematics and Workspace Analysis
