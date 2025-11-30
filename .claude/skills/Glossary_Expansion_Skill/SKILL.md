---
name: "robotics-glossary-expander"
description: "Create and expand glossary entries for robotics technical terms. Provides precise definitions, cross-references, and usage examples. Use when user needs terminology defined or glossary updated."
version: "1.0.0"
---

# Robotics Glossary Expansion Skill

## When to Use This Skill

- User asks "define [technical term]"
- User wants to add terms to glossary
- User needs consistent terminology across chapters
- User requests cross-references between related concepts
- User is building or updating a technical glossary

## How This Skill Works

1. **Identify Terms**: Extract technical terminology from text or user request
2. **Research Context**: Understand how term is used in robotics
3. **Write Definition**: Create precise, accessible 1-3 sentence definition
4. **Add Cross-References**: Link to related terms
5. **Provide Examples**: Show usage in context
6. **Format Entry**: Structure for glossary file

## Output Format

### Standard Glossary Entry

**Term** (Acronym if applicable): [Precise definition in 1-3 sentences. Includes domain context, mathematical representation if relevant, and practical significance]

*See also*: [Related Term 1], [Related Term 2], [Related Term 3]

*Example*: [Usage in sentence showing proper context]

*Symbol/Notation*: [If mathematical, show standard notation]

### Entry Categories

**Basic Term** (Beginner-level):
- Simple language
- Analogies when helpful
- No heavy mathematics

**Technical Term** (Intermediate):
- Precise engineering definition
- Light mathematical notation
- Assumptions stated

**Advanced Term** (Graduate-level):
- Rigorous mathematical definition
- Theoretical foundations
- References to literature

## Example Entries

### Example 1: Kinematics Term

**Jacobian Matrix**: A matrix that relates joint velocities to end-effector velocities in a robot manipulator, defined as the partial derivatives of forward kinematics equations with respect to joint variables. The Jacobian is crucial for velocity control, singularity analysis, and force mapping between joint space and task space.

*See also*: [Forward Kinematics], [Singularity], [Differential Kinematics], [Configuration Space]

*Example*: "The robot's Jacobian becomes singular when det(J) = 0, causing loss of mobility in certain directions."

*Symbol/Notation*: **J**(q) or J  ^(6×n) for n-DOF manipulator

*Mathematical Form*:
$$J(q) = \frac{\partial f(q)}{\partial q}$$
where f(q) is the forward kinematics function.

---

### Example 2: Control Term

**Computed Torque Control**: A nonlinear control strategy that uses the inverse dynamics model of a robot to linearize and decouple the system, allowing independent PID control of each joint. Also known as inverse dynamics control or feedback linearization.

*See also*: [Inverse Dynamics], [PID Control], [Model-Based Control], [Trajectory Tracking]

*Example*: "Computed torque control compensates for Coriolis and gravitational forces, improving tracking performance compared to simple PID controllers."

*Symbol/Notation*: Ä = M(q)(q_d + K_v· + K_p·e) + C(q,q) + G(q)

*Assumptions*: Requires accurate dynamic model; model errors degrade performance.

---

### Example 3: Planning Term

**Configuration Space** (C-space): The space of all possible joint configurations of a robot, where each point represents a unique robot pose. For an n-DOF robot, C-space is n-dimensional, and obstacles in workspace map to forbidden regions in C-space.

*See also*: [Workspace], [Degrees of Freedom], [Motion Planning], [Collision Detection]

*Example*: "Path planning in configuration space allows efficient collision checking by representing the robot as a point and obstacles as C-space obstacles."

*Notation*: C or Q, dimension: dim(C) = n for n-DOF system

*Contrast with*: **Workspace** (task space) - the 3D Cartesian space where the end-effector moves

---

### Example 4: Perception Term

**SLAM** (Simultaneous Localization and Mapping): The problem of building a map of an unknown environment while simultaneously determining the robot's location within that map, using sensor measurements and motion estimates. SLAM is fundamental for autonomous navigation in GPS-denied environments.

*See also*: [Localization], [Mapping], [Kalman Filter], [Particle Filter], [Loop Closure]

*Example*: "Visual SLAM uses camera images to detect landmarks and estimate both the robot's trajectory and the 3D structure of the environment."

*Common Approaches*: EKF-SLAM, FastSLAM, GraphSLAM, ORB-SLAM

*Applications*: Autonomous vehicles, drones, indoor robots

---

## Acronym Handling

When defining acronyms:

**Format**:
**ACRONYM** (Full Expansion): [Definition]

**Examples**:
- **DOF** (Degrees of Freedom): The number of independent parameters needed to specify a robot's configuration
- **ROS** (Robot Operating System): A flexible framework for writing robot software, providing tools and libraries for hardware abstraction, device control, and inter-process communication
- **IMU** (Inertial Measurement Unit): A sensor that measures acceleration and angular velocity, typically combining accelerometers, gyroscopes, and sometimes magnetometers

## Cross-Reference Guidelines

**Link related concepts**:
- Hierarchical: General ’ Specific (e.g., Kinematics ’ Forward Kinematics)
- Contrasting: Opposite concepts (e.g., Forward vs Inverse Kinematics)
- Prerequisites: Foundational concepts (e.g., Jacobian ’ Linear Algebra)
- Applications: Where used (e.g., DH Parameters ’ Forward Kinematics)

**Standard phrases**:
- *See also*: Related concepts at same level
- *Contrast with*: Opposite or commonly confused terms
- *Prerequisite*: Required background knowledge
- *Used in*: Applications and higher-level concepts

## Multilingual Support

For bilingual glossaries (English-Urdu):

**English Entry**:
**End-Effector**: The device at the end of a robotic manipulator...

**Urdu Entry** (in glossary-ur.md):
**'ÌFˆ 'ÌAÌ©y1** (End-Effector): 1H(Hy ('2H ©Ò ".1 EÌº F5( "DÁ ,H...

## Quality Checklist

Before adding glossary entry:
- [ ] Definition is 1-3 sentences
- [ ] Technically accurate
- [ ] Accessible to target audience
- [ ] Includes notation if mathematical
- [ ] Has 2-4 cross-references
- [ ] Example shows proper usage
- [ ] No circular definitions
- [ ] Consistent with existing entries
