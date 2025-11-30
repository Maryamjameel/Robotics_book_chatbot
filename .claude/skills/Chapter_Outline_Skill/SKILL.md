---
name: "robotics-chapter-outline"
description: "Generate comprehensive chapter outlines for robotics and humanoid AI textbooks. Creates structured learning objectives, section hierarchies, key concepts, equations, code examples, and exercises. Use when planning or structuring robotics textbook chapters."
version: "1.0.0"
---

# Robotics Chapter Outline Skill

## When to Use This Skill

- User asks to "create a chapter outline" for robotics topics
- User mentions planning textbook content for robotics, kinematics, dynamics, control, perception, or AI
- User needs to structure a new chapter or reorganize existing content
- User wants learning outcomes and section hierarchy for a robotics concept

## How This Skill Works

1. **Identify Topic**: Determine the robotics domain (kinematics, dynamics, control, perception, planning, etc.)
2. **Define Learning Objectives**: Create 3-5 Bloom's taxonomy-aligned outcomes
3. **Structure Sections**: Organize content from foundations to advanced applications
4. **Identify Key Concepts**: List equations, algorithms, and terminology to cover
5. **Plan Examples**: Suggest worked examples and code implementations
6. **Design Exercises**: Outline practice problems for chapter end

## Output Format

Provide a structured outline with:

### 1. Chapter Metadata
- **Chapter Number & Title**
- **Estimated Length**: (pages/sections)
- **Prerequisites**: Required prior knowledge
- **Target Audience**: Undergraduate/Graduate/Professional

### 2. Learning Outcomes (3-5 objectives)
Format: "Students will be able to [verb] [concept] [context]"
- Use action verbs: Derive, Implement, Analyze, Design, Evaluate

### 3. Section Hierarchy
```
X.1 Introduction & Motivation
    X.1.1 Real-world applications
    X.1.2 Historical context

X.2 Theoretical Foundations
    X.2.1 Mathematical preliminaries
    X.2.2 Core equations/principles

X.3 Implementation & Methods
    X.3.1 Algorithmic approaches
    X.3.2 Computational considerations

X.4 Advanced Topics (optional)

X.5 Summary & Key Takeaways

X.6 Exercises & Problems
```

### 4. Key Concepts to Cover
- **Equations**: List main mathematical formulas (e.g., Jacobian, DH parameters)
- **Algorithms**: Core procedures (e.g., inverse kinematics solver, PID controller)
- **Terminology**: Technical terms requiring glossary entries
- **Notation**: Symbolic conventions (e.g., θ for joint angles, q for configuration)

### 5. Worked Examples (2-4 examples)
- **Example 1**: Simple illustrative case
- **Example 2**: Realistic application scenario
- **Example 3**: Edge case or special consideration

### 6. Code Implementations
- Language/framework (Python, MATLAB, ROS, C++)
- Key functions to implement
- Expected inputs/outputs

### 7. Practice Problems
- **Beginner** (30%): Direct application, recall
- **Intermediate** (50%): Multi-step problem solving
- **Advanced** (20%): Design, optimization, analysis

### 8. Cross-References
- **Related Chapters**: Links to prerequisite or follow-up content
- **External Resources**: Key papers, textbooks, documentation

## Example 1: Kinematics Chapter

**Input**: "Create a chapter outline for robot kinematics"

**Output**:

### Chapter 3: Robot Kinematics

**Metadata**:
- Estimated Length: 35-40 pages (8 sections)
- Prerequisites: Linear algebra, coordinate transformations (Chapter 2)
- Target Audience: Undergraduate robotics students

**Learning Outcomes**:
1. Derive forward kinematics using Denavit-Hartenberg convention for serial manipulators
2. Implement inverse kinematics solutions using geometric and analytical methods
3. Compute Jacobian matrices and analyze singularities in robot workspace
4. Evaluate the suitability of different IK approaches for real-time control applications

**Section Hierarchy**:
```
3.1 Introduction to Robot Kinematics
    3.1.1 Forward vs. Inverse Kinematics
    3.1.2 Applications in manipulation and locomotion

3.2 Coordinate Frames and Transformations
    3.2.1 Homogeneous transformation matrices
    3.2.2 Denavit-Hartenberg parameters

3.3 Forward Kinematics
    3.3.1 DH convention methodology
    3.3.2 Chain multiplication for serial arms
    3.3.3 Worked example: 3-DOF planar arm

3.4 Inverse Kinematics
    3.4.1 Analytical solutions (geometric approach)
    3.4.2 Numerical solutions (Jacobian-based)
    3.4.3 Multiple solutions and workspace limits

3.5 Differential Kinematics
    3.5.1 Jacobian matrix derivation
    3.5.2 Velocity and force mapping
    3.5.3 Singularity analysis

3.6 Case Study: 6-DOF Industrial Manipulator

3.7 Summary and Key Takeaways

3.8 Exercises and Programming Assignments
```

**Key Concepts**:
- **Equations**:
  - Homogeneous transform: T = [R | p; 0 | 1]
  - DH transformation: T_i = Rot_z(θ) Trans_z(d) Trans_x(a) Rot_x(α)
  - Jacobian: J(q) = ∂f(q)/∂q
  - Inverse kinematics: q = f⁻¹(x)

- **Algorithms**:
  - DH parameter table construction
  - Geometric inverse kinematics solver
  - Jacobian pseudo-inverse method
  - Singularity detection

- **Terminology**:
  - End-effector, workspace, configuration space, singularity,
  - joint space, task space, degrees of freedom (DOF)

- **Notation**:
  - θ_i: joint angle for joint i
  - q: configuration vector (joint angles)
  - x: end-effector pose
  - J(q): Jacobian matrix

**Worked Examples**:
1. **3-DOF Planar Arm FK**: Calculate end-effector position from given joint angles
2. **2-Link Arm IK**: Solve analytically using geometric approach with elbow-up/down solutions
3. **Jacobian Computation**: Derive 6×6 Jacobian for PUMA 560 manipulator
4. **Singularity Analysis**: Identify and classify singularities in workspace

**Code Implementations** (Python/NumPy):
```python
def forward_kinematics(dh_params, joint_angles):
    """Compute FK using DH parameters"""
    # Returns: 4x4 transformation matrix

def inverse_kinematics_geometric(target_pose, arm_length):
    """Solve 2-link IK analytically"""
    # Returns: [theta1, theta2] or None if unreachable

def compute_jacobian(q):
    """Numerical Jacobian calculation"""
    # Returns: 6xN Jacobian matrix
```

**Practice Problems**:
- **Beginner**:
  - P3.1: Construct DH table for given robot diagram
  - P3.2: Calculate FK for 2-DOF arm with θ₁=30°, θ₂=45°

- **Intermediate**:
  - P3.3: Implement IK solver for 3-DOF planar arm
  - P3.4: Analyze workspace boundaries and unreachable regions
  - P3.5: Compute manipulability index across workspace

- **Advanced**:
  - P3.6: Design redundancy resolution strategy for 7-DOF arm
  - P3.7: Implement real-time IK with obstacle avoidance

**Cross-References**:
- **Prerequisites**: Chapter 2 (Spatial Transformations)
- **Follow-up**: Chapter 4 (Differential Kinematics), Chapter 5 (Dynamics)
- **External**: Craig's "Introduction to Robotics", Spong's "Robot Modeling and Control"

---

## Example 2: Control Systems Chapter

**Input**: "Outline a chapter on robot control"

**Output**:

### Chapter 7: Robot Control Systems

**Metadata**:
- Estimated Length: 45-50 pages (9 sections)
- Prerequisites: Dynamics (Ch 5), State-space methods (Ch 6)
- Target Audience: Graduate robotics/controls students

**Learning Outcomes**:
1. Design PID controllers for joint-space trajectory tracking with stability guarantees
2. Implement computed-torque control for nonlinear robot dynamics compensation
3. Analyze stability using Lyapunov methods for adaptive control schemes
4. Evaluate trade-offs between joint-space and task-space control architectures

**Section Hierarchy**:
```
7.1 Introduction to Robot Control
    7.1.1 Control objectives and performance metrics
    7.1.2 Joint-space vs. task-space control

7.2 PID Control for Robots
    7.2.1 Independent joint control
    7.2.2 Tuning methods and practical considerations
    7.2.3 Limitations and coupling effects

7.3 Computed-Torque Control
    7.3.1 Inverse dynamics formulation
    7.3.2 Model uncertainty and robustness

7.4 Task-Space Control
    7.4.1 Operational space formulation
    7.4.2 Impedance control

7.5 Adaptive and Learning Control
    7.5.1 Parameter adaptation
    7.5.2 Learning from demonstration

7.6 Whole-Body Control for Humanoids
    7.6.1 Hierarchical task prioritization
    7.6.2 Contact constraints and balance

7.7 Implementation and Real-Time Considerations

7.8 Summary and Key Takeaways

7.9 Exercises and Control Design Projects
```

**Key Concepts**:
- **Equations**:
  - Robot dynamics: M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
  - PID control law: τ = K_p·e + K_i·∫e + K_d·ė
  - Computed torque: τ = M(q)(q̈_d + K_v·ė + K_p·e) + C + G
  - Lyapunov stability: V̇ ≤ 0

**Code Implementations** (Python/ROS):
```python
class PIDController:
    def compute_control(self, q_desired, q_actual, dt):
        """PID control law implementation"""

class ComputedTorqueController:
    def control(self, q_d, qd_d, qdd_d, q, qd):
        """Inverse dynamics control"""
```

---

## Tips for Using This Skill

1. **Start Broad**: Request high-level outline first, then drill into specific sections
2. **Iterate**: Refine sections based on depth and target audience
3. **Coordinate with Agents**:
   - Use **textbook-author** agent to write the actual chapter content from this outline
   - Use **glossary-manager** to create definitions for identified terminology
   - Use **robotics-quiz-generator** to create the exercises outlined here
4. **Check Prerequisites**: Ensure earlier chapters cover the prerequisite knowledge listed

## Advanced Usage

**Combine multiple topics**:
- "Create outline combining kinematics and control for mobile manipulators"

**Specify constraints**:
- "Create a 20-page chapter outline on vision-based control, suitable for undergraduates"

**Request variations**:
- "Give me two versions: one mathematical, one implementation-focused"