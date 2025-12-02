---
name: "robotics-summary-builder"
description: "Create concise summaries of robotics chapters, sections, or concepts. Generates study guides, key takeaways, and chapter reviews. Use when user needs condensed versions of content."
version: "1.0.0"
---

# Robotics Summary Builder Skill

## When to Use This Skill

- User asks "summarize this chapter" or "give me key points"
- User needs study guide or review material
- User wants chapter overview before reading
- User requests condensed version for quick reference
- User is preparing for exams or presentations

## How This Skill Works

1. **Analyze Content**: Read through chapter/section
2. **Extract Core Concepts**: Identify main topics and learning objectives
3. **Distill Key Points**: Select most important information
4. **Organize Hierarchically**: Structure from high-level to details
5. **Format for Readability**: Use bullets, sections, clear headings
6. **Add References**: Link to full content for deeper study

## Output Format

### Chapter Summary Structure

```markdown
# Summary: Chapter X - [Title]

## Overview
[2-3 sentence high-level summary of what chapter covers]

## Learning Objectives
1. [Objective 1]
2. [Objective 2]
3. [Objective 3]

## Key Concepts

### Concept 1: [Name]
**Definition**: [Brief definition]
**Importance**: [Why it matters]
**Key Equation**: $formula$ (if applicable)

### Concept 2: [Name]
**Definition**: [Brief definition]
**Application**: [Where it's used]

## Important Equations
1. $equation_1$  [What it computes]
2. $equation_2$  [Purpose]

## Algorithms & Methods
- **Algorithm 1**: [What it does, when to use]
- **Method 2**: [Description, pros/cons]

## Practical Applications
- Application 1: [Brief description]
- Application 2: [Example use case]

## Common Pitfalls
- Mistake 1: [What students often get wrong]
- Mistake 2: [Misconception to avoid]

## Key Takeaways
1. [Most important point #1]
2. [Critical insight #2]
3. [Essential skill #3]

## Related Topics
- **Prerequisites**: [What you should know first]
- **Builds To**: [What comes next]
- **Cross-References**: [Related chapters]

## Study Tips
- Focus on: [Most important areas]
- Practice: [Recommended exercises]
- Review: [Key sections to re-read]
```

## Example 1: Chapter Summary

**Input**: "Summarize Chapter 3 on Robot Kinematics"

**Output**:

```markdown
# Summary: Chapter 3 - Robot Kinematics

## Overview
This chapter covers the geometry of robot motion, focusing on computing the position and orientation of robot parts. You'll learn forward kinematics (joint angles í end-effector pose) and inverse kinematics (desired pose í joint angles), which are fundamental for robot programming and control.

## Learning Objectives
1. Derive forward kinematics using Denavit-Hartenberg convention
2. Solve inverse kinematics using geometric and numerical methods
3. Compute Jacobian matrices for velocity mapping
4. Analyze singularities and workspace limitations

## Key Concepts

### Forward Kinematics (FK)
**Definition**: Computing end-effector pose from joint angles
**Method**: Chain multiplication of transformation matrices using DH parameters
**Key Equation**: $T = T_1 \cdot T_2 \cdot ... \cdot T_n$

### Inverse Kinematics (IK)
**Definition**: Finding joint angles that achieve desired end-effector pose
**Methods**:
- Analytical: Closed-form geometric solutions (fast, limited to specific robots)
- Numerical: Jacobian-based iteration (general, slower)
**Challenge**: Multiple solutions (typically 8 for 6-DOF arms)

### Jacobian Matrix
**Definition**: Maps joint velocities to end-effector velocities
**Formula**: $J(q) = \frac{\partial f(q)}{\partial q}$
**Uses**: Velocity control, force mapping, singularity analysis

### Singularities
**Definition**: Configurations where robot loses mobility (det(J) = 0)
**Types**: Wrist, elbow, shoulder singularities
**Avoidance**: Path planning, damped least squares

## Important Equations
1. $T_i = Rot_z(\theta) \cdot Trans_z(d) \cdot Trans_x(a) \cdot Rot_x(\alpha)$  DH transformation
2. $\dot{x} = J(q) \dot{q}$  Velocity mapping
3. $\Delta q = J^+(\Delta x)$  Numerical IK step
4. $J^+ = J^T(JJ^T + \lambda I)^{-1}$  Damped pseudo-inverse

## Algorithms & Methods
- **DH Parameter Assignment**: Systematic frame placement using 4 rules
- **Geometric IK for 2-Link Arm**: Trigonometry-based closed form
- **Jacobian Pseudo-Inverse Method**: Iterative numerical IK
- **Singularity Detection**: Check det(J) or condition number

## Practical Applications
- **Industrial Pick-and-Place**: FK for current position, IK for target reach
- **Humanoid Arm Control**: Multiple IK solutions for obstacle avoidance
- **Trajectory Planning**: Jacobian for smooth velocity profiles

## Common Pitfalls
- **Forgetting cumulative angles in FK**: Must add angles for serial chains
- **Ignoring multiple IK solutions**: Need criteria to select best solution
- **Using degrees in trig functions**: Python/MATLAB use radians
- **Not handling singularities**: Causes numerical instability in IK

## Key Takeaways
1. **FK is unique, IK is not**: One set of joint angles gives one pose, but one pose can come from many joint angle sets
2. **Jacobian is central**: Used in velocity control, force analysis, and numerical IK
3. **Singularities must be handled**: They're unavoidable for most robots but can be managed

## Related Topics
- **Prerequisites**: Chapter 2 (Spatial Transformations, homogeneous matrices)
- **Builds To**: Chapter 4 (Differential Kinematics), Chapter 5 (Dynamics)
- **Cross-References**: Chapter 8 (Motion Planning - uses IK extensively)

## Study Tips
- **Focus on**: DH convention rules, Jacobian computation, singularity types
- **Practice**: Problems 3.2-3.7 (2-link and 3-link arms)
- **Code**: Implement FK and IK for planar arm (Project 3.1)
- **Review**: Example 3.4 (PUMA 560 IK) shows real robot application
```

## Example 2: Section Summary

**Input**: "Summarize Section 7.2 on PID Control"

**Output**:

```markdown
# Summary: Section 7.2 - PID Control for Robots

## Overview
PID (Proportional-Integral-Derivative) control is the most common industrial robot controller, combining three terms to track desired trajectories with zero steady-state error.

## Key Points

### Control Law
$$\tau = K_p e + K_i \int e \, dt + K_d \dot{e}$$
where $e = q_d - q$ is tracking error

### Three Gains Explained
- **Kö (Proportional)**: Immediate response to current error
  - Higher Kö í faster response but more overshoot
- **Kb (Integral)**: Eliminates steady-state error
  - Accumulates past errors, drives error to zero
- **Kê (Derivative)**: Adds damping to reduce overshoot
  - Predicts future error based on rate of change

### Tuning Process (Ziegler-Nichols)
1. Start with Kb = Kê = 0
2. Increase Kö until steady oscillation
3. Add Kê to dampen oscillations
4. Add Kb if steady-state error remains

### Limitations for Robots
- Assumes **decoupled joints** (not true for robots)
- Ignores Coriolis and centrifugal forces
- Poor performance at high speeds or heavy loads
- Solution: Use computed torque control (Section 7.3)

## Key Takeaway
PID works well for low-speed, light-load robots but model-based control is better for demanding applications.
```

## Summary Types

### 1. Executive Summary (1 paragraph)
**Best for**: Quick overview, chapter introduction
**Length**: 3-5 sentences covering main topic and key insights

### 2. Study Guide Summary (1-2 pages)
**Best for**: Exam prep, quick reference
**Includes**: All key concepts, equations, algorithms, pitfalls
**Format**: Highly structured with clear sections

### 3. Concept Map Summary (Hierarchical)
**Best for**: Understanding relationships between topics
**Format**: Indented outline showing concept hierarchy

### 4. Equation Reference (Minimal)
**Best for**: Quick formula lookup
**Includes**: Only equations with brief descriptions

### 5. Practical Summary (Application-focused)
**Best for**: Engineers needing implementation guidance
**Emphasizes**: When to use each method, code examples, common issues

## Customization Options

User can request:
- **Length**: "1-paragraph summary" vs "detailed 2-page study guide"
- **Focus**: "Summarize just the equations" or "emphasize applications"
- **Audience**: "For beginners" (simpler language) vs "For experts" (technical depth)
- **Format**: "Bullet points only" vs "Narrative paragraphs"

## Quality Checklist

Good summary:
-  Captures all major concepts from original
-  Uses clear, concise language
-  Maintains technical accuracy
-  Organized logically
-  Includes key equations/algorithms
-  Notes common pitfalls
-  Provides context (prerequisites, applications)
-  Has clear takeaways

Avoid:
-  Copying large blocks verbatim
-  Including minor details
-  Vague statements ("very important", "complex topic")
-  Missing key concepts
-  Incorrect simplifications
