---
name: "robotics-context-extractor"
description: "Extract key context from robotics book chapters or research papers. Identifies main concepts, equations, algorithms, terminology, and learning objectives. Use when user needs to understand what a chapter covers or extract structured information."
version: "1.0.0"
---

# Robotics Context Extraction Skill

## When to Use This Skill

- User asks "what does this chapter cover?" or "summarize the main topics"
- User needs to extract equations, algorithms, or terminology from text
- User wants to identify learning objectives or prerequisites
- User is comparing chapters or identifying knowledge gaps
- User needs structured metadata from unstructured content

## How This Skill Works

1. **Scan Content**: Read through provided chapter or document
2. **Identify Domains**: Recognize robotics topics (kinematics, control, etc.)
3. **Extract Equations**: Pull out all mathematical formulas
4. **List Algorithms**: Identify computational procedures
5. **Catalog Terminology**: Find technical terms and jargon
6. **Infer Learning Objectives**: Determine what students should learn
7. **Map Prerequisites**: Identify required background knowledge

## Output Format

### Chapter Metadata
- **Title & Number**
- **Domain**: Kinematics/Dynamics/Control/Perception/Planning
- **Estimated Difficulty**: Undergraduate/Graduate/Advanced
- **Length**: Sections count, approximate pages

### Main Topics Covered
1. Topic 1 (with brief description)
2. Topic 2
3. ...

### Key Equations Extracted
- Equation 1: $formula$ - [Description]
- Equation 2: $formula$ - [Purpose]

### Algorithms & Procedures
- **Algorithm 1**: Name - [What it does]
- **Algorithm 2**: Name - [Application]

### Technical Terminology Found
- Term 1: [Brief definition]
- Term 2: [Context where used]
- ...

### Inferred Learning Objectives
1. [Verb] [Concept] [Context]
2. [Verb] [Concept] [Context]

### Prerequisites Identified
- Mathematical: Linear algebra, calculus, probability
- Robotics: Prior chapters, concepts needed

### Cross-References
- References to other chapters
- External citations

## Example Output

**Input**: [Chapter 3 on Robot Kinematics]

**Output**:

### Metadata
- Title: Chapter 3 - Robot Kinematics
- Domain: Kinematics (Manipulation)
- Difficulty: Undergraduate (2nd/3rd year)
- Length: 8 sections, ~40 pages

### Main Topics
1. **Forward Kinematics**: Computing end-effector pose from joint angles
2. **Inverse Kinematics**: Solving for joint angles from desired pose
3. **Denavit-Hartenberg Convention**: Systematic coordinate frame assignment
4. **Jacobian Matrices**: Velocity/force relationships
5. **Singularities**: Workspace analysis

### Key Equations
- $T = T_1 \cdot T_2 \cdot ... \cdot T_n$ - Forward kinematics chain
- $T_i = Rot_z(\theta) Trans_z(d) Trans_x(a) Rot_x(\alpha)$ - DH transform
- $J(q) = \frac{\partial f(q)}{\partial q}$ - Jacobian definition
- $det(J) = 0$ - Singularity condition

### Algorithms
- **DH Parameter Construction**: Systematic method for frame assignment
- **Geometric IK Solver**: Closed-form solution for 2-link arm
- **Jacobian Pseudo-inverse**: Numerical IK using $\Delta q = J^+ \Delta x$

### Terminology
- End-effector, workspace, configuration space
- Singularity (wrist, elbow, shoulder types)
- Degrees of freedom (DOF), redundancy
- Joint space vs task space

### Learning Objectives
1. Derive forward kinematics using DH convention for serial manipulators
2. Implement inverse kinematics using geometric and numerical methods
3. Compute Jacobian matrices for velocity mapping
4. Analyze singularities and workspace limitations

### Prerequisites
- **Math**: Linear algebra (matrices, transformations), trigonometry
- **Robotics**: Chapter 2 (Spatial Transformations)
- **Programming**: Basic Python/MATLAB for implementations

### Cross-References
- Builds on: Chapter 2 (Coordinate Frames)
- Leads to: Chapter 4 (Differential Kinematics), Chapter 5 (Dynamics)
- External: Craig's textbook, Spong & Vidyasagar

## Use Cases

1. **Chapter Planning**: Extract context from existing chapters to plan new ones
2. **Gap Analysis**: Identify what topics are missing between chapters
3. **Study Guide Creation**: Generate topic summaries for students
4. **Prerequisite Mapping**: Build dependency graphs for curriculum design
5. **Content Reuse**: Extract reusable components (equations, algorithms)
