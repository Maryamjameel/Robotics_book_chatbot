---
name: "robotics-quiz-builder"
description: "Generate comprehensive assessments for robotics chapters: MCQs, coding tasks, conceptual questions, troubleshooting scenarios, and real-world problems. Creates both student and instructor versions. Use when user needs chapter-end exercises or practice materials."
version: "1.0.0"
---

# Robotics Quiz & Assessment Builder Skill

## When to Use This Skill

- User asks for "practice problems" or "end-of-chapter exercises"
- User needs assessment materials for students
- User wants MCQs, coding tasks, or conceptual questions
- User is creating exams or quizzes for robotics courses
- User requests difficulty-varied problem sets

## How This Skill Works

1. **Extract Learning Objectives**: Identify what chapter teaches
2. **Design Question Mix**: Create MCQs, coding, conceptual, troubleshooting, application problems
3. **Vary Difficulty**: Generate beginner (30%), intermediate (50%), advanced (20%)
4. **Write Solutions**: Create detailed answer keys and rubrics
5. **Format Output**: Provide student and instructor versions

## Output Format

### Student Version Structure

```markdown
# Chapter X Assessment: [Topic]

**Instructions**: [Time limit, allowed resources, grading breakdown]

**Total Points**: [X points]

---

## Part 1: Multiple Choice (20 points)

**Q1.1** [Difficulty: Beginner] (2 points)
What is the primary purpose of forward kinematics?

A) Compute joint angles from end-effector pose
B) Compute end-effector pose from joint angles 
C) Optimize trajectory planning
D) Calculate robot dynamics

---

## Part 2: Coding Tasks (30 points)

**Q2.1** [Difficulty: Intermediate] (15 points)
Implement a function to compute the Jacobian matrix for a 3-DOF planar arm.

**Requirements**:
- Input: joint angles ∏ = [∏Å, ∏Ç, ∏É], link lengths L = [LÅ, LÇ, LÉ]
- Output: 2◊3 Jacobian matrix
- Language: Python using NumPy

**Starter Code**:
```python
import numpy as np

def compute_jacobian(theta, L):
    """
    Compute Jacobian for 3-DOF planar arm

    Args:
        theta: array of 3 joint angles (radians)
        L: array of 3 link lengths (meters)

    Returns:
        J: 2x3 Jacobian matrix
    """
    # YOUR CODE HERE
    pass
```

**Test Cases**:
- Input: ∏=[0, 0, 0], L=[1, 1, 1] í Expected: J = [[0, 0, 0], [3, 2, 1]]
- Input: ∏=[¿/2, 0, 0], L=[1, 1, 1] í Expected: J = [[-3, -2, -1], [0, 0, 0]]

---

## Part 3: Conceptual Questions (20 points)

**Q3.1** [Difficulty: Intermediate] (10 points)
Explain why a 6-DOF robot arm can experience singularities and describe two methods to avoid them. (4-6 sentences)

**Grading Criteria**:
- Definition of singularity (3 points)
- Physical explanation (3 points)
- Two avoidance methods (2 points each)

---

## Part 4: Troubleshooting (15 points)

**Q4.1** [Difficulty: Intermediate] (15 points)
A student's inverse kinematics solver sometimes fails with the error "Solution not found" even for reachable targets.

**Given Code**:
```python
def inverse_kinematics(target_x, target_y, L1, L2):
    r = np.sqrt(target_x**2 + target_y**2)
    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(cos_theta2)  # ERROR HAPPENS HERE
    # ... rest of solution
```

**Tasks**:
1. Identify the bug (5 points)
2. Explain why it fails (5 points)
3. Provide a fix (5 points)

---

## Part 5: Real-World Application (15 points)

**Q5.1** [Difficulty: Advanced] (15 points)
Design a controller for a robot arm to pour water from a pitcher into a cup.

**Scenario**:
- 6-DOF robot arm holds pitcher
- Cup is stationary on table
- Pitcher must tilt smoothly to control flow rate
- Goal: Fill cup without spilling

**Deliverables**:
1. Control strategy description (5 points)
2. Key challenges and solutions (5 points)
3. Pseudocode or block diagram (5 points)
```

### Instructor Answer Key Structure

```markdown
# Chapter X Assessment - INSTRUCTOR ANSWER KEY
**CONFIDENTIAL**

## Part 1: MCQ Solutions

**Q1.1**: B (Correct Answer)
**Explanation**: Forward kinematics maps joint angles í end-effector pose using transformation matrices.
**Common Errors**:
- Students confuse with inverse kinematics (Option A)
**Points**: 2 points for correct answer

---

## Part 2: Coding Solutions

**Q2.1**: Jacobian Computation

**Model Solution**:
```python
import numpy as np

def compute_jacobian(theta, L):
    """Compute Jacobian for 3-DOF planar arm"""
    # Cumulative angles
    q1 = theta[0]
    q2 = theta[0] + theta[1]
    q3 = theta[0] + theta[1] + theta[2]

    # Jacobian columns (partial derivatives)
    J = np.zeros((2, 3))

    # dP/d∏Å
    J[0, 0] = -(L[0]*np.sin(q1) + L[1]*np.sin(q2) + L[2]*np.sin(q3))
    J[1, 0] = L[0]*np.cos(q1) + L[1]*np.cos(q2) + L[2]*np.cos(q3)

    # dP/d∏Ç
    J[0, 1] = -(L[1]*np.sin(q2) + L[2]*np.sin(q3))
    J[1, 1] = L[1]*np.cos(q2) + L[2]*np.cos(q3)

    # dP/d∏É
    J[0, 2] = -L[2]*np.sin(q3)
    J[1, 2] = L[2]*np.cos(q3)

    return J
```

**Grading Rubric** (15 points total):
- Correct cumulative angle calculation (3 points)
- Proper Jacobian structure (2◊3 matrix) (2 points)
- Correct partial derivatives for each column (3 points each = 9 points)
- Code quality (comments, variable names) (1 point)

**Common Mistakes**:
- Forgetting cumulative angles (use ∏_total = ∏Å + ∏Ç + ∏É, not individual ∏b) í Deduct 5 points
- Sign errors in sin/cos í Deduct 2 points per column
- Wrong matrix dimensions í Deduct 3 points

**Partial Credit**:
- Correct approach but implementation errors: 50-80% of points
- Incorrect approach but shows understanding: 20-40% of points

---

## Part 3: Conceptual Answer

**Model Answer**:
"A 6-DOF robot arm experiences singularities when the Jacobian matrix becomes rank-deficient (det(J) = 0), causing loss of mobility in certain directions. This occurs at wrist, elbow, or shoulder singularities where joint axes align or the arm becomes fully extended.

Two avoidance methods:
1. **Singularity-Robust Inverse**: Use damped least squares (J^+ = J^T(JJ^T + ªI)^-1) to prevent numerical instability near singularities
2. **Path Planning with Singularity Avoidance**: Pre-compute workspace regions where det(J) < threshold and constrain paths to avoid these regions"

**Grading**:
- Singularity definition (det(J)=0, loss of mobility): 3 points
- Physical explanation (joint alignment, examples): 3 points
- Method 1 with formula/description: 2 points
- Method 2 with description: 2 points

---

## Part 4: Troubleshooting Solution

**Bug Identification**: `np.arccos()` fails when `cos_theta2` is outside [-1, 1] due to numerical errors or unreachable targets

**Explanation**: Floating-point arithmetic can produce values like cos_theta2 = 1.0000001 or -1.0000001, causing arccos domain error

**Fix**:
```python
cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
cos_theta2 = np.clip(cos_theta2, -1, 1)  # Clamp to valid range
theta2 = np.arccos(cos_theta2)
```

**Grading**:
- Correct bug (arccos domain error): 5 points
- Explanation (numerical precision or unreachable target): 5 points
- Correct fix (clipping or try-except): 5 points

---

## Part 5: Application Solution

**Model Answer**:
1. **Control Strategy**:
   - Use computed torque control for accurate trajectory tracking
   - Implement tilting motion as smooth spline from vertical (0∞) to pour angle (30-45∞)
   - Regulate tilt rate based on flow rate model (F = k∑sin(∏)∑h)

2. **Key Challenges**:
   - Challenge 1: Liquid dynamics (sloshing) - Solution: Slow acceleration/deceleration
   - Challenge 2: Variable pitcher weight - Solution: Adaptive impedance control
   - Challenge 3: Precise positioning - Solution: Vision feedback for cup detection

3. **Pseudocode**:
```
Initialize: pitcher_full_weight, pour_angle_target, flow_rate_desired
While (cup not full):
  current_tilt = get_pitcher_angle()
  flow_rate = estimate_flow(current_tilt, pitcher_weight)
  error = flow_rate_desired - flow_rate
  tilt_adjustment = PID_controller(error)
  target_angle = current_tilt + tilt_adjustment
  execute_trajectory(target_angle, smooth_spline)
End
```

**Grading**:
- Control strategy clarity and feasibility: 5 points
- Identified 2+ real challenges with solutions: 5 points
- Pseudocode or diagram completeness: 5 points
```

## Question Type Guidelines

### 1. MCQs (Multiple Choice Questions)
**Best for**: Recall, recognition, simple application
**Format**: 4 options, 1 correct, 3 plausible distractors
**Difficulty distribution**: 40% beginner, 40% intermediate, 20% advanced

### 2. Coding Tasks
**Best for**: Implementation skills, algorithm understanding
**Provide**: Starter code, test cases, requirements
**Grade on**: Correctness (50%), efficiency (20%), code quality (30%)

### 3. Conceptual Questions
**Best for**: Deep understanding, explanation ability
**Format**: Open-ended, 3-6 sentences expected
**Grade on**: Accuracy, completeness, clarity

### 4. Troubleshooting
**Best for**: Debugging skills, error analysis
**Format**: Buggy code + symptoms, ask to fix
**Grade on**: Bug identification, explanation, fix quality

### 5. Real-World Applications
**Best for**: Synthesis, design, integration
**Format**: Complex scenario requiring multiple concepts
**Grade on**: Feasibility, creativity, technical depth

## Tips for Best Results

1. **Align with Learning Objectives**: Every question should map to a stated objective
2. **Vary Difficulty**: 30% beginner, 50% intermediate, 20% advanced
3. **Provide Context**: Use realistic robotics scenarios
4. **Include Solutions**: Always create instructor answer key
5. **Test Yourself**: Solve your own problems to verify clarity
