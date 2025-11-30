---
name: "robotics-example-generator"
description: "Generate worked examples, case studies, and practice problems for robotics textbook chapters. Creates step-by-step solutions with code implementations. Use when user needs illustrative examples or practice exercises."
version: "1.0.0"
---

# Robotics Example Generator Skill

## When to Use This Skill

- User asks for "worked examples" or "practice problems"
- User needs to illustrate a concept with concrete calculations
- User wants code implementations of algorithms
- User requests case studies or real-world applications
- User is building exercise sets for chapters

## How This Skill Works

1. **Identify Concept**: Determine what robotics topic to exemplify
2. **Select Difficulty**: Choose beginner, intermediate, or advanced level
3. **Design Scenario**: Create realistic robotics context
4. **Develop Solution**: Work through step-by-step
5. **Provide Code**: Implement in Python/MATLAB/ROS
6. **Add Variations**: Offer alternative approaches or edge cases

## Output Format

### Example Structure

**Example X.Y: [Descriptive Title]**

**Difficulty**: Beginner | Intermediate | Advanced

**Concept Illustrated**: [Core concept being taught]

**Problem Statement**:
[Clear description of what needs to be solved]

**Given**:
- Parameter 1: [Value and units]
- Parameter 2: [Value and units]

**Find**:
- [What to calculate]

**Solution Approach**:
[Brief overview of solution strategy]

**Step-by-Step Solution**:

**Step 1**: [First step with explanation]
$$mathematical\_expression$$
[Interpretation]

**Step 2**: [Next step]
...

**Final Answer**:
[Result with units and physical interpretation]

**Code Implementation**:
```python
# Well-commented code
# Following solution steps
```

**Verification**:
[How to check the answer is correct]

**Common Mistakes**:
- Mistake 1: [What students often do wrong]
- Why it's wrong: [Explanation]

**Extensions**:
- What if parameter changed to X?
- How would this work for different robot configuration?

## Example 1: Forward Kinematics

**Example 3.2: 2-Link Planar Arm Forward Kinematics**

**Difficulty**: Beginner

**Concept**: Computing end-effector position from joint angles

**Problem Statement**:
A 2-link planar robot arm has link lengths LÅ = 0.5m and LÇ = 0.3m.
If joint angles are ∏Å = 30∞ and ∏Ç = 45∞, find the end-effector position (x, y).

**Given**:
- LÅ = 0.5 m (first link length)
- LÇ = 0.3 m (second link length)
- ∏Å = 30∞ = ¿/6 rad (first joint angle)
- ∏Ç = 45∞ = ¿/4 rad (second joint angle)

**Find**:
- End-effector position (x, y) in base frame

**Solution Approach**:
Use geometric method: sum position vectors of both links

**Step 1**: Convert angles to radians
- ∏Å = 30∞ ◊ (¿/180) = 0.5236 rad
- ∏Ç = 45∞ ◊ (¿/180) = 0.7854 rad

**Step 2**: Calculate first link endpoint
$$x_1 = L_1 \cos(\theta_1) = 0.5 \times \cos(30∞) = 0.433\text{ m}$$
$$y_1 = L_1 \sin(\theta_1) = 0.5 \times \sin(30∞) = 0.250\text{ m}$$

**Step 3**: Calculate second link contribution (relative to first link)
$$x_2 = L_2 \cos(\theta_1 + \theta_2) = 0.3 \times \cos(75∞) = 0.078\text{ m}$$
$$y_2 = L_2 \sin(\theta_1 + \theta_2) = 0.3 \times \sin(75∞) = 0.290\text{ m}$$

**Step 4**: Sum to get end-effector position
$$x = x_1 + x_2 = 0.433 + 0.078 = 0.511\text{ m}$$
$$y = y_1 + y_2 = 0.250 + 0.290 = 0.540\text{ m}$$

**Final Answer**:
End-effector position: **(0.511 m, 0.540 m)**

This means the robot tip is 0.511m in the x-direction and 0.540m in the y-direction from the base.

**Code Implementation**:
```python
import numpy as np

def forward_kinematics_2link(L1, L2, theta1_deg, theta2_deg):
    """
    Compute FK for 2-link planar arm

    Args:
        L1, L2: Link lengths (meters)
        theta1_deg, theta2_deg: Joint angles (degrees)

    Returns:
        (x, y): End-effector position (meters)
    """
    # Convert to radians
    theta1 = np.radians(theta1_deg)
    theta2 = np.radians(theta2_deg)

    # Calculate end-effector position
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)

    return x, y

# Test with given values
x, y = forward_kinematics_2link(0.5, 0.3, 30, 45)
print(f"End-effector position: ({x:.3f}m, {y:.3f}m)")
# Output: End-effector position: (0.511m, 0.540m)
```

**Verification**:
- Check workspace: (x≤ + y≤) = 0.746m < LÅ + LÇ = 0.8m 
- Check components: x < LÅ + LÇ and y < LÅ + LÇ 

**Common Mistakes**:
- **Mistake**: Forgetting to add angles (using ∏Ç instead of ∏Å + ∏Ç)
  - This gives wrong position for second link
  - Always add angles for serial chains
- **Mistake**: Using degrees in trigonometric functions
  - Python/MATLAB use radians by default
  - Always convert: rad = deg ◊ ¿/180

**Extensions**:
- **What if ∏Ç = -45∞** (elbow down)?
  - x would be larger, y would be smaller
- **How to find workspace boundary?**
  - Maximum reach: LÅ + LÇ = 0.8m (both links aligned)
  - Minimum reach: |LÅ - LÇ| = 0.2m (links opposite)

## Example Types Supported

### 1. Calculation Examples
- Forward/inverse kinematics
- Dynamics (forces, torques)
- Trajectory planning

### 2. Algorithm Examples
- Implementing DH parameters
- PID controller tuning
- Path planning (RRT, A*)

### 3. Code Examples
- Complete implementations
- Function templates
- ROS node examples

### 4. Case Studies
- Industrial robot applications
- Humanoid locomotion scenarios
- Mobile manipulation tasks

### 5. Troubleshooting Examples
- Debugging common errors
- Numerical stability issues
- Real-world constraints

## Tips for Best Results

1. **Specify Difficulty**: "Create a beginner example for..."
2. **Provide Context**: "For a 6-DOF industrial arm..."
3. **Request Variations**: "Show both geometric and numerical solutions"
4. **Ask for Code**: "Include Python implementation"
