---
name: "robotics-code-explainer"
description: "Explain Python robotics code with detailed algorithm breakdowns, line-by-line annotations, mathematical foundations, optimization suggestions, and security analysis. Use when user asks to explain, analyze, or understand robotics code."
version: "1.0.0"
---

# Robotics Code Explanation Skill

## When to Use This Skill

- User pastes robotics code and asks "explain this code" or "how does this work?"
- User wants to understand kinematics, dynamics, control, or planning algorithms
- User needs optimization suggestions for real-time robotics applications
- User requests security analysis for robot control code
- User asks about alternative implementations or better approaches

## How This Skill Works

1. **Identify Algorithm Domain**: Determine if code is kinematics, dynamics, control, perception, or planning
2. **Extract Mathematical Foundation**: Identify core equations and principles being implemented
3. **Provide Line-by-Line Walkthrough**: Explain each code section with technical context
4. **Analyze Performance**: Assess time/space complexity for real-time suitability
5. **Suggest Optimizations**: Provide concrete improvements with expected impact
6. **Check Security**: Flag vulnerabilities in control code

## Output Format

### 1. Algorithm Overview
- **Domain**: Kinematics/Dynamics/Control/Perception/Planning
- **Purpose**: What the code accomplishes
- **Use Case**: Where this would be used in robotics systems

### 2. Mathematical Foundation
- Core equations being implemented
- Theoretical basis (e.g., DH convention, Newton-Euler, Lyapunov)
- Key assumptions or approximations

### 3. Annotated Code Walkthrough
```python
def example_function(params):
    # Line-by-line explanation here
    # Mathematical meaning of each operation
    # Physical interpretation
```

### 4. Performance Analysis
- **Time Complexity**: O(n) analysis for typical robot DOF
- **Space Complexity**: Memory requirements
- **Real-time Suitability**: Can this run at 100-1000 Hz control rates?
- **Bottlenecks**: Identified performance issues

### 5. Optimization Recommendations
**High-Impact**:
- Specific optimization with code example
- Expected speedup (e.g., "2-3x faster")

**Medium-Impact**:
- Additional improvements

**Numerical Stability**:
- Conditioning and error propagation fixes

### 6. Security Check
-   Vulnerabilities found (command injection, buffer overflow, etc.)
-  Security best practices verified
- = Recommended fixes

### 7. Alternative Approaches
- **Approach 1**: Description, pros/cons, when to use
- **Approach 2**: Alternative method comparison

## Example Output

**Input**: "Explain this forward kinematics code"

**Output**:
- Algorithm domain identified: Forward Kinematics using DH parameters
- Mathematical foundation: $T = T_1 \cdot T_2 \cdot ... \cdot T_n$
- Line-by-line walkthrough with physical interpretations
- Performance: O(n) time, suitable for 1000+ Hz control
- Optimizations: Trigonometric caching, vectorization opportunities
- Security:  No issues (pure math, no I/O)
- Alternatives: Product of Exponentials, Dual Quaternions

## Common Robotics Code Patterns Supported

- **Kinematics**: FK, IK (geometric, numerical), Jacobians
- **Dynamics**: Lagrangian, Newton-Euler, recursive algorithms
- **Control**: PID, computed torque, impedance control
- **Planning**: RRT, A*, trajectory optimization
- **Perception**: Kalman filters, particle filters, SLAM
- **Learning**: Neural network controllers, reinforcement learning
