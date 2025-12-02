---
name: "robotics-retrieval-query"
description: "Query the robotics book knowledge base using semantic search. Retrieves relevant passages, definitions, and explanations from book chapters with source citations. Use when user asks questions about book content."
version: "1.0.0"
---

# Robotics Book Retrieval Query Skill

## When to Use This Skill

- User asks "What does the book say about [topic]?"
- User needs information from specific chapters
- User wants definitions of robotics terms from the book
- User requests explanations grounded in book content
- User is studying and needs to look up concepts

## How This Skill Works

1. **Parse Query**: Extract key concepts and technical terms
2. **Search Book Content**: Use semantic search across chapters
3. **Retrieve Passages**: Find most relevant sections (top 5-10)
4. **Filter & Rank**: Select passages that best answer question
5. **Synthesize Answer**: Combine information from multiple sources
6. **Cite Sources**: Provide chapter/section references

## Output Format

### Direct Answer
[Concise 1-2 sentence answer to the question]

### Detailed Explanation
[Comprehensive explanation synthesized from book content]

**Key Points**:
- Point 1 from book [Chapter X, Section Y]
- Point 2 from book [Chapter Z]
- Point 3 from book [Page N]

### Relevant Equations (if applicable)
$$equation_{from\_book}$$  [Chapter X.Y]

### Related Concepts
- Concept A: [Brief note with source]
- Concept B: [Brief note with source]

### Sources
- **Chapter 3, Section 3.4**: "Inverse Kinematics" (pages 45-52)
- **Chapter 5, Section 5.2**: "Dynamics Formulation" (pages 89-95)

## Example Response

**Query**: "What does the book say about inverse kinematics?"

**Direct Answer**:
The book describes inverse kinematics as computing joint angles that achieve a desired end-effector posethe reverse of forward kinematics.

**Detailed Explanation**:
Chapter 3 presents two main IK approaches: analytical (geometric closed-form solutions for simple robots) and numerical (Jacobian-based iterative methods). The book emphasizes that most 6-DOF robots have multiple IK solutions (typically 8) and discusses solution selection criteria.

**Key Points**:
- Analytical methods use trigonometry for specific geometries [Ch 3.4.1]
- Numerical methods use $\Delta q = J^+ \Delta x$ for any robot [Ch 3.4.2]
- Multiple solutions require selection strategies [Ch 3.4.3]

**Relevant Equations**:
$$\Delta q = J^+(\Delta x)$$ where $J^+ = J^T(JJ^T + \lambda I)^{-1}$ [Ch 3.4.2]

**Sources**:
- **Chapter 3, Section 3.4**: "Inverse Kinematics" (pages 45-58)

## Search Strategies

1. **Keyword Search**: Exact term matches
2. **Concept Search**: Semantic search across sections
3. **Chapter-Specific**: Constrained to specified chapter
4. **Cross-Chapter**: Synthesizes from multiple chapters

## Tips for Best Results

- Be specific with technical terms
- Specify chapters if known
- Ask follow-up questions for clarification
- Request examples or equations from specific sections
