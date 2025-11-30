---
name: "robotics-markdown-formatter"
description: "Format and validate markdown for robotics textbook chapters. Ensures proper heading hierarchy, LaTeX syntax, code blocks, citations, and academic formatting standards. Use when user needs markdown cleaned up or validated."
version: "1.0.0"
---

# Robotics Markdown Formatting Skill

## When to Use This Skill

- User asks to "format this chapter" or "fix markdown syntax"
- User has markdown with formatting errors
- User needs LaTeX equations properly formatted
- User wants consistent academic formatting
- User requests markdown validation before publication

## How This Skill Works

1. **Scan Markdown**: Parse existing markdown structure
2. **Validate Syntax**: Check headings, lists, code blocks, links
3. **Fix LaTeX**: Ensure proper $ and $$ delimiters
4. **Format Citations**: Standardize [Author Year] format
5. **Organize Sections**: Ensure logical heading hierarchy
6. **Output Clean Version**: Return properly formatted markdown

## Formatting Standards

### 1. Heading Hierarchy
```markdown
# Chapter X: Chapter Title

## X.1 Major Section

### X.1.1 Subsection

#### Key Point (use sparingly)
```

**Rules**:
- No skipped levels (# í ### is invalid)
- Chapter title uses #, sections use ##, subsections use ###
- Maximum 4 levels deep

### 2. LaTeX Mathematics

**Inline Math** (within text):
```markdown
The joint angle $\theta_1$ determines the position.
```

**Display Math** (centered equations):
```markdown
$$
J(q) = \frac{\partial f(q)}{\partial q}
$$
```

**Common Fixes**:
- L `$$equation$$` inline í  `$equation$`
- L Unescaped underscores: `theta_1` í  `$\theta_1$` or `theta\_1`
- L Missing closing delimiter í  Balanced $ or $$

### 3. Code Blocks

**Always use language tags**:
```markdown
```python
def forward_kinematics(theta):
    return T_matrix
```
```

**Not** (no language tag):
```markdown
```
def forward_kinematics(theta):
```
```

**Supported languages**: python, cpp, matlab, bash, yaml, json

### 4. Citations

**Standard format**:
```markdown
As shown by [Craig 2005], the DH convention provides...

...proven effective [Spong 2006; Siciliano 2009].
```

**Not**:
- L Craig (2005)
- L (Craig, 2005)
- L [1]

### 5. Lists

**Numbered lists**:
```markdown
1. First item
2. Second item
   - Sub-item with 3-space indent
3. Third item
```

**Bulleted lists**:
```markdown
- Item one
- Item two
  - Nested with 2-space indent
```

### 6. Tables

**Robotics parameter tables**:
```markdown
| Joint | ∏ (deg) | d (m) | a (m) | ± (deg) |
|-------|---------|-------|-------|---------|
| 1     | ∏Å      | 0.5   | 0     | 90      |
| 2     | ∏Ç      | 0     | 0.3   | 0       |
```

**Rules**:
- Header separator row required
- Align columns with spaces for readability
- Use UTF-8 symbols (∏, ±) or LaTeX in cells

### 7. Links and Cross-References

**Internal chapter references**:
```markdown
See [Section 3.4](#34-inverse-kinematics) for details.

As discussed in [Chapter 2](chapter-02-transformations.md)...
```

**External links**:
```markdown
[ROS Documentation](https://www.ros.org/documentation)
```

## Example: Before and After

### Before (Messy Markdown)
```markdown
##Forward Kinematics

The position is calculated using theta_1 and theta_2.

The equation is: $$x = L_1*cos(theta_1) + L_2*cos(theta_1+theta_2)

Code:
```
def fk(t1,t2):
return x,y
```

References: Craig (2005), Spong et. al 2006
```

### After (Clean Markdown)
```markdown
## 3.1 Forward Kinematics

The position is calculated using joint angles $\theta_1$ and $\theta_2$.

The equation is:
$$
x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2)
$$

**Code Implementation**:
```python
def forward_kinematics(theta1, theta2, L1, L2):
    """Compute end-effector position for 2-link arm."""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y
```

**References**: [Craig 2005; Spong 2006]
```

## Validation Checklist

Before considering markdown ready:
- [ ] All headings follow hierarchy (no skipped levels)
- [ ] All code blocks have language tags
- [ ] LaTeX uses proper $ and $$ delimiters
- [ ] No unescaped underscores outside math mode
- [ ] Citations in [Author Year] format
- [ ] Tables have header separators
- [ ] Lists properly indented
- [ ] Internal links use correct anchors
- [ ] No trailing whitespace
- [ ] Consistent line breaks (2 newlines between sections)

## Common Errors Fixed

1. **Mixed inline/display math**: Standardize to $ or $$
2. **Missing language tags on code**: Add appropriate tag
3. **Broken heading hierarchy**: Insert missing levels
4. **Inconsistent citations**: Convert all to [Author Year]
5. **Unescaped special chars**: Escape or wrap in code/math
6. **Malformed tables**: Fix alignment and separators
7. **Broken internal links**: Update anchors to match headings
