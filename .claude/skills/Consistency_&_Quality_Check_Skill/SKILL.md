---
name: "robotics-qa-checker"
description: "Perform comprehensive quality assurance on robotics textbook chapters. Checks technical accuracy, terminology consistency, citation format, LaTeX syntax, code validity, and cross-references. Use when reviewing or validating chapter content."
version: "1.0.0"
---

# Robotics Quality Assurance & Consistency Check Skill

## When to Use This Skill

- User asks to "review this chapter" or "check quality"
- User wants to validate technical accuracy before publication
- User needs consistency checks across multiple chapters
- User requests formatting validation (markdown, LaTeX, citations)
- User wants to ensure glossary terms are used correctly

## How This Skill Works

1. **Automated Checks**: Run syntax and format validations
2. **Technical Review**: Verify equations, algorithms, and concepts
3. **Consistency Audit**: Check terminology and notation alignment
4. **Citation Validation**: Ensure references are complete and formatted correctly
5. **Cross-Reference Check**: Verify internal links and chapter references
6. **Generate Report**: Provide actionable findings with severity ratings

## Output Format

### Executive Summary
- **Overall Status**: Ready / Needs Minor Revisions / Needs Major Revisions / Requires Rework
- **Critical Issues**: [Count]
- **Major Issues**: [Count]
- **Minor Issues**: [Count]
- **Top 3 Strengths**
- **Top 3 Priority Fixes**

### Automated Validation Results

#### Markdown Syntax / /L
- Heading hierarchy check
- Code fence balance
- List formatting
- Link validity

#### LaTeX/Math Validation / /L
- Inline math ($...$) syntax
- Display math ($$...$$) syntax
- Bracket/brace matching
- Common errors flagged

#### Code Block Validation / /L
- Language tags present
- Syntax validity
- Consistent indentation

#### Citations / /L
- [Author Year] format consistency
- All citations in References section
- No duplicate citations

### Technical Accuracy Review

**Equations**:
-  Correct: [Equation X.Y]
-   Verify: [Equation X.Z - check units]
- L Error: [Equation X.W - sign error in line 3]

**Algorithms**:
- Pseudocode correctness
- Complexity analysis accuracy
- Edge case coverage

**Terminology**:
-  Consistent usage
-   Inconsistencies found: [List]
- Missing glossary entries: [Terms]

### Consistency Findings

**Notation**:
- ¸ vs theta: [Chapter uses both - standardize]
- Bold vectors: **q** vs q [Inconsistent]

**Terminology**:
- "end-effector" vs "end effector": [15 instances of variation]
- Recommendation: Use "end-effector" (matches glossary)

**Cross-References**:
-  Chapter 3 reference valid
- L "See Section 7.5" - Section 7.5 doesn't exist

### Detailed Issues by Severity

#### Critical Issues (Blocks Publication)
1. **[Line 147]**: Equation 3.15 has dimensional mismatch
   - **Issue**: Left side is 3×1, right side is 4×1
   - **Fix**: Add homogeneous coordinate [0] to left side

#### Major Issues (Requires Revision)
1. **[Section 3.4]**: Jacobian singularity discussion incomplete
   - **Issue**: Mentions singularities but doesn't define or classify them
   - **Fix**: Add subsection 3.4.3 defining wrist/elbow/shoulder singularities

2. **[Lines 89-92]**: Citation format inconsistent
   - **Issue**: Uses "Craig (2005)" instead of [Craig 2005]
   - **Fix**: Change to standard bracket format

#### Minor Issues (Polish/Enhancement)
1. **[Line 23]**: Heading level skipped (jumps from ## to ####)
   - **Fix**: Change #### to ###

2. **[Code Block at line 156]**: Missing language tag
   - **Fix**: Change ```to ```python

### Recommendations

**Immediate Actions**:
1. Fix critical equation error (Line 147)
2. Standardize citation format throughout
3. Add missing singularity definitions

**Before Next Review**:
- Run automated markdown linter
- Cross-check all equation dimensions
- Verify all cross-references resolve

**Optional Improvements**:
- Add diagram descriptions for figures
- Expand worked examples with more steps
- Include common student errors in examples

## Example Usage

**Input**: "Check this chapter on robot kinematics for quality"

**Output**:
```
Executive Summary:
- Status: Needs Minor Revisions
- Critical: 1, Major: 3, Minor: 8
- Strengths: Clear explanations, good examples, comprehensive coverage
- Priority: Fix equation 3.15, add singularity definitions, standardize notation

Automated Checks:
 Markdown syntax valid
  LaTeX: 2 equations missing closing braces
 Code blocks: All have language tags
  Citations: 5 instances of incorrect format

Technical Review:
- 12 equations checked, 1 error found
- DH parameter derivation verified correct
- Jacobian computation algorithm validated

Consistency:
- Terminology: 3 inconsistencies (end-effector vs end effector)
- Notation: ¸ used throughout (consistent )
- Cross-refs: All valid except 1 broken link

[Detailed breakdown follows...]
```

## Quality Dimensions Checked

1. **Correctness**: Technical accuracy, equations, algorithms
2. **Structure**: Logical flow, section hierarchy, learning progression
3. **Clarity**: Readability, explanation quality, terminology precision
4. **Consistency**: Notation, terminology, formatting, voice/tone
5. **Formatting**: Academic standards, markdown, LaTeX, citations
6. **Completeness**: All learning outcomes addressed, examples included

## Tips for Best Results

1. **Provide Full Chapter**: Complete content gives better context
2. **Mention Target Audience**: Helps assess appropriate technical depth
3. **Specify Publication Deadline**: Prioritizes critical vs. optional fixes
4. **Reference Style Guide**: If you have specific formatting requirements
