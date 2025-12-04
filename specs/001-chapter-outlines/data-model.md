# Data Model: Chapter Outlines

## Entity: Chapter Outline

**Description**: Represents a structured outline for a single chapter of the Physical AI & Humanoid Robotics textbook.

**Fields (conceptual, not implementation-specific)**:
- **Chapter Number**: (Integer) Unique identifier for the chapter (e.g., 1, 2, 3).
- **Chapter Title**: (String) Main title of the chapter.
- **Learning Outcomes**: (List of Strings) 3-5 measurable learning objectives for the chapter.
- **Sections**: (List of Section Objects) Hierarchical structure of the chapter content.
  - **Section Object Fields**:
    - **Title**: (String) Title of the section.
    - **Level**: (Integer) Hierarchy level (e.g., 2 for ##, 3 for ###).
    - **Key Concepts**: (List of Strings) Important terms and ideas covered in the section.
    - **Worked Examples**: (List of Example Objects) Plans for practical demonstrations.
      - **Example Object Fields**:
        - **Title**: (String) Title of the example.
        - **Description**: (String) Brief description of the example.
        - **Code Language**: (String) e.g., "Python", "ROS 2".
    - **Equations/Algorithms**: (List of Strings) Descriptions or LaTeX representations of equations and algorithms.
    - **Code Examples**: (List of Code Snippet Objects) Plans for relevant code snippets.
      - **Code Snippet Object Fields**:
        - **Description**: (String) What the code demonstrates.
        - **Language**: (String) e.g., "Python", "ROS 2".
- **Output Format**: Markdown file (`.md` extension).

**Relationships**: None directly within this feature, as outlines are standalone content artifacts.

**Validation Rules (applied by Chapter_Outline_Skill)**:
- Chapter Number must be unique.
- Each outline must contain 3-5 learning outcomes.
- Section hierarchy must be logical (e.g., H3 must follow an H2).
- All required fields (title, concepts, etc.) must be present.

**Notes**: The `Chapter_Outline_Skill` is responsible for generating and validating the internal consistency of the Chapter Outline structure before writing to the filesystem.