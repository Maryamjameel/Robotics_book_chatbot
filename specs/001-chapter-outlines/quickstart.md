# Quickstart: Generating Chapter Outlines

This guide provides a quick overview of how to generate chapter outlines using the `Chapter_Outline_Skill`.

## Prerequisites

- Access to the `Chapter_Outline_Skill` (typically invoked via a specialized agent or tool).
- A Docusaurus project setup where generated markdown files can be stored.

## How to Use

1.  **Define Chapter Requirements**: Prepare the necessary details for each chapter, including its title, covered topics, and specific requirements for learning outcomes, examples, equations, and code. This information will be provided as input to the `Chapter_Outline_Skill`.

2.  **Invoke the `Chapter_Outline_Skill`**: Call the skill with the chapter requirements. The specific invocation method will depend on the environment (e.g., through a CLI command, an API endpoint, or an agent prompt).

    *Example (conceptual invocation):*
    ```
    invoke_skill("Chapter_Outline_Skill", {
      chapter_number: 1,
      title: "Introduction to Physical AI & ROS 2",
      covers: "Foundations of Physical AI (Weeks 1-2) + ROS 2 Fundamentals (Weeks 3-5)",
      topics: ["Embodied intelligence", "ROS 2 architecture", "nodes/topics/services", "URDF"],
      # ... other required parameters as per the skill's interface
    })
    ```

3.  **Retrieve and Verify Output**: The skill will generate a markdown file for each chapter outline. These files will be saved to the `frontend/docs/chapters/` directory with the naming convention `chapter-0X-outline.md`.

4.  **Integrate with Docusaurus**: Ensure your `docusaurus.config.ts` and `sidebars.ts` are configured to automatically pick up new markdown files in the `frontend/docs/chapters/` directory for navigation and display.

## Example Outline Structure

The generated markdown will follow a structure similar to this:

```markdown
# Chapter 1: Introduction to Physical AI & ROS 2

## Learning Outcomes

- Outcome 1
- Outcome 2

## Section 1: Foundations of Physical AI

### Key Concepts
- Concept A
- Concept B

### Worked Example: Basic Robot Control

### Equations & Algorithms
- Equation [X]
- Algorithm [Y]

### Code Example: ROS 2 Publisher/Subscriber (Python)

## Section 2: ROS 2 Fundamentals

# ... and so on
```