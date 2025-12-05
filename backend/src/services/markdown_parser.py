"""Markdown parser service for extracting chapters and sections from markdown files."""

import re
from pathlib import Path
from typing import List

from ..models import ChapterChunk


def parse_chapter(chapter_path: str, chapter_id: str) -> List[ChapterChunk]:
    """
    Parse a single markdown chapter file and extract sections.

    Args:
        chapter_path: Path to the markdown file
        chapter_id: Unique identifier for the chapter

    Returns:
        List of ChapterChunk objects extracted from the markdown file

    Raises:
        FileNotFoundError: If the chapter file does not exist
        ValueError: If the markdown format is invalid
    """
    path = Path(chapter_path)
    if not path.exists():
        raise FileNotFoundError(f"Chapter file not found: {chapter_path}")

    content = path.read_text(encoding="utf-8")
    if not content.strip():
        raise ValueError(f"Chapter file is empty: {chapter_path}")

    chunks = []

    # Extract chapter title from first H1
    title_match = re.search(r"^#\s+(.+?)$", content, re.MULTILINE)
    chapter_title = title_match.group(1).strip() if title_match else "Untitled"

    # Split by ## (section headers)
    section_pattern = r"^##\s+(.+?)$"
    sections = re.split(section_pattern, content, flags=re.MULTILINE)

    # sections[0] is content before first ##, sections[1] is first title, sections[2] is first content, etc.
    section_number = 0

    # Process sections (skip first element which is preamble before first ##)
    for i in range(1, len(sections), 2):
        if i + 1 < len(sections):
            section_title = sections[i].strip()
            section_content = sections[i + 1].strip()

            if section_content:  # Only include non-empty sections
                section_number += 1
                chunk = ChapterChunk(
                    chapter_id=chapter_id,
                    chapter_title=chapter_title,
                    section_number=section_number,
                    section_title=section_title,
                    content=section_content,
                    metadata={
                        "source_file": path.name,
                        "chapter_id": chapter_id,
                        "section_number": section_number,
                        "section_title": section_title,
                    },
                )
                chunks.append(chunk)

    if not chunks:
        raise ValueError(f"No sections found in chapter: {chapter_path}")

    return chunks


def parse_all_chapters(chapters_dir: str) -> List[ChapterChunk]:
    """
    Parse all markdown files in a directory and extract sections.

    Args:
        chapters_dir: Directory containing markdown chapter files

    Returns:
        List of all ChapterChunk objects from all chapters

    Raises:
        FileNotFoundError: If the directory does not exist
    """
    dir_path = Path(chapters_dir)
    if not dir_path.exists():
        raise FileNotFoundError(f"Chapters directory not found: {chapters_dir}")

    if not dir_path.is_dir():
        raise ValueError(f"Path is not a directory: {chapters_dir}")

    all_chunks = []

    # Find all markdown files
    md_files = sorted(dir_path.glob("*.md"))

    for md_file in md_files:
        # Extract chapter ID from filename (e.g., "ch01.md" -> "ch01")
        chapter_id = md_file.stem

        try:
            chunks = parse_chapter(str(md_file), chapter_id)
            all_chunks.extend(chunks)
        except (FileNotFoundError, ValueError) as e:
            # Log but continue processing other files
            print(f"Warning: Failed to parse {md_file}: {e}")
            continue

    return all_chunks
