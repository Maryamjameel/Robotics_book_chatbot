#!/usr/bin/env python
"""CLI script for parsing markdown chapters and generating chunks."""

import json
import sys
from pathlib import Path

from src.config import logger
from src.services import parse_all_chapters


def main():
    """Parse markdown chapters and output chunks as JSON."""
    if len(sys.argv) < 2:
        print("Usage: python chunk_chapters.py <chapters_directory> [output_file]")
        print("Example: python chunk_chapters.py ../data/chapters chunks.json")
        sys.exit(1)

    chapters_dir = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "chunks.json"

    logger.info(
        "Starting chapter chunking",
        extra={
            "operation": "chunk_chapters",
            "status": "starting",
            "chapters_dir": chapters_dir,
        },
    )

    try:
        # Parse all chapters
        chunks = parse_all_chapters(chapters_dir)

        if not chunks:
            logger.warning(
                "No chunks found in chapters",
                extra={
                    "operation": "chunk_chapters",
                    "status": "warning",
                },
            )
            print("No chunks found", file=sys.stderr)
            sys.exit(1)

        # Save chunks as JSON
        output_path = Path(output_file)
        chunks_data = [chunk.model_dump() for chunk in chunks]

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(chunks_data, f, indent=2)

        logger.info(
            "Chapter chunking completed",
            extra={
                "operation": "chunk_chapters",
                "status": "success",
                "chunks_count": len(chunks),
                "output_file": str(output_path),
            },
        )

        print(f"Successfully chunked {len(chunks)} sections into {output_file}")

    except FileNotFoundError as e:
        logger.error(
            "Chapters directory not found",
            extra={
                "operation": "chunk_chapters",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except Exception as e:
        logger.error(
            "Failed to chunk chapters",
            extra={
                "operation": "chunk_chapters",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
