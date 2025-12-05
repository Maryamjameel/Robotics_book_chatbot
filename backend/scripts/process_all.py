#!/usr/bin/env python
"""CLI script for running the complete embedding pipeline."""

import json
import sys
import tempfile
from pathlib import Path

from src.config import logger
from src.models import ChapterChunk, TextEmbedding
from src.services import (
    embed_chunks,
    initialize_collection,
    insert_embeddings,
    parse_all_chapters,
    verify_insertion,
)


def main():
    """Run the complete embedding pipeline: parse -> embed -> insert -> verify."""
    if len(sys.argv) < 2:
        print("Usage: python process_all.py <chapters_directory> [collection_name] [--verify]")
        print("Example: python process_all.py ../data/chapters robotics_chapters --verify")
        sys.exit(1)

    chapters_dir = sys.argv[1]
    collection_name = sys.argv[2] if len(sys.argv) > 2 else None
    verify = "--verify" in sys.argv

    logger.info(
        "Starting complete pipeline",
        extra={
            "operation": "process_all",
            "status": "starting",
            "chapters_dir": chapters_dir,
            "verify": verify,
        },
    )

    try:
        # Step 1: Parse chapters
        logger.info(
            "Step 1: Parsing chapters",
            extra={
                "operation": "process_all",
                "status": "parsing",
                "chapters_dir": chapters_dir,
            },
        )

        chapters = parse_all_chapters(chapters_dir)

        if not chapters:
            raise ValueError("No chapters found to process")

        logger.info(
            "Parsed chapters",
            extra={
                "operation": "process_all",
                "status": "parsed",
                "chunks_count": len(chapters),
            },
        )

        print(f"Step 1 ✓ Parsed {len(chapters)} sections from chapters")

        # Step 2: Generate embeddings
        logger.info(
            "Step 2: Generating embeddings",
            extra={
                "operation": "process_all",
                "status": "embedding",
                "chunks_count": len(chapters),
            },
        )

        embeddings = embed_chunks(chapters)

        if not embeddings:
            raise ValueError("No embeddings generated")

        logger.info(
            "Generated embeddings",
            extra={
                "operation": "process_all",
                "status": "embedded",
                "embeddings_count": len(embeddings),
            },
        )

        print(f"Step 2 ✓ Generated {len(embeddings)} embeddings")

        # Step 3: Initialize collection
        logger.info(
            "Step 3: Initializing Qdrant collection",
            extra={
                "operation": "process_all",
                "status": "initializing",
                "collection": collection_name,
            },
        )

        initialize_collection(collection_name)

        logger.info(
            "Initialized collection",
            extra={
                "operation": "process_all",
                "status": "initialized",
                "collection": collection_name,
            },
        )

        print(f"Step 3 ✓ Initialized Qdrant collection")

        # Step 4: Insert embeddings
        logger.info(
            "Step 4: Inserting embeddings into Qdrant",
            extra={
                "operation": "process_all",
                "status": "inserting",
                "count": len(embeddings),
            },
        )

        result = insert_embeddings(embeddings, collection_name)

        logger.info(
            "Inserted embeddings",
            extra={
                "operation": "process_all",
                "status": "inserted",
                "total": result.total,
                "inserted": result.inserted,
                "failed": result.failed,
            },
        )

        print(f"Step 4 ✓ Inserted {result.inserted}/{result.total} embeddings")

        if result.failed > 0:
            print(f"⚠ Warning: {result.failed} embeddings failed")

        # Step 5: Verify (optional)
        if verify:
            logger.info(
                "Step 5: Verifying insertion",
                extra={
                    "operation": "process_all",
                    "status": "verifying",
                },
            )

            verification = verify_insertion(collection_name, sample_size=50)

            logger.info(
                "Verified insertion",
                extra={
                    "operation": "process_all",
                    "status": "verified",
                    "total_points": verification.total_points,
                    "valid": verification.valid,
                    "invalid": verification.invalid,
                },
            )

            print(f"Step 5 ✓ Verification completed")
            print(f"  Total points: {verification.total_points}")
            print(f"  Validity: {verification.checks['sample_valid_percentage']:.1f}%")

        # Pipeline complete
        logger.info(
            "Complete pipeline finished",
            extra={
                "operation": "process_all",
                "status": "success",
                "chunks": len(chapters),
                "embeddings": len(embeddings),
                "inserted": result.inserted,
            },
        )

        print("\n✓ Pipeline completed successfully!")
        print(f"  Chapters parsed: {len(chapters)}")
        print(f"  Embeddings generated: {len(embeddings)}")
        print(f"  Embeddings inserted: {result.inserted}")

    except FileNotFoundError as e:
        logger.error(
            "Directory not found",
            extra={
                "operation": "process_all",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except ValueError as e:
        logger.error(
            "Invalid input",
            extra={
                "operation": "process_all",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except Exception as e:
        logger.error(
            "Pipeline failed",
            extra={
                "operation": "process_all",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
