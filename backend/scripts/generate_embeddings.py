#!/usr/bin/env python
"""CLI script for generating embeddings from chunks."""

import json
import sys
from pathlib import Path

from src.config import logger
from src.models import ChapterChunk
from src.services import embed_batch, embed_chunks


def main():
    """Generate embeddings from chunks JSON file."""
    if len(sys.argv) < 2:
        print("Usage: python generate_embeddings.py <chunks_file> [output_file]")
        print("Example: python generate_embeddings.py chunks.json embeddings.json")
        sys.exit(1)

    chunks_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "embeddings.json"

    logger.info(
        "Starting embedding generation",
        extra={
            "operation": "generate_embeddings",
            "status": "starting",
            "chunks_file": chunks_file,
        },
    )

    try:
        # Load chunks from JSON
        chunks_path = Path(chunks_file)
        if not chunks_path.exists():
            raise FileNotFoundError(f"Chunks file not found: {chunks_file}")

        with open(chunks_path, encoding="utf-8") as f:
            chunks_data = json.load(f)

        if not chunks_data:
            raise ValueError("Chunks file is empty")

        # Convert to ChapterChunk objects
        chunks = [ChapterChunk(**chunk) for chunk in chunks_data]

        logger.info(
            "Loaded chunks",
            extra={
                "operation": "generate_embeddings",
                "status": "loaded",
                "count": len(chunks),
            },
        )

        # Generate embeddings
        embeddings = embed_chunks(chunks)

        if not embeddings:
            logger.warning(
                "No embeddings generated",
                extra={
                    "operation": "generate_embeddings",
                    "status": "warning",
                },
            )
            print("No embeddings generated", file=sys.stderr)
            sys.exit(1)

        # Save embeddings as JSON
        output_path = Path(output_file)
        embeddings_data = [embedding.model_dump() for embedding in embeddings]

        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(embeddings_data, f, indent=2)

        logger.info(
            "Embedding generation completed",
            extra={
                "operation": "generate_embeddings",
                "status": "success",
                "embeddings_count": len(embeddings),
                "output_file": str(output_path),
            },
        )

        print(f"Successfully generated {len(embeddings)} embeddings to {output_file}")

    except FileNotFoundError as e:
        logger.error(
            "Chunks file not found",
            extra={
                "operation": "generate_embeddings",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except ValueError as e:
        logger.error(
            "Invalid chunks data",
            extra={
                "operation": "generate_embeddings",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except Exception as e:
        logger.error(
            "Failed to generate embeddings",
            extra={
                "operation": "generate_embeddings",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
