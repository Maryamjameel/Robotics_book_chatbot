#!/usr/bin/env python
"""CLI script for inserting embeddings into Qdrant collection."""

import json
import sys
from pathlib import Path

from src.config import logger
from src.models import TextEmbedding
from src.services import initialize_collection, insert_embeddings, verify_insertion


def main():
    """Insert embeddings from JSON file into Qdrant."""
    if len(sys.argv) < 2:
        print("Usage: python insert_qdrant.py <embeddings_file> [collection_name] [--verify]")
        print("Example: python insert_qdrant.py embeddings.json robotics_chapters --verify")
        sys.exit(1)

    embeddings_file = sys.argv[1]
    collection_name = sys.argv[2] if len(sys.argv) > 2 else None
    verify = "--verify" in sys.argv

    logger.info(
        "Starting Qdrant insertion",
        extra={
            "operation": "insert_qdrant",
            "status": "starting",
            "embeddings_file": embeddings_file,
            "verify": verify,
        },
    )

    try:
        # Load embeddings from JSON
        embeddings_path = Path(embeddings_file)
        if not embeddings_path.exists():
            raise FileNotFoundError(f"Embeddings file not found: {embeddings_file}")

        with open(embeddings_path, encoding="utf-8") as f:
            embeddings_data = json.load(f)

        if not embeddings_data:
            raise ValueError("Embeddings file is empty")

        # Convert to TextEmbedding objects
        embeddings = [TextEmbedding(**emb) for emb in embeddings_data]

        logger.info(
            "Loaded embeddings",
            extra={
                "operation": "insert_qdrant",
                "status": "loaded",
                "count": len(embeddings),
            },
        )

        # Initialize collection
        logger.info(
            "Initializing collection",
            extra={
                "operation": "insert_qdrant",
                "status": "initializing",
                "collection": collection_name,
            },
        )

        initialize_collection(collection_name)

        # Insert embeddings
        logger.info(
            "Inserting embeddings",
            extra={
                "operation": "insert_qdrant",
                "status": "inserting",
                "count": len(embeddings),
            },
        )

        result = insert_embeddings(embeddings, collection_name)

        logger.info(
            "Insertion completed",
            extra={
                "operation": "insert_qdrant",
                "status": "inserted",
                "total": result.total,
                "inserted": result.inserted,
                "failed": result.failed,
            },
        )

        print(f"Inserted {result.inserted}/{result.total} embeddings")

        if result.failed > 0:
            print(f"Warning: {result.failed} embeddings failed to insert")
            for error in result.errors:
                print(f"  - {error['chunk_id']}: {error['error']}")

        # Verify if requested
        if verify:
            logger.info(
                "Verifying insertion",
                extra={
                    "operation": "insert_qdrant",
                    "status": "verifying",
                },
            )

            verification = verify_insertion(collection_name, sample_size=50)

            logger.info(
                "Verification completed",
                extra={
                    "operation": "insert_qdrant",
                    "status": "verified",
                    "total_points": verification.total_points,
                    "valid": verification.valid,
                    "invalid": verification.invalid,
                },
            )

            print(f"\nVerification Results:")
            print(f"  Total points: {verification.total_points}")
            print(f"  Sampled: {verification.sampled}")
            print(f"  Valid: {verification.valid}")
            print(f"  Invalid: {verification.invalid}")
            print(f"  Validity: {verification.checks['sample_valid_percentage']:.1f}%")

    except FileNotFoundError as e:
        logger.error(
            "Embeddings file not found",
            extra={
                "operation": "insert_qdrant",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except ValueError as e:
        logger.error(
            "Invalid embeddings data",
            extra={
                "operation": "insert_qdrant",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    except Exception as e:
        logger.error(
            "Failed to insert embeddings",
            extra={
                "operation": "insert_qdrant",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
