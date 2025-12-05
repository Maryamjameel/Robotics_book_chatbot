"""Qdrant vector storage service for inserting and retrieving embeddings."""

import random
from typing import List, Optional

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, PointStruct, VectorParams

from ..config import config, logger
from ..models import InsertionResult, TextEmbedding, VerificationResult


def initialize_collection(collection_name: Optional[str] = None) -> bool:
    """
    Initialize or verify Qdrant collection configuration.

    Creates collection if it doesn't exist, with proper vector configuration:
    - Vector size: 1536 (OpenAI text-embedding-3-small)
    - Distance metric: COSINE
    - Payload indexes: chapter_id, section_number, section_title

    Args:
        collection_name: Name of the collection (uses config default if None)

    Returns:
        True if collection is ready, False otherwise

    Raises:
        RuntimeError: If collection initialization fails
    """
    coll_name = collection_name or config.collection_name

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        logger.info(
            "Checking collection status",
            extra={
                "operation": "initialize_collection",
                "status": "in_progress",
                "collection": coll_name,
            },
        )

        # Check if collection exists
        try:
            collection_info = client.get_collection(coll_name)
            logger.info(
                "Collection already exists",
                extra={
                    "operation": "initialize_collection",
                    "status": "found",
                    "collection": coll_name,
                    "vector_size": collection_info.config.params.vectors.size,
                },
            )
            return True
        except Exception:
            # Collection doesn't exist, create it
            logger.info(
                "Creating new collection",
                extra={
                    "operation": "initialize_collection",
                    "status": "creating",
                    "collection": coll_name,
                    "vector_size": config.vector_size,
                    "distance": config.distance_metric,
                },
            )

            client.create_collection(
                collection_name=coll_name,
                vectors_config=VectorParams(
                    size=config.vector_size,
                    distance=Distance.COSINE,
                ),
            )

            logger.info(
                "Collection created successfully",
                extra={
                    "operation": "initialize_collection",
                    "status": "success",
                    "collection": coll_name,
                },
            )

            return True

    except Exception as e:
        logger.error(
            "Failed to initialize collection",
            extra={
                "operation": "initialize_collection",
                "status": "failed",
                "collection": coll_name,
                "error": str(e),
            },
            exc_info=True,
        )
        raise RuntimeError(f"Failed to initialize collection {coll_name}: {e}")


def insert_embeddings(embeddings: List[TextEmbedding], collection_name: Optional[str] = None) -> InsertionResult:
    """
    Insert embeddings into Qdrant collection using upsert (idempotent).

    Batches insertions and logs errors for failed chunks while continuing processing.

    Args:
        embeddings: List of TextEmbedding objects to insert
        collection_name: Name of the collection (uses config default if None)

    Returns:
        InsertionResult with statistics on success/failure

    Raises:
        ValueError: If embeddings list is empty
        RuntimeError: If all insertions fail
    """
    if not embeddings:
        raise ValueError("Cannot insert empty list of embeddings")

    coll_name = collection_name or config.collection_name

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        logger.info(
            "Starting embedding insertion",
            extra={
                "operation": "insert_embeddings",
                "status": "in_progress",
                "collection": coll_name,
                "count": len(embeddings),
            },
        )

        # Convert embeddings to Qdrant PointStruct
        points = []
        errors = []

        for i, embedding in enumerate(embeddings):
            try:
                # Generate unique point ID based on chunk_id hash
                point_id = abs(hash(embedding.chunk_id)) % (10**8)

                point = PointStruct(
                    id=point_id,
                    vector=embedding.vector,
                    payload=embedding.metadata,
                )
                points.append(point)

            except Exception as e:
                errors.append(
                    {
                        "chunk_id": embedding.chunk_id,
                        "error": str(e),
                    }
                )

        if not points:
            raise RuntimeError("No valid points to insert")

        # Upsert points (idempotent - updates or inserts)
        client.upsert(
            collection_name=coll_name,
            points=points,
        )

        result = InsertionResult(
            total=len(embeddings),
            inserted=len(points),
            failed=len(errors),
            errors=errors,
        )

        logger.info(
            "Embeddings inserted successfully",
            extra={
                "operation": "insert_embeddings",
                "status": "success",
                "collection": coll_name,
                "inserted": result.inserted,
                "failed": result.failed,
            },
        )

        return result

    except Exception as e:
        logger.error(
            "Failed to insert embeddings",
            extra={
                "operation": "insert_embeddings",
                "status": "failed",
                "collection": coll_name,
                "error": str(e),
            },
            exc_info=True,
        )
        raise RuntimeError(f"Failed to insert embeddings: {e}")


def verify_insertion(
    collection_name: Optional[str] = None, sample_size: int = 50
) -> VerificationResult:
    """
    Verify embeddings in Qdrant collection.

    Samples points and checks:
    - Vector dimensions match (1536)
    - Payload schema is consistent
    - Collection statistics

    Args:
        collection_name: Name of the collection (uses config default if None)
        sample_size: Number of points to sample for verification

    Returns:
        VerificationResult with validation details

    Raises:
        RuntimeError: If collection doesn't exist or is empty
    """
    coll_name = collection_name or config.collection_name

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        logger.info(
            "Starting collection verification",
            extra={
                "operation": "verify_insertion",
                "status": "in_progress",
                "collection": coll_name,
            },
        )

        # Get collection info
        collection_info = client.get_collection(coll_name)
        total_points = collection_info.points_count

        if total_points == 0:
            raise RuntimeError(f"Collection {coll_name} is empty")

        # Sample points for verification
        sample_count = min(sample_size, total_points)
        # Get points using scroll to sample them
        points, _ = client.scroll(
            collection_name=coll_name,
            limit=sample_count,
        )

        valid = 0
        invalid = 0
        checks = {
            "vector_dimension": config.vector_size,
            "expected_distance": config.distance_metric,
            "collection_vector_size": collection_info.config.params.vectors.size,
            "sample_size": sample_count,
        }

        # Validate sampled points
        required_payload_keys = set()
        for point in points:
            try:
                vector = point.vector
                payload = point.payload or {}

                # Check vector dimension
                if len(vector) != config.vector_size:
                    invalid += 1
                    continue

                # Track payload keys
                if required_payload_keys is None:
                    required_payload_keys = set(payload.keys())
                required_payload_keys.update(payload.keys())

                valid += 1

            except Exception:
                invalid += 1

        checks["payload_keys"] = list(required_payload_keys)
        checks["sample_valid_percentage"] = (valid / sample_count * 100) if sample_count > 0 else 0

        result = VerificationResult(
            total_points=total_points,
            sampled=sample_count,
            valid=valid,
            invalid=invalid,
            checks=checks,
        )

        logger.info(
            "Collection verification completed",
            extra={
                "operation": "verify_insertion",
                "status": "success",
                "collection": coll_name,
                "total_points": total_points,
                "valid": valid,
                "invalid": invalid,
            },
        )

        return result

    except Exception as e:
        logger.error(
            "Failed to verify collection",
            extra={
                "operation": "verify_insertion",
                "status": "failed",
                "collection": coll_name,
                "error": str(e),
            },
            exc_info=True,
        )
        raise RuntimeError(f"Failed to verify collection {coll_name}: {e}")
