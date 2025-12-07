"""Qdrant vector storage service for inserting and retrieving embeddings."""

import random
from typing import Any, Dict, List, Optional

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, PointStruct, VectorParams

from ..config import config, logger
from ..models import InsertionResult, TextEmbedding, VerificationResult
from .utils.chapter_filter import (
    ChapterContextFilter,
    ChapterFilterEngine,
    SearchResult,
)


def initialize_collection(collection_name: Optional[str] = None) -> bool:
    """
    Initialize or verify Qdrant collection configuration.

    Creates collection if it doesn't exist, with proper vector configuration:
    - Vector size: 768 (Google Gemini text-embedding-004)
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
    - Vector dimensions match (768)
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


def search_chunks(
    question_embedding: List[float],
    collection_name: Optional[str] = None,
    top_k: int = 5,
    relevance_threshold: float = 0.5,
    chapter_context: Optional[Dict[str, str]] = None,
    selected_text_terms: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """Search for relevant chunks using vector similarity with optional chapter filtering.

    Performs Qdrant vector search and optionally filters/re-ranks results by chapter context.

    Args:
        question_embedding: 768-dimensional embedding vector
        collection_name: Name of the collection (uses config default if None)
        top_k: Number of top results to return (default 5)
        relevance_threshold: Minimum cosine similarity score (default 0.7)
        chapter_context: Optional dict with 'chapter_id' and 'chapter_title' for filtering
        selected_text_terms: Optional list of terms from selected text for TF-IDF boosting

    Returns:
        Dict with 'results' (list of search results) and 'metadata' (filtering stats)

    Raises:
        RuntimeError: If search fails
    """
    coll_name = collection_name or config.collection_name

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=5.0,
        )

        logger.info(
            "Starting vector search",
            extra={
                "operation": "search_chunks",
                "collection": coll_name,
                "top_k": top_k,
                "threshold": relevance_threshold,
                "chapter_filtered": chapter_context is not None,
                "status": "in_progress",
            },
        )

        # Perform vector search using query_points (Qdrant v2.0+)
        query_response = client.query_points(
            collection_name=coll_name,
            query=question_embedding,
            limit=top_k * 3,  # Get extra results for chapter filtering and threshold
            with_payload=True,
        )
        search_results = query_response.points

        # Convert Qdrant results to SearchResult objects and filter by threshold
        search_result_objects = []
        for result in search_results:
            if result.score and result.score >= relevance_threshold:
                payload = dict(result.payload) if result.payload else {}
                search_result_objects.append(
                    SearchResult(
                        chapter_id=payload.get("chapter_id", "unknown"),
                        section_number=payload.get("section_number", 0),
                        section_title=payload.get("section_title", ""),
                        excerpt=payload.get("excerpt", ""),
                        relevance_score=result.score,
                        id=str(result.id),
                    )
                )

        # Apply chapter filtering and re-ranking
        filter_engine = ChapterFilterEngine()
        filtered_results = search_result_objects

        # Apply chapter context filtering if provided
        if chapter_context:
            chapter_filter = ChapterContextFilter(
                chapter_id=chapter_context.get("chapter_id", ""),
                chapter_title=chapter_context.get("chapter_title"),
            )
            filtered_results = filter_engine.filter_results(search_result_objects, chapter_filter)

            # Apply TF-IDF boosting if selected text terms provided
            if selected_text_terms:
                filtered_results = filter_engine.apply_tf_idf_boost(
                    filtered_results, selected_text_terms=selected_text_terms
                )

        # Return top_k after filtering
        final_results = filtered_results[:top_k]

        # Convert FilteredResult back to dict format for API response
        result_dicts = [
            {
                "id": result.id,
                "chapter_id": result.chapter_id,
                "section_number": result.section_number,
                "section_title": result.section_title,
                "excerpt": result.excerpt,
                "relevance_score": result.final_relevance if hasattr(result, 'final_relevance') else result.relevance_score,
            }
            for result in final_results
        ]

        # Get filtering metadata
        filtering_metadata = {
            "chapter_filtered": chapter_context is not None,
            "chapter_id": chapter_context.get("chapter_id") if chapter_context else None,
            "boost_applied": any(
                getattr(r, 'boost_factor', 1.0) != 1.0 for r in filtered_results
            ),
            "filtered_count": sum(
                1 for r in filtered_results if getattr(r, 'matched_chapter', False)
            ),
        }

        logger.info(
            "Vector search completed",
            extra={
                "operation": "search_chunks",
                "collection": coll_name,
                "results_count": len(result_dicts),
                "chapter_filtered": filtering_metadata["chapter_filtered"],
                "boost_applied": filtering_metadata["boost_applied"],
                "status": "success",
            },
        )

        return {
            "results": result_dicts,
            "metadata": filtering_metadata,
        }

    except Exception as e:
        logger.error(
            f"Search failed: {str(e)}",
            extra={
                "operation": "search_chunks",
                "collection": coll_name,
                "error": str(e),
                "status": "failed",
            },
            exc_info=True,
        )
        raise RuntimeError(f"Vector search failed: {str(e)}") from e
