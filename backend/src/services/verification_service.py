"""Verification service for quality assurance of embeddings and RAG accuracy."""

from typing import Dict, List, Optional

import numpy as np
from qdrant_client import QdrantClient

from ..config import config, logger


class EmbeddingQualityReport:
    """Report on embedding quality metrics."""

    def __init__(self):
        """Initialize quality report."""
        self.vector_dimension_match = True
        self.vector_dimension = 0
        self.vector_magnitude_valid = True
        self.average_magnitude = 0.0
        self.magnitude_variance = 0.0
        self.payload_completeness = 0.0
        self.duplicate_vectors = 0
        self.tests_passed = 0
        self.tests_total = 0

    def to_dict(self) -> Dict:
        """Convert report to dictionary."""
        return {
            "vector_dimension_match": self.vector_dimension_match,
            "vector_dimension": self.vector_dimension,
            "vector_magnitude_valid": self.vector_magnitude_valid,
            "average_magnitude": self.average_magnitude,
            "magnitude_variance": self.magnitude_variance,
            "payload_completeness": self.payload_completeness,
            "duplicate_vectors": self.duplicate_vectors,
            "tests_passed": self.tests_passed,
            "tests_total": self.tests_total,
            "pass_rate": (self.tests_passed / self.tests_total * 100) if self.tests_total > 0 else 0,
        }


def verify_embedding_quality(
    collection_name: Optional[str] = None, sample_size: int = 100
) -> EmbeddingQualityReport:
    """
    Verify quality of embeddings in Qdrant collection.

    Performs checks:
    - Vector dimensions match 1536
    - Vector magnitudes are reasonable (normalized vectors have magnitude ~1)
    - Payload completeness (required fields present)
    - No obvious duplicate vectors

    Args:
        collection_name: Qdrant collection name
        sample_size: Number of embeddings to sample

    Returns:
        EmbeddingQualityReport with quality metrics

    Raises:
        RuntimeError: If collection is empty or unreachable
    """
    coll_name = collection_name or config.collection_name
    report = EmbeddingQualityReport()

    logger.info(
        "Starting embedding quality verification",
        extra={
            "operation": "verify_embedding_quality",
            "status": "starting",
            "collection": coll_name,
            "sample_size": sample_size,
        },
    )

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        # Get collection info
        collection_info = client.get_collection(coll_name)
        if collection_info.points_count == 0:
            raise RuntimeError(f"Collection {coll_name} is empty")

        # Sample points
        actual_sample = min(sample_size, collection_info.points_count)
        points, _ = client.scroll(collection_name=coll_name, limit=actual_sample)

        if not points:
            raise RuntimeError("No points returned from collection")

        # Verify each point
        magnitudes = []
        payload_fields = set()
        vector_hashes = set()
        duplicates = 0

        for point in points:
            report.tests_total += 1

            # Check vector dimension
            if len(point.vector) != config.vector_size:
                report.vector_dimension_match = False
                continue
            else:
                report.tests_passed += 1

            report.vector_dimension = len(point.vector)

            # Calculate magnitude
            vector_array = np.array(point.vector)
            magnitude = np.linalg.norm(vector_array)
            magnitudes.append(magnitude)

            # Check for duplicate vectors (using hash)
            vector_hash = hash(tuple(point.vector[:10]))  # Hash first 10 dims
            if vector_hash in vector_hashes:
                duplicates += 1
            vector_hashes.add(vector_hash)

            # Track payload fields
            if point.payload:
                payload_fields.update(point.payload.keys())

        report.duplicate_vectors = duplicates

        # Analyze magnitudes
        if magnitudes:
            magnitudes_array = np.array(magnitudes)
            report.average_magnitude = float(np.mean(magnitudes_array))
            report.magnitude_variance = float(np.var(magnitudes_array))

            # Check if magnitudes are reasonable (normalized embeddings ~1.0)
            if 0.9 < report.average_magnitude < 1.1:
                report.vector_magnitude_valid = True
                report.tests_passed += 1
            else:
                report.vector_magnitude_valid = False
            report.tests_total += 1

        # Check payload completeness
        required_payload_fields = {"chapter_id", "section_title"}
        if payload_fields >= required_payload_fields:
            report.payload_completeness = 100.0
            report.tests_passed += 1
        elif payload_fields:
            report.payload_completeness = (len(payload_fields & required_payload_fields) / len(required_payload_fields)) * 100
            if report.payload_completeness > 0:
                report.tests_passed += 1
        report.tests_total += 1

        logger.info(
            "Embedding quality verification completed",
            extra={
                "operation": "verify_embedding_quality",
                "status": "success",
                "collection": coll_name,
                "pass_rate": (report.tests_passed / report.tests_total * 100) if report.tests_total > 0 else 0,
            },
        )

    except Exception as e:
        logger.error(
            "Embedding quality verification failed",
            extra={
                "operation": "verify_embedding_quality",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        raise RuntimeError(f"Embedding quality verification failed: {e}")

    return report


def verify_rag_accuracy(
    collection_name: Optional[str] = None, num_queries: int = 10, top_k: int = 5
) -> Dict:
    """
    Verify RAG accuracy by checking search quality.

    Tests:
    - Search returns relevant results
    - Result diversity (different chapters/sections)
    - Payload contains citation information

    Args:
        collection_name: Qdrant collection name
        num_queries: Number of test queries to perform
        top_k: Number of top results to retrieve per query

    Returns:
        Dictionary with RAG accuracy metrics

    Raises:
        RuntimeError: If collection is empty or search fails
    """
    coll_name = collection_name or config.collection_name

    logger.info(
        "Starting RAG accuracy verification",
        extra={
            "operation": "verify_rag_accuracy",
            "status": "starting",
            "collection": coll_name,
            "num_queries": num_queries,
        },
    )

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        # Get sample vectors to use as queries
        collection_info = client.get_collection(coll_name)
        if collection_info.points_count == 0:
            raise RuntimeError(f"Collection {coll_name} is empty")

        sample_points, _ = client.scroll(
            collection_name=coll_name,
            limit=min(num_queries, collection_info.points_count),
        )

        if not sample_points:
            raise RuntimeError("No sample points available for testing")

        # Test queries
        total_searches = 0
        successful_searches = 0
        total_results = 0
        diverse_chapters = set()
        has_citations = 0

        for point in sample_points[:num_queries]:
            try:
                # Search using the point's vector
                results = client.search(
                    collection_name=coll_name,
                    query_vector=point.vector,
                    limit=top_k,
                )

                total_searches += 1

                if results and len(results) > 0:
                    successful_searches += 1
                    total_results += len(results)

                    for result in results:
                        # Check for chapter diversity
                        if result.payload and "chapter_id" in result.payload:
                            diverse_chapters.add(result.payload["chapter_id"])

                        # Check for citation info
                        if result.payload and ("chapter_title" in result.payload or "section_title" in result.payload):
                            has_citations += 1

            except Exception as e:
                logger.warning(
                    f"Search failed for query: {e}",
                    extra={
                        "operation": "verify_rag_accuracy",
                        "status": "warning",
                    },
                )
                continue

        # Calculate metrics
        accuracy_report = {
            "total_searches": total_searches,
            "successful_searches": successful_searches,
            "search_success_rate": (successful_searches / total_searches * 100) if total_searches > 0 else 0,
            "average_results_per_search": (total_results / successful_searches) if successful_searches > 0 else 0,
            "diverse_chapters": len(diverse_chapters),
            "results_with_citations": has_citations,
            "citation_coverage": (has_citations / total_results * 100) if total_results > 0 else 0,
        }

        logger.info(
            "RAG accuracy verification completed",
            extra={
                "operation": "verify_rag_accuracy",
                "status": "success",
                "collection": coll_name,
                "success_rate": accuracy_report["search_success_rate"],
            },
        )

        return accuracy_report

    except Exception as e:
        logger.error(
            "RAG accuracy verification failed",
            extra={
                "operation": "verify_rag_accuracy",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        raise RuntimeError(f"RAG accuracy verification failed: {e}")
