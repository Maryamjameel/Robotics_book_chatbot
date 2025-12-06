"""Logging utilities for chapter context filtering operations.

Provides structured logging for chapter detection, filtering, and boosting.
"""

import logging
from typing import Any, Dict, Optional, List


class ChapterFilterLogger:
    """Logger for chapter filtering operations with structured logging."""

    def __init__(self, logger: logging.Logger):
        """Initialize with a logger instance.

        Args:
            logger: Python logger instance
        """
        self.logger = logger

    def log_chapter_detected(
        self,
        chapter_id: str,
        chapter_title: str,
        confidence: str,
        source: str,
        request_id: Optional[str] = None,
    ) -> None:
        """Log when chapter context is detected.

        Args:
            chapter_id: Detected chapter ID
            chapter_title: Detected chapter title
            confidence: Confidence level (high/medium/low)
            source: Source of detection (url/dom/hybrid)
            request_id: Optional request ID for tracing
        """
        self.logger.info(
            "Chapter context detected",
            extra={
                "operation": "chapter_detect",
                "chapter_id": chapter_id,
                "chapter_title": chapter_title,
                "confidence": confidence,
                "source": source,
                "request_id": request_id,
                "status": "detected",
            },
        )

    def log_chapter_filtering_applied(
        self,
        chapter_id: str,
        total_results: int,
        matched_count: int,
        boost_factor: float,
        request_id: Optional[str] = None,
    ) -> None:
        """Log when chapter filtering is applied to search results.

        Args:
            chapter_id: Chapter ID used for filtering
            total_results: Total results before filtering
            matched_count: Results matching the chapter
            boost_factor: Boost factor applied (1.0 = no boost)
            request_id: Optional request ID for tracing
        """
        self.logger.info(
            "Chapter filtering applied",
            extra={
                "operation": "chapter_filter",
                "chapter_id": chapter_id,
                "total_results": total_results,
                "matched_count": matched_count,
                "boost_factor": boost_factor,
                "match_percentage": (matched_count / total_results * 100) if total_results > 0 else 0,
                "request_id": request_id,
                "status": "filtered",
            },
        )

    def log_tfidf_boost_applied(
        self,
        terms: List[str],
        max_boost_factor: float,
        results_boosted: int,
        request_id: Optional[str] = None,
    ) -> None:
        """Log when TF-IDF boosting is applied.

        Args:
            terms: Terms extracted from selected text
            max_boost_factor: Maximum boost factor applied
            results_boosted: Number of results with boost > 1.0
            request_id: Optional request ID for tracing
        """
        self.logger.info(
            "TF-IDF boosting applied",
            extra={
                "operation": "tfidf_boost",
                "terms_count": len(terms),
                "terms": terms[:10],  # Log first 10 terms
                "max_boost_factor": max_boost_factor,
                "results_boosted": results_boosted,
                "request_id": request_id,
                "status": "boosted",
            },
        )

    def log_chapter_filtering_skipped(
        self,
        reason: str,
        request_id: Optional[str] = None,
    ) -> None:
        """Log when chapter filtering is skipped.

        Args:
            reason: Reason for skipping (e.g., no_context, invalid_context, error)
            request_id: Optional request ID for tracing
        """
        self.logger.debug(
            "Chapter filtering skipped",
            extra={
                "operation": "chapter_filter",
                "reason": reason,
                "request_id": request_id,
                "status": "skipped",
            },
        )

    def log_chapter_context_extraction_failed(
        self,
        error: str,
        source: str,
        request_id: Optional[str] = None,
    ) -> None:
        """Log when chapter context extraction fails.

        Args:
            error: Error message
            source: Source of extraction failure (url/dom/both)
            request_id: Optional request ID for tracing
        """
        self.logger.warning(
            f"Chapter context extraction failed: {error}",
            extra={
                "operation": "chapter_detect",
                "source": source,
                "error": error,
                "request_id": request_id,
                "status": "failed",
            },
        )

    def log_invalid_chapter_context(
        self,
        chapter_context: Dict[str, Any],
        validation_error: str,
        request_id: Optional[str] = None,
    ) -> None:
        """Log when chapter context fails validation.

        Args:
            chapter_context: Invalid chapter context
            validation_error: Validation error message
            request_id: Optional request ID for tracing
        """
        self.logger.warning(
            f"Invalid chapter context: {validation_error}",
            extra={
                "operation": "chapter_validate",
                "chapter_id": chapter_context.get("chapter_id"),
                "validation_error": validation_error,
                "request_id": request_id,
                "status": "invalid",
            },
        )

    def log_search_results_reranked(
        self,
        before_ranks: List[int],
        after_ranks: List[int],
        request_id: Optional[str] = None,
    ) -> None:
        """Log when search results are reranked due to filtering.

        Args:
            before_ranks: Result positions before reranking
            after_ranks: Result positions after reranking
            request_id: Optional request ID for tracing
        """
        self.logger.debug(
            "Search results reranked",
            extra={
                "operation": "chapter_rerank",
                "before_ranks": before_ranks[:5],  # First 5
                "after_ranks": after_ranks[:5],
                "request_id": request_id,
                "status": "reranked",
            },
        )

    def log_chapter_filter_performance(
        self,
        filtering_latency_ms: float,
        results_processed: int,
        request_id: Optional[str] = None,
    ) -> None:
        """Log performance metrics for chapter filtering.

        Args:
            filtering_latency_ms: Time spent on filtering in milliseconds
            results_processed: Number of results processed
            request_id: Optional request ID for tracing
        """
        throughput = results_processed / (filtering_latency_ms / 1000) if filtering_latency_ms > 0 else 0

        self.logger.debug(
            "Chapter filtering performance",
            extra={
                "operation": "chapter_filter",
                "latency_ms": filtering_latency_ms,
                "results_processed": results_processed,
                "throughput_results_per_sec": throughput,
                "request_id": request_id,
                "status": "perf_metric",
            },
        )


def create_chapter_filter_logger(logger: logging.Logger) -> ChapterFilterLogger:
    """Factory function to create a chapter filter logger.

    Args:
        logger: Python logger instance

    Returns:
        ChapterFilterLogger instance
    """
    return ChapterFilterLogger(logger)
