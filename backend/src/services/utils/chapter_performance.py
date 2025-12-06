"""Performance metrics collection for chapter context operations.

Tracks latency, throughput, and resource usage for chapter detection and filtering.
"""

import time
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional


@dataclass
class PerformanceMetrics:
    """Performance metrics for chapter operations."""

    operation_name: str
    """Name of the operation (e.g., 'chapter_detect', 'chapter_filter')"""

    start_time_ms: float
    """Start time in milliseconds"""

    end_time_ms: Optional[float] = None
    """End time in milliseconds (set when operation completes)"""

    latency_ms: Optional[float] = None
    """Total latency in milliseconds"""

    items_processed: int = 0
    """Number of items processed"""

    items_per_second: Optional[float] = None
    """Throughput: items processed per second"""

    memory_usage_mb: Optional[float] = None
    """Memory usage in megabytes"""

    error: Optional[str] = None
    """Error message if operation failed"""

    tags: Dict[str, str] = field(default_factory=dict)
    """Custom tags for categorization"""

    def complete(self) -> None:
        """Mark operation as complete and calculate metrics."""
        self.end_time_ms = time.time() * 1000
        self.latency_ms = self.end_time_ms - self.start_time_ms

        if self.items_processed > 0 and self.latency_ms > 0:
            self.items_per_second = (self.items_processed / self.latency_ms) * 1000

    def to_dict(self) -> Dict:
        """Convert to dictionary format."""
        return asdict(self)


class PerformanceMonitor:
    """Monitor and collect performance metrics for chapter operations."""

    def __init__(self):
        """Initialize performance monitor."""
        self.metrics: List[PerformanceMetrics] = []
        self.active_timers: Dict[str, PerformanceMetrics] = {}

    def start_timer(
        self,
        operation_name: str,
        tags: Optional[Dict[str, str]] = None,
    ) -> str:
        """Start timing an operation.

        Args:
            operation_name: Name of the operation
            tags: Optional tags for categorization

        Returns:
            Timer ID for use in stop_timer
        """
        timer_id = f"{operation_name}_{id(object())}"
        metrics = PerformanceMetrics(
            operation_name=operation_name,
            start_time_ms=time.time() * 1000,
            tags=tags or {},
        )
        self.active_timers[timer_id] = metrics
        return timer_id

    def stop_timer(
        self,
        timer_id: str,
        items_processed: int = 0,
        error: Optional[str] = None,
    ) -> PerformanceMetrics:
        """Stop timing an operation and record metrics.

        Args:
            timer_id: Timer ID from start_timer
            items_processed: Number of items processed
            error: Optional error message if operation failed

        Returns:
            Completed PerformanceMetrics

        Raises:
            ValueError: If timer_id not found
        """
        if timer_id not in self.active_timers:
            raise ValueError(f"Timer {timer_id} not found")

        metrics = self.active_timers.pop(timer_id)
        metrics.items_processed = items_processed
        metrics.error = error
        metrics.complete()

        self.metrics.append(metrics)
        return metrics

    def get_summary(self, operation_name: Optional[str] = None) -> Dict:
        """Get performance summary for operations.

        Args:
            operation_name: Optional filter by operation name

        Returns:
            Dictionary with performance statistics
        """
        filtered_metrics = self.metrics
        if operation_name:
            filtered_metrics = [m for m in self.metrics if m.operation_name == operation_name]

        if not filtered_metrics:
            return {
                "operation_name": operation_name,
                "count": 0,
                "metrics": [],
            }

        latencies = [m.latency_ms for m in filtered_metrics if m.latency_ms is not None]
        latencies_sorted = sorted(latencies)

        return {
            "operation_name": operation_name,
            "count": len(filtered_metrics),
            "total_items_processed": sum(m.items_processed for m in filtered_metrics),
            "latency_stats": {
                "min_ms": min(latencies) if latencies else None,
                "max_ms": max(latencies) if latencies else None,
                "avg_ms": sum(latencies) / len(latencies) if latencies else None,
                "p50_ms": latencies_sorted[len(latencies_sorted) // 2] if latencies else None,
                "p95_ms": latencies_sorted[int(len(latencies_sorted) * 0.95)] if len(latencies_sorted) >= 20 else None,
                "p99_ms": latencies_sorted[int(len(latencies_sorted) * 0.99)] if len(latencies_sorted) >= 100 else None,
            },
            "throughput_stats": {
                "avg_items_per_sec": (
                    sum(m.items_per_second for m in filtered_metrics if m.items_per_second is not None)
                    / len([m for m in filtered_metrics if m.items_per_second is not None])
                )
                if any(m.items_per_second is not None for m in filtered_metrics)
                else None,
            },
            "error_rate": (
                sum(1 for m in filtered_metrics if m.error) / len(filtered_metrics)
            )
            if filtered_metrics
            else 0,
        }

    def clear(self) -> None:
        """Clear all recorded metrics."""
        self.metrics.clear()
        self.active_timers.clear()

    def export_metrics(self) -> List[Dict]:
        """Export all metrics as list of dictionaries.

        Returns:
            List of metric dictionaries
        """
        return [m.to_dict() for m in self.metrics]


class ChapterFilterPerformanceTracker:
    """Track performance metrics specific to chapter filtering."""

    def __init__(self):
        """Initialize tracker."""
        self.monitor = PerformanceMonitor()

    def track_detection(
        self,
        chapter_id: str,
        detection_method: str,
        latency_ms: float,
    ) -> None:
        """Track chapter detection performance.

        Args:
            chapter_id: Detected chapter ID
            detection_method: Method used (url/dom/both)
            latency_ms: Detection latency
        """
        self.monitor.metrics.append(
            PerformanceMetrics(
                operation_name="chapter_detect",
                start_time_ms=0,
                end_time_ms=latency_ms,
                latency_ms=latency_ms,
                items_processed=1,
                tags={
                    "chapter_id": chapter_id,
                    "detection_method": detection_method,
                },
            )
        )

    def track_filtering(
        self,
        total_results: int,
        filtered_results: int,
        latency_ms: float,
        boost_applied: bool,
    ) -> None:
        """Track chapter filtering performance.

        Args:
            total_results: Total results before filtering
            filtered_results: Results after filtering
            latency_ms: Filtering latency
            boost_applied: Whether boosting was applied
        """
        self.monitor.metrics.append(
            PerformanceMetrics(
                operation_name="chapter_filter",
                start_time_ms=0,
                end_time_ms=latency_ms,
                latency_ms=latency_ms,
                items_processed=total_results,
                tags={
                    "filtered_results": str(filtered_results),
                    "boost_applied": str(boost_applied),
                },
            )
        )

    def track_tfidf_boost(
        self,
        terms_count: int,
        results_boosted: int,
        latency_ms: float,
    ) -> None:
        """Track TF-IDF boosting performance.

        Args:
            terms_count: Number of terms used for boosting
            results_boosted: Number of results with boost > 1.0
            latency_ms: Boosting latency
        """
        self.monitor.metrics.append(
            PerformanceMetrics(
                operation_name="tfidf_boost",
                start_time_ms=0,
                end_time_ms=latency_ms,
                latency_ms=latency_ms,
                items_processed=results_boosted,
                tags={
                    "terms_count": str(terms_count),
                },
            )
        )

    def get_performance_report(self) -> Dict:
        """Generate comprehensive performance report.

        Returns:
            Dictionary with all performance metrics and statistics
        """
        return {
            "total_operations": len(self.monitor.metrics),
            "detection_summary": self.monitor.get_summary("chapter_detect"),
            "filtering_summary": self.monitor.get_summary("chapter_filter"),
            "tfidf_summary": self.monitor.get_summary("tfidf_boost"),
            "all_operations": self.monitor.export_metrics(),
        }


# Global instance for singleton pattern
_global_tracker: Optional[ChapterFilterPerformanceTracker] = None


def get_performance_tracker() -> ChapterFilterPerformanceTracker:
    """Get global performance tracker instance.

    Returns:
        ChapterFilterPerformanceTracker instance
    """
    global _global_tracker
    if _global_tracker is None:
        _global_tracker = ChapterFilterPerformanceTracker()
    return _global_tracker


def reset_performance_tracker() -> None:
    """Reset global performance tracker."""
    global _global_tracker
    if _global_tracker is not None:
        _global_tracker.monitor.clear()
        _global_tracker = None
