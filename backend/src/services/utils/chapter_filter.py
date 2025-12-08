"""Chapter context filtering and prioritization for Qdrant search results.

This module provides the ChapterFilterEngine class which:
- Filters Qdrant search results by chapter context
- Re-ranks results to prioritize current chapter matches
- Calculates boost factors for relevance adjustment
- Supports TF-IDF boosting for selected text terms
"""

import math
from typing import List, Optional, Dict, Set
from pydantic import BaseModel


class ChapterContextFilter(BaseModel):
    """Chapter context for filtering search results."""

    chapter_id: str
    """Chapter identifier (e.g., 'ch03', 'chapter-3')"""

    chapter_title: Optional[str] = None
    """Optional chapter title for logging"""


class SearchResult(BaseModel):
    """Search result from Qdrant with metadata."""

    chapter_id: str
    """Chapter identifier"""

    chapter_title: Optional[str] = "Unknown"
    """Chapter title"""

    section_number: int
    """Section number within chapter"""

    section_title: str
    """Section title"""

    excerpt: str
    """Relevant excerpt"""

    relevance_score: float
    """Cosine similarity score (0-1)"""

    id: Optional[str] = None
    """Optional unique ID"""


class FilteredResult(SearchResult):
    """Search result after filtering and re-ranking."""

    original_relevance: float
    """Original relevance score before boost"""

    boost_factor: float
    """Boost factor applied (1.0 = no boost)"""

    final_relevance: float
    """Final relevance after boosting"""

    matched_chapter: bool
    """Whether result matches filter chapter"""


class ChapterFilterEngine:
    """Engine for filtering and re-ranking Qdrant results by chapter context."""

    # Boost parameters
    CHAPTER_MATCH_BOOST = 1.5
    """Boost factor when result matches current chapter (1.5x relevance)"""

    TF_IDF_BASE_BOOST = 1.2
    """Base boost factor for TF-IDF (1.2x minimum)"""

    TF_IDF_MAX_BOOST = 5.0
    """Maximum boost factor from TF-IDF (5.0x maximum)"""

    MIN_RELEVANCE_THRESHOLD = 0.3
    """Minimum relevance score to include in results"""

    def __init__(self, chapter_filter: Optional[ChapterContextFilter] = None):
        """Initialize the filter engine.

        Args:
            chapter_filter: Optional chapter context for filtering results
        """
        self.chapter_filter = chapter_filter
        self.filtered_count = 0
        self.boost_applied = False

    def filter_results(
        self,
        results: List[SearchResult],
        chapter_filter: Optional[ChapterContextFilter] = None,
    ) -> List[FilteredResult]:
        """Filter and re-rank results by chapter context.

        Args:
            results: List of search results from Qdrant
            chapter_filter: Optional chapter context (uses instance filter if not provided)

        Returns:
            List of filtered and re-ranked results
        """
        # Use provided filter or instance filter
        active_filter = chapter_filter or self.chapter_filter

        if not active_filter:
            # No filtering, return results as-is
            return self._convert_results(results, boost_factor=1.0)

        # Filter results by chapter match
        filtered_results = []
        for result in results:
            matched_chapter = result.chapter_id.lower() == active_filter.chapter_id.lower()

            # Always include results, but boost chapter matches
            final_relevance = result.relevance_score * self.CHAPTER_MATCH_BOOST if matched_chapter else result.relevance_score

            filtered_result = FilteredResult(
                **result.dict(),
                original_relevance=result.relevance_score,
                boost_factor=self.CHAPTER_MATCH_BOOST if matched_chapter else 1.0,
                final_relevance=final_relevance,
                matched_chapter=matched_chapter,
            )
            filtered_results.append(filtered_result)

        # Track filtering stats
        self.filtered_count = sum(1 for r in filtered_results if r.matched_chapter)
        self.boost_applied = self.filtered_count > 0

        # Sort by final relevance (highest first)
        filtered_results.sort(key=lambda x: x.final_relevance, reverse=True)

        return filtered_results

    def apply_tf_idf_boost(
        self,
        results: List[FilteredResult],
        selected_text_terms: Optional[List[str]] = None,
    ) -> List[FilteredResult]:
        """Apply TF-IDF boosting based on selected text terms.

        Args:
            results: List of filtered results
            selected_text_terms: List of key terms from selected text

        Returns:
            Results with TF-IDF boost applied
        """
        if not selected_text_terms or len(selected_text_terms) == 0:
            return results

        # Calculate term frequencies in result excerpts
        boosted_results = []
        for result in results:
            # Count term matches in excerpt (case-insensitive)
            excerpt_lower = result.excerpt.lower()
            term_count = sum(
                excerpt_lower.count(term.lower()) for term in selected_text_terms if term
            )

            # Calculate TF-IDF boost (logarithmic to avoid over-boosting)
            if term_count > 0:
                # Log-scaled boost: log(term_count + 1) normalized to [1.2, 5.0]
                tf_score = math.log(term_count + 1)
                boost_factor = min(self.TF_IDF_BASE_BOOST + (tf_score * 0.3), self.TF_IDF_MAX_BOOST)
            else:
                boost_factor = 1.0

            # Apply boost to final relevance
            new_final_relevance = result.final_relevance * boost_factor

            result.boost_factor = result.boost_factor * boost_factor
            result.final_relevance = new_final_relevance

            boosted_results.append(result)

        # Re-sort after TF-IDF boost
        boosted_results.sort(key=lambda x: x.final_relevance, reverse=True)

        return boosted_results

    def get_boost_metadata(self) -> Dict[str, any]:
        """Get metadata about filtering and boosting applied.

        Returns:
            Dictionary with boost statistics
        """
        return {
            "chapter_filtered": self.chapter_filter is not None,
            "chapter_id": self.chapter_filter.chapter_id if self.chapter_filter else None,
            "boost_applied": self.boost_applied,
            "filtered_count": self.filtered_count,
        }

    def _convert_results(
        self, results: List[SearchResult], boost_factor: float = 1.0
    ) -> List[FilteredResult]:
        """Convert SearchResults to FilteredResults without filtering.

        Args:
            results: List of search results
            boost_factor: Boost factor to apply (default 1.0 = no boost)

        Returns:
            List of filtered results with boost applied
        """
        filtered_results = []
        for result in results:
            final_relevance = result.relevance_score * boost_factor
            filtered_result = FilteredResult(
                **result.dict(),
                original_relevance=result.relevance_score,
                boost_factor=boost_factor,
                final_relevance=final_relevance,
                matched_chapter=False,
            )
            filtered_results.append(filtered_result)

        return filtered_results


def calculate_chapter_relevance_boost(
    result_chapter_id: str,
    filter_chapter_id: str,
    base_boost: float = 1.5,
) -> float:
    """Calculate boost factor for chapter match.

    Helper function for standalone chapter relevance calculation.

    Args:
        result_chapter_id: Chapter ID from search result
        filter_chapter_id: Chapter ID from filter
        base_boost: Base boost factor (default 1.5)

    Returns:
        Boost factor (1.0 for no match, base_boost for match)
    """
    return base_boost if result_chapter_id.lower() == filter_chapter_id.lower() else 1.0


def extract_tf_idf_terms(text: str, max_terms: int = 10) -> List[str]:
    """Extract key terms from text for TF-IDF boosting.

    Helper function to extract important terms from selected text.

    Args:
        text: Input text to extract terms from
        max_terms: Maximum number of terms to return (default 10)

    Returns:
        List of key terms (lowercased, deduplicated)
    """
    if not text or len(text.strip()) == 0:
        return []

    # Simple term extraction: split by whitespace, filter short terms
    words = text.lower().split()
    # Remove common stop words and short terms
    stop_words = {
        "the",
        "a",
        "an",
        "and",
        "or",
        "but",
        "in",
        "on",
        "at",
        "to",
        "for",
        "of",
        "with",
        "is",
        "are",
        "be",
        "been",
    }

    terms = [
        word.strip(".,!?;:()[]{}\"'")
        for word in words
        if len(word) > 2 and word.lower() not in stop_words
    ]

    # Return unique terms, limited to max_terms
    unique_terms = list(dict.fromkeys(terms))  # Preserve order while deduplicating
    return unique_terms[:max_terms]
