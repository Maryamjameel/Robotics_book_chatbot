"""Search result boosting using TF-IDF weighting for selected text."""

import re
from typing import Any, Dict, List, Optional
from src.config import logger

# Common English stopwords to exclude from TF-IDF calculation
STOPWORDS = {
    'a', 'an', 'and', 'are', 'as', 'at', 'be', 'by', 'for', 'from', 'has',
    'he', 'in', 'is', 'it', 'its', 'of', 'on', 'or', 'that', 'the', 'to',
    'was', 'will', 'with', 'this', 'but', 'they', 'have', 'had', 'do', 'does',
    'did', 'should', 'would', 'could', 'can', 'if', 'else', 'what', 'which',
    'who', 'when', 'where', 'why', 'how', 'all', 'each', 'every', 'both',
    'some', 'any', 'only', 'just', 'very', 'even', 'more', 'most', 'other',
}


class SearchBoostingEngine:
    """Apply TF-IDF-based boosting to search results using selected text."""

    def __init__(self, boost_factor: float = 1.5, max_boost: float = 5.0):
        """Initialize boosting engine with configurable boost factor.

        Args:
            boost_factor: Multiplier for boost application (default 1.5 = 50% boost)
            max_boost: Maximum allowed boost factor (default 5.0)
        """
        self.boost_factor = boost_factor
        self.max_boost = max_boost
        self.extracted_terms: List[str] = []

    def _tokenize_and_clean(self, text: str) -> List[str]:
        """Tokenize text and remove stopwords.

        Args:
            text: Input text to tokenize

        Returns:
            List of lowercase non-stopword tokens
        """
        # Convert to lowercase and remove special characters
        text = text.lower()
        # Split on whitespace and punctuation
        tokens = re.findall(r'\b[a-z]+\b', text)
        # Filter out stopwords
        return [token for token in tokens if token not in STOPWORDS and len(token) > 2]

    def extract_terms(self, selected_text: str) -> List[str]:
        """Extract key terms from selected text using stopword filtering.

        Args:
            selected_text: The selected text from the page

        Returns:
            List of key terms (non-stopwords)
        """
        if not selected_text or not selected_text.strip():
            return []

        terms = self._tokenize_and_clean(selected_text)
        self.extracted_terms = terms

        logger.debug(
            f"Extracted {len(terms)} terms from selected text",
            extra={
                "operation": "extract_terms",
                "terms_count": len(terms),
                "terms": terms[:5],  # Log first 5 for debugging
                "status": "success",
            },
        )

        return terms

    def calculate_term_frequency(self, result_text: str, terms: List[str]) -> float:
        """Calculate TF (term frequency) score for result against selected terms.

        Args:
            result_text: Text from search result to score
            terms: Key terms from selected text

        Returns:
            TF score (0-1, higher means more term matches)
        """
        if not result_text or not terms:
            return 0.0

        result_tokens = self._tokenize_and_clean(result_text)
        if not result_tokens:
            return 0.0

        # Count term matches (case-insensitive)
        term_set = set(terms)
        result_set = set(result_tokens)
        matches = len(term_set & result_set)

        # TF = matches / total unique terms
        tf = matches / len(term_set) if term_set else 0.0
        return min(1.0, tf)  # Cap at 1.0

    def apply_boost_factor(self, base_score: float, tf_weight: float = 0.5) -> float:
        """Apply boost factor to a base relevance score.

        Args:
            base_score: Original cosine similarity score
            tf_weight: Weight for TF component (default 0.5)

        Returns:
            Boosted score using formula: score * (1 + tf_weight * (boost_factor - 1))
        """
        # Boost formula: score * (1 + tf_weight * (boost_factor - 1))
        # Examples:
        # - If boost_factor=1.5 and tf_weight=0.5: multiplier = 1 + 0.5*(1.5-1) = 1.25
        # - If boost_factor=5.0 and tf_weight=1.0: multiplier = 1 + 1.0*(5.0-1) = 5.0
        boost_multiplier = 1.0 + tf_weight * (self.boost_factor - 1.0)
        boosted_score = base_score * boost_multiplier

        # Clamp boosted score to max valid range
        return min(boosted_score, 1.0)

    def boost_scores(
        self,
        search_results: List[Dict[str, Any]],
        selected_text: str,
        tf_weight: float = 0.5,
    ) -> List[Dict[str, Any]]:
        """Re-rank search results by boosting scores based on selected text.

        Algorithm:
        1. Extract key terms from selected text (non-stopwords)
        2. Calculate TF score for each result
        3. Apply boost to each result's relevance score
        4. Re-sort by boosted scores

        Args:
            search_results: List of dicts with 'score' and 'payload' keys
            selected_text: Selected text from page
            tf_weight: Weight for TF in boost calculation (default 0.5)

        Returns:
            Re-ranked list sorted by boosted scores (highest first)
        """
        if not search_results or not selected_text.strip():
            return search_results

        try:
            # Extract terms from selected text
            terms = self.extract_terms(selected_text)
            if not terms:
                logger.debug(
                    "No significant terms extracted from selected text",
                    extra={
                        "operation": "boost_scores",
                        "selected_text_length": len(selected_text),
                        "status": "no_terms",
                    },
                )
                return search_results

            # Apply boosting to each result
            boosted_results = []
            for result in search_results:
                original_score = result.get("score", 0.0)
                payload = result.get("payload", {})
                result_text = payload.get("text", "")

                # Calculate TF for this result
                tf = self.calculate_term_frequency(result_text, terms)

                # Apply boost if TF > 0
                if tf > 0.0:
                    boosted_score = self.apply_boost_factor(original_score, tf_weight * tf)
                else:
                    boosted_score = original_score

                boosted_results.append({
                    **result,
                    "score": boosted_score,
                    "_original_score": original_score,  # Keep original for debugging
                    "_tf_score": tf,  # Keep TF for metadata
                })

            # Sort by boosted score (descending)
            boosted_results.sort(key=lambda x: x["score"], reverse=True)

            logger.debug(
                "Search results boosted successfully",
                extra={
                    "operation": "boost_scores",
                    "results_count": len(boosted_results),
                    "terms_count": len(terms),
                    "status": "success",
                },
            )

            return boosted_results

        except Exception as e:
            logger.warning(
                f"Boosting failed, returning original results: {str(e)}",
                extra={
                    "operation": "boost_scores",
                    "error": str(e),
                    "status": "failed",
                },
            )
            return search_results

    def get_boost_metadata(self, selected_text: str) -> Dict[str, Any]:
        """Get metadata about the boosting operation for response.

        Args:
            selected_text: Selected text that was used for boosting

        Returns:
            Dict with boost_factor, terms, and other metadata
        """
        return {
            "boost_factor": self.boost_factor,
            "terms": self.extracted_terms,
            "terms_count": len(self.extracted_terms),
            "selected_text_length": len(selected_text) if selected_text else 0,
        }
