"""
TF-IDF and search boosting utilities for selected text search result ranking.

Implements term frequency calculation and cosine similarity score boosting
to improve search relevance when selected text context is provided.
"""

import re
from typing import List, Dict, Optional
from collections import Counter

# Common stopwords to exclude from boosting calculation
STOPWORDS = {
    'a', 'an', 'and', 'are', 'as', 'at', 'be', 'by', 'for', 'from',
    'has', 'he', 'in', 'is', 'it', 'its', 'of', 'on', 'or', 'that',
    'the', 'to', 'was', 'will', 'with', 'i', 'me', 'my', 'we', 'you',
    'your', 'they', 'them', 'this', 'these', 'what', 'which', 'who',
}


def extract_terms(text: Optional[str]) -> List[str]:
    """
    Extract individual terms from selected text for boosting.

    Tokenizes text, removes stopwords, and returns unique terms in lowercase.

    Args:
        text: Selected text to extract terms from (0-500 chars)

    Returns:
        List of unique terms suitable for search boosting

    Example:
        >>> extract_terms("forward kinematics")
        ['forward', 'kinematics']

        >>> extract_terms("the quick brown fox")
        ['quick', 'brown', 'fox']  # 'the' removed as stopword
    """
    if not text or not isinstance(text, str):
        return []

    # Convert to lowercase
    text_lower = text.lower()

    # Remove special characters and split by whitespace
    # Keep alphanumeric and hyphens
    words = re.findall(r'\b[\w-]+\b', text_lower)

    # Filter out stopwords and single characters
    terms = [word for word in words if word not in STOPWORDS and len(word) > 1]

    # Remove duplicates while preserving order
    seen = set()
    unique_terms = []
    for term in terms:
        if term not in seen:
            seen.add(term)
            unique_terms.append(term)

    return unique_terms


def calculate_term_frequency(terms: List[str], text: str) -> Dict[str, float]:
    """
    Calculate term frequency (TF) scores for given terms in text.

    TF is calculated as: (count of term) / (total number of words)

    Args:
        terms: List of terms to calculate frequency for
        text: Text to calculate frequency in

    Returns:
        Dictionary mapping terms to their frequency scores (0-1)

    Example:
        >>> calculate_term_frequency(
        ...     ['forward', 'kinematics'],
        ...     'forward kinematics forward kinematics definition'
        ... )
        {'forward': 0.4, 'kinematics': 0.4}
    """
    if not terms or not text:
        return {}

    # Tokenize text
    words = re.findall(r'\b[\w-]+\b', text.lower())
    total_words = len(words)

    if total_words == 0:
        return {}

    # Count term occurrences
    term_counts = Counter(words)

    # Calculate frequency for each term
    tf_scores = {}
    for term in terms:
        count = term_counts.get(term, 0)
        tf_scores[term] = count / total_words if total_words > 0 else 0

    return tf_scores


def apply_boost_factor(
    score: float,
    tf_weight: float,
    boost_factor: float = 1.5
) -> float:
    """
    Apply boost factor to cosine similarity score based on term frequency.

    Boosted score = original_score * (1 + tf_weight * (boost_factor - 1))

    This formula ensures:
    - If tf_weight = 0 (no matching terms), boosted = original
    - If tf_weight = 1 (high term frequency), boosted = original * boost_factor
    - Intermediate values scale linearly

    Args:
        score: Original cosine similarity score (0-1)
        tf_weight: Term frequency weight (0-1)
        boost_factor: Boost multiplier (default 1.5, range 1.0-5.0)

    Returns:
        Boosted score

    Example:
        >>> apply_boost_factor(0.8, 0.5, boost_factor=1.5)
        0.9  # 0.8 * (1 + 0.5 * (1.5 - 1)) = 0.8 * (1 + 0.25) = 1.0
        # Note: capped at actual result

        >>> apply_boost_factor(0.9, 1.0, boost_factor=2.0)
        1.8  # 0.9 * (1 + 1.0 * (2.0 - 1)) = 0.9 * 2 = 1.8
    """
    # Ensure valid inputs
    score = max(0, min(1, score))  # Clamp to [0, 1]
    tf_weight = max(0, min(1, tf_weight))  # Clamp to [0, 1]
    boost_factor = max(1.0, min(5.0, boost_factor))  # Clamp to [1.0, 5.0]

    # Apply boost: score * (1 + tf_weight * (boost_factor - 1))
    boosted = score * (1 + tf_weight * (boost_factor - 1))

    return boosted


def calculate_max_tf_weight(tf_scores: Dict[str, float]) -> float:
    """
    Calculate maximum term frequency weight from TF scores.

    Uses the highest TF score as the weight (represents strongest match).

    Args:
        tf_scores: Dictionary of term -> TF score mappings

    Returns:
        Maximum TF score (0-1), or 0 if no scores provided

    Example:
        >>> calculate_max_tf_weight({'forward': 0.4, 'kinematics': 0.3})
        0.4
    """
    if not tf_scores:
        return 0.0

    return max(tf_scores.values())


def calculate_mean_tf_weight(tf_scores: Dict[str, float]) -> float:
    """
    Calculate mean term frequency weight from TF scores.

    Averages all TF scores (for cases where multiple terms should be weighted).

    Args:
        tf_scores: Dictionary of term -> TF score mappings

    Returns:
        Mean TF score (0-1), or 0 if no scores provided

    Example:
        >>> calculate_mean_tf_weight({'forward': 0.4, 'kinematics': 0.3})
        0.35
    """
    if not tf_scores:
        return 0.0

    scores = list(tf_scores.values())
    return sum(scores) / len(scores) if scores else 0.0


class SearchBoostingEngine:
    """
    Engine for boosting search results based on selected text context.

    Manages term extraction, TF calculation, and result re-ranking.
    """

    def __init__(self, boost_factor: float = 1.5):
        """
        Initialize boosting engine.

        Args:
            boost_factor: Default boost multiplier (1.5 = 50% increase)
        """
        self.boost_factor = max(1.0, min(5.0, boost_factor))

    def boost_scores(
        self,
        search_results: List[Dict],
        selected_text: str,
        boost_factor: Optional[float] = None
    ) -> List[Dict]:
        """
        Boost search result scores based on selected text terms.

        Args:
            search_results: List of search results with 'text' and 'score' keys
            selected_text: Selected text from user (0-500 chars)
            boost_factor: Optional override for boost factor

        Returns:
            List of results with added 'boosted_score' and 'tf_score' fields
        """
        boost = boost_factor or self.boost_factor

        if not selected_text or not search_results:
            return search_results

        # Extract terms from selected text
        terms = extract_terms(selected_text)
        if not terms:
            return search_results

        boosted_results = []

        for result in search_results:
            result_copy = result.copy()
            original_score = result_copy.get('score', 0.0)

            # Calculate TF scores for this result
            result_text = result_copy.get('text', '')
            tf_scores = calculate_term_frequency(terms, result_text)

            # Get TF weight (use max as primary metric)
            tf_weight = calculate_max_tf_weight(tf_scores)

            # Apply boost
            boosted_score = apply_boost_factor(original_score, tf_weight, boost)

            # Add boosting information to result
            result_copy['boosted_score'] = boosted_score
            result_copy['tf_score'] = tf_weight
            result_copy['tf_scores_detail'] = tf_scores

            boosted_results.append(result_copy)

        # Re-sort by boosted score (descending)
        boosted_results.sort(key=lambda x: x['boosted_score'], reverse=True)

        return boosted_results

    def get_boost_metadata(self, selected_text: str) -> Dict:
        """
        Get metadata about boosting for a given selected text.

        Args:
            selected_text: Selected text

        Returns:
            Dictionary with boosting metadata
        """
        terms = extract_terms(selected_text)

        return {
            'selectedTextBoosted': len(terms) > 0,
            'selectedTextTerms': terms,
            'boostFactor': self.boost_factor,
            'termCount': len(terms),
        }
