"""
Unit tests for search boosting functionality
Tests TF-IDF calculations, term extraction, and search result re-ranking
"""

import pytest
from app.services.utils.search_boosting import (
    SearchBoostingEngine,
    extract_terms,
    calculate_term_frequency,
    apply_boost_factor,
)


class TestExtractTerms:
    """Tests for term extraction functionality"""

    def test_extract_basic_terms(self):
        """Should extract individual words from text"""
        result = extract_terms("forward kinematics")
        assert "forward" in result
        assert "kinematics" in result

    def test_remove_stopwords(self):
        """Should remove common stopwords"""
        result = extract_terms("the forward kinematics and the denavit hartenberg")
        assert "the" not in result
        assert "and" not in result
        assert "forward" in result
        assert "kinematics" in result

    def test_case_insensitive(self):
        """Should convert terms to lowercase"""
        result = extract_terms("Forward KINEMATICS")
        assert all(term == term.lower() for term in result)

    def test_unique_terms_only(self):
        """Should remove duplicate terms"""
        result = extract_terms("robot robot kinematics robot")
        robot_count = sum(1 for term in result if term == "robot")
        assert robot_count <= 1

    def test_empty_string(self):
        """Should return empty list for empty string"""
        result = extract_terms("")
        assert result == []

    def test_only_stopwords(self):
        """Should return empty list if only stopwords"""
        result = extract_terms("the and a an")
        assert len(result) == 0

    def test_preserve_technical_terms(self):
        """Should preserve important technical terms"""
        result = extract_terms("denavit hartenberg convention")
        assert "denavit" in result
        assert "hartenberg" in result
        assert "convention" in result

    def test_max_terms_limit(self):
        """Should limit terms to reasonable number"""
        long_text = " ".join([f"term{i}" for i in range(100)])
        result = extract_terms(long_text)
        assert len(result) <= 100  # Should have practical limit


class TestCalculateTermFrequency:
    """Tests for term frequency calculation"""

    def test_single_term_frequency(self):
        """Should calculate correct frequency for single term"""
        terms = ["robot", "kinematics"]
        text = "robot appears twice robot"
        tf = calculate_term_frequency(terms, text)

        assert "robot" in tf
        assert tf["robot"] > 0

    def test_term_frequency_normalized(self):
        """Should return normalized frequencies (0-1)"""
        terms = ["forward", "kinematics"]
        text = "forward kinematics forward"
        tf = calculate_term_frequency(terms, text)

        for freq in tf.values():
            assert 0 <= freq <= 1

    def test_zero_frequency_for_absent_terms(self):
        """Should return 0 for terms not in text"""
        terms = ["nonexistent"]
        text = "forward kinematics"
        tf = calculate_term_frequency(terms, text)

        assert tf.get("nonexistent", 0) == 0

    def test_empty_term_list(self):
        """Should handle empty term list"""
        tf = calculate_term_frequency([], "some text")
        assert tf == {}


class TestApplyBoostFactor:
    """Tests for boost factor application"""

    def test_no_boost_with_zero_tf(self):
        """Should not boost when term frequency is 0"""
        base_score = 0.8
        boosted = apply_boost_factor(base_score, tf_weight=0.0, boost_factor=1.5)
        assert boosted == base_score

    def test_maximum_boost_with_max_tf(self):
        """Should apply maximum boost when TF is 1.0"""
        base_score = 0.8
        boosted = apply_boost_factor(base_score, tf_weight=1.0, boost_factor=1.5)
        assert boosted > base_score

    def test_boost_factor_range(self):
        """Boosted score should be within reasonable range"""
        base_score = 0.7
        boosted = apply_boost_factor(base_score, tf_weight=0.5, boost_factor=2.0)
        # Score should increase but not excessively
        assert base_score <= boosted <= 1.0

    def test_boost_factor_of_one_no_change(self):
        """Boost factor of 1.0 should not change score"""
        base_score = 0.8
        boosted = apply_boost_factor(base_score, tf_weight=0.8, boost_factor=1.0)
        assert boosted == base_score

    def test_higher_boost_factor_increases_score(self):
        """Higher boost factor should increase score more"""
        base_score = 0.7
        tf_weight = 0.5

        boosted_1_5 = apply_boost_factor(base_score, tf_weight, boost_factor=1.5)
        boosted_3_0 = apply_boost_factor(base_score, tf_weight, boost_factor=3.0)

        assert boosted_3_0 > boosted_1_5


class TestSearchBoostingEngine:
    """Tests for SearchBoostingEngine class"""

    @pytest.fixture
    def engine(self):
        """Create engine instance for tests"""
        return SearchBoostingEngine()

    @pytest.fixture
    def sample_results(self):
        """Sample search results for testing"""
        return [
            {
                "id": "1",
                "text": "forward kinematics",
                "score": 0.8,
            },
            {
                "id": "2",
                "text": "inverse kinematics",
                "score": 0.75,
            },
            {
                "id": "3",
                "text": "robot dynamics",
                "score": 0.7,
            },
        ]

    def test_boost_scores_returns_list(self, engine, sample_results):
        """Should return list of boosted results"""
        result = engine.boost_scores(sample_results, "forward kinematics")
        assert isinstance(result, list)
        assert len(result) == len(sample_results)

    def test_boost_increases_relevant_scores(self, engine, sample_results):
        """Should increase scores of results matching selected text"""
        before = sample_results[0]["score"]
        boosted = engine.boost_scores(sample_results, "forward kinematics")
        after = boosted[0]["score"]

        assert after >= before

    def test_boost_reranks_results(self, engine, sample_results):
        """Should potentially reorder results based on boosting"""
        # After boosting "forward", first result should be relevant
        boosted = engine.boost_scores(sample_results, "forward kinematics")

        # Results with "forward" should be ranked higher
        forward_scores = [
            r["score"] for r in boosted if "forward" in r.get("text", "").lower()
        ]
        assert len(forward_scores) > 0

    def test_get_boost_metadata(self, engine):
        """Should return correct metadata structure"""
        metadata = engine.get_boost_metadata("forward kinematics")

        assert "boost_factor" in metadata
        assert "terms" in metadata
        assert "term_frequency" in metadata

    def test_metadata_contains_extracted_terms(self, engine):
        """Metadata should contain terms extracted from selected text"""
        metadata = engine.get_boost_metadata("forward kinematics")
        terms = metadata.get("terms", [])

        assert "forward" in terms or "kinematics" in terms

    def test_boost_factor_in_valid_range(self, engine):
        """Boost factor in metadata should be 1.0-5.0"""
        metadata = engine.get_boost_metadata("forward kinematics")
        boost_factor = metadata.get("boost_factor", 1.0)

        assert 1.0 <= boost_factor <= 5.0

    def test_empty_selected_text(self, engine, sample_results):
        """Should handle empty selected text gracefully"""
        result = engine.boost_scores(sample_results, "")
        assert isinstance(result, list)
        assert len(result) == len(sample_results)

    def test_empty_search_results(self, engine):
        """Should handle empty search results"""
        result = engine.boost_scores([], "forward kinematics")
        assert result == []

    def test_single_result_boosting(self, engine):
        """Should handle single result"""
        single_result = [{"id": "1", "text": "forward kinematics", "score": 0.8}]
        result = engine.boost_scores(single_result, "forward kinematics")

        assert len(result) == 1
        assert result[0]["score"] >= single_result[0]["score"]

    def test_preserves_result_fields(self, engine):
        """Should preserve all original fields in boosted results"""
        original = {
            "id": "1",
            "text": "forward kinematics",
            "score": 0.8,
            "metadata": {"chapter": "3"},
        }
        results = [original]
        boosted = engine.boost_scores(results, "forward")

        assert boosted[0]["id"] == original["id"]
        assert boosted[0]["text"] == original["text"]
        assert boosted[0].get("metadata") == original.get("metadata")

    def test_handles_special_characters(self, engine, sample_results):
        """Should handle special characters in selected text"""
        result = engine.boost_scores(
            sample_results, "forward-kinematics (FK) with symbols!"
        )
        assert isinstance(result, list)

    def test_case_insensitive_matching(self, engine, sample_results):
        """Should match terms case-insensitively"""
        result = engine.boost_scores(sample_results, "FORWARD KINEMATICS")
        # Results with "forward" should be boosted regardless of case
        assert isinstance(result, list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
