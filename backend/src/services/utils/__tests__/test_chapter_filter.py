"""Unit tests for ChapterFilterEngine.

Tests filtering and re-ranking of Qdrant results by chapter context.
"""

import pytest
from services.utils.chapter_filter import (
    ChapterContextFilter,
    SearchResult,
    FilteredResult,
    ChapterFilterEngine,
    calculate_chapter_relevance_boost,
    extract_tf_idf_terms,
)


@pytest.fixture
def sample_results():
    """Create sample search results from different chapters."""
    return [
        SearchResult(
            chapter_id="ch03",
            section_number=1,
            section_title="Forward Kinematics",
            excerpt="Forward kinematics calculates the end-effector position given joint angles.",
            relevance_score=0.95,
            id="result-1",
        ),
        SearchResult(
            chapter_id="ch03",
            section_number=2,
            section_title="Inverse Kinematics",
            excerpt="Inverse kinematics computes joint angles from desired end-effector position.",
            relevance_score=0.88,
            id="result-2",
        ),
        SearchResult(
            chapter_id="ch04",
            section_number=1,
            section_title="Dynamics",
            excerpt="Dynamics describes forces and torques that cause motion.",
            relevance_score=0.82,
            id="result-3",
        ),
        SearchResult(
            chapter_id="ch02",
            section_number=3,
            section_title="Transformations",
            excerpt="Transformation matrices represent rotations and translations.",
            relevance_score=0.85,
            id="result-4",
        ),
    ]


@pytest.fixture
def chapter_filter():
    """Create a chapter filter for chapter 3."""
    return ChapterContextFilter(chapter_id="ch03", chapter_title="Kinematics")


class TestChapterFilterEngineBasics:
    """Test basic filtering functionality."""

    def test_filter_with_no_chapter_context(self, sample_results):
        """Test filtering with no chapter context (should not boost)."""
        engine = ChapterFilterEngine()
        results = engine.filter_results(sample_results)

        # All results should be included with boost factor 1.0
        assert len(results) == 4
        assert all(r.boost_factor == 1.0 for r in results)
        assert all(r.final_relevance == r.original_relevance for r in results)

    def test_filter_with_chapter_context(self, sample_results, chapter_filter):
        """Test filtering with chapter context."""
        engine = ChapterFilterEngine(chapter_filter)
        results = engine.filter_results(sample_results)

        # Should have 4 results
        assert len(results) == 4

        # First two should be from ch03 (boosted)
        assert results[0].chapter_id == "ch03"
        assert results[0].boost_factor == 1.5
        assert results[0].matched_chapter is True

        assert results[1].chapter_id == "ch03"
        assert results[1].boost_factor == 1.5

    def test_chapter_matching_is_case_insensitive(self, sample_results):
        """Test that chapter matching ignores case."""
        filter_upper = ChapterContextFilter(chapter_id="CH03", chapter_title="Kinematics")
        engine = ChapterFilterEngine(filter_upper)
        results = engine.filter_results(sample_results)

        # Should still match ch03
        ch03_results = [r for r in results if r.matched_chapter]
        assert len(ch03_results) == 2

    def test_results_sorted_by_final_relevance(self, sample_results, chapter_filter):
        """Test that results are sorted by final relevance."""
        engine = ChapterFilterEngine(chapter_filter)
        results = engine.filter_results(sample_results)

        # Check sorted order
        for i in range(len(results) - 1):
            assert results[i].final_relevance >= results[i + 1].final_relevance

    def test_boost_metadata(self, sample_results, chapter_filter):
        """Test boost metadata generation."""
        engine = ChapterFilterEngine(chapter_filter)
        engine.filter_results(sample_results)

        metadata = engine.get_boost_metadata()
        assert metadata["chapter_filtered"] is True
        assert metadata["chapter_id"] == "ch03"
        assert metadata["boost_applied"] is True
        assert metadata["filtered_count"] == 2


class TestChapterFilterEngineOverrides:
    """Test overriding filter at call time."""

    def test_override_filter_at_call_time(self, sample_results, chapter_filter):
        """Test that filter can be overridden at call time."""
        engine = ChapterFilterEngine(chapter_filter)

        # Override with different chapter
        override_filter = ChapterContextFilter(chapter_id="ch04", chapter_title="Dynamics")
        results = engine.filter_results(sample_results, chapter_filter=override_filter)

        # Should match ch04 results, not ch03
        ch04_results = [r for r in results if r.matched_chapter]
        assert len(ch04_results) == 1
        assert ch04_results[0].chapter_id == "ch04"


class TestTFIDFBoosting:
    """Test TF-IDF boosting functionality."""

    def test_tfidf_boost_with_matching_terms(self, sample_results, chapter_filter):
        """Test TF-IDF boost with matching terms."""
        engine = ChapterFilterEngine(chapter_filter)
        filtered = engine.filter_results(sample_results)

        # Apply TF-IDF boost with terms
        terms = ["kinematics", "forward"]
        boosted = engine.apply_tf_idf_boost(filtered, selected_text_terms=terms)

        # Forward Kinematics result should have higher boost
        forward_result = next(r for r in boosted if "Forward Kinematics" in r.section_title)
        assert forward_result.boost_factor > 1.5  # Should be higher than chapter match boost

    def test_tfidf_boost_without_terms(self, sample_results, chapter_filter):
        """Test that TF-IDF boost without terms doesn't change results."""
        engine = ChapterFilterEngine(chapter_filter)
        filtered = engine.filter_results(sample_results)

        # Apply TF-IDF with empty terms
        boosted = engine.apply_tf_idf_boost(filtered, selected_text_terms=[])

        # Should be unchanged
        assert len(boosted) == len(filtered)
        for original, boosted_result in zip(filtered, boosted):
            assert boosted_result.boost_factor == original.boost_factor

    def test_tfidf_boost_respects_max_factor(self, sample_results, chapter_filter):
        """Test that TF-IDF boost respects maximum boost factor."""
        engine = ChapterFilterEngine(chapter_filter)
        filtered = engine.filter_results(sample_results)

        # Apply TF-IDF with many repeated terms
        terms = ["kinematics"] * 50  # Lots of repetition
        boosted = engine.apply_tf_idf_boost(filtered, selected_text_terms=terms)

        # Boost should never exceed TF_IDF_MAX_BOOST
        for result in boosted:
            # Boost factor components: chapter match (1.5) * TF-IDF boost (max 5.0) = max 7.5
            # But we limit individually, so check if reasonable
            assert result.boost_factor <= engine.TF_IDF_MAX_BOOST * engine.CHAPTER_MATCH_BOOST

    def test_tfidf_case_insensitive_matching(self, sample_results, chapter_filter):
        """Test that TF-IDF matching is case-insensitive."""
        engine = ChapterFilterEngine(chapter_filter)
        filtered = engine.filter_results(sample_results)

        # Apply with uppercase terms
        terms = ["KINEMATICS", "FORWARD"]
        boosted = engine.apply_tf_idf_boost(filtered, selected_text_terms=terms)

        # Should still match
        forward_result = next(r for r in boosted if "Forward Kinematics" in r.section_title)
        assert forward_result.boost_factor > 1.5


class TestRelevanceCalculation:
    """Test relevance score calculations."""

    def test_final_relevance_calculation(self, sample_results, chapter_filter):
        """Test final relevance = original_relevance * boost_factor."""
        engine = ChapterFilterEngine(chapter_filter)
        results = engine.filter_results(sample_results)

        for result in results:
            expected_final = result.original_relevance * result.boost_factor
            assert abs(result.final_relevance - expected_final) < 0.001

    def test_chapter_match_boost_amount(self, sample_results, chapter_filter):
        """Test that chapter match boost is exactly CHAPTER_MATCH_BOOST."""
        engine = ChapterFilterEngine(chapter_filter)
        results = engine.filter_results(sample_results)

        ch03_results = [r for r in results if r.matched_chapter]
        for result in ch03_results:
            assert result.boost_factor == engine.CHAPTER_MATCH_BOOST


class TestHelperFunctions:
    """Test helper functions."""

    def test_calculate_chapter_relevance_boost_match(self):
        """Test calculate_chapter_relevance_boost for matching chapters."""
        boost = calculate_chapter_relevance_boost("ch03", "ch03")
        assert boost == 1.5

    def test_calculate_chapter_relevance_boost_no_match(self):
        """Test calculate_chapter_relevance_boost for non-matching chapters."""
        boost = calculate_chapter_relevance_boost("ch03", "ch04")
        assert boost == 1.0

    def test_calculate_chapter_relevance_boost_case_insensitive(self):
        """Test calculate_chapter_relevance_boost is case-insensitive."""
        boost = calculate_chapter_relevance_boost("CH03", "ch03")
        assert boost == 1.5

    def test_calculate_chapter_relevance_boost_custom_factor(self):
        """Test calculate_chapter_relevance_boost with custom boost."""
        boost = calculate_chapter_relevance_boost("ch03", "ch03", base_boost=2.0)
        assert boost == 2.0

    def test_extract_tf_idf_terms_basic(self):
        """Test basic term extraction."""
        text = "Forward kinematics calculates position angles"
        terms = extract_tf_idf_terms(text)

        assert "forward" in terms
        assert "kinematics" in terms
        assert "calculates" in terms
        # Should not include short words or stop words
        assert "the" not in terms

    def test_extract_tf_idf_terms_empty_text(self):
        """Test term extraction with empty text."""
        terms = extract_tf_idf_terms("")
        assert terms == []

    def test_extract_tf_idf_terms_max_terms(self):
        """Test term extraction respects max_terms."""
        text = "one two three four five six seven eight nine ten eleven twelve"
        terms = extract_tf_idf_terms(text, max_terms=5)
        assert len(terms) <= 5

    def test_extract_tf_idf_terms_removes_punctuation(self):
        """Test that term extraction removes punctuation."""
        text = "forward-kinematics, inverse-kinematics. robotics!"
        terms = extract_tf_idf_terms(text)

        assert "forward-kinematics" in terms or "forward" in terms
        assert all(term.strip(".,!?;:()[]{}\"'") == term for term in terms)


class TestEdgeCases:
    """Test edge cases and error conditions."""

    def test_empty_results_list(self, chapter_filter):
        """Test filtering empty results list."""
        engine = ChapterFilterEngine(chapter_filter)
        results = engine.filter_results([])
        assert results == []

    def test_single_result(self, chapter_filter):
        """Test filtering single result."""
        single_result = [
            SearchResult(
                chapter_id="ch03",
                section_number=1,
                section_title="Test",
                excerpt="Test excerpt",
                relevance_score=0.9,
            )
        ]
        engine = ChapterFilterEngine(chapter_filter)
        results = engine.filter_results(single_result)

        assert len(results) == 1
        assert results[0].matched_chapter is True
        assert results[0].boost_factor == 1.5

    def test_all_results_same_chapter(self, chapter_filter):
        """Test filtering when all results are from same chapter."""
        same_chapter_results = [
            SearchResult(
                chapter_id="ch03",
                section_number=i,
                section_title=f"Section {i}",
                excerpt=f"Excerpt {i}",
                relevance_score=0.9 - (i * 0.05),
            )
            for i in range(1, 4)
        ]
        engine = ChapterFilterEngine(chapter_filter)
        results = engine.filter_results(same_chapter_results)

        assert len(results) == 3
        assert all(r.matched_chapter is True for r in results)
        assert all(r.boost_factor == 1.5 for r in results)

    def test_chapter_filter_with_hyphenated_ids(self):
        """Test filtering with hyphenated chapter IDs."""
        filter_ctx = ChapterContextFilter(
            chapter_id="chapter-03", chapter_title="Kinematics"
        )
        result = SearchResult(
            chapter_id="chapter-03",
            section_number=1,
            section_title="Test",
            excerpt="Test",
            relevance_score=0.9,
        )

        engine = ChapterFilterEngine(filter_ctx)
        filtered = engine.filter_results([result])

        assert filtered[0].matched_chapter is True

    def test_extract_terms_from_long_text(self):
        """Test term extraction from very long text."""
        long_text = " ".join(["robotics"] * 100 + ["kinematics"] * 50 + ["dynamics"] * 25)
        terms = extract_tf_idf_terms(long_text)

        # Should deduplicate
        assert len(terms) <= 10
        # Should contain the important terms
        assert "robotics" in terms or "kinematics" in terms
