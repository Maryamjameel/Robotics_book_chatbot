"""Integration tests for chapter filtering in search_chunks.

Tests the integration between Qdrant search and ChapterFilterEngine.
"""

import pytest
from unittest.mock import Mock, patch
from services.qdrant_service import search_chunks
from services.utils.chapter_filter import SearchResult


# Mock Qdrant search result
class MockQdrantResult:
    """Mock Qdrant search result."""

    def __init__(self, id: int, score: float, payload: dict):
        self.id = id
        self.score = score
        self.payload = payload


@pytest.fixture
def mock_qdrant_results():
    """Create mock Qdrant results from different chapters."""
    return [
        MockQdrantResult(
            id=1,
            score=0.95,
            payload={
                "chapter_id": "ch03",
                "section_number": 1,
                "section_title": "Forward Kinematics",
                "excerpt": "Forward kinematics calculates the end-effector position given joint angles.",
            },
        ),
        MockQdrantResult(
            id=2,
            score=0.88,
            payload={
                "chapter_id": "ch03",
                "section_number": 2,
                "section_title": "Inverse Kinematics",
                "excerpt": "Inverse kinematics computes joint angles from desired end-effector position.",
            },
        ),
        MockQdrantResult(
            id=3,
            score=0.82,
            payload={
                "chapter_id": "ch04",
                "section_number": 1,
                "section_title": "Dynamics",
                "excerpt": "Dynamics describes forces and torques that cause motion.",
            },
        ),
        MockQdrantResult(
            id=4,
            score=0.85,
            payload={
                "chapter_id": "ch02",
                "section_number": 3,
                "section_title": "Transformations",
                "excerpt": "Transformation matrices represent rotations and translations in 3D space.",
            },
        ),
        MockQdrantResult(
            id=5,
            score=0.75,
            payload={
                "chapter_id": "ch03",
                "section_number": 3,
                "section_title": "Jacobian",
                "excerpt": "The Jacobian matrix relates joint velocities to end-effector velocities.",
            },
        ),
    ]


class TestSearchChunksBasic:
    """Test basic search_chunks functionality."""

    @patch('services.qdrant_service.QdrantClient')
    def test_search_without_chapter_context(self, mock_client_class, mock_qdrant_results):
        """Test search_chunks without chapter context."""
        # Setup mock
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        # Execute search
        embedding = [0.1] * 1536
        result = search_chunks(embedding, top_k=5)

        # Verify results
        assert "results" in result
        assert "metadata" in result
        assert len(result["results"]) == 5
        assert result["metadata"]["chapter_filtered"] is False

    @patch('services.qdrant_service.QdrantClient')
    def test_search_respects_threshold(self, mock_client_class, mock_qdrant_results):
        """Test that search respects relevance threshold."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        result = search_chunks(embedding, relevance_threshold=0.90, top_k=5)

        # Only results with score >= 0.90 should be included
        assert len(result["results"]) == 1
        assert result["results"][0]["relevance_score"] >= 0.90

    @patch('services.qdrant_service.QdrantClient')
    def test_search_returns_top_k(self, mock_client_class, mock_qdrant_results):
        """Test that search returns exactly top_k results."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        for k in [1, 2, 3, 5]:
            result = search_chunks(embedding, top_k=k)
            assert len(result["results"]) == min(k, len(mock_qdrant_results))


class TestSearchChunksWithChapterContext:
    """Test search_chunks with chapter context filtering."""

    @patch('services.qdrant_service.QdrantClient')
    def test_search_with_chapter_context(self, mock_client_class, mock_qdrant_results):
        """Test search_chunks with chapter context."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        chapter_context = {"chapter_id": "ch03", "chapter_title": "Kinematics"}
        result = search_chunks(embedding, chapter_context=chapter_context, top_k=5)

        # Verify metadata
        assert result["metadata"]["chapter_filtered"] is True
        assert result["metadata"]["chapter_id"] == "ch03"

        # ch03 results should be boosted
        ch03_results = [r for r in result["results"] if r["chapter_id"] == "ch03"]
        assert len(ch03_results) > 0

    @patch('services.qdrant_service.QdrantClient')
    def test_chapter_filtering_reranks_results(self, mock_client_class, mock_qdrant_results):
        """Test that chapter filtering re-ranks results."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        chapter_context = {"chapter_id": "ch03", "chapter_title": "Kinematics"}
        result = search_chunks(embedding, chapter_context=chapter_context, top_k=5)

        # First results should be from ch03 (boosted)
        first_chapter = result["results"][0]["chapter_id"]
        assert first_chapter == "ch03"

    @patch('services.qdrant_service.QdrantClient')
    def test_chapter_filtering_boost_metadata(self, mock_client_class, mock_qdrant_results):
        """Test that chapter filtering sets boost metadata."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        chapter_context = {"chapter_id": "ch03", "chapter_title": "Kinematics"}
        result = search_chunks(embedding, chapter_context=chapter_context, top_k=5)

        assert result["metadata"]["boost_applied"] is True
        assert result["metadata"]["filtered_count"] >= 0


class TestSearchChunksWithTFIDFBoosting:
    """Test search_chunks with TF-IDF boosting."""

    @patch('services.qdrant_service.QdrantClient')
    def test_search_with_selected_text_terms(self, mock_client_class, mock_qdrant_results):
        """Test search_chunks with selected text terms for TF-IDF boosting."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        chapter_context = {"chapter_id": "ch03", "chapter_title": "Kinematics"}
        selected_terms = ["kinematics", "forward"]

        result = search_chunks(
            embedding,
            chapter_context=chapter_context,
            selected_text_terms=selected_terms,
            top_k=5,
        )

        # Should have results with boosted relevance
        assert len(result["results"]) > 0
        # Forward Kinematics should be highly ranked
        first_section = result["results"][0]["section_title"]
        assert "kinematics" in first_section.lower()

    @patch('services.qdrant_service.QdrantClient')
    def test_tfidf_boost_without_chapter_context(self, mock_client_class, mock_qdrant_results):
        """Test TF-IDF boosting without chapter context."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        selected_terms = ["forward", "inverse"]

        result = search_chunks(
            embedding,
            selected_text_terms=selected_terms,
            top_k=5,
        )

        # TF-IDF should still work without chapter context
        assert len(result["results"]) > 0


class TestSearchChunksResultFormat:
    """Test search_chunks result format."""

    @patch('services.qdrant_service.QdrantClient')
    def test_result_contains_required_fields(self, mock_client_class, mock_qdrant_results):
        """Test that results contain required fields."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = [mock_qdrant_results[0]]

        embedding = [0.1] * 1536
        result = search_chunks(embedding, top_k=1)

        # Check result structure
        assert len(result["results"]) == 1
        search_result = result["results"][0]

        # Required fields
        assert "id" in search_result
        assert "chapter_id" in search_result
        assert "section_number" in search_result
        assert "section_title" in search_result
        assert "excerpt" in search_result
        assert "relevance_score" in search_result

    @patch('services.qdrant_service.QdrantClient')
    def test_metadata_contains_required_fields(self, mock_client_class, mock_qdrant_results):
        """Test that metadata contains required fields."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        result = search_chunks(embedding)

        # Check metadata structure
        assert "chapter_filtered" in result["metadata"]
        assert "chapter_id" in result["metadata"]
        assert "boost_applied" in result["metadata"]
        assert "filtered_count" in result["metadata"]


class TestSearchChunksEdgeCases:
    """Test edge cases in search_chunks."""

    @patch('services.qdrant_service.QdrantClient')
    def test_search_with_empty_chapter_context(self, mock_client_class, mock_qdrant_results):
        """Test search with empty chapter context dict."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        chapter_context = {}  # Empty context

        result = search_chunks(embedding, chapter_context=chapter_context, top_k=5)

        # Should handle gracefully
        assert "results" in result
        assert "metadata" in result

    @patch('services.qdrant_service.QdrantClient')
    def test_search_with_no_matching_threshold(self, mock_client_class, mock_qdrant_results):
        """Test search when no results meet threshold."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        result = search_chunks(embedding, relevance_threshold=0.99, top_k=5)

        # Should return empty results
        assert len(result["results"]) == 0

    @patch('services.qdrant_service.QdrantClient')
    def test_search_with_missing_payload_fields(self, mock_client_class):
        """Test search when Qdrant results have missing payload fields."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client

        # Create result with minimal payload
        minimal_result = MockQdrantResult(
            id=1, score=0.9, payload={"chapter_id": "ch03"}  # Missing other fields
        )
        mock_client.search.return_value = [minimal_result]

        embedding = [0.1] * 1536
        result = search_chunks(embedding, top_k=1)

        # Should handle missing fields gracefully with defaults
        assert len(result["results"]) == 1
        assert result["results"][0]["chapter_id"] == "ch03"
        assert result["results"][0]["section_number"] == 0  # Default value
        assert result["results"][0]["section_title"] == ""  # Default value


class TestSearchChunksIntegrationFlow:
    """Test complete integration flow."""

    @patch('services.qdrant_service.QdrantClient')
    def test_complete_flow_with_all_features(self, mock_client_class, mock_qdrant_results):
        """Test complete flow using all features together."""
        mock_client = Mock()
        mock_client_class.return_value = mock_client
        mock_client.search.return_value = mock_qdrant_results

        embedding = [0.1] * 1536
        chapter_context = {"chapter_id": "ch03", "chapter_title": "Kinematics"}
        selected_terms = ["kinematics", "jacobian"]

        result = search_chunks(
            embedding,
            top_k=3,
            relevance_threshold=0.70,
            chapter_context=chapter_context,
            selected_text_terms=selected_terms,
        )

        # Verify complete flow
        assert result["metadata"]["chapter_filtered"] is True
        assert len(result["results"]) <= 3
        assert all(r["relevance_score"] >= 0.70 for r in result["results"])

        # Verify ch03 results are prioritized
        ch03_count = sum(1 for r in result["results"] if r["chapter_id"] == "ch03")
        assert ch03_count > 0
