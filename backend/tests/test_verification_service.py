"""Tests for the verification service."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from src.services import EmbeddingQualityReport, verify_embedding_quality, verify_rag_accuracy


class TestEmbeddingQualityReport:
    """Tests for EmbeddingQualityReport class."""

    def test_report_initialization(self):
        """Test EmbeddingQualityReport initialization."""
        report = EmbeddingQualityReport()

        assert report.vector_dimension_match is True
        assert report.vector_dimension == 0
        assert report.vector_magnitude_valid is True
        assert report.tests_total == 0
        assert report.tests_passed == 0

    def test_report_to_dict(self):
        """Test converting report to dictionary."""
        report = EmbeddingQualityReport()
        report.tests_total = 10
        report.tests_passed = 8
        report.vector_dimension = 1536

        report_dict = report.to_dict()

        assert report_dict["vector_dimension"] == 1536
        assert report_dict["tests_total"] == 10
        assert report_dict["tests_passed"] == 8
        assert report_dict["pass_rate"] == 80.0


class TestVerifyEmbeddingQuality:
    """Tests for verify_embedding_quality function."""

    def test_verify_quality_success(self, mock_qdrant_client):
        """Test successful quality verification."""
        # Create mock points with valid embeddings
        mock_points = []
        for i in range(10):
            mock_point = MagicMock()
            mock_point.vector = np.random.randn(1536).tolist()
            # Normalize to magnitude ~1
            vector_array = np.array(mock_point.vector)
            magnitude = np.linalg.norm(vector_array)
            mock_point.vector = (vector_array / magnitude).tolist()

            mock_point.payload = {
                "chapter_id": f"ch{i:02d}",
                "section_title": f"Section {i}",
            }
            mock_points.append(mock_point)

        # Mock collection info
        mock_collection = MagicMock()
        mock_collection.points_count = 10
        mock_qdrant_client.get_collection.return_value = mock_collection
        mock_qdrant_client.scroll.return_value = (mock_points, None)

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            report = verify_embedding_quality(sample_size=10)

        assert report.vector_dimension == 1536
        assert report.vector_dimension_match is True
        assert report.tests_total > 0
        assert report.pass_rate >= 0

    def test_verify_quality_invalid_dimension(self, mock_qdrant_client):
        """Test detection of invalid vector dimensions."""
        # Create mock point with wrong dimension
        mock_point = MagicMock()
        mock_point.vector = [0.1] * 768  # Wrong dimension
        mock_point.payload = {"chapter_id": "ch01"}
        mock_points = [mock_point]

        mock_collection = MagicMock()
        mock_collection.points_count = 1
        mock_qdrant_client.get_collection.return_value = mock_collection
        mock_qdrant_client.scroll.return_value = (mock_points, None)

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            report = verify_embedding_quality(sample_size=1)

        assert report.vector_dimension_match is False

    def test_verify_quality_empty_collection(self, mock_qdrant_client):
        """Test error when collection is empty."""
        mock_collection = MagicMock()
        mock_collection.points_count = 0
        mock_qdrant_client.get_collection.return_value = mock_collection

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            with pytest.raises(RuntimeError, match="empty"):
                verify_embedding_quality()

    def test_verify_quality_missing_payload(self, mock_qdrant_client):
        """Test detection of missing payload fields."""
        # Create mock point without required payload
        mock_point = MagicMock()
        mock_point.vector = [0.1] * 1536
        mock_point.payload = {}  # Missing required fields
        mock_points = [mock_point]

        mock_collection = MagicMock()
        mock_collection.points_count = 1
        mock_qdrant_client.get_collection.return_value = mock_collection
        mock_qdrant_client.scroll.return_value = (mock_points, None)

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            report = verify_embedding_quality(sample_size=1)

        assert report.payload_completeness < 100

    @pytest.mark.unit
    def test_verify_quality_magnitude_check(self, mock_qdrant_client):
        """Test that vector magnitudes are checked."""
        # Create normalized vector with magnitude ~1
        vector = np.random.randn(1536)
        vector = vector / np.linalg.norm(vector)

        mock_point = MagicMock()
        mock_point.vector = vector.tolist()
        mock_point.payload = {
            "chapter_id": "ch01",
            "section_title": "Section",
        }
        mock_points = [mock_point]

        mock_collection = MagicMock()
        mock_collection.points_count = 1
        mock_qdrant_client.get_collection.return_value = mock_collection
        mock_qdrant_client.scroll.return_value = (mock_points, None)

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            report = verify_embedding_quality(sample_size=1)

        assert report.vector_magnitude_valid is True


class TestVerifyRagAccuracy:
    """Tests for verify_rag_accuracy function."""

    def test_verify_rag_success(self, mock_qdrant_client):
        """Test successful RAG accuracy verification."""
        # Create mock points for queries
        mock_query_points = []
        for i in range(3):
            mock_point = MagicMock()
            mock_point.vector = [0.1] * 1536
            mock_point.payload = {"chapter_id": f"ch{i}"}
            mock_query_points.append(mock_point)

        # Create mock search results
        def mock_search(*args, **kwargs):
            results = []
            for i in range(5):
                result = MagicMock()
                result.payload = {
                    "chapter_id": f"ch{i % 2}",
                    "chapter_title": f"Chapter {i}",
                    "section_title": f"Section {i}",
                }
                result.score = 0.9 - (i * 0.1)
                results.append(result)
            return results

        mock_collection = MagicMock()
        mock_collection.points_count = 10
        mock_qdrant_client.get_collection.return_value = mock_collection
        mock_qdrant_client.scroll.return_value = (mock_query_points, None)
        mock_qdrant_client.search.side_effect = mock_search

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            results = verify_rag_accuracy(num_queries=3, top_k=5)

        assert "total_searches" in results
        assert "successful_searches" in results
        assert "search_success_rate" in results
        assert results["search_success_rate"] > 0

    def test_verify_rag_empty_collection(self, mock_qdrant_client):
        """Test error when collection is empty."""
        mock_collection = MagicMock()
        mock_collection.points_count = 0
        mock_qdrant_client.get_collection.return_value = mock_collection

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            with pytest.raises(RuntimeError, match="empty"):
                verify_rag_accuracy()

    def test_verify_rag_citation_coverage(self, mock_qdrant_client):
        """Test measurement of citation coverage."""
        mock_query_points = [MagicMock(vector=[0.1] * 1536)]

        def mock_search(*args, **kwargs):
            results = []
            # Half with citations, half without
            for i in range(4):
                result = MagicMock()
                if i < 2:
                    result.payload = {
                        "chapter_id": "ch01",
                        "chapter_title": "Chapter 1",
                        "section_title": "Section",
                    }
                else:
                    result.payload = {"chapter_id": "ch01"}
                results.append(result)
            return results

        mock_collection = MagicMock()
        mock_collection.points_count = 10
        mock_qdrant_client.get_collection.return_value = mock_collection
        mock_qdrant_client.scroll.return_value = (mock_query_points, None)
        mock_qdrant_client.search.side_effect = mock_search

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            results = verify_rag_accuracy(num_queries=1, top_k=4)

        assert "citation_coverage" in results
        assert results["citation_coverage"] == 50.0  # 2 out of 4 have citations

    @pytest.mark.unit
    def test_verify_rag_diversity(self, mock_qdrant_client):
        """Test measurement of chapter diversity in results."""
        mock_query_points = [MagicMock(vector=[0.1] * 1536)]

        def mock_search(*args, **kwargs):
            results = []
            # Results from multiple chapters
            for i in range(5):
                result = MagicMock()
                result.payload = {
                    "chapter_id": f"ch{i % 3}",  # 3 different chapters
                    "section_title": "Section",
                }
                results.append(result)
            return results

        mock_collection = MagicMock()
        mock_collection.points_count = 10
        mock_qdrant_client.get_collection.return_value = mock_collection
        mock_qdrant_client.scroll.return_value = (mock_query_points, None)
        mock_qdrant_client.search.side_effect = mock_search

        with patch("src.services.verification_service.QdrantClient", return_value=mock_qdrant_client):
            results = verify_rag_accuracy(num_queries=1, top_k=5)

        assert results["diverse_chapters"] == 3
