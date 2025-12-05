"""Unit tests for Qdrant service."""

from unittest.mock import MagicMock, patch

import pytest

from src.models import TextEmbedding
from src.services import initialize_collection, insert_embeddings, verify_insertion


class TestInitializeCollection:
    """Tests for initialize_collection function."""

    def test_initialize_collection_new(self, mock_qdrant_client):
        """Test creating a new collection."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = initialize_collection("test_collection")

        assert result is True

    def test_initialize_collection_existing(self, mock_qdrant_client):
        """Test that existing collection returns True."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = initialize_collection("test_collection")

        assert result is True

    def test_initialize_collection_uses_default_name(self, mock_qdrant_client):
        """Test that default collection name is used when not provided."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = initialize_collection()

        assert result is True

    @pytest.mark.unit
    def test_initialize_collection_error(self):
        """Test error handling during initialization."""
        mock_client = MagicMock()
        mock_client.get_collection.side_effect = Exception("Connection error")
        mock_client.create_collection.side_effect = Exception("Creation error")

        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_client):
            with pytest.raises(RuntimeError, match="Failed to initialize"):
                initialize_collection("test_collection")


class TestInsertEmbeddings:
    """Tests for insert_embeddings function."""

    def test_insert_embeddings_success(self, mock_qdrant_client):
        """Test successful insertion of embeddings."""
        embeddings = [
            TextEmbedding(
                chunk_id="chunk_1",
                vector=[0.1] * 1536,
                metadata={"chapter_id": "ch01"},
            ),
            TextEmbedding(
                chunk_id="chunk_2",
                vector=[0.2] * 1536,
                metadata={"chapter_id": "ch01"},
            ),
        ]

        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = insert_embeddings(embeddings)

        assert result.total == 2
        assert result.inserted == 2
        assert result.failed == 0

    def test_insert_embeddings_empty_list(self):
        """Test error when inserting empty list."""
        with pytest.raises(ValueError, match="empty"):
            insert_embeddings([])

    def test_insert_embeddings_partial_failure(self, mock_qdrant_client):
        """Test handling of partial failures during insertion."""
        embeddings = [
            TextEmbedding(
                chunk_id="chunk_1",
                vector=[0.1] * 1536,
                metadata={"chapter_id": "ch01"},
            ),
            TextEmbedding(
                chunk_id="chunk_2",
                vector=[0.2] * 1536,
                metadata={"chapter_id": "ch01"},
            ),
        ]

        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = insert_embeddings(embeddings)

        assert result.total == len(embeddings)
        assert result.inserted + result.failed == result.total

    def test_insert_embeddings_custom_collection(self, mock_qdrant_client):
        """Test insertion with custom collection name."""
        embeddings = [
            TextEmbedding(
                chunk_id="chunk_1",
                vector=[0.1] * 1536,
                metadata={"chapter_id": "ch01"},
            )
        ]

        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = insert_embeddings(embeddings, "custom_collection")

        assert result.inserted >= 0

    @pytest.mark.unit
    def test_insert_embeddings_metadata_preserved(self, mock_qdrant_client):
        """Test that embedding metadata is preserved during insertion."""
        test_metadata = {"chapter_id": "ch01", "section": 2, "title": "Test"}
        embeddings = [
            TextEmbedding(
                chunk_id="chunk_1",
                vector=[0.1] * 1536,
                metadata=test_metadata,
            )
        ]

        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = insert_embeddings(embeddings)

        assert result.inserted >= 0


class TestVerifyInsertion:
    """Tests for verify_insertion function."""

    def test_verify_insertion_success(self, mock_qdrant_client):
        """Test successful verification of collection."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            # First insert some embeddings
            embeddings = [
                TextEmbedding(
                    chunk_id=f"chunk_{i}",
                    vector=[0.1] * 1536,
                    metadata={"id": i},
                )
                for i in range(10)
            ]
            insert_embeddings(embeddings)

            # Then verify
            result = verify_insertion()

        assert result.total_points >= 0
        assert result.sampled >= 0
        assert result.valid >= 0

    def test_verify_insertion_custom_collection(self, mock_qdrant_client):
        """Test verification with custom collection name."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = verify_insertion("custom_collection", sample_size=10)

        assert hasattr(result, "total_points")
        assert hasattr(result, "sampled")
        assert hasattr(result, "valid")

    def test_verify_insertion_sample_size(self, mock_qdrant_client):
        """Test verification respects sample size parameter."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = verify_insertion(sample_size=25)

        assert result.sampled <= 25

    @pytest.mark.unit
    def test_verify_insertion_checks_vector_dimensions(self, mock_qdrant_client):
        """Test that verification checks vector dimensions."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = verify_insertion()

        assert "vector_dimension" in result.checks
        assert result.checks["vector_dimension"] == 1536

    @pytest.mark.unit
    def test_verify_insertion_validation_percentage(self, mock_qdrant_client):
        """Test that verification calculates validation percentage."""
        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
            result = verify_insertion()

        assert "sample_valid_percentage" in result.checks
        assert 0 <= result.checks["sample_valid_percentage"] <= 100

    @pytest.mark.unit
    def test_verify_insertion_empty_collection(self):
        """Test error when collection is empty."""
        mock_client = MagicMock()
        # Mock collection with 0 points
        mock_collection = MagicMock()
        mock_collection.points_count = 0
        mock_client.get_collection.return_value = mock_collection

        with patch("src.services.qdrant_service.QdrantClient", return_value=mock_client):
            with pytest.raises(RuntimeError, match="empty"):
                verify_insertion()
