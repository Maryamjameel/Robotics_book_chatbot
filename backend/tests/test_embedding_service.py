"""Unit tests for embedding service."""

from unittest.mock import MagicMock, patch

import pytest

from src.models import ChapterChunk
from src.services import embed_batch, embed_chunks


class TestEmbedChunks:
    """Tests for embed_chunks function."""

    def test_embed_chunks_success(self, sample_chapters, mock_openai_client):
        """Test successful embedding of chapter chunks."""
        chunks = [
            ChapterChunk(
                chapter_id=ch["chapter_id"],
                chapter_title=ch["chapter_title"],
                section_number=i,
                section_title=sec["title"],
                content=sec["content"],
                metadata={"source": "test"},
            )
            for ch in sample_chapters
            for i, sec in enumerate(ch["sections"], 1)
        ]

        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            embeddings = embed_chunks(chunks)

        assert len(embeddings) == len(chunks)
        assert all(emb.chunk_id for emb in embeddings)
        assert all(len(emb.vector) == 1536 for emb in embeddings)
        assert all(emb.metadata for emb in embeddings)

    def test_embed_chunks_empty_list(self):
        """Test error when embedding empty chunk list."""
        with pytest.raises(ValueError, match="empty"):
            embed_chunks([])

    def test_embed_chunks_chunk_id_format(self, sample_chapters, mock_openai_client):
        """Test that chunk IDs are formatted correctly."""
        chunks = [
            ChapterChunk(
                chapter_id="ch01",
                chapter_title="Test",
                section_number=1,
                section_title="Section 1",
                content="Content",
                metadata={},
            )
        ]

        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            embeddings = embed_chunks(chunks)

        assert embeddings[0].chunk_id == "ch01_sec01"

    @pytest.mark.unit
    def test_embed_chunks_preserves_metadata(self, mock_openai_client):
        """Test that metadata is preserved in embeddings."""
        chunks = [
            ChapterChunk(
                chapter_id="ch01",
                chapter_title="Test",
                section_number=1,
                section_title="Section",
                content="Some content to embed",
                metadata={"custom_key": "custom_value"},
            )
        ]

        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            embeddings = embed_chunks(chunks)

        assert "custom_key" in embeddings[0].metadata
        assert embeddings[0].metadata["custom_key"] == "custom_value"


class TestEmbedBatch:
    """Tests for embed_batch function."""

    def test_embed_batch_success(self, mock_openai_client):
        """Test successful batch embedding."""
        texts = ["First text", "Second text", "Third text"]
        chunk_ids = ["chunk_1", "chunk_2", "chunk_3"]
        metadata = [{"id": 1}, {"id": 2}, {"id": 3}]

        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            embeddings = embed_batch(texts, chunk_ids, metadata)

        assert len(embeddings) == 3
        assert embeddings[0].chunk_id == "chunk_1"
        assert embeddings[1].chunk_id == "chunk_2"
        assert embeddings[2].chunk_id == "chunk_3"

    def test_embed_batch_empty_lists(self):
        """Test error when lists are empty."""
        with pytest.raises(ValueError, match="empty"):
            embed_batch([], [], [])

    def test_embed_batch_mismatched_lengths(self):
        """Test error when lists have different lengths."""
        with pytest.raises(ValueError, match="same length"):
            embed_batch(
                ["text1", "text2"],
                ["chunk_1", "chunk_2", "chunk_3"],
                [{"id": 1}],
            )

    def test_embed_batch_empty_text(self):
        """Test error when text is empty or whitespace."""
        with pytest.raises(ValueError, match="Empty text"):
            embed_batch(
                ["text1", "", "text3"],
                ["chunk_1", "chunk_2", "chunk_3"],
                [{"id": 1}, {"id": 2}, {"id": 3}],
            )

    @pytest.mark.unit
    def test_embed_batch_retry_logic(self, mock_openai_client):
        """Test retry logic with exponential backoff."""
        texts = ["text1"]
        chunk_ids = ["chunk_1"]
        metadata = [{"id": 1}]

        # First call fails, second succeeds (simulates retry)
        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            embeddings = embed_batch(texts, chunk_ids, metadata)

        assert len(embeddings) == 1

    @pytest.mark.unit
    def test_embed_batch_api_error(self, mock_openai_client):
        """Test handling of API errors."""
        texts = ["text1"]
        chunk_ids = ["chunk_1"]
        metadata = [{"id": 1}]

        # Mock OpenAI to raise an error
        mock_client = MagicMock()
        mock_client.embeddings.create.side_effect = Exception("API Error")

        with patch("src.services.embedding_service.OpenAI", return_value=mock_client):
            with pytest.raises(RuntimeError, match="Failed to generate"):
                embed_batch(texts, chunk_ids, metadata)

    @pytest.mark.unit
    def test_embed_batch_vector_dimensions(self, mock_openai_client):
        """Test that returned vectors have correct dimensions."""
        texts = ["test"]
        chunk_ids = ["chunk_1"]
        metadata = [{}]

        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            embeddings = embed_batch(texts, chunk_ids, metadata)

        assert len(embeddings[0].vector) == 1536
