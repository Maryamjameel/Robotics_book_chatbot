"""Integration tests for the complete embedding pipeline."""

from unittest.mock import patch

import pytest

from src.models import ChapterChunk
from src.services import (
    embed_chunks,
    initialize_collection,
    insert_embeddings,
    parse_all_chapters,
    verify_insertion,
)


class TestEndToEndPipeline:
    """Integration tests for the complete pipeline."""

    @pytest.mark.integration
    def test_parse_embed_insert_pipeline(self, temp_chapters_dir, mock_openai_client, mock_qdrant_client):
        """Test complete pipeline: parse -> embed -> insert."""
        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
                # Step 1: Parse
                chunks = parse_all_chapters(temp_chapters_dir)
                assert len(chunks) > 0

                # Step 2: Embed
                embeddings = embed_chunks(chunks)
                assert len(embeddings) == len(chunks)
                assert all(len(e.vector) == 1536 for e in embeddings)

                # Step 3: Initialize collection
                result = initialize_collection()
                assert result is True

                # Step 4: Insert
                insert_result = insert_embeddings(embeddings)
                assert insert_result.total > 0
                assert insert_result.inserted > 0

    @pytest.mark.integration
    def test_parse_and_verify_pipeline(self, temp_chapters_dir, mock_openai_client, mock_qdrant_client):
        """Test parsing with verification."""
        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
                # Parse
                chunks = parse_all_chapters(temp_chapters_dir)
                assert len(chunks) > 0

                # Embed
                embeddings = embed_chunks(chunks)

                # Initialize and insert
                initialize_collection()
                insert_result = insert_embeddings(embeddings)
                assert insert_result.inserted > 0

                # Verify
                verification = verify_insertion(sample_size=5)
                assert verification.total_points > 0
                assert verification.sampled > 0
                assert "sample_valid_percentage" in verification.checks

    @pytest.mark.integration
    def test_multiple_chapters_processing(self, temp_chapters_dir, mock_openai_client, mock_qdrant_client):
        """Test processing multiple chapters in sequence."""
        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
                # Parse all chapters
                chunks = parse_all_chapters(temp_chapters_dir)

                # Verify we got chapters from multiple files
                chapter_ids = set(c.chapter_id for c in chunks)
                assert len(chapter_ids) >= 2

                # Process all at once
                embeddings = embed_chunks(chunks)
                assert len(embeddings) == len(chunks)

    @pytest.mark.integration
    def test_metadata_preservation_through_pipeline(
        self, temp_chapters_dir, mock_openai_client, mock_qdrant_client
    ):
        """Test that metadata is preserved through the entire pipeline."""
        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
                # Parse
                chunks = parse_all_chapters(temp_chapters_dir)
                first_chunk = chunks[0]

                # Embed
                embeddings = embed_chunks(chunks)
                first_embedding = embeddings[0]

                # Verify metadata is preserved
                assert first_embedding.chunk_id.startswith(first_chunk.chapter_id)
                assert "chapter_id" in first_embedding.metadata or len(first_chunk.metadata) == 0

    @pytest.mark.integration
    def test_empty_chapters_handling(self, tmp_path, mock_openai_client, mock_qdrant_client):
        """Test handling of directories with no valid chapters."""
        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
                # Try to parse empty directory
                chunks = parse_all_chapters(str(tmp_path))

                # Should return empty list, not raise
                assert chunks == []

    @pytest.mark.integration
    def test_batch_processing_consistency(self, temp_chapters_dir, mock_openai_client, mock_qdrant_client):
        """Test that batch processing produces consistent results."""
        with patch("src.services.embedding_service.OpenAI", return_value=mock_openai_client):
            with patch("src.services.qdrant_service.QdrantClient", return_value=mock_qdrant_client):
                # Parse
                chunks = parse_all_chapters(temp_chapters_dir)

                # Generate embeddings twice
                embeddings1 = embed_chunks(chunks)
                embeddings2 = embed_chunks(chunks)

                # Should have same count
                assert len(embeddings1) == len(embeddings2)

                # Same chunk IDs in same order
                ids1 = [e.chunk_id for e in embeddings1]
                ids2 = [e.chunk_id for e in embeddings2]
                assert ids1 == ids2
