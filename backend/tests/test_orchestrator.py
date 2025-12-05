"""Tests for the pipeline orchestrator."""

from unittest.mock import patch

import pytest

from src.services import PipelineResult, run_pipeline


class TestPipelineResult:
    """Tests for PipelineResult class."""

    def test_pipeline_result_initialization(self):
        """Test PipelineResult initialization."""
        result = PipelineResult()

        assert result.status == "pending"
        assert result.chapters_parsed == 0
        assert result.embeddings_generated == 0
        assert result.embeddings_inserted == 0
        assert result.errors == []
        assert result.warnings == []

    def test_pipeline_result_to_dict(self):
        """Test converting PipelineResult to dictionary."""
        result = PipelineResult()
        result.status = "success"
        result.chapters_parsed = 10
        result.embeddings_generated = 50

        result_dict = result.to_dict()

        assert result_dict["status"] == "success"
        assert result_dict["chapters_parsed"] == 10
        assert result_dict["embeddings_generated"] == 50


class TestRunPipeline:
    """Tests for run_pipeline function."""

    def test_run_pipeline_success(self, temp_chapters_dir, mock_openai_client, mock_qdrant_client):
        """Test successful pipeline execution."""
        with patch("src.services.orchestrator.parse_all_chapters") as mock_parse:
            with patch("src.services.orchestrator.embed_chunks") as mock_embed:
                with patch("src.services.orchestrator.initialize_collection"):
                    with patch("src.services.orchestrator.insert_embeddings") as mock_insert:
                        from src.models import ChapterChunk, TextEmbedding

                        # Mock parse
                        chunks = [
                            ChapterChunk(
                                chapter_id="ch01",
                                chapter_title="Test",
                                section_number=1,
                                section_title="Section",
                                content="Content",
                                metadata={},
                            )
                        ]
                        mock_parse.return_value = chunks

                        # Mock embed
                        embeddings = [
                            TextEmbedding(
                                chunk_id="ch01_sec01",
                                vector=[0.1] * 1536,
                                metadata={},
                            )
                        ]
                        mock_embed.return_value = embeddings

                        # Mock insert
                        from src.models import InsertionResult

                        insert_result = InsertionResult(
                            total=1,
                            inserted=1,
                            failed=0,
                            errors=[],
                        )
                        mock_insert.return_value = insert_result

                        # Run pipeline
                        result = run_pipeline(temp_chapters_dir, verify=False)

                        assert result.status == "success"
                        assert result.chapters_parsed == 1
                        assert result.embeddings_generated == 1
                        assert result.embeddings_inserted == 1

    def test_run_pipeline_no_chapters(self, tmp_path):
        """Test pipeline when no chapters are found."""
        with patch("src.services.orchestrator.parse_all_chapters") as mock_parse:
            mock_parse.return_value = []

            with pytest.raises(ValueError, match="No chapters"):
                run_pipeline(str(tmp_path))

    def test_run_pipeline_invalid_directory(self):
        """Test pipeline with invalid directory."""
        with patch("src.services.orchestrator.parse_all_chapters") as mock_parse:
            mock_parse.side_effect = FileNotFoundError("Directory not found")

            with pytest.raises(ValueError, match="Failed to parse"):
                run_pipeline("/nonexistent/path")

    def test_run_pipeline_embedding_failure(self, temp_chapters_dir):
        """Test pipeline when embedding generation fails."""
        with patch("src.services.orchestrator.parse_all_chapters") as mock_parse:
            with patch("src.services.orchestrator.embed_chunks") as mock_embed:
                from src.models import ChapterChunk

                chunks = [
                    ChapterChunk(
                        chapter_id="ch01",
                        chapter_title="Test",
                        section_number=1,
                        section_title="Section",
                        content="Content",
                        metadata={},
                    )
                ]
                mock_parse.return_value = chunks
                mock_embed.side_effect = RuntimeError("API Error")

                with pytest.raises(RuntimeError, match="Failed to generate"):
                    run_pipeline(temp_chapters_dir)

    def test_run_pipeline_insertion_failure(self, temp_chapters_dir):
        """Test pipeline when insertion fails."""
        with patch("src.services.orchestrator.parse_all_chapters") as mock_parse:
            with patch("src.services.orchestrator.embed_chunks") as mock_embed:
                with patch("src.services.orchestrator.initialize_collection"):
                    with patch("src.services.orchestrator.insert_embeddings") as mock_insert:
                        from src.models import ChapterChunk, TextEmbedding

                        chunks = [
                            ChapterChunk(
                                chapter_id="ch01",
                                chapter_title="Test",
                                section_number=1,
                                section_title="Section",
                                content="Content",
                                metadata={},
                            )
                        ]
                        mock_parse.return_value = chunks

                        embeddings = [
                            TextEmbedding(
                                chunk_id="ch01_sec01",
                                vector=[0.1] * 1536,
                                metadata={},
                            )
                        ]
                        mock_embed.return_value = embeddings
                        mock_insert.side_effect = RuntimeError("Qdrant Error")

                        with pytest.raises(RuntimeError, match="Failed to insert"):
                            run_pipeline(temp_chapters_dir)

    @pytest.mark.integration
    def test_run_pipeline_with_verification(self, temp_chapters_dir, mock_openai_client, mock_qdrant_client):
        """Test pipeline with verification enabled."""
        with patch("src.services.orchestrator.parse_all_chapters") as mock_parse:
            with patch("src.services.orchestrator.embed_chunks") as mock_embed:
                with patch("src.services.orchestrator.initialize_collection"):
                    with patch("src.services.orchestrator.insert_embeddings") as mock_insert:
                        with patch("src.services.orchestrator.verify_insertion") as mock_verify:
                            from src.models import ChapterChunk, TextEmbedding, InsertionResult, VerificationResult

                            chunks = [
                                ChapterChunk(
                                    chapter_id="ch01",
                                    chapter_title="Test",
                                    section_number=1,
                                    section_title="Section",
                                    content="Content",
                                    metadata={},
                                )
                            ]
                            mock_parse.return_value = chunks

                            embeddings = [
                                TextEmbedding(
                                    chunk_id="ch01_sec01",
                                    vector=[0.1] * 1536,
                                    metadata={},
                                )
                            ]
                            mock_embed.return_value = embeddings

                            mock_insert.return_value = InsertionResult(
                                total=1, inserted=1, failed=0, errors=[]
                            )

                            verification = VerificationResult(
                                total_points=1,
                                sampled=1,
                                valid=1,
                                invalid=0,
                                checks={"sample_valid_percentage": 100.0},
                            )
                            mock_verify.return_value = verification

                            result = run_pipeline(temp_chapters_dir, verify=True)

                            assert result.status == "success"
                            assert result.verification is not None
                            mock_verify.assert_called_once()

    @pytest.mark.integration
    def test_run_pipeline_partial_insertion_failure(self, temp_chapters_dir):
        """Test pipeline handles partial insertion failures gracefully."""
        with patch("src.services.orchestrator.parse_all_chapters") as mock_parse:
            with patch("src.services.orchestrator.embed_chunks") as mock_embed:
                with patch("src.services.orchestrator.initialize_collection"):
                    with patch("src.services.orchestrator.insert_embeddings") as mock_insert:
                        from src.models import ChapterChunk, TextEmbedding, InsertionResult

                        chunks = [
                            ChapterChunk(
                                chapter_id="ch01",
                                chapter_title="Test",
                                section_number=i,
                                section_title=f"Section {i}",
                                content=f"Content {i}",
                                metadata={},
                            )
                            for i in range(1, 4)
                        ]
                        mock_parse.return_value = chunks

                        embeddings = [
                            TextEmbedding(
                                chunk_id=f"ch01_sec0{i}",
                                vector=[0.1] * 1536,
                                metadata={},
                            )
                            for i in range(1, 4)
                        ]
                        mock_embed.return_value = embeddings

                        # 2 succeeded, 1 failed
                        mock_insert.return_value = InsertionResult(
                            total=3,
                            inserted=2,
                            failed=1,
                            errors=[{"chunk_id": "ch01_sec02", "error": "Duplicate"}],
                        )

                        result = run_pipeline(temp_chapters_dir, verify=False)

                        assert result.status == "success"  # Still successful despite 1 failure
                        assert result.embeddings_inserted == 2
                        assert result.embeddings_failed == 1
                        assert len(result.warnings) == 1
