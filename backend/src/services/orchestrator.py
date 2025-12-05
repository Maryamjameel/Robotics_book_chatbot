"""Orchestrator service for coordinating the complete embedding pipeline."""

from typing import Dict, List, Optional

from ..config import config, logger
from ..models import ChapterChunk, InsertionResult, TextEmbedding, VerificationResult
from .embedding_service import embed_chunks
from .markdown_parser import parse_all_chapters
from .qdrant_service import initialize_collection, insert_embeddings, verify_insertion


class PipelineResult:
    """Result of a complete pipeline execution."""

    def __init__(self):
        """Initialize pipeline result."""
        self.status = "pending"
        self.chapters_parsed = 0
        self.embeddings_generated = 0
        self.embeddings_inserted = 0
        self.embeddings_failed = 0
        self.errors: List[Dict] = []
        self.warnings: List[Dict] = []
        self.verification: Optional[VerificationResult] = None

    def to_dict(self) -> Dict:
        """Convert result to dictionary."""
        return {
            "status": self.status,
            "chapters_parsed": self.chapters_parsed,
            "embeddings_generated": self.embeddings_generated,
            "embeddings_inserted": self.embeddings_inserted,
            "embeddings_failed": self.embeddings_failed,
            "total_errors": len(self.errors),
            "total_warnings": len(self.warnings),
            "errors": self.errors,
            "warnings": self.warnings,
            "verification": self.verification.model_dump() if self.verification else None,
        }


def run_pipeline(
    chapters_dir: str,
    collection_name: Optional[str] = None,
    verify: bool = True,
) -> PipelineResult:
    """
    Run the complete embedding pipeline.

    Orchestrates: parse chapters -> generate embeddings -> insert into Qdrant -> verify

    Args:
        chapters_dir: Directory containing markdown chapter files
        collection_name: Qdrant collection name (uses config default if None)
        verify: Whether to verify insertion results

    Returns:
        PipelineResult with execution status and statistics

    Raises:
        ValueError: If chapters_dir is invalid
        RuntimeError: If critical pipeline steps fail
    """
    result = PipelineResult()

    logger.info(
        "Pipeline started",
        extra={
            "operation": "run_pipeline",
            "status": "starting",
            "chapters_dir": chapters_dir,
            "verify": verify,
        },
    )

    try:
        # Step 1: Parse chapters
        logger.info(
            "Step 1: Parsing chapters",
            extra={
                "operation": "run_pipeline",
                "status": "parsing",
            },
        )

        try:
            chunks = parse_all_chapters(chapters_dir)
        except FileNotFoundError as e:
            result.status = "failed"
            result.errors.append(
                {
                    "step": "parse_chapters",
                    "error": str(e),
                    "type": "FileNotFoundError",
                }
            )
            logger.error(
                "Failed to parse chapters",
                extra={
                    "operation": "run_pipeline",
                    "status": "failed",
                    "error": str(e),
                },
                exc_info=True,
            )
            raise ValueError(f"Failed to parse chapters: {e}")

        if not chunks:
            result.status = "failed"
            result.errors.append(
                {
                    "step": "parse_chapters",
                    "error": "No chapters found",
                    "type": "ValueError",
                }
            )
            logger.error(
                "No chapters found",
                extra={
                    "operation": "run_pipeline",
                    "status": "failed",
                },
            )
            raise ValueError("No chapters found in the specified directory")

        result.chapters_parsed = len(chunks)

        logger.info(
            "Chapters parsed successfully",
            extra={
                "operation": "run_pipeline",
                "status": "parsed",
                "chunks_count": len(chunks),
            },
        )

        # Step 2: Generate embeddings
        logger.info(
            "Step 2: Generating embeddings",
            extra={
                "operation": "run_pipeline",
                "status": "embedding",
            },
        )

        try:
            embeddings = embed_chunks(chunks)
        except Exception as e:
            result.status = "failed"
            result.errors.append(
                {
                    "step": "embed_chunks",
                    "error": str(e),
                    "type": type(e).__name__,
                }
            )
            logger.error(
                "Failed to generate embeddings",
                extra={
                    "operation": "run_pipeline",
                    "status": "failed",
                    "error": str(e),
                },
                exc_info=True,
            )
            raise RuntimeError(f"Failed to generate embeddings: {e}")

        if not embeddings:
            result.status = "failed"
            result.errors.append(
                {
                    "step": "embed_chunks",
                    "error": "No embeddings generated",
                    "type": "ValueError",
                }
            )
            logger.error(
                "No embeddings generated",
                extra={
                    "operation": "run_pipeline",
                    "status": "failed",
                },
            )
            raise RuntimeError("No embeddings were generated")

        result.embeddings_generated = len(embeddings)

        logger.info(
            "Embeddings generated successfully",
            extra={
                "operation": "run_pipeline",
                "status": "embedded",
                "embeddings_count": len(embeddings),
            },
        )

        # Step 3: Initialize Qdrant collection
        logger.info(
            "Step 3: Initializing Qdrant collection",
            extra={
                "operation": "run_pipeline",
                "status": "initializing",
            },
        )

        try:
            initialize_collection(collection_name)
        except Exception as e:
            result.status = "failed"
            result.errors.append(
                {
                    "step": "initialize_collection",
                    "error": str(e),
                    "type": type(e).__name__,
                }
            )
            logger.error(
                "Failed to initialize collection",
                extra={
                    "operation": "run_pipeline",
                    "status": "failed",
                    "error": str(e),
                },
                exc_info=True,
            )
            raise RuntimeError(f"Failed to initialize collection: {e}")

        logger.info(
            "Collection initialized",
            extra={
                "operation": "run_pipeline",
                "status": "initialized",
            },
        )

        # Step 4: Insert embeddings
        logger.info(
            "Step 4: Inserting embeddings",
            extra={
                "operation": "run_pipeline",
                "status": "inserting",
            },
        )

        try:
            insertion_result = insert_embeddings(embeddings, collection_name)
        except Exception as e:
            result.status = "failed"
            result.errors.append(
                {
                    "step": "insert_embeddings",
                    "error": str(e),
                    "type": type(e).__name__,
                }
            )
            logger.error(
                "Failed to insert embeddings",
                extra={
                    "operation": "run_pipeline",
                    "status": "failed",
                    "error": str(e),
                },
                exc_info=True,
            )
            raise RuntimeError(f"Failed to insert embeddings: {e}")

        result.embeddings_inserted = insertion_result.inserted
        result.embeddings_failed = insertion_result.failed

        if insertion_result.errors:
            for error in insertion_result.errors:
                result.warnings.append(
                    {
                        "step": "insert_embeddings",
                        "chunk_id": error.get("chunk_id"),
                        "error": error.get("error"),
                    }
                )

        logger.info(
            "Embeddings inserted",
            extra={
                "operation": "run_pipeline",
                "status": "inserted",
                "inserted": insertion_result.inserted,
                "failed": insertion_result.failed,
            },
        )

        # Step 5: Verify (optional)
        if verify:
            logger.info(
                "Step 5: Verifying insertion",
                extra={
                    "operation": "run_pipeline",
                    "status": "verifying",
                },
            )

            try:
                verification = verify_insertion(collection_name, sample_size=50)
                result.verification = verification

                logger.info(
                    "Verification completed",
                    extra={
                        "operation": "run_pipeline",
                        "status": "verified",
                        "total_points": verification.total_points,
                        "valid": verification.valid,
                        "invalid": verification.invalid,
                    },
                )
            except Exception as e:
                result.warnings.append(
                    {
                        "step": "verify_insertion",
                        "error": str(e),
                        "type": type(e).__name__,
                    }
                )
                logger.warning(
                    "Verification failed (non-critical)",
                    extra={
                        "operation": "run_pipeline",
                        "status": "warning",
                        "error": str(e),
                    },
                )

        # Mark as successful
        result.status = "success"

        logger.info(
            "Pipeline completed successfully",
            extra={
                "operation": "run_pipeline",
                "status": "success",
                "chapters_parsed": result.chapters_parsed,
                "embeddings_inserted": result.embeddings_inserted,
            },
        )

    except (ValueError, RuntimeError) as e:
        # Already handled and logged above
        if result.status != "failed":
            result.status = "failed"
        raise

    except Exception as e:
        # Unexpected error
        result.status = "failed"
        result.errors.append(
            {
                "step": "unknown",
                "error": str(e),
                "type": type(e).__name__,
            }
        )
        logger.error(
            "Unexpected pipeline error",
            extra={
                "operation": "run_pipeline",
                "status": "failed",
                "error": str(e),
            },
            exc_info=True,
        )
        raise RuntimeError(f"Unexpected pipeline error: {e}")

    return result
