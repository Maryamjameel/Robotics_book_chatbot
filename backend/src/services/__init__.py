"""Services for vector embeddings pipeline."""

from .embedding_service import embed_batch, embed_chunks
from .markdown_parser import parse_all_chapters, parse_chapter
from .orchestrator import PipelineResult, run_pipeline
from .qdrant_service import initialize_collection, insert_embeddings, verify_insertion
from .verification_service import EmbeddingQualityReport, verify_embedding_quality, verify_rag_accuracy

__all__ = [
    "parse_chapter",
    "parse_all_chapters",
    "embed_batch",
    "embed_chunks",
    "initialize_collection",
    "insert_embeddings",
    "verify_insertion",
    "run_pipeline",
    "PipelineResult",
    "verify_embedding_quality",
    "verify_rag_accuracy",
    "EmbeddingQualityReport",
]
