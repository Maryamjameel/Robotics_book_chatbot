"""Pydantic models for vector embeddings pipeline."""

from typing import Any, Dict, List, Optional
from pydantic import BaseModel, Field


class ChapterChunk(BaseModel):
    """Represents a chunk of text extracted from a markdown chapter."""

    chapter_id: str = Field(..., description="Unique identifier for the chapter")
    chapter_title: str = Field(..., description="Title of the chapter")
    section_number: int = Field(..., description="Sequential number of the section within chapter")
    section_title: str = Field(..., description="Title of the section")
    content: str = Field(..., description="Actual text content of the section")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional metadata for citations")

    class Config:
        """Pydantic config for ChapterChunk."""

        json_schema_extra = {
            "example": {
                "chapter_id": "ch01",
                "chapter_title": "Introduction to Robotics",
                "section_number": 1,
                "section_title": "Overview",
                "content": "Robotics is an interdisciplinary field...",
                "metadata": {"source_url": "https://example.com/ch01"},
            }
        }


class TextEmbedding(BaseModel):
    """Represents an embedding vector with metadata."""

    chunk_id: str = Field(..., description="Unique identifier for the source chunk")
    vector: List[float] = Field(..., description="1536-dimensional embedding vector")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Metadata from source chunk")

    class Config:
        """Pydantic config for TextEmbedding."""

        json_schema_extra = {
            "example": {
                "chunk_id": "ch01_sec01",
                "vector": [0.001, 0.002, -0.003, "... 1533 more values ..."],
                "metadata": {"chapter_id": "ch01", "section_title": "Overview"},
            }
        }


class InsertionResult(BaseModel):
    """Result of inserting embeddings into Qdrant collection."""

    total: int = Field(..., description="Total embeddings attempted to insert")
    inserted: int = Field(..., description="Number successfully inserted")
    failed: int = Field(..., description="Number that failed to insert")
    errors: List[Dict[str, Any]] = Field(
        default_factory=list, description="List of errors encountered during insertion"
    )

    class Config:
        """Pydantic config for InsertionResult."""

        json_schema_extra = {
            "example": {
                "total": 100,
                "inserted": 98,
                "failed": 2,
                "errors": [
                    {"chunk_id": "ch01_sec05", "error": "Vector dimension mismatch"}
                ],
            }
        }


class VerificationResult(BaseModel):
    """Result of verifying embeddings in Qdrant collection."""

    total_points: int = Field(..., description="Total points in collection")
    sampled: int = Field(..., description="Number of points sampled for verification")
    valid: int = Field(..., description="Number of points that passed validation")
    invalid: int = Field(..., description="Number of points that failed validation")
    checks: Dict[str, Any] = Field(
        default_factory=dict, description="Detailed check results (vector_dims, payload_schema, etc.)"
    )

    class Config:
        """Pydantic config for VerificationResult."""

        json_schema_extra = {
            "example": {
                "total_points": 500,
                "sampled": 50,
                "valid": 50,
                "invalid": 0,
                "checks": {
                    "vector_dimension": 1536,
                    "payload_keys": ["chapter_id", "section_title", "content"],
                    "sample_valid_percentage": 100.0,
                },
            }
        }
