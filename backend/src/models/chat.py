"""Pydantic models for RAG chat API."""

from typing import List, Optional

from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="Natural language question about robotics textbook content",
        example="What is forward kinematics?",
    )
    filters: Optional[dict] = Field(
        default=None,
        description="Optional filters for vector search (e.g., chapter_id, section_number)",
        example={"chapter_id": "ch03"},
    )

    class Config:
        """Pydantic config."""

        json_schema_extra = {
            "examples": [
                {
                    "question": "What is the difference between forward and inverse kinematics?",
                    "filters": None,
                },
                {
                    "question": "Explain the Denavit-Hartenberg convention",
                    "filters": {"chapter_id": "ch03"},
                },
            ]
        }


class Source(BaseModel):
    """Source information from retrieved chunks."""

    chapter_id: str = Field(
        ..., description="Chapter identifier (e.g., ch03)", example="ch03"
    )
    chapter_title: str = Field(
        ..., description="Chapter title", example="Kinematics"
    )
    section_number: int = Field(
        ..., description="Section number within chapter", example=1
    )
    section_title: str = Field(
        ..., description="Section title", example="Forward Kinematics"
    )
    excerpt: str = Field(
        ...,
        description="Relevant excerpt from the section",
        example="Forward kinematics is the process of calculating the end-effector position...",
    )
    relevance_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Cosine similarity score from vector search (0-1)",
        example=0.89,
    )


class RAGMetadata(BaseModel):
    """Metadata about RAG pipeline execution."""

    confidence_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence score for the answer (based on relevance + citation validity)",
        example=0.92,
    )
    search_latency_ms: float = Field(
        ..., description="Vector search latency in milliseconds", example=125.0
    )
    generation_latency_ms: float = Field(
        ..., description="LLM generation latency in milliseconds", example=1450.0
    )
    total_latency_ms: float = Field(
        ..., description="Total request latency in milliseconds", example=1575.0
    )


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""

    answer: str = Field(
        ...,
        description="Generated answer with embedded citations in format 'Source: Chapter X, Section Y - Title'",
        example="Forward kinematics is... Source: Chapter 3, Section 1 - Forward Kinematics",
    )
    sources: List[Source] = Field(
        default_factory=list,
        description="List of sources retrieved from vector search",
    )
    metadata: RAGMetadata = Field(
        ..., description="Execution metadata including confidence and latency"
    )

    class Config:
        """Pydantic config."""

        json_schema_extra = {
            "examples": [
                {
                    "answer": "Forward kinematics is the process of calculating the end-effector position and orientation given joint angles. Source: Chapter 3, Section 1 - Forward Kinematics",
                    "sources": [
                        {
                            "chapter_id": "ch03",
                            "chapter_title": "Kinematics",
                            "section_number": 1,
                            "section_title": "Forward Kinematics",
                            "excerpt": "Forward kinematics is...",
                            "relevance_score": 0.89,
                        }
                    ],
                    "metadata": {
                        "confidence_score": 0.92,
                        "search_latency_ms": 125.0,
                        "generation_latency_ms": 1450.0,
                        "total_latency_ms": 1575.0,
                    },
                }
            ]
        }
