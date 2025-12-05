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
    selected_text: Optional[str] = Field(
        default=None,
        min_length=0,
        max_length=500,
        description="Optional selected text from the page for context-aware search boosting",
        example="Forward kinematics calculates the end-effector position given joint angles",
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
                    "selected_text": None,
                    "filters": None,
                },
                {
                    "question": "Explain the Denavit-Hartenberg convention",
                    "selected_text": "The Denavit-Hartenberg convention is a standard method for describing robot kinematics.",
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
    selected_text_boosted: Optional[bool] = Field(
        default=False,
        description="Whether search results were boosted using selected text via TF-IDF",
        example=True,
    )
    boost_factor: Optional[float] = Field(
        default=1.0,
        ge=1.0,
        le=5.0,
        description="Boost factor applied to search results (1.0 = no boost, 5.0 = maximum)",
        example=1.5,
    )
    selected_text_terms: Optional[List[str]] = Field(
        default_factory=list,
        description="Key terms extracted from selected text and used for TF-IDF boosting",
        example=["forward", "kinematics"],
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
                        "selected_text_boosted": False,
                        "boost_factor": 1.0,
                        "selected_text_terms": [],
                    },
                },
                {
                    "answer": "Forward kinematics calculates the end-effector position using the Denavit-Hartenberg parameters. The process involves multiplying transformation matrices. Source: Chapter 3, Section 1 - Forward Kinematics",
                    "sources": [
                        {
                            "chapter_id": "ch03",
                            "chapter_title": "Kinematics",
                            "section_number": 1,
                            "section_title": "Forward Kinematics",
                            "excerpt": "Forward kinematics is...",
                            "relevance_score": 0.95,
                        }
                    ],
                    "metadata": {
                        "confidence_score": 0.96,
                        "search_latency_ms": 115.0,
                        "generation_latency_ms": 1380.0,
                        "total_latency_ms": 1495.0,
                        "selected_text_boosted": True,
                        "boost_factor": 1.5,
                        "selected_text_terms": ["forward", "kinematics", "denavit", "hartenberg"],
                    },
                },
            ]
        }
