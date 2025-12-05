"""
Chat request/response models with selected text support.
Extends existing ChatMessage models with optional selected_text parameter for search boosting.
"""

from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field


class PageContext(BaseModel):
    """Optional metadata about the page where question was asked"""

    url: Optional[str] = Field(None, description="Full page URL including protocol")
    pathname: Optional[str] = Field(None, description="URL pathname without domain")
    chapter: Optional[str] = Field(None, description="Chapter title if extracted from breadcrumb")
    section: Optional[str] = Field(None, description="Section title if extracted from breadcrumb")
    confidence: Optional[str] = Field(
        None,
        description="Confidence in page context extraction: 'high' if from breadcrumb, 'medium' if from URL, 'low' if guessed",
    )


class ChatRequest(BaseModel):
    """
    Extended request payload for RAG backend endpoint with optional selected text.
    The selected_text parameter enables search result boosting for improved relevance.
    """

    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User question (1-2000 characters)",
        example="What is forward kinematics?",
    )

    selected_text: Optional[str] = Field(
        None,
        min_length=0,
        max_length=500,
        description="Optional highlighted text from documentation page (used for search result boosting)",
        example="forward kinematics",
    )

    pageContext: Optional[PageContext] = Field(
        None,
        description="Optional page metadata where question was asked",
    )

    sessionId: Optional[str] = Field(
        None,
        description="Optional session identifier for tracking conversation",
        example="550e8400-e29b-41d4-a716-446655440000",
    )

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is forward kinematics?",
                "selected_text": "forward kinematics",
                "pageContext": {
                    "url": "http://localhost:3000/docs/chapter-3#forward-kinematics",
                    "pathname": "/docs/chapter-3",
                    "chapter": "Chapter 3",
                    "section": "Forward Kinematics",
                    "confidence": "high",
                },
                "sessionId": "550e8400-e29b-41d4-a716-446655440000",
            }
        }


class SearchResult(BaseModel):
    """
    Search result from Qdrant with boosting information.
    Includes original and boosted scores when selected_text boosting is applied.
    """

    chunk_id: str = Field(..., description="Unique chunk ID from vector database")
    text: str = Field(..., description="Text content of the chunk")
    original_score: float = Field(
        ...,
        ge=0,
        le=1,
        description="Original cosine similarity score (0-1)",
    )
    boosted_score: Optional[float] = Field(
        None,
        ge=0,
        le=2,
        description="Boosted score after applying selected text weighting (optional, only if boosting applied)",
    )
    tf_score: Optional[float] = Field(
        None,
        ge=0,
        le=1,
        description="Term frequency score for selected text terms (optional)",
    )
    title: Optional[str] = Field(None, description="Title of the source")
    url: Optional[str] = Field(None, description="URL to the source")


class ChatMetadata(BaseModel):
    """Operational metrics and search boosting information in response"""

    searchLatencyMs: int = Field(..., ge=0, description="Time spent on vector search (milliseconds)")
    generationLatencyMs: int = Field(
        ..., ge=0, description="Time spent on LLM generation (milliseconds)"
    )
    totalLatencyMs: int = Field(..., ge=0, description="Total end-to-end latency (milliseconds)")
    chunksRetrieved: Optional[int] = Field(
        None, ge=0, description="Number of chunks retrieved from vector database before boosting"
    )
    chunksUsed: Optional[int] = Field(
        None, ge=0, description="Number of chunks passed to LLM context window"
    )
    model: Optional[str] = Field(None, description="LLM model used for generation")
    tokensUsed: Optional[int] = Field(
        None, ge=0, description="Approximate total tokens used in generation"
    )
    selectedTextBoosted: Optional[bool] = Field(
        None,
        description="Whether selected_text parameter was provided and used for search boosting",
    )
    selectedTextTerms: Optional[List[str]] = Field(
        None,
        description="Terms extracted from selected_text that were used for boosting (only if boosted=true)",
    )
    boostFactor: Optional[float] = Field(
        None,
        ge=1.0,
        le=5.0,
        description="Multiplier applied to cosine similarity scores when selected_text matched (only if boosted=true)",
    )
    termFrequency: Optional[Dict[str, float]] = Field(
        None,
        description="Term frequency scores for selected_text terms in top results (optional, for debugging)",
    )


class ChatResponse(BaseModel):
    """
    Extended response payload from RAG backend with selected_text search boosting information.
    """

    answer: str = Field(
        ...,
        min_length=1,
        description="LLM-generated answer with optional citations",
        example="Forward kinematics is the process of determining the position and orientation of the end-effector given joint angles. See Chapter 3, Section 3.2 for details.",
    )

    sources: List[SearchResult] = Field(
        default_factory=list,
        max_items=10,
        description="Retrieved source chunks from vector database",
    )

    confidence: float = Field(
        ...,
        ge=0,
        le=1,
        description="Overall confidence in the answer (0-1). <0.8 means uncertain - 'I'm not entirely certain...' should be displayed",
        example=0.92,
    )

    metadata: ChatMetadata = Field(
        ...,
        description="Operational metrics and search boosting information",
    )

    error: Optional[Dict[str, str]] = Field(
        None,
        description="Error information (only present if request failed)",
    )

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "Forward kinematics is the process of determining the position and orientation of the end-effector given the joint angles. See Chapter 3, Section 3.2 for detailed mathematical derivations.",
                "sources": [
                    {
                        "chunk_id": "chunk-3-2-1",
                        "title": "Chapter 3, Section 3.2 - Forward Kinematics",
                        "text": "Forward kinematics is the process of determining the spatial position and orientation of the end-effector...",
                        "url": "http://localhost:3000/docs/chapter-3#forward-kinematics",
                        "original_score": 0.95,
                        "boosted_score": 1.425,
                        "tf_score": 0.9,
                    }
                ],
                "confidence": 0.92,
                "metadata": {
                    "searchLatencyMs": 145,
                    "generationLatencyMs": 1230,
                    "totalLatencyMs": 1375,
                    "chunksRetrieved": 5,
                    "chunksUsed": 3,
                    "model": "gemini-1.5-flash",
                    "tokensUsed": 450,
                    "selectedTextBoosted": True,
                    "selectedTextTerms": ["forward", "kinematics"],
                    "boostFactor": 1.5,
                    "termFrequency": {"forward": 0.95, "kinematics": 0.98},
                },
            }
        }
