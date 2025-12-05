"""
Request schema for chat endpoint with selected text support.
Validates incoming requests with optional selected_text parameter for search boosting.
"""

from typing import Optional
from pydantic import BaseModel, Field, validator


class ChatRequestSchema(BaseModel):
    """
    Validated chat request schema.
    Ensures question and selected_text meet length and format requirements.
    """

    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User question",
    )

    selected_text: Optional[str] = Field(
        None,
        min_length=0,
        max_length=500,
        description="Optional selected text from page for search boosting",
    )

    @validator("question")
    def validate_question(cls, v):
        """Validate question is not just whitespace"""
        if v and not v.strip():
            raise ValueError("Question cannot be only whitespace")
        return v.strip()

    @validator("selected_text")
    def validate_selected_text(cls, v):
        """Validate selected_text if provided"""
        if v is not None:
            # Allow empty string (field is optional), but if provided, strip whitespace
            if isinstance(v, str):
                v = v.strip()
                # If it's now empty after stripping, treat as None
                if not v:
                    return None
        return v

    class Config:
        """Pydantic config"""

        str_strip_whitespace = True
        extra = "forbid"  # Reject unknown fields


class ChatResponseMetadataSchema(BaseModel):
    """Metadata in chat response"""

    searchLatencyMs: int = Field(..., ge=0)
    generationLatencyMs: int = Field(..., ge=0)
    totalLatencyMs: int = Field(..., ge=0)
    selectedTextBoosted: Optional[bool] = None
    selectedTextTerms: Optional[list[str]] = None
    boostFactor: Optional[float] = None


class ChatResponseSchema(BaseModel):
    """Validated chat response schema"""

    answer: str = Field(..., min_length=1)
    sources: list = Field(default_factory=list)
    confidence: float = Field(..., ge=0, le=1)
    metadata: ChatResponseMetadataSchema

    class Config:
        """Pydantic config"""

        extra = "forbid"  # Reject unknown fields
