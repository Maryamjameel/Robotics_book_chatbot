"""FastAPI router for RAG chat endpoint."""

import time
import uuid
from typing import Optional

from fastapi import APIRouter, HTTPException, Request

from src.config import logger
from src.models import ChatRequest, ChatResponse
from src.services.rag_service import RAGService

# Initialize router and RAG service
router = APIRouter(prefix="/api/v1/chat", tags=["chat"])
rag_service = RAGService()


@router.post("/ask", response_model=ChatResponse, status_code=200)
async def ask_question(request: ChatRequest, http_request: Request) -> ChatResponse:
    """Ask a natural language question about robotics textbook content.

    Returns answer with citations and confidence score.

    Args:
        request: ChatRequest with question and optional filters
        http_request: FastAPI request object for context

    Returns:
        ChatResponse with answer, sources, and metadata

    Raises:
        HTTPException: 400 if invalid input, 429 if rate limited, 503 if service unavailable
    """
    # Generate request ID for tracing
    request_id = str(uuid.uuid4())
    pipeline_start = time.time()

    try:
        # Validate input
        if not request.question or len(request.question.strip()) == 0:
            logger.warning(
                "Empty question submitted",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "status": "bad_request",
                },
            )
            raise HTTPException(
                status_code=400, detail="Question cannot be empty"
            )

        if len(request.question) > 2000:
            logger.warning(
                "Question exceeds maximum length",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "question_length": len(request.question),
                    "status": "bad_request",
                },
            )
            raise HTTPException(
                status_code=400,
                detail="Question must be 2000 characters or less",
            )

        logger.info(
            "Chat question received",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "question_length": len(request.question),
                "has_filters": request.filters is not None,
                "status": "received",
            },
        )

        # TODO: In Phase 4, integrate with Qdrant vector search
        # For now, use mock chunks for demonstration
        mock_chunks = [
            {
                "score": 0.89,
                "payload": {
                    "chapter_id": "ch03",
                    "chapter_title": "Kinematics",
                    "section_number": 1,
                    "section_title": "Forward Kinematics",
                    "text": "Forward kinematics is the process of calculating the end-effector position and orientation given the joint angles of a robotic arm.",
                },
            }
        ]

        # Call RAG service
        search_start = time.time()
        search_latency_ms = (time.time() - search_start) * 1000

        response = await rag_service.answer_question(
            question=request.question,
            retrieved_chunks=mock_chunks,
            request_id=request_id,
        )

        # Update search latency in metadata
        response.metadata.search_latency_ms = search_latency_ms
        total_latency_ms = (time.time() - pipeline_start) * 1000
        response.metadata.total_latency_ms = total_latency_ms

        logger.info(
            "Chat question answered successfully",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "answer_length": len(response.answer),
                "source_count": len(response.sources),
                "confidence": response.metadata.confidence_score,
                "total_latency_ms": total_latency_ms,
                "status": "success",
            },
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions (validation errors)
        raise

    except ValueError as e:
        # Configuration errors
        logger.error(
            f"Configuration error: {str(e)}",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "error": str(e),
                "status": "configuration_error",
            },
        )
        raise HTTPException(
            status_code=503, detail="Service temporarily unavailable"
        ) from e

    except RuntimeError as e:
        # Service errors (rate limit, timeout, generation failure)
        error_msg = str(e).lower()

        if "rate limit" in error_msg:
            logger.warning(
                "Rate limit exceeded",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "status": "rate_limited",
                },
            )
            raise HTTPException(
                status_code=429,
                detail="Rate limit exceeded. Maximum 5 requests per second.",
                headers={"Retry-After": "1"},
            ) from e

        elif "timeout" in error_msg:
            logger.warning(
                "Generation timeout",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "status": "timeout",
                },
            )
            raise HTTPException(
                status_code=503, detail="Request processing timed out"
            ) from e

        else:
            logger.error(
                f"LLM service error: {str(e)}",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "error": str(e),
                    "status": "service_error",
                },
            )
            raise HTTPException(
                status_code=503,
                detail="LLM service temporarily unavailable",
            ) from e

    except Exception as e:
        # Unexpected errors
        logger.error(
            f"Unexpected error in chat endpoint: {type(e).__name__}: {str(e)}",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "error_type": type(e).__name__,
                "error": str(e),
                "status": "unexpected_error",
            },
        )
        raise HTTPException(
            status_code=503, detail="Internal server error"
        ) from e
