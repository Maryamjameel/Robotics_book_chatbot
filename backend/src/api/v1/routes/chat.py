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

        # Validate selected_text if provided
        selected_text_boosted = False
        if request.selected_text is not None:
            if len(request.selected_text) > 500:
                logger.warning(
                    "Selected text exceeds maximum length",
                    extra={
                        "operation": "chat_ask",
                        "request_id": request_id,
                        "selected_text_length": len(request.selected_text),
                        "status": "bad_request",
                    },
                )
                raise HTTPException(
                    status_code=400,
                    detail="Selected text must be 500 characters or less",
                )

            # Check if selected text is not just whitespace
            if request.selected_text.strip():
                selected_text_boosted = True
                logger.debug(
                    "Selected text provided for search boosting",
                    extra={
                        "operation": "chat_ask",
                        "request_id": request_id,
                        "selected_text_length": len(request.selected_text),
                    },
                )

        logger.info(
            "Chat question received",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "question_length": len(request.question),
                "selected_text_provided": selected_text_boosted,
                "has_filters": request.filters is not None,
                "status": "received",
            },
        )

        # Phase 4: Real vector search integration
        from src.services.embedding_service import embed_question
        from src.services.qdrant_service import search_chunks
        from src.services.utils.search_boosting import SearchBoostingEngine

        search_start = time.time()

        # Step 1: Generate question embedding
        question_embedding = await embed_question(request.question)

        # Step 2: Search for relevant chunks in Qdrant
        retrieved_chunks = search_chunks(
            question_embedding=question_embedding,
            top_k=5,
            relevance_threshold=0.7,
        )

        search_latency_ms = (time.time() - search_start) * 1000

        # Step 3: Apply search boosting if selected text provided
        boosting_engine = SearchBoostingEngine()
        boost_metadata = {}

        if selected_text_boosted and retrieved_chunks:
            try:
                # Apply TF-IDF boosting to re-rank results
                retrieved_chunks = boosting_engine.boost_scores(
                    search_results=retrieved_chunks,
                    selected_text=request.selected_text
                )

                # Get boosting metadata for response
                boost_metadata = boosting_engine.get_boost_metadata(
                    selected_text=request.selected_text
                )

                logger.debug(
                    "Search results boosted with selected text",
                    extra={
                        "operation": "chat_ask",
                        "request_id": request_id,
                        "boost_factor": boost_metadata.get("boost_factor"),
                        "terms_extracted": len(boost_metadata.get("terms", [])),
                        "status": "boosting_applied",
                    },
                )
            except Exception as e:
                logger.warning(
                    f"Search boosting failed, using original results: {str(e)}",
                    extra={
                        "operation": "chat_ask",
                        "request_id": request_id,
                        "error": str(e),
                        "status": "boosting_failed",
                    },
                )
                # Continue with original results if boosting fails

        # Step 4: Handle no results case
        if not retrieved_chunks:
            logger.info(
                "No relevant content found",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "status": "no_results",
                },
            )
            return ChatResponse(
                answer="No relevant content found in the robotics textbook about your query. Please try rephrasing your question.",
                sources=[],
                metadata=RAGMetadata(
                    confidence_score=0.0,
                    search_latency_ms=search_latency_ms,
                    generation_latency_ms=0.0,
                    total_latency_ms=search_latency_ms,
                ),
            )

        # Step 5: Call RAG service to generate answer from chunks
        response = await rag_service.answer_question(
            question=request.question,
            retrieved_chunks=retrieved_chunks,
            request_id=request_id,
        )

        # Step 6: Update response metadata with latency and boosting info
        response.metadata.search_latency_ms = search_latency_ms
        total_latency_ms = (time.time() - pipeline_start) * 1000
        response.metadata.total_latency_ms = total_latency_ms

        # Add boosting metadata if boosting was applied
        if boost_metadata:
            response.metadata.selected_text_boosted = True
            response.metadata.boost_factor = boost_metadata.get("boost_factor", 1.0)
            response.metadata.selected_text_terms = boost_metadata.get("terms", [])

        logger.info(
            "Chat question answered successfully",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "answer_length": len(response.answer),
                "source_count": len(response.sources),
                "confidence": response.metadata.confidence_score,
                "total_latency_ms": total_latency_ms,
                "selected_text_boosted": bool(boost_metadata),
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
