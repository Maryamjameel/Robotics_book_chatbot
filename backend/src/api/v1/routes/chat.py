"""FastAPI router for RAG chat endpoint."""

import time
import uuid
from typing import Optional

from fastapi import APIRouter, HTTPException, Request

from src.config import logger
from src.models import ChatRequest, ChatResponse, RAGMetadata, Source
from src.services.rag_service import RAGService
from src.services.utils.chapter_filter import extract_tf_idf_terms

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

        # Validate chapter_context if provided
        chapter_context_valid = True
        if request.chapter_context:
            try:
                # Validate chapter_id is present and not empty
                chapter_id = request.chapter_context.get("chapter_id")
                if not chapter_id or not isinstance(chapter_id, str) or len(chapter_id.strip()) == 0:
                    logger.warning(
                        "Invalid chapter_context: chapter_id is required",
                        extra={
                            "operation": "chat_ask",
                            "request_id": request_id,
                            "status": "invalid_chapter_context",
                        },
                    )
                    chapter_context_valid = False
                    request.chapter_context = None  # Disable chapter filtering
                elif len(chapter_id) > 100:
                    logger.warning(
                        "Invalid chapter_context: chapter_id exceeds max length",
                        extra={
                            "operation": "chat_ask",
                            "request_id": request_id,
                            "chapter_id_length": len(chapter_id),
                            "status": "invalid_chapter_context",
                        },
                    )
                    chapter_context_valid = False
                    request.chapter_context = None
            except (TypeError, AttributeError) as e:
                logger.warning(
                    f"Invalid chapter_context format: {str(e)}",
                    extra={
                        "operation": "chat_ask",
                        "request_id": request_id,
                        "error": str(e),
                        "status": "invalid_chapter_context",
                    },
                )
                chapter_context_valid = False
                request.chapter_context = None

        logger.info(
            "Chat question received",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "question_length": len(request.question),
                "selected_text_provided": selected_text_boosted,
                "has_filters": request.filters is not None,
                "has_chapter_context": request.chapter_context is not None,
                "chapter_context_valid": chapter_context_valid,
                "status": "received",
            },
        )

        # Phase 4: Real vector search integration with chapter filtering
        from src.services.embedding_service import embed_question
        from src.services.qdrant_service import search_chunks
        from src.services.utils.search_boosting import SearchBoostingEngine

        search_start = time.time()

        # Step 1: Generate question embedding (now with rate limiting and retries)
        question_embedding = await embed_question(request.question)

        # Step 2: Extract terms from selected text for TF-IDF boosting
        selected_text_terms = None
        if request.selected_text and request.selected_text.strip():
            selected_text_terms = extract_tf_idf_terms(request.selected_text)
            logger.debug(
                "Extracted TF-IDF terms from selected text",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "terms_count": len(selected_text_terms),
                    "terms": selected_text_terms,
                },
            )

        # Step 3: Search for relevant chunks in Qdrant with chapter filtering
        search_result = search_chunks(
            question_embedding=question_embedding,
            top_k=5,
            relevance_threshold=0.5,  # Lowered for Gemini embeddings
            chapter_context=request.chapter_context,
            selected_text_terms=selected_text_terms,
        )

        # Extract results and metadata from search response
        retrieved_chunks = search_result.get("results", [])
        search_metadata = search_result.get("metadata", {})

        search_latency_ms = (time.time() - search_start) * 1000

        # Log chapter filtering information
        chapter_filtered = search_metadata.get("chapter_filtered", False)
        if chapter_filtered:
            logger.info(
                "Chapter context applied to search",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "chapter_id": search_metadata.get("chapter_id"),
                    "filtered_count": search_metadata.get("filtered_count"),
                    "boost_applied": search_metadata.get("boost_applied"),
                    "status": "chapter_filtered",
                },
            )

        # Convert dict results to expected format
        # The search_chunks now returns formatted results with chapter_title
        if retrieved_chunks and isinstance(retrieved_chunks[0], dict):
            # Results are already in dict format from search_chunks
            # Ensure all required fields are present
            for chunk in retrieved_chunks:
                if "chapter_title" not in chunk:
                    chunk["chapter_title"] = "Unknown"
        else:
            # Convert to expected format if needed
            retrieved_chunks = [
                {
                    "chapter_id": r.get("chapter_id", "unknown"),
                    "chapter_title": r.get("chapter_title", "Unknown"),
                    "section_number": r.get("section_number", 0),
                    "section_title": r.get("section_title", ""),
                    "excerpt": r.get("excerpt", ""),
                    "relevance_score": r.get("relevance_score", 0.0),
                }
                for r in retrieved_chunks
            ]

        # Step 4: Apply legacy search boosting if selected text provided (for backward compatibility)
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

        # Step 5: Handle no results case
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
                confidence=0.0,
                metadata=RAGMetadata(
                    confidence_score=0.0,
                    search_latency_ms=search_latency_ms,
                    generation_latency_ms=0.0,
                    total_latency_ms=search_latency_ms,
                ),
            )

        # Step 6: Call RAG service to generate answer from chunks
        # RAG service now always returns a response, even if LLM fails (with fallback)
        try:
            response = await rag_service.answer_question(
                question=request.question,
                retrieved_chunks=retrieved_chunks,
                request_id=request_id,
            )
        except ValueError as e:
            # Configuration error - return fallback response
            logger.error(
                f"Configuration error, returning fallback: {str(e)}",
                extra={
                    "operation": "chat_ask",
                    "request_id": request_id,
                    "error": str(e),
                    "status": "configuration_error_fallback",
                },
            )
            # Extract sources from retrieved chunks for fallback
            sources = [
                Source(
                    chapter_id=chunk.get("chapter_id", "unknown"),
                    chapter_title=chunk.get("chapter_title", "Unknown"),
                    section_number=chunk.get("section_number", 0),
                    section_title=chunk.get("section_title", "Unknown"),
                    excerpt=chunk.get("excerpt", "")[:500],
                    relevance_score=chunk.get("relevance_score", 0.0),
                )
                for chunk in retrieved_chunks
            ]

            response = ChatResponse(
                answer=(
                    "I apologize, but there's a configuration issue preventing me from generating a complete answer. "
                    "However, I found relevant content in the textbook. Please review the sources below, "
                    "or contact support if this issue persists."
                ),
                sources=sources,
                confidence=0.5,
                metadata=RAGMetadata(
                    confidence_score=0.5,
                    search_latency_ms=search_latency_ms,
                    generation_latency_ms=0.0,
                    total_latency_ms=(time.time() - pipeline_start) * 1000,
                ),
            )

        # Step 7: Update response metadata with latency, chapter filtering, and boosting info
        response.metadata.search_latency_ms = search_latency_ms
        total_latency_ms = (time.time() - pipeline_start) * 1000
        response.metadata.total_latency_ms = total_latency_ms

        # Set confidence at top level (matches metadata.confidence_score)
        response.confidence = response.metadata.confidence_score

        # Add chapter filtering metadata if chapter context was applied
        if chapter_filtered:
            response.metadata.chapter_filtered = True
            response.metadata.chapter_id = search_metadata.get("chapter_id")

        # Add boosting metadata if boosting was applied
        if boost_metadata:
            response.metadata.selected_text_boosted = True
            response.metadata.boost_factor = boost_metadata.get("boost_factor", 1.0)
            response.metadata.selected_text_terms = boost_metadata.get("terms", [])

        # Add TF-IDF boosting metadata from selected text terms if provided
        if selected_text_terms:
            response.metadata.selected_text_boosted = True
            response.metadata.selected_text_terms = selected_text_terms

        logger.info(
            "Chat question answered successfully",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "answer_length": len(response.answer),
                "source_count": len(response.sources),
                "confidence": response.metadata.confidence_score,
                "total_latency_ms": total_latency_ms,
                "selected_text_boosted": bool(boost_metadata or selected_text_terms),
                "chapter_filtered": chapter_filtered,
                "chapter_id": search_metadata.get("chapter_id") if chapter_filtered else None,
                "status": "success",
            },
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions (validation errors only)
        raise

    except Exception as e:
        # Catch-all for any unexpected errors - return a helpful response instead of failing
        logger.error(
            f"Unexpected error in chat endpoint: {type(e).__name__}: {str(e)}",
            extra={
                "operation": "chat_ask",
                "request_id": request_id,
                "error_type": type(e).__name__,
                "error": str(e),
                "status": "unexpected_error_fallback",
            },
        )

        # Return a helpful fallback response instead of raising 503
        return ChatResponse(
            answer=(
                "I apologize, but I encountered an unexpected error while processing your question. "
                "Please try asking your question again, or rephrase it. If the problem persists, "
                "please contact support."
            ),
            sources=[],
            confidence=0.3,
            metadata=RAGMetadata(
                confidence_score=0.3,
                search_latency_ms=0.0,
                generation_latency_ms=0.0,
                total_latency_ms=(time.time() - pipeline_start) * 1000,
            ),
        )
