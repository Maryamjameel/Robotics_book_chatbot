"""Embedding generation service using Google Gemini API."""

import asyncio
import time
from typing import List

import google.generativeai as genai

from ..config import config, logger
from ..models import ChapterChunk, TextEmbedding


# Configure Gemini API
genai.configure(api_key=config.gemini_api_key)


def embed_chunks(chunks: List[ChapterChunk]) -> List[TextEmbedding]:
    """
    Generate embeddings for a list of chapter chunks.

    Args:
        chunks: List of ChapterChunk objects to embed

    Returns:
        List of TextEmbedding objects with generated vectors

    Raises:
        ValueError: If chunk content is empty or invalid
        RuntimeError: If embedding API call fails after retries
    """
    if not chunks:
        raise ValueError("Cannot embed empty list of chunks")

    # Extract texts and metadata
    texts = [chunk.content for chunk in chunks]
    chunk_ids = [
        f"{chunk.chapter_id}_sec{chunk.section_number:02d}" for chunk in chunks
    ]

    # Create complete metadata with all required fields for RAG service
    metadata_list = []
    for chunk in chunks:
        metadata = {
            "chapter_id": chunk.chapter_id,
            "chapter_title": chunk.chapter_title,
            "section_number": chunk.section_number,
            "section_title": chunk.section_title,
            "text": chunk.content,  # Include the actual content for context
            **chunk.metadata,  # Include any additional metadata from the chunk
        }
        metadata_list.append(metadata)

    return embed_batch(texts, chunk_ids, metadata_list)


def embed_batch(texts: List[str], chunk_ids: List[str], metadata_list: List[dict]) -> List[TextEmbedding]:
    """
    Generate embeddings for a batch of texts with exponential backoff retry logic.

    Args:
        texts: List of text strings to embed
        chunk_ids: Corresponding chunk identifiers for each text
        metadata_list: Corresponding metadata dictionaries for each text

    Returns:
        List of TextEmbedding objects with generated vectors

    Raises:
        ValueError: If lists are not the same length or empty
        RuntimeError: If all retry attempts fail
    """
    if not texts or not chunk_ids or not metadata_list:
        raise ValueError("Cannot embed empty lists")

    if not (len(texts) == len(chunk_ids) == len(metadata_list)):
        raise ValueError("Texts, chunk_ids, and metadata_list must have same length")

    # Validate that all texts are non-empty strings
    for i, text in enumerate(texts):
        if not text or not text.strip():
            raise ValueError(f"Empty text at index {i}")

    embeddings = []
    last_error = None

    # Retry logic with exponential backoff
    for attempt in range(config.max_retries):
        try:
            logger.info(
                "Generating embeddings with Gemini",
                extra={
                    "operation": "embed_batch",
                    "status": "in_progress",
                    "batch_size": len(texts),
                    "attempt": attempt + 1,
                    "model": config.gemini_embedding_model,
                },
            )

            # Generate embeddings for each text using Gemini
            for i, text in enumerate(texts):
                result = genai.embed_content(
                    model=f"models/{config.gemini_embedding_model}",
                    content=text,
                    task_type="retrieval_document",
                )

                embedding = TextEmbedding(
                    chunk_id=chunk_ids[i],
                    vector=result['embedding'],
                    metadata=metadata_list[i],
                )
                embeddings.append(embedding)

            logger.info(
                "Embeddings generated successfully",
                extra={
                    "operation": "embed_batch",
                    "status": "success",
                    "count": len(embeddings),
                },
            )

            return embeddings

        except Exception as e:
            last_error = e
            embeddings = []  # Reset on failure
            if attempt < config.max_retries - 1:
                # Calculate backoff: 2^attempt seconds
                backoff = config.initial_backoff_seconds * (2**attempt)
                logger.warning(
                    f"Embedding generation failed, retrying in {backoff}s",
                    extra={
                        "operation": "embed_batch",
                        "status": "retry",
                        "attempt": attempt + 1,
                        "error": str(e),
                        "backoff_seconds": backoff,
                    },
                )
                time.sleep(backoff)
            else:
                logger.error(
                    "Embedding generation failed after all retries",
                    extra={
                        "operation": "embed_batch",
                        "status": "failed",
                        "attempts": config.max_retries,
                        "error": str(e),
                    },
                    exc_info=True,
                )

    raise RuntimeError(f"Failed to generate embeddings after {config.max_retries} attempts: {last_error}")


async def embed_question(question: str) -> List[float]:
    """Generate embedding for a single question with rate limiting (async wrapper).

    Args:
        question: Question text to embed

    Returns:
        768-dimensional embedding vector (Gemini text-embedding-004)

    Raises:
        RuntimeError: If embedding fails or rate limit exceeded
    """
    # Import global rate limiter to avoid circular imports
    from .rate_limiter import gemini_rate_limiter

    # Retry logic with exponential backoff for rate limiting
    max_attempts = 3
    last_error = None

    for attempt in range(max_attempts):
        # Acquire rate limit slot BEFORE making API call
        await gemini_rate_limiter.acquire()

        try:
            loop = asyncio.get_event_loop()
            embedding = await asyncio.wait_for(
                loop.run_in_executor(None, _embed_question_sync, question),
                timeout=10.0,
            )

            logger.info(
                "Question embedding generated",
                extra={
                    "operation": "embed_question",
                    "question_length": len(question),
                    "embedding_dim": len(embedding),
                    "status": "success",
                    "attempt": attempt + 1,
                },
            )

            return embedding

        except asyncio.TimeoutError as e:
            last_error = e
            logger.error(
                "Embedding request timed out",
                extra={
                    "operation": "embed_question",
                    "error": "timeout",
                    "status": "failed",
                    "attempt": attempt + 1,
                },
            )
            if attempt == max_attempts - 1:
                raise RuntimeError("Embedding service timeout") from e

        except Exception as e:
            last_error = e
            error_msg = str(e).lower()

            # Check if it's a rate limit error
            if "rate limit" in error_msg or "429" in error_msg or "quota" in error_msg:
                if attempt < max_attempts - 1:
                    # Exponential backoff for rate limit errors
                    backoff = 1.0 * (2 ** attempt)
                    logger.warning(
                        f"Rate limit hit, retrying in {backoff}s",
                        extra={
                            "operation": "embed_question",
                            "error": str(e),
                            "status": "rate_limited",
                            "attempt": attempt + 1,
                            "backoff_seconds": backoff,
                        },
                    )
                    await asyncio.sleep(backoff)
                    continue
                else:
                    logger.error(
                        "Rate limit exceeded after all retries",
                        extra={
                            "operation": "embed_question",
                            "error": str(e),
                            "status": "failed",
                            "attempts": max_attempts,
                        },
                    )
                    raise RuntimeError("Rate limit exceeded. Please try again later.") from e
            else:
                logger.error(
                    f"Embedding failed: {str(e)}",
                    extra={
                        "operation": "embed_question",
                        "error": str(e),
                        "status": "failed",
                        "attempt": attempt + 1,
                    },
                )
                if attempt == max_attempts - 1:
                    raise RuntimeError(f"Embedding failed: {str(e)}") from e

    # If we get here, all retries failed
    raise RuntimeError(f"Embedding failed after {max_attempts} attempts: {last_error}")


def _embed_question_sync(question: str) -> List[float]:
    """Synchronous embedding implementation for a single question.

    Args:
        question: Text to embed

    Returns:
        768-dimensional embedding vector
    """
    result = genai.embed_content(
        model=f"models/{config.gemini_embedding_model}",
        content=question,
        task_type="retrieval_query",
    )
    return result['embedding']
