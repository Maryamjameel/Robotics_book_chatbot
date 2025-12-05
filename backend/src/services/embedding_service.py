"""Embedding generation service using OpenAI API."""

import time
from typing import List

from openai import OpenAI

from ..config import config, logger
from ..models import ChapterChunk, TextEmbedding


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
    metadata_list = [chunk.metadata for chunk in chunks]

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

    client = OpenAI(api_key=config.openai_api_key)
    embeddings = []
    last_error = None

    # Retry logic with exponential backoff
    for attempt in range(config.max_retries):
        try:
            logger.info(
                "Generating embeddings",
                extra={
                    "operation": "embed_batch",
                    "status": "in_progress",
                    "batch_size": len(texts),
                    "attempt": attempt + 1,
                },
            )

            response = client.embeddings.create(
                input=texts,
                model=config.embedding_model,
            )

            # Process response
            for i, embedding_data in enumerate(response.data):
                embedding = TextEmbedding(
                    chunk_id=chunk_ids[i],
                    vector=embedding_data.embedding,
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
