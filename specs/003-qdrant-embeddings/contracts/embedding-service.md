# Service Contract: Embedding Service

**Service**: `backend/src/services/embedding_service.py`
**Purpose**: Generate vector embeddings for text chunks using OpenAI API

## Functions

### `embed_batch(texts: List[str], model: str = "text-embedding-3-small", batch_id: str = None) -> List[List[float]]`

Generate embeddings for a batch of text chunks with automatic retry on rate limits.

**Input**:
```python
texts: List[str]  # 1-32 text strings to embed (each < 8000 chars)
model: str = "text-embedding-3-small"  # Embedding model (default from research)
batch_id: str = None  # Optional batch identifier for logging
```

**Output**:
```python
List[List[float]]  # List of 1536-dimensional vectors (one per text)
```

**Behavior**:
1. Validate batch size: 1-32 texts
2. Validate each text: 1-8000 characters
3. Call OpenAI embedding API: `client.embeddings.create(input=texts, model=model)`
4. Extract vectors from response
5. Validate dimensions: exactly 1536 per vector
6. Return vectors in same order as input texts

**Retry Logic** (on rate limit or transient error):
1. First attempt: immediate
2. Rate limit error: wait 2^(retry_count) seconds, retry (max 3 attempts, 1s + 2s + 4s = 7s total)
3. After 3 failed retries: raise `EmbeddingAPIError` with context

**Error Handling**:
- Invalid batch size: Raise `ValueError("Batch size must be 1-32")`
- Text too long: Raise `ValueError(f"Text exceeds 8000 chars: {len(text)}")`
- API authentication error: Raise `AuthenticationError(f"Invalid OpenAI API key")`
- Rate limit (transient): Retry with exponential backoff
- Rate limit (max retries): Raise `RateLimitError(f"Max retries exceeded after {total_time}s")`
- Network timeout: Retry with exponential backoff (count as rate limit)
- Other API error: Raise `EmbeddingAPIError(f"OpenAI API error: {error}")`

**Example**:

```python
from backend.src.services.embedding_service import embed_batch

texts = [
    "Forward kinematics solves for end-effector position given joint angles.",
    "Inverse kinematics solves the reverse problem: joint angles given end-effector position."
]

embeddings = embed_batch(texts, batch_id="ch03_batch_1")
# Returns:
# [
#   [0.0234, -0.0187, 0.0456, ..., (1536 total dimensions)],
#   [0.0125, 0.0456, 0.0234, ..., (1536 total dimensions)]
# ]
```

**Rate Limit Handling Example**:

```python
# First attempt fails with 429 (rate limit)
# Service waits 2^1 = 2 seconds
# Second attempt fails with 429
# Service waits 2^2 = 4 seconds
# Third attempt succeeds
# Returns embeddings after ~6s total wait
```

---

### `embed_chunks(chunks: List[ChapterChunk], batch_size: int = 32) -> List[TextEmbedding]`

Generate embeddings for multiple chunks with automatic batching and error recovery.

**Input**:
```python
chunks: List[ChapterChunk]  # 1-1000+ chunks to embed
batch_size: int = 32  # Number of chunks per API batch (1-32)
```

**Output**:
```python
List[TextEmbedding]  # Embedded chunks with 1536-dim vectors
# Note: Failed chunks skipped (graceful degradation)
```

**Behavior**:
1. Split chunks into batches of `batch_size`
2. For each batch:
   a. Extract content text from chunks
   b. Call `embed_batch(texts, batch_id=...)`
   c. Create TextEmbedding objects with chunk metadata
   d. Collect results or errors
3. Log statistics: total, succeeded, failed
4. Return list of successfully embedded chunks

**Error Handling** (per batch):
- If batch fails after 3 retries: log error, skip batch, continue (graceful degradation)
- Partial batch failure: skip failed chunks, continue with next batch
- No exceptions raised: always returns list (possibly shorter if some batches failed)

**Example**:

```python
from backend.src.services.embedding_service import embed_chunks
from backend.src.models.chunk import ChapterChunk

chunks = [
    ChapterChunk(chapter_id="ch01", section_number=0, content="..."),
    ChapterChunk(chapter_id="ch01", section_number=1, content="..."),
    # ... 500 more chunks
]

embeddings = embed_chunks(chunks, batch_size=32)
# Process 500 chunks in 16 batches (~1.5 minutes with rate limits)
# Returns ~495 TextEmbedding objects (5 failed, logged and skipped)
print(f"Embedded {len(embeddings)}/{len(chunks)} chunks")
```

---

## Data Contracts

### TextEmbedding Model (Pydantic)

```python
from pydantic import BaseModel, Field, validator
from typing import List, Dict, Any, Optional

class TextEmbedding(BaseModel):
    chunk_id: str = Field(..., min_length=1)  # "{chapter_id}_{section_number}"
    vector: List[float] = Field(..., min_items=1536, max_items=1536)
    metadata: Dict[str, Any] = Field(...)

    @validator('vector')
    def vector_dimensions(cls, v):
        if len(v) != 1536:
            raise ValueError(f"Vector must have exactly 1536 dimensions, got {len(v)}")
        return v

    @validator('vector')
    def vector_range(cls, v):
        for val in v:
            if not -1.0 <= val <= 1.0:
                raise ValueError(f"Vector values must be in [-1, 1] range")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "ch03_2",
                "vector": [0.0234, -0.0187, 0.0456, ... ],  # 1536 values
                "metadata": {
                    "chapter_id": "ch03",
                    "section_number": 2,
                    "title": "Kinematics and Motion Planning > Forward Kinematics",
                    "content": "Forward kinematics is..."
                }
            }
        }
```

---

## Configuration Contract

**Environment Variables** (required):

```bash
OPENAI_API_KEY=sk-...  # OpenAI API key (stored in .env, never committed)
EMBEDDING_MODEL=text-embedding-3-small  # Optional, defaults to text-embedding-3-small
EMBEDDING_BATCH_SIZE=32  # Optional, defaults to 32
EMBEDDING_MAX_RETRIES=3  # Optional, max retry attempts
```

**Configuration Defaults**:

```python
DEFAULT_EMBEDDING_MODEL = "text-embedding-3-small"
DEFAULT_BATCH_SIZE = 32
DEFAULT_MAX_RETRIES = 3
DEFAULT_INITIAL_BACKOFF = 1  # seconds
DEFAULT_MAX_BACKOFF = 8  # seconds
```

---

## Testing Contract

**Unit Tests** (`backend/tests/unit/test_embedding_service.py`):

1. **test_embed_batch_single_text**: Embed 1 text, verify 1536-dim output
2. **test_embed_batch_multiple_texts**: Embed 32 texts, verify all returned correctly
3. **test_embed_batch_invalid_size**: Batch size 0 or 33 raises ValueError
4. **test_embed_batch_text_too_long**: Text > 8000 chars raises ValueError
5. **test_embed_batch_empty_text**: Empty text raises ValueError
6. **test_embed_batch_rate_limit_retry**: Mock API rate limit, verify exponential backoff and success on retry
7. **test_embed_batch_max_retries_exceeded**: Mock API rate limit 3+ times, verify error raised
8. **test_embed_chunks_batching**: 100 chunks with batch_size=32, verify 4 batches created
9. **test_embed_chunks_graceful_failure**: 1 batch fails, others succeed, verify partial results returned
10. **test_embed_chunks_logging**: Verify embedding statistics logged (total, succeeded, failed)

**Integration Tests** (`backend/tests/integration/test_end_to_end_pipeline.py`):

- Mock OpenAI API with sample embeddings
- Embed 50-chunk sample, verify TextEmbedding structure correct
- Verify metadata properly attached to embeddings

---

## Performance Expectations

- **Single text embedding**: ~20ms (OpenAI API latency)
- **Batch of 32 texts**: ~200ms (API latency + processing)
- **1000 chunks (32 batches)**: ~15 minutes (including rate limit backoff)
- **Memory per batch**: < 5MB (1536-dim × 32 chunks × 4 bytes float)
- **Retry overhead**: 0-7 seconds per rate limit (exponential backoff)

---

## Rate Limit Behavior

**OpenAI API Rate Limits**:
- Requests per minute: 3,500 (for text-embedding-3-small)
- Tokens per minute: unlimited for embedding API

**Expected behavior at scale**:
- 32-text batch = ~8,000 tokens
- ~3500 batches/min ÷ 32 = ~110 batches/min
- 100 chapters × 5 sections = 500 chunks
- 500 chunks ÷ 32 batch_size = 16 batches
- 16 batches at 110/min = ~9 seconds (no rate limit hits)

**If rate limit encountered**:
- Service automatically backs off with exponential delay
- Logs rate limit hits for observability
- Retries up to 3 times before failing

---

## Backwards Compatibility

This is a new service. No backwards compatibility concerns.

**Future Extensibility**:
- Support alternative embedding models (configurable via environment variable)
- Stream embedding responses for very large batches
- Cache embeddings to avoid re-embedding unchanged content
