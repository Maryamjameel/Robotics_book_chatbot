# ADR-003: Modular Service Architecture with Batch Processing and Retry Logic

**Status**: Proposed
**Decided**: 2025-12-05
**Feature**: 003-qdrant-embeddings
**References**: specs/003-qdrant-embeddings/plan.md, specs/003-qdrant-embeddings/contracts/

## Context

The embeddings pipeline must index large textbooks (100+ chapters, 500+ sections) reliably while handling external API constraints (OpenAI rate limits, Qdrant connection issues). Three core operations are required:

1. **Chunking**: Parse markdown files → extract sections
2. **Embedding**: Call OpenAI API → generate vectors
3. **Insertion**: Write vectors → Qdrant storage

Constitutional principle "Modular & Testable Architecture" requires decoupled services with clear contracts and testable boundaries. Additionally, "Production-Grade Quality" requires comprehensive error handling and graceful degradation.

Key constraints:
- OpenAI API: 3500 requests/minute (2.4 requests/second)
- Transient failures: Rate limits, timeouts, connection issues
- User expectation: Index 100-chapter textbook without manual intervention
- Observability requirement: Log all operations for debugging

A monolithic script would couple concerns and make testing/debugging difficult. A modular approach enables:
- Independent testing of each service
- Batch processing to respect rate limits
- Graceful degradation (skip failed batches, continue)
- Clear error logging and retry logic

## Decision

Adopt **Modular Service Architecture with Async Batch Processing and Exponential Backoff Retry**:

### Architecture Components

1. **Markdown Parser Service** (`backend/src/services/markdown_parser.py`)
   - Pure functions: `parse_chapter(filepath)`, `parse_all_chapters(directory)`
   - Input: filesystem paths
   - Output: List[ChapterChunk] (in-memory)
   - No external dependencies (file I/O only)

2. **Embedding Service** (`backend/src/services/embedding_service.py`)
   - Functions: `embed_batch(texts)`, `embed_chunks(chunks, batch_size)`
   - Input: List[ChapterChunk]
   - Output: List[TextEmbedding] with 1536-dim vectors
   - External dependency: OpenAI API (injected via environment variable)
   - Implements: Batch processing (32 chunks/batch), exponential backoff retry (max 3 attempts)

3. **Qdrant Service** (`backend/src/services/qdrant_service.py`)
   - Functions: `initialize_collection()`, `insert_embeddings()`, `verify_insertion()`
   - Input: List[TextEmbedding]
   - Output: InsertionResult (success count, error list)
   - External dependency: Qdrant Cloud instance (injected via env)
   - Implements: Upsert semantics (idempotent), insertion verification

4. **Pipeline Orchestrator** (`backend/scripts/process_all.py`)
   - Coordinates three services: chunk → embed → insert
   - Implements: Error handling, logging, summary reporting
   - No business logic; pure orchestration

### Batch Processing Pattern

```
Input: 500 chunks
├─ MarkdownParser: 100 files → 500 ChapterChunk objects (sequential)
├─ EmbeddingService:
│  ├─ Batch 1: [chunk 0-31] → embed_batch() → [vector 0-31]
│  ├─ Batch 2: [chunk 32-63] → embed_batch() → [vector 32-63]
│  ├─ ... (16 batches total)
│  └─ Handles rate limits: if 429 error, exponential backoff (1s, 2s, 4s)
├─ QdrantService:
│  ├─ Upsert batch 1 (32 vectors)
│  ├─ Upsert batch 2 (32 vectors)
│  └─ ... (all batches inserted)
└─ Summary: {total: 500, embedded: 495, inserted: 495, failed: 5, time: 287s}
```

### Error Handling Strategy

**Graceful Degradation**:
- Failed chunks logged with error reason
- Pipeline continues processing remaining chunks
- Final report summarizes successes and failures
- No exceptions raised; always return results (partial or complete)

**Retry Logic**:
- OpenAI API rate limit: exponential backoff (2^attempt seconds, max 3 attempts)
- Qdrant connection timeout: retry once with 1s delay
- Other errors: log and skip (no retry)

**Logging**:
- JSON structured logs with timestamp, operation, file/chunk id, status
- Format: `{"timestamp": "2025-12-05T14:32:10Z", "operation": "embed", "batch_id": "ch03_batch_1", "status": "success", "latency_ms": 1850}`

## Consequences

### Positive Outcomes

- **Testability**: Each service independently testable (unit tests without external APIs)
- **Reusability**: Services can be used in other contexts (e.g., incremental chunk updates, real-time indexing)
- **Clear responsibility**: MarkdownParser = parsing, EmbeddingService = API calls, QdrantService = storage
- **Graceful degradation**: Failed batches don't block pipeline; administrator sees clear error log
- **Rate limit handling**: Batch processing (32 chunks) respects OpenAI 3500 req/min limit with exponential backoff
- **Observability**: JSON logs enable debugging and performance monitoring
- **Idempotent insertion**: Upsert semantics allow re-indexing without duplicates

### Negative Outcomes & Tradeoffs

- **Architectural overhead**: 3 service classes + 1 orchestrator script (vs. monolithic script)
  - Mitigation: Clear service contracts; minimal boilerplate
- **In-memory chunking**: All 500 chunks loaded in memory before embedding
  - Risk: 100MB+ memory for large textbooks (acceptable; typical server has GB+ RAM)
  - Mitigation: Implement streaming for very large textbooks (future enhancement)
- **Sequential chunking**: Markdown parsing is not parallelized
  - Acceptable: parsing is fast (<1s for 100 files); bottleneck is embedding API calls
  - Mitigation: Parallelize markdown parsing if needed (low priority)
- **No async/await**: Synchronous API calls to OpenAI and Qdrant
  - Acceptable: Batch size is small (32 chunks); latency dominated by API, not I/O overhead
  - Mitigation: Convert to async if throughput becomes critical

### Performance Impact

- **Chunking**: < 1s for 100 chapters
- **Embedding**: 3-5 minutes for 500 chunks (16 batches × ~200ms each + 1-2 min rate limit backoff)
- **Insertion**: < 1 minute for 500 chunks
- **Total**: ~5-7 minutes for 100-chapter textbook (acceptable for administrative task)

## Alternatives Considered

### Alternative 1: Monolithic Script

**Components**:
```python
# Single script: process_all.py
def main():
    chunks = parse_all_chapters("frontend/docs/chapters/")
    embeddings = []
    for chunk in chunks:
        vectors = openai.Embedding.create(input=chunk.content)
        embeddings.append(vectors)
    qdrant.insert(embeddings)
```

**Pros**:
- Simplicity (single file, straightforward logic)
- No service abstraction overhead

**Cons**:
- Monolithic: all concerns (parsing, API calls, storage) tangled together
- Hard to test: unit tests require mocking both OpenAI and Qdrant
- No error handling: single failure stops entire pipeline
- No batch processing: calls OpenAI API once per chunk (exceeds rate limits, 5x slower)
- No observability: prints to stdout; hard to parse logs
- Not reusable: can't extract parsing logic for other features

**Why not chosen**: Violates constitution requirement for modular, testable architecture. Rate limit handling crucial for production readiness.

### Alternative 2: Streaming Pipeline with Async/Await

**Components**:
```python
# Async services with streaming
async def main():
    async for chunk in stream_chapters("frontend/docs/chapters/"):
        async for embedding in embed_service.embed(chunk):
            await qdrant_service.insert(embedding)
```

**Pros**:
- Memory efficient (streaming chunks, not all in memory)
- Better throughput with async I/O

**Cons**:
- More complex implementation (async/await, error handling in async context)
- Overkill for current scale (500 chunks = ~100MB memory, acceptable)
- Harder to debug (async errors harder to trace)
- No proportional performance gain: bottleneck is OpenAI API latency, not I/O concurrency

**Why not chosen**: Complexity outweighs benefit for current scale. Defer to future optimization if needed.

### Alternative 3: Kubernetes Jobs / Distributed Workers

**Components**:
- Chunking job → embedding job → insertion job (independent Kubernetes pods)
- Message queue (RabbitMQ/Kafka) to pass data between jobs
- Horizontal scaling (multiple embedding workers in parallel)

**Pros**:
- Horizontal scalability (10 embedding workers → 10x throughput)
- Job isolation (failure of one job doesn't affect others)
- Cloud-native (fits Kubernetes deployment model)

**Cons**:
- Massive over-engineering for initial scale (100-chapter textbook)
- Operational burden: Kubernetes setup, message queue, container orchestration
- Cost: multiple services vs. single script execution
- Complexity: debugging distributed systems is harder

**Why not chosen**: Over-engineered for MVP. Textbook-scale data doesn't justify distributed architecture. Revisit if volume grows to millions of documents.

## Acceptance Criteria

- ✅ Three services independently testable with unit tests (no external API calls)
- ✅ Pipeline handles 100 chapters without manual intervention
- ✅ Graceful degradation: failed batches logged, pipeline continues
- ✅ Rate limit compliance: batch processing respects OpenAI 3500 req/min limit
- ✅ Exponential backoff: 3 retry attempts with 1s, 2s, 4s wait
- ✅ JSON structured logs: timestamp, operation, status, error details
- ✅ Insertion idempotency: re-running pipeline doesn't create duplicates

## Implementation Notes

### Service Contracts (Pydantic Models)

```python
# Input/output contracts published in contracts/

class ChapterChunk(BaseModel):
    chapter_id: str
    chapter_title: str
    section_number: int
    section_title: str
    content: str

class TextEmbedding(BaseModel):
    chunk_id: str
    vector: List[float]  # 1536 dimensions
    metadata: Dict[str, Any]

class InsertionResult(BaseModel):
    total: int
    inserted: int
    failed: int
    errors: List[Dict[str, Any]]
```

### Dependency Injection

Services accept dependencies via constructor or environment variables:

```python
class EmbeddingService:
    def __init__(self, api_key: str, model: str = "text-embedding-3-small"):
        self.client = OpenAI(api_key=api_key)
        self.model = model

# Usage
service = EmbeddingService(api_key=os.getenv("OPENAI_API_KEY"))
```

### Error Handling Pattern

```python
def embed_chunks(chunks: List[ChapterChunk]) -> List[TextEmbedding]:
    results = []
    errors = []

    for batch in chunk_batches(chunks, size=32):
        try:
            embeddings = embed_batch(batch)
            results.extend(embeddings)
        except RateLimitError as e:
            # Retry with exponential backoff
            for attempt in range(3):
                try:
                    embeddings = embed_batch(batch)
                    results.extend(embeddings)
                    break
                except RateLimitError:
                    wait_time = 2 ** (attempt + 1)
                    log_warning(f"Rate limit, retrying in {wait_time}s")
                    time.sleep(wait_time)
        except Exception as e:
            # Log and skip
            log_error(f"Batch failed: {e}")
            errors.append({"batch": batch_id, "error": str(e)})

    return results  # Partial results acceptable
```

## Risks & Mitigations

| Risk | Severity | Mitigation |
|------|----------|-----------|
| OpenAI API unavailable | High | Retry with backoff (3 attempts); inform admin to retry later if all fail |
| Qdrant Free Tier quota exceeded | Medium | Monitor storage monthly; plan paid tier migration if needed |
| Memory exhaustion (500+ chunks) | Low | ~100MB typical (acceptable); implement streaming if > 1M chunks |
| Race condition: concurrent re-indexing | Low | Upsert semantics ensure idempotency; no concurrency issues |

## Revision History

- **2025-12-05**: Initial proposal (Proposed status)
