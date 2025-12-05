# Research: Vector Embeddings with Qdrant for RAG

**Phase 0 Output**: Research findings for implementation planning
**Date**: 2025-12-05

## Research Topics & Decisions

### 1. Embedding Model Selection

**Decision**: OpenAI text-embedding-3-small (1536 dimensions)

**Rationale**:
- Officially recommended for RAG by OpenAI and Qdrant docs
- Lower cost than large model (~$0.02 per 1M tokens)
- 1536-dim vectors provide sufficient semantic richness for robotics content
- Direct integration with OpenAI API (already used for chat responses)
- Qdrant natively supports arbitrary dimensions (1536 fits requirement)

**Alternatives Considered**:
- `text-embedding-3-large`: More dimensions (3072), higher cost (~$0.08/1M tokens) - overkill for educational content
- Open-source models (sentence-transformers): Requires self-hosting, more operational burden
- Cohere Embed: Good quality but adds dependency on second vendor, more expensive for our volume

**Implementation Note**: Constitution permits specifying embedding model at deployment time. Default to text-embedding-3-small for cost-effectiveness, but design supports substitution.

---

### 2. Text Chunking Strategy

**Decision**: Section-based chunking (split by `##` Markdown headers) with metadata extraction

**Rationale**:
- Markdown chapters naturally have hierarchical structure with `#` title, `##` sections
- Section-level granularity (200-500 tokens per chunk) enables precise citation: "Chapter X, Section Y"
- Reduces hallucination risk: smaller context window per query reduces model confusion
- Improves relevance: section boundaries align with conceptual boundaries in robotics education

**Implementation Details**:
- Read markdown file → extract chapter metadata (title, id from filename)
- Split by `##` headers → each section becomes a chunk with metadata: {chapter_id, section_number, section_title, content}
- Token counting: Use `tiktoken` library to respect max chunk size (512 tokens for safety)
- Chunk overlap: 50-token overlap between sections preserves context at boundaries

**Alternatives Considered**:
- Paragraph-level chunking: Too granular, loses section context for citations
- Fixed-size (e.g., 512 tokens): Doesn't respect content boundaries, may break mid-concept
- Full-chapter chunking: Too large for semantic search, exceeds LLM context windows

---

### 3. Batch Processing & Rate Limiting

**Decision**: Batch requests to OpenAI (32 chunks per batch), exponential backoff retry for rate limits

**Rationale**:
- OpenAI embedding API supports batch requests: costs same as individual but more efficient
- 32-chunk batch: ~15k tokens typical (well below OpenAI's 100k token context limit)
- Rate limit handling: 3500 requests/minute → ~2-3 second batch wait time for 100-chapter textbook
- Exponential backoff: retry with 2^attempt seconds (1s, 2s, 4s, 8s...) up to 3 attempts
- Max retry ensures graceful failure after 15s total wait

**Implementation Pattern**: Try batch request → on rate limit error, wait and retry with exponential backoff → after 3 failed attempts, log failure and skip batch

**Alternatives Considered**:
- Individual requests: Higher latency (one request per chunk)
- Larger batches (128+): Risk exceeding token limits
- No retry: Loses data on transient API failures
- Fixed-delay retry: Slower recovery from rate limits

---

### 4. Qdrant Collection Configuration

**Decision**: Create collection "chapter_embeddings" with:
- Vector size: 1536 (from embedding model)
- Distance metric: COSINE (standard for text similarity)
- Payload schema: chapter_id (string), section_number (int), title (string), content (string)
- Indexes: chapter_id, section_number for filtering queries

**Rationale**:
- COSINE distance: standard for NLP, ranges [0, 2] (perfect match = 0)
- Payload indexing enables filtering: "show only results from Chapter 3"
- Qdrant Free Tier (1GB) supports ~100-200k vectors at 1536 dims (plenty for textbook)
- Payload storage includes full chunk content for RAG context (avoids second lookup)

**Qdrant Setup Pattern**: Connect to Qdrant instance → create collection with VectorParams(size=1536, distance=COSINE) → create payload indexes for chapter_id and section_number

**Alternatives Considered**:
- Multiple collections per chapter: More complex queries, no benefit for textbook scale
- EUCLIDEAN distance: Valid but COSINE more standard for text
- No payload indexing: Slower RAG queries, must retrieve all results and filter in code

---

### 5. Error Handling & Logging Strategy

**Decision**:
- Log all operations (chunking, embedding, insertion) in JSON format with timestamp, operation type, file/chunk id
- Failed chunks logged with error reason, skipped without stopping pipeline
- Final summary report: {total_files, processed, skipped, errors, processing_time}

**Rationale**:
- JSON logging integrates with constitution requirement for structured logging
- Graceful degradation: one file error doesn't block 99 others
- Administrator visibility: clear error log enables debugging and re-processing

**Log Format Example**:
```json
{"timestamp": "2025-12-05T14:32:10Z", "operation": "chunk", "file": "ch01.md", "chunks": 5, "status": "success"}
{"timestamp": "2025-12-05T14:32:12Z", "operation": "embed", "chunks": 32, "status": "success", "latency_ms": 1850}
{"timestamp": "2025-12-05T14:32:15Z", "operation": "insert_qdrant", "chunks": 32, "status": "success"}
```

**Alternatives Considered**:
- Stop on first error: Loses data from subsequent chapters
- Silent failure: No visibility into problems
- Print logging: Hard to parse in production, not structured

---

### 6. Testing Strategy

**Decision**:
- Unit tests: Markdown parser (edge cases), embedding request construction, Qdrant payload formatting
- Integration tests: Mock OpenAI API, test Qdrant collection setup, verify insertion round-trip
- E2E test: Real Qdrant instance (test environment), sample 5-chapter textbook, verify queries work

**Rationale**:
- Follows constitution requirement for 80%+ coverage of critical paths
- Unit tests fast feedback (< 1s)
- Integration tests verify service contracts without external APIs
- E2E confirms real-world readiness

**Alternatives Considered**:
- No testing: Risk of data loss or corruption
- E2E only: Slow feedback, hard to debug failures
- Unit tests only: Miss integration issues (API contracts, Qdrant operations)

---

## Summary of Key Design Decisions

| Decision | Choice | Key Rationale |
|----------|--------|---------------|
| Embedding Model | text-embedding-3-small (1536-dim) | Cost-effective, OpenAI recommended, RAG-proven |
| Chunking | Section-based (by ## headers) | Aligns with content structure, enables precise citations |
| Batch Processing | 32-chunk batches with exponential backoff | Optimal throughput while respecting rate limits |
| Distance Metric | COSINE | Standard for NLP semantic search |
| Error Strategy | Skip failed chunks, continue processing | Graceful degradation, high reliability |
| Logging | JSON structured logs | Observable, debuggable, integrates with monitoring |

**No NEEDS CLARIFICATION items remain**. All technical decisions are justified by best practices and constitution requirements.
