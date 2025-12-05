# ADR-001: Vector Storage and Embedding Stack for RAG

**Status**: Proposed
**Decided**: 2025-12-05
**Feature**: 003-qdrant-embeddings
**References**: specs/003-qdrant-embeddings/plan.md, specs/003-qdrant-embeddings/research.md

## Context

The robotics textbook platform requires semantic search capability to power the RAG (Retrieval-Augmented Generation) chatbot. Users need to ask natural language questions and receive answers citing relevant chapter sections. This requires:

1. Converting text chapters into vector embeddings for similarity search
2. Storing embeddings in a queryable vector database
3. Retrieving relevant chunks based on semantic similarity

The system must support:
- Cost-effective embedding generation (~100-500k tokens from 100-chapter textbook)
- Efficient storage within free-tier limits (~1GB)
- Fast semantic search (<500ms p95 latency)
- Integration with existing OpenAI-based RAG infrastructure

Multiple technology stacks were evaluated for cost, integration, operational overhead, and scaling characteristics.

## Decision

Adopt **OpenAI Embedding Stack with Qdrant Cloud**:

### Components

1. **Embedding Model**: OpenAI `text-embedding-3-small` (1536 dimensions)
   - Cost: ~$0.02 per 1M tokens (vs $0.08 for large model)
   - Dimensions: 1536 (sufficient for educational content, fits Qdrant Free Tier)
   - Availability: Same OpenAI API account as existing chat infrastructure

2. **Vector Database**: Qdrant Cloud (Free Tier)
   - Managed service (no ops burden)
   - 1GB storage (~100-200k vectors at 1536 dims = 200+ textbook chapters)
   - COSINE distance metric (standard for NLP semantic search)
   - Payload indexing (enables filtering by chapter_id, section_number)

3. **Integration Pattern**:
   - Scripts call OpenAI API in batches (32 chunks per batch)
   - Batch requests to OpenAI (same cost, better throughput)
   - Upsert vectors to Qdrant with full chunk content in payload
   - Support for alternative embedding models via environment variable (future extensibility)

## Consequences

### Positive Outcomes

- **Cost-effective**: text-embedding-3-small is 75% cheaper than large model with sufficient quality for robotics education
- **Tight OpenAI integration**: Reuses existing API keys and infrastructure (FastAPI backend already uses OpenAI ChatKit SDK)
- **Managed infrastructure**: Qdrant Cloud eliminates vector database ops, scaling concerns, backup management
- **Fast implementation**: Both services have mature Python SDKs with excellent documentation
- **Proven for RAG**: OpenAI embeddings + vector search is the de facto standard for RAG systems
- **Scaling path**: Can upgrade to Qdrant paid tier or migrate to alternative if storage exceeds 1GB

### Negative Outcomes & Tradeoffs

- **Vendor lock-in**: Tight coupling to OpenAI API (cost increases if token volume grows)
  - Mitigation: Design supports alternative embedding models via config; data model is vendor-agnostic
- **Rate limiting complexity**: OpenAI API has strict rate limits (3500 requests/min)
  - Mitigation: Batch processing (32 chunks/batch) designed to respect limits with exponential backoff retry
- **Qdrant Free Tier limits**: 1GB storage sufficient for 200 chapters but requires paid tier for larger corpora
  - Mitigation: Long-term planning for scaling; monitoring of storage usage
- **Cold start latency**: Vector generation adds 1-5 minutes to textbook indexing
  - Mitigation: Batch processing + parallelization in future; acceptable for administrative task

### Performance Impact

- Embedding generation: ~200ms per batch (32 chunks) with OpenAI API latency
- 100-chapter textbook: ~500 chunks → 16 batches → ~3-5 minutes total (including rate limit backoff)
- Qdrant insertion: < 100ms per batch
- Semantic search latency: < 500ms p95 (measured against Qdrant Free Tier)

## Alternatives Considered

### Alternative 1: Self-Hosted Open-Source Embeddings (sentence-transformers)

**Components**:
- sentence-transformers (BERT-based, 384-768 dimensions)
- Qdrant self-hosted (Docker container or Kubernetes)
- GPU for inference (cost: $100-500/month for cloud GPU)

**Pros**:
- No per-token cost (one-time compute)
- Full control over embedding model and updates
- Can run offline (no API dependency)

**Cons**:
- Operational burden: self-hosting, GPU scaling, model updates
- GPU costs exceed OpenAI API cost for 100-chapter textbook (~$30-50 vs $2-3 with OpenAI)
- Embedding quality may be lower than OpenAI's proprietary models
- Adds infrastructure complexity (DevOps cost)

**Why not chosen**: Operational overhead outweighs cost savings for initial scope. Revisit if volume exceeds 1M tokens/month.

### Alternative 2: Pinecone Serverless Vector Database

**Components**:
- OpenAI text-embedding-3-small (same as chosen)
- Pinecone Serverless (managed vector database)
- Integration via Pinecone Python SDK

**Pros**:
- Managed service (no ops)
- Generous free tier (12M vectors)
- Excellent query performance

**Cons**:
- Pinecone pricing: $0.07-0.50 per 1M vectors (100k vectors = $7-50/month)
- Qdrant Free Tier is completely free (better for educational use case)
- Proprietary API (less community support than Qdrant)
- Overkill complexity for textbook-scale data (similar to Qdrant but more expensive)

**Why not chosen**: Qdrant provides equivalent capability at lower cost; both are managed services with similar complexity.

### Alternative 3: Hybrid Approach (Multiple Vectors + Elasticsearch)

**Components**:
- OpenAI embeddings for semantic search
- Elasticsearch for full-text search
- Two databases (complexity)

**Pros**:
- Semantic + full-text search enables richer queries
- Elasticsearch mature ecosystem

**Cons**:
- Adds infrastructure complexity (two databases to maintain)
- Double storage cost
- Overkill for educational use case (semantic search sufficient)
- Slower to implement and maintain

**Why not chosen**: Semantic search alone is sufficient for RAG use case; full-text search adds complexity without proportional value.

## Acceptance Criteria

- ✅ Embedding generation completes for 100 chapters in < 5 minutes
- ✅ Semantic search latency < 500ms p95
- ✅ Cost < $50/month for textbook-scale data (OpenAI + Qdrant)
- ✅ Support for alternative embedding models without code changes (config-driven)
- ✅ JSON structured logging captures embedding API latency and rate limit hits
- ✅ 99% of embeddings generated successfully (failed batches logged and skipped)

## Implementation Notes

### Configuration Management

Environment variables drive technology choices:

```bash
EMBEDDING_MODEL=text-embedding-3-small  # Supports substitution
QDRANT_URL=https://xxx.qdrantcloud.io
QDRANT_API_KEY=xxx
```

### Extensibility

Future changes can be accommodated:

1. **Alternative embedding model**: Change `EMBEDDING_MODEL` variable, reimplement `EmbeddingService.embed_batch()` for new API
2. **Alternative vector database**: Implement `QdrantService` interface for Pinecone, Milvus, etc.
3. **Hybrid search**: Add Elasticsearch sidecar without disrupting embedding/vector stack

## Risks & Mitigations

| Risk | Severity | Mitigation |
|------|----------|-----------|
| OpenAI API rate limits cause processing delays | Medium | Batch processing (32 chunks) + exponential backoff handles 3500 req/min limit |
| Qdrant Free Tier storage exceeded | Low | Monitor storage monthly; plan migration to paid tier if needed |
| OpenAI embedding model updates break compatibility | Low | Design stores full chunk content in payload; can regenerate vectors if model changes |
| Cost growth as textbook expands | Medium | Per-token cost scales linearly; revisit self-hosted option if > 1M tokens/month |

## Revision History

- **2025-12-05**: Initial proposal (Proposed status)
