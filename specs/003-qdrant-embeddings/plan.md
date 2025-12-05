# Implementation Plan: Vector Embeddings with Qdrant for RAG

**Branch**: `003-qdrant-embeddings` | **Date**: 2025-12-05 | **Spec**: [specs/003-qdrant-embeddings/spec.md](spec.md)
**Input**: Feature specification from `/specs/003-qdrant-embeddings/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Establish vector embeddings pipeline to enable semantic search and RAG (Retrieval-Augmented Generation) for the robotics textbook. This feature chunks markdown chapter files, generates 768-dimensional text embeddings, and stores them in Qdrant's vector database. The system supports batch processing with error handling and rate limiting, enabling administrators to index large textbooks via a single command. This is foundational for the RAG chatbot to retrieve and cite relevant content sections when answering user questions.

## Technical Context

**Language/Version**: Python 3.11+ (backend scripts)
**Primary Dependencies**:
  - `qdrant-client` (v2.0+) for Qdrant vector database operations
  - `openai` (v1.0+) for embedding generation (text-embedding-3-small)
  - `python-dotenv` for environment variable management
  - `pydantic` v2 for data validation and schemas

**Storage**:
  - Qdrant Cloud (vector database) - Free Tier sufficient (~1GB for ~200-300 textbook pages)
  - Markdown files on filesystem (`frontend/docs/chapters/`)
  - No relational database required for this feature (embeddings stored in Qdrant)

**Testing**: pytest for unit tests, integration tests with test Qdrant instance
**Target Platform**: Linux/macOS server (CLI scripts executed on admin machine or CI/CD pipeline)
**Project Type**: Backend utility scripts (no frontend component in this feature)
**Performance Goals**:
  - Index 100-chapter textbook (10k-50k pages) in < 5 minutes
  - Embedding generation: 99% success rate with automatic retry on rate limits
  - Qdrant insertion: <1s per chunk (after batching)

**Constraints**:
  - OpenAI API rate limits: 3500 requests per minute (handles ~2000 chapters/minute with batch processing)
  - Qdrant Free Tier: 1GB storage (adequate for course textbook)
  - Chunk size: max 512 tokens (to fit in LLM context windows)

**Scale/Scope**:
  - Initial scope: 100-chapter robotics textbook (100k-500k tokens total)
  - Future scale: extensible to 1000s of chapters as Qdrant storage increases

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I: Production-Grade Quality**
- ✅ Comprehensive Error Handling: Scripts handle API rate limits with exponential backoff, connection failures, and malformed files
- ✅ Type Safety: Python type hints required for all functions (Pydantic models for data validation)
- ✅ Testing Before Deployment: Unit tests for chunking logic, embedding validation, Qdrant insertion verification
- ✅ Graceful Degradation: Failed chunks logged and skipped; pipeline continues processing remaining chapters
- ✅ Performance Monitoring: Logs include processing time per chapter, rate limit hits, retry counts

**Principle III: RAG Accuracy & Source Citation (HIGH PRIORITY)**
- ✅ Metadata Preservation: Each chunk carries chapter_id, section_number, title for citation in RAG responses
- ✅ Chunk Quality: Section-level granularity (split by ## headers) ensures contextual relevance for citations
- ✅ Relevance Indexing: COSINE distance metric in Qdrant enables similarity-based retrieval for relevance filtering

**Principle IV: Modular & Testable Architecture**
- ✅ Decoupled Services: Chunking, embedding generation, and insertion are separate scripts with clear input/output contracts
- ✅ Dependency Injection: Qdrant client and OpenAI client are configurable via environment variables
- ✅ Integration Testing: Test Qdrant collection setup, sample chunk insertion, and query retrieval

**Principle VI: Observability & Debugging**
- ✅ Structured Logging: JSON logs with timestamp, operation (chunk/embed/insert), file name, error details if applicable
- ✅ Performance Metrics: Log embedding API latency, retry attempts, chunk processing time
- ✅ Error Tracking: All exceptions logged with context (file, chunk number, API response)

**GATE RESULT**: ✅ PASS - All constitution principles applicable to this feature are met or can be satisfied with outlined design.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── scripts/
│   ├── chunk_chapters.py          # Phase 1.1: Markdown chunking utility
│   ├── generate_embeddings.py     # Phase 1.2: OpenAI embedding generation
│   ├── insert_qdrant.py           # Phase 1.3: Qdrant vector insertion
│   ├── process_all.py             # Phase 1.4: End-to-end pipeline orchestrator
│   └── __init__.py
├── src/
│   ├── models/
│   │   └── chunk.py               # Phase 1: ChapterChunk data model (Pydantic)
│   └── services/
│       ├── embedding_service.py   # Phase 1: Embedding generation with retry logic
│       ├── qdrant_service.py      # Phase 1: Qdrant collection + insertion
│       └── markdown_parser.py     # Phase 1: Markdown reading and chunking
└── tests/
    ├── unit/
    │   ├── test_markdown_parser.py      # Phase 2: Chunking logic tests
    │   ├── test_embedding_service.py    # Phase 2: Embedding generation tests
    │   └── test_qdrant_service.py       # Phase 2: Qdrant operations tests
    └── integration/
        └── test_end_to_end_pipeline.py  # Phase 2: Full pipeline test

frontend/docs/chapters/
└── [existing chapter markdown files - used as input]
```

**Structure Decision**: Backend utility scripts with supporting Python modules for chunking, embedding, and Qdrant operations. Test structure mirrors source code layout with unit and integration test categories. Follows constitution requirement for modular, testable architecture with clear service separation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
