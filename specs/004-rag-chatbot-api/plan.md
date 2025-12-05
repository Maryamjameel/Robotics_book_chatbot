# Implementation Plan: RAG-Powered Chatbot API

**Branch**: `004-rag-chatbot-api` | **Date**: 2025-12-05 | **Spec**: [specs/004-rag-chatbot-api/spec.md](spec.md)
**Input**: Feature specification from `/specs/004-rag-chatbot-api/spec.md`

## Summary

Build a production-ready FastAPI REST API endpoint (`/api/v1/chat/ask`) that answers user questions about robotics textbook content using semantic search against pre-indexed Qdrant vector embeddings and Gemini LLM for answer generation with mandatory source citations. The system must respond in under 3 seconds (p95), handle 50+ concurrent requests, and achieve 100% citation precision with no hallucinated sources.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI 0.110+, Pydantic v2, Google Generative AI SDK, Qdrant client, httpx (async HTTP)
**Storage**: Qdrant (vector embeddings, read-only), Neon Postgres (optional: RAG query logging)
**Testing**: pytest (unit, integration), pytest-asyncio for async endpoint testing
**Target Platform**: Linux server (FastAPI application)
**Project Type**: Backend REST API (no frontend in this feature; consumed by existing Docusaurus frontend)
**Performance Goals**: p95 latency < 3 seconds (vector search + LLM), handle 50+ concurrent requests
**Constraints**: Gemini API rate limits (5 req/sec), question size limit (2000 chars), vector search top-5 default
**Scale/Scope**: Educational platform with ~100-300 textbook pages (pre-indexed in Qdrant), single API endpoint as MVP

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Production-Grade Quality
- ✅ Error handling: Graceful degradation when Gemini/Qdrant unavailable (return clear error messages)
- ✅ Type safety: Python 3.11+ with type hints, Pydantic v2 for all request/response schemas
- ✅ Testing: pytest with 80%+ coverage for RAG pipeline, endpoint validation, error cases
- ✅ Performance: p95 < 3 seconds, 50+ concurrent requests supported

### ✅ RAG Accuracy & Source Citation (HIGH PRIORITY)
- ✅ Mandatory citations: Every response cites chapter/section (e.g., "Source: Chapter 3, Section 3.2")
- ✅ Confidence scoring: Track relevance scores from vector search and include in response
- ✅ Hallucination detection: Verify LLM output grounded in retrieved chunks; flag uncertain answers
- ✅ Relevance threshold: Minimum cosine similarity 0.7 for chunk retrieval
- ✅ Context window: Top-5 chunks with chapter/section titles for LLM context

### ✅ Modular & Testable Architecture
- ✅ Decoupled services: GeminiService, QdrantService, RAGService with dependency injection
- ✅ API-first design: OpenAPI spec via Pydantic models, `/api/v1/chat/ask` endpoint
- ✅ Stateless: No session state in API; all queries stateless and idempotent
- ✅ Async/await: All I/O async (Qdrant search, Gemini calls via httpx)

### ✅ Observability & Debugging
- ✅ Structured logging: JSON format with timestamp, service, log level, request_id, user_id
- ✅ RAG query logging: Log question, retrieved chunks, relevance scores, confidence, latency
- ✅ Performance metrics: Track p50/p95/p99 latency for vector search, LLM generation

### ✅ Spec-Driven Development
- ✅ Specification first: Complete spec.md with user stories, functional requirements, success criteria
- ✅ Architecture planning: This plan.md documenting approach, data models, API contracts

**GATE STATUS**: ✅ **PASS** - All constitution principles aligned. No violations.

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

**Selected Structure**: Option 2 - Backend API (extends existing backend directory)

```text
backend/
├── src/
│   ├── models/
│   │   ├── chunk.py           # Existing: ChapterChunk, TextEmbedding
│   │   ├── chat.py            # NEW: ChatRequest, ChatResponse, Source
│   │   └── __init__.py
│   ├── services/
│   │   ├── embedding_service.py    # Existing: For user question embeddings
│   │   ├── qdrant_service.py       # Existing: For vector search
│   │   ├── gemini_service.py       # NEW: Gemini LLM integration
│   │   ├── rag_service.py          # NEW: RAG orchestration (search + generate)
│   │   ├── logger.py               # Existing or NEW: Structured JSON logging
│   │   └── __init__.py
│   ├── api/
│   │   ├── v1/
│   │   │   ├── routes/
│   │   │   │   ├── chat.py         # NEW: POST /api/v1/chat/ask endpoint
│   │   │   │   └── __init__.py
│   │   │   └── __init__.py
│   │   └── __init__.py
│   ├── config.py              # Existing: Enhanced with Gemini API key
│   ├── main.py                # Existing or NEW: FastAPI app with CORS, error handlers
│   └── __init__.py
├── tests/
│   ├── unit/
│   │   ├── test_gemini_service.py      # NEW: Gemini SDK mocking, prompt construction
│   │   ├── test_rag_service.py         # NEW: RAG pipeline orchestration
│   │   ├── test_chat_endpoint.py       # NEW: Endpoint request/response validation
│   │   └── __init__.py
│   ├── integration/
│   │   ├── test_rag_pipeline.py        # NEW: Full pipeline with mocked Qdrant/Gemini
│   │   ├── test_chat_endpoint_integration.py  # NEW: Real async endpoint testing
│   │   └── __init__.py
│   └── conftest.py            # Updated: Mocks for Gemini, Qdrant services
├── pyproject.toml             # Updated: Add google-generative-ai dependency
├── .env.example               # Updated: Add GEMINI_API_KEY
└── README.md                  # Updated: RAG endpoint documentation
```

**Structure Decision**: Extends existing `backend/` directory following modular architecture. New services (GeminiService, RAGService) are decoupled and injectable. Endpoint routes follow FastAPI conventions. Tests organized by type (unit vs integration) with shared fixtures in conftest.py.

## Phase 0: Research & Clarification

### Resolved Questions

All technical decisions are clear from specification and constitution. No NEEDS CLARIFICATION items.

**Key Research Findings**:

1. **Gemini API Integration**
   - Decision: Use Google Generative AI SDK (python: `google-generative-ai`)
   - Rationale: Direct integration with Gemini, better error handling, native async support via httpx
   - Alternatives: OpenAI GPT-4 (cost higher, already using for embeddings), Anthropic Claude (higher latency, different rate limits)

2. **Prompt Engineering for Citations**
   - Decision: System prompt enforces citation format; few-shot examples demonstrate desired response structure
   - Rationale: Gemini responds well to explicit formatting instructions; examples prevent hallucinations
   - Citation Format: "Source: Chapter {chapter_id}, Section {section_number} - {section_title}" at end of answer

3. **Latency Optimization (3-second SLA)**
   - Vector Search: Qdrant top-5 search ~ 100-200ms (pre-indexed collection)
   - LLM Generation: Gemini typical latency 1-1.5s for ~200-token response
   - Timeout Strategy: 1.5s timeout for vector search (fail if exceeded), 3s timeout for LLM call
   - Total Budget: 2s for search + generation, 1s buffer for serialization/network

4. **Concurrency & Rate Limiting**
   - Gemini Rate Limit: 5 requests/second (API quota)
   - Implementation: Use asyncio.Semaphore(5) to limit concurrent Gemini calls
   - Fallback: Queue requests with 429 response if capacity exceeded

5. **Error Handling Strategy**
   - Qdrant Down: Return 503 with "Vector search temporarily unavailable. Please try again."
   - Gemini Down/Rate Limited: Return 503 with "LLM service temporarily unavailable. Retrieved content:"  + raw chunks for user review
   - Invalid Question: Return 400 with validation error details
   - Retrieve Found No Results: Return 200 with answer "No relevant content found" + confidence=0

## Phase 1: Data Model & API Contracts

### Data Models (NEW: models/chat.py)

```python
from pydantic import BaseModel, Field
from typing import List, Optional

class ChatRequest(BaseModel):
    """User question and optional search filters"""
    question: str = Field(..., min_length=1, max_length=2000, description="User's question about robotics")
    filters: Optional[dict] = Field(None, description="Optional metadata filters (chapter_id, section_number)")

class Source(BaseModel):
    """Citation information from retrieved content"""
    chapter_id: str
    chapter_title: str
    section_number: int
    section_title: str
    excerpt: str = Field(..., description="First 200 chars of section content")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity (0.0-1.0)")

class ChatResponse(BaseModel):
    """Complete answer with sources and metadata"""
    answer: str = Field(..., description="AI-generated answer grounded in sources")
    sources: List[Source] = Field(..., description="Retrieved sources informing the answer")
    metadata: dict = Field(default_factory=dict, description="Response metadata (confidence, processing_time_ms, etc.)")

class RAGMetadata(BaseModel):
    """Internal metadata for RAG processing"""
    question_embedding_time_ms: float
    vector_search_time_ms: float
    llm_generation_time_ms: float
    total_time_ms: float
    confidence_score: float = Field(..., ge=0.0, le=1.0)
    avg_relevance_score: float
    num_sources_retrieved: int
```

### API Contracts

**Endpoint**: `POST /api/v1/chat/ask`

**Request**:
```json
{
  "question": "What is forward kinematics?",
  "filters": {
    "chapter_id": "ch03"
  }
}
```

**Response (200 OK)**:
```json
{
  "answer": "Forward kinematics is the process of calculating the end-effector position and orientation given the joint angles and link lengths of a robot...\n\nSource: Chapter 3, Section 3.1 - Forward Kinematics",
  "sources": [
    {
      "chapter_id": "ch03",
      "chapter_title": "Kinematics",
      "section_number": 1,
      "section_title": "Forward Kinematics",
      "excerpt": "Forward kinematics is the process of calculating the end-effector position and orientation given the joint angles and link lengths of a robot.",
      "relevance_score": 0.89
    },
    {
      "chapter_id": "ch03",
      "chapter_title": "Kinematics",
      "section_number": 2,
      "section_title": "Denavit-Hartenberg Convention",
      "excerpt": "The Denavit-Hartenberg (DH) convention is a systematic way to assign coordinate frames to robot links...",
      "relevance_score": 0.76
    }
  ],
  "metadata": {
    "question_embedding_time_ms": 45,
    "vector_search_time_ms": 120,
    "llm_generation_time_ms": 1200,
    "total_time_ms": 1365,
    "confidence_score": 0.92,
    "avg_relevance_score": 0.825,
    "num_sources_retrieved": 2
  }
}
```

**Error Responses**:

- **400 Bad Request**: Invalid question (empty, too long)
```json
{"detail": "Question must be between 1 and 2000 characters"}
```

- **503 Service Unavailable**: Qdrant or Gemini down
```json
{"detail": "Vector search temporarily unavailable. Please try again."}
```

### OpenAPI Schema

Generated automatically by FastAPI via Pydantic models. Available at `/api/v1/docs`.

## Phase 2: Implementation Strategy

### Service Architecture

**GeminiService** (`services/gemini_service.py`)
- Async wrapper around google-generative-ai SDK
- Methods: `generate_answer(context: str, question: str) -> str`
- Responsibilities: Prompt construction, API calls, error handling, logging

**RAGService** (`services/rag_service.py`)
- Orchestrates embedding → search → generation pipeline
- Methods: `answer_question(question: str, filters: Optional[dict]) -> ChatResponse`
- Responsibilities: Question embedding, vector search, LLM generation, citation extraction, metadata calculation

**Endpoint Handler** (`api/v1/routes/chat.py`)
- FastAPI route for POST /api/v1/chat/ask
- Responsibilities: Request validation, response serialization, error mapping

### Key Implementation Details

1. **Prompt Engineering**
   ```
   System: You are a robotics education assistant. Always cite sources from the provided context.

   Context:
   {chunk_text_with_headers}

   Question: {user_question}

   Requirements:
   - Answer based solely on provided context
   - End answer with "Source: Chapter X, Section Y - {title}"
   - If uncertain, start with "I'm not entirely certain, but..."
   - Do not invent information
   ```

2. **Citation Extraction**
   - Parse LLM response to extract section references
   - Match against retrieved chunks to ensure citations are grounded
   - Flag as "uncertain" if citation doesn't match retrieved content

3. **Confidence Scoring**
   - Calculate average cosine similarity of retrieved chunks
   - If avg similarity < 0.7: confidence 0.5
   - If avg similarity 0.7-0.85: confidence 0.7
   - If avg similarity > 0.85: confidence 0.9
   - Adjust based on LLM confidence indicators in response

4. **Concurrency Control**
   - Use asyncio.Semaphore(5) to limit concurrent Gemini calls
   - Queue excess requests with 429 response + retry-after header
   - Log rate limit events for monitoring

### Testing Strategy

**Unit Tests**:
- GeminiService: Prompt construction, citation extraction, error mapping
- RAGService: Pipeline orchestration, metadata calculation
- Endpoint: Request validation, response serialization

**Integration Tests**:
- Full RAG pipeline with mocked Gemini/Qdrant
- Async endpoint testing via pytest-asyncio
- End-to-end latency validation (< 3s)

**Mocking Strategy**:
- Mock Gemini responses with realistic generated answers + citations
- Mock Qdrant search with pre-configured relevant chunks
- Test edge cases: no results, service timeouts, malformed responses

### Deployment Considerations

1. **Environment Variables**:
   - GEMINI_API_KEY (required)
   - GEMINI_MODEL (default: "gemini-1.5-flash")
   - QDRANT_URL, QDRANT_API_KEY (from embeddings feature)

2. **Health Check**:
   - Add `/health/rag` endpoint that tests Qdrant + Gemini connectivity
   - Return 200 if both available, 503 if either unavailable

3. **Performance Monitoring**:
   - Log all RAG queries with metadata (latency, confidence, relevance)
   - Track error rates and types
   - Alert if p95 latency exceeds 3 seconds

## Complexity Tracking

> No constitution violations. All principles satisfied.
