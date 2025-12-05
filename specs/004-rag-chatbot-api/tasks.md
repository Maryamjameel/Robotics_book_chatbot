# Implementation Tasks: RAG-Powered Chatbot API

**Feature**: 004-rag-chatbot-api | **Branch**: `004-rag-chatbot-api` | **Date**: 2025-12-05
**Specification**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

---

## Executive Summary

**Total Tasks**: 48 across 5 phases
**MVP Scope**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1)
**Parallel Opportunities**: 15 tasks marked [P] can run concurrently
**Critical Path**: Setup → Foundational → US1 → US2 → US3 (sequential dependencies between phases)
**Estimated Effort**: ~15-20 development days (MVP), additional 10 days for P2/P3 features

### User Story Dependencies

```
Phase 3 (US1): Ask Questions & Get Answers ─┐
                 ↓                           │
Phase 4 (US2): Retrieve Relevant Content ───┼─→ Phase 5 (US3): Complete RAG Pipeline
                                              │
                                              ↓
                         (US1 and US2 implement independently; US3 requires both)
```

**Independent Test Criteria**:
- **US1**: POST /api/v1/chat/ask with question → 200 response with answer + sources (async endpoint test)
- **US2**: Vector search function with query → top-5 results with relevance scores (service unit test)
- **US3**: RAG pipeline orchestration → embedding → search → generate → response (integration test)

---

## Phase 1: Setup & Infrastructure

*Complete before starting any user story tasks*

- [ ] T001 Add `google-generative-ai` dependency to `backend/pyproject.toml` with version constraint
- [ ] T002 Create `backend/.env.example` with GEMINI_API_KEY, GEMINI_MODEL placeholders
- [ ] T003 Create `backend/src/api/` directory structure (v1/routes/ subdirectories)
- [ ] T004 Create `backend/src/api/__init__.py` file (empty module marker)
- [ ] T005 Create `backend/src/api/v1/__init__.py` file (empty module marker)
- [ ] T006 Create `backend/src/api/v1/routes/__init__.py` file (empty module marker)
- [ ] T007 [P] Create `backend/tests/unit/__init__.py` file (empty module marker)
- [ ] T008 [P] Create `backend/tests/integration/__init__.py` file (empty module marker)
- [ ] T009 Run `poetry install` to verify dependencies resolve correctly
- [ ] T010 [P] Create `backend/README.md` section documenting RAG endpoint (update existing if exists)

**Acceptance Criteria**:
- All directories exist
- All imports work without errors
- Dependencies resolve cleanly

---

## Phase 2: Foundational Services & Models

*Required for all user stories; must complete before US1/US2/US3*

### Models & Data Validation

- [ ] T011 Create `backend/src/models/chat.py` with ChatRequest Pydantic model (question field: 1-2000 chars, optional filters)
- [ ] T012 [P] Add Source Pydantic model to `backend/src/models/chat.py` (chapter_id, section_number, title, excerpt, relevance_score)
- [ ] T013 [P] Add ChatResponse Pydantic model to `backend/src/models/chat.py` (answer, sources list, metadata dict)
- [ ] T014 [P] Add RAGMetadata Pydantic model to `backend/src/models/chat.py` (timing fields, confidence, relevance)
- [ ] T015 Update `backend/src/models/__init__.py` to export all chat models (ChatRequest, Source, ChatResponse, RAGMetadata)

**Acceptance Criteria**:
- All models validate correctly with Pydantic
- Type hints are complete
- Field descriptions present for OpenAPI generation

### Configuration & Logging

- [ ] T016 Update `backend/src/config.py` to load GEMINI_API_KEY and GEMINI_MODEL from environment
- [ ] T017 [P] Add GEMINI_API_KEY validation in config (raise error if not found in production)
- [ ] T018 [P] Update `backend/src/logger.py` (or create) to support RAG query logging format (question, chunks, confidence, latency)

**Acceptance Criteria**:
- Config loads all required environment variables
- Logger outputs JSON format with required fields
- Missing GEMINI_API_KEY raises clear error in non-dev environments

### FastAPI App Setup

- [ ] T019 Update `backend/src/main.py` FastAPI app initialization to include CORS middleware (allow frontend origins)
- [ ] T020 [P] Add error handlers to `backend/src/main.py` (400, 429, 503 status codes with user-friendly messages)
- [ ] T021 [P] Add `/health` endpoint to `backend/src/main.py` (returns {"status": "ok"})

**Acceptance Criteria**:
- CORS headers present in API responses
- Error handlers map exceptions to correct HTTP status codes
- Health endpoint returns 200 with status

---

## Phase 3: User Story 1 - Ask Natural Language Questions (P1)

*Core MVP feature: Questions in → Answers out with citations*

### GeminiService Implementation

- [ ] T022 [US1] Create `backend/src/services/gemini_service.py` file with GeminiService class
- [ ] T023 [US1] Implement GeminiService `__init__` method with API key, model configuration
- [ ] T024 [US1] Implement async `generate_answer(context: str, question: str) -> str` method in GeminiService
- [ ] T025 [US1] Add system prompt in GeminiService that enforces citation format: "Source: Chapter X, Section Y - Title"
- [ ] T026 [US1] Add error handling in GeminiService for API errors, rate limits, timeouts (raise descriptive exceptions)
- [ ] T027 [US1] [P] Add JSON logging to GeminiService for all API calls (question, answer, tokens, latency)

**Acceptance Criteria**:
- GeminiService.generate_answer returns string answer with embedded citations
- Citations match format: "Source: Chapter {id}, Section {num} - {title}"
- Errors logged with context, not swallowed silently

### RAGService & Citation Extraction

- [ ] T028 [US1] Create `backend/src/services/rag_service.py` file with RAGService class
- [ ] T029 [US1] Implement `_extract_sources` method in RAGService to convert search results to Source objects
- [ ] T030 [US1] Implement `_format_context` method in RAGService to create LLM input from retrieved chunks
- [ ] T031 [US1] Implement `_calculate_confidence` method in RAGService (relevance score → confidence score mapping)
- [ ] T032 [US1] Implement `_validate_citations` method in RAGService to verify answer citations are grounded in retrieved chunks
- [ ] T033 [US1] [P] Add logging to RAGService for RAG query tracking (question, chunks, confidence, latency)

**Acceptance Criteria**:
- Confidence scores range 0-1 based on relevance
- Citation validation flags uncertain answers
- All RAG steps are logged for observability

### Chat Endpoint Implementation

- [ ] T034 [US1] Create `backend/src/api/v1/routes/chat.py` file with FastAPI router
- [ ] T035 [US1] Implement POST `/api/v1/chat/ask` endpoint in chat.py with ChatRequest/ChatResponse types
- [ ] T036 [US1] Add request validation to endpoint (non-empty question, ≤2000 chars)
- [ ] T037 [US1] Implement error handling in endpoint: ValueError → 400, TimeoutError → 503, generic → 503
- [ ] T038 [US1] [P] Add request_id generation to endpoint for tracing across logs
- [ ] T039 [US1] Include endpoint in `backend/src/main.py` via `app.include_router(router)`

**Acceptance Criteria**:
- POST endpoint returns 200 with ChatResponse (answer + sources + metadata)
- Invalid input returns 400 with clear error message
- Service unavailability returns 503

### Unit Tests for US1

- [ ] T040 [US1] Create `backend/tests/unit/test_gemini_service.py` with mock google-generative-ai
- [ ] T041 [US1] [P] Write test: GeminiService.generate_answer returns answer with citations
- [ ] T042 [US1] [P] Write test: GeminiService handles API errors gracefully
- [ ] T043 [US1] Create `backend/tests/unit/test_rag_service.py`
- [ ] T044 [US1] [P] Write test: RAGService._calculate_confidence returns correct mapping (0.7→0.7, 0.85→0.9)
- [ ] T045 [US1] [P] Write test: RAGService._validate_citations flags mismatched citations

**Acceptance Criteria**:
- All GeminiService tests pass with mocked API
- All RAGService tests pass with fixture data
- Coverage ≥80% for services

### Integration Test for US1

- [ ] T046 [US1] Create `backend/tests/integration/test_chat_endpoint_integration.py` with async test client
- [ ] T047 [US1] Write integration test: POST /api/v1/chat/ask with valid question returns 200
- [ ] T048 [US1] [P] Write integration test: POST /api/v1/chat/ask with empty question returns 400

**Acceptance Criteria**:
- Endpoint tests pass with mocked Qdrant/Gemini
- Response structure matches ChatResponse schema
- p95 latency < 3 seconds (measured in test)

**MVP Completion**: Phase 3 delivers P1 user story (ask questions, get answers with citations). Ready for end-to-end testing.

---

## Phase 4: User Story 2 - Retrieve Relevant Content (P2)

*Vector search infrastructure: Question → Top-5 similar sections*

### Vector Search Service Enhancement

- [ ] T049 [US2] Create `backend/src/services/qdrant_service.py` file (new search method for RAG, or enhance existing)
- [ ] T050 [US2] Implement async `search_chunks(question_embedding: List[float], top_k: int, filters: dict) -> List[Dict]` method
- [ ] T051 [US2] Add relevance filtering logic (skip results with cosine similarity < 0.7)
- [ ] T052 [US2] [P] Add timeout handling (1.5s max for search, return empty if exceeded)
- [ ] T053 [US2] Add logging to search method (question, top_k, filters, results count, latency)

**Acceptance Criteria**:
- Search returns top-5 results sorted by relevance (descending)
- Results include chapter_id, section_number, section_title, excerpt, relevance_score
- Relevance threshold (0.7) enforced

### Question Embedding for RAG

- [ ] T054 [US2] Create `backend/src/services/embedding_service.py` (enhance or create) with `embed_question` method
- [ ] T055 [US2] [P] Implement async `embed_question(question: str) -> List[float]` using existing OpenAI embedding model
- [ ] T056 [US2] [P] Add caching for frequently asked questions (optional: Redis or in-memory)
- [ ] T057 [US2] Add error handling for embedding API failures (retry with backoff)

**Acceptance Criteria**:
- Question embeddings have 1536 dimensions (match Qdrant collection)
- Embeddings are normalized (magnitude ~1.0)
- Errors are logged and propagated

### Metadata Filtering

- [ ] T058 [US2] Implement optional metadata filtering in RAGService (chapter_id, section_number filters)
- [ ] T059 [US2] [P] Add validation for filter values before passing to Qdrant

**Acceptance Criteria**:
- Filters are optional (None → no filtering)
- Invalid filters raise 400 error

### Unit Tests for US2

- [ ] T060 [US2] Create `backend/tests/unit/test_vector_search.py`
- [ ] T061 [US2] [P] Write test: search_chunks returns top-5 results with relevance scores
- [ ] T062 [US2] [P] Write test: search_chunks filters results by relevance threshold (< 0.7 excluded)
- [ ] T063 [US2] [P] Write test: embed_question returns 1536-dim vector

**Acceptance Criteria**:
- Search tests pass with mocked Qdrant
- Embedding tests pass with mocked OpenAI API
- Coverage ≥80%

### Integration Test for US2

- [ ] T064 [US2] Create `backend/tests/integration/test_vector_search_integration.py`
- [ ] T065 [US2] [P] Write integration test: Full search pipeline (question → embed → search → results)

**Acceptance Criteria**:
- Integration test passes with mocked services
- Latency measured and < 300ms (per SLA budget)

---

## Phase 5: User Story 3 - Complete RAG Pipeline (P3)

*Full orchestration: Question in → Answer out with proper error handling*

### RAG Pipeline Orchestration

- [ ] T066 [US3] Implement async `answer_question(question: str, filters: dict) -> ChatResponse` method in RAGService
- [ ] T067 [US3] [P] Orchestrate steps: embed question → search chunks → generate answer → extract citations → calculate metadata
- [ ] T068 [US3] [P] Add timeout strategy: 1.5s for search, 3s for generation, 1s buffer (total 5.5s max)
- [ ] T069 [US3] Implement fallback when no relevant chunks found (return answer: "No relevant content found", confidence: 0)
- [ ] T070 [US3] Implement service failure handling: Qdrant → 503, Gemini → 503 (return chunks for user review)

**Acceptance Criteria**:
- Pipeline completes end-to-end in < 3 seconds (p95)
- All errors are caught and mapped to HTTP status codes
- Fallback responses are user-friendly

### Rate Limiting for Gemini

- [ ] T071 [US3] Implement asyncio.Semaphore(5) for concurrent Gemini calls in RAGService initialization
- [ ] T072 [US3] [P] Apply semaphore in `generate_answer` to enforce 5 req/sec limit
- [ ] T073 [US3] Return 429 (Too Many Requests) when semaphore capacity exceeded, with Retry-After header

**Acceptance Criteria**:
- No more than 5 concurrent Gemini calls
- 429 responses include Retry-After header

### Logging & Observability

- [ ] T074 [US3] Add structured JSON logging to RAG pipeline (question, chunks retrieved, confidence, latency breakdown)
- [ ] T075 [US3] [P] Log per-step latency (embedding_time, search_time, generation_time, total_time)
- [ ] T076 [US3] [P] Log request_id for full request tracing across services

**Acceptance Criteria**:
- All logs are JSON format
- Logs include required fields: timestamp, service, level, request_id, operation
- Latency data is separate from other logs for metrics collection

### Confidence Scoring Refinement

- [ ] T077 [US3] Refine confidence calculation in RAGService to consider:
  - Average chunk relevance score
  - LLM response uncertainty signals (e.g., "I'm not certain...")
  - Citation validation (grounded in chunks)
- [ ] T078 [US3] [P] Populate metadata.confidence_score in ChatResponse

**Acceptance Criteria**:
- Confidence ranges 0-1
- Low relevance chunks → lower confidence
- Uncertain LLM signals → flagged in response

### Performance Benchmarking

- [ ] T079 [US3] Implement latency measurement in RAGService (per-step and total)
- [ ] T080 [US3] [P] Add load test script (or test case) to validate p95 < 3 seconds with 50 concurrent users
- [ ] T081 [US3] [P] Document latency breakdown: embedding (50ms), search (150ms), generation (1200ms), serialization (100ms)

**Acceptance Criteria**:
- p95 latency < 3000ms
- p99 latency < 3500ms
- No errors under 50 concurrent load

### Unit Tests for US3

- [ ] T082 [US3] Create `backend/tests/unit/test_rag_orchestration.py`
- [ ] T083 [US3] [P] Write test: answer_question returns ChatResponse with all fields populated
- [ ] T084 [US3] [P] Write test: Timeout handling (search timeout → 503)
- [ ] T085 [US3] [P] Write test: Rate limiting (semaphore blocks after 5 concurrent calls)
- [ ] T086 [US3] [P] Write test: No results fallback (answer: "No relevant content found")

**Acceptance Criteria**:
- All orchestration tests pass
- Timeout tests validate max latency
- Rate limiting tests verify semaphore behavior
- Coverage ≥80%

### Integration Test for US3

- [ ] T087 [US3] Create `backend/tests/integration/test_rag_pipeline_integration.py`
- [ ] T088 [US3] [P] Write integration test: Full RAG pipeline with mocked Qdrant + Gemini (happy path)
- [ ] T089 [US3] [P] Write integration test: Service failure scenarios (Qdrant down, Gemini rate limited)

**Acceptance Criteria**:
- Happy path integration test passes
- Error scenarios return correct HTTP status codes
- Latency measured and < 3 seconds

---

## Phase 6: Polish & Cross-Cutting Concerns

*Production readiness, documentation, deployment*

### Health Check & Readiness

- [ ] T090 Create `/health/rag` endpoint to verify Qdrant + Gemini connectivity
- [ ] T091 [P] Implement health check logic (test vector search, test Gemini call with timeout)
- [ ] T092 [P] Return 200 if both services healthy, 503 if either unavailable

**Acceptance Criteria**:
- Health endpoint returns JSON with status of each service
- Used for Kubernetes readiness probes

### Documentation

- [ ] T093 Update `backend/README.md` with RAG endpoint documentation (endpoint, examples, error codes)
- [ ] T094 [P] Document environment variables (GEMINI_API_KEY, GEMINI_MODEL)
- [ ] T095 [P] Add troubleshooting section (rate limit handling, service failures, timeouts)

**Acceptance Criteria**:
- README covers all required configuration
- Examples are copy-paste ready
- Troubleshooting section helps developers debug

### Code Quality & Coverage

- [ ] T096 Run `pytest --cov=backend/src` to measure test coverage
- [ ] T097 [P] Ensure coverage ≥80% for src/services and src/api/v1/routes
- [ ] T098 [P] Run mypy type checking on all code
- [ ] T099 [P] Run black formatter to validate code style

**Acceptance Criteria**:
- Coverage report shows ≥80% for critical modules
- No mypy errors
- Code passes black formatting

### OpenAPI Documentation

- [ ] T100 Verify OpenAPI schema auto-generation at `/api/v1/docs`
- [ ] T101 [P] Validate OpenAPI schema matches contracts/chat-api.yaml

**Acceptance Criteria**:
- /api/v1/docs accessible and shows POST /api/v1/chat/ask
- Schema shows all required fields and status codes

### Deployment Checklist

- [ ] T102 Create deployment checklist (environment variables, dependencies, tests, health check)
- [ ] T103 [P] Test containerization (if deploying to Docker)
- [ ] T104 [P] Configure CI/CD pipeline to run tests on PR

**Acceptance Criteria**:
- All checklist items documented
- CI/CD runs unit + integration tests

---

## Implementation Strategy & Parallel Execution

### Recommended Execution Order

**Serial Path (Critical Path)**:
1. Phase 1: Setup (T001-T010) - 1 day
2. Phase 2: Foundational (T011-T021) - 2-3 days
3. Phase 3: US1 (T022-T048) - 4-5 days ← **MVP Deployment Point**
4. Phase 4: US2 (T049-T065) - 3-4 days
5. Phase 5: US3 (T066-T089) - 4-5 days
6. Phase 6: Polish (T090-T104) - 2-3 days

**Parallel Opportunities** (marked [P]):
- Phase 1: T007, T008, T010 can run in parallel
- Phase 2: T012-T014, T017-T018, T020-T021 can run concurrently
- Phase 3: T027, T033 logging tasks can run parallel with implementation
- Phase 4: T052, T053, T056, T057 can run parallel
- Phase 5: T071, T075-T076, T081, T083-T089 can run concurrently

### MVP Scope

**Minimum Viable Product = Phase 1 + Phase 2 + Phase 3 (US1)**

Delivers: Working `/api/v1/chat/ask` endpoint that accepts questions and returns answers with citations.

**Not in MVP**:
- Vector search optimization (Phase 4 / US2)
- Complete error handling & rate limiting (Phase 5 / US3)
- Health checks, monitoring, deployment (Phase 6)

### Incremental Delivery Timeline

| Milestone | Tasks | Duration | Deployed |
|-----------|-------|----------|----------|
| **MVP Release** | Phase 1-3 (T001-T048) | 7-10 days | Day 10 |
| **US2 Completion** | Phase 4 (T049-T065) | 3-4 days | Day 14 |
| **Full Feature** | Phase 5 (T066-T089) | 4-5 days | Day 19 |
| **Production Ready** | Phase 6 (T090-T104) | 2-3 days | Day 22 |

---

## Task Dependencies & Blocking Relationships

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ├─→ T011-T015 (Models) ←─┬─── Phase 3 (US1)
    ├─→ T016-T018 (Config)   │       ├─→ T022-T048 (GeminiService, RAGService, Endpoint, Tests)
    ├─→ T019-T021 (FastAPI)  │
                              │
                              ├─── Phase 4 (US2)
                              │       ├─→ T049-T065 (QdrantService, Embedding, Tests)
                              │
                              └─── Phase 5 (US3)
                                    ├─→ T066-T089 (Pipeline Orchestration, Rate Limiting, Tests)

Phase 6 (Polish) ← Depends on Phase 5 completion
    ├─→ T090-T104 (Health, Docs, Coverage, Deployment)
```

**Blocking Tasks** (must complete before dependent task):
- T011-T015 (Models) must complete before T022-T049 (Services)
- T016-T018 (Config) must complete before T023, T024 (Gemini setup)
- T019-T021 (FastAPI app) must complete before T034-T039 (Endpoint)
- T049-T065 (US2) should be complete before T066 (US3 orchestration)

---

## Success Criteria & Validation

### Per User Story

**US1 Completion Criteria**:
- ✅ POST /api/v1/chat/ask endpoint responds in < 3 seconds
- ✅ Response includes answer text grounded in sources
- ✅ Every response includes proper source citations
- ✅ Request validation rejects invalid questions
- ✅ Errors return proper HTTP status codes (400, 503)
- ✅ All unit tests pass, ≥80% coverage

**US2 Completion Criteria**:
- ✅ Vector search returns top-5 results with relevance scores
- ✅ Relevance filtering (0.7 threshold) enforced
- ✅ Metadata filtering (chapter_id) works
- ✅ Search latency < 300ms
- ✅ Unit and integration tests pass

**US3 Completion Criteria**:
- ✅ Full RAG pipeline executes in < 3 seconds (p95)
- ✅ Confidence scores calculated and included
- ✅ Rate limiting enforced (5 req/sec max)
- ✅ All error scenarios handled gracefully
- ✅ All integration tests pass with proper error handling

### Overall Feature Validation

- ✅ Constitution compliance: All 7 principles satisfied
- ✅ Test coverage: ≥80% for critical services/endpoints
- ✅ Documentation: README, OpenAPI, troubleshooting guides complete
- ✅ Performance: p95 latency < 3 seconds under 50 concurrent load
- ✅ Logging: All RAG queries logged in JSON format with latency
- ✅ Availability: 99.9% uptime SLA (< 0.1% 5xx errors)
- ✅ Security: Input validation, rate limiting, error message sanitization

---

## Notes for Implementation Teams

1. **Start with Phase 1 & 2**: Infrastructure is prerequisite for all work
2. **US1 is the MVP**: Focus Phase 3 effort here; can deploy after completion
3. **Parallel work**: Once Phase 2 is done, US1 and US2 can be developed concurrently by different team members
4. **Testing is mandatory**: Don't skip tests; they validate functionality and prevent regressions
5. **Logging matters**: Production support depends on proper RAG query logging
6. **Performance budgeting**: Track latency per step; optimize if exceeding 3-second SLA
7. **Error handling first**: Better to fail fast with clear messages than hang or return wrong data
8. **Document as you go**: Keep README and OpenAPI spec in sync with code changes

---

## Appendix: Test Template Examples

### Unit Test Template (test_rag_service.py)

```python
import pytest
from services.rag_service import RAGService
from models.chat import ChatResponse

@pytest.mark.asyncio
async def test_answer_question_returns_response():
    """Test RAGService.answer_question returns ChatResponse with all fields"""
    rag_service = RAGService(mock_gemini_service)

    with patch('services.qdrant_service.search_chunks') as mock_search:
        mock_search.return_value = [{
            'chapter_id': 'ch03',
            'section_number': 1,
            'content': 'Forward kinematics...',
            'score': 0.89
        }]

        response = await rag_service.answer_question("What is forward kinematics?")

        assert isinstance(response, ChatResponse)
        assert response.answer is not None
        assert len(response.sources) > 0
        assert response.metadata['total_time_ms'] < 3000
```

### Integration Test Template (test_chat_endpoint_integration.py)

```python
import pytest
from fastapi.testclient import TestClient

@pytest.mark.asyncio
async def test_chat_endpoint_valid_question():
    """Test POST /api/v1/chat/ask with valid question"""
    client = TestClient(app)

    response = client.post(
        "/api/v1/chat/ask",
        json={"question": "What is forward kinematics?"}
    )

    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert "metadata" in data
    assert data['metadata']['total_time_ms'] < 3000
```

---

**Next**: Review tasks with team, assign ownership, begin Phase 1 implementation.
