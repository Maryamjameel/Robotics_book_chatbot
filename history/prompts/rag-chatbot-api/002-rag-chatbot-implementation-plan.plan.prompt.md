---
id: 002
title: RAG Chatbot API Implementation Plan
stage: plan
date: 2025-12-05
surface: agent
model: claude-haiku-4-5-20251001
feature: rag-chatbot-api
branch: 004-rag-chatbot-api
user: developer
command: /sp.plan
labels: ["planning", "api", "rag", "architecture", "data-models", "contracts", "production-ready"]
links:
  spec: specs/004-rag-chatbot-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-chatbot-api/plan.md
  - specs/004-rag-chatbot-api/contracts/chat-api.yaml
  - specs/004-rag-chatbot-api/quickstart.md
tests:
  - Specification validation checklist (requirements.md) - PASSED
  - Constitution compliance checks - ALL PASSED
  - Latency SLA validation plan (p95 < 3s)
---

## Prompt

User executed: `/sp.plan`

This command transitioned the RAG chatbot API feature from specification phase to implementation planning. The user's prior work had completed the feature specification with 3 prioritized user stories, 15 functional requirements, 8 measurable success criteria, and comprehensive validation.

The planning phase required:

1. Load technical context and validate against constitution
2. Execute Phase 0 research (resolve all clarifications)
3. Execute Phase 1 design (data models, API contracts, project structure)
4. Generate implementation strategy (Phase 2)
5. Create PHR for the planning phase

## Response snapshot

### Planning Workflow Complete

**Technical Context Filled**:
- Language/Version: Python 3.11+
- Primary Dependencies: FastAPI 0.110+, Pydantic v2, Google Generative AI SDK, Qdrant client
- Storage: Qdrant (vector embeddings, read-only), optional Neon Postgres for logging
- Testing: pytest with asyncio for async endpoint testing
- Target Platform: Linux server (FastAPI application)
- Performance Goals: p95 < 3 seconds, 50+ concurrent requests
- Constraints: Gemini API rate limits (5 req/sec), 2000-char question limit, top-5 search default
- Scale/Scope: ~100-300 textbook pages in Qdrant, single MVP endpoint

**Constitution Check: PASSED ✅**
- Production-Grade Quality: Error handling, type safety, 80%+ test coverage, <3s latency
- RAG Accuracy & Citations: Mandatory citations, confidence scoring, 0.7 relevance threshold, hallucination detection
- Modular Architecture: GeminiService, RAGService with dependency injection, stateless
- Observability: JSON structured logging, RAG query logging, performance metrics
- Spec-Driven Development: Complete spec + plan before implementation

**Project Structure (Option 2: Backend API)**
```
backend/
├── src/
│   ├── models/
│   │   ├── chunk.py (existing)
│   │   └── chat.py (NEW: ChatRequest, ChatResponse, Source)
│   ├── services/
│   │   ├── embedding_service.py (existing)
│   │   ├── qdrant_service.py (existing)
│   │   ├── gemini_service.py (NEW: Gemini LLM wrapper)
│   │   └── rag_service.py (NEW: Pipeline orchestration)
│   ├── api/v1/routes/
│   │   └── chat.py (NEW: POST /api/v1/chat/ask endpoint)
│   ├── main.py (existing or NEW: FastAPI app)
│   └── config.py (existing: enhanced with GEMINI_API_KEY)
├── tests/
│   ├── unit/
│   │   ├── test_gemini_service.py (NEW)
│   │   ├── test_rag_service.py (NEW)
│   │   └── test_chat_endpoint.py (NEW)
│   ├── integration/
│   │   ├── test_rag_pipeline.py (NEW)
│   │   └── test_chat_endpoint_integration.py (NEW)
│   └── conftest.py (updated: Gemini mocks)
├── pyproject.toml (updated: google-generative-ai)
└── .env.example (updated: GEMINI_API_KEY)
```

**Phase 0: Research Findings**
1. Gemini API Integration: Use google-generative-ai SDK for direct integration, async support
2. Prompt Engineering: System prompt enforces citations; examples prevent hallucinations
3. Latency Optimization: 100-200ms vector search + 1-1.5s LLM generation = ~1.5s total (within 3s SLA)
4. Concurrency Control: asyncio.Semaphore(5) for rate limiting (5 req/sec)
5. Error Handling: Graceful degradation (Qdrant down → 503, Gemini rate limit → 429, invalid input → 400)

**Phase 1: Data Models**
- ChatRequest: question (str, 1-2000 chars) + optional filters (dict)
- Source: chapter_id, chapter_title, section_number, section_title, excerpt (200 chars), relevance_score (0-1)
- ChatResponse: answer (str) + sources (List[Source]) + metadata (dict)
- RAGMetadata: question_embedding_time_ms, vector_search_time_ms, llm_generation_time_ms, total_time_ms, confidence_score, avg_relevance_score, num_sources_retrieved

**Phase 1: API Contracts (OpenAPI/YAML)**
- Endpoint: POST /api/v1/chat/ask
- Request: {"question": "...", "filters": {"chapter_id": "ch03"}}
- Response (200): {"answer": "...", "sources": [...], "metadata": {...}}
- Error Responses: 400 (invalid), 429 (rate limit), 503 (service unavailable)
- OpenAPI schema generated via Pydantic models, available at /api/v1/docs

**Phase 2: Implementation Strategy**
1. GeminiService: Async wrapper, `generate_answer(context: str, question: str) -> str`
   - System prompt enforces citations, few-shot examples prevent hallucinations
   - Error handling: Capture API errors, rate limits, timeouts

2. RAGService: Orchestrates RAG pipeline
   - `answer_question(question: str, filters: Optional[dict]) -> ChatResponse`
   - Steps: Embed question → Search Qdrant → Generate answer → Extract citations → Calculate metadata

3. Endpoint Handler: FastAPI route
   - Request validation via Pydantic
   - Error mapping (ValueError → 400, TimeoutError → 503)
   - Response serialization

4. Testing Strategy:
   - Unit tests: GeminiService (prompt construction, error handling), RAGService (pipeline), Endpoint (validation)
   - Integration tests: Full RAG pipeline with mocks, async endpoint testing
   - Mocking: Gemini responses with realistic answers, Qdrant with pre-configured chunks

### Artifacts Generated

**plan.md** (1000+ lines)
- Complete technical context and constitution compliance
- Project structure with all new/modified files documented
- Phase 0 research with 5 key findings (Gemini, prompting, latency, concurrency, errors)
- Phase 1 data models (4 Pydantic models with field descriptions)
- Phase 1 API contracts (OpenAPI spec snippet, request/response examples)
- Phase 2 implementation strategy (service architecture, key details, testing, deployment)

**contracts/chat-api.yaml** (300+ lines)
- Full OpenAPI 3.0 specification
- POST /api/v1/chat/ask endpoint with descriptions
- Request schema (ChatRequest with examples)
- Response schema (ChatResponse with examples)
- Error responses (400, 429, 503 with examples)
- Component schemas (Source, RAGMetadata, ErrorResponse)

**quickstart.md** (400+ lines)
- Step-by-step implementation guide
- Architecture overview diagram
- 6 implementation phases with code examples:
  1. Dependencies (poetry, .env, install)
  2. Data models (Pydantic classes)
  3. Gemini service (async wrapper with generation)
  4. RAG service (orchestration, context formatting, citation extraction)
  5. Chat endpoint (FastAPI route with error handling)
  6. FastAPI app integration (CORS, router inclusion)
- Testing examples (unit tests with mocks, integration tests)
- Deployment checklist (12 items)

## Outcome

- ✅ Impact: Complete RAG chatbot API implementation plan with architecture, data models, API contracts, and quickstart guide. Ready for task breakdown and implementation.
- 🧪 Tests: Constitution compliance verified (ALL CHECKS PASSED). Phase 0 research complete (0 NEEDS CLARIFICATION items). Latency SLA achievable (1.5s for search + generation, 1s buffer for network).
- 📁 Files: 3 major artifacts created (plan.md, contracts/chat-api.yaml, quickstart.md). 12 new backend files documented (models, services, routes, tests). 3 existing files to be enhanced (config.py, main.py, pyproject.toml).
- 🔁 Next prompts: (1) Run `/sp.tasks` to generate implementation task breakdown, (2) Run `/sp.adr` to document architectural decisions (Gemini vs GPT-4, rate limiting strategy, citation format), (3) Execute implementation phase, (4) Run code reviews and integration testing.
- 🧠 Reflection: Planning phase unlocked implementation clarity. Technical decisions are well-founded (Gemini for cost-effectiveness, Semaphore for rate limiting, asyncio for concurrency). Latency SLA is achievable with proper timeouts. Constitution principles ensure production quality from day 1 (error handling, logging, testing, modular architecture).

## Evaluation notes (flywheel)

- Failure modes observed: None. All planning steps completed without blockers.
- Graders run and results (PASS/FAIL): Constitution Check - PASSED. Phase 0 Research - COMPLETE (0 clarifications). Phase 1 Design - COMPLETE (4 models, 1 endpoint schema). Phase 2 Strategy - COMPLETE (service architecture, testing plan, deployment checklist).
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run `/sp.tasks` phase to generate granular implementation tasks. Validate that 3-second SLA is achievable by estimating latency for each service (embedding: 50ms, search: 150ms, generation: 1200ms, serialization: 100ms = 1500ms total).
