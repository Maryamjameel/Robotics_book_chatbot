---
description: "Task list for vector embeddings with Qdrant for RAG"
---

# Tasks: Vector Embeddings with Qdrant for RAG

**Input**: Design documents from `/specs/003-qdrant-embeddings/`
**Prerequisites**: plan.md (architecture), spec.md (user stories), research.md (decisions), data-model.md (entities), contracts/ (service APIs), quickstart.md (test scenarios)

**Organization**: Tasks grouped by user story priority (P1, P2, P3) to enable independent implementation and testing.

**Format**: `[ID] [P?] [Story] Description with file path`
- **[P]**: Can run in parallel (different files/services, no dependencies)
- **[Story]**: User story label (US1, US2, US3)
- **ID**: Sequential task number (T001, T002, ...)
- **File paths**: Exact locations for code creation

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize backend project structure, dependencies, configuration

**Checkpoint**: Backend project skeleton ready; dependencies installed; environment configured

- [ ] T001 Create backend project structure per plan.md in `backend/` directory
- [ ] T002 Initialize Python 3.11+ project with Poetry dependencies (qdrant-client, openai, pydantic, python-dotenv, pytest)
- [ ] T003 [P] Create `.env.example` template with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY placeholders in `backend/`
- [ ] T004 [P] Create `backend/__init__.py` and `backend/src/__init__.py` package markers
- [ ] T005 [P] Create `backend/src/models/__init__.py` for model exports
- [ ] T006 [P] Create `backend/src/services/__init__.py` for service exports
- [ ] T007 [P] Create `backend/scripts/__init__.py` for script utilities
- [ ] T008 [P] Configure Python path in `backend/pyproject.toml` with Poetry package structure
- [ ] T009 Create `backend/pytest.ini` or `pyproject.toml` [tool.pytest.ini_options] for test discovery

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core data models and service scaffolding that all user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

**Checkpoint**: Models defined, service interfaces ready, configuration management functional

### Data Models (Required for all stories)

- [ ] T010 [P] Create ChapterChunk Pydantic model in `backend/src/models/chunk.py` with fields: chapter_id, chapter_title, section_number, section_title, content, metadata (from data-model.md)
- [ ] T011 [P] Create TextEmbedding Pydantic model in `backend/src/models/chunk.py` with fields: chunk_id, vector (1536-dim float array), metadata dict
- [ ] T012 [P] Create InsertionResult Pydantic model in `backend/src/models/chunk.py` with fields: total, inserted, failed, errors list
- [ ] T013 [P] Create VerificationResult Pydantic model in `backend/src/models/chunk.py` with fields: total_points, sampled, valid, invalid, checks dict

### Configuration & Logging

- [ ] T014 Create logging configuration in `backend/src/config.py` with JSON structured logging (timestamp, operation, status, error_details)
- [ ] T015 Create environment variable loader in `backend/src/config.py` reading OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, EMBEDDING_BATCH_SIZE, EMBEDDING_MODEL defaults

### Service Base Classes (Shared Infrastructure)

- [ ] T016 Create markdown_parser.py stub in `backend/src/services/markdown_parser.py` with function signatures: parse_chapter(), parse_all_chapters() (from contracts/markdown-parser.md)
- [ ] T017 Create embedding_service.py stub in `backend/src/services/embedding_service.py` with function signatures: embed_batch(), embed_chunks() with retry logic (from contracts/embedding-service.md)
- [ ] T018 Create qdrant_service.py stub in `backend/src/services/qdrant_service.py` with function signatures: initialize_collection(), insert_embeddings(), verify_insertion() (from contracts/qdrant-service.md)

**Checkpoint**: Foundation complete - ready for user story implementations in parallel

---

## Phase 3: User Story 1 - Index Chapter Content for Semantic Search (Priority: P1) 🎯 MVP

**Goal**: Implement single chapter chunking, embedding generation, and Qdrant insertion to demonstrate core RAG functionality.

**Independent Test**: Upload single chapter → verify vectors stored in Qdrant with correct metadata → semantic query returns relevant sections.

**Acceptance Scenarios** (from spec.md):
1. Markdown section → split by ## headers with metadata
2. Sections → 1536-dimensional embeddings (note: spec says 768, research.md says 1536; use 1536 per research/ADR-001)
3. Embeddings → stored in Qdrant collection
4. Semantic query → returns ranked results by COSINE similarity

### Implementation for User Story 1

#### Markdown Parsing Service

- [ ] T019 [US1] Implement `parse_chapter(filepath: str) -> List[ChapterChunk]` in `backend/src/services/markdown_parser.py` reading markdown file, extracting chapter_id from filename (ch01.md → ch01), extracting chapter_title from # heading, splitting by ## headers into sections
- [ ] T020 [P] [US1] Implement `parse_all_chapters(directory: str) -> List[ChapterChunk]` in `backend/src/services/markdown_parser.py` listing all .md files, calling parse_chapter() for each, validating no duplicate (chapter_id, section_number) pairs
- [ ] T021 [US1] Add error handling to markdown parser: FileNotFoundError for missing files, ValueError for invalid markdown (no # heading, empty file), UnicodeDecodeError for encoding issues (log and skip invalid files)
- [ ] T022 [P] [US1] Add token counting with `tiktoken` library to markdown parser for each section (validate 50 <= tokens <= 512 per chunk)

#### Embedding Generation Service

- [ ] T023 [US1] Implement `embed_batch(texts: List[str], batch_id: str = None) -> List[List[float]]` in `backend/src/services/embedding_service.py` validating batch size 1-32, calling OpenAI API (model from config), extracting 1536-dim vectors, returning in same order as input
- [ ] T024 [US1] Add retry logic to embed_batch() with exponential backoff: on RateLimitError retry with wait 2^attempt seconds (max 3 attempts = 1s, 2s, 4s), raise EmbeddingAPIError after max retries, log retry attempts
- [ ] T025 [P] [US1] Add input validation to embedding service: empty batch raises ValueError, text > 8000 chars raises ValueError, API auth error raises AuthenticationError with helpful message
- [ ] T026 [US1] Implement `embed_chunks(chunks: List[ChapterChunk], batch_size: int = 32) -> List[TextEmbedding]` in `backend/src/services/embedding_service.py` batching chunks, calling embed_batch(), creating TextEmbedding objects with metadata, logging statistics (total, succeeded, failed)

#### Qdrant Vector Storage

- [ ] T027 [US1] Implement `initialize_collection(collection_name: str = "chapter_embeddings") -> bool` in `backend/src/services/qdrant_service.py` connecting to Qdrant, checking if collection exists, creating if not with VectorParams(size=1536, distance=COSINE), creating payload indexes for chapter_id, section_number, title
- [ ] T028 [US1] Implement `insert_embeddings(embeddings: List[TextEmbedding], collection_name: str = "chapter_embeddings") -> InsertionResult` in `backend/src/services/qdrant_service.py` creating PointStruct objects from embeddings (ID = hash(chunk_id)), upserting to Qdrant, returning InsertionResult with count stats
- [ ] T029 [P] [US1] Add error handling to qdrant_service: QdrantConnectionError for connectivity issues, skip invalid payloads (graceful degradation), retry once on transient errors, log all failures

#### CLI Script for User Story 1

- [ ] T030 [US1] Create `backend/scripts/chunk_chapters.py` script taking --input-dir argument, calling parse_all_chapters(), outputting chunks to JSON file (for testing/debugging single chapter chunking)
- [ ] T031 [P] [US1] Create `backend/scripts/generate_embeddings.py` script taking --input chunks.json argument, calling embed_chunks(), outputting embeddings to JSON file (for testing/debugging single chapter embedding)
- [ ] T032 [P] [US1] Create `backend/scripts/insert_qdrant.py` script taking --input embeddings.json and --collection arguments, initializing collection, calling insert_embeddings(), outputting summary report
- [ ] T033 [US1] Add logging to all scripts: JSON structured logs with timestamp, operation, file/chunk id, status, errors; log to stdout and optionally to file

#### Testing for User Story 1

- [ ] T034 [P] [US1] Create unit test file `backend/tests/unit/test_markdown_parser.py` with tests: parse_chapter_valid_file (verify chunks extracted), parse_chapter_missing_title (ValueError), parse_chapter_empty_file (ValueError), parse_chapter_sequential_sections (section_number 0-indexed), parse_all_chapters_valid_directory (list chunks), parse_all_chapters_missing_directory (FileNotFoundError)
- [ ] T035 [P] [US1] Create unit test file `backend/tests/unit/test_embedding_service.py` with tests: embed_batch_single_text (1536-dim output), embed_batch_multiple_texts (32 texts), embed_batch_invalid_size (error), embed_batch_rate_limit_retry (mock rate limit, verify exponential backoff), embed_chunks_batching (verify batches), embed_chunks_graceful_failure (skip failed batches, return partial results)
- [ ] T036 [P] [US1] Create unit test file `backend/tests/unit/test_qdrant_service.py` with tests: initialize_collection_creates_new (verify schema), initialize_collection_existing (returns True), insert_embeddings_success (verify insertion), insert_embeddings_partial_failure (1 fails, skip, continue), insert_embeddings_upsert (re-insert same chunk updates)

#### Integration Test for User Story 1

- [ ] T037 [US1] Create integration test `backend/tests/integration/test_end_to_end_pipeline.py` with: sample 2-chapter markdown files in `backend/tests/fixtures/chapters/`, test parse_chapter → embed_chunks → insert_embeddings end-to-end, verify vectors in Qdrant match expected metadata, verify semantic search retrieves correct sections

**Checkpoint**: User Story 1 complete - single chapter → vector → Qdrant insertion works. MVP deliverable: can index one chapter and query it semantically.

---

## Phase 4: User Story 2 - Batch Process All Chapters (Priority: P2)

**Goal**: Implement end-to-end pipeline orchestrator that processes 100+ chapters reliably with error handling and rate limiting.

**Independent Test**: Run orchestrator on 5-chapter directory → verify all processed, error report generated, failed batches skipped gracefully.

**Acceptance Scenarios** (from spec.md):
1. Multiple chapters → processed in sequence through chunking, embedding, insertion
2. Rate limit error → logged, retried with exponential backoff, pipeline continues
3. Pipeline completion → summary report showing processed count, skipped, errors

### Implementation for User Story 2

#### Pipeline Orchestrator

- [ ] T038 [US2] Implement `process_all(input_dir: str, collection_name: str = "chapter_embeddings")` orchestrator in `backend/scripts/process_all.py` calling: markdown_parser.parse_all_chapters() → embedding_service.embed_chunks() → qdrant_service.insert_embeddings(), aggregating results into ProcessingResult (total, embedded, inserted, failed files, failed chunks, processing time)
- [ ] T039 [US2] Add error recovery to orchestrator: on embedding rate limit → exponential backoff with max 3 retries at batch level (not individual chunks), on Qdrant connection error → log and skip batch, continue with next chapters
- [ ] T040 [P] [US2] Add graceful degradation to orchestrator: failed chapters logged with error reason (file format, encoding, API error), pipeline continues, final report summarizes success/failure statistics
- [ ] T041 [US2] Implement summary reporting in orchestrator: print human-readable report "Indexed X/Y chapters, Z chunks successfully embedded, A errors encountered. Processing time: B minutes." Also output JSON log for programmatic use.

#### Configuration for Batch Processing

- [ ] T042 [P] [US2] Add batch processing configuration to `backend/src/config.py`: EMBEDDING_BATCH_SIZE (default 32), EMBEDDING_MAX_RETRIES (default 3), INITIAL_BACKOFF (default 1 second), MAX_BACKOFF (default 8 seconds)
- [ ] T043 [P] [US2] Create CLI wrapper `backend/scripts/process_all.py` with argparse: --input-dir (required), --output-log (optional), --collection (default "chapter_embeddings") arguments

#### Enhanced Error Handling

- [ ] T044 [US2] Implement rate limit handling with exponential backoff in `backend/src/services/embedding_service.py` refining embed_chunks() to retry entire batch on rate limit, logging retry attempts, giving up after max retries with clear error message
- [ ] T045 [P] [US2] Add batch-level error recovery to embedding_service: if batch fails after 3 retries, log batch_id, error details, and affected chapter_ids; return partial results (other batches still processed)

#### Testing for User Story 2

- [ ] T046 [P] [US2] Create integration test `backend/tests/integration/test_batch_processing.py` mocking OpenAI API with rate limit errors (fail twice, succeed on 3rd attempt), verifying exponential backoff, verifying batch processes eventually succeeds
- [ ] T047 [P] [US2] Create integration test for graceful degradation: one chapter file invalid (bad markdown), others process successfully, verify pipeline completes with partial results and clear error log
- [ ] T048 [US2] Create end-to-end test `backend/tests/integration/test_e2e_orchestrator.py` running orchestrator on 5-chapter fixture directory, verifying ProcessingResult statistics, verifying all valid chapters indexed, verifying error report for any invalid chapters

**Checkpoint**: User Story 2 complete - full textbook (100+ chapters) can be indexed in one command with reliable error handling. Production-ready feature.

---

## Phase 5: User Story 3 - Verify Embedding Quality (Priority: P3)

**Goal**: Implement data validation and query performance verification to ensure embeddings are correctly indexed.

**Independent Test**: Query Qdrant with sample vectors → verify metadata correct, payload indexes fast, vectors retrievable.

**Acceptance Scenarios** (from spec.md):
1. Sample vectors retrieved → compared against expected metadata (chapter_id, section_number, title)
2. Payload index filtering → queries with chapter_id filter return results fast (< 100ms)

### Implementation for User Story 3

#### Verification Service

- [ ] T049 [US3] Implement `verify_insertion(sample_size: int = 10, collection_name: str = "chapter_embeddings") -> VerificationResult` in `backend/src/services/qdrant_service.py` getting collection info, retrieving sample_size random points, validating: vector dimensions exactly 1536, all metadata fields present (chapter_id, section_number, title, content), metadata types correct (chapter_id string, section_number int)
- [ ] T050 [P] [US3] Add performance checks to verify_insertion(): test payload filtering queries complete in < 100ms, test full collection search completes in < 1 second, log query latencies
- [ ] T051 [US3] Create VerificationResult model in `backend/src/models/chunk.py` with fields: total_points, sampled, valid, invalid, checks dict (dimensions, metadata, payload_indexing, query_performance all bool)

#### Verification CLI Script

- [ ] T052 [US3] Create `backend/scripts/verify_qdrant.py` script taking --collection argument, calling qdrant_service.verify_insertion(), printing human-readable results: "✓ All X sampled vectors verified" or "✗ Y/X vectors failed checks: {details}", printing performance metrics (query latencies)

#### Testing for User Story 3

- [ ] T053 [P] [US3] Create unit test `backend/tests/unit/test_qdrant_verification.py` mocking Qdrant client: test_verify_insertion_valid (10 samples pass all checks), test_verify_insertion_invalid_dimensions (mock vector with wrong dims, detected), test_verify_insertion_missing_metadata (mock point missing field, detected)
- [ ] T054 [P] [US3] Create performance test `backend/tests/integration/test_query_performance.py` inserting 100 sample vectors, measuring query latencies: payload filter queries (chapter_id) < 100ms, full search < 1s, logging latencies

#### Documentation for User Story 3

- [ ] T055 [US3] Create `backend/VERIFICATION.md` documenting how to verify embeddings: run `python -m scripts.verify_qdrant --collection chapter_embeddings`, interpret results, example output, troubleshooting guide (missing vectors, slow queries, etc.)

**Checkpoint**: User Story 3 complete - data integrity and performance verified. QA checks available for production deployment.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, documentation, deployment readiness

**Checkpoint**: Feature production-ready, fully tested, documented, deployed

### Comprehensive Testing

- [ ] T056 [P] Create `backend/tests/conftest.py` with pytest fixtures: temp_chapters_dir (5 test chapters), mock_qdrant_client (mock Qdrant for unit tests), mock_openai_client (mock OpenAI API with sample embeddings)
- [ ] T057 [P] Configure pytest to run all tests in `backend/tests/` with coverage reporting (pytest --cov=backend/src), target 80%+ coverage for critical paths (parsing, embedding, insertion)
- [ ] T058 Create GitHub Actions workflow in `.github/workflows/test-embeddings.yml` running on PR: `pytest backend/tests/ --cov=backend/src --cov-fail-under=80`, failing if coverage below 80%

### Documentation

- [ ] T059 Create `backend/README.md` with: project overview, setup instructions, how to run each script (chunk, embed, insert, process_all, verify), environment variables, example usage, troubleshooting guide
- [ ] T060 [P] Create `backend/API.md` documenting all service APIs: MarkdownParser functions, EmbeddingService functions, QdrantService functions, input/output contracts from contracts/ directory
- [ ] T061 [P] Create `backend/ARCHITECTURE.md` explaining design: modular services, data flow (markdown → chunks → vectors → Qdrant), error handling strategy, logging format, deployment considerations
- [ ] T062 [P] Update `specs/003-qdrant-embeddings/quickstart.md` with actual command syntax once scripts are complete (currently template)

### Deployment & CI/CD

- [ ] T063 Create `backend/.env.example` with all required environment variables and defaults documented
- [ ] T064 [P] Create `backend/Dockerfile` for containerized deployment (Python 3.11, Poetry, run process_all.py)
- [ ] T065 [P] Create `backend/docker-compose.yml` for local testing with Qdrant container
- [ ] T066 Create CI/CD step in GitHub Actions to auto-run embeddings pipeline on chapter updates (trigger: changes to frontend/docs/chapters/)

### Final Validation

- [ ] T067 Run full integration test suite: `pytest backend/tests/integration/ -v` all tests pass
- [ ] T068 [P] Verify quickstart scenario works end-to-end: create 5 sample chapters, run orchestrator, verify results in Qdrant, run verification script
- [ ] T069 Manual testing with real OpenAI API and Qdrant Cloud instance (staging environment): process 10 chapters, verify costs, verify performance metrics
- [ ] T070 Code review checklist: all functions typed (Python type hints), all public functions documented (docstrings), all errors logged (no silent failures), error messages user-friendly

**Checkpoint**: Feature complete, tested, documented, deployable to production. Ready for team review and release.

---

## Implementation Strategy

### MVP (Minimal Viable Product)

**Scope**: User Story 1 (P1) + Phase 2 Foundational
- **Deliverable**: Single chapter indexing with semantic search
- **Timeline**: Tasks T001-T037 (~80 tasks including tests)
- **User Value**: Demonstrate RAG capability on small dataset
- **Next**: Add batch processing (US2) for production scale

### Phase 2 (Operational Readiness)

**Scope**: User Story 2 (P2) + enhanced error handling
- **Deliverable**: 100-chapter textbook indexing in one command
- **Timeline**: Tasks T038-T048 (~10 additional tasks)
- **User Value**: Production-ready pipeline for content teams
- **Next**: Add verification (US3) for QA workflows

### Phase 3 (Quality Assurance)

**Scope**: User Story 3 (P3) + comprehensive testing
- **Deliverable**: Data integrity validation and performance monitoring
- **Timeline**: Tasks T049-T055 (~7 additional tasks)
- **User Value**: Confidence in data quality before RAG deployment
- **Next**: Documentation and deployment

### Parallelization Opportunities

**Tasks that can run in parallel** (marked with [P]):
- **Phase 1**: All dependency setup and configuration (T003-T009)
- **Phase 2**: All model definitions (T010-T013), all service stubs (T016-T018)
- **Phase 3**: Markdown parser (T019-T022), embedding service impl (T023-T026), Qdrant service impl (T027-T029), all unit tests (T034-T036)
- **Phase 4**: Configuration updates (T042-T043), error handling (T044-T045), tests (T046-T048)
- **Phase 5**: All tests (T053-T054), documentation (T055)
- **Phase 6**: All documentation (T059-T062), CI/CD (T063-T066)

**Example parallel execution**:
```
Phase 1:
  T001 (sequential: create dir)
  T002-T009 (parallel: setup dependencies, config)

Phase 2:
  T010-T018 (parallel: all models and service stubs)

Phase 3 (can be parallelized into 3 teams):
  Team A: T019-T022 (markdown parser)
  Team B: T023-T026 (embedding service)
  Team C: T027-T029 (Qdrant service)

  Then in parallel:
    T030-T033 (scripts)
    T034-T037 (tests)
```

---

## Task Dependencies

```
Phase 1 (Setup):
  T001 → T002-T009 (all depend on project structure)

Phase 2 (Foundational):
  T002 → T010-T018 (models and services require dependencies)

Phase 3 (User Story 1):
  T002, T010-T018 → T019-T033 (requires models, dependencies)
  T019-T026 → T034-T037 (tests depend on implementations)

Phase 4 (User Story 2):
  T019-T029, T038 → T039-T048 (orchestrator depends on services)

Phase 5 (User Story 3):
  T028-T029 → T049-T055 (verification depends on Qdrant service)

Phase 6 (Polish):
  T001-T055 → T056-T070 (final integration, docs, testing)
```

---

## Success Criteria for Task Completion

Each task is complete when:

1. **Code exists** at specified file path with expected functions/classes
2. **Behavior matches** contract specification (input/output types, error handling)
3. **Tested** with unit or integration tests (tests marked T034-T070 verify implementations)
4. **Logged** with JSON structured logging (timestamp, operation, status, error_details)
5. **Documented** with docstrings and error messages (no cryptic failures)
6. **Integrated** into orchestrator workflow (functions called correctly by Phase 4 orchestrator)

**Overall feature complete when**:
- ✅ All 70 tasks completed
- ✅ All tests pass (pytest backend/tests/ --cov=backend/src)
- ✅ Coverage >= 80% (critical business logic: parsing, embedding, insertion)
- ✅ MVP scenario works: single chapter → indexed → queryable
- ✅ Production scenario works: 100 chapters → indexed in < 5 min with error handling
- ✅ QA scenario works: verify_qdrant script passes all checks

---

## Notes

- **Tests**: Included per specification request (tests are part of acceptance criteria)
- **Configuration**: Externalized to `backend/src/config.py` (environment variables, sensible defaults)
- **Error Handling**: Graceful degradation (failed chunks logged and skipped, pipeline continues)
- **Logging**: JSON structured logs throughout (not print statements)
- **Architecture**: Modular services with clear contracts (testable independently)
- **Documentation**: Comprehensive (README, API, Architecture, Quickstart, Verification)

