# Tasks: Chapter Context Awareness

**Input**: Design documents from `/specs/007-chapter-context/`
**Prerequisites**: plan.md (✅), spec.md (✅), research.md (✅), data-model.md (✅), contracts/ (✅)
**Branch**: `007-chapter-context`

**Organization**: Tasks grouped by user story (US1-US4) to enable independent, parallel implementation. Each story is independently testable and deliverable.

**Test Strategy**: Tests are OPTIONAL. Generate only if TDD approach requested. Implementation uses existing test patterns from Phase 2.1-2.4.

---

## Format: `- [ ] [TaskID] [P?] [Story] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: User story label (US1, US2, US3, US4) for story-phase tasks only
- **File paths**: Absolute paths relative to repository root

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Verify project state and establish development environment

- [ ] T001 Verify existing ChatKit integration in frontend/src/components/ChatKit/
- [ ] T002 Verify existing Qdrant indexing with chapter_id metadata in backend/src/services/qdrant_service.py
- [ ] T003 Review constitution compliance checklist (all 7 principles passed)
- [ ] T004 Create feature branch tracking: git branch 007-chapter-context (already exists)

**Checkpoint**: Project structure verified; ready for foundational tasks

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core type definitions and API schema that all user stories depend on

**⚠️ CRITICAL**: These tasks MUST complete before US1-US4 can begin. They are blocking prerequisites.

### Backend Schema Extension

- [ ] T005 Extend ChatRequest schema in backend/src/models/chat.py:
  - Add optional `chapter_context: Optional[dict] = None` field
  - Add Pydantic validation: chapter_id matches pattern `^[a-z0-9\-]+$`
  - Add docstring referencing FR-007
  - File: `backend/src/models/chat.py` (modify ChatRequest class)

- [ ] T006 Extend ResponseMetadata schema in backend/src/models/chat.py:
  - Add `chapter_filtered: Optional[bool] = False` field
  - Add `chapter_id: Optional[str] = None` field
  - Update ChatResponse example to include new metadata fields
  - File: `backend/src/models/chat.py` (modify ResponseMetadata class)

### Frontend Type Definitions

- [ ] T007 Add ChapterContext type to frontend/src/components/ChatKit/types/chatkit.types.ts:
  ```typescript
  export interface ChapterContext {
    chapterId: string;
    chapterTitle: string;
    chapterSlug: string;
    confidence: 'high' | 'medium' | 'low';
  }
  ```
  - File: `frontend/src/components/ChatKit/types/chatkit.types.ts`

- [ ] T008 Extend RAGRequest type in frontend/src/components/ChatKit/types/chatkit.types.ts:
  - Add `chapter_context?: ChapterContext | null` field
  - Update RAGResponse example to include chapter_filtered metadata
  - File: `frontend/src/components/ChatKit/types/chatkit.types.ts`

**Checkpoint**: API contract defined in code; ready for user story implementation

---

## Phase 3: User Story 1 - Extract and Display Current Chapter (Priority: P1) 🎯

**Goal**: Frontend automatically detects current chapter and displays it as a badge in ChatKit header

**Independent Test**: Navigate to `/docs/chapter-3-kinematics` → ChatKit widget displays "Ch. 3 - Kinematics" badge

### Implementation for US1

- [ ] T009 [P] [US1] Create useChapterContext hook in frontend/src/components/ChatKit/hooks/useChapterContext.ts:
  - Function: `extractChapterIdFromUrl(pathname: string): string | null`
    - Regex pattern: `/chapter[_-]?(\w+)/i`
    - Example: `/docs/chapter-3-kinematics` → `"chapter-3"`
  - Function: `extractTitleFromDOM(): string | null`
    - Query: `document.querySelector('h1')?.textContent`
  - Function: `titleToSlug(title: string): string`
    - Lowercase, replace spaces with hyphens
  - Hook: `useChapterContext(): ChapterContext | null`
    - Uses `useLocation()` from react-router-dom
    - Extracts chapter ID and title
    - Returns `{ chapterId, chapterTitle, chapterSlug, confidence }` or `null`
    - Confidence: "high" if both URL and h1 match, "medium" if only one matches
  - File: `frontend/src/components/ChatKit/hooks/useChapterContext.ts` (NEW)

- [ ] T010 [P] [US1] Create unit tests for useChapterContext hook in frontend/src/components/ChatKit/hooks/__tests__/useChapterContext.test.ts:
  - Test URL extraction: `/docs/chapter-3-kinematics` → `"chapter-3"`
  - Test URL extraction with special chars: `/docs/chapter-advanced-motion-planning` → `"chapter-advanced-motion-planning"`
  - Test DOM extraction: h1 tag content → chapter title
  - Test slug generation: "Inverse Kinematics" → "inverse-kinematics"
  - Test null return on non-chapter pages
  - Test confidence scoring (high/medium/low)
  - File: `frontend/src/components/ChatKit/hooks/__tests__/useChapterContext.test.ts` (NEW)

- [ ] T011 [P] [US1] Update ChatKitWidget.tsx to display chapter badge in frontend/src/components/ChatKit/ChatKitWidget.tsx:
  - Import useChapterContext hook
  - Call hook in component: `const chapterContext = useChapterContext()`
  - Display badge in header: `{chapterContext && <div className="chapter-badge">Ch. {chapterId} - {chapterTitle}</div>}`
  - Hide badge on non-chapter pages (chapterContext === null)
  - Update chapter badge when chapterId changes (dependency array)
  - File: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (MODIFY)

- [ ] T012 [P] [US1] Add chapter badge styling in frontend/src/components/ChatKit/styles/chatkit.css:
  - CSS class: `.chapter-badge`
    - Position: top-left of ChatKit header
    - Style: gray background, rounded corners, padding 4px 8px
    - Font: small, sans-serif
    - Example: "Ch. 3 - Kinematics"
  - Responsive: hide on mobile if space constrained
  - File: `frontend/src/components/ChatKit/styles/chatkit.css` (MODIFY)

**Checkpoint**: Chapter detection and UI display complete. User Story 1 independently testable.

---

## Phase 4: User Story 2 - Filter Search Results by Current Chapter (Priority: P1)

**Goal**: Backend receives chapter_context and prioritizes Qdrant search results from that chapter

**Independent Test**: Submit question on Chapter 3 page → Qdrant results from Chapter 3 rank first

### Implementation for US2

- [ ] T013 [P] [US2] Create chapter_filter.py service in backend/src/services/utils/chapter_filter.py:
  - Class: `ChapterFilterEngine`
  - Method: `filter_by_chapter(search_results: List[dict], chapter_id: str, preserve_all: bool = True) -> List[dict]`
    - Separate results into two groups:
      - Group 1: results where `payload.chapter_id == chapter_id`
      - Group 2: results where `payload.chapter_id != chapter_id`
    - If `preserve_all=True`: return `[...group1, ...group2]` (preserve all results)
    - If `preserve_all=False`: return only `group1` (dangerous fallback)
    - Maintain original score ordering within each group
  - Method: `get_boost_metadata(chapter_id: str) -> dict`
    - Returns: `{ "chapter_filtered": true, "chapter_id": chapter_id }`
  - File: `backend/src/services/utils/chapter_filter.py` (NEW)

- [ ] T014 [P] [US2] Create unit tests for chapter filtering in backend/tests/unit/test_chapter_filter.py:
  - Test filter with chapter-matching results
  - Test filter with no chapter-matching results (fallback)
  - Test filter preserves all results (not filtering-only)
  - Test filter maintains score ordering within groups
  - Test with empty search results
  - File: `backend/tests/unit/test_chapter_filter.py` (NEW)

- [ ] T015 [US2] Update qdrant_service.py to accept chapter_id parameter in backend/src/services/qdrant_service.py:
  - Modify `search()` method signature to accept optional `chapter_id: Optional[str] = None`
  - If `chapter_id` provided:
    - Call ChapterFilterEngine.filter_by_chapter() on results
    - Return re-ranked results
  - If `chapter_id` is None: return results unchanged (backward compatible)
  - File: `backend/src/services/qdrant_service.py` (MODIFY search method)

- [ ] T016 [US2] Create integration test for chapter filtering in backend/tests/integration/test_chapter_filtering.py:
  - Test end-to-end: query → Qdrant search with chapter_id → re-ranked results
  - Verify chapter-matching results appear first
  - Verify cross-chapter results appear after chapter-matching
  - Verify no results dropped (all preserved)
  - File: `backend/tests/integration/test_chapter_filtering.py` (NEW)

**Checkpoint**: Backend filtering logic complete. User Story 2 independently testable.

---

## Phase 5: User Story 3 - Send Chapter Context to Backend (Priority: P1)

**Goal**: Frontend extracts chapter context and includes it in all RAG API requests

**Independent Test**: Make API request from chapter page → verify request body includes `chapter_context` field with chapter_id and chapter_title

### Implementation for US3

- [ ] T017 [P] [US3] Update apiService.ts to send chapter_context in frontend/src/components/ChatKit/services/apiService.ts:
  - Modify `sendQuestion()` signature to accept optional `chapterContext: ChapterContext | null` parameter
  - When sending POST request to `/api/v1/chat/ask`:
    - If `chapterContext` provided: include in payload as `chapter_context: { chapter_id, chapter_title }`
    - If `chapterContext` is null: omit from payload (backward compatible)
  - Example request body:
    ```json
    {
      "question": "What is forward kinematics?",
      "chapter_context": {
        "chapter_id": "ch03",
        "chapter_title": "Kinematics"
      }
    }
    ```
  - File: `frontend/src/components/ChatKit/services/apiService.ts` (MODIFY sendQuestion function)

- [ ] T018 [P] [US3] Update ChatKitWidget.tsx to pass chapter_context to API calls in frontend/src/components/ChatKit/ChatKitWidget.tsx:
  - In `handleSendQuestion()` function:
    - Get chapter context: `const chapterContext = useChapterContext()`
    - Pass to sendQuestion: `await sendQuestion(question, selectedText, pageContext, chapterContext)`
  - File: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (MODIFY handleSendQuestion)

- [ ] T019 [P] [US3] Create unit test for chapter_context payload in frontend/src/components/ChatKit/services/__tests__/apiService.test.ts:
  - Test sendQuestion includes chapter_context when provided
  - Test sendQuestion omits chapter_context when null
  - Test chapter_context shape: `{ chapter_id, chapter_title }`
  - File: `frontend/src/components/ChatKit/services/__tests__/apiService.test.ts` (MODIFY)

**Checkpoint**: Frontend sends chapter context to backend. User Story 3 independently testable.

---

## Phase 6: User Story 4 - Prioritize Chapter Results in Qdrant (Priority: P2)

**Goal**: Backend implements advanced result prioritization using chapter-based boosting

**Independent Test**: Search for chapter-specific topic on chapter page → top 3 results from that chapter (when relevant)

### Implementation for US4

- [ ] T020 [US4] Update chat.py endpoint to process chapter_context in backend/src/api/v1/routes/chat.py:
  - In `/api/v1/chat/ask` route:
    - Extract chapter_id: `chapter_id = request.chapter_context.get("chapter_id") if request.chapter_context else None`
    - Pass to Qdrant search: `retrieved_chunks = await qdrant_service.search(query_embedding, chapter_id=chapter_id, limit=5)`
    - Update response metadata:
      ```python
      response.metadata.chapter_filtered = bool(chapter_id)
      response.metadata.chapter_id = chapter_id
      ```
  - File: `backend/src/api/v1/routes/chat.py` (MODIFY ask function)

- [ ] T021 [US4] Create integration test for end-to-end chapter-aware RAG in backend/tests/integration/test_chapter_aware_rag.py:
  - Test full RAG pipeline with chapter context:
    1. User on Chapter 3 asks question
    2. Frontend sends request with chapter_context
    3. Backend receives and processes chapter_context
    4. Qdrant search returns results re-ranked by chapter
    5. Gemini generates answer using chapter-relevant context
    6. Response includes metadata with chapter_filtered=true
  - Verify response latency increase < 100ms
  - Verify chapter results rank first (when relevant)
  - File: `backend/tests/integration/test_chapter_aware_rag.py` (NEW)

- [ ] T022 [US4] Add logging for chapter filtering in backend/src/api/v1/routes/chat.py:
  - Log when chapter_context received: `logger.info(f"Chapter filtering applied: {chapter_id}")`
  - Log search re-ranking result count: `logger.info(f"Chapter results: X, Others: Y")`
  - Include in structured logging with request_id
  - File: `backend/src/api/v1/routes/chat.py` (MODIFY ask function)

- [ ] T023 [US4] Add error handling for invalid chapter_context in backend/src/api/v1/routes/chat.py:
  - If chapter_context provided but chapter_id doesn't match pattern: log warning
  - If chapter_id not found in Qdrant: gracefully fall back to global search (no error)
  - Return helpful error message if validation fails
  - File: `backend/src/api/v1/routes/chat.py` (MODIFY ask function)

**Checkpoint**: Advanced prioritization complete. User Story 4 independently testable.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Integration, documentation, and quality assurance

### Documentation & Observability

- [ ] T024 [P] Update API documentation in docs/api.md:
  - Add chapter_context parameter description
  - Add example request with chapter context
  - Add metadata response fields explanation
  - File: `docs/api.md` (MODIFY or CREATE)

- [ ] T025 [P] Add chapter context to observability logging in backend/src/config.py:
  - Ensure chapter_context included in structured logs
  - Log level: INFO for successful filtering, WARN for edge cases
  - File: `backend/src/config.py` (VERIFY logging config)

### Edge Case Handling

- [ ] T026 [US1] Handle special characters in chapter URLs in frontend/src/components/ChatKit/hooks/useChapterContext.ts:
  - Test URL like `/docs/chapter-3-advanced-motion-planning`
  - Verify extraction: `"chapter-3-advanced-motion-planning"`
  - File: `frontend/src/components/ChatKit/hooks/useChapterContext.ts` (VERIFY)

- [ ] T027 [US1] Handle malformed h1 tags in frontend/src/components/ChatKit/hooks/useChapterContext.ts:
  - If h1 not found: fall back to extracting from URL
  - If h1 empty: mark confidence as "low"
  - File: `frontend/src/components/ChatKit/hooks/useChapterContext.ts` (VERIFY)

- [ ] T028 [US2] Handle pages with no chapter context in backend/src/services/qdrant_service.py:
  - If chapter_id is None: perform global search (no filtering)
  - Verify graceful fallback (no errors)
  - File: `backend/src/services/qdrant_service.py` (VERIFY)

- [ ] T029 [US2] Handle chapters with no matching results in backend/src/services/utils/chapter_filter.py:
  - If chapter_id provided but no chapter-matching results: return all results (fallback)
  - Log warning: "No results from chapter, falling back to global search"
  - File: `backend/src/services/utils/chapter_filter.py` (VERIFY)

### Performance & Metrics

- [ ] T030 [P] Add performance metrics for chapter extraction in frontend/src/components/ChatKit/hooks/useChapterContext.ts:
  - Measure time for URL extraction (should be <10ms)
  - Measure time for DOM extraction (should be <20ms)
  - Measure time for slug generation (should be <5ms)
  - Total should be <50ms
  - File: `frontend/src/components/ChatKit/hooks/useChapterContext.ts` (ADD performance logging)

- [ ] T031 [P] Add performance metrics for Qdrant re-ranking in backend/src/services/utils/chapter_filter.py:
  - Measure time for filtering operation (should be <50ms for 5 results)
  - Log if exceeds threshold (warn at >75ms)
  - File: `backend/src/services/utils/chapter_filter.py` (ADD performance logging)

### Backward Compatibility

- [ ] T032 Verify backward compatibility with existing ChatKit behavior:
  - Test ChatKit on non-chapter pages (no chapter badge displayed)
  - Test API requests without chapter_context (graceful handling)
  - Test Qdrant search without chapter_id (returns results unchanged)
  - Verify existing integration tests still pass
  - File: Run existing test suite

### Code Quality & Testing

- [ ] T033 [P] Ensure code follows constitution standards:
  - Type safety: All functions have type hints (TypeScript/Python)
  - Error handling: All error paths handled with user-friendly messages
  - Logging: Structured logging with request_id for tracing
  - Testing: Unit + integration tests for all new code
  - File: Review against constitution.md

- [ ] T034 [P] Run linting and formatting:
  - Frontend: ESLint + Prettier on new files
  - Backend: Pylint + black on new files
  - File: Run `npm run lint` and `python -m pylint`

- [ ] T035 Run all test suites:
  - Unit tests: `npm run test` (frontend), `pytest` (backend)
  - Integration tests: `pytest tests/integration/`
  - Verify coverage > 80% for new code
  - File: Run test commands

### End-to-End Testing (Manual)

- [ ] T036 [P] Manual E2E test - Chapter badge display:
  - Navigate to `/docs/chapter-3-kinematics`
  - Verify ChatKit header shows "Ch. 3 - Kinematics" badge
  - Navigate to `/docs` (non-chapter page)
  - Verify badge not displayed
  - Navigate to different chapter
  - Verify badge updates to new chapter name

- [ ] T037 [P] Manual E2E test - Chapter-aware search:
  - On Chapter 3 page, ask: "What is forward kinematics?"
  - Verify first result is from Chapter 3 (if relevant content exists)
  - Verify other chapters' results appear after
  - Ask same question from Chapter 5 page
  - Verify Chapter 5 results rank first

- [ ] T038 [P] Manual E2E test - Non-chapter pages:
  - Open ChatKit on homepage (no chapter context)
  - Ask question
  - Verify search works (global search, no chapter filtering)
  - Verify no errors logged
  - Verify response metadata has chapter_filtered=false

**Checkpoint**: All features implemented, tested, and verified. Ready for merge.

---

## Summary & Execution Strategy

### Task Counts by Phase

| Phase | Tasks | Purpose |
|-------|-------|---------|
| Phase 1: Setup | 4 | Project initialization |
| Phase 2: Foundational | 4 | API schema & types (BLOCKING) |
| Phase 3: US1 | 4 | Chapter detection & UI (P1) |
| Phase 4: US2 | 4 | Qdrant filtering (P1) |
| Phase 5: US3 | 3 | API payload (P1) |
| Phase 6: US4 | 4 | Advanced prioritization (P2) |
| Phase 7: Polish | 11 | Integration, edge cases, testing |
| **TOTAL** | **38 tasks** | |

### Independent User Stories (Can Execute in Parallel After Phase 2)

1. **US1** (4 tasks): Extract chapter & display badge
2. **US2** (4 tasks): Filter Qdrant results
3. **US3** (3 tasks): Send chapter context in API
4. **US4** (4 tasks): Advanced result prioritization

**Suggested Execution**:
- Execute Phase 1-2 sequentially (setup & schema blocking)
- Execute US1-US3 tasks in parallel (no dependencies between them)
- Execute US4 after US2 (depends on filtering logic)
- Execute Phase 7 polish tasks after all user stories complete

### MVP Scope

**Minimum Viable Product** (Delivers Value at Each Checkpoint):
1. ✅ Phase 1-2: Foundation complete
2. ✅ Phase 3 (US1): Chapter badge displays (immediate UX improvement)
3. ✅ Phase 4 (US2): Search filtering works (solves core problem)
4. ✅ Phase 5 (US3): API payload sent (enables backend processing)
5. ✅ Phase 6 (US4): Advanced prioritization (optimization)

**Deliver after Phase 5**: Core feature complete, user-visible value delivered
**Deliver after Phase 6**: All planned features complete
**Deliver after Phase 7**: Production-ready with full test coverage & documentation

### Testing Strategy

**Unit Tests**: ✅ Included in tasks (useChapterContext, chapter_filter)
**Integration Tests**: ✅ Included in tasks (end-to-end RAG pipeline)
**E2E Tests**: ✅ Included as manual tests (browser-based verification)

**Test Framework**:
- Frontend: Vitest (existing from Phase 2.3)
- Backend: pytest (existing from Phase 2.2)
- E2E: Manual (Playwright integration provided in phase 2.5 test milestone)

### Success Criteria (from Specification)

Each task directly addresses success criteria:

- ✅ SC-001: 100% chapter extraction → T009 (useChapterContext)
- ✅ SC-002: Chapter badge displays → T011 (ChatKitWidget update)
- ✅ SC-003: API includes chapter_context → T017-T018 (payload)
- ✅ SC-004: Top 3 results from chapter → T013-T015 (filtering)
- ✅ SC-005: <50ms additional latency → T030-T031 (performance)
- ✅ SC-006: Graceful fallback → T028-T029 (edge cases)
- ✅ SC-007: Context preserved through pipeline → T020-T022 (logging)

---

## Next Steps

1. **Verify Prerequisites**: Confirm all design documents complete (spec ✅, plan ✅, research ✅)
2. **Execute Phase 1-2**: Setup infrastructure and schema extensions (4+4 = 8 tasks)
3. **Parallel Execution**: Start US1-US3 tasks simultaneously (independent implementation)
4. **Sequential Closure**: Complete US4 tasks (depends on US2)
5. **Quality Assurance**: Run Phase 7 polish and E2E tests (11 tasks)
6. **Integration Review**: Verify all success criteria met
7. **Create PR**: Commit changes and request review against constitution

---

## File Modification Summary

### New Files Created
```
frontend/src/components/ChatKit/hooks/useChapterContext.ts
frontend/src/components/ChatKit/hooks/__tests__/useChapterContext.test.ts
backend/src/services/utils/chapter_filter.py
backend/tests/unit/test_chapter_filter.py
backend/tests/integration/test_chapter_filtering.py
backend/tests/integration/test_chapter_aware_rag.py
```

### Existing Files Modified
```
backend/src/models/chat.py (ChatRequest & ResponseMetadata)
backend/src/api/v1/routes/chat.py (ask endpoint)
backend/src/services/qdrant_service.py (search method)
frontend/src/components/ChatKit/ChatKitWidget.tsx (display & API calls)
frontend/src/components/ChatKit/services/apiService.ts (sendQuestion)
frontend/src/components/ChatKit/types/chatkit.types.ts (types)
frontend/src/components/ChatKit/styles/chatkit.css (styling)
```

---

## Constitution Compliance Checklist

Before creating PR, verify:

- ✅ **Production-Grade Quality**: Error handling, type safety, tests (T010, T014, T019, T021-T023, T033)
- ✅ **Privacy-First**: No new PII collected; chapter context is operational metadata only
- ✅ **RAG Accuracy**: Chapter filtering improves citation accuracy (T020-T022)
- ✅ **Modular & Testable**: Each user story independently testable (T009-T038)
- ✅ **Content Quality**: No changes to content; improves accessibility
- ✅ **Observability**: Logging captures chapter_context (T022, T025, T030-T031)
- ✅ **Spec-Driven Development**: Following SDD workflow (spec → plan → tasks → implementation)

---

**Status**: ✨ **READY FOR IMPLEMENTATION** - All 38 tasks defined, prioritized, and independently testable.
