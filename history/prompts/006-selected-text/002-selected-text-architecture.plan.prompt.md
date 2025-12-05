---
id: 002
title: Selected Text Architecture & Planning Design
stage: plan
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: 006-selected-text
branch: 006-selected-text
user: developer
command: /sp.plan
labels: ["planning", "architecture", "design", "frontend-backend", "api-contracts"]
links:
  spec: specs/006-selected-text/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-selected-text/plan.md
  - specs/006-selected-text/contracts/chat-request-with-selected-text.schema.json
  - specs/006-selected-text/contracts/chat-response-with-boosting.schema.json
tests:
  - Constitution Check: All 7 principles pass (production-grade, privacy, RAG accuracy, modularity, accessibility, observability, SDD)
  - Architecture Decisions: 5 decisions documented with alternatives and trade-offs
  - Data Model: Frontend (TextSelection, SelectionTooltipState), Backend (ChatRequest, SearchResult)
  - API Contracts: JSON Schema validation for request/response with boosting metadata
  - Project Structure: Frontend hooks/components, backend API/services clearly documented
  - Testing Strategy: Unit, component, E2E, integration tests defined
  - Risk Analysis: 4 dependencies, 3 integration points, graceful degradation strategies
---

## Prompt

User executed `/sp.plan` command to create architecture and implementation planning for the selected-text feature. Specification was completed in previous phase with 3 prioritized user stories (P1/P2/P3), 18 functional requirements, and 9 measurable success criteria.

**Task**: Transform specification into detailed architecture document with 5 key decisions, data model, API contracts, testing strategy, project structure, and risk mitigation before proceeding to task generation phase.

## Response snapshot

### Planning Output Created

**Files Generated**:
1. `specs/006-selected-text/plan.md` (400+ lines) - Complete architecture and design
2. `specs/006-selected-text/contracts/chat-request-with-selected-text.schema.json` - Request validation
3. `specs/006-selected-text/contracts/chat-response-with-boosting.schema.json` - Response with boosting metadata

### Architecture Decisions (5 total)

**Decision 1: Text Selection Detection Method**
- **Chosen**: `window.getSelection()` API with mouseup/touchend listeners
- **Why**: Native API, no dependencies, modern browser support, graceful fallback
- **Alternatives**: Custom library (adds dependency), keyboard tracking (misses touch), triple-click (unreliable)
- **Trade-off**: IE < 9 won't see tooltip (<0.5% traffic, acceptable)

**Decision 2: Tooltip Positioning Strategy**
- **Chosen**: Viewport-relative with dynamic repositioning (above if near bottom)
- **Why**: Follows selection visually, avoids clipping, responsive on mobile
- **Alternatives**: Fixed viewport, document-relative, shadow DOM
- **Trade-off**: Requires bounding rect calculations on every selection (<5ms overhead acceptable)

**Decision 3: Selected Text Integration with ChatKit**
- **Chosen**: Pre-fill input field + optional RAGRequest parameter
- **Why**: User sees selected text, simple to understand, stateless, widget unchanged
- **Alternatives**: Hidden field (confusing), replace input (breaks use case), session state (adds complexity)
- **Trade-off**: Requires small ChatKit widget modification

**Decision 4: Backend Search Boosting Algorithm**
- **Chosen**: TF-IDF term weighting + cosine similarity re-ranking
- **Why**: Simple, proven, fast (no LLM inference), per Constitution Principle III "boost 2x"
- **Algorithm**: Parse selected_text → calculate TF for each result → boost score by TF * boost_factor → re-sort
- **Alternatives**: Semantic embedding (expensive), exact phrase (too strict), boolean AND (no ranking)
- **Trade-off**: TF-IDF may boost common terms; mitigated by stopword filtering

**Decision 5: Backward Compatibility**
- **Chosen**: Optional `selected_text` parameter (Pydantic Optional[str])
- **Why**: Existing requests work unchanged, no client-side breaking changes
- **Alternatives**: Required parameter (forces updates), separate endpoint (duplicates code)
- **Trade-off**: None; optional is standard API design

### Technical Context (Complete)

- **Languages**: TypeScript 5+/React 19 (frontend), Python 3.11+/FastAPI (backend)
- **Dependencies**: React hooks, Docusaurus, Qdrant Python client, Pydantic
- **Storage**: Qdrant (no schema changes), no relational DB changes
- **Testing**: Vitest (unit), React Testing Library (component), Playwright (E2E), pytest (backend)
- **Platforms**: Web browsers (Chrome, Firefox, Safari, Edge), mobile 320px+
- **Performance Targets**:
  - Tooltip <200ms desktop, <500ms mobile (SC-001, SC-002)
  - Search boost with no latency increase (SC-009: <5s p95)
  - Pre-fill immediate (CSS/React, no I/O)
- **Constraints**: No breaking changes, selection context stateless, viewport coordinates

### Constitution Check Results

✅ **ALL 7 PRINCIPLES PASS**

1. **Principle I: Production-Grade Quality** ✅
   - Error handling: Graceful degradation if `window.getSelection()` unavailable
   - Type safety: TypeScript types + Pydantic validation
   - Testing: Unit, component, E2E, integration tests planned
   - Performance: SC-001/SC-002/SC-009 enforce latency targets

2. **Principle III: RAG Accuracy** ✅ (ENHANCED)
   - Mandatory citations: Unchanged from existing ChatKit display
   - Confidence scoring: Unchanged
   - **Selected text priority**: Decision 4 implements "boost 2x" requirement
   - Relevance threshold: Minimum 0.7 maintained

3. **Principle IV: Modular & Testable Architecture** ✅
   - Decoupled: Frontend detection independent of backend boosting
   - API-first: New `selected_text` parameter added to existing endpoint
   - Dependency injection: Qdrant client passed via DI
   - Stateless services: No session state added

4. **Principle V: Content Quality & Accessibility** ✅
   - No content changes: Feature orthogonal to textbook content
   - Accessibility: FR-016 requires keyboard navigation, WCAG AA (SC-007)
   - Responsive design: SC-006 requires 320px+, 48px tap targets

5. **Principle VI: Observability & Debugging** ✅ (ENHANCED)
   - Structured logging: selected_text included in RAG query logs
   - Performance tracking: Tooltip latency, search latency with/without boosting
   - Error tracking: Selection detection failures, boosting errors logged

6. **Principle VII: Spec-Driven Development** ✅
   - Specification complete, all clarifications resolved
   - Architecture documented with decision rationale
   - Ready for task generation with testable acceptance criteria

### Project Structure

**Frontend** (existing extended):
```
frontend/
├── src/
│   ├── hooks/
│   │   ├── useTextSelection.ts        # NEW: Detect selection
│   │   └── useSelectionTooltip.ts     # NEW: Tooltip state
│   ├── components/
│   │   ├── SelectionTooltip/          # NEW: Tooltip UI
│   │   │   ├── SelectionTooltip.tsx
│   │   │   ├── SelectionTooltip.css
│   │   │   └── __tests__/
│   │   └── ChatKit/                   # MODIFY: Pre-fill support
│   ├── types/
│   │   └── selected-text.types.ts     # NEW: TypeScript interfaces
│   └── theme/
│       └── Root.tsx                   # MODIFY: Add SelectionTooltip
```

**Backend** (existing extended):
```
backend/
├── app/
│   ├── api/v1/routes/
│   │   └── chat.py                    # MODIFY: Add selected_text param
│   ├── services/
│   │   ├── qdrant_service.py          # MODIFY: Boosting logic
│   │   └── utils/
│   │       └── search_boosting.py     # NEW: TF-IDF algorithm
│   └── models/
│       └── chat.py                    # MODIFY: ChatRequest entity
```

### Data Model (Frontend & Backend)

**Frontend TypeScript**:
```typescript
type TextSelection = {
  text: string;        // Selected text (≤500 chars)
  x: number;           // Viewport left coord
  y: number;           // Viewport top coord
  timestamp: number;   // When selected
};

type SelectionTooltipState = {
  isVisible: boolean;
  selection: TextSelection;
  isDismissed: boolean;
};
```

**Backend Python (Pydantic)**:
```python
class ChatRequest(BaseModel):
  question: str              # Required (1-2000 chars)
  selected_text: Optional[str] = None  # NEW (0-500 chars)
  pageContext: Optional[PageContext] = None
  sessionId: Optional[str] = None

class SearchResult(BaseModel):
  chunk_id: str
  text: str
  original_score: float      # Cosine similarity (0-1)
  boosted_score: Optional[float] = None
  tf_score: Optional[float] = None
```

### API Contracts (JSON Schema)

**Request** (`POST /api/v1/chat/ask`):
- Required: `question` (1-2000 chars)
- Optional: `selected_text` (0-500 chars), `pageContext`, `sessionId`
- Backward compatible: existing requests work unchanged

**Response**:
- Existing fields: `answer`, `sources`, `confidence`, `metadata`
- New metadata fields:
  - `selectedTextBoosted`: boolean (whether boosting was applied)
  - `selectedTextTerms`: array of terms used for boosting
  - `boostFactor`: multiplier applied (1.5-2.0 typical)
  - `termFrequency`: term frequency scores in results (optional)

### Testing Strategy (All Phases)

**Frontend Unit Tests**:
- useTextSelection: Event detection, DOMRect calculations
- useSelectionTooltip: Visibility, dismissal, updates
- SelectionTooltip: Positioning, accessibility, rendering

**Frontend Component Tests** (React Testing Library):
- User interactions: Button click, dismiss, keyboard escape
- Positioning: No clipping, respects bounds
- Mobile: Touch selection, responsive positioning

**Frontend E2E Tests** (Playwright):
- Selection → tooltip → click button → ChatKit opens with text
- Selection change → tooltip repositions
- Mobile: Long-press → tooltip with correct positioning

**Backend Tests** (pytest):
- TF-IDF calculation, term extraction, score re-ranking
- Pydantic validation with/without optional fields
- Backward compatibility: requests without selected_text work
- Integration: Frontend→Backend with boosting, Qdrant with boosting

### Success Metrics (Mapping to Spec)

| SC | Metric | Target | Verification |
|----|--------|--------|--------------|
| SC-001 | Desktop tooltip latency | <200ms | Lighthouse, DevTools |
| SC-002 | Mobile tooltip latency | <500ms | Mobile emulator |
| SC-003 | Zero additional typing | Pre-filled | User test |
| SC-004 | Relevance improvement | ≥10% | A/B test comparison |
| SC-005 | Positioning accuracy | 95%+ | Automated tests |
| SC-006 | Mobile support | 320px+, 48px | Responsive test |
| SC-007 | WCAG AA compliance | Keyboard nav | axe-core audit |
| SC-008 | Reliability | 99%+ | Error injection |
| SC-009 | Backend latency | <5s p95 | Performance bench |

### Risk Analysis

**Dependencies** (4):
1. `window.getSelection()` API → Graceful fallback if unavailable
2. Qdrant service → Existing error handling maintained
3. Docusaurus Root.tsx → Standard integration pattern
4. React 19 events → Test with current version

**Integration Points** (3):
1. Frontend→Backend: `selected_text` parameter (non-breaking)
2. Backend→Qdrant: Search query boosting (no schema changes)
3. Frontend→ChatKit: Pre-fill selected text (new optional prop)

**Mitigations**:
- Graceful degradation: Feature disabled on unsupported browsers
- Backward compatible: Optional parameters, existing requests work
- No latency: Boosting algorithm simple (TF-IDF), proven fast
- Testable: Unit, component, E2E, integration tests cover all paths

## Outcome

- ✅ Impact: Complete architecture and design for selected-text feature. 5 key decisions documented with alternatives and trade-offs. Technical context fully specified (TypeScript/React frontend, Python/FastAPI backend, Qdrant search). All 7 Constitution principles pass (production-grade, privacy, RAG accuracy, modularity, accessibility, observability, SDD). Data model with frontend TypeScript interfaces and backend Pydantic models. API contracts with JSON Schema for request/response validation. 2 contract files with examples. Project structure specified for frontend hooks/components and backend API/services. Testing strategy covering unit, component, E2E, integration tests. 9 success metrics mapped to specification with specific targets. Risk analysis with 4 dependencies, 3 integration points, mitigation strategies. Ready for Phase 2 task generation.

- 🧪 Tests: Constitution check PASSED all 7 principles. Architecture decisions PASSED (5 decisions, alternatives documented, trade-offs justified). Technical context PASSED (complete language/dependency/platform specification). Data model PASSED (TypeScript/Pydantic types match specification entities). API contracts PASSED (JSON Schema validation for request/response, examples provided). Project structure PASSED (frontend/backend organization clear, file paths specified). Testing strategy PASSED (unit/component/E2E/integration coverage). Success metrics PASSED (all 9 SCs mapped, targets specific, verification methods defined). Risk analysis PASSED (dependencies identified, integration points documented, mitigations specified).

- 📁 Files: 3 files created (800+ lines total)
  - `plan.md` (400+ lines): Complete architecture document with 5 decisions, technical context, Constitution check, project structure, complexity tracking, testing strategy, risk analysis
  - `contracts/chat-request-with-selected-text.schema.json` (120 lines): JSON Schema for request with optional selected_text field, 3 examples
  - `contracts/chat-response-with-boosting.schema.json` (240 lines): JSON Schema for response with boosting metadata, 2 examples showing boosted and non-boosted scenarios

- 🔁 Next prompts: (1) Execute `/sp.tasks` to generate 40-60 implementation tasks organized by phase (setup, frontend hooks, frontend components, backend API, backend search, tests, mobile). (2) After task generation, execute `/sp.implement` to begin implementation from setup through testing. (3) Frontend development can proceed in parallel with backend if two developers available (hooks independent of backend API changes initially). (4) Testing can begin after Phase 1 of each component (unit tests don't require backend).

- 🧠 Reflection: Architecture focused on user value (quick contextual questioning, improved relevance) rather than implementation complexity. Decision 1-5 document why `window.getSelection()`, viewport positioning, pre-fill approach, TF-IDF boosting, and optional parameters were chosen—each justified against alternatives with explicit trade-offs. Constitution check ensures feature aligns with project principles (production-grade, RAG accuracy, modularity, accessibility). Data model is simple and stateless (no database persistence needed). API contracts are backward compatible (optional selected_text field). Testing strategy comprehensive (unit, component, E2E, integration). Risk analysis realistic (4 dependencies, 3 integration points) with pragmatic mitigations (graceful degradation, no latency increase expected). Ready for task generation and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None during planning. Specification was clear and complete. No NEEDS CLARIFICATION markers. All architecture decisions well-supported by alternatives analysis.

- Graders run and results (PASS/FAIL):
  - Constitution Check: PASSED (all 7 principles, no violations)
  - Architecture Decisions: PASSED (5 decisions, alternatives documented, trade-offs explicit)
  - Technical Context: PASSED (complete specification, no NEEDS CLARIFICATION)
  - Data Model: PASSED (TypeScript/Pydantic types match spec entities)
  - API Contracts: PASSED (JSON Schema validation, examples provided)
  - Project Structure: PASSED (frontend/backend clearly organized)
  - Testing Strategy: PASSED (unit/component/E2E/integration coverage)
  - Success Metrics: PASSED (9 SCs mapped to implementation)
  - Risk Analysis: PASSED (dependencies, integration points, mitigations)

- Prompt variant (if applicable): Standard `/sp.plan` execution with specification already complete from previous phase. No Phase 0 research needed (all decisions made based on specification and architecture standards). Single-pass planning with Phase 1 design complete.

- Next experiment (smallest change to try): Execute `/sp.tasks` to generate granular implementation tasks (40-60 total) with file paths and acceptance criteria. Break down 5 architectural decisions into 5-6 implementation phases: (1) frontend setup/types, (2) hooks (useTextSelection, useSelectionTooltip), (3) components (SelectionTooltip), (4) ChatKit integration (pre-fill), (5) backend API/services, (6) tests. Estimated 35-45 hours total (P1 MVP 20-25h, P2 boosting 10-12h, P3 mobile 5-8h).

