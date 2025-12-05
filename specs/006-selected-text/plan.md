# Implementation Plan: Selected Text Context for ChatKit

**Branch**: `006-selected-text` | **Date**: 2025-12-06 | **Spec**: [specs/006-selected-text/spec.md](spec.md)
**Input**: Feature specification for text selection detection, tooltip UI, backend search boosting

---

## Summary

Add text selection awareness to the ChatKit widget: when users select text while reading documentation, a tooltip appears with an "Ask about this" button. Clicking opens ChatKit with selected text pre-filled, improving question relevance. The backend boosts Qdrant search results containing the selected text, making answers more contextually precise. This feature delivers P1 MVP value (quick contextual questioning) with optional P2 (search boosting) and P3 (mobile) enhancements.

**Technical approach**: Frontend hook (`useTextSelection`) detects selections via `window.getSelection()`. React component (`SelectionTooltip`) displays tooltip at selection coordinates. ChatKit widget updated to accept pre-filled selected text. Backend FastAPI endpoint enhanced to accept optional `selected_text` parameter. Qdrant search query modified to boost results containing selected text terms using relevance weighting.

---

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5+, React 19, Docusaurus 3.9
- Backend: Python 3.11+, FastAPI 0.104+

**Primary Dependencies**:
- Frontend: React (hooks), Docusaurus (Root.tsx integration), CSS Variables (dark mode)
- Backend: FastAPI, Qdrant Python client, Pydantic for validation

**Storage**:
- Vector DB: Qdrant (existing, no schema changes needed)
- Relational: None (selection context stateless, not persisted)

**Testing**:
- Frontend: Vitest (unit), React Testing Library (component), Playwright (E2E)
- Backend: pytest, with fixtures for Qdrant mock

**Target Platform**:
- Frontend: Web browsers (Chrome, Firefox, Safari, Edge 2020+), mobile responsive (320px+)
- Backend: Linux server (FastAPI ASGI)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Tooltip latency: <200ms on desktop, <500ms on mobile (FR-001, FR-002 from spec)
- Search boost: No latency increase vs. unmodified Qdrant search (SC-009: <5s p95)
- Widget pre-fill: Immediate (CSS/React state, no I/O)

**Constraints**:
- No breaking changes: `selected_text` field optional in API (backward compatible)
- Selection context not persisted (stateless)
- Tooltip positioning uses viewport coordinates (not scroll-relative)
- Graceful degradation for unsupported browsers (no `window.getSelection()`)

**Scale/Scope**:
- ~5-10K lines new code (frontend hooks, components; backend API changes)
- ~1,000 concurrent documentation readers
- ~100-500 RAG queries/minute with selected text context

---

## Constitution Check

*GATE: Must pass before Phase 1 design. Re-check after Phase 1 design.*

### ✅ Principle I: Production-Grade Quality

**Status**: PASS

- **Error Handling**: Tooltip silently disabled if `window.getSelection()` not available (FR-010); backend gracefully handles missing `selected_text` (FR-014); network errors display user-friendly messages (from ChatKit existing error handling)
- **Type Safety**: All TypeScript types defined in `types/selected-text.types.ts`; Python Pydantic models for request/response validation
- **Testing**: Unit tests for selection detection, tooltip positioning, search boosting logic; component tests for SelectionTooltip; E2E tests for user workflows
- **Graceful Degradation**: Feature disabled on IE11 or browsers without `window.getSelection()`; existing ChatKit functionality unaffected if selection detection fails
- **Performance**: SC-001/SC-002 enforce <200ms desktop, <500ms mobile tooltip latency; SC-009 enforces <5s p95 backend latency with boosting

### ✅ Principle III: RAG Accuracy & Source Citation

**Status**: PASS (ENHANCED)

- **Mandatory Source Citations**: Selected text boosting improves source relevance (SC-004: ≥10% improvement); existing ChatKit citation display unaffected
- **Confidence Scoring**: Backend uses cosine similarity + selected text term presence for ranking; low-confidence results unchanged in display
- **Selected Text Priority**: FR-013 explicitly boosts Qdrant results containing selected_text (constitution requirement "boost selected chunk relevance by 2x")
- **Hallucination Detection**: Selected text boosting doesn't change answer generation; verification against chunks unchanged
- **Relevance Threshold**: Minimum similarity 0.7 maintained; selected text boosting re-ranks within existing threshold
- **Context Window**: No changes to chunk composition; selected text only affects ranking

### ✅ Principle IV: Modular & Testable Architecture

**Status**: PASS

- **Decoupled Services**: Frontend selection detection independent of backend; backend Qdrant boosting independent of ChatKit logic
- **API-First Design**: New `selected_text` parameter added to existing `/api/v1/chat/ask` endpoint (FR-011, FR-012)
- **Dependency Injection**: Backend Qdrant client passed via DI; frontend `SelectionTooltip` accepts callbacks for testing
- **Contract Testing**: OpenAPI spec updated for `selected_text` parameter (optional field); Pydantic models validate request structure
- **Integration Testing**: Frontend→Backend: test question with selected_text; Backend→Qdrant: test search boosting without breaking existing queries
- **Stateless Services**: No session state added; selection context only in current request lifecycle

### ✅ Principle V: Content Quality & Accessibility

**Status**: PASS

- **Content Review**: No changes to content; selection detection/boosting doesn't modify source material
- **Accessibility**: SC-007 requires WCAG AA compliance (FR-016 requires keyboard navigation, color contrast); tooltip must be keyboard accessible
- **Responsive Design**: SC-006 requires mobile support (320px+, 48px tap targets); tooltip positioning responsive to viewport
- **Progressive Disclosure**: No changes to content complexity; selection feature orthogonal to content progression

### ✅ Principle VI: Observability & Debugging

**Status**: PASS (ENHANCED)

- **Structured Logging**: Log selected_text in RAG query logs (what user selected, how boosting affected ranking)
- **RAG Query Logging**: Include selected_text in query metadata; track if boosting improved relevance (A/B comparison)
- **Performance Metrics**: Track tooltip latency (SC-001/SC-002), search latency with/without boosting (SC-009)
- **Error Tracking**: Log selection detection failures (graceful fallback), Qdrant boosting errors, request validation errors

### ✅ Principle VII: Spec-Driven Development

**Status**: PASS

- Specification complete with 3 prioritized user stories (P1/P2/P3), 18 FRs, 9 SCs, 9 edge cases
- Architecture decisions documented below
- Data model and contracts generated in Phase 1
- Tasks will be generated with testable acceptance criteria

**GATE VERDICT**: ✅ ALL PRINCIPLES PASS - Ready for Phase 1 design

---

## Project Structure

### Documentation (this feature)

```
specs/006-selected-text/
├── spec.md                  # Feature specification (DONE)
├── plan.md                  # This file (architecture & design)
├── research.md              # Phase 0: Research decisions (TBD if needed)
├── data-model.md            # Phase 1: Entity definitions
├── contracts/               # Phase 1: API request/response schemas
│   ├── selected-text-request.schema.json
│   └── selected-text-response.schema.json
├── quickstart.md            # Phase 1: Developer implementation guide
├── checklists/
│   └── requirements.md      # Quality validation (DONE)
└── tasks.md                 # Phase 2: Implementation tasks (generated by /sp.tasks)
```

### Source Code (repository root)

**Frontend** (existing ChatKit structure extended):

```
frontend/
├── src/
│   ├── hooks/
│   │   ├── useTextSelection.ts        # NEW: Detect text selection
│   │   └── useSelectionTooltip.ts     # NEW: Tooltip visibility/positioning
│   ├── components/
│   │   ├── SelectionTooltip/          # NEW: Tooltip UI component
│   │   │   ├── SelectionTooltip.tsx
│   │   │   ├── SelectionTooltip.css
│   │   │   └── __tests__/
│   │   │       ├── SelectionTooltip.test.tsx
│   │   │       ├── positioning.test.ts
│   │   │       └── accessibility.test.ts
│   │   └── ChatKit/                   # MODIFY: Add selected text support
│   │       ├── ChatKitWidget.tsx      # Add pre-filled text logic
│   │       └── __tests__/
│   │           └── ChatKitWidget.test.tsx
│   ├── types/
│   │   └── selected-text.types.ts     # NEW: TypeScript interfaces
│   ├── theme/
│   │   └── Root.tsx                   # MODIFY: Add SelectionTooltip
│   └── styles/
│       └── selection-tooltip.css      # NEW: Tooltip styles
├── tests/
│   ├── e2e/
│   │   └── selected-text.spec.ts      # NEW: E2E tests (Playwright)
│   └── integration/
│       └── selected-text.integration.ts # NEW: Integration tests
```

**Backend** (existing RAG structure extended):

```
backend/
├── app/
│   ├── api/
│   │   ├── v1/
│   │   │   └── routes/
│   │   │       ├── chat.py            # MODIFY: Add selected_text parameter
│   │   │       └── __tests__/
│   │   │           └── test_chat_selected_text.py
│   │   └── schemas/
│   │       ├── chat_request.py        # MODIFY: Add selected_text field
│   │       └── chat_response.py       # Existing (no changes needed)
│   ├── services/
│   │   ├── qdrant_service.py          # MODIFY: Add search boosting logic
│   │   ├── __tests__/
│   │   │   └── test_qdrant_boosting.py
│   │   └── utils/
│   │       └── search_boosting.py     # NEW: Boost algorithm (extract terms, weight by frequency)
│   └── models/
│       └── chat.py                    # MODIFY: Add selected_text to ChatRequest
```

**Structure Decision**: Web application with decoupled frontend and backend. Frontend hooks/components handle selection detection and tooltip UI. Backend FastAPI endpoint enhanced to accept optional `selected_text` parameter. Qdrant service modified to apply search boosting when `selected_text` present. No database schema changes (selection context stateless).

---

## Key Architectural Decisions

### Decision 1: Text Selection Detection Method

**Chosen**: `window.getSelection()` API with mouseup/touchend event listeners

**Rationale**:
- Native browser API, no dependencies
- Works across all modern browsers (Chrome 1+, Firefox 3.6+, Safari 2+, Edge all versions)
- Supports both mouse and touch selections
- Returns DOMRect for positioning calculations
- Graceful fallback: if not available, simply don't display tooltip

**Alternatives Considered**:
- Custom text selection library (e.g., `rangy`): Adds dependency, unnecessary complexity
- Keyboard event tracking (Ctrl+C): Doesn't capture all selections, misses touch
- Triple-click + drag detection: Fragile, unreliable

**Trade-offs**: Older browsers (IE < 9) won't see tooltip, but they're < 0.5% of traffic; acceptable tradeoff per Constitution

---

### Decision 2: Tooltip Positioning Strategy

**Chosen**: Viewport-relative positioning with dynamic repositioning (above selection if near bottom)

**Rationale**:
- Tooltip follows selection visually (user sees context)
- Avoids clipping at viewport edges
- Responsive on mobile (repositions as user scrolls)
- CSS position:absolute on main Root wrapper

**Alternatives Considered**:
- Fixed viewport position (always top-right): User loses visual connection to selection
- Document-relative (scroll with page): Complex calculations, tooltip hidden when selection scrolls out
- Shadow DOM tooltip: Harder to style consistently

**Trade-offs**: Requires calculating bounding rects on every selection; <5ms overhead acceptable

---

### Decision 3: Selected Text Integration with ChatKit Widget

**Chosen**: Pre-fill input field + optional parameter in RAGRequest

**Rationale**:
- User sees their selected text in input
- Simple to understand (expected behavior)
- Selected text stored only for current request (stateless)
- ChatKit widget unchanged; feature is additive

**Alternatives Considered**:
- Hidden field approach (append to question): Confusing UI, selected text not visible
- Replace input with selected text: Breaks use case of combining selection with additional context
- Store in session state: Adds complexity, not needed

**Trade-offs**: Requires ChatKit widget modification; small change (one new optional prop)

---

### Decision 4: Backend Search Boosting Algorithm

**Chosen**: TF-IDF term weighting + cosine similarity re-ranking

**Rationale**:
- Simple, proven algorithm for relevance boosting
- No LLM inference needed (fast, no cost)
- Works with Qdrant's existing similarity search
- Per Constitution Principle III: "boost selected chunk relevance by 2x"

**Implementation**:
1. Parse `selected_text` into terms (tokenize, remove stopwords)
2. For each search result chunk, calculate term frequency (TF) of selected text terms
3. Boost cosine similarity score: `boosted_score = original_score * (1 + TF_weight * boost_factor)`
4. Re-sort results by boosted score
5. Return top-K results

**Alternatives Considered**:
- Semantic similarity via embedding (LLM-based): Expensive, latency increases
- Exact phrase matching: Too strict, doesn't handle variations
- Simple boolean AND filter: Loses ranking, may return no results if no exact match

**Trade-offs**: TF-IDF simple but may boost on common terms (e.g., "the"); mitigated by stopword filtering

---

### Decision 5: Backward Compatibility

**Chosen**: Optional `selected_text` parameter (Pydantic `Optional[str]`)

**Rationale**:
- Existing requests (without `selected_text`) work unchanged
- Servers can be updated without client-side breakage
- Future clients can benefit from improved search without server upgrade
- No database migrations needed

**Alternatives Considered**:
- Required parameter: Forces client updates, breaks existing code
- Separate endpoint: Duplicates code, confusing API

**Trade-offs**: None; optional is standard API design

---

## Data Model

### Frontend Entities

**TextSelection**:
```typescript
{
  text: string;              // Selected text (up to 500 chars)
  x: number;                 // Left coordinate (viewport)
  y: number;                 // Top coordinate (viewport)
  timestamp: number;         // When selected (ms since epoch)
}
```

**SelectionTooltipState**:
```typescript
{
  isVisible: boolean;        // Tooltip shown
  selection: TextSelection;  // Current selection
  isDismissed: boolean;      // User clicked dismiss (don't show again for selection)
}
```

### Backend Entities

**ChatRequest** (extended):
```python
{
  question: str;             # User question (1-2000 chars)
  selected_text?: str;       # Highlighted text context (0-500 chars, optional)
  pageContext?: PageContext; # Existing field
  sessionId?: str;           # Existing field
}
```

**SearchResult** (internal):
```python
{
  chunk_id: str;
  text: str;
  original_score: float;     # Cosine similarity (0-1)
  boosted_score?: float;     # Score after selected_text boosting (0-1)
  tf_score?: float;          # Term frequency of selected_text in chunk
}
```

---

## API Contracts

### Request Schema

**POST** `/api/v1/chat/ask`

```json
{
  "question": "What is forward kinematics?",
  "selected_text": "forward kinematics",  // OPTIONAL (new field)
  "pageContext": {
    "url": "http://localhost:3000/docs/chapter-3",
    "pathname": "/docs/chapter-3",
    "chapter": "Chapter 3",
    "section": "Kinematics",
    "confidence": "high"
  },
  "sessionId": "uuid-1234-5678"
}
```

**Validation Rules**:
- `question`: Required, 1-2000 characters
- `selected_text`: Optional, 0-500 characters, must be string (no array/object)
- Other fields: Existing validation unchanged

### Response Schema

```json
{
  "answer": "Forward kinematics is the process of determining the position and orientation of the end-effector given joint angles. See Chapter 3, Section 3.2 for details.",
  "sources": [
    {
      "id": "chunk-3-2-1",
      "title": "Chapter 3, Section 3.2 - Forward Kinematics",
      "snippet": "Forward kinematics is the process of...",
      "url": "http://localhost:3000/docs/chapter-3#forward-kinematics",
      "similarity": 0.92
    }
  ],
  "confidence": 0.87,
  "metadata": {
    "searchLatencyMs": 145,
    "generationLatencyMs": 1230,
    "totalLatencyMs": 1375,
    "chunksRetrieved": 5,
    "chunksUsed": 3,
    "model": "gemini-1.5-flash",
    "selectedTextBoosted": true,  // NEW: indicates if selected_text boosting was applied
    "selectedTextTerms": ["forward", "kinematics"],  // NEW: which terms matched
    "boostFactor": 1.5  // NEW: how much scores were boosted
  }
}
```

---

## Testing Strategy

### Frontend Unit Tests (Vitest)

- **useTextSelection hook**: Selection detection, DOMRect calculations, event cleanup
- **useSelectionTooltip hook**: Visibility state, dismissal logic, update on new selection
- **SelectionTooltip component**: Positioning calculations, accessibility (keyboard, ARIA), rendering
- **ChatKitWidget modified**: Pre-filled text display, integration with selected text

### Frontend Component Tests (React Testing Library)

- **SelectionTooltip**: User interactions (click button, dismiss, keyboard Escape)
- **Positioning accuracy**: Tooltip doesn't overlap selection, respects viewport bounds
- **Mobile interactions**: Touch selection, responsive positioning on small screens

### Frontend E2E Tests (Playwright)

- User selects text → tooltip appears → clicks button → ChatKit opens with text pre-filled
- Selection changes → tooltip updates position
- Mobile test: Long-press selection → tooltip appears with correct positioning

### Backend Unit Tests (pytest)

- **Search boosting**: TF-IDF calculation, term extraction, score re-ranking
- **Pydantic validation**: Request/response schemas with/without optional fields
- **Backward compatibility**: Requests without `selected_text` processed normally

### Backend Integration Tests

- Frontend → Backend: POST with `selected_text` parameter → backend returns boosted results
- Qdrant → Backend: Search with boosting applied → results ranked correctly
- Error handling: `selected_text` too long, invalid format, Qdrant unavailable

---

## Success Metrics (from Specification)

| Metric | Target | How Verified |
|--------|--------|-------------|
| SC-001: Desktop tooltip latency | <200ms | Lighthouse, browser DevTools timing |
| SC-002: Mobile tooltip latency | <500ms | Mobile emulator, performance profiler |
| SC-003: Zero additional typing | Pre-filled input | Manual user test |
| SC-004: Relevance improvement | ≥10% higher with context | A/B test comparison, relevance metrics |
| SC-005: Tooltip positioning | 95%+ accuracy (no clipping) | Automated tests, screenshot comparison |
| SC-006: Mobile support | 320px+, 48px tap targets | Responsive design test, mobile emulator |
| SC-007: WCAG AA compliance | Keyboard accessible, contrast OK | axe-core accessibility audit |
| SC-008: 99%+ reliability | Graceful degradation on errors | Error injection tests |
| SC-009: Backend latency | <5s p95 with boosting | Performance benchmarking |

---

## Dependencies & Risks

### External Dependencies

| Dependency | Risk | Mitigation |
|-----------|------|-----------|
| `window.getSelection()` API | Not available in IE < 9 | Graceful fallback: check availability, disable tooltip if not present |
| Qdrant vector search | Service downtime/errors | Existing error handling; boosting doesn't change fallback behavior |
| Docusaurus Root.tsx | Breaking changes | Standard integration pattern, no Docusaurus modifications needed |
| React 19 event system | Event bubbling changes | Test with current React version, verify mouseup/touchend triggered |

### Integration Points

1. **Frontend → Backend**: `selected_text` parameter in existing `/api/v1/chat/ask` endpoint (non-breaking)
2. **Backend → Qdrant**: Search query modified to boost selected text terms (no schema changes)
3. **Frontend → ChatKit**: Widget accepts pre-filled selected text (new optional prop)

### Risk Mitigation

- **Graceful degradation**: Feature disabled on unsupported browsers; existing ChatKit unaffected
- **No breaking changes**: Optional parameters, existing requests work unchanged
- **Backward compatible**: Old clients work with new server; new clients work with old server
- **Performance**: Boosting algorithm simple (TF-IDF), no latency impact expected

---

## Complexity Tracking

**No Constitution violations**; all principles pass. Feature is:
- **Type-safe**: Full TypeScript frontend, Pydantic validation backend
- **Error-handled**: Graceful fallback for unsupported browsers, validation on all inputs
- **Tested**: Unit, component, E2E, integration tests planned
- **Accessible**: WCAG AA required (FR-016, SC-007)
- **Production-ready**: No unproven dependencies, simple algorithms, proven patterns

---

## Next Steps

1. **Phase 0 (Optional)**: If any NEEDS CLARIFICATION markers detected, execute research
2. **Phase 1 (Design)**: Generate data-model.md, contracts/, quickstart.md
3. **Phase 2 (Tasks)**: Run `/sp.tasks` to generate implementation tasks with testable criteria
4. **Phase 3 (Implementation)**: Execute tasks in dependency order (frontend hooks → component → backend → tests)

**Estimated Effort**:
- Phase 1 MVP (P1): 20-25 hours (selection detection, tooltip, ChatKit integration)
- Phase 2 Boosting (P2): 10-12 hours (backend Qdrant boosting, testing)
- Phase 3 Mobile (P3): 5-8 hours (mobile positioning, touch selection)
- Total: 35-45 hours (2-3 weeks with one developer)

