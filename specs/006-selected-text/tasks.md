# Implementation Tasks: Selected Text Context for ChatKit

**Feature**: 006-selected-text | **Branch**: `006-selected-text` | **Date**: 2025-12-06
**Total Tasks**: 52 | **Estimated Effort**: 35-45 hours | **MVP Scope**: Phase 1-2 (20-25 hours)

---

## Quick Reference

**User Stories**:
- **P1 (MVP)**: Text selection → tooltip → "Ask about this" → ChatKit opens with pre-filled text
- **P2 (Enhancement)**: Backend boosts Qdrant search results containing selected text
- **P3 (Mobile)**: Mobile-optimized selection experience (long-press, responsive positioning)

**Implementation Strategy**:
1. Phase 1 (Setup): Types, directory structure, base configurations
2. Phase 2 (Foundational): Hooks and core utilities (independent, used by all stories)
3. Phase 3 (US1): Frontend selection detection, tooltip component, ChatKit integration
4. Phase 4 (US2): Backend API changes, Qdrant search boosting
5. Phase 5 (US3): Mobile optimizations and touch event handling
6. Phase 6 (Testing): Full test coverage across all phases
7. Phase 7 (Polish): Accessibility audit, performance optimization

**Parallel Opportunities**:
- Frontend (Phase 2-3) can proceed in parallel with Backend (Phase 4)
- US1 (selection + tooltip) is independent MVP; doesn't require US2 (boosting)
- Unit tests can proceed immediately after components are implemented

**Dependencies**:
- Phase 1 BLOCKS all other phases (setup required)
- Phase 2 BLOCKS Phase 3 (hooks required for components)
- Phase 3 and Phase 4 are INDEPENDENT (can run in parallel)
- Phase 5 depends on Phase 3 (mobile enhancements to existing components)
- Phase 6 can start after Phase 2 (unit tests) or Phase 3 (component tests)

---

## Phase 1: Setup & Type Definitions

**Goal**: Initialize project structure, TypeScript types, and foundational utilities

**Independent Test Criteria**:
- TypeScript compilation succeeds with no errors
- Directory structure matches planned layout
- All types compile and are importable

**Tasks**:

- [x] T001 Create TypeScript types file at `frontend/src/types/selected-text.types.ts` with TextSelection, SelectionTooltipState, and SelectionCoordinates interfaces
- [x] T002 Create directory structure: `frontend/src/hooks/`, `frontend/src/components/SelectionTooltip/`, `frontend/src/components/SelectionTooltip/__tests__/`
- [x] T003 Create backend directory structure: `backend/app/services/utils/`, `backend/app/api/schemas/`
- [x] T004 Create test directories: `frontend/tests/e2e/`, `frontend/tests/integration/`, `backend/app/api/routes/__tests__/`
- [x] T005 Create Pydantic model file at `backend/app/models/chat.py` with ChatRequest including optional `selected_text` field
- [x] T006 Create backend schema file at `backend/app/api/schemas/chat_request.py` with request validation (question, selected_text, pageContext, sessionId)
- [x] T007 Create utility types file at `frontend/src/utils/selection.utils.ts` with helper functions for text validation and coordinate calculations
- [x] T008 Create constants file at `frontend/src/constants/selection.constants.ts` with MAX_SELECTED_TEXT_LENGTH=500, TOOLTIP_DISPLAY_DELAY_MS=0, DEBOUNCE_DELAY_MS=50

---

## Phase 2: Foundational Hooks & Utilities

**Goal**: Implement reusable hooks and utility functions that all user stories depend on

**Independent Test Criteria**:
- `useTextSelection` hook detects selections and returns text + coordinates
- `useSelectionTooltip` hook manages tooltip visibility state
- Search boosting utility functions (TF-IDF) work correctly in isolation
- Backend utility functions validate and process selected_text

**Parallelizable Tasks** (Frontend and Backend can run simultaneously):

**Frontend Hooks**:

- [x] T009 [P] Implement `frontend/src/hooks/useTextSelection.ts` hook to:
  - Listen to mouseup and touchend events on document
  - Call window.getSelection() to get selected text
  - Calculate DOMRect coordinates for selected range
  - Return {text, x, y, timestamp} or null if no selection
  - Include error handling for unsupported browsers

- [x] T010 [P] Implement `frontend/src/hooks/useSelectionTooltip.ts` hook to:
  - Manage tooltip visibility state (isVisible, isDismissed)
  - Handle tooltip dismissal (Escape key, click outside, scroll)
  - Debounce selection updates (50ms)
  - Return tooltip state and handlers (show, hide, dismiss)

- [x] T011 [P] Implement selection validation utility at `frontend/src/utils/selection.utils.ts`:
  - `validateSelection(text: string): boolean` - check not empty/whitespace
  - `truncateSelection(text: string, maxLength: number): string` - trim long selections
  - `normalizeText(text: string): string` - clean whitespace

- [x] T012 [P] Implement tooltip positioning utility at `frontend/src/utils/positioning.utils.ts`:
  - `calculateTooltipPosition(rect: DOMRect, viewportHeight: number): {x, y}` - position above/below selection
  - `isTooltipVisible(x: number, y: number, viewportWidth: number, viewportHeight: number): boolean` - check clipping
  - `adjustTooltipForViewport(x: number, y: number, tooltipWidth: number, tooltipHeight: number): {x, y}` - reposition if clipped

**Backend Utilities**:

- [x] T013 [P] Implement TF-IDF utilities at `backend/app/services/utils/search_boosting.py`:
  - `extract_terms(text: str) -> List[str]` - tokenize and remove stopwords
  - `calculate_term_frequency(terms: List[str], text: str) -> Dict[str, float]` - compute TF scores
  - `apply_boost_factor(score: float, tf_weight: float, boost_factor: float = 1.5) -> float` - apply boost to cosine similarity

- [x] T014 [P] Implement request validation utility at `backend/app/services/utils/validation.py`:
  - `validate_selected_text(text: Optional[str]) -> bool` - check length (0-500), encoding
  - `validate_question(question: str) -> bool` - check length (1-2000)
  - `normalize_selected_text(text: Optional[str]) -> Optional[str]` - strip whitespace

- [ ] T015 [P] Implement Qdrant mock for testing at `backend/tests/fixtures/qdrant_mock.py`:
  - Mock Qdrant client with search method that returns vector results
  - Include fixture for seeded results with known similarity scores

---

## Phase 3: User Story 1 - Selection Detection & Tooltip (P1 MVP)

**Goal**: Implement core user-facing feature - detect text selection and display tooltip with "Ask about this" button

**Story**: User selects text → tooltip appears → clicks button → ChatKit opens with pre-filled text

**Independent Test Criteria**:
- Tooltip appears within 200ms of text selection (desktop)
- Tooltip positions correctly without clipping
- Clicking "Ask about this" button passes selected text to ChatKit
- Dismiss button closes tooltip
- Escape key dismisses tooltip
- No tooltip appears for empty/whitespace selection

**Parallelizable Tasks**:

- [ ] T016 [P] [US1] Implement SelectionTooltip component at `frontend/src/components/SelectionTooltip/SelectionTooltip.tsx`:
  - Display at (x, y) coordinates
  - Show "💬 Ask about this" button and dismiss (×) button
  - Accept onAsk and onDismiss callbacks
  - Show truncated preview of selected text (max 50 chars)
  - Accept isVisible prop to control visibility

- [ ] T017 [P] [US1] Create SelectionTooltip styles at `frontend/src/components/SelectionTooltip/SelectionTooltip.css`:
  - Position absolute on viewport (z-index 1000)
  - Min width 200px, max width 350px
  - Button styling with 48px minimum tap target
  - Dark mode support with CSS variables
  - Mobile responsive (max 90% viewport width on narrow screens)

- [ ] T018 [P] [US1] Implement SelectionTooltip accessibility features:
  - Keyboard focus management (Tab to cycle through buttons)
  - ARIA labels for buttons
  - Color contrast ratio ≥4.5:1 per WCAG AA
  - Dismiss on Escape key

- [ ] T019 [US1] Integrate SelectionTooltip into Root.tsx at `frontend/src/theme/Root.tsx`:
  - Import useTextSelection and useSelectionTooltip hooks
  - Render SelectionTooltip component at root level
  - Pass tooltip state and handlers to SelectionTooltip

- [ ] T020 [US1] Enhance ChatKitWidget to accept pre-filled selected text at `frontend/src/components/ChatKit/ChatKitWidget.tsx`:
  - Accept `selectedText` prop (optional)
  - If selectedText provided, pre-fill input field with it
  - Handle prefilling without triggering auto-send
  - Preserve selectedText in request payload to backend

- [ ] T021 [US1] Integrate selection tooltip with ChatKit in Root.tsx:
  - When "Ask about this" button clicked, call ChatKit's setSelectedText method
  - Open ChatKit widget and focus input field
  - Tooltip dismisses after button click

- [ ] T022 [US1] Implement selection tooltip dismissal logic in `useSelectionTooltip` hook:
  - Dismiss when Escape key pressed (add keydown listener)
  - Dismiss when clicking outside tooltip bounds
  - Dismiss when user scrolls
  - Dismiss after 5 seconds of inactivity (optional)

---

## Phase 4: User Story 2 - Search Result Boosting (P2 Enhancement)

**Goal**: Enhance backend to boost Qdrant search results that contain selected text terms

**Story**: Backend receives selected_text → boosts relevant results in ranking → user sees more contextual answers

**Independent Test Criteria**:
- Selected text terms are extracted correctly
- TF-IDF weights are calculated accurately
- Qdrant results are re-ranked with boosted scores
- Backward compatible: requests without selected_text work unchanged
- Performance: Search with boosting <5s p95 latency

**Parallelizable Tasks** (can run in parallel with Phase 3):

- [ ] T023 [P] [US2] Modify FastAPI chat endpoint at `backend/app/api/v1/routes/chat.py`:
  - Accept optional `selected_text` parameter in POST /api/v1/chat/ask
  - Pass selected_text to Qdrant search service
  - Include selectedTextBoosted and boostFactor in response metadata

- [ ] T024 [P] [US2] Enhance Qdrant service at `backend/app/services/qdrant_service.py`:
  - Add parameter `selected_text: Optional[str]` to search method
  - If selected_text provided, extract terms and calculate boost weights
  - Apply boost to cosine similarity scores before ranking
  - Return original_score and boosted_score in results
  - Handle edge case: selected_text with no matching terms (fall back to standard search)

- [ ] T025 [P] [US2] Implement search boosting algorithm in `backend/app/services/utils/search_boosting.py`:
  - `boost_search_results(results: List[SearchResult], selected_text: str, boost_factor: float = 1.5) -> List[SearchResult]`
  - Extract unique terms from selected_text
  - For each result, calculate term frequency (TF) of boost terms
  - Apply boost factor: `boosted_score = original_score * (1 + TF_weight * (boost_factor - 1))`
  - Re-sort results by boosted score
  - Return results with original_score and boosted_score fields

- [ ] T026 [P] [US2] Create response model enhancement at `backend/app/models/chat.py`:
  - Add metadata fields: selectedTextBoosted (bool), selectedTextTerms (List[str]), boostFactor (float), termFrequency (Dict[str, float])
  - Include optional tf_score in SearchResult model

- [ ] T027 [US2] Add request validation for selected_text in `backend/app/api/v1/routes/chat.py`:
  - Validate selected_text length (0-500 chars)
  - Check for non-ASCII characters (should work, not error)
  - Return 400 Bad Request with clear error message if validation fails

- [ ] T028 [US2] Implement graceful fallback in Qdrant service:
  - If selected_text provided but no chunks match, return standard search results
  - Log when boosting is applied vs. skipped
  - Set selectedTextBoosted=false in metadata if no boost applied

- [ ] T029 [US2] Create integration test for search boosting at `backend/app/services/__tests__/test_qdrant_boosting.py`:
  - Mock Qdrant with 5 test chunks (2 containing "forward kinematics", 3 other topics)
  - Search with selected_text="forward kinematics"
  - Verify chunks containing "forward kinematics" appear higher in results
  - Verify backward compatibility: search without selected_text works

---

## Phase 5: User Story 3 - Mobile Optimization (P3 Enhancements)

**Goal**: Optimize selection experience for mobile/touch devices

**Story**: Mobile user long-presses to select text → tooltip appears → button is easily tappable → remains visible during scroll

**Independent Test Criteria**:
- Touch selection (long-press) triggers tooltip
- Tooltip repositions if near viewport bottom
- Tooltip width adapts to narrow viewports (<640px)
- Touch targets are 48px minimum
- Tooltip remains visible during page scroll
- Feature works on iOS Safari and Android Chrome

**Parallelizable Tasks** (depends on Phase 3 completion):

- [ ] T030 [P] [US3] Enhance useTextSelection hook for touch events at `frontend/src/hooks/useTextSelection.ts`:
  - Add touchend event listener (already in place, verify it fires)
  - Handle touch selection coordinates (use getClientBoundingRect from touch event)
  - Test with long-press on mobile emulator

- [ ] T031 [P] [US3] Enhance tooltip positioning for mobile at `frontend/src/utils/positioning.utils.ts`:
  - Add mobile viewport detection (window.matchMedia('(max-width: 768px)'))
  - Adjust tooltip position if near bottom (reposition above selection)
  - Constrain width to 90% of viewport on narrow screens
  - Account for mobile statusbar and notch height (if needed)

- [ ] T032 [P] [US3] Update SelectionTooltip styles for mobile at `frontend/src/components/SelectionTooltip/SelectionTooltip.css`:
  - Button height minimum 48px (WCAG compliance)
  - Button width minimum 48px on mobile
  - Padding adjusted for touch (12px padding minimum)
  - Font size ≥14px for readability on mobile
  - Media query for <640px viewport

- [ ] T033 [US3] Handle scroll while tooltip visible at `frontend/src/hooks/useSelectionTooltip.ts`:
  - Listen for scroll events on window
  - Update tooltip position during scroll (if selection still visible)
  - Dismiss tooltip if selection scrolls out of view

- [ ] T034 [US3] Test mobile experience at `frontend/tests/e2e/selected-text-mobile.spec.ts`:
  - Emulate iPhone 12 Pro viewport (390px width)
  - Test long-press selection and tooltip display
  - Verify button is tappable (48px)
  - Test scroll with tooltip visible

- [ ] T035 [US3] Add touch event handling for ChatKit pre-fill:
  - Ensure touch tapping "Ask about this" button works reliably
  - No accidental double-taps or mis-taps
  - Test on both iOS and Android

---

## Phase 6: Testing & Quality Assurance

**Goal**: Comprehensive test coverage for all user stories and edge cases

**Independent Test Criteria**:
- Unit tests: 90%+ coverage of hooks and utilities
- Component tests: All SelectionTooltip interactions tested
- E2E tests: Full user workflows tested on desktop and mobile
- Integration tests: Frontend→Backend→Qdrant pipeline tested
- All edge cases from spec handled gracefully

**Tasks**:

- [ ] T036 [P] Create unit tests for useTextSelection hook at `frontend/src/hooks/__tests__/useTextSelection.test.ts`:
  - Test selection detection via mouseup event
  - Test selection detection via touchend event
  - Test DOMRect calculation for tooltip positioning
  - Test empty selection (returns null)
  - Test whitespace-only selection (returns null)
  - Test graceful fallback if window.getSelection not available

- [ ] T037 [P] Create unit tests for useSelectionTooltip hook at `frontend/src/hooks/__tests__/useSelectionTooltip.test.ts`:
  - Test tooltip visibility state changes
  - Test dismissal via Escape key
  - Test dismissal via click outside
  - Test dismissal via scroll
  - Test debouncing of selection updates

- [ ] T038 [P] Create unit tests for selection utilities at `frontend/src/utils/__tests__/selection.utils.test.ts`:
  - Test validateSelection (empty, whitespace, valid)
  - Test truncateSelection (normal, long, edge cases)
  - Test normalizeText (whitespace handling, special chars)

- [ ] T039 [P] Create unit tests for positioning utilities at `frontend/src/utils/__tests__/positioning.utils.test.ts`:
  - Test calculateTooltipPosition (above, below, centered)
  - Test adjustTooltipForViewport (clipping detection, repositioning)
  - Test mobile viewport calculations

- [ ] T040 [P] Create component tests for SelectionTooltip at `frontend/src/components/SelectionTooltip/__tests__/SelectionTooltip.test.tsx`:
  - Test rendering with selected text
  - Test button click triggers onAsk callback
  - Test dismiss button triggers onDismiss callback
  - Test Escape key dismisses
  - Test keyboard focus management
  - Test ARIA labels

- [ ] T041 [P] Create accessibility tests for SelectionTooltip at `frontend/src/components/SelectionTooltip/__tests__/accessibility.test.ts`:
  - Test color contrast (≥4.5:1)
  - Test keyboard navigation (Tab key)
  - Test focus indicators visible
  - Test ARIA labels descriptive

- [ ] T042 Create backend unit tests for TF-IDF at `backend/app/services/__tests__/test_search_boosting.py`:
  - Test term extraction (tokenization, stopword removal)
  - Test TF calculation
  - Test boost factor application
  - Test edge cases (empty terms, single character, special chars)

- [ ] T043 Create backend unit tests for request validation at `backend/app/services/__tests__/test_validation.py`:
  - Test selected_text length validation (0, 500, 501 chars)
  - Test special characters and non-ASCII handling
  - Test question validation
  - Test normalization (whitespace trimming)

- [ ] T044 Create backend unit tests for chat endpoint at `backend/app/api/v1/routes/__tests__/test_chat_selected_text.py`:
  - Test POST /api/v1/chat/ask with selected_text parameter
  - Test POST /api/v1/chat/ask without selected_text (backward compatible)
  - Test selected_text validation and error responses
  - Test response includes selectedTextBoosted and boostFactor metadata

- [ ] T045 Create E2E test for selection → tooltip → ChatKit at `frontend/tests/e2e/selected-text.spec.ts`:
  - Navigate to documentation page
  - Select text via mouse drag
  - Verify tooltip appears within 200ms
  - Click "Ask about this" button
  - Verify ChatKit widget opens
  - Verify selected text is pre-filled in input
  - Submit question and verify response

- [ ] T046 Create E2E test for mobile selection at `frontend/tests/e2e/selected-text-mobile.spec.ts`:
  - Emulate mobile viewport (390px)
  - Perform long-press selection
  - Verify tooltip appears within 500ms
  - Verify button is 48px+ (tappable)
  - Click button and verify ChatKit opens
  - Test scroll with tooltip visible

- [ ] T047 Create integration test for frontend→backend at `frontend/tests/integration/selected-text.integration.ts`:
  - Select text on page
  - Click "Ask about this"
  - Verify request to backend includes selected_text
  - Verify response includes selectedTextBoosted metadata

- [ ] T048 Create integration test for backend→Qdrant at `backend/tests/integration/test_qdrant_selected_text.py`:
  - Send request with selected_text to chat endpoint
  - Verify Qdrant service applies boosting
  - Verify results are re-ranked correctly
  - Verify backward compatibility (no selected_text still works)

- [ ] T049 Create edge case test for long selections at `frontend/tests/e2e/selected-text-edge-cases.spec.ts`:
  - Test selection >500 chars (should truncate in tooltip, full text to backend)
  - Test selection with special characters/non-ASCII
  - Test selection with formatting (if applicable)
  - Test re-selection of identical text (tooltip repositions)

- [ ] T050 Create performance benchmark test at `frontend/tests/performance/selected-text-latency.spec.ts`:
  - Measure tooltip latency (should be <200ms desktop, <500ms mobile)
  - Measure selection update latency
  - Measure ChatKit pre-fill latency
  - Report p50, p95, p99 latencies

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Accessibility audit, performance optimization, documentation

**Independent Test Criteria**:
- WCAG AA compliance verified with accessibility audit tool (axe-core)
- All latency targets met: SC-001 (<200ms), SC-002 (<500ms), SC-009 (<5s p95)
- Code documented with JSDoc/docstrings
- Responsive design verified on 320px-1440px viewports

**Tasks**:

- [ ] T051 Run accessibility audit with axe-core at `frontend/tests/a11y/selected-text.a11y.test.ts`:
  - Verify WCAG AA compliance (Level AA)
  - Check color contrast
  - Check keyboard navigation
  - Check focus indicators
  - Check ARIA labels
  - Report any violations

- [ ] T052 Performance optimization & monitoring:
  - Profile tooltip latency with Chrome DevTools
  - Optimize useTextSelection hook (ensure debouncing)
  - Optimize tooltip re-renders (React.memo if needed)
  - Verify Qdrant search boosting adds <50ms latency
  - Document performance characteristics in README

---

## Summary by User Story

| User Story | Tasks | Est. Hours | Status |
|-----------|-------|-----------|--------|
| **Setup & Types** (Phase 1) | T001-T008 | 3-4h | Ready |
| **Foundational** (Phase 2) | T009-T015 | 6-8h | Ready |
| **US1: Selection + Tooltip (P1)** | T016-T022 | 8-10h | Ready |
| **US2: Search Boosting (P2)** | T023-T029 | 8-10h | Ready |
| **US3: Mobile Optimization (P3)** | T030-T035 | 5-6h | Ready |
| **Testing & QA (Phase 6)** | T036-T050 | 12-15h | Ready |
| **Polish & Accessibility (Phase 7)** | T051-T052 | 2-3h | Ready |

**Total**: 52 tasks | 44-56 hours estimated | MVP (Phases 1-3): 17-22 hours

---

## Parallel Execution Plan

### Scenario A: Two-Developer Team

**Developer 1 (Frontend)**:
1. Complete Phase 1 (Setup) - 3-4 hours
2. Complete Phase 2 (Hooks/Utilities) - 6-8 hours
3. Complete Phase 3 (US1 Tooltip) - 8-10 hours
4. Contribute to Phase 6 (Frontend Tests) - 6-8 hours
5. Complete Phase 7 (Polish) - 2-3 hours

**Developer 2 (Backend)**:
1. Complete Phase 1 (Setup) - 1 hour (parallel with Dev 1)
2. Complete Phase 2 (Utilities) - 3-4 hours (parallel with Dev 1)
3. Complete Phase 4 (US2 Search Boosting) - 8-10 hours (parallel with Dev 1 on US1)
4. Contribute to Phase 6 (Backend Tests) - 4-5 hours
5. Assist Phase 5 (Mobile) if needed

**Timeline**: ~25-30 hours total (parallelization saves 10-15 hours)

### Scenario B: Solo Developer

**Recommended Order**:
1. Phase 1 (Setup) - 3-4h
2. Phase 2 (Foundational) - 6-8h
3. Phase 3 (US1 MVP) - 8-10h (completes MVP)
4. Phase 4 (US2 Boosting) - 8-10h
5. Phase 5 (US3 Mobile) - 5-6h
6. Phase 6 (Testing) - 12-15h
7. Phase 7 (Polish) - 2-3h

**Timeline**: ~44-56 hours total (sequential)

---

## MVP Scope & Milestones

**Milestone 1: Specification ✅ COMPLETE**
- Feature specification with 3 user stories, 18 FRs, 9 SCs
- Checklist validation: All 16 items PASSED

**Milestone 2: Architecture ✅ COMPLETE**
- Implementation plan with 5 architectural decisions
- API contracts (request/response JSON Schema)
- Constitution check: All 7 principles PASS

**Milestone 3: Tasks ⏳ IN PROGRESS**
- Task generation: 52 implementation tasks (this document)
- Phase-based organization with dependencies
- Ready for Phase 1 implementation

**Milestone 4: MVP Implementation** (Next)
- Implement Phases 1-3 (Setup + US1 Tooltip)
- Timeline: 17-22 hours
- Deliverable: Users can select text, see tooltip, open ChatKit with pre-filled text

**Milestone 5: Full Feature** (After MVP)
- Implement Phases 4-7 (Boosting + Mobile + Testing + Polish)
- Timeline: 22-34 hours
- Deliverable: Complete feature with search boosting, mobile support, full test coverage

---

## Task Execution Rules

### Before Starting Phase 1
- [ ] Confirm branch `006-selected-text` is active: `git branch | grep '006-selected-text'`
- [ ] Ensure all dependencies installed: `npm install` and `pip install -r requirements.txt`
- [ ] Run type checker before any implementation: `npm run typecheck`

### During Implementation
- [ ] After each task: Run tests for that component
- [ ] After each phase: Commit changes with clear message (e.g., "feat: Implement US1 tooltip component")
- [ ] Maintain TypeScript strict mode: all new code must pass `npm run typecheck`
- [ ] Follow project code style (existing ChatKit patterns)

### After Each Phase
- [ ] Run full test suite: `npm test` (frontend), `pytest` (backend)
- [ ] Verify no console errors or warnings
- [ ] Check accessibility compliance (if phase included UI)
- [ ] Commit and push to feature branch

### Before Merging to Main
- [ ] All 52 tasks completed
- [ ] Test coverage ≥90%
- [ ] Performance benchmarks met (SC-001, SC-002, SC-009)
- [ ] WCAG AA compliance verified
- [ ] Code review completed
- [ ] Documentation updated

---

## Success Criteria Validation

| Criterion | Task(s) | Verification |
|-----------|---------|--------------|
| **SC-001** (Desktop tooltip <200ms) | T016, T036, T050 | Latency benchmark ≥1ms, ≤200ms |
| **SC-002** (Mobile tooltip <500ms) | T031, T034, T050 | Mobile latency benchmark ≤500ms |
| **SC-003** (Zero typing) | T020, T045 | Pre-fill verified in E2E test |
| **SC-004** (≥10% relevance improvement) | T025, T048 | Boosting results ranked higher |
| **SC-005** (95%+ positioning accuracy) | T012, T039 | Positioning tests pass, no clipping |
| **SC-006** (Mobile support 320px+, 48px) | T032, T034 | Mobile tests on 390px, button ≥48px |
| **SC-007** (WCAG AA) | T018, T040, T051 | axe-core audit passes, contrast ≥4.5:1 |
| **SC-008** (99%+ reliability) | T036, T049 | Edge case tests all pass |
| **SC-009** (<5s p95 backend) | T044, T048, T050 | Performance benchmark ≤5000ms |

---

## Next Steps

After tasks are approved:
1. Run `/sp.implement` to begin execution
2. Start with Phase 1 (Setup) - should complete in 3-4 hours
3. Proceed to Phase 2 (Hooks) - 6-8 hours, unblocks both frontend and backend
4. Split Phases 3 & 4 if multiple developers available
5. Complete testing (Phase 6) after main implementation
6. Polish and accessibility (Phase 7) can overlap with testing

**Ready for implementation** ✅
