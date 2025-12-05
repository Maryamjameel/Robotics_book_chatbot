---
id: 003
title: Selected Text Implementation Tasks Generation
stage: tasks
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: 006-selected-text
branch: 006-selected-text
user: developer
command: /sp.tasks
labels: ["task-generation", "implementation-planning", "user-story-breakdown", "phase-organization", "parallel-execution"]
links:
  spec: specs/006-selected-text/spec.md
  plan: specs/006-selected-text/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-selected-text/tasks.md
tests:
  - Task Organization: 52 tasks organized across 7 phases (Setup, Foundational, US1, US2, US3, Testing, Polish)
  - User Story Mapping: All 18 FRs mapped to user stories; 3 user stories prioritized (P1/P2/P3)
  - Phase Dependencies: Setup BLOCKS all phases; Foundational BLOCKS US stories; US3 depends on US1 completion
  - Parallel Opportunities: Frontend (Phases 2-3) and Backend (Phase 4) can run in parallel; 2-dev team saves ~10-15 hours
  - MVP Scope: Phases 1-3 (Setup + Foundational + US1 MVP) = 17-22 hours; complete feature = 44-56 hours
  - Success Criteria Mapping: All 9 SCs mapped to specific tasks with verification methods
  - Task Format: All 52 tasks follow strict checklist format [ID][P?][Story?] with file paths
---

## Prompt

User executed `/sp.tasks` command to generate implementation tasks for Feature 006 (Selected Text Context).

**Context**:
- Specification completed in Phase 0 (3 user stories, 18 FRs, 9 SCs)
- Architecture planned in Phase 1 (5 decisions, Constitution check passed, API contracts defined)
- Ready for implementation planning with actionable task breakdown

**Task**: Transform architecture plan and specification into 40-60 granular implementation tasks organized by user story (P1/P2/P3), with clear dependencies, parallel execution opportunities, success criteria mapping, and MVP scope definition.

## Response snapshot

### Tasks Generated

**File**: `specs/006-selected-text/tasks.md` (500+ lines)

**Summary**:
- **Total Tasks**: 52 (organized across 7 phases)
- **Estimated Effort**: 44-56 hours total (17-22 hours for MVP, 27-34 hours for full feature)
- **Phases**:
  1. Setup & Types (T001-T008, 3-4h): Directory structure, TypeScript types, Pydantic models
  2. Foundational (T009-T015, 6-8h): Hooks (useTextSelection, useSelectionTooltip), utility functions (TF-IDF, validation)
  3. US1 MVP (T016-T022, 8-10h): SelectionTooltip component, ChatKit integration, tooltip dismissal
  4. US2 Enhancement (T023-T029, 8-10h): Backend API changes, Qdrant search boosting, TF-IDF algorithm
  5. US3 Mobile (T030-T035, 5-6h): Touch event handling, mobile positioning, responsive design
  6. Testing & QA (T036-T050, 12-15h): Unit, component, E2E, integration, accessibility tests
  7. Polish (T051-T052, 2-3h): Performance optimization, accessibility audit, documentation

### Task Organization

**Phase 1: Setup & Type Definitions** (8 tasks)
- T001: TypeScript types file (TextSelection, SelectionTooltipState)
- T002: Directory structure creation (frontend/src/hooks, /components, /types)
- T003: Backend directory structure
- T004: Test directories
- T005-T008: Pydantic models, schema files, constants

**Phase 2: Foundational Hooks & Utilities** (7 tasks, parallelizable)
- T009-T012 [P] (Frontend): useTextSelection, useSelectionTooltip hooks, validation, positioning utilities
- T013-T015 [P] (Backend): TF-IDF utilities, validation, Qdrant mock
- Milestone: Both frontend and backend can proceed independently after Phase 2

**Phase 3: User Story 1 - Selection Detection & Tooltip (P1 MVP)** (7 tasks)
- T016-T022 [US1]: SelectionTooltip component, styles, accessibility, Root.tsx integration, ChatKit pre-fill, dismissal logic
- **Milestone**: MVP complete after Phase 3; users can select text → see tooltip → open ChatKit with pre-filled text

**Phase 4: User Story 2 - Search Boosting (P2 Enhancement)** (7 tasks, parallelizable with Phase 3)
- T023-T029 [US2] [P]: FastAPI endpoint, Qdrant service enhancement, TF-IDF algorithm, response models, validation, graceful fallback, integration test
- **Milestone**: Search results boosted when selected_text provided; backward compatible (requests without selected_text still work)

**Phase 5: User Story 3 - Mobile Optimization (P3)** (6 tasks, depends on Phase 3)
- T030-T035 [US3]: Touch event handling, mobile positioning, responsive styles, scroll handling, E2E mobile tests

**Phase 6: Testing & Quality Assurance** (15 tasks)
- T036-T039 [P]: Unit tests (hooks, utilities) - can start after Phase 2
- T040-T041 [P]: Component tests (SelectionTooltip) - after Phase 3
- T042-T050: Backend tests, E2E tests, integration tests, edge cases, performance benchmarks
- **Coverage**: 90%+ target across frontend and backend

**Phase 7: Polish & Cross-Cutting Concerns** (2 tasks)
- T051: Accessibility audit (WCAG AA compliance with axe-core)
- T052: Performance optimization and monitoring

### Task Format Validation

✅ **ALL 52 TASKS FOLLOW REQUIRED FORMAT**:
- `- [ ]` checkbox start
- Task ID (T001-T052 sequential)
- [P] parallelization marker (where applicable)
- [US1], [US2], [US3] user story labels (Phase 3+ only)
- Clear action description with exact file paths

**Example Format**:
- ✅ `- [ ] T016 [P] [US1] Implement SelectionTooltip component at frontend/src/components/SelectionTooltip/SelectionTooltip.tsx`
- ✅ `- [ ] T023 [P] [US2] Modify FastAPI chat endpoint at backend/app/api/v1/routes/chat.py`
- ✅ `- [ ] T001 Create TypeScript types file at frontend/src/types/selected-text.types.ts`

### Dependency Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational) ← BLOCKS ALL user stories
    ├→ Phase 3 (US1 MVP) ──┐
    ├→ Phase 4 (US2) ────┐  (parallelizable)
    └→ Phase 6 Unit Tests │

Phase 3 (US1)
    ├→ Phase 5 (US3 Mobile) ← depends on US1
    └→ Phase 6 Component Tests

Phase 4 (US2)
    └→ Phase 6 Integration Tests

Phase 6 (Testing) ← can start after Phase 2 (unit tests)
Phase 7 (Polish) ← can run in parallel with Phase 6
```

### Success Criteria Mapping

All 9 success criteria mapped to specific implementation tasks:

| SC | Metric | Task(s) | Verification |
|----|--------|---------|--------------|
| **SC-001** | Desktop tooltip <200ms | T016, T036, T050 | Latency benchmark ≤200ms |
| **SC-002** | Mobile tooltip <500ms | T031, T034, T050 | Mobile latency benchmark ≤500ms |
| **SC-003** | Zero typing (pre-fill) | T020, T045 | E2E test verifies pre-fill |
| **SC-004** | ≥10% relevance improvement | T025, T048 | Integration test: boosted results rank higher |
| **SC-005** | 95%+ positioning accuracy | T012, T039 | Positioning unit tests pass, no clipping |
| **SC-006** | Mobile 320px+, 48px targets | T032, T034 | Mobile E2E on 390px, buttons ≥48px |
| **SC-007** | WCAG AA compliance | T018, T040, T051 | axe-core audit passes, contrast ≥4.5:1 |
| **SC-008** | 99%+ reliability | T036, T049 | Edge case tests all pass |
| **SC-009** | <5s p95 backend latency | T044, T048, T050 | Performance benchmark ≤5000ms |

### Parallel Execution Scenarios

**Two-Developer Team**:
- Developer 1 (Frontend): Phase 1 (3-4h) → Phase 2 Frontend (3-4h) → Phase 3 (8-10h) → Phase 6 Frontend tests → Phase 7
- Developer 2 (Backend): Phase 1 (1h) → Phase 2 Backend (3-4h) → Phase 4 (8-10h parallel with Dev1) → Phase 6 Backend tests
- **Total time**: ~25-30 hours (saves 10-15 hours vs. sequential)

**Single Developer**:
- Sequential: Phase 1 → 2 → 3 → 4 → 5 → 6 → 7
- **Total time**: 44-56 hours

### MVP Scope

**Milestone 4: MVP Implementation** (Target 17-22 hours)
- Phases 1-3 only (Setup + Foundational + US1 Tooltip)
- **Deliverable**: Users can select text → tooltip appears → click "Ask about this" → ChatKit opens with pre-filled text
- **Standalone Value**: P1 user story complete; doesn't require P2 boosting or P3 mobile to be useful
- **Backward Compatible**: No breaking changes; existing ChatKit works unchanged if selection feature fails

**After MVP**:
- Phase 4: Search result boosting (8-10h)
- Phase 5: Mobile optimization (5-6h)
- Phase 6-7: Full test coverage + accessibility (14-18h)

### Quality Validation

✅ **Task Completeness**:
- All 18 functional requirements addressed in tasks
- All 3 user stories have dedicated task phases
- All 9 success criteria have verification tasks
- All edge cases from spec covered in testing phase (T049)

✅ **Phase Dependencies**:
- Setup (Phase 1) blocks all others ✓
- Foundational (Phase 2) unblocks parallel US development ✓
- US1 and US2 can run in parallel (independent) ✓
- US3 depends on US1 completion (mobile enhancements) ✓
- Testing can start after Phase 2 (unit tests) ✓

✅ **Implementation Readiness**:
- Each task is specific and actionable (file paths, acceptance criteria)
- No ambiguous tasks ("implement feature X" without file path)
- Testing tasks included (unit, component, E2E, integration)
- Performance and accessibility requirements explicit

### Task Execution Rules

**Before Phase 1**:
- Confirm branch `006-selected-text` active
- Ensure dependencies installed (npm, pip)
- Run type checker: `npm run typecheck`

**During Implementation**:
- Run tests after each task
- Maintain TypeScript strict mode
- Follow existing project code style
- Commit after each phase

**Before Merge to Main**:
- All 52 tasks complete
- Test coverage ≥90%
- Performance targets met (SC-001, SC-002, SC-009)
- WCAG AA compliance verified
- Code review completed

## Outcome

- ✅ **Impact**: Complete task breakdown for Feature 006 with 52 granular, actionable implementation tasks. All 18 FRs mapped to user stories. All 3 user stories (P1/P2/P3) have dedicated task phases. All 9 success criteria have explicit task assignments and verification methods. Task organization enables parallel development (2-dev team saves 10-15 hours vs. solo). MVP scope clearly defined (Phases 1-3, 17-22 hours) with standalone value. Full feature scope includes testing and accessibility (44-56 hours total). Phase dependencies documented for correct execution order. Parallel execution scenarios provided for both team and solo developers.

- 🧪 **Tests**: Task Format Validation PASSED (all 52 tasks follow strict checklist format with IDs, labels, file paths). Task Organization PASSED (7 phases, clear dependencies, Setup→Foundational→US Stories→Testing→Polish). User Story Mapping PASSED (18 FRs distributed across P1/P2/P3). Success Criteria Mapping PASSED (all 9 SCs assigned to specific tasks). MVP Scope PASSED (clear definition: Phases 1-3 = 17-22h; full feature = 44-56h). Parallel Opportunities PASSED (2-dev scenario detailed; frontend/backend can run in parallel after Phase 2).

- 📁 **Files**: 1 file created (600+ lines)
  - `specs/006-selected-text/tasks.md` (600+ lines): Complete implementation task breakdown with:
    - 52 tasks organized across 7 phases (Setup, Foundational, US1, US2, US3, Testing, Polish)
    - Phase-by-phase breakdown with estimated hours
    - Task checklist with ID, P marker, Story label, file paths
    - Success criteria table mapping all 9 SCs to tasks
    - Parallel execution scenarios (2-dev team vs. solo developer)
    - MVP scope and milestones
    - Task execution rules (before/during/after)
    - Summary table by user story

- 🔁 **Next prompts**: (1) Execute `/sp.implement` to begin Phase 1 implementation (Setup). (2) Developer(s) should create project structure and TypeScript types per T001-T008 (3-4 hours). (3) Proceed to Phase 2 (Hooks, 6-8 hours) after Setup complete. (4) Frontend and Backend can split at Phase 2: Dev1 does Phases 3, Dev2 does Phase 4 (parallel execution). (5) Testing (Phase 6) can begin after each component is implemented (unit tests after Phase 2, component tests after Phase 3). (6) Polish (Phase 7) overlaps with testing and final optimization.

- 🧠 **Reflection**: Task generation focused on independent, testable increments organized by user story (P1/P2/P3). Each phase has clear acceptance criteria (tests pass, components work). Phase dependencies enable parallel development: Setup and Foundational are blocking prerequisites, but Phases 3 & 4 (US1 and US2) are independent and can run simultaneously, cutting development time in half with two developers. MVP scope (Phases 1-3) delivers core value in 17-22 hours; optional enhancements (Phases 4-5) and testing (Phases 6-7) can proceed incrementally. All 52 tasks follow strict format with file paths and task IDs, making each task immediately actionable without additional context. Success criteria are explicit (each SC has verification task); performance and accessibility requirements are not afterthoughts but built into task descriptions.

## Evaluation notes (flywheel)

- **Failure modes observed**: None during task generation. Specification was clear and complete; plan provided detailed architecture and decisions. Task generation proceeded smoothly, mapping all FRs to user stories, and all SCs to verification tasks.

- **Graders run and results (PASS/FAIL)**:
  - Task Format: PASSED (all 52 tasks follow [ID][P?][Story?] format with file paths)
  - User Story Organization: PASSED (3 phases for P1/P2/P3, each independently testable)
  - Phase Dependencies: PASSED (Setup→Foundational→US Stories→Testing→Polish order enforced)
  - Success Criteria Mapping: PASSED (all 9 SCs assigned to 1-3 tasks each with verification methods)
  - Parallel Opportunities: PASSED (2-dev scenario saves 10-15 hours; Phase 2 is parallel execution point)
  - MVP Definition: PASSED (clear: Phases 1-3 = 17-22h, delivers core value, stands alone)
  - Task Actionability: PASSED (each task includes file path, acceptance criteria, no ambiguity)

- **Prompt variant (if applicable)**: Standard `/sp.tasks` execution with empty ARGUMENTS (no special parameters). Task generation proceeded from specification (3 user stories, 18 FRs, 9 SCs) and plan (5 decisions, Constitution check). Generated 52 tasks in 7 phases, organized by user story with dependencies.

- **Next experiment (smallest change to try)**: Execute `/sp.implement` to begin Phase 1 (Setup) implementation. Estimated 3-4 hours to create directory structure and TypeScript types (T001-T008). After Phase 1, proceed to Phase 2 (Foundational hooks and utilities, 6-8 hours). Phase 2 completion unblocks parallel US development (Phase 3 for frontend, Phase 4 for backend). MVP is complete after Phase 3 (17-22 hours total).

