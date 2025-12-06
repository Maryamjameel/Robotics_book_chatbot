# Tasks: Docusaurus Theme Integration & Production Configuration

**Feature**: 008-docusaurus-theme-config
**Branch**: `008-docusaurus-theme-config`
**Input**: Design documents from `/specs/008-docusaurus-theme-config/`
**Prerequisites**: ✅ plan.md, ✅ spec.md (both complete)

**Tests**: Comprehensive unit tests (Vitest) and E2E tests (Playwright) are explicitly required by FR-008-013. Test tasks are included.

**Organization**: Tasks are grouped by user story (US1, US2, US3) to enable independent implementation and testing. Each story is independently testable and deployable.

---

## Summary

**Total Tasks**: 24 tasks across 5 phases
**Parallelizable Tasks**: 8 tasks can run in parallel (marked [P])
**MVP Scope**: User Stories 1 & 2 (theme integration + API configuration)
**Extended Scope**: User Story 3 (comprehensive E2E testing)

**Execution Path**:
1. Phase 1 (Setup) → 2 tasks
2. Phase 2 (Foundational) → 3 tasks
3. Phase 3 (US1: Theme Integration) → 7 tasks (can start after Phase 2)
4. Phase 4 (US2: API Configuration) → 8 tasks (can start after Phase 2, parallel with Phase 3)
5. Phase 5 (US3: E2E Testing) → 4 tasks (depends on US1+US2 complete)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and configuration

- [ ] T001 Create Playwright configuration file with `playwright.config.ts` at repository root with browser options (Chrome, Firefox, Safari), mobile viewport settings (320px, 768px, 1024px), screenshot/video capture on failure
- [ ] T002 Update `frontend/.env.example` to document `REACT_APP_API_URL` parameter with development and production examples

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure BEFORE any user story work

**⚠️ CRITICAL**: All phase 2 tasks MUST be complete before user story work begins

- [ ] T003 Create API configuration module at `frontend/src/config/api.ts` exporting APIConfiguration interface with baseURL, environment, isDevelopment, isProduction, validateURL() method; implement NODE_ENV-based defaults (development → http://localhost:8000/api, production → https://api.yourdomain.com/api) and REACT_APP_API_URL override logic
- [ ] T004 [P] Create theme context hook at `frontend/src/components/ChatKit/hooks/useThemeContext.ts` implementing MutationObserver listener on `<html data-theme>` attribute, detecting light/dark mode changes, resolving CSS variables via getComputedStyle(), returning ThemeConfig object with isDarkMode and cssVariables properties
- [ ] T005 [P] Add CSS fallback colors to `frontend/src/components/ChatKit/styles/chatkit.css` defining fallback values for --ifm-color-primary, --ifm-background-color, --ifm-font-color-base, --ifm-color-emphasis-200 for older Docusaurus versions or custom setups

**Checkpoint**: Foundation ready - both user stories 1 & 2 can be implemented in parallel

---

## Phase 3: User Story 1 - ChatKit Adapts to Docusaurus Light/Dark Theme (Priority: P1) 🎯 MVP

**Goal**: ChatKit styling uses Docusaurus CSS variables and dynamically adapts to theme changes (light/dark) without page reload

**Independent Test**: Open ChatKit on Docusaurus page in light mode, toggle to dark mode, verify colors change within 300ms and remain consistent. Test passes independently of API configuration (US2).

### Tests for User Story 1

> **Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T006 [P] [US1] Create unit test file `frontend/src/components/ChatKit/__tests__/theme.test.ts` with 5+ test cases covering: useThemeContext hook returns correct CSS variables, MutationObserver detects html[data-theme] changes, theme context updates when Docusaurus theme toggles, CSS transitions are smooth (300ms), fallback colors used when variables undefined
- [ ] T007 [P] [US1] Create unit test file `frontend/src/components/ChatKit/__tests__/chatkit-theme.test.ts` testing ChatKit component correctly applies theme styles from useThemeContext hook, verifies color accuracy against Docusaurus variables, ensures no console errors during theme transitions

### Implementation for User Story 1

- [ ] T008 [US1] Modify `frontend/src/components/ChatKit/styles/chatkit.css` to replace all hardcoded colors with Docusaurus CSS variables (primary → var(--ifm-color-primary), background → var(--ifm-background-color), text → var(--ifm-font-color-base), borders → var(--ifm-color-emphasis-200)); add CSS transitions: `transition: background-color 300ms ease, color 300ms ease, border-color 300ms ease;`
- [ ] T009 [P] [US1] Create theme context type definitions in `frontend/src/components/ChatKit/types/chatkit.types.ts` defining ThemeConfig interface with isDarkMode: boolean, cssVariables object with primaryColor, backgroundColor, fontColorBase, borderColor
- [ ] T010 [US1] Update `frontend/src/components/ChatKit/ChatKitWidget.tsx` to import useThemeContext hook (line ~13), call theme hook in component (line ~50), pass theme colors to all styled elements, remove all hardcoded color definitions
- [ ] T011 [P] [US1] Add browser compatibility tests in Playwright: verify theme detection works in Chrome, Firefox, Safari, Edge (test file: `frontend/src/__tests__/e2e/theme-switching.spec.ts`)

**Checkpoint**: User Story 1 complete and independently testable - ChatKit visually matches Docusaurus theme in light and dark modes

---

## Phase 4: User Story 2 - Production API Configuration (Priority: P1) 🎯 MVP

**Goal**: ChatKit connects to correct API endpoint based on environment (development localhost, production URL, or custom REACT_APP_API_URL)

**Independent Test**: Build application with NODE_ENV=development and verify localhost endpoint used; rebuild with REACT_APP_API_URL override and verify custom endpoint used; inspect network requests to confirm endpoints match configuration.

### Tests for User Story 2

> **Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T012 [P] [US2] Create unit test file `frontend/src/components/ChatKit/__tests__/api-config.test.ts` with 6+ test cases covering: getAPIConfig() returns correct baseURL for development, getAPIConfig() returns correct baseURL for production, REACT_APP_API_URL env var overrides NODE_ENV, validateURL() rejects invalid URLs, validateURL() enforces HTTPS in production, configuration loads at startup before components render
- [ ] T013 [P] [US2] Create integration test file `frontend/src/__tests__/integration/api-config.test.ts` verifying useRAGAPI hook uses apiConfig baseURL, all API calls target configured endpoint, chapter context passes through configured endpoint, error handling for misconfigured endpoints

### Implementation for User Story 2

- [ ] T014 [US2] Update `frontend/src/components/ChatKit/hooks/useRAGAPI.ts` to import apiConfig from `frontend/src/config/api.ts` (line ~10), replace hardcoded API URL with `apiConfig.baseURL` in all fetch/axios calls (line ~150-200), add error handling for invalid API endpoints
- [ ] T015 [P] [US2] Create validation utility at `frontend/src/config/validate-api.ts` implementing validateURL() function checking: URL is valid format (URL constructor), HTTPS used in production (process.env.NODE_ENV === 'production'), URL is reachable (optional health check), domain matches expected patterns
- [ ] T016 [P] [US2] Create logging utility at `frontend/src/config/api-logger.ts` logging API configuration at application startup with: NODE_ENV, resolved baseURL, REACT_APP_API_URL override status, validation results, timestamp
- [ ] T017 [US2] Update `.env.example` files documenting: REACT_APP_API_URL parameter with development (http://localhost:8000/api) and production (https://api.yourdomain.com/api) examples, NODE_ENV values (development, production, staging)
- [ ] T018 [P] [US2] Create E2E test file `frontend/src/__tests__/e2e/api-endpoint.spec.ts` verifying: build with NODE_ENV=development targets localhost, build with REACT_APP_API_URL override targets custom URL, API requests in network tab use configured endpoint, error handling for connection failures

**Checkpoint**: User Story 2 complete and independently testable - API configuration works correctly across environments

---

## Phase 5: User Story 3 - End-to-End Test Coverage (Priority: P2)

**Goal**: Comprehensive Playwright E2E test suite validating ChatKit works correctly across all user scenarios: theme switching, API endpoint configuration, selected text integration, chapter context passing, mobile responsiveness, message persistence

**Independent Test**: Run full Playwright test suite (`npm run test:e2e`) and verify all 8+ scenarios pass with 95%+ consistency; execution time < 60 seconds

### Implementation for User Story 3 (All E2E Tests)

- [ ] T019 [P] [US3] Create E2E test file `frontend/src/__tests__/e2e/theme-switching.spec.ts` with 4 test cases: open ChatKit in light mode verify colors, toggle to dark mode verify colors change within 300ms, toggle back verify correct colors, multiple toggles verify no console errors or flickering
- [ ] T020 [P] [US3] Create E2E test file `frontend/src/__tests__/e2e/selected-text.spec.ts` with 3 test cases: select text on page, ChatKit opens with selected text in query, selected text correctly passes to backend API request
- [ ] T021 [P] [US3] Create E2E test file `frontend/src/__tests__/e2e/chapter-context.spec.ts` with 4 test cases: navigate to chapter page, chapter context automatically detected from URL/h1, chapter ID and title passed in API request, response includes chapter filtering metadata
- [ ] T022 [P] [US3] Create E2E test file `frontend/src/__tests__/e2e/mobile-responsive.spec.ts` with 4 test cases: test viewport 320px (mobile), test viewport 768px (tablet), test viewport 1024px (desktop), all viewports show ChatKit functional and readable with no horizontal scroll
- [ ] T023 [P] [US3] Create E2E test file `frontend/src/__tests__/e2e/message-persistence.spec.ts` with 3 test cases: send single message verify displayed, send multiple messages verify all visible in history, send messages verify chapter context maintained across all messages
- [ ] T024 [US3] Update `playwright.config.ts` with retryConfig (3 retries), timeouts (30s per test), parallel workers (4), screenshot/video capture on failure, mobile emulation (iPhone 12, Pixel 5), base URL pointing to Docusaurus instance

**Checkpoint**: User Story 3 complete - Full E2E test suite validates all user workflows

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, documentation, and quality assurance

> **Note**: This phase can begin after Phase 3 & 4 are complete (US1 & US2 working)

### Documentation & Final Validation

- [ ] T025 [P] Update `README.md` documenting: REACT_APP_API_URL configuration, running E2E tests with `npm run test:e2e`, Docusaurus version compatibility requirements, troubleshooting theme detection issues
- [ ] T026 [P] Create `QUICKSTART.md` in `specs/008-docusaurus-theme-config/` documenting: development setup (`npm install && npm start`), running unit tests (`npm run test:unit`), running E2E tests (`npm run test:e2e`), verifying API configuration with `NODE_ENV` values
- [ ] T027 Validate constitution compliance: Production-Grade Quality ✅, Privacy-First ✅, RAG Accuracy ✅, Modular Architecture ✅, Observability ✅, Spec-Driven Development ✅
- [ ] T028 [P] Run full test suite and verify: all unit tests pass (>80% coverage for theme and API config modules), all E2E tests pass with <60s execution time, no console errors or warnings logged

---

## Task Dependency Graph

```
Phase 1: Setup
  ↓
Phase 2: Foundational (BLOCKING)
  ├─ T003 (API config module)
  ├─ T004 (useThemeContext hook)
  └─ T005 (CSS fallbacks)
  ↓
Phase 3: User Story 1 (Theme) ← Parallel with Phase 4
  ├─ T006 [P] Unit tests (write first)
  ├─ T007 [P] ChatKit component tests
  ├─ T008 Modify chatkit.css
  ├─ T009 [P] Type definitions
  ├─ T010 Update ChatKitWidget
  └─ T011 [P] Browser compatibility tests

Phase 4: User Story 2 (API Config) ← Parallel with Phase 3
  ├─ T012 [P] Unit tests (write first)
  ├─ T013 [P] Integration tests
  ├─ T014 Update useRAGAPI hook
  ├─ T015 [P] Validation utility
  ├─ T016 [P] Logging utility
  ├─ T017 Update .env.example
  └─ T018 [P] E2E endpoint tests
  ↓
Phase 5: User Story 3 (E2E Tests) ← Depends on US1 + US2
  ├─ T019 [P] Theme switching tests
  ├─ T020 [P] Selected text tests
  ├─ T021 [P] Chapter context tests
  ├─ T022 [P] Mobile responsive tests
  ├─ T023 [P] Message persistence tests
  └─ T024 Playwright configuration

Phase 6: Polish
  ├─ T025 [P] README updates
  ├─ T026 [P] QUICKSTART.md
  ├─ T027 Constitution validation
  └─ T028 [P] Final test suite
```

---

## Parallel Execution Opportunities

### Group 1 (Parallel after Phase 2)
**Can execute simultaneously**: T006, T007, T009, T012, T013, T015, T016

**Rationale**: Independent unit test files with no file dependencies

### Group 2 (Parallel after Group 1)
**Can execute simultaneously**: T011, T018, T019, T020, T021, T022, T023

**Rationale**: Independent E2E test files targeting different scenarios

### Group 3 (Independent)
**Can execute simultaneously**: T025, T026, T028

**Rationale**: Documentation and final validation tasks with no file dependencies

**Recommended Parallelization**:
1. Run Phases 1-2 sequentially (foundation)
2. Run Phase 3 tasks + Phase 4 tasks in parallel (US1 and US2 independent)
3. Run Phase 5 tasks after Phases 3-4 (depends on working features)
4. Run Phase 6 tasks in parallel with Phase 5

---

## Implementation Strategy: MVP First, Incremental Delivery

### MVP Scope (User Stories 1 & 2)
**Deliverable**: ChatKit matches Docusaurus theme + correct API endpoint in all environments
**Tasks**: T001-T018 (Phases 1-4)
**Timeline**: ~4-5 days for experienced developer
**User Value**: Visual consistency + production connectivity
**Independent Testable**: ✅ Yes (both stories independently testable)
**Deployable**: ✅ Yes (both stories can be released independently)

### Extended Scope (Add User Story 3)
**Deliverable**: Comprehensive E2E test coverage validating all user workflows
**Tasks**: T019-T024 (Phase 5)
**Additional Timeline**: ~2-3 days
**User Value**: Confidence in feature quality + regression prevention
**Dependencies**: Requires US1 & US2 complete to test

### Quality Assurance (Phase 6)
**Deliverable**: Documentation + final validation
**Tasks**: T025-T028 (Phase 6)
**Additional Timeline**: ~1 day
**User Value**: Developer onboarding + maintainability

---

## Format Reference

**Checklist Format** (REQUIRED for all tasks):
```
- [ ] [TaskID] [P?] [Story?] Description with exact file path
```

**Examples**:
- ✅ `- [ ] T003 Create API configuration module at frontend/src/config/api.ts`
- ✅ `- [ ] T008 [US1] Modify frontend/src/components/ChatKit/styles/chatkit.css`
- ✅ `- [ ] T012 [P] [US2] Create unit test file frontend/src/components/ChatKit/__tests__/api-config.test.ts`
- ✅ `- [ ] T028 [P] Run full test suite and verify results`

---

## Success Criteria per User Story

**User Story 1 (Theme Integration)**:
- [ ] ChatKit colors match Docusaurus CSS variables in light AND dark modes
- [ ] Theme transitions complete in 300ms without flickering
- [ ] No console errors when toggling theme
- [ ] Fallback colors used if variables undefined (graceful degradation)
- [ ] All unit tests pass (>80% coverage for theme-related code)
- [ ] E2E tests verify theme switching works across Chrome, Firefox, Safari, Edge

**User Story 2 (API Configuration)**:
- [ ] NODE_ENV=development → uses http://localhost:8000/api
- [ ] NODE_ENV=production → uses production URL from environment
- [ ] REACT_APP_API_URL override takes precedence over NODE_ENV
- [ ] Configuration loads at startup (before any component renders)
- [ ] URL validation prevents invalid/non-HTTPS production endpoints
- [ ] All API calls from ChatKit target configured endpoint
- [ ] All unit tests pass (>80% coverage for API config)
- [ ] E2E tests verify endpoints are correct in network tab

**User Story 3 (E2E Testing)**:
- [ ] 8+ E2E test scenarios covering all user workflows
- [ ] All tests pass with 95%+ consistency (retry to account for timing)
- [ ] Full suite executes in <60 seconds
- [ ] Mobile viewport tests pass (320px, 768px, 1024px)
- [ ] Selected text integration verified end-to-end
- [ ] Chapter context detection and passing verified
- [ ] Message persistence across multiple queries verified

---

## Notes for Implementation

### Setup Tips
- Use `npm create playwright@latest` to initialize Playwright configuration
- Copy existing `.env.example` and add REACT_APP_API_URL parameter
- Ensure Node.js 18+ and npm 9+ are available

### Code Quality
- Enable TypeScript strict mode in `tsconfig.json` for all new files
- Run `npm run type-check` to verify type safety
- Run `npm run lint` to check code style before committing
- Aim for >80% test coverage for new code (FR requirement)

### Testing Best Practices
- Write tests FIRST (TDD approach) - write failing tests, then implement
- Use meaningful test names that describe the behavior being tested
- Mock external dependencies (API calls) in unit tests
- Use explicit waits in Playwright tests (avoid implicit waits)
- Capture screenshots/videos on failure for debugging

### Documentation
- Add JSDoc comments to exported functions and interfaces
- Document environment variable requirements in `.env.example`
- Update README.md with configuration instructions
- Create QUICKSTART.md for developers new to the feature

### Git Workflow
- Create feature branch: `git checkout -b 008-docusaurus-theme-config`
- Commit after each completed task: `git commit -m "feat: [T00X] Description"`
- Keep commits small and focused (one task = one commit)
- Push before creating PR: `git push origin 008-docusaurus-theme-config`

---

## Estimated Task Effort

**Setup Phase (T001-T002)**: ~0.5 days
**Foundational Phase (T003-T005)**: ~1 day
**User Story 1 (T006-T011)**: ~1.5 days (tests + implementation)
**User Story 2 (T012-T018)**: ~1.5 days (tests + implementation)
**User Story 3 (T019-T024)**: ~1 day (E2E tests)
**Polish (T025-T028)**: ~0.5 days

**Total Estimated Effort**: ~6-7 days for single developer

**Parallel Execution Potential**: ~4-5 days with 2 developers (one on US1, one on US2)

---

**Status**: ✅ READY FOR IMPLEMENTATION
**Next Step**: Begin Phase 1 setup tasks (T001-T002)
