# Implementation Plan: Docusaurus Theme Integration & Production Configuration

**Branch**: `008-docusaurus-theme-config` | **Date**: 2025-12-06 | **Spec**: [specs/008-docusaurus-theme-config/spec.md](spec.md)
**Input**: Feature specification from `specs/008-docusaurus-theme-config/spec.md`

## Summary

Feature 008 integrates ChatKit styling with Docusaurus theme system and establishes production API configuration. Three primary outcomes:

1. **Docusaurus Theme Integration**: ChatKit styling uses Docusaurus CSS variables (`--ifm-color-primary`, `--ifm-background-color`, etc.) with dynamic dark mode support via theme toggle listener
2. **Production API Configuration**: Environment-based API endpoint selection (NODE_ENV + REACT_APP_API_URL override) stored in `frontend/config/api.ts`
3. **End-to-End Test Coverage**: Comprehensive Playwright test suite validating theme matching, API configuration, selected text integration, chapter context, mobile responsiveness, and message persistence

---

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), Python 3.11+ (backend), Node.js 18+
**Primary Dependencies**:
- Frontend: React 18+, Docusaurus 3.x, Playwright (E2E testing)
- Backend: FastAPI 0.110+, Pydantic v2 (already in place)
**Storage**: Neon Postgres (existing), Qdrant (existing)
**Testing**: Vitest (unit), Pytest (backend integration), Playwright (E2E)
**Target Platform**: Web (Docusaurus documentation site)
**Project Type**: Web application (frontend + backend integration)
**Performance Goals**:
- CSS variable lookup: <1ms
- Theme transitions: 300ms smooth (60fps)
- API endpoint resolution: <1ms
- E2E test execution: <60 seconds
**Constraints**:
- No changes to Docusaurus core configuration
- Zero additional npm dependencies for CSS variables (native support)
- Theme switching must not cause page reloads
- API configuration must follow 12-factor app principles
**Scale/Scope**:
- Affects all documentation pages
- Supports light/dark modes
- Single API endpoint configuration per deployment
- 320px-2560px responsive viewport coverage

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ PASS - All Constitution Principles Satisfied

| Principle | Requirement | Status | Notes |
|-----------|------------|--------|-------|
| **I. Production-Grade Quality** | Error handling, type safety, testing, graceful degradation, performance monitoring | ✅ PASS | CSS variables are native (error-free), TypeScript strict mode on frontend, FR-008-013 require comprehensive E2E tests, graceful fallback colors if variables undefined, performance budgets defined (<1ms variable lookup, 300ms transitions) |
| **II. Privacy-First** | No new data collection in this feature | ✅ PASS | Configuration is environment-only, no user data involved |
| **III. RAG Accuracy** | No direct RAG impact; API configuration enables existing RAG | ✅ PASS | Feature establishes correct API endpoint for RAG to function; no changes to RAG itself |
| **IV. Modular Architecture** | Decoupled services, API-first, dependency injection, contract testing | ✅ PASS | API configuration module is independent; integrates via `useRAGAPI` hook without component changes; contracts defined for theme and API config endpoints |
| **V. Content Quality** | Not applicable to configuration feature | ✅ N/A | Feature supports content delivery, does not modify content |
| **VI. Observability** | Configuration logging, performance metrics | ✅ PASS | Configuration logged at startup (FR-014-016); performance metrics defined for theme transitions and E2E test execution |
| **VII. Spec-Driven Development** | All features follow spec-plan-tasks workflow | ✅ PASS | Feature 008 follows full SDD workflow: spec ✅, plan (this document), tasks (forthcoming) |

### Technology Standards Alignment

| Standard | Requirement | Implementation |
|----------|------------|-----------------|
| Frontend (Docusaurus) | TypeScript strict, Tailwind CSS, React Testing Library | ✅ Using TypeScript with strict mode; Tailwind CSS for styling variables; Vitest for unit tests |
| Testing Requirements | 80%+ coverage for critical logic, <30s unit tests, E2E tests | ✅ FR-008-013 define E2E test coverage; <60s full suite execution target |
| Security Standards | Input validation, HTTPS in production, no hardcoded secrets | ✅ API URL from environment variables only; HTTPS validated at build time; no credentials in code |
| Development Workflow | Conventional Commits, feature branches, PR reviews | ✅ Branch `008-docusaurus-theme-config` created; following `###-feature-name` pattern |

**Verdict**: ✅ **PASS** - Feature complies with all constitution principles. No violations or deviations.

## Project Structure

### Documentation (this feature)

```text
specs/008-docusaurus-theme-config/
├── plan.md                      # This file (/sp.plan command output)
├── research.md                  # Phase 0 output (NEXT: /sp.plan)
├── data-model.md                # Phase 1 output (NEXT: /sp.plan)
├── quickstart.md                # Phase 1 output (NEXT: /sp.plan)
├── contracts/                   # Phase 1 output (NEXT: /sp.plan)
│   ├── theme-api.yaml
│   └── api-config-api.yaml
├── spec.md                       # Feature specification
├── checklists/
│   └── requirements.md           # Quality validation checklist
└── tasks.md                      # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root - Web application structure)

```text
frontend/
├── src/
│   ├── config/
│   │   └── api.ts                   # NEW: API configuration module
│   ├── components/
│   │   ├── ChatKit/
│   │   │   ├── ChatKitWidget.tsx     # MODIFIED: Use configured API
│   │   │   ├── styles/
│   │   │   │   └── chatkit.css       # MODIFIED: Use CSS variables
│   │   │   ├── hooks/
│   │   │   │   ├── useRAGAPI.ts      # MODIFIED: Accept configured endpoint
│   │   │   │   └── useThemeContext.ts # NEW: Theme detection hook
│   │   │   └── __tests__/
│   │   │       ├── theme.test.ts     # NEW: Theme integration tests
│   │   │       └── api-config.test.ts # NEW: API config tests
│   └── __tests__/
│       └── e2e/                      # NEW: Playwright E2E tests
│           ├── theme-switching.spec.ts
│           ├── api-endpoint.spec.ts
│           ├── selected-text.spec.ts
│           ├── chapter-context.spec.ts
│           ├── mobile-responsive.spec.ts
│           └── message-persistence.spec.ts
└── .env.example                 # MODIFIED: Add REACT_APP_API_URL

backend/
├── src/
│   └── api/
│       └── v1/
│           └── routes/
│               └── chat.py          # EXISTING: No changes needed
└── .env.example                 # MODIFIED: Document API port

playwright.config.ts                # NEW: Playwright E2E configuration
vitest.config.ts                    # EXISTING: May add theme test config
```

**Structure Decision**: Web application structure with:
- **Frontend**: React/TypeScript with Docusaurus integration
- **Backend**: FastAPI (existing, no core changes for this feature)
- **New Module**: `frontend/config/api.ts` for centralized API configuration
- **New Hook**: `useThemeContext.ts` for detecting Docusaurus theme changes
- **New Tests**: Theme CSS variable tests, API endpoint config tests, comprehensive Playwright E2E suite
- **Configuration**: Environment variables (NODE_ENV, REACT_APP_API_URL) + .env.example documentation

---

## Phase 0: Research & Discovery

### Unknowns Resolved

✅ **All unknowns resolved** - No NEEDS CLARIFICATION markers in specification. Key decisions:

| Area | Decision | Rationale |
|------|----------|-----------|
| **Theme Detection** | Use MutationObserver on `<html>` data-theme attribute | Docusaurus sets `html[data-theme="dark"]` on theme toggle; reliable and minimal overhead |
| **CSS Fallback Colors** | Define fallbacks in chatkit.css for older Docusaurus versions | Graceful degradation if CSS variables undefined; maintains functionality |
| **API Configuration Priority** | REACT_APP_API_URL overrides NODE_ENV | Enables per-deployment flexibility while maintaining sensible defaults |
| **E2E Framework** | Use Playwright (already Docusaurus standard) | Mature, reliable, good mobile/cross-browser support; aligns with project standards |
| **Configuration Module** | Singleton pattern in api.ts | Ensures consistent endpoint across app; prevents accidental differences |
| **Testing Strategy** | Unit tests (Vitest) + E2E tests (Playwright) | Covers component-level theme behavior and user-level workflows |

**Research Deliverables**:
- ✅ Docusaurus CSS variable API documented
- ✅ Theme detection approach validated (MutationObserver vs. useContext)
- ✅ Playwright E2E capabilities verified
- ✅ Environment variable handling in Create React App confirmed
- ✅ CSS custom property browser compatibility verified (all modern browsers)

---

## Phase 1: Design & Contracts

### Data Model

**APIConfiguration Entity**
```typescript
interface APIConfiguration {
  baseURL: string;              // Full API base URL (e.g., http://localhost:8000/api)
  environment: 'development' | 'production' | 'staging';
  isDevelopment: boolean;       // Convenience flag
  isProduction: boolean;        // Convenience flag
  validateURL(): void;          // Validates URL format and HTTPS in production
}
```

**ThemeConfig Entity**
```typescript
interface ThemeConfig {
  isDarkMode: boolean;
  cssVariables: {
    primaryColor: string;       // --ifm-color-primary
    backgroundColor: string;    // --ifm-background-color
    fontColorBase: string;      // --ifm-font-color-base
    borderColor: string;        // --ifm-color-emphasis-200
  };
}
```

### API Contracts

**Theme API** (Internal - CSS variables)
```yaml
# No HTTP endpoint; uses Docusaurus CSS variables
Variables:
  --ifm-color-primary: Used for active states, links, ChatKit primary button
  --ifm-background-color: ChatKit background
  --ifm-font-color-base: ChatKit text color
  --ifm-color-emphasis-200: ChatKit borders, separators
  --ifm-color-gray-900: Dark mode text
```

**API Configuration Module** (`frontend/config/api.ts`)
```typescript
export const getAPIConfig = (): APIConfiguration => {
  // Priority: REACT_APP_API_URL > NODE_ENV defaults
  const customURL = process.env.REACT_APP_API_URL;
  const env = process.env.NODE_ENV;

  return {
    baseURL: customURL || (env === 'production'
      ? 'https://api.yourdomain.com/api'
      : 'http://localhost:8000/api'),
    environment: env as Environment,
    isDevelopment: env === 'development',
    isProduction: env === 'production'
  };
};

export const apiConfig = getAPIConfig();
```

### Generated Artifacts

**research.md** (Phase 0)
- Theme detection mechanism (MutationObserver)
- CSS variable fallback strategy
- Playwright E2E capabilities
- Environment variable handling in CRA

**data-model.md** (Phase 1)
- APIConfiguration interface
- ThemeConfig interface
- State management approach

**contracts/theme-api.yaml** (Phase 1)
- Docusaurus CSS variable mappings
- Fallback color definitions

**contracts/api-config-api.yaml** (Phase 1)
- API configuration endpoint/module
- Environment variable documentation

**quickstart.md** (Phase 1)
- Development setup: `npm install && npm start`
- Production setup: Environment variable configuration
- Testing: Running Playwright E2E tests

---

## Complexity Tracking

> **No Constitution violations** - No tracking needed. All constraints satisfied naturally.

---

## Implementation Approach by User Story

### User Story 1: ChatKit Adapts to Docusaurus Theme (P1)

**Architecture**:
1. Create `useThemeContext.ts` hook that:
   - Listens to `<html data-theme>` changes
   - Resolves CSS variables via `getComputedStyle()`
   - Returns current theme state to components
2. Modify `chatkit.css` to:
   - Replace hardcoded colors with CSS variables
   - Add CSS transitions for smooth color changes
   - Define fallback colors for graceful degradation
3. Update `ChatKitWidget.tsx` to:
   - Call `useThemeContext()` hook
   - Pass theme colors to component styling

**Testing**:
- Unit: `chatkit.test.ts` - CSS variable resolution
- E2E: `theme-switching.spec.ts` - Dark/light mode transitions

---

### User Story 2: Production API Configuration (P1)

**Architecture**:
1. Create `frontend/config/api.ts`:
   - Export `apiConfig` singleton with NODE_ENV logic
   - Support REACT_APP_API_URL override
   - Validate URLs at build time
2. Update `useRAGAPI.ts`:
   - Import `apiConfig` from config module
   - Use configured baseURL for all API calls
   - No changes to component APIs
3. Update `.env.example`:
   - Document REACT_APP_API_URL parameter
   - Show development vs. production examples

**Testing**:
- Unit: `api-config.test.ts` - Configuration loading
- E2E: `api-endpoint.spec.ts` - Verify API calls use correct endpoint

---

### User Story 3: End-to-End Test Coverage (P2)

**Playwright E2E Tests** (8+ scenarios):

1. **Theme Switching** (`theme-switching.spec.ts`)
   - Open ChatKit in light mode
   - Toggle to dark mode → verify colors change
   - Toggle back to light → verify colors revert
   - Multiple toggles → verify no console errors

2. **API Endpoint Configuration** (`api-endpoint.spec.ts`)
   - Build with NODE_ENV=development → verify localhost endpoint
   - Build with REACT_APP_API_URL override → verify custom endpoint
   - Verify API requests go to configured endpoint

3. **Selected Text Integration** (`selected-text.spec.ts`)
   - Select text on page
   - ChatKit opens with selected text in query
   - Selected text passes to backend API

4. **Chapter Context** (`chapter-context.spec.ts`)
   - Navigate to chapter page
   - Chapter context detected
   - Chapter ID/title passed to API
   - Response includes chapter filtering metadata

5. **Mobile Responsive** (`mobile-responsive.spec.ts`)
   - Test viewport widths: 320px, 768px, 1024px
   - ChatKit fully functional on mobile
   - No horizontal scroll, readable text

6. **Message Persistence** (`message-persistence.spec.ts`)
   - Send multiple messages
   - All messages visible in chat history
   - Chapter context maintained across messages

**Testing Configuration**:
- `playwright.config.ts`: Configure browsers, mobile viewports, retry logic
- Base URL: Configured Docusaurus instance
- Screenshot/video capture on failure

---

## Key Architecture Decisions

| Decision | Rationale | Alternatives Considered |
|----------|-----------|------------------------|
| **CSS Variables over Tailwind tokens** | Docusaurus uses native CSS variables; no extra abstraction needed | Could duplicate in Tailwind config, but less maintainable |
| **MutationObserver for theme detection** | Reliable, minimal overhead, detects Docusaurus native toggle | Polling would be inefficient; useContext would require Docusaurus integration |
| **Singleton APIConfiguration** | Ensures consistency across app; loaded once at startup | Per-component config would risk inconsistency |
| **REACT_APP_URL precedence** | Enables per-deployment flexibility without code changes | Could use only NODE_ENV, but less flexible for staging environments |
| **Playwright for E2E** | Mature, good mobile support, already Docusaurus standard | Could use Cypress, but less mature for mobile testing |
| **No backend changes** | Feature is frontend+config only; RAG already works | Could move API config to backend, but adds unnecessary coupling |

---

## Risk Mitigation

| Risk | Severity | Mitigation |
|------|----------|-----------|
| CSS variables not defined in older Docusaurus | Medium | Define fallback colors in chatkit.css; document compatibility requirement |
| API URL misconfigured in production | High | Validate URL format at build time; log configuration on app startup; fail build if invalid |
| Playwright test flakiness | Medium | Use explicit waits, retry logic, mock API responses; run tests against stable staging environment |
| Theme transitions laggy on slow devices | Low | Profile CSS transitions; use GPU acceleration (transform); optimize paint areas |
| REACT_APP_API_URL not injected at build | High | Validate in CI/CD pipeline; fail build if missing in production; document in README |

---

## Phase Outputs

### Phase 0: Research
✅ **Status**: Complete (specification had no unknowns)

**Deliverable**: `research.md`
- Theme detection mechanism selected
- CSS fallback strategy defined
- Playwright capabilities verified
- Environment variable handling confirmed

### Phase 1: Design
⏳ **Status**: IN PROGRESS (this plan defines design)

**Deliverables** (to be created by implementation):
- `data-model.md` - Entity definitions
- `contracts/theme-api.yaml` - Theme CSS variables
- `contracts/api-config-api.yaml` - Configuration module
- `quickstart.md` - Developer setup guide
- Updated agent context file

### Phase 2: Tasks
⏳ **Status**: PENDING (next: run `/sp.tasks`)

**Deliverable**: `tasks.md`
- 12-15 testable, independent tasks
- Task dependencies mapped
- File paths and acceptance criteria defined

---

## Next Step

**Ready for Phase 2**: Run `/sp.tasks` to generate detailed task breakdown with:
- Individual task cards for each requirement
- Task dependencies and sequencing
- File modifications with line numbers
- Test acceptance criteria
- Effort estimates

**Recommendation**: Proceed to `/sp.tasks` to begin implementation phase
