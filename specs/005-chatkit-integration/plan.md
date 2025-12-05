# Implementation Plan: ChatKit Docusaurus Integration

**Branch**: `005-chatkit-integration` | **Date**: 2025-12-06 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `specs/005-chatkit-integration/spec.md`

## Summary

Integrate ChatKit SDK into Docusaurus to provide an interactive RAG-powered chatbot widget embedded in the documentation. Users can ask questions about robotics textbook content and receive context-aware answers with citations from the backend RAG API (feature 004). The widget will support three user stories: P1 basic Q&A, P2 page context enhancement, and P3 selected-text feature. Technical approach: React Provider pattern for mounting in Docusaurus Root.tsx, Client SDK for ChatKit UI/message handling, localStorage for session persistence, CSS variables for dark mode theming, fetch API for backend communication with 30-second timeout and comprehensive error handling.

## Technical Context

**Language/Version**: TypeScript 5+ (React 18+, Docusaurus 3.x)
**Primary Dependencies**: @anthropic/chatkit, react, react-dom, tailwind-css
**Storage**: Browser localStorage for session persistence (10KB limit)
**Testing**: Vitest for unit tests, React Testing Library for component tests, Playwright for E2E
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge 2020+), mobile responsive (320px+)
**Project Type**: Frontend web component (integrated into existing Docusaurus monorepo)
**Performance Goals**: Widget load <3s, backend response latency <5s p95, dark mode toggle <100ms
**Constraints**: Max 2000 character questions, 30s backend timeout, 10KB localStorage limit, no authentication for MVP
**Scale/Scope**: Single widget component, 3 prioritized user stories, 15 functional requirements, 8 success criteria

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I: Production-Grade Quality** ✅ PASS
- Error Handling: Widget will catch backend errors and display user-friendly messages (FR-015)
- Type Safety: TypeScript strict mode with React component types
- Testing: React Testing Library for component tests, Vitest for unit tests, Playwright for E2E
- Graceful Degradation: Widget remains functional if backend unavailable; shows error state
- Performance: Load <3s (SC-001), answer latency <5s p95 (SC-002)

**Principle III: RAG Accuracy & Source Citation** ✅ PASS
- Mandatory Sources: Widget displays sources from backend response as clickable links (FR-005)
- Selected Text Priority: P3 user story implements selected-text feature for RAG relevance boost
- No hallucination risk: Widget displays only backend-generated content, no local generation
- Context window: Page context captured and passed to RAG backend (FR-004)

**Principle IV: Modular & Testable Architecture** ✅ PASS
- Decoupled: Widget is stateless UI component; state in localStorage; communication via REST API
- API-First: Uses existing `/api/v1/chat/ask` endpoint from RAG API (004)
- Dependency Injection: ChatKit configuration injected via props
- Contract Testing: Will test request/response schema matching with backend
- Stateless: Widget state stored in localStorage, not component memory

**Principle VII: Spec-Driven Development** ✅ PASS
- Specification First: Comprehensive spec.md with 3 user stories, 15 FRs, 8 success criteria
- Architecture Planning: This plan.md documents technical approach and decisions
- Task Breakdown: Will create tasks.md with granular implementation steps
- PHRs & ADRs: PHR created for spec phase; ADR will document SDK selection decision

**Complexity Violations**: None. Feature scope is bounded, dependencies are minimal, no production constraints violated.

## Project Structure

### Documentation (this feature)

```text
specs/005-chatkit-integration/
├── spec.md                          # Feature specification (COMPLETE)
├── plan.md                          # This file (architecture & design)
├── research.md                      # Phase 0 output (technology decisions) - TBD
├── data-model.md                    # Phase 1 output (data structures) - TBD
├── quickstart.md                    # Phase 1 output (developer guide) - TBD
├── contracts/                       # Phase 1 output (API contracts) - TBD
│   ├── chatkit-config.schema.json
│   ├── chat-request.schema.json
│   └── chat-response.schema.json
├── checklists/
│   └── requirements.md              # Quality validation (COMPLETE)
└── tasks.md                         # Phase 2 output (/sp.tasks command) - TBD
```

### Source Code (Docusaurus Frontend)

```text
frontend/
├── src/
│   ├── components/
│   │   └── ChatKit/                 # ChatKit widget component (NEW)
│   │       ├── ChatKitWidget.tsx      # Main widget component
│   │       ├── ChatKitProvider.tsx    # Provider wrapper
│   │       ├── hooks/
│   │       │   ├── useChatHistory.ts  # localStorage persistence hook
│   │       │   ├── usePageContext.ts  # Page context detection hook
│   │       │   └── useRAGAPI.ts       # Backend API communication hook
│   │       ├── services/
│   │       │   ├── storageService.ts  # localStorage operations
│   │       │   ├── apiService.ts      # Backend API client
│   │       │   └── pageContextService.ts  # Extract page metadata
│   │       ├── types/
│   │       │   └── chatkit.types.ts   # TypeScript interfaces
│   │       ├── styles/
│   │       │   ├── chatkit.css        # Dark mode + responsive styles
│   │       │   └── themes.css         # Docusaurus theme variables
│   │       └── __tests__/
│   │           ├── ChatKitWidget.test.tsx
│   │           ├── hooks/
│   │           ├── services/
│   │           └── integration/
│   ├── config/
│   │   └── chatkit.config.ts        # ChatKit SDK configuration (NEW)
│   ├── theme/
│   │   └── Root.tsx                 # Root provider integration (MODIFY)
│   └── ...existing structure...
│
├── tests/
│   ├── e2e/
│   │   ├── chatkit-basic.spec.ts     # E2E test: open widget, ask question
│   │   ├── chatkit-dark-mode.spec.ts # E2E test: dark mode toggle
│   │   ├── chatkit-mobile.spec.ts    # E2E test: mobile responsiveness
│   │   └── chatkit-rag-integration.spec.ts # E2E test: RAG backend
│   └── ...existing structure...
│
├── package.json                     # Update dependencies (add @anthropic/chatkit)
├── tsconfig.json                    # Ensure strict mode enabled
└── ...existing Docusaurus files...
```

**Structure Decision**:
- **Integration Point**: Root.tsx - wrap entire app with ChatKitProvider for global context
- **Component Location**: `src/components/ChatKit/` - self-contained, testable widget
- **State Management**: localStorage for persistence + React Context for session state
- **API Layer**: Dedicated `apiService.ts` for backend communication with error handling
- **Testing**: Vitest for unit tests, React Testing Library for components, Playwright for E2E
- **Dependencies**: Add `@anthropic/chatkit` via `npm install` (or equivalent)

## Key Architectural Decisions

### 1. SDK Selection: ChatKit vs Alternatives

**Decision**: Use Anthropic ChatKit SDK for ChatBot widget

**Why**:
- **Direct Integration**: ChatKit SDK provides pre-built React Provider and UI components, reducing custom work
- **LLM Context**: Integrates with Claude models for context understanding (not used in MVP, but available for future)
- **Open Standards**: Works with any OpenAI-compatible API endpoint (our RAG API)
- **Type Safety**: Full TypeScript support with proper interfaces

**Alternatives Considered**:
- OpenAI Assistants API: Would require custom UI building (more work)
- Vercel AI SDK: Generic, less tailored for Docusaurus integration
- Custom React Component: Full control but significant dev time (estimate 40+ hours)

**Trade-off**: ChatKit may have opinionated UI; customization via theming will be needed for Docusaurus match

### 2. State Management: localStorage + React Context

**Decision**: Use browser localStorage for persistent chat history + React Context for session state

**Why**:
- **Session Persistence**: Users expect chat history to survive page refresh
- **No Backend Dependency**: Don't require database for MVP (simplifies deployment)
- **Privacy**: Chat stays on user's device; not sent to server
- **Simplicity**: No auth system needed for MVP

**Constraints**:
- 10KB localStorage limit (approximately 50-100 messages)
- Not shareable across devices
- Cleared when user deletes browser data

### 3. Page Context Detection: URL + Document Analysis

**Decision**: Extract page context from Docusaurus sidebar data + document title

**Why**:
- **Docusaurus Native**: Docusaurus stores sidebar/page metadata as window data
- **No Extra Dependencies**: Use existing theme context
- **Accurate Detection**: Chapter/section info already in page structure

**Implementation**:
- Hook into Docusaurus theme context for current page metadata
- Extract chapter/section from URL pathname + sidebar data
- Pass as metadata in RAG request

### 4. Dark Mode Support: CSS Variables

**Decision**: Use CSS variables that reference Docusaurus theme tokens

**Why**:
- **Auto-sync**: When user toggles dark mode, widget updates automatically
- **Consistent**: Widget colors match Docusaurus design system
- **No JavaScript Logic**: Pure CSS, fast theme switching

### 5. Backend Communication: Fetch API with Timeout

**Decision**: Use native fetch API with AbortController for timeout handling

**Why**:
- **No Dependencies**: Fetch is built-in to modern browsers
- **Timeout Support**: AbortController provides clean timeout mechanism
- **Simple Error Handling**: Easy to catch and display errors

**Timeout Strategy**:
- 30-second timeout per spec requirement (FR-010)
- Show loading indicator immediately
- Display user-friendly timeout error after 30s

## Implementation Phases

### Phase 0: Research & Technology Validation (TBD via /sp.plan)

**Outputs**: research.md

**Tasks**:
- Research @anthropic/chatkit latest version and API documentation
- Validate ChatKit works with Docusaurus 3.x React context system
- Test ChatKit theming/styling customization options
- Confirm localStorage capacity for MVP use case

### Phase 1: Design & Data Modeling (TBD via /sp.plan)

**Outputs**: data-model.md, contracts/, quickstart.md

**Tasks**:
- Document ChatMessage, ChatSession, RAGRequest, RAGResponse entities
- Generate JSON Schema contracts for backend communication
- Design component API and prop interfaces
- Create developer quickstart guide

### Phase 2: Task Generation (/sp.tasks command)

**Output**: tasks.md

Will break specification into 30-50 granular tasks:
- Setup & dependencies (npm install, tsconfig)
- Configuration (chatkit.config.ts)
- Core components (ChatKitWidget, ChatKitProvider)
- Hooks (useChatHistory, usePageContext, useRAGAPI)
- Services (storage, API, pageContext)
- Styling (dark mode, mobile responsive)
- Testing (unit, component, E2E)
- Integration (Root.tsx modification)

### Phase 3: Implementation (red/green/refactor pattern)

Tasks will be executed in dependency order:
1. Setup phase (days 1-2): Scaffold component structure, setup tests
2. Core feature (days 3-5): Implement ChatKit widget, backend communication
3. Enhancement (days 6-7): Page context, dark mode, selected text
4. Testing (days 8-9): Unit tests, component tests, E2E tests
5. Polish (day 10): Error handling, accessibility, documentation

## Dependencies & Risk Management

### External Dependencies

| Dependency | Version | Risk | Mitigation |
|------------|---------|------|-----------|
| @anthropic/chatkit | Latest | API breaking changes | Pin version, test upgrades |
| React | 18+ | Context API changes | Docusaurus 3.x includes React 18+ |
| Docusaurus | 3.x | Theme API changes | Use theme/Root.tsx pattern (standard) |
| Backend RAG API | Feature 004 | API downtime | Error handling, graceful degradation (FR-015) |

### Integration Points & Risks

1. **Root.tsx Integration**
   - Risk: Provider wrapping breaks existing theme/layout
   - Mitigation: Test with existing pages, revert if issues

2. **Docusaurus Theme Context**
   - Risk: Theme API might differ between versions
   - Mitigation: Use standard pattern documented in Docusaurus docs

3. **Backend API Communication**
   - Risk: API might be unavailable, slow, or return errors
   - Mitigation: Timeout handling (30s), error messages, offline detection

### Non-Risks (Clarified in Assumptions)

- **No auth**: MVP assumes anonymous users (simplifies implementation)
- **No persistence to DB**: localStorage only (reduces scope)
- **No advanced ChatKit features**: Use basic chat widget (reduce complexity)
- **No mobile app**: Web only (Docusaurus is web)

## Success Metrics & Validation

**Specification Success Criteria** (from spec.md):
- SC-001: Load in <3 seconds ✓ (performance measurement)
- SC-002: Backend latency <5s p95 ✓ (includes backend response time)
- SC-003: Works on mobile 320px+ ✓ (Playwright E2E test)
- SC-004: 100% error handling ✓ (test error scenarios)
- SC-005: History persistence & clearing ✓ (test localStorage)
- SC-006: 90% user task completion ✓ (manual testing)
- SC-007: WCAG AA contrast ✓ (axe accessibility tool)
- SC-008: 95% selected-text capture ✓ (integration test)

**Testing Strategy**:
- Unit tests: Hooks and services (Vitest)
- Component tests: ChatKitWidget render, input validation (React Testing Library)
- E2E tests: Full user flows (Playwright)
- Accessibility: axe-core automated checks + manual review
- Performance: Lighthouse CI for page load, custom timing for API calls

## Next Steps

1. **Execute `/sp.plan` Phase 0**: Run research tasks, create research.md
2. **Execute `/sp.plan` Phase 1**: Design data model, create contracts, quickstart
3. **Execute `/sp.tasks`**: Generate 30-50 implementation tasks
4. **Execute `/sp.implement`**: Implement tasks in phases (setup → core → enhance → test → polish)
