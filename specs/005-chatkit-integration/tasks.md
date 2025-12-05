# Implementation Tasks: ChatKit Docusaurus Integration

**Feature**: ChatKit Docusaurus Integration (005-chatkit-integration)
**Branch**: `005-chatkit-integration`
**Date**: 2025-12-06
**Status**: Ready for Implementation
**Total Tasks**: 48 tasks across 6 phases

---

## Summary

This document contains 48 implementation tasks organized by user story (P1, P2, P3) and cross-cutting concerns. Tasks are ordered for parallel execution where possible. Each task includes:
- Unique ID (T001-T048)
- Story label ([US1], [US2], [US3]) for story-specific tasks
- Parallel indicator [P] where applicable
- File paths for all code changes
- Acceptance criteria

**MVP Scope**: User Story 1 (P1) - Basic Q&A with chat history and error handling
**Estimated Duration**: 40-50 hours across 2-3 weeks with one developer

---

## Phase 1: Setup & Project Initialization (4 tasks)

> **Goal**: Initialize project structure, install dependencies, setup TypeScript configuration
> **Independent Test**: `npm install` succeeds, types compile with `tsc --noEmit`, file structure matches plan

### Setup Tasks

- [ ] T001 Install ChatKit SDK and dependencies in frontend/package.json

  **File**: `frontend/package.json`

  **Acceptance Criteria**:
  - npm install @anthropic/chatkit succeeds
  - npm install vitest @testing-library/react --save-dev succeeds
  - All dependencies resolve without conflicts
  - package-lock.json updated and committed

- [ ] T002 Verify and update TypeScript strict mode configuration

  **File**: `frontend/tsconfig.json`

  **Acceptance Criteria**:
  - "strict": true is set
  - "noImplicitAny": true is set
  - "noUnusedLocals": true is set
  - "noUnusedParameters": true is set
  - tsc --noEmit compiles without errors

- [ ] T003 Create ChatKit component directory structure

  **Files**:
  - `frontend/src/components/ChatKit/` (create directory)
  - `frontend/src/components/ChatKit/hooks/` (create directory)
  - `frontend/src/components/ChatKit/services/` (create directory)
  - `frontend/src/components/ChatKit/types/` (create directory)
  - `frontend/src/components/ChatKit/styles/` (create directory)
  - `frontend/src/components/ChatKit/__tests__/` (create directory)

  **Acceptance Criteria**:
  - All directories created successfully
  - Directory structure matches plan.md project structure section
  - Can list directory tree: `ls -R frontend/src/components/ChatKit/`

- [ ] T004 Create ChatKit configuration file with environment variables

  **File**: `frontend/src/config/chatkit.config.ts`

  **Acceptance Criteria**:
  - File exports chatKitConfig object
  - Contains apiEndpoint: process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000/api/v1/chat/ask'
  - Contains position, defaultOpen, darkMode, enableSelectedText, maxHistoryMessages
  - Matches configuration structure from quickstart.md
  - TypeScript compiles without errors

---

## Phase 2: Foundational Services & Types (9 tasks)

> **Goal**: Implement shared types, entities, and service layer that all user stories depend on
> **Independent Test**: All types compile, services export functions, no circular dependencies

### Foundational Tasks

- [ ] T005 [P] Define TypeScript types and interfaces for ChatKit entities

  **File**: `frontend/src/components/ChatKit/types/chatkit.types.ts`

  **Acceptance Criteria**:
  - Defines ChatMessage interface with id, role, content, timestamp, sessionId, status fields
  - Defines ChatSession interface with sessionId, createdAt, lastMessageAt, messages array
  - Defines PageContext interface with url, pathname, chapter, section, confidence
  - Defines RAGRequest interface with question, selectedText?, pageContext?, sessionId?
  - Defines RAGResponse interface with answer, sources[], confidence, metadata
  - Defines SourceReference interface with id, title, snippet, url, similarity
  - Defines ResponseMetadata interface with latency fields and model info
  - All types export for use in other modules
  - No TypeScript compilation errors (tsc --noEmit passes)

- [ ] T006 [P] Implement storage service for localStorage persistence

  **File**: `frontend/src/components/ChatKit/services/storageService.ts`

  **Acceptance Criteria**:
  - Exports saveSession(session: ChatSession): void function
  - Exports loadSession(sessionId: string): ChatSession | null function
  - Exports getActiveSessionId(): string | null function
  - Exports clearHistory(): void function
  - saveSession stores session as JSON with key `chatkit-session-{sessionId}`
  - loadSession retrieves and parses session from localStorage
  - getActiveSessionId retrieves active session UUID from localStorage
  - clearHistory removes all keys starting with 'chatkit-'
  - All functions handle errors gracefully (null/empty checks)
  - No direct window.localStorage access outside this file
  - Unit tests pass in storageService.test.ts

- [ ] T007 [P] Implement API service for backend communication with timeout

  **File**: `frontend/src/components/ChatKit/services/apiService.ts`

  **Acceptance Criteria**:
  - Exports sendQuestion(request: RAGRequest, timeoutMs?: number): Promise<RAGResponse>
  - Uses fetch API with POST method to /api/v1/chat/ask
  - Implements AbortController for timeout handling
  - Default timeout: 30 seconds (per FR-010)
  - Sends Content-Type: application/json header
  - Catches AbortError and throws timeout error message
  - Catches network errors and throws descriptive error
  - Parses response JSON and returns RAGResponse object
  - No API key validation needed (MVP, no auth)
  - Unit tests pass in apiService.test.ts

- [ ] T008 [P] Implement page context service to extract Docusaurus metadata

  **File**: `frontend/src/components/ChatKit/services/pageContextService.ts`

  **Acceptance Criteria**:
  - Exports getPageContext(): PageContext function
  - Returns object with url: window.location.href, pathname: window.location.pathname
  - Attempts to extract chapter/section from Docusaurus sidebar/breadcrumb
  - Sets confidence based on extraction success: 'high' | 'medium' | 'low'
  - Gracefully handles missing Docusaurus context (returns low confidence with partial data)
  - Works on any Docusaurus page (not just chapter pages)
  - Returns consistent format matching PageContext interface
  - Unit tests pass in pageContextService.test.ts

- [ ] T009 [P] Implement useRAGAPI hook for backend communication

  **File**: `frontend/src/components/ChatKit/hooks/useRAGAPI.ts`

  **Acceptance Criteria**:
  - Exports hook that returns { sendQuestion, isLoading, error }
  - sendQuestion(question, selectedText?, pageContext?): Promise<RAGResponse | null>
  - Sets isLoading=true when request starts, false when complete
  - Clears error before request, sets error message on failure
  - Calls apiService.sendQuestion() with timeout
  - Returns RAGResponse on success, null on failure
  - Works with AbortController cleanup on unmount
  - Handles network timeouts (>30s) with user-friendly error message
  - No external dependencies except services and types
  - Hook tests pass in hooks/useRAGAPI.test.tsx

- [ ] T010 [P] Implement useChatHistory hook for session persistence

  **File**: `frontend/src/components/ChatKit/hooks/useChatHistory.ts`

  **Acceptance Criteria**:
  - Exports hook that returns { session, addMessage, clearHistory }
  - session: ChatSession | null (null until loaded from localStorage)
  - addMessage(message: ChatMessage): void adds message and saves to localStorage
  - clearHistory(): void clears all history and creates new session
  - Loads active session on component mount
  - Creates new session if none exists
  - Updates lastMessageAt timestamp on each message
  - Saves session to localStorage after each update
  - Handles localStorage quota errors gracefully
  - Hook tests pass in hooks/useChatHistory.test.tsx

- [ ] T011 [P] Implement usePageContext hook for tracking current page

  **File**: `frontend/src/components/ChatKit/hooks/usePageContext.ts`

  **Acceptance Criteria**:
  - Exports hook that returns PageContext | null
  - Initializes context on component mount
  - Updates context when window.location.pathname changes
  - Calls pageContextService.getPageContext() to get metadata
  - Returns null until context is available
  - Handles missing Docusaurus context gracefully
  - No circular dependencies with other hooks
  - Hook tests pass in hooks/usePageContext.test.tsx

- [ ] T012 [P] Create unit tests for storage, API, and page context services

  **Files**:
  - `frontend/src/components/ChatKit/__tests__/services/storageService.test.ts`
  - `frontend/src/components/ChatKit/__tests__/services/apiService.test.ts`
  - `frontend/src/components/ChatKit/__tests__/services/pageContextService.test.ts`

  **Acceptance Criteria**:
  - storageService tests: saveSession, loadSession, clearHistory, getActiveSessionId
  - apiService tests: successful response, timeout error, network error, invalid response
  - pageContextService tests: extraction with/without Docusaurus context, confidence levels
  - All tests use mocked window/fetch objects
  - Tests use Vitest: `npm run test:services` passes
  - Coverage >80% for service modules
  - No integration with actual backend or localStorage

- [ ] T013 [P] Create unit tests for React hooks (useChatHistory, usePageContext, useRAGAPI)

  **Files**:
  - `frontend/src/components/ChatKit/__tests__/hooks/useChatHistory.test.tsx`
  - `frontend/src/components/ChatKit/__tests__/hooks/usePageContext.test.tsx`
  - `frontend/src/components/ChatKit/__tests__/hooks/useRAGAPI.test.tsx`

  **Acceptance Criteria**:
  - useChatHistory tests: session creation, addMessage, clearHistory, localStorage save/load
  - usePageContext tests: context extraction, pathname change detection, update on route change
  - useRAGAPI tests: successful API call, error handling, timeout handling, loading state
  - Tests use React Testing Library renderHook() function
  - All tests mock storageService, apiService, pageContextService
  - Tests check: useState updates, useEffect side effects, cleanup
  - Tests use Vitest: `npm run test:hooks` passes
  - Coverage >80% for hook modules

---

## Phase 3: User Story 1 - Access RAG Chatbot from Documentation (P1, 13 tasks)

> **Goal**: Implement basic chat widget, backend integration, history persistence, error handling
> **Independent Test**: User opens Docusaurus page → clicks chat icon → asks question → receives answer with sources → history persists
> **Acceptance**: Can complete all 5 P1 acceptance scenarios from spec.md
> **MVP Scope**: This phase alone constitutes a viable MVP

### US1: Widget Component & Initialization

- [ ] T014 [US1] Create ChatKitWidget.tsx main component with input and message display

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx`

  **Acceptance Criteria**:
  - Component displays in fixed bottom-right position (FR-002)
  - Contains message list container showing ChatMessage[] (empty on first load)
  - Contains input field for question (validates 1-2000 chars per FR-008)
  - Contains send button (disabled while isLoading or input empty)
  - Contains clear history button (FR-014)
  - Renders ChatMessage with role-specific styling (user left, assistant right)
  - Displays loading indicator while awaiting response (FR-009)
  - Renders error message in red when error present (FR-015)
  - Uses useRAGAPI, useChatHistory, usePageContext hooks
  - TypeScript strict mode compilation passes
  - Component tests pass in ChatKitWidget.test.tsx

- [ ] T015 [US1] Implement message sending logic with backend API integration

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (extends T014)

  **Acceptance Criteria**:
  - handleSendQuestion() function triggered by send button click
  - Validates input: reject empty or whitespace-only questions
  - Creates ChatMessage with role='user', content=input, timestamp, sessionId
  - Adds user message to chat history via useChatHistory hook
  - Clears input field after sending
  - Calls useRAGAPI.sendQuestion() with question, selectedText, pageContext
  - Creates ChatMessage with role='assistant' when response received (FR-005)
  - Displays sources as clickable links with target="_blank"
  - Creates error ChatMessage if API call fails (FR-015)
  - Shows timeout error after 30 seconds (FR-010)
  - Shows generic error if network unavailable (graceful degradation)
  - No console errors or warnings

- [ ] T016 [US1] Implement message history persistence across page navigations

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (extends T014-T015)

  **Acceptance Criteria**:
  - useChatHistory hook loads session on component mount
  - Displays all messages from session.messages array (FR-007)
  - addMessage() saves each new message to localStorage
  - useEffect with useCallback ensures no memory leaks
  - Session persists when user navigates between pages
  - Session persists on page reload (browser refresh)
  - Clear history button calls clearHistory() and removes all messages
  - localStorage quota errors handled gracefully (error message shown)
  - Message order preserved (timestamp order)

- [ ] T017 [US1] Implement dark mode support with CSS variables

  **File**: `frontend/src/components/ChatKit/styles/chatkit.css`

  **Acceptance Criteria**:
  - Define CSS variables: --chatkit-primary, --chatkit-bg, --chatkit-text, --chatkit-border
  - Light mode defaults: white bg, dark text, light border
  - Dark mode ([data-theme='dark']): dark bg, light text, light border
  - Apply variables to widget container, messages, input, buttons
  - Colors maintain WCAG AA contrast (4.5:1 for text, 3:1 for borders) (FR-006, SC-007)
  - Dark mode toggle in Docusaurus automatically updates [data-theme] attribute
  - No hardcoded colors in component styles
  - Test dark mode with browser dev tools theme switch
  - Axe accessibility check passes (axe-core)

- [ ] T018 [US1] Implement responsive mobile layout for 320px+ screens

  **File**: `frontend/src/components/ChatKit/styles/chatkit.css` (extends T017)

  **Acceptance Criteria**:
  - Desktop (>640px): fixed 350px width, bottom-right corner (FR-002)
  - Mobile (<640px): full width with padding, stacks vertically (FR-013)
  - Touch events work equivalently to mouse (no touch-specific bugs)
  - Input field grows as user types (multiline support)
  - Message list scrolls within fixed height container
  - Buttons accessible with touch (48px minimum tap target)
  - Test on 320px, 375px, 640px, 1920px viewport widths
  - Lighthouse mobile score >90

- [ ] T019 [US1] Implement error handling and user-friendly error messages

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (extends T014-T018)

  **Acceptance Criteria**:
  - Backend unavailable (503): "Service unavailable. Please try again later." (FR-015, US1-5)
  - Network timeout (>30s): "Your request timed out. Please try again." (FR-010)
  - Invalid input (empty): "Please enter a question" (FR-008)
  - Long question (>2000): "Question too long (max 2000 characters)"
  - No technical error details exposed to user (no API errors, stack traces)
  - Error message displayed in red error state for 5 seconds then dismissible
  - User can retry after error (question text preserved)
  - All error scenarios tested in error.test.tsx
  - No console.log of error details (proper logging only)

- [ ] T020 [US1] [P] Create ChatKitProvider.tsx wrapper for context

  **File**: `frontend/src/components/ChatKit/ChatKitProvider.tsx`

  **Acceptance Criteria**:
  - Exports ChatKitProvider component and ChatKitContext
  - Wraps children with ChatKitContext.Provider (for future global state)
  - Renders ChatKitWidget as child (always visible)
  - Accepts children as prop and renders them unchanged
  - No side effects that block rendering
  - Provider tests pass in ChatKitProvider.test.tsx

- [ ] T021 [US1] [P] Integrate ChatKitProvider into Root.tsx

  **File**: `frontend/src/theme/Root.tsx`

  **Acceptance Criteria**:
  - Import ChatKitProvider from components/ChatKit
  - Wrap children with ChatKitProvider (e.g., `<ChatKitProvider>{children}</ChatKitProvider>`)
  - Existing theme/layout components work unchanged
  - ChatKitWidget appears on every page after Root.tsx renders
  - No breaking changes to existing Docusaurus functionality
  - Build succeeds: `npm run build`
  - Manual testing: Widget visible on all documentation pages

- [ ] T022 [US1] [P] Create component tests for ChatKitWidget

  **Files**:
  - `frontend/src/components/ChatKit/__tests__/ChatKitWidget.test.tsx`
  - `frontend/src/components/ChatKit/__tests__/ChatKitProvider.test.tsx`

  **Acceptance Criteria**:
  - ChatKitWidget tests: render, input validation, send button, message display, error display
  - Test user opens widget on page load (visible in DOM)
  - Test user types question and clicks send
  - Test user sees loading indicator while response pending
  - Test user sees answer message with source links
  - Test user sees error message if backend fails
  - Test user can clear history with clear button
  - ChatKitProvider tests: renders children, renders ChatKitWidget
  - Use React Testing Library userEvent for interactions
  - Mock useRAGAPI, useChatHistory hooks with test data
  - Tests pass: `npm run test:components`
  - Coverage >80% for ChatKitWidget and ChatKitProvider

- [ ] T023 [US1] [P] Create E2E test for basic chat workflow

  **File**: `frontend/tests/e2e/chatkit-basic.spec.ts` (Playwright)

  **Acceptance Criteria**:
  - Test: Open Docusaurus page → find chat widget → send question → receive answer
  - Navigate to: http://localhost:3000/docs/chapter-1 (or any chapter page)
  - Verify ChatKit widget visible in bottom-right corner
  - Type question: "What is forward kinematics?"
  - Click send button
  - Wait for response (max 5 seconds per SC-002)
  - Verify answer text appears in chat
  - Verify sources array rendered as clickable links
  - Verify message history shows both user message and assistant response
  - Test passes: `npx playwright test chatkit-basic.spec.ts`
  - Test runs against backend at http://localhost:8000 (must be running)

- [ ] T024 [US1] [P] Create E2E test for history persistence

  **File**: `frontend/tests/e2e/chatkit-history.spec.ts` (Playwright)

  **Acceptance Criteria**:
  - Test: Send message → navigate to different page → return → verify history persists
  - Start on Chapter 1 page, send question "What is ROS 2?"
  - Verify message appears in chat history
  - Navigate to Chapter 2 page (different URL)
  - Verify chat widget is visible on Chapter 2
  - Verify previous message "What is ROS 2?" still appears in history
  - Navigate back to Chapter 1
  - Verify complete history (both Chapter 1 and Chapter 2 messages) preserved
  - Reload page (browser refresh) and verify history still present
  - Click clear history button and verify all messages removed
  - Test passes: `npx playwright test chatkit-history.spec.ts`

- [ ] T025 [US1] [P] Create E2E test for error handling

  **File**: `frontend/tests/e2e/chatkit-errors.spec.ts` (Playwright)

  **Acceptance Criteria**:
  - Test 1: Backend unavailable → user sees error message (FR-015, US1-5)
    - Stop backend API service
    - Send question from widget
    - Verify error message: "Service unavailable..."
    - Restart backend
  - Test 2: Network timeout
    - Mock slow backend response (>30s delay)
    - Send question
    - Wait 30+ seconds
    - Verify timeout error message appears
  - Test 3: Empty question validation (FR-008)
    - Click send without entering text
    - Verify error: "Please enter a question"
  - Test 4: Very long question (>2000 chars)
    - Paste 2500 character text
    - Verify error: "Question too long..."
  - All tests pass: `npx playwright test chatkit-errors.spec.ts`

- [ ] T026 [US1] Verify Phase 1 acceptance criteria

  **No coding required** - Review & Manual Testing

  **Acceptance Criteria**:
  - Can complete all 5 P1 acceptance scenarios from spec.md:
    1. ✅ User clicks ChatKit widget button → window opens in bottom-right
    2. ✅ User types question and presses send → question sent to backend
    3. ✅ Backend returns response → sources displayed as clickable links
    4. ✅ User closes widget and navigates → history persists and visible upon reopen
    5. ✅ Backend unavailable → user sees friendly error message
  - Widget loads in <3 seconds (SC-001)
  - Backend latency <5s p95 (SC-002)
  - All unit tests pass: `npm run test`
  - All E2E tests pass: `npm run test:e2e`
  - No console errors or warnings
  - Manual testing on Chrome, Firefox, Safari, Edge
  - Manual testing on mobile (320px, 375px viewports)
  - Accessibility check passes: `npx axe-core http://localhost:3000/docs`

---

## Phase 4: User Story 2 - Use Page Context for Better Answers (P2, 8 tasks)

> **Goal**: Capture page metadata and pass to RAG backend for contextual answers
> **Independent Test**: User on Chapter 1 page asks question → backend receives chapter/section metadata → answer mentions Chapter 1
> **Acceptance**: Can complete all 3 P2 acceptance scenarios from spec.md
> **Dependency**: Phase 3 (US1) must be complete

### US2: Page Context Detection & Integration

- [ ] T027 [US2] Enhance PageContext detection for Docusaurus chapter/section extraction

  **File**: `frontend/src/components/ChatKit/services/pageContextService.ts` (extends T008)

  **Acceptance Criteria**:
  - getCurrentChapterFromSidebar(): string | null function extracts chapter title from Docusaurus sidebar
  - getCurrentSectionFromBreadcrumb(): string | null function extracts section from breadcrumb
  - pageContext.confidence set to:
    - 'high': chapter and section both extracted successfully
    - 'medium': only chapter or only URL extracted
    - 'low': only URL available (no chapter/section metadata found)
  - Works on all chapter pages in documentation
  - Works even if sidebar/breadcrumb hidden or styled unusually
  - Returns consistent format matching spec: `{url, pathname, chapter?, section?, confidence}`
  - Unit tests pass: `npm run test:services`

- [ ] T028 [US2] Update ChatKitWidget to capture and pass page context to backend

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (extends T014-T019)

  **Acceptance Criteria**:
  - usePageContext hook returns PageContext object (from T011)
  - handleSendQuestion() passes pageContext to useRAGAPI.sendQuestion()
  - RAGRequest includes: question, selectedText?, pageContext
  - Backend receives request with chapter/section metadata (verify via network tab)
  - Backend can use context to improve search relevance
  - Component handles pageContext=null gracefully (if extraction fails)
  - Context captured at time of question (timestamp matters for consistency)
  - No impact on send latency (<5s still achievable, SC-002)

- [ ] T029 [US2] Update API service to include pageContext in RAGRequest

  **File**: `frontend/src/components/ChatKit/services/apiService.ts` (extends T007)

  **Acceptance Criteria**:
  - sendQuestion(question, selectedText?, pageContext?) accepts pageContext parameter
  - pageContext serialized to JSON in request body
  - Request body: `{question, selectedText?, pageContext, sessionId?}`
  - Matches RAGRequest schema from contracts/chat-request.schema.json
  - Backend receives all context fields (url, pathname, chapter, section, confidence)
  - pageContext optional (API call succeeds without it, for backward compatibility)

- [ ] T030 [US2] Create unit test for page context extraction

  **File**: `frontend/src/components/ChatKit/__tests__/services/pageContextService.test.ts` (extends T012)

  **Acceptance Criteria**:
  - Test successful chapter/section extraction: high confidence
  - Test partial extraction (only URL): medium confidence
  - Test no extraction available (new page): low confidence
  - Mock Docusaurus sidebar/breadcrumb objects
  - Test URL pathname parsing
  - Test context update on pathname change
  - Tests pass: `npm run test:services`
  - Coverage >80% for pageContextService

- [ ] T031 [US2] Create E2E test for page context integration

  **File**: `frontend/tests/e2e/chatkit-page-context.spec.ts` (Playwright)

  **Acceptance Criteria**:
  - Test: Navigate to Chapter 1 page → send question → verify backend receives chapter context
  - Open browser DevTools Network tab
  - Navigate to: http://localhost:3000/docs/chapter-1-intro-ros2
  - Verify sidebar shows "Chapter 1 - Introduction to ROS 2"
  - Send question: "Explain this chapter"
  - Inspect POST request to /api/v1/chat/ask in Network tab
  - Verify request body includes: `pageContext: {chapter: "Chapter 1...", section: "...", confidence: "high"}`
  - Verify response answer mentions Chapter 1 or ROS 2 specifics
  - Repeat on Chapter 3 page to verify context changes
  - Test passes: `npx playwright test chatkit-page-context.spec.ts`

- [ ] T032 [US2] Create E2E test for dark mode toggle persistence

  **File**: `frontend/tests/e2e/chatkit-dark-mode.spec.ts` (Playwright)

  **Acceptance Criteria**:
  - Test: Toggle dark mode → verify widget colors update → verify colors persist
  - Load page in light mode
  - Verify widget has light background (white or light gray)
  - Click Docusaurus dark mode toggle (usually top-right corner)
  - Verify widget background changed to dark (dark gray or black)
  - Verify text color changed to light for contrast
  - Reload page
  - Verify dark mode colors still applied (persisted via Docusaurus theme storage)
  - Toggle back to light mode
  - Verify colors revert to light theme
  - Axe accessibility check on both light and dark modes (SC-007)
  - Test passes: `npx playwright test chatkit-dark-mode.spec.ts`

- [ ] T033 [US2] Update component tests for page context integration

  **File**: `frontend/src/components/ChatKit/__tests__/ChatKitWidget.test.tsx` (extends T022)

  **Acceptance Criteria**:
  - Add test: useChatHistory hook integration with page context
  - Mock usePageContext to return sample context
  - Verify pageContext passed to useRAGAPI.sendQuestion()
  - Verify ChatMessage stores pageContext in metadata
  - Test different pageContext.confidence levels (high, medium, low)
  - Verify component handles missing pageContext gracefully
  - New tests pass: `npm run test:components`

- [ ] T034 [US2] Verify Phase 2 acceptance criteria

  **No coding required** - Review & Manual Testing

  **Acceptance Criteria**:
  - Can complete all 3 P2 acceptance scenarios from spec.md:
    1. ✅ User on Chapter 1 page sends question → request includes chapter/section metadata
    2. ✅ Page context available → RAG pipeline uses to improve search relevance
    3. ✅ User asks follow-up on same page → context captured again, consistent
  - Page context confidence level correct ('high', 'medium', or 'low')
  - Backend receives all context fields in RAGRequest
  - Answer quality improved with context (subjective, verified manually)
  - E2E tests pass: `npm run test:e2e:us2`
  - No regression in US1 functionality (all P1 tests still pass)
  - Performance target maintained: <5s p95 latency (SC-002)

---

## Phase 5: User Story 3 - Select Text and Ask Questions (P3, 11 tasks)

> **Goal**: Implement right-click context menu to ask about selected text
> **Independent Test**: User selects text → right-click → "Ask ChatKit" option → question modal with selected text → backend receives selectedText
> **Acceptance**: Can complete all 3 P3 acceptance scenarios from spec.md
> **Dependency**: Phase 3 (US1) must be complete

### US3: Selected Text Context Feature

- [ ] T035 [US3] Implement selected text detection and context menu

  **File**: `frontend/src/components/ChatKit/services/selectedTextService.ts` (new file)

  **Acceptance Criteria**:
  - Exports detectSelectedText(): string | null function
  - Returns selected text if any, null otherwise
  - Works with window.getSelection() API (cross-browser)
  - Trims whitespace from selection
  - Returns null for empty or whitespace-only selections
  - Does not modify original text
  - Unit tests pass: `npm run test:services`

- [ ] T036 [US3] Create right-click context menu for selected text

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (extends T014-T019)

  **Acceptance Criteria**:
  - Add useEffect that listens for 'contextmenu' event on document
  - When user right-clicks with text selected:
    - Show context menu near cursor position
    - Menu includes "Ask ChatKit about this" option
    - Styled to match Docusaurus theme
  - When "Ask ChatKit about this" clicked:
    - Store selected text in state variable
    - Scroll chat widget into view (if off-screen)
    - Focus question input field (FR-012)
  - Context menu disappears after selection or clicking elsewhere
  - Works only when text is selected (disabled if no selection)
  - No impact on default browser context menu for other elements
  - Does not interfere with text selection workflow

- [ ] T037 [US3] Create question input modal with pre-filled selected text

  **File**: `frontend/src/components/ChatKit/components/QuestionModal.tsx` (new file)

  **Acceptance Criteria**:
  - Component displays modal/dialog when selectedText is available
  - Shows "Ask about this text" header
  - Displays selected text in read-only format (gray background, smaller font)
  - Contains textarea for user's question (empty initially)
  - Question field placeholder: "What would you like to know about this text?"
  - Contains "Ask" button and "Cancel" button
  - "Ask" sends question with selectedText to backend
  - "Cancel" closes modal without sending
  - Pressing Escape also closes modal
  - Modal styled consistently with chatbot theme
  - Modal accessible (keyboard navigation, screen reader support)

- [ ] T038 [US3] Update ChatKitWidget to integrate selected text modal

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (extends T035-T037)

  **Acceptance Criteria**:
  - Add selectedText state variable
  - Render QuestionModal component (hidden when selectedText=null)
  - When user selects "Ask ChatKit about this" from context menu:
    - Set selectedText state
    - Show modal
  - When user submits question from modal:
    - Add selectedText to message metadata
    - Call useRAGAPI.sendQuestion() with selectedText parameter (FR-012)
    - Close modal
    - Clear selectedText state
  - Display selected text in chat history (show what user asked about)
  - Backend receives RAGRequest with selectedText field
  - No impact on regular (non-selected-text) question flow

- [ ] T039 [US3] Update API service to handle selectedText in requests

  **File**: `frontend/src/components/ChatKit/services/apiService.ts` (extends T007, T029)

  **Acceptance Criteria**:
  - sendQuestion(question, selectedText?, pageContext?, sessionId?) signature unchanged
  - selectedText serialized in RAGRequest body
  - Request body: `{question, selectedText?, pageContext?, sessionId?}`
  - Matches RAGRequest schema: chat-request.schema.json
  - selectedText sent to backend for RAG relevance boosting
  - Backend can use selectedText to narrow search scope (per FR-012)
  - API call succeeds with or without selectedText (optional parameter)

- [ ] T040 [US3] Store selected text in chat history for reference

  **File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx` (extends T038)

  **Acceptance Criteria**:
  - ChatMessage.selectedText field stores text user selected (FR-012)
  - When rendering message, show selected text source (e.g., "Asked about: [truncated text]...")
  - Selected text displayed with different styling (italic, gray background)
  - Clicking on displayed selected text scrolls to original location in docs (if visible)
  - Message history preserves selectedText for later review
  - localStorage stores selectedText in ChatMessage

- [ ] T041 [US3] Create unit tests for selected text functionality

  **Files**:
  - `frontend/src/components/ChatKit/__tests__/services/selectedTextService.test.ts`
  - `frontend/src/components/ChatKit/__tests__/components/QuestionModal.test.tsx`

  **Acceptance Criteria**:
  - selectedTextService tests: detectSelectedText() with/without selection
  - QuestionModal tests: render, button clicks, keyboard interaction (Escape)
  - Test displaying selected text excerpt
  - Test form validation (question required)
  - Test "Ask" button submits with selectedText
  - Test "Cancel" button closes without sending
  - Tests pass: `npm run test`
  - Coverage >80% for selectedText-related code

- [ ] T042 [US3] Create E2E test for selected text workflow

  **File**: `frontend/tests/e2e/chatkit-selected-text.spec.ts` (Playwright)

  **Acceptance Criteria**:
  - Test: Select text from documentation → right-click → "Ask ChatKit" → ask question
  - Open Docusaurus chapter page with content
  - Select text passage (e.g., "Forward kinematics is the process...")
  - Right-click on selection
  - Verify context menu shows "Ask ChatKit about this" option
  - Click "Ask ChatKit about this"
  - Verify modal appears with selected text displayed
  - Type question: "Can you elaborate on this?"
  - Click "Ask" button
  - Verify question sent to backend with selectedText parameter (check Network tab)
  - Verify response displays below selected text message in chat
  - Verify selected text excerpt shown in message history
  - Test passes: `npx playwright test chatkit-selected-text.spec.ts`

- [ ] T043 [US3] Create E2E test for mobile selected text on touch devices

  **File**: `frontend/tests/e2e/chatkit-selected-text-mobile.spec.ts` (Playwright)

  **Acceptance Criteria**:
  - Test: Mobile user selects text via touch → long-press → context menu
  - Emulate mobile device (iPhone 12, Android device)
  - Perform text selection via touch (triple-tap or drag select)
  - Long-press (2+ seconds) on selection
  - Verify context menu appears (OS-native may vary, but "Ask ChatKit" should appear)
  - Tap "Ask ChatKit about this" option
  - Modal appears and works as expected (T042)
  - Send question successfully
  - Test passes on both iPhone and Android emulation: `npx playwright test chatkit-selected-text-mobile.spec.ts`

- [ ] T044 [US3] Verify Phase 3 acceptance criteria

  **No coding required** - Review & Manual Testing

  **Acceptance Criteria**:
  - Can complete all 3 P3 acceptance scenarios from spec.md:
    1. ✅ User selects text → right-click → "Ask ChatKit about this" option appears
    2. ✅ User clicks option → question modal opens with selected text auto-filled
    3. ✅ User adds question → backend receives selectedText in request
  - Selected text feature correctly captures highlighted text (95% accuracy, SC-008)
  - Selected text passed to RAG backend for relevance boosting
  - E2E tests pass: `npm run test:e2e:us3`
  - No regression in US1, US2 functionality (all P1, P2 tests still pass)
  - Works on mobile (320px+ width)
  - Works with dark mode
  - Accessibility check passes

---

## Phase 6: Polish, Testing & Quality Assurance (6 tasks)

> **Goal**: Complete testing coverage, accessibility validation, performance optimization, documentation
> **Independent Test**: Full test suite passes, accessibility check passes, Lighthouse score >90, all requirements met

### Quality Assurance & Polish Tasks

- [ ] T045 Run full unit test suite and achieve >80% coverage

  **Command**: `npm run test`

  **Acceptance Criteria**:
  - All unit tests pass (0 failures)
  - All component tests pass
  - All hook tests pass
  - All service tests pass
  - Code coverage >80% for ChatKit component directory
  - No skipped tests (all test cases executed)
  - No console errors or warnings during test run
  - Test output shows: "X tests passed" with no failures
  - Coverage report generated: `npm run test:coverage`

- [ ] T046 Run full E2E test suite across all user stories

  **Command**: `npm run test:e2e`

  **Acceptance Criteria**:
  - E2E test passes: chatkit-basic.spec.ts (US1)
  - E2E test passes: chatkit-history.spec.ts (US1)
  - E2E test passes: chatkit-errors.spec.ts (US1)
  - E2E test passes: chatkit-page-context.spec.ts (US2)
  - E2E test passes: chatkit-dark-mode.spec.ts (US2)
  - E2E test passes: chatkit-selected-text.spec.ts (US3)
  - E2E test passes: chatkit-selected-text-mobile.spec.ts (US3)
  - All E2E tests run against real backend (http://localhost:8000)
  - All tests complete in <5 minutes
  - No flaky tests (consistent pass rate >95%)

- [ ] T047 Verify accessibility compliance (WCAG 2.1 AA)

  **Command**: `npx axe-core http://localhost:3000/docs/chapter-1`

  **Acceptance Criteria**:
  - Run axe accessibility audit on multiple pages
  - 0 Critical violations
  - 0 Serious violations
  - Color contrast ratio maintained at 4.5:1 for text (SC-007, FR-006)
  - All buttons/links keyboard accessible (Tab navigation)
  - Aria labels applied to interactive elements
  - Form inputs have associated labels
  - Focus indicator visible for all interactive elements
  - Screen reader compatible (test with NVDA or JAWS simulation)
  - Mobile tap targets minimum 48px (for US3 context menu)
  - Lighthouse Accessibility score >90
  - Manual review: test with screen reader on actual assistive technology

- [ ] T048 Run code quality checks and ensure production readiness

  **Commands**:
  - `npm run lint` (ESLint)
  - `npm run type-check` (TypeScript)
  - `npm run format` (Prettier)
  - `npm run build` (Production build)

  **Acceptance Criteria**:
  - ESLint: 0 errors, 0 warnings
  - TypeScript: 0 compilation errors with strict mode
  - Code formatting: all files formatted (Prettier passes)
  - Production build succeeds: `npm run build`
  - Build output includes ChatKit component (verify in bundle)
  - No console errors or warnings in production build
  - Bundle size reasonable (<100KB for ChatKit component)
  - No unused imports or dead code
  - All dependencies resolved and available
  - .gitignore properly configured (node_modules, dist, coverage)
  - README.md updated with ChatKit setup instructions
  - All files properly committed to git branch 005-chatkit-integration

---

## Implementation Strategy & Execution

### Execution Order (Dependency Graph)

```
Phase 1: Setup (T001-T004)
    ↓
Phase 2: Foundations (T005-T013) [Parallel: T005-T011, T012-T013]
    ├─ T005: Types (required by all)
    ├─ T006: Storage Service (required by T010)
    ├─ T007: API Service (required by T009)
    ├─ T008: Page Context Service (required by T011)
    ├─ T009: useRAGAPI Hook (requires T007)
    ├─ T010: useChatHistory Hook (requires T006)
    ├─ T011: usePageContext Hook (requires T008)
    ├─ T012: Service Unit Tests (requires T006-T008)
    └─ T013: Hook Unit Tests (requires T009-T011)
    ↓
Phase 3: US1 (T014-T026) [Parallel: T014-T019, T020-T021, T022-T025]
    ├─ T014-T019: ChatKitWidget core
    ├─ T020-T021: Provider + Root integration
    ├─ T022: Component tests
    ├─ T023: E2E basic
    ├─ T024: E2E history
    ├─ T025: E2E errors
    └─ T026: Acceptance verification
    ↓
Phase 4: US2 (T027-T034) [Parallel: T027-T029, T030-T032, T033]
    ├─ T027-T028: Page context enhancement
    ├─ T029: API integration
    ├─ T030-T031: Tests
    ├─ T032: Dark mode E2E
    └─ T033: Component test updates
    ↓
Phase 5: US3 (T035-T044) [Parallel: T035-T037, T038-T040, T041-T043]
    ├─ T035-T037: Selected text UI
    ├─ T038-T040: Integration
    ├─ T041: Unit tests
    └─ T042-T043: E2E tests
    ↓
Phase 6: Quality (T045-T048)
    ├─ T045: Unit test coverage
    ├─ T046: E2E test coverage
    ├─ T047: Accessibility verification
    └─ T048: Code quality checks
```

### Parallel Execution Opportunities

**Setup Phase** (T001-T004): All independent, can run in parallel
- T001: npm install
- T002: tsconfig.json
- T003: Create directories
- T004: Create config file

**Foundations Phase**:
- **Batch 1** (can run immediately after T005):
  - T006, T007, T008: Services (no dependencies, all independent)
  - T012: Service tests (depends on T006-T008)
- **Batch 2** (parallel with Batch 1):
  - T009, T010, T011: Hooks (each depends on one service)
  - T013: Hook tests (depends on T009-T011)

**US1 Phase**:
- **Batch 1**: T014-T019 (ChatKitWidget - serial, one after another)
- **Batch 2** (parallel with Batch 1): T020-T021 (Provider + integration)
- **Batch 3** (after Batch 1, parallel with Batch 2): T022, T023-T025 (tests and E2E)

**US2 Phase** (can overlap with US1 later phases):
- Batch 1: T027-T029 (Feature development)
- Batch 2 (parallel): T030-T032 (Tests)
- Batch 3: T033 (Update existing tests)

**US3 Phase** (can overlap with US1/US2 later phases):
- Batch 1: T035-T037 (Selected text UI)
- Batch 2 (parallel): T038-T040 (Integration)
- Batch 3 (parallel with Batches 1-2): T041-T043 (Tests)

**Quality Phase** (all parallel):
- T045, T046, T047, T048 can all run in parallel

### MVP Scope & Incremental Delivery

**MVP (Minimum Viable Product)**: User Story 1 Only
- **Deliverable**: ChatKit widget with basic Q&A, history, error handling
- **Scope**: T001-T026 (26 tasks, ~30 hours)
- **Time**: 3-4 days for one developer
- **Value**: Users can ask questions and get RAG-powered answers with sources

**MVP+**: Add US2 (Page Context)
- **Scope**: T027-T034 (8 tasks, ~8 hours)
- **Time**: +1-2 days
- **Value**: Better answer relevance based on current page

**Full Feature**: Add US3 (Selected Text)
- **Scope**: T035-T044 (10 tasks, ~10 hours)
- **Time**: +1-2 days
- **Value**: Power users can ask about specific text passages

**Production Ready**: Add Phase 6 (Quality)
- **Scope**: T045-T048 (4 tasks, ~6 hours)
- **Time**: +1 day
- **Value**: Full test coverage, accessibility, performance validated

**Total Estimate**: 48 tasks, 40-50 hours, 2-3 weeks for one developer

---

## Testing Checklist by Phase

### Phase 1 Setup
- [ ] npm install succeeds
- [ ] TypeScript compiles
- [ ] Directories created
- [ ] Config file created

### Phase 2 Foundations
- [ ] All service functions exported
- [ ] All hooks properly typed
- [ ] No circular dependencies
- [ ] Unit tests: >80% coverage
- [ ] Services testable without backend

### Phase 3 US1
- [ ] Widget renders on all pages
- [ ] Can type and send question
- [ ] Backend response displayed with sources
- [ ] History persists on page reload
- [ ] Error messages user-friendly
- [ ] Loads <3s (SC-001)
- [ ] Latency <5s p95 (SC-002)
- [ ] Dark mode works
- [ ] Mobile responsive (320px+)
- [ ] E2E tests pass
- [ ] Manual testing on 3+ browsers

### Phase 4 US2
- [ ] Page context extracted correctly
- [ ] Context passed to backend
- [ ] Backend receives metadata
- [ ] Context confidence levels accurate
- [ ] Answers reflect page context
- [ ] Works on all chapter pages
- [ ] E2E tests pass
- [ ] No regression in US1

### Phase 5 US3
- [ ] Context menu appears on right-click with selection
- [ ] Modal shows selected text
- [ ] Selected text sent to backend
- [ ] Capture accuracy >95% (SC-008)
- [ ] Works on mobile (long-press)
- [ ] E2E tests pass
- [ ] No regression in US1, US2

### Phase 6 Quality
- [ ] Unit coverage >80%
- [ ] All E2E tests pass
- [ ] 0 accessibility violations
- [ ] 0 TypeScript errors
- [ ] 0 ESLint errors
- [ ] Lighthouse >90 (performance, accessibility, best practices)
- [ ] Production build succeeds
- [ ] README updated

---

## Success Criteria Summary

| Criterion | Metric | Status |
|-----------|--------|--------|
| **SC-001** | Widget loads <3 seconds | After T048 |
| **SC-002** | Answer latency <5s p95 | After T026 (verified T048) |
| **SC-003** | Mobile responsive 320px+ | After T018, verified T043 |
| **SC-004** | 100% error handling | After T019, verified T025 |
| **SC-005** | History persistence | After T016, verified T024 |
| **SC-006** | 90% task completion | After T026 manual testing |
| **SC-007** | WCAG AA contrast | After T017, verified T047 |
| **SC-008** | 95% selected text capture | After T042, verified T043 |

All success criteria achieved by completion of Phase 6 (T048).

---

## References & Documentation

- **Specification**: `specs/005-chatkit-integration/spec.md`
- **Architecture Plan**: `specs/005-chatkit-integration/plan.md`
- **Data Model**: `specs/005-chatkit-integration/data-model.md`
- **API Contracts**: `specs/005-chatkit-integration/contracts/*.schema.json`
- **Developer Quickstart**: `specs/005-chatkit-integration/quickstart.md`
- **Backend API**: `backend/README.md` (feature 004-rag-chatbot-api)
- **Constitution**: `.specify/memory/constitution.md`

---

**Ready for implementation.** Execute tasks in order: Phase 1 → 2 → 3 (MVP) → 4 → 5 → 6 (Full Feature + Quality)
