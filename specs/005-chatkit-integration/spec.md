# Feature Specification: ChatKit Docusaurus Integration

**Feature Branch**: `005-chatkit-integration`
**Created**: 2025-12-06
**Status**: Draft
**Input**: Integrate ChatKit chatbot SDK into Docusaurus with RAG backend integration

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Access RAG Chatbot from Documentation (Priority: P1)

Users reading the Robotics textbook chapters need immediate access to ask questions about the content without leaving the documentation. They can open the ChatKit widget and ask questions related to forward kinematics, robotics concepts, or any chapter material, receiving context-aware answers powered by the RAG pipeline.

**Why this priority**: Core value proposition—enables users to learn interactively from documentation without context switching. Essential for MVP viability.

**Independent Test**: User opens Docusaurus page → clicks chat icon → asks "What is forward kinematics?" → receives RAG-generated answer with sources. Fully demonstrates feature value.

**Acceptance Scenarios**:

1. **Given** user is reading a documentation chapter, **When** user clicks the ChatKit widget button, **Then** chat window opens in bottom-right corner without page interruption
2. **Given** chat window is open, **When** user types a question and presses send, **Then** question is sent to backend RAG endpoint (`/api/v1/chat/ask`)
3. **Given** backend returns RAG response, **When** response contains sources, **Then** sources are displayed as clickable links below the answer
4. **Given** user closes chat window, **When** user navigates to another page, **Then** chat history is preserved and visible upon reopening
5. **Given** backend is unavailable, **When** user sends a question, **Then** user sees friendly error message: "Service unavailable. Please try again later."

---

### User Story 2 - Use Page Context for Better Answers (Priority: P2)

When users ask questions, the system should automatically capture the current page context (chapter name, section, URL) and pass it to the RAG pipeline for more relevant answers tailored to their current location in the textbook.

**Why this priority**: Improves answer relevance and user experience. Enables contextual follow-up questions. High-value enhancement for learning.

**Independent Test**: User is on "Chapter 1 - Introduction to ROS 2" page → asks a question → backend receives page context in request metadata → answers reflect chapter context (mentions "Chapter 1" or ROS 2 specifics).

**Acceptance Scenarios**:

1. **Given** user is on a specific chapter page, **When** user sends a question, **Then** request includes metadata: `{chapter: "Chapter 1 - Introduction", url: "http://...", section: "..."}` (if detectable)
2. **Given** page context is available, **When** RAG pipeline processes query, **Then** context is used to improve search relevance in vector database
3. **Given** user asks follow-up question on same page, **When** context is captured again, **Then** follow-up maintains same page context for consistency

---

### User Story 3 - Select Text and Ask Questions (Priority: P3)

Power users should be able to select text from the documentation and ask questions specifically about that selection, enabling them to clarify confusing passages or request elaboration on specific concepts.

**Why this priority**: Enhances usability for advanced learners. Reduces friction for follow-up questions. Medium priority—valuable feature but users can achieve similar results manually.

**Independent Test**: User selects text passage from documentation → right-click or highlight → "Ask about this" option appears → question modal opens with selected text → user adds their question → backend receives selected text as context. Demonstrates independent value.

**Acceptance Scenarios**:

1. **Given** user selects text in documentation, **When** user right-clicks selection, **Then** context menu includes "Ask ChatKit about this" option
2. **Given** user clicks "Ask about this", **When** question modal opens, **Then** selected text is automatically included in request as `selectedText` field
3. **Given** selected text is provided, **When** backend processes question, **Then** RAG pipeline uses selected text to narrow search scope for more focused answers

---

### Edge Cases

- What happens when user opens chat widget before page fully loads? → Widget should remain visible and responsive; page context captured once available
- How does system handle very long questions (>2000 characters)? → Display input validation warning; truncate or reject gracefully per backend constraints
- What if backend takes >10 seconds to respond? → Show loading indicator; implement timeout with user-friendly message
- What happens when user's network connection is lost mid-request? → Display error and allow retry; preserve question text for user
- How does dark mode work? → Widget colors and theme automatically adapt to match Docusaurus dark/light mode toggle

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: ChatKit widget MUST be mountable in Docusaurus Root.tsx component using React Provider pattern
- **FR-002**: Widget MUST display in bottom-right corner of viewport and remain visible on scroll
- **FR-003**: Widget MUST communicate with FastAPI backend at `http://localhost:8000/api/v1/chat/ask` endpoint
- **FR-004**: Each chat message MUST be sent as JSON request: `{question: string, selectedText?: string, pageContext?: object}`
- **FR-005**: Widget MUST display backend response with message text and sources array formatted as clickable links
- **FR-006**: Widget MUST support dark mode through CSS variables that adapt to Docusaurus theme
- **FR-007**: Chat history MUST persist across page navigations within same session (localStorage or React context)
- **FR-008**: Widget MUST validate user input and reject empty questions with inline error message
- **FR-009**: Widget MUST display loading indicator while awaiting backend response
- **FR-010**: Widget MUST implement 30-second timeout for backend requests with user-facing timeout error
- **FR-011**: Widget configuration MUST accept API endpoint URL as parameter for flexibility across environments (dev/staging/prod)
- **FR-012**: System MUST support selected-text feature with right-click context menu integration
- **FR-013**: Widget MUST be fully responsive on mobile devices (stack vertically or slide in from side)
- **FR-014**: Widget MUST allow user to clear chat history with button
- **FR-015**: Error responses from backend MUST be caught and displayed with appropriate user messages (no technical error details exposed)

### Key Entities

- **ChatMessage**: Represents user question or bot response
  - `id`: Unique identifier (UUID)
  - `role`: "user" or "assistant"
  - `content`: Message text
  - `sources`: Array of source links (for assistant messages)
  - `timestamp`: When message was created
  - `pageContext`: Metadata about page where question was asked (chapter, URL, section)

- **ChatSession**: Container for chat conversation
  - `sessionId`: Identifier for current conversation
  - `messages`: Array of ChatMessage objects
  - `startedAt`: When session was created
  - `lastMessageAt`: When last message was sent

- **RAGRequest**: Payload sent to backend
  - `question`: User's question text (required, 1-2000 characters)
  - `selectedText`: Highlighted text from documentation (optional)
  - `pageContext`: Chapter/URL/section metadata (optional)
  - `sessionId`: For tracking related questions (optional)

- **RAGResponse**: Response from backend endpoint
  - `answer`: Generated answer text
  - `sources`: Array of {title, url, snippet} objects
  - `confidence`: Confidence score (0-1)
  - `metadata`: {searchLatencyMs, generationLatencyMs, totalLatencyMs}

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chat widget loads within 3 seconds of page load without blocking Docusaurus rendering
- **SC-002**: Users can send a question and receive an answer in under 5 seconds (p95 latency)
- **SC-003**: Chat widget works on mobile devices with viewport width ≥320px with proper responsive behavior
- **SC-004**: 100% of backend error responses are caught and displayed as user-friendly messages (no console errors exposed)
- **SC-005**: Chat history persists across page navigations and can be cleared by user action
- **SC-006**: At least 90% of test user sessions successfully complete: open widget → ask question → receive answer
- **SC-007**: Dark mode colors maintain WCAG AA contrast ratio (4.5:1 for text)
- **SC-008**: Selected-text feature correctly captures highlighted text and includes it in request to backend in at least 95% of attempts

## Assumptions

- **Backend API**: RAG API is available at `http://localhost:8000/api/v1/chat/ask` in development; configurable for other environments
- **ChatKit SDK**: @anthropic/chatkit or equivalent SDK provides React Provider and UI components; TypeScript types included
- **Docusaurus Setup**: Frontend project uses React 18+, TypeScript, and standard Docusaurus Root.tsx layout pattern
- **LocalStorage Support**: Browsers support localStorage for chat history persistence (graceful fallback if not available)
- **CORS Configuration**: Backend enables CORS for requests from Docusaurus frontend domain
- **Mobile Interaction**: Touch events work equivalently to mouse events for mobile users
- **Error Handling**: Backend returns JSON errors with consistent structure (follows OpenAPI spec from backend)
- **No Authentication**: MVP does not require user authentication; all requests are anonymous

## Dependencies & Integration Points

### External Dependencies

- **ChatKit SDK** (`@anthropic/chatkit` or similar): React Provider, chat components, message handling
- **Backend RAG API** (`004-rag-chatbot-api`): `/api/v1/chat/ask` endpoint for question answering
- **Docusaurus** (existing): Theme system, layout, navigation context
- **React Context/Hooks** (existing): State management for chat history

### Integration Points

1. **Frontend → Backend**: HTTP POST to `/api/v1/chat/ask` with RAG request payload
2. **Page Context Detection**: Extract chapter name from Docusaurus sidebar/breadcrumb data
3. **Theme Integration**: Listen to Docusaurus theme toggle for dark/light mode sync
4. **History Persistence**: Use browser localStorage with JSON serialization

## Out of Scope

- User authentication or login system
- Persisting chat history to database (session-only localStorage)
- Multi-language UI (English only for MVP)
- Voice input/output
- Streaming responses (one request → one full response)
- Advanced prompt engineering or system messages
- Analytics or user feedback collection
- Integration with other chat services
- Desktop-specific features (Electron, etc.)

## Constraints

- **Input Size**: Questions limited to 1-2000 characters per backend validation
- **Response Time**: Backend response timeout set to 30 seconds maximum
- **Rate Limiting**: Backend enforces 5 requests per second; frontend should not exceed this
- **Storage**: Chat history limited to 10KB localStorage (approximately 50-100 messages)
- **Deployment**: Widget must work on localhost for development before staging/production
- **Browser Support**: Modern browsers only (Chrome, Firefox, Safari, Edge 2020+)

## Success Metrics (Observable in Production)

- Widget opens in response to 95%+ of user clicks
- Average message latency (send to receive) is under 5 seconds
- Error rate for backend requests is <2%
- Users complete question → answer flow with 90%+ success rate
- Dark mode toggle correctly updates widget colors in 100% of cases
