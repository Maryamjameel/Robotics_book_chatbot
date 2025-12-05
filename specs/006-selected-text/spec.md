# Feature Specification: Selected Text Context for ChatKit

**Feature Branch**: `006-selected-text`
**Created**: 2025-12-06
**Status**: Draft
**Input**: Implement selected-text feature with ChatKit - text selection detection, tooltip UI, backend search boosting

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask About Selected Text via Tooltip (Priority: P1)

A user reading the documentation selects some text (e.g., "forward kinematics") to quickly ask a question about it. A tooltip appears near the selection with an "Ask about this" button. Clicking the button opens the ChatKit widget with the selected text pre-filled as context. The backend uses this context to find more relevant answers.

**Why this priority**: This is the core user value of the feature - quick, contextual questioning without manual typing. Users want to ask about specific terminology they encounter while reading. This is an MVP feature that delivers standalone value.

**Independent Test**: Can be fully tested by selecting text on a Docusaurus page, clicking the tooltip button, seeing the ChatKit widget open with the selected text pre-filled, and receiving answers that reference the selected text context.

**Acceptance Scenarios**:

1. **Given** a user is reading documentation, **When** they select text (mouse or touch), **Then** a tooltip appears at the selection coordinates within 200ms
2. **Given** the tooltip is visible, **When** the user clicks "💬 Ask about this" button, **Then** the ChatKit widget opens and the selected text is pre-filled in the input field
3. **Given** the ChatKit widget is open with selected text, **When** the user submits the question, **Then** the backend receives the selected_text parameter and uses it to boost search results
4. **Given** the tooltip is visible, **When** the user clicks the dismiss button or clicks elsewhere, **Then** the tooltip disappears
5. **Given** selection changes (new text selected), **When** the previous tooltip is still visible, **Then** the tooltip updates to the new selection coordinates within 200ms

---

### User Story 2 - Search Boosting by Selected Text (Priority: P2)

When a question includes selected text context, the RAG backend boosts Qdrant vector search results that contain the selected text. Passages matching the selected text are ranked higher, making answers more contextually relevant to the exact terminology the user was reading about.

**Why this priority**: Improves answer quality and relevance when users ask about specific terms. Requires backend changes to Qdrant search logic. Could be implemented independently, but requires coordinated frontend→backend integration.

**Independent Test**: Can be tested by sending a question with selected_text parameter to the backend, verifying that results containing the selected text appear higher in the ranking, and comparing ranking with/without the parameter.

**Acceptance Scenarios**:

1. **Given** a RAG request with selected_text="forward kinematics", **When** the backend searches Qdrant, **Then** chunks containing "forward kinematics" are ranked higher in results
2. **Given** boosting is applied, **When** multiple results match the selected text, **Then** relevance (cosine similarity) is weighted by presence of selected text terms
3. **Given** selected_text is provided but no chunks match it, **When** the backend searches, **Then** results fall back to standard ranking without error
4. **Given** selected_text is very long (>100 chars), **When** the backend searches, **Then** only the most relevant terms are used for boosting (fuzzy matching)

---

### User Story 3 - Mobile-Optimized Selection Experience (Priority: P3)

On mobile/touch devices, the text selection tooltip appears with appropriate positioning and sizing. Touch selections (long-press) are supported alongside mouse selections. Tooltip is mobile-optimized with larger touch targets and doesn't interfere with normal scrolling.

**Why this priority**: Extends the feature to mobile users (common documentation readers on tablets/phones). Could be deferred but improves user experience on all device types. Requires responsive positioning logic for tooltip.

**Independent Test**: Can be tested on mobile devices/emulators by selecting text with touch, verifying tooltip appears with adequate spacing, and ensuring "Ask about this" button is easily tappable (48px minimum).

**Acceptance Scenarios**:

1. **Given** a user is on a mobile device, **When** they long-press to select text, **Then** a tooltip appears with mobile-appropriate sizing and positioning
2. **Given** the tooltip is near the bottom of viewport, **When** it would be cut off, **Then** it repositions above the selection to remain visible
3. **Given** the tooltip is visible, **When** the user scrolls the page, **Then** the tooltip remains visible at the new scroll position
4. **Given** user is on a narrow viewport (<640px), **When** they select text, **Then** the tooltip width adapts to fit without blocking content

---

### Edge Cases

- What happens when user selects zero text or only whitespace? → Tooltip should not appear, or appear disabled
- How does system handle very long selections (>500 chars)? → Tooltip shows truncated preview, full text still sent to backend
- What if selected text contains special characters or non-ASCII? → Tooltip displays correctly, backend handles encoding properly
- How does tooltip behave when multiple instances of selected text exist on page? → Tooltip positions near the actual selection coordinates
- What if user has text-selection disabled in browser? → Tooltip doesn't appear, no error shown
- What if ChatKit widget is already open? → Clicking "Ask about this" updates the pre-filled text in the existing widget instead of opening new one
- What if selected text is identical to previous selection? → Tooltip repositions to new location if selection coordinates changed

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST detect text selection via `window.getSelection()` on mouseup and touchend events
- **FR-002**: System MUST calculate tooltip position (x, y) relative to selection bounding rect
- **FR-003**: System MUST display a tooltip UI with "💬 Ask about this" button and dismiss button within 200ms of selection
- **FR-004**: System MUST not display tooltip if no text is selected or selection is whitespace-only
- **FR-005**: System MUST update tooltip position and content when user selects different text without dismissing/re-opening
- **FR-006**: System MUST dismiss tooltip when user clicks dismiss button, clicks elsewhere on page, or scrolls past selection
- **FR-007**: System MUST handle text selection on touch devices (long-press) with same behavior as mouse selection
- **FR-008**: ChatKit widget MUST accept pre-filled selected text and include it in the input field when "Ask about this" is clicked
- **FR-009**: ChatKit widget MUST send selected_text in RAGRequest payload to backend (optional field, backward compatible)
- **FR-010**: Frontend MUST NOT expose any errors to user if selection detection fails; silently disable feature
- **FR-011**: Backend MUST accept optional selected_text parameter in chat request (POST /api/v1/chat/ask)
- **FR-012**: Backend MUST pass selected_text to Qdrant search query for relevance boosting
- **FR-013**: Backend MUST boost Qdrant results that contain selected_text terms with weighted relevance increase
- **FR-014**: Backend MUST gracefully handle missing selected_text (standard search if not provided)
- **FR-015**: Backend MUST handle selected_text up to 500 characters in length
- **FR-016**: Tooltip MUST be keyboard accessible (dismiss via Escape key, focusable buttons)
- **FR-017**: Tooltip MUST not interfere with normal page scrolling on mobile devices
- **FR-018**: System MUST support partial matching for selected_text boosting (fuzzy match for typos/variations)

### Key Entities

- **TextSelection**: Represents the currently selected text on the page
  - `text`: The selected text string (up to 500 chars)
  - `x, y`: Coordinates for tooltip positioning (relative to viewport)
  - `timestamp`: When selection was made (for debouncing updates)

- **SelectionTooltip**: UI component that appears near text selection
  - `text`: Preview of selected text (truncated if >50 chars)
  - `isVisible`: Whether tooltip is currently shown
  - `position`: {x, y} coordinates
  - `isDismissible`: Can user close the tooltip

- **RAGRequest** (extended): Existing ChatMessage request type with optional field
  - `selected_text?`: Optional selected text context for search boosting (new field, backward compatible)
  - All other fields remain unchanged

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Tooltip appears within 200ms of text selection on desktop (mouseup event latency)
- **SC-002**: Tooltip appears within 500ms of text selection on mobile (touchend event latency, acceptable for touch UX)
- **SC-003**: Users can ask about selected text with zero additional typing (pre-filled input)
- **SC-004**: Answers that include selected text context appear with ≥10% higher relevance score vs. same question without context
- **SC-005**: Tooltip is correctly positioned at selection coordinates with 95%+ accuracy (no clipping, visible on screen)
- **SC-006**: Feature works on mobile devices (320px+ width) with 48px minimum tap targets
- **SC-007**: WCAG AA accessibility compliance for tooltip UI (keyboard navigation, focus indicators, color contrast)
- **SC-008**: Tooltip remains functional for 99%+ of text selections (no failures, graceful degradation for edge cases)
- **SC-009**: Backend supports up to 500 character selected_text without performance degradation (search latency <5s p95)

---

## Dependencies & Constraints

### External Dependencies

- **Docusaurus 3.x**: Root.tsx integration point for SelectionTooltip component
- **window.getSelection() API**: Browser text selection detection (supported in all modern browsers)
- **FastAPI Backend**: POST /api/v1/chat/ask endpoint (existing, needs selected_text parameter support)
- **Qdrant Vector DB**: Search query API (existing, needs relevance boosting logic)

### Constraints

- **No breaking changes**: New selected_text field is optional; existing requests must work unchanged
- **Storage**: Selection context not persisted; only used for current request (not stored in chat history)
- **Performance**: Tooltip calculations must not block main thread; use requestAnimationFrame for positioning
- **Mobile**: Must work on iOS Safari, Android Chrome (key documentation platforms)
- **Accessibility**: Must maintain WCAG AA contrast and keyboard navigation

---

## Assumptions

- Users have modern browsers with `window.getSelection()` API support (IE11 and below not supported)
- Text selection is always done by user (not programmatically hidden text)
- Selected text is relevant context for improving RAG search results (not all selected text will improve results)
- Backend has sufficient Qdrant query capacity to support relevance boosting without impacting latency SLA
- Tooltip positioning uses viewport coordinates (not scroll-relative positioning)
- Selected text boosting uses term frequency/presence, not semantic similarity (simple keyword boost)

---

## Non-Goals

- Copying selected text to clipboard (out of scope)
- Rich text/formatting preservation (selected text is plain string)
- Selection highlighting/underlining on page (feature is tooltip-based, not inline markup)
- Voice/dictation selection support (keyboard/mouse/touch only)
- Server-side storage of user selection history (stateless feature)
- A/B testing infrastructure for boosting effectiveness (monitoring/metrics only)

