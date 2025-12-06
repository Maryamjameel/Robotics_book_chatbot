# Feature Specification: Chapter Context Awareness

**Feature Branch**: `007-chapter-context`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Implement chapter context awareness with ChatKit"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Extract and Display Current Chapter (Priority: P1)

When a student is reading Chapter 3 on kinematics, they should immediately know they're in that chapter context when they open the chat widget. The system automatically detects which chapter they're viewing from the page URL and page content, and displays it prominently in the chat interface.

**Why this priority**: This is the foundational feature enabling chapter-aware search. Without knowing which chapter the user is in, we cannot provide contextually relevant answers. This is a prerequisite for all downstream chapter-based filtering.

**Independent Test**: Can be fully tested by navigating to any chapter page (e.g., `/docs/chapter-3-kinematics`) and verifying the chapter badge appears in the ChatKit header with the correct chapter name. Delivers immediate visual feedback that the system understands context.

**Acceptance Scenarios**:

1. **Given** student is on chapter page with chapter title in URL, **When** ChatKit loads, **Then** system displays chapter name/number in header badge
2. **Given** student is on section page within a chapter, **When** ChatKit loads, **Then** system extracts chapter context from breadcrumb or page structure
3. **Given** student is on non-chapter page (e.g., homepage), **When** ChatKit loads, **Then** system gracefully handles missing chapter context (no badge or "General" label)
4. **Given** student navigates between chapters, **When** chapter changes, **Then** chapter badge updates in real-time without page refresh

---

### User Story 2 - Filter Search Results by Current Chapter (Priority: P1)

When a student asks a question while reading Chapter 3, the search system should prioritize results from Chapter 3, making answers more relevant to what they're currently studying. If relevant content exists in the current chapter, show that first; content from other chapters appears lower in results.

**Why this priority**: This directly improves answer relevance and learning experience. Students get answers tailored to their current location, reducing cognitive load and improving comprehension.

**Independent Test**: Can be tested by asking a question on a chapter page and comparing results with the same question asked from another chapter context. Results should be reordered with current chapter prioritized. Delivers measurably more relevant answers.

**Acceptance Scenarios**:

1. **Given** student on Chapter 3 asks "what is forward kinematics", **When** backend receives request with chapter_id, **Then** results from Chapter 3 appear first, others sorted by relevance
2. **Given** Chapter 3 contains no relevant content, **When** student searches, **Then** system falls back to cross-chapter results
3. **Given** student on homepage (no chapter context) asks question, **When** backend receives request without chapter_id, **Then** system performs global search without chapter filtering

---

### User Story 3 - Send Chapter Context to Backend (Priority: P1)

The frontend needs to extract chapter information from the current page and include it when sending questions to the backend, enabling the backend to filter and prioritize Qdrant search results by chapter.

**Why this priority**: Without sending chapter context, the backend cannot filter results. This is the API contract that enables Story 2.

**Independent Test**: Can be tested by making API requests and verifying the JSON payload includes chapter context. Verify backend receives and processes chapter_id parameter.

**Acceptance Scenarios**:

1. **Given** user asks question on chapter page, **When** API request is sent, **Then** request body includes `chapter_context` with chapter_id
2. **Given** user on non-chapter page, **When** API request sent, **Then** request body either omits chapter_context or includes null/empty values
3. **Given** useChapterContext hook is called, **When** it extracts chapter from URL, **Then** it returns object with chapterId, chapterTitle, chapterSlug

---

### User Story 4 - Prioritize Chapter Results in Qdrant (Priority: P2)

The backend's Qdrant search should implement chapter-based result prioritization: if the current chapter has matching results, re-rank them to appear first in the response, without filtering out cross-chapter results.

**Why this priority**: Improves relevance without losing information. Students still see the full picture but with current context emphasized. Medium priority because Story 2 (basic filtering) provides value first.

**Independent Test**: Can be tested by searching on a chapter with chapter-specific content and verifying top results are from the current chapter. Verify ranking formula applies correctly.

**Acceptance Scenarios**:

1. **Given** Qdrant search for "kinematics" on Chapter 3 page, **When** chapter_id is provided, **Then** search results with chapter_id==3 rank higher than others
2. **Given** prioritization is applied, **When** calculating result scores, **Then** ranking uses boost factor (e.g., 1.2x multiplier for chapter matches)
3. **Given** no chapter context, **When** search is performed, **Then** results are returned in standard relevance order without boost

---

### Edge Cases

- What happens when a student accesses a chapter with special characters in the URL (e.g., `chapter-3-advanced-motion-planning`)?
- What happens when the page h1 title doesn't match chapter title expectations (malformed or custom headings)?
- How does the system handle dynamic page loads (e.g., hash-based routing where URL doesn't change)?
- What happens when a student opens ChatKit on a page with no chapter context (e.g., homepage, FAQ)?
- What if the chapter ID in the URL doesn't correspond to any actual chapter in the system?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST extract chapter identifier from URL pathname (e.g., extract "ch03" or "chapter-3" from `/docs/chapter-3-kinematics`)
- **FR-002**: System MUST extract chapter title from the page's primary heading (h1 tag) if available
- **FR-003**: System MUST create a slug/ID from chapter title for consistent referencing (e.g., "advanced-kinematics" → "advanced-kinematics")
- **FR-004**: Frontend MUST provide `useChapterContext` hook that returns `{ chapterId, chapterTitle, chapterSlug }` or null if not on chapter page
- **FR-005**: Frontend MUST display chapter context in ChatKit header as a badge or label (e.g., "Ch. 3 - Kinematics")
- **FR-006**: Frontend MUST include chapter_context in all API requests sent to the RAG backend (via RAGRequest payload)
- **FR-007**: Backend ChatRequest schema MUST accept optional `chapter_context` field with chapter_id and chapter_title
- **FR-008**: Backend MUST filter/prioritize Qdrant search results by chapter_id if provided in the request
- **FR-009**: Backend MUST update response metadata to indicate whether chapter filtering was applied and what boost factor was used
- **FR-010**: System MUST handle gracefully when chapter context is unavailable (no errors, fallback to global search)

### Key Entities

- **ChapterContext**: Object containing `chapterId` (string), `chapterTitle` (string), `chapterSlug` (string), extracted from page
- **ChatRequest**: Extended with optional `chapter_context` field (chapter_id: string, chapter_title: string)
- **SearchResult**: Qdrant result, re-ranked by chapter priority if chapter_context provided

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chapter context is correctly extracted on 100% of chapter pages (verified by automated tests)
- **SC-002**: Chapter badge displays in ChatKit header on 100% of chapter pages
- **SC-003**: When user asks question on chapter page, API request includes chapter_context in payload
- **SC-004**: Qdrant search results from current chapter rank in top 3 results (when chapter has relevant content)
- **SC-005**: Search result latency does not increase by more than 50ms when chapter filtering is applied
- **SC-006**: System handles missing chapter context without errors (fallback to global search with no user-facing errors)
- **SC-007**: Chapter context information is preserved through the entire RAG pipeline (request → search → response)

## Assumptions

- Chapter URLs follow predictable patterns (e.g., `/docs/chapter-X-title` or `/chapter/X`)
- Chapter identifiers are unique and consistent across the system
- Page h1 contains chapter title (or can be reliably extracted from URL)
- Qdrant database has chapter metadata indexed (chapter_id in payload)
- Backend has access to chapter definitions (for validation and prioritization)

## Out of Scope

- Multi-chapter searches or cross-chapter learning paths
- Chapter-level permissions or access control
- Chapter outline navigation or chapter selector dropdown
- Dark mode theming (covered separately in Phase 2.5)
- End-to-end test implementation (covered in Phase 2.5 testing milestone)
