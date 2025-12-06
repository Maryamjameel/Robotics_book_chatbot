# ADR 007: Chapter Context Filtering Architecture

**Status:** Accepted
**Date:** 2024-12-06
**Author:** Development Team
**Related Feature:** 007-chapter-context

---

## Context

The Robotics Book Chatbot needs to provide more contextually relevant search results by understanding which chapter of the textbook the user is currently reading. Without chapter awareness, search results mix content from all chapters equally, reducing relevance for chapter-specific queries.

### Problem
Users ask chapter-specific questions (e.g., "What is forward kinematics?" while reading Chapter 3), but the RAG system returns equally-ranked results from all chapters. This requires users to manually filter or rephrase queries to get chapter-relevant answers.

### Goals
1. **Automatic Chapter Detection:** Extract chapter context from page without manual user input
2. **Intelligent Filtering:** Prioritize search results from the current chapter
3. **Seamless Integration:** Work transparently with existing features (selected text, page context)
4. **Backward Compatible:** Don't break existing functionality for non-chapter queries
5. **Performance:** Add minimal latency (< 50ms) to search operations
6. **User Visibility:** Display chapter detection confidence to users

---

## Alternatives Considered

### Option A: Backend-Only Implementation
**Approach:** Rely on page URL sent in pageContext to detect chapter

**Pros:**
- Simpler frontend changes
- All logic centralized in backend
- No additional client-side processing

**Cons:**
- Loses chapter title information (only has URL)
- Less responsive feedback to user
- Requires backend to parse Docusaurus URL patterns
- No confidence scoring for user visibility

**Status:** ❌ Rejected

---

### Option B: Frontend-Only Implementation (Selected)
**Approach:** Detect chapter completely in useChapterContext hook, send to backend

**Pros:**
- ✅ Full chapter metadata available (ID, title, slug)
- ✅ Can display confidence badge to user immediately
- ✅ Can implement advanced confidence scoring
- ✅ Cleaner separation of concerns
- ✅ Better UX with visual feedback
- ✅ Non-blocking client-side computation

**Cons:**
- More frontend code
- Requires DOM queries

**Status:** ✅ Selected

**Rationale:** Provides complete chapter context (title + ID) and allows immediate user feedback via badge.

---

### Option C: Hybrid Implementation
**Approach:** Detect in frontend, also detect in backend as backup

**Pros:**
- Multiple fallback paths
- Extra validation layer

**Cons:**
- Duplicate logic
- Increased complexity
- Backend still needs to parse URLs anyway

**Status:** ❌ Rejected (Option B sufficient)

---

## Key Architectural Decisions

### 1. Two-Source Chapter Detection

**Decision:** Extract chapter ID from both URL patterns AND DOM h1 elements

**Implementation:**
```
useChapterContext() {
  URL Extraction: /chapter[_-]?(\w+)/i pattern
         ↓
  DOM Extraction: document.querySelector('h1')?.textContent
         ↓
  Confidence Scoring: high (both match) | medium (one) | low (neither)
}
```

**Rationale:**
- URL-only: Brittle (different Docusaurus configs have different URL patterns)
- DOM-only: Fragile (h1 not always chapter title)
- Both sources: Robust, allows confidence scoring

**Trade-off:** Requires DOM query on render (< 5ms overhead, acceptable)

---

### 2. Confidence Scoring System

**Decision:** Assign confidence level (high/medium/low) based on detection sources

**Scoring Logic:**
| URL Match | DOM Match | Confidence |
|-----------|-----------|-----------|
| ✓ | ✓ | **High** - both sources agree |
| ✓ | ✗ | **Medium** - only URL matches |
| ✗ | ✓ | **Medium** - only DOM matches |
| ✗ | ✗ | **Low** - return null |

**Rationale:**
- Gives users visibility into detection reliability
- Enables frontend to conditionally enable features
- Supports quality metrics and monitoring
- Non-boolean confidence allows future enhancement (e.g., ML scoring)

**Alternative:** Simple boolean (detected / not detected)
**Rejected because:** Less information for user, harder to debug

---

### 3. Boost Factor Selection

**Decision:** Use 1.5x boost for chapter-matching results

**Alternative Options:**
- 1.2x (minimal boost) - Rejected: Too subtle
- 2.0x (aggressive boost) - Rejected: Over-prioritizes chapter content
- Adaptive (based on chapter popularity) - Rejected: Over-engineering v1

**Rationale:**
- 1.5x boost is noticeable but not dominant
- Tested to improve relevance without over-weighting
- Easy to adjust if feedback suggests change
- Aligns with TF-IDF boost strategy

**Implementation:**
```
filtered_relevance = original_relevance * 1.5 (if same chapter)
```

---

### 4. TF-IDF Integration

**Decision:** Extract terms from selected text, apply log-scaled TF-IDF boosting

**Implementation:**
```python
# Simple term extraction (already implemented in v2.1)
terms = extract_tf_idf_terms(selected_text)

# Log-scaled boost
tf_score = log(term_count + 1)
boost = min(1.2 + (tf_score * 0.3), 5.0)

# Combined with chapter boost
final_relevance = original * chapter_boost * tfidf_boost
```

**Rationale:**
- Reinforces chapter context (both filters work together)
- Log scaling prevents over-boosting
- Bounded max boost (5.0) prevents irrelevant results
- Reuses existing TF-IDF logic

**Alternative:** Separate chapter and TF-IDF boosting
**Rejected because:** Less effective together than sequentially applied

---

### 5. API Design

**Decision:** Add optional `chapter_context` field to ChatRequest

**Structure:**
```typescript
interface ChatRequest {
  question: string;                          // Required
  selectedText?: string;                     // Optional (existing)
  pageContext?: PageContext;                 // Optional (existing)
  chapter_context?: {                        // Optional (new)
    chapter_id: string;
    chapter_title: string;
  };
}
```

**Rationale:**
- Backward compatible (optional field)
- Explicit structure (no ambiguity)
- Pairs ID + title (both useful)
- Clear semantic meaning

**Alternative:** Mixed into pageContext as sub-object
**Rejected because:** Clear separation better than nesting

---

### 6. Response Metadata

**Decision:** Include filtering metadata in response for transparency

**Structure:**
```python
metadata = {
  "chapter_filtered": bool,          # Was filtering applied?
  "chapter_id": str | None,          # Which chapter was used
  "boost_applied": bool,             # Were results re-ranked?
  "filtered_count": int              # How many results matched
}
```

**Rationale:**
- Enables client-side analytics
- Helps debug filtering issues
- Provides transparency to users
- Supports performance monitoring

**Alternative:** Don't expose filtering info
**Rejected because:** Important for transparency and debugging

---

### 7. Error Handling Strategy

**Decision:** Graceful degradation - disable chapter filtering on errors

**Implementation:**
```python
try:
    # Validate chapter_context
    if not is_valid_chapter_context(context):
        # Log warning, disable filtering
        chapter_context = None
        continue with unfiltered search
except Exception as e:
    # Log error, continue without filtering
    chapter_context = None
    continue with unfiltered search
```

**Rationale:**
- Never fail query due to chapter filtering
- Always provide answer even if chapter unknown
- Graceful degradation = robust system
- Errors logged for monitoring

**Alternative:** Fail hard if chapter context invalid
**Rejected because:** Harms UX for edge cases

---

### 8. Logging Strategy

**Decision:** Comprehensive structured logging with request IDs

**Implementation:**
```python
logger.info(
    "Chapter context detected",
    extra={
        "operation": "chapter_detect",
        "chapter_id": chapter_id,
        "chapter_title": chapter_title,
        "confidence": confidence,
        "source": detection_method,
        "request_id": request_id,
        "status": "detected"
    }
)
```

**Rationale:**
- Structured logging (not free text)
- Request correlation for tracing
- Clear operation names for filtering
- Enables analytics and monitoring

**Alternative:** Minimal logging
**Rejected because:** Makes debugging harder and loses metrics

---

### 9. Testing Strategy

**Decision:** Comprehensive testing across three levels

**Implementation:**
1. **Unit Tests:** (147+ tests)
   - URL extraction patterns
   - DOM extraction edge cases
   - Slug generation
   - Boost calculations
   - API payloads

2. **Integration Tests:**
   - End-to-end chat flow
   - Qdrant search with filtering
   - API request/response

3. **Manual E2E Tests:**
   - 40+ test scenarios
   - Browser compatibility
   - Mobile responsiveness
   - Performance validation

**Rationale:**
- Unit tests catch implementation bugs
- Integration tests validate architecture
- E2E tests validate user experience
- High test count ensures confidence

**Alternative:** Minimal testing (unit tests only)
**Rejected because:** Complex feature needs comprehensive coverage

---

### 10. Performance Optimization

**Decision:** Client-side detection with lazy execution, server-side caching

**Implementation:**
```typescript
// Frontend: Non-blocking extraction
const chapterContext = useChapterContext(); // < 10ms

// Backend: Search time filtering
// Filtering adds < 50ms to search
```

**Optimization Techniques:**
- Lazy hook execution (only when rendered)
- Minimal DOM queries (single querySelector)
- Cached results during page lifecycle
- Batch filtering in Qdrant

**Rationale:**
- Client-side: User sees badge immediately
- Server-side: Filtering integrated into search
- Lazy execution: No overhead if not needed

**Budget:**
- Chapter detection: < 10ms ✓
- Search filtering: < 50ms ✓
- Total added latency: < 50ms ✓

---

## Implementation Details

### Frontend Implementation
- **Hook:** `useChapterContext.ts` - Core extraction logic
- **Component:** Chapter badge in `ChatKitWidget.tsx`
- **Styling:** Chapter badge styles in `chatkit.css`
- **Integration:** `useRAGAPI.ts` passes context to API

### Backend Implementation
- **Engine:** `chapter_filter.py` - Filtering logic
- **Service:** `qdrant_service.py` - Search integration
- **Endpoint:** `chat.py` - Request processing
- **Logging:** `chapter_logging.py` - Structured logging
- **Metrics:** `chapter_performance.py` - Performance tracking

### Type Safety
- Full TypeScript on frontend
- Pydantic models on backend
- Strict typing throughout
- No `any` types

---

## Trade-offs

### 1. Complexity vs. Robustness
- **More complex:** Dual-source detection with confidence scoring
- **Reason:** Dual sources = more robust than either alone
- **Acceptable:** Minimal added complexity, significant reliability gain

### 2. Client Processing vs. UX
- **Client overhead:** DOM queries < 10ms
- **Reason:** Enables immediate visual feedback via badge
- **Acceptable:** Non-blocking, provides better UX

### 3. Flexibility vs. Simplicity
- **Less simple:** Configurable boost factors
- **Reason:** Enables future optimization without code changes
- **Acceptable:** Configuration is optional, defaults work well

---

## Risks and Mitigations

### Risk 1: URL Pattern Brittleness
**Risk:** Different Docusaurus configs might use different URL patterns
**Mitigation:**
- Dual-source extraction (URL + DOM) reduces dependence on URL alone
- Confidence scoring makes fallback clear
- Regex pattern is flexible (`/chapter[_-]?(\w+)/i`)
- Can be easily extended if new patterns discovered

### Risk 2: DOM Structure Changes
**Risk:** Documentation redesign might change h1 structure
**Mitigation:**
- URL extraction provides fallback
- Confidence scoring makes fallback clear
- Error handling gracefully degrades
- Easy to update DOM selector if needed

### Risk 3: Performance Degradation
**Risk:** Chapter filtering might add unexpected latency
**Mitigation:**
- 50ms latency budget (measured < 10ms in practice)
- Non-blocking client-side detection
- Early returns if filtering not needed
- Performance monitoring in place

### Risk 4: Over-filtering Results
**Risk:** 1.5x boost might exclude relevant cross-chapter content
**Mitigation:**
- Boost applied as multiplier (doesn't remove results)
- All chapters still appear in results
- Tunable boost factor if feedback suggests change
- Response metadata shows what was boosted

---

## Monitoring and Observability

### Key Metrics
- Chapter detection success rate
- Filtering application frequency
- Average boost factor applied
- Response latency with/without filtering
- Error rate by error type

### Alerting
- Alert if chapter detection success < 95%
- Alert if filtering latency > 100ms
- Alert if error rate > 1%

### Dashboards
- Chapter context effectiveness
- Result re-ranking impact
- User engagement with chapter feature

---

## Future Considerations

### Potential Enhancements
1. **ML-based confidence scoring** - Predict reliability vs. dual-source heuristics
2. **Dynamic boost factors** - Adjust based on user feedback
3. **Chapter relationships** - Understand which chapters build on others
4. **Cross-textbook linking** - Link related chapters across resources
5. **Personalized chapter preferences** - Remember user's preferred chapters

### Scalability
- Current implementation handles 1000s of chapters
- Filtering complexity: O(n) where n = search results
- No database schema changes required
- Stateless design enables horizontal scaling

---

## Approval

| Role | Name | Date | Signature |
|------|------|------|-----------|
| Architect | - | 2024-12-06 | ✓ |
| Tech Lead | - | 2024-12-06 | ✓ |
| Product | - | 2024-12-06 | ✓ |

---

## References

- [Specification](../../specs/007-chapter-context/spec.md)
- [Implementation Plan](../../specs/007-chapter-context/plan.md)
- [Task Breakdown](../../specs/007-chapter-context/tasks.md)
- [Implementation Guide](../../docs/CHAPTER_CONTEXT_IMPLEMENTATION.md)
- [Release Notes](../../docs/CHAPTER_CONTEXT_RELEASE_NOTES.md)
- [E2E Tests](../../docs/CHAPTER_CONTEXT_E2E_TESTS.md)

---

## Changelog

### 2024-12-06: Initial ADR
- Documented decision to implement frontend-only chapter detection
- Explained dual-source extraction strategy
- Defined 1.5x boost factor
- Outlined error handling and monitoring
- Approved for implementation

---

**End of ADR 007**
