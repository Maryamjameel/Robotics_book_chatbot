# Chapter Context Awareness Implementation Guide

## Overview

Chapter Context Awareness enables the Robotics Book Chatbot to intelligently detect which chapter the user is reading and use that information to provide more relevant search results. This guide documents the complete implementation.

---

## Architecture

### High-Level Flow

```
User navigates to chapter page
    ↓
useChapterContext hook (Frontend)
    ├─ Extract chapter ID from URL
    ├─ Extract chapter title from DOM
    └─ Calculate confidence score
    ↓
Chapter badge displays in ChatKit widget
    ↓
User asks question
    ↓
apiService.sendQuestion() includes chapter_context
    ↓
Backend /api/v1/chat/ask endpoint
    ├─ Validates chapter context
    ├─ Calls search_chunks with chapter_context
    └─ Logs filtering metrics
    ↓
Qdrant search with chapter filtering
    ├─ Retrieve top-k results
    ├─ Filter/boost chapter matches (1.5x)
    ├─ Apply TF-IDF boosting if selected text
    └─ Return re-ranked results
    ↓
RAG endpoint processes results
    ├─ Generate LLM answer
    ├─ Populate response metadata
    └─ Include chapter filtering stats
    ↓
Frontend displays answer with chapter-aware sources
```

---

## Component Details

### 1. Frontend: useChapterContext Hook

**Location:** `frontend/src/components/ChatKit/hooks/useChapterContext.ts`

**Purpose:** Extract chapter metadata from current Docusaurus page

**Key Functions:**

| Function | Purpose |
|----------|---------|
| `extractChapterIdFromUrl()` | Extract chapter ID from URL pathname using regex `/chapter[_-]?(\w+)/i` |
| `extractTitleFromDOM()` | Query h1 element for chapter title |
| `titleToSlug()` | Convert title to URL-safe slug (e.g., "Inverse Kinematics" → "inverse-kinematics") |
| `determineConfidence()` | Calculate confidence level based on extraction sources |
| `useChapterContext()` | Main hook that orchestrates extraction and returns ChapterContext object |

**Return Type:**
```typescript
ChapterContext | null {
  chapterId: string;          // e.g., "ch03", "chapter-3"
  chapterTitle: string;       // e.g., "Kinematics"
  chapterSlug: string;        // e.g., "kinematics"
  confidence: 'high' | 'medium' | 'low'
}
```

**Confidence Scoring:**
- **High:** Both URL and DOM match
- **Medium:** Only one source (URL or DOM) matches
- **Low:** Neither source produces clear match (returns null)

### 2. Frontend: Chapter Badge Display

**Location:** `frontend/src/components/ChatKit/ChatKitWidget.tsx`

**Function:** `renderChapterBadge()`

**Features:**
- Color-coded confidence indicators (green/amber/red)
- Icon indicators (✓/◐/?)
- Responsive text wrapping
- Accessible markup

**Styling:** `frontend/src/components/ChatKit/styles/chatkit.css`

### 3. Frontend: API Request

**Location:** `frontend/src/components/ChatKit/hooks/useRAGAPI.ts`

**Changes:**
- Accepts `chapterContext?: ChapterContext` parameter
- Maps ChapterContext to RAGRequest payload:
  ```typescript
  chapter_context: {
    chapter_id: chapterContext.chapterId,
    chapter_title: chapterContext.chapterTitle
  }
  ```

### 4. Backend: Chat Request Schema

**Location:** `backend/src/models/chat.py`

**New Field in ChatRequest:**
```python
chapter_context: Optional[dict] = Field(
    default=None,
    description="Optional chapter context for filtering search results",
    example={"chapter_id": "ch03", "chapter_title": "Kinematics"}
)
```

### 5. Backend: Chapter Filtering Engine

**Location:** `backend/src/services/utils/chapter_filter.py`

**Classes:**

| Class | Purpose |
|-------|---------|
| `ChapterContextFilter` | Data model for chapter filter context |
| `SearchResult` | Input result from Qdrant |
| `FilteredResult` | Output result with filtering metadata |
| `ChapterFilterEngine` | Main filtering and re-ranking engine |

**Methods:**

```python
# Filter results by chapter
filtered_results = engine.filter_results(results, chapter_filter)

# Apply TF-IDF boosting
boosted_results = engine.apply_tf_idf_boost(filtered_results, selected_text_terms)

# Get metadata about filtering applied
metadata = engine.get_boost_metadata()
```

**Boost Factors:**
- Chapter match boost: 1.5x relevance
- TF-IDF base boost: 1.2x
- TF-IDF max boost: 5.0x

### 6. Backend: Qdrant Integration

**Location:** `backend/src/services/qdrant_service.py`

**Modified Function:** `search_chunks()`

**New Parameters:**
- `chapter_context`: Optional chapter context dict
- `selected_text_terms`: Optional list of terms for TF-IDF boosting

**Return Value:**
```python
{
  "results": [...],           # List of search results
  "metadata": {               # Filtering/boosting metadata
    "chapter_filtered": bool,
    "chapter_id": str,
    "boost_applied": bool,
    "filtered_count": int
  }
}
```

### 7. Backend: Chat Endpoint

**Location:** `backend/src/api/v1/routes/chat.py`

**Changes:**
1. **Validation**: Validate chapter_context if provided
2. **Term Extraction**: Extract key terms from selected_text for TF-IDF
3. **Search Integration**: Pass chapter_context to search_chunks()
4. **Metadata Population**: Add chapter filtering stats to response metadata
5. **Logging**: Log all chapter filtering operations

**Request Flow:**
```
ChatRequest received
    ↓
Validate chapter_context (chapter_id required, max 100 chars)
    ↓
Extract TF-IDF terms from selected_text
    ↓
Call search_chunks(question_embedding, chapter_context, selected_text_terms)
    ↓
Get filtered and re-ranked results from search_chunks
    ↓
Call rag_service.answer_question()
    ↓
Populate response.metadata with chapter filtering stats
    ↓
Return ChatResponse
```

---

## Data Flow Examples

### Example 1: User on Chapter 3 page asks about kinematics

**URL:** `/docs/chapter-3-kinematics`

**Chapter Detection:**
- URL pattern matches: chapter_id = "chapter-3"
- h1 text = "Kinematics"
- confidence = "high" (both sources match)

**API Payload:**
```json
{
  "question": "What is forward kinematics?",
  "chapter_context": {
    "chapter_id": "chapter-3",
    "chapter_title": "Kinematics"
  }
}
```

**Qdrant Results BEFORE filtering:**
1. [Ch4] Dynamics section - score: 0.92
2. [Ch3] Forward Kinematics - score: 0.95
3. [Ch5] Control system kinematics - score: 0.88

**Qdrant Results AFTER filtering (with 1.5x boost for Ch3):**
1. [Ch3] Forward Kinematics - score: 0.95 * 1.5 = 1.425
2. [Ch4] Dynamics section - score: 0.92
3. [Ch5] Control system kinematics - score: 0.88

**Response Metadata:**
```json
{
  "chapter_filtered": true,
  "chapter_id": "chapter-3",
  "boost_applied": true,
  "filtered_count": 1
}
```

---

### Example 2: User selects text and asks question with chapter context

**Selected Text:** "Forward kinematics calculates position"

**TF-IDF Terms Extracted:** ["forward", "kinematics", "calculates", "position"]

**TF-IDF Boost Calculation:**
- Result 1: 2 matches (forward, kinematics) → boost = 1.2 + (0.2 * 2) = 1.6x
- Result 2: 0 matches → boost = 1.0x
- Result 3: 1 match (kinematics) → boost = 1.2 + (0.2 * 1) = 1.4x

**Response Metadata:**
```json
{
  "chapter_filtered": true,
  "chapter_id": "chapter-3",
  "selected_text_boosted": true,
  "selected_text_terms": ["forward", "kinematics", "calculates", "position"],
  "boost_factor": 1.6
}
```

---

## Testing Strategy

### Unit Tests
- **URL Extraction:** 20+ tests covering various URL patterns
- **DOM Extraction:** 15+ tests for h1 tag edge cases
- **Slug Generation:** 10+ tests for special characters
- **Chapter Filtering:** 30+ tests for filtering logic
- **TF-IDF Boosting:** 15+ tests for boost calculations

### Integration Tests
- **Chat Endpoint:** End-to-end flow with chapter context
- **Qdrant Integration:** Search with filtering
- **Payload Validation:** API request/response structure

### Edge Case Tests
- Malformed URLs
- Missing DOM elements
- Special characters in titles
- Very long chapter IDs
- Concurrent requests
- Error handling

### Manual E2E Tests
See `CHAPTER_CONTEXT_E2E_TESTS.md` for comprehensive test scenarios.

---

## Performance Considerations

### Latency Budget
- Chapter detection: < 10ms (client-side)
- API overhead: < 50ms (chapter filtering adds minimal cost)
- Total response time: < 2000ms (unchanged from baseline)

### Optimization Techniques
1. **Lazy Extraction:** Only extract when hook is rendered
2. **Memoization:** Cache chapter context during page lifecycle
3. **Batch Filtering:** Apply chapter filtering to all results at once
4. **Early Return:** Skip filtering if no chapter context provided

### Performance Metrics
Located in `backend/src/services/utils/chapter_performance.py`:
- Track chapter detection latency
- Monitor filtering throughput
- Collect TF-IDF boost performance
- Generate performance reports

---

## Backward Compatibility

### API Compatibility
- All chapter_context fields are optional
- Existing code without chapter context continues to work
- No changes to existing API contracts
- Graceful degradation when chapter context unavailable

### Database Compatibility
- No database schema changes required
- Vector embeddings unchanged
- No migration needed

---

## Error Handling

### Invalid Chapter Context
- Missing chapter_id → Disable filtering (log warning)
- chapter_id too long (> 100 chars) → Disable filtering
- Malformed chapter_context → Disable filtering
- Always fail gracefully with normal search

### Search Failures
- If filtering fails → Use unfiltered results
- If TF-IDF boost fails → Use original results
- If Qdrant unavailable → Return error (unchanged)

---

## Logging

Comprehensive logging for debugging and monitoring:
- Chapter detection (source: URL/DOM/both)
- Filtering applied (chapter_id, boost_factor)
- TF-IDF boosting (terms, boost amount)
- Errors and validation failures
- Performance metrics

See `backend/src/services/utils/chapter_logging.py` for logger utility.

---

## Configuration

### Environment Variables (if needed)
```bash
# Chapter filtering boost factor (default: 1.5)
CHAPTER_BOOST_FACTOR=1.5

# TF-IDF max boost (default: 5.0)
TFIDF_MAX_BOOST=5.0

# Enable chapter filtering (default: true)
ENABLE_CHAPTER_FILTERING=true
```

### Feature Flags
- Can disable chapter filtering via config
- Can adjust boost factors per deployment
- Can disable TF-IDF boosting independently

---

## Monitoring and Observability

### Key Metrics
- Chapter detection success rate
- Filtering application rate
- Boost factor distribution
- Performance percentiles (p50, p95, p99)
- Error rate by error type

### Dashboards
- Chapter context detection trend
- Search filtering effectiveness
- Response latency with/without filtering
- Error rates and patterns

---

## Future Enhancements

### Phase 2 Features
- [ ] Learning from user feedback which chapters are most relevant
- [ ] Dynamic boost factor adjustment based on user satisfaction
- [ ] Multi-chapter filtering (user specifies interest in multiple chapters)
- [ ] Chapter relationship detection (kinematics prerequisite for dynamics)

### Performance Optimizations
- [ ] Precompute chapter relevance scores
- [ ] Cache chapter metadata during session
- [ ] Parallel chapter detection (URL + DOM simultaneously)

---

## Troubleshooting

### Chapter Badge Not Displaying
1. Check if useChapterContext hook is returning null
2. Verify URL matches pattern or h1 tag exists
3. Check browser console for errors
4. Verify ChatKitWidget is rendering renderChapterBadge()

### Filtering Not Applied
1. Check if chapter_context in API request
2. Verify chapter_id format (should be lowercase)
3. Check backend logs for validation errors
4. Verify search_chunks receiving chapter_context parameter

### Results Not Reranked
1. Verify filtered results include chapter-matching items
2. Check boost factor calculation
3. Ensure TF-IDF terms correctly extracted
4. Check response metadata shows filtering applied

---

## Support and Questions

For questions or issues:
1. Check E2E test scenarios in `CHAPTER_CONTEXT_E2E_TESTS.md`
2. Review unit test examples in test files
3. Check server logs and monitoring dashboards
4. File issue with reproduction steps
