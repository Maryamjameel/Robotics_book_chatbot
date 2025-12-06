# Data Model: Chapter Context Awareness

**Feature**: `007-chapter-context` | **Date**: 2025-12-06 | **Phase**: 1 Design

## Entity Definitions

### ChapterContext

**Purpose**: Represents extracted chapter metadata from current page (frontend entity)

**Fields**:
- `chapterId: string` - Unique chapter identifier (e.g., "ch03", "chapter-3")
  - Source: Extracted from URL pathname
  - Format: lowercase, no spaces
  - Example: "chapter-3-kinematics" → "chapter-3"

- `chapterTitle: string` - Human-readable chapter name (e.g., "Kinematics")
  - Source: Extracted from page h1 tag
  - Format: Title Case
  - Example: "Chapter 3: Kinematics" → "Kinematics"

- `chapterSlug: string` - URL-safe slug derived from title
  - Source: Generated from chapterTitle via titleToSlug()
  - Format: lowercase, hyphens
  - Example: "Inverse Kinematics" → "inverse-kinematics"
  - Usage: For consistent chapter references across frontend

- `confidence: 'high' | 'medium' | 'low'` - Extraction confidence level
  - `high`: Both URL and h1 tag confirm chapter context
  - `medium`: Only URL matches, or only h1 matches
  - `low`: Uncertain extraction
  - Usage: Inform UI warnings; consider hiding badge if `low`

**Validation Rules**:
- `chapterId` must match pattern: `/^[a-z0-9\-]+$/` (alphanumeric and hyphens)
- `chapterTitle` must be non-empty and max 100 characters
- `chapterSlug` must be non-empty
- `confidence` must be one of the three defined values

**State Transitions**:
- When URL changes → re-extract chapter context
- When extraction succeeds → ChapterContext object returned
- When extraction fails → null returned (no chapter page)
- When chapter changes → UI badge updates (no API call needed immediately)

**Relationships**:
- Used by `ChatKitWidget` to display chapter badge
- Sent to backend via `ChatRequest.chapter_context` field
- Logged for debugging in RAG pipeline traces

---

### ChatRequest (Extended)

**Purpose**: Backend request model for RAG queries, extended with chapter context

**New Field**:
```python
chapter_context: Optional[dict] = None
```

**Structure** (when provided):
```python
{
  "chapter_id": "ch03",        # string: chapter identifier
  "chapter_title": "Kinematics"  # string: chapter name
}
```

**Validation**:
- If provided, both `chapter_id` and `chapter_title` must be present
- `chapter_id` must match format `/^[a-z0-9\-]+$/`
- `chapter_title` must be max 100 characters
- If invalid format, reject with 422 Unprocessable Entity

**Schema Update**:
```python
class ChatRequest(BaseModel):
    question: str
    selectedText: Optional[str] = None
    pageContext: Optional[PageContext] = None
    chapter_context: Optional[ChapterContextPayload] = None  # NEW
    sessionId: Optional[str] = None

class ChapterContextPayload(BaseModel):
    chapter_id: str = Field(..., max_length=100, pattern="^[a-z0-9\\-]+$")
    chapter_title: str = Field(..., max_length=100)
```

**Usage in Pipeline**:
1. Backend extracts `chapter_context` from request
2. Passes `chapter_id` to Qdrant search
3. Stores `chapter_id` in response metadata for logging
4. Enables filtering/prioritization of results

---

### ChatResponse (Extended)

**Purpose**: Backend response model, extended to include chapter filtering metadata

**New Fields in Metadata**:
```python
class ResponseMetadata(BaseModel):
    # ... existing fields ...
    chapter_filtered: Optional[bool] = False       # NEW
    chapter_id: Optional[str] = None               # NEW
```

**Semantics**:
- `chapter_filtered: true` → Chapter context was provided and search was filtered
- `chapter_filtered: false` → No chapter context provided; global search performed
- `chapter_id: "ch03"` → Which chapter was used for filtering (for logging/debugging)

**Example Response**:
```json
{
  "answer": "Forward kinematics is the process...",
  "sources": [...],
  "confidence": 0.92,
  "metadata": {
    "confidence_score": 0.92,
    "search_latency_ms": 150,
    "generation_latency_ms": 1200,
    "total_latency_ms": 1450,
    "chapter_filtered": true,
    "chapter_id": "ch03"
  }
}
```

---

### QdrantSearchResult (Re-Ranked)

**Purpose**: Represents a Qdrant search result, with chapter-based re-ranking applied

**Fields** (existing from Phase 2.1):
```python
{
  "payload": {
    "chapter_id": "ch03",
    "chapter_title": "Kinematics",
    "section_number": 1,
    "section_title": "Forward Kinematics",
    "text": "Forward kinematics involves...",
    "source": "textbook"
  },
  "score": 0.87,  # cosine similarity (0-1)
  "id": "uuid"
}
```

**Re-Ranking Logic** (when chapter_context provided):
1. Separate results into two groups:
   - **Chapter-matching**: `payload.chapter_id == request.chapter_id`
   - **Non-matching**: `payload.chapter_id != request.chapter_id`
2. Return order: `[...chapter-matching, ...non-matching]`
3. Preserve original score for tie-breaking within each group
4. Maximum 5 results returned (unchanged)

**Example Re-Ranking**:
```
Before re-ranking (by similarity score):
  1. "Section 1.2" from Chapter 2 (score: 0.88)
  2. "Section 3.1" from Chapter 3 (score: 0.86)  ← Matching chapter
  3. "Section 5.1" from Chapter 5 (score: 0.85)

After re-ranking (with chapter_id="ch03"):
  1. "Section 3.1" from Chapter 3 (score: 0.86)  ← Moved to top
  2. "Section 1.2" from Chapter 2 (score: 0.88)
  3. "Section 5.1" from Chapter 5 (score: 0.85)
```

**Impact on Response**:
- First-ranked result used for answer generation
- All results provided to LLM context
- Citation validation uses top result first
- Confidence scoring considers chapter relevance

---

## Data Relationships

```
User (on Docusaurus page)
  ↓
useChapterContext() Hook
  ↓ extracts
ChapterContext { chapterId, chapterTitle, chapterSlug, confidence }
  ↓ serialized to
ChatRequest { chapter_context: { chapter_id, chapter_title } }
  ↓ sent via POST /api/v1/chat/ask
Backend receives ChatRequest
  ↓ extracts chapter_id
Qdrant search with optional chapter filtering
  ↓ retrieves and re-ranks
QdrantSearchResult[] (chapter-matching first)
  ↓ passed to RAG pipeline
Gemini generates answer using chapter-relevant context
  ↓ includes metadata
ChatResponse { metadata: { chapter_filtered, chapter_id } }
  ↓ returned to frontend
ChatKitWidget displays answer with context
```

---

## Validation & Constraints

### Frontend Validation (useChapterContext Hook)

- ✅ Chapter ID must match pattern: `/^[a-z0-9\-]+$/`
- ✅ Chapter title must be non-empty, max 100 chars
- ✅ Chapter slug must be derivable from title
- ✅ Confidence must be one of: high, medium, low

### Backend Validation (ChatRequest Pydantic Model)

- ✅ chapter_context is optional; if provided, both fields required
- ✅ chapter_id pattern: `/^[a-z0-9\-]+$/`
- ✅ chapter_title max 100 characters
- ✅ Invalid requests rejected with 422 status

### Qdrant Validation

- ✅ chapter_id in payload exists (assumed from Phase 2.1 setup)
- ✅ Re-ranking handles missing chapter_id in some results (e.g., legacy chunks)
- ✅ Search succeeds even if no chapter-matching results

---

## Summary

Chapter Context is a lightweight operational enhancement with minimal data modeling:
- **Frontend**: Simple extraction of URL + DOM data into ChapterContext object
- **Backend**: Optional field in ChatRequest, used for search filtering
- **Storage**: No new persistence; uses existing Qdrant metadata
- **Validation**: Pattern matching and type checking via Pydantic
- **Relationships**: Linear pipeline from extraction → request → search → response

No new database tables or schema changes required. Pure algorithmic enhancement.
