# Quickstart: Chapter Context Awareness Implementation

**Feature**: `007-chapter-context` | **Phase**: 2 Implementation (Ready to Start)

## Overview

This quickstart provides developers with a rapid reference for implementing chapter context awareness. See `plan.md` for full architectural details.

---

## Key Concepts

1. **Chapter Extraction** (Frontend)
   - Extract chapter ID from URL: `/docs/chapter-3-kinematics` → `"chapter-3"`
   - Extract chapter title from h1 tag: `"# Kinematics"` → `"Kinematics"`
   - Generate slug: `"Kinematics"` → `"kinematics"`
   - Confidence scoring: high/medium/low based on extraction success

2. **API Contract Update** (Backend)
   - Extend `ChatRequest` with optional `chapter_context` field
   - Include in all requests: `{ "chapter_id": "ch03", "chapter_title": "Kinematics" }`
   - Backend validates and processes chapter context

3. **Search Filtering** (Backend)
   - Receive `chapter_id` from request
   - Search Qdrant (existing logic unchanged)
   - Re-rank results: chapter-matching results first, others after
   - Return top 5 (no change to result count)

4. **Response Metadata** (Backend)
   - Add to response: `chapter_filtered: true`, `chapter_id: "ch03"`
   - Enables frontend logging and debugging

---

## Implementation Checklist

### Frontend

- [ ] Create `useChapterContext.ts` hook
  - Extract chapter ID via regex from `location.pathname`
  - Extract title via DOM query `document.querySelector('h1')`
  - Generate slug via `titleToSlug()` helper
  - Return `{ chapterId, chapterTitle, chapterSlug, confidence }` or `null`

- [ ] Update `ChatKitWidget.tsx`
  - Call `useChapterContext()` in component
  - Display chapter badge: "Ch. 3 - Kinematics" (if chapter context available)
  - Hide badge on non-chapter pages

- [ ] Update `chatkit.types.ts`
  - Add `ChapterContext` interface
  - Update `RAGRequest` to include `chapter_context: ChapterContext | null`

- [ ] Update `apiService.ts`
  - Modify `sendQuestion()` to accept `chapterContext` parameter
  - Include in request: `{ chapter_context: { chapter_id, chapter_title } }`

### Backend

- [ ] Update `ChatRequest` schema (`chat.py`)
  - Add optional field: `chapter_context: Optional[dict] = None`
  - Pydantic validation for chapter_id format

- [ ] Create `chapter_filter.py` service
  - Implement `ChapterFilterEngine` class
  - Method: `filter_by_chapter(results, chapter_id)` → re-ranked results
  - Logic: separate chapter-matching and non-matching, return [chapter, others]

- [ ] Update `qdrant_service.py`
  - Modify `search()` method to accept optional `chapter_id` parameter
  - After Qdrant search, apply chapter filtering if chapter_id provided
  - Return re-ranked results

- [ ] Update `chat.py` endpoint
  - Extract `chapter_id` from request: `request.chapter_context.get("chapter_id")`
  - Pass to Qdrant search: `search(embedding, chapter_id=chapter_id)`
  - Update response metadata: `metadata.chapter_filtered = bool(chapter_id)`

### Testing

- [ ] Frontend Unit Tests
  - Test `useChapterContext()` on chapter page
  - Test `useChapterContext()` on non-chapter page
  - Test chapter extraction from various URL patterns
  - Test h1 extraction from DOM

- [ ] Backend Unit Tests
  - Test `ChapterFilterEngine.filter_by_chapter()` with sample results
  - Test ChatRequest validation with valid/invalid chapter_context

- [ ] Integration Tests
  - End-to-end: Submit question from chapter page, verify chapter_context sent
  - Verify Qdrant returns chapter results first
  - Verify response metadata includes `chapter_filtered: true`

---

## File Modifications Summary

### New Files
```
frontend/src/components/ChatKit/hooks/useChapterContext.ts
backend/src/services/utils/chapter_filter.py
backend/tests/unit/test_chapter_filter.py
backend/tests/integration/test_chapter_filtering.py
frontend/src/components/ChatKit/hooks/__tests__/useChapterContext.test.ts
```

### Modified Files
```
backend/src/models/chat.py
  - Add chapter_context field to ChatRequest

backend/src/api/v1/routes/chat.py
  - Extract and pass chapter_context to Qdrant search
  - Update response metadata

backend/src/services/qdrant_service.py
  - Add chapter_id parameter to search() method
  - Implement chapter-based re-ranking

frontend/src/components/ChatKit/ChatKitWidget.tsx
  - Display chapter badge

frontend/src/components/ChatKit/services/apiService.ts
  - Include chapter_context in requests

frontend/src/components/ChatKit/types/chatkit.types.ts
  - Add ChapterContext interface
  - Update RAGRequest type
```

---

## API Contracts

### Request
```typescript
POST /api/v1/chat/ask

{
  "question": "What is forward kinematics?",
  "chapter_context": {
    "chapter_id": "ch03",
    "chapter_title": "Kinematics"
  }
}
```

### Response
```typescript
{
  "answer": "Forward kinematics...",
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

See `contracts/chapter-context-api.yaml` for full OpenAPI spec.

---

## Testing Strategy

### Unit Tests
- **Frontend**: `useChapterContext()` extraction logic
- **Backend**: `ChapterFilterEngine` re-ranking logic

### Integration Tests
- End-to-end RAG pipeline with chapter context
- Verify Qdrant re-ranking produces correct result order

### E2E Tests (Playwright)
- Navigate to chapter page
- Open ChatKit, verify chapter badge displays
- Ask question, verify chapter context is used
- Verify chapter-specific results rank higher

---

## Performance Expectations

- Chapter extraction: <50ms (DOM query + regex parsing)
- Search re-ranking: <50ms (array manipulation on 5 results)
- Total RAG latency increase: <100ms (acceptable per constitution)
- No impact on success rate (fallback to global search if chapter unavailable)

---

## Success Criteria (from spec.md)

- ✅ Chapter context extracted on 100% of chapter pages
- ✅ Chapter badge displays in header
- ✅ API request includes chapter_context
- ✅ Chapter results rank in top 3 (when relevant)
- ✅ Latency increase <50ms
- ✅ Graceful handling of missing chapter context
- ✅ Chapter context preserved through entire pipeline

---

## Common Patterns

### URL Extraction Pattern
```typescript
const match = pathname.match(/chapter[_-]?(\w+)/i);
const chapterId = match ? match[1].toLowerCase() : null;
```

### DOM Extraction Pattern
```typescript
const h1 = document.querySelector('h1');
const chapterTitle = h1?.textContent?.trim() ?? null;
```

### Backend Re-Ranking Pattern
```python
chapter_results = [r for r in results if r['chapter_id'] == chapter_id]
other_results = [r for r in results if r['chapter_id'] != chapter_id]
return chapter_results + other_results
```

---

## Debugging Tips

1. **Chapter context not detected**: Check browser console for extraction logs
2. **Chapter results not ranking first**: Verify chapter_id matches in Qdrant payload
3. **API receiving chapter_context**: Add logging in chat.py to inspect request
4. **Latency spike**: Profile chapter filtering logic; should be negligible

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown
2. Implement Frontend tasks (hooks, types, UI updates)
3. Implement Backend tasks (schema, filtering, endpoint updates)
4. Write tests as you go (TDD approach)
5. Integration test end-to-end RAG pipeline
6. Create PR with all changes
7. Review against constitution for compliance

See `tasks.md` for detailed implementation sequence.
