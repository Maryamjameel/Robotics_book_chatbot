# ADR-004: Data Validation & Citation Grounding Strategy

**Status**: Accepted
**Date**: 2025-12-05
**Feature**: 004-rag-chatbot-api
**Decision ID**: RAG-CITATIONS-004

---

## Context

The RAG chatbot must prevent hallucinated sources (a constitutional requirement: "RAG Accuracy & Source Citation - HIGH PRIORITY"). The system must ensure:

- **Citation Accuracy**: 100% of citations exist in retrieved chunks (no fabricated sources)
- **Citation Grounding**: All citations are verifiable from retrieved content
- **Data Consistency**: ChatRequest, ChatResponse, Source models are always valid
- **Type Safety**: All data flows through Pydantic validation
- **Confidence Tracking**: Uncertain answers are clearly marked

The architecture must balance:
- **Strict Validation**: Prevent invalid data from corrupting system
- **Usability**: Don't reject reasonable user input
- **Performance**: Validation overhead is negligible (<10ms)
- **Observable**: Validation failures are logged for quality monitoring

---

## Decision

**Chosen Approach**: Pydantic v2 Models + Post-Generation Citation Validation

**Specific Components**:

1. **Request Validation** (Pydantic):
   - ChatRequest model: question (1-2000 chars), optional filters (dict)
   - Automatic validation on FastAPI route (400 Bad Request if invalid)

2. **Response Validation** (Pydantic):
   - ChatResponse model: answer (str), sources (List[Source]), metadata (dict)
   - Source model: chapter_id (str), section_number (int), relevance_score (0.0-1.0)
   - Automatic serialization with type checking

3. **Citation Grounding** (Post-Generation):
   - Extract source references from LLM-generated answer
   - Match against retrieved chunks (by chapter_id + section_number)
   - Flag answer as "uncertain" if citation doesn't match (confidence=0.5)
   - Log mismatches for hallucination detection

4. **Confidence Scoring** (Confidence Model):
   - Calculate based on average chunk relevance (0.7→0.5, 0.85→0.7, >0.85→0.9)
   - Adjust for citation validity (grounded=+0.1, uncertain=-0.2)
   - Return confidence in metadata for user awareness

---

## Rationale

### Why Pydantic v2 Models Over Manual Validation

**Pydantic v2 Models** (Chosen):
- **Automatic Validation**: Schemas validated on every route/serialization
- **Type Safety**: Python type hints enforced at runtime
- **OpenAPI Integration**: Models auto-generate OpenAPI documentation
- **Error Messages**: Validation errors are clear and actionable
- **Performance**: Zero-cost abstractions (C extensions for JSON parsing)
- **Ecosystem**: Works seamlessly with FastAPI
- **Future-Proof**: Pydantic is industry standard for Python APIs

**Manual Validation** (Rejected):
- ❌ Boilerplate: Check every field in every function
- ❌ Error-Prone: Easy to miss edge cases
- ❌ No OpenAPI: Must manually document API schema
- ❌ Poor Error Messages: Custom validation is verbose
- ✅ Lightweight: One less dependency (not worth the tradeoff)

### Why Post-Generation Citation Validation

**Chosen**: After Gemini generates answer, validate citations against chunks

```python
# After LLM generation
generated_answer = await gemini_service.generate_answer(context, question)

# Extract citation references from answer
cited_chapters = extract_citations(generated_answer)

# Validate against retrieved chunks
grounded = validate_citations(cited_chapters, retrieved_chunks)
if not grounded:
    confidence = 0.5  # Mark as uncertain
    answer = answer + "\n(Note: This answer is uncertain - sources may not match)"
```

**Rationale**:
- Catch hallucinations after generation (only place we can verify)
- LLM doesn't guarantee grounded citations (explicit validation required)
- Constitution requires 100% citation accuracy (validation is mandatory)
- Performance: Validation is <10ms (negligible overhead)

**Alternative** (Rejected): Trust LLM with explicit prompt
- ❌ Insufficient: Gemini still hallucinates 5-10% of the time
- ❌ Violates Constitution: "100% of citations from retrieved sections"
- ✅ Simpler: No validation code needed
- **Decision**: Constitution compliance outweighs simplicity

### Why Confidence Scoring Model

**Chosen**: Multi-factor confidence based on relevance + citation validity

```python
base_confidence = 0.5 if avg_relevance < 0.7 else 0.7 if avg_relevance < 0.85 else 0.9
if citations_grounded:
    confidence = base_confidence + 0.1
else:
    confidence = base_confidence - 0.2
```

**Rationale**:
- Users need to know answer reliability
- Confidence helps prioritize answers for follow-up
- Data-driven (based on relevance scores + citation validity)
- Constitutional: "Include confidence scoring in responses"

**Alternative** (Rejected): Binary confidence (yes/no)
- ❌ Less informative: No gradation
- ❌ Harder to act on: User doesn't know degree of certainty
- ✅ Simpler: Fewer levels to track

### Why 1-2000 Character Limit on Questions

**Chosen**: Maximum 2000 characters per question

**Rationale**:
- Prevent token overflow (Gemini context budget ~4k tokens)
- Keep latency predictable (<2s generation)
- Educational use: 2000 chars is enough for detailed question
- API clarity: Clear limit prevents surprises

**Boundary Testing**:
- Empty question: 400 Bad Request
- 1 character: Valid
- 2000 characters: Valid
- 2001 characters: 400 Bad Request

---

## Consequences

### Positive Outcomes

✅ **Type Safety**: All data validated through Pydantic (prevents runtime errors)
✅ **Citation Accuracy**: Post-generation validation catches hallucinations
✅ **Confidence Transparency**: Users see how confident system is
✅ **Clear Error Messages**: Validation errors help users fix requests
✅ **OpenAPI Auto-Generated**: Documentation stays in sync with code
✅ **Constitutional Compliance**: 100% citation accuracy achievable
✅ **Observable**: Mismatched citations logged for hallucination analysis

### Tradeoffs & Constraints

⚠️ **Stricter Input Validation**: Some edge-case questions might be rejected (rare)
⚠️ **Lower Reported Confidence**: Post-validation can lower confidence scores (more conservative)
⚠️ **Citation Extraction Complexity**: Parsing LLM output requires careful regex
⚠️ **No Automatic Recovery**: Can't fix hallucinated citations (only flag as uncertain)
⚠️ **Validation Latency**: Citation validation adds ~5-10ms (acceptable)

### Implementation Implications

- Must create Pydantic models: ChatRequest, Source, ChatResponse, RAGMetadata
- Must implement citation extraction (regex pattern matching)
- Must implement citation matching (compare to retrieved chunks)
- Must populate confidence scores in all responses
- Must log validation failures for monitoring
- Must handle edge cases (citations at start/middle/end of answer)

---

## Alternatives Considered

### Alternative 1: Few-Shot Prompting for Citation Accuracy

**Approach**: Use few-shot examples in Gemini prompt to teach citation format

**Pros**:
- Prevents hallucinations upfront (before generation)
- Reduces invalid citations in LLM output
- LLM naturally follows examples

**Cons**:
- Still ~5-10% hallucination rate (not 100%)
- Consumes context window (~200 tokens for examples)
- Harder to debug (must re-prompt entire context)
- Doesn't provide certainty assurance to users

**Decision**: Use + post-validation (belt and suspenders)

### Alternative 2: Re-Generation with Stricter Prompt if Hallucination Detected

**Approach**: If citations don't match, ask Gemini to regenerate with stricter requirements

**Pros**:
- Could fix some hallucinations automatically
- Higher success rate
- Better user experience

**Cons**:
- Doubles latency (regenerate takes another 1.5s → violates SLA)
- No guarantee second attempt succeeds
- Increases Gemini API cost
- Complex error handling

**Decision**: Rejected for MVP; flag as uncertain instead (faster, simpler)

### Alternative 3: Chunk-Level Confidence Thresholds

**Approach**: Only use chunks with relevance >0.8 (stricter filtering)

**Pros**:
- Higher quality context for LLM
- Fewer hallucinations naturally
- Clearer what's relevant

**Cons**:
- May miss relevant content (false negatives)
- "No relevant content" answers increase
- Less educational (fewer sources to learn from)

**Decision**: Use lower threshold (0.7) but validate citations after generation

### Alternative 4: No Citation Validation (Trust LLM)

**Approach**: Accept whatever citations Gemini generates, don't validate

**Pros**:
- Simplest: No validation code
- Faster: Skip validation step

**Cons**:
- ❌ Constitutional Violation: "100% citations from retrieved sections"
- ❌ Quality Risk: 5-10% hallucination rate unacceptable
- ❌ Trust Issue: Users can't verify sources

**Decision**: Rejected; constitution requires validation

---

## Implementation Strategy

### Phase 1: Pydantic Models
```python
# models/chat.py
class ChatRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000)
    filters: Optional[dict] = None

class Source(BaseModel):
    chapter_id: str
    section_number: int
    section_title: str
    excerpt: str
    relevance_score: float = Field(..., ge=0, le=1)

class ChatResponse(BaseModel):
    answer: str
    sources: List[Source]
    metadata: dict
```

### Phase 2: Citation Extraction
```python
def extract_citations(answer: str) -> List[tuple]:
    """Extract chapter_id, section from answer
    Returns: [(chapter_id, section_number), ...]
    """
    pattern = r'Chapter\s+(\w+),\s+Section\s+(\d+)'
    return re.findall(pattern, answer)
```

### Phase 3: Citation Validation
```python
def validate_citations(answer: str, retrieved_chunks: List[dict]) -> bool:
    """Check if citations in answer match retrieved chunks"""
    cited = extract_citations(answer)
    chunks = {(ch['chapter_id'], ch['section_number']) for ch in retrieved_chunks}
    return all(cite in chunks for cite in cited)
```

### Phase 4: Confidence Calculation
```python
def calculate_confidence(avg_relevance: float, citations_valid: bool) -> float:
    base = 0.5 if avg_relevance < 0.7 else 0.7 if avg_relevance < 0.85 else 0.9
    return (base + 0.1) if citations_valid else (base - 0.2)
```

---

## Monitoring & Success Metrics

**Citation Accuracy**: ≥99% of citations match retrieved chunks (constitution requirement)
**Hallucination Rate**: <1% of answers contain ungrounded citations (validated)
**Validation Overhead**: <10ms per request (negligible)
**Confidence Distribution**: >70% of answers have confidence ≥0.7
**User Clarity**: 100% of responses include confidence_score in metadata

---

## Related Decisions

- **ADR-001**: LLM Provider (Gemini citation capabilities)
- **ADR-003**: Error Handling (how to handle uncertain answers)

---

## References

- **Plan**: `specs/004-rag-chatbot-api/plan.md` (Phase 1: Data Models)
- **Specification**: `specs/004-rag-chatbot-api/spec.md` (FR-003: ChatResponse, FR-008: Citations, SC-004: Citation Precision)
- **Constitution**: `.specify/memory/constitution.md` (III. RAG Accuracy & Source Citation)
- **Implementation**: Task T011-T015 (models), T032 (citation validation), T077 (confidence) in tasks.md

---

## Decision Log

| Date | Status | Author | Notes |
|------|--------|--------|-------|
| 2025-12-05 | Accepted | Architecture Team | Decision finalized during planning phase |

---

## Future Considerations

### If Hallucination Rate Exceeds 1%
- Implement re-generation with stricter prompts
- Use few-shot examples more aggressively
- Increase chunk relevance threshold

### If Confidence Scoring Inaccurate
- Adjust confidence formula based on real-world data
- Add user feedback (users rate answer accuracy)
- Implement calibration: train confidence model on feedback

---

**ADR Status**: ✅ Accepted for implementation
