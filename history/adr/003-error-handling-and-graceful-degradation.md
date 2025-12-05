# ADR-003: Error Handling & Graceful Degradation Strategy for Service Failures

**Status**: Accepted
**Date**: 2025-12-05
**Feature**: 004-rag-chatbot-api
**Decision ID**: RAG-ERRORS-003

---

## Context

The RAG chatbot API depends on two external services:
1. **Qdrant**: Vector database for semantic search
2. **Gemini**: LLM for answer generation

Both services can fail temporarily (network outages, quota limits, maintenance). The API must handle these failures gracefully to maintain availability and provide useful error messages to clients. The architecture must balance:

- **Availability**: Keep serving requests even if services fail
- **User Experience**: Clear error messages (not generic "500 Internal Server Error")
- **Data Consistency**: Don't fabricate data when services are unavailable
- **Observability**: Log failures for debugging and monitoring
- **Compliance**: Meet 99.9% uptime SLA (≤0.1% 5xx errors)

---

## Decision

**Chosen Approach**: Graceful Degradation with Service-Specific Error Responses

**Error Handling Strategy**:

| Service | Failure Mode | HTTP Status | Response | User-Facing Message |
|---------|--------------|------------|----------|-------------------|
| **Qdrant** | Offline / Timeout | 503 | JSON error | "Vector search temporarily unavailable. Please try again." |
| **Qdrant** | No Results Found | 200 | ChatResponse | "No relevant content found in the textbook." |
| **Gemini** | Rate Limited | 429 | JSON error | "Rate limit exceeded. Maximum 5 requests per second." |
| **Gemini** | Offline / Timeout | 503 | JSON error | "LLM service temporarily unavailable. Please try again." |
| **Gemini** | Generation Fails | 503 | Fallback | Return retrieved chunks for user review |
| **Input** | Invalid Question | 400 | JSON error | Validation error details |
| **Pipeline** | Timeout Exceeded | 503 | JSON error | "Request processing timed out. Please try again." |

**Specific Behaviors**:

1. **Qdrant Offline**: Return 503 immediately, don't attempt LLM generation
2. **Gemini Offline**: Return 503, but include retrieved chunks so user has reference material
3. **No Relevant Results**: Return 200 (success), with answer "No relevant content found", confidence=0
4. **Question Too Long**: Return 400 with validation error
5. **Timeout Any Step**: Return 503 with descriptive timeout message
6. **Cascade Failure**: If multiple services fail, return earliest error (user knows what failed first)

---

## Rationale

### Why Graceful Degradation Over Fail-Fast

**Graceful Degradation** (Chosen):
- **User-Centric**: Users get partial results (chunks) rather than error
- **Helpful**: Chunks are better than nothing (educational value)
- **Transparent**: Clear messages about what failed
- **Resilient**: System remains useful even during outages
- **Constitutional**: Aligns with "Graceful Degradation" principle

**Fail-Fast** (Rejected):
- ❌ Harsh UX: Returns generic 500 error
- ❌ Less Helpful: No content provided to user
- ❌ Poor Observability: Doesn't differentiate failure types
- ✅ Simpler: Less code to maintain
- **Decision**: User experience outweighs simplicity

### Why Service-Specific HTTP Status Codes

**Chosen**: Map each failure to appropriate HTTP status
- **400**: Invalid input (client error, client can retry with valid input)
- **429**: Rate limited (client should back off exponentially)
- **503**: Service unavailable (client can retry immediately or after Retry-After)

**Alternative** (Rejected): Always return 500 for any service failure
- ❌ Loses information: Client doesn't know if it's their fault or service fault
- ❌ Wrong semantics: 500 means "server error", but rate limit is temporary quota issue
- ❌ Bad UX: Client can't implement intelligent retry logic

### Why Return Chunks on Gemini Failure

**Chosen**: When Gemini fails, return retrieved chunks for user review

```json
{
  "answer": "Generation failed. Retrieved relevant content below for manual review:",
  "sources": [
    { "chapter_id": "ch03", "section_title": "Forward Kinematics", "excerpt": "...", "relevance_score": 0.89 },
    { "chapter_id": "ch03", "section_title": "DH Convention", "excerpt": "...", "relevance_score": 0.76 }
  ],
  "metadata": { "confidence_score": 0.0, "error": "LLM service unavailable" }
}
```

**Rationale**:
- User has already invested in asking question
- Chunks alone provide educational value
- User can read chunks manually (slow but functional)
- Better than returning nothing (empty response)
- Constitutional requirement: "System remains functional when non-critical services fail"

**Alternative** (Rejected): Return 503 without chunks
- ❌ No value to user: Empty error response
- ❌ Wasted vector search: Why did we search if not using results?
- ✅ Simpler: Less data to return

### Why 200 for No Results (Not 204)

**Chosen**: Return HTTP 200 with answer "No relevant content found"

```json
{
  "answer": "No relevant content found in the textbook about your query.",
  "sources": [],
  "metadata": { "confidence_score": 0.0, "num_sources_retrieved": 0 }
}
```

**Rationale**:
- 200 means "request processed successfully" (true, nothing failed)
- No results is not an error (acceptable outcome)
- ChatResponse schema is consistent (always answer + sources + metadata)
- User can adjust query and retry

**Alternative** (Rejected): Return 204 No Content
- ❌ No response body: Can't explain why no results
- ❌ Client confusion: 204 implies no data to return
- ✅ Simpler: Fewer bytes sent
- **Decision**: Clarity outweighs bandwidth savings

---

## Consequences

### Positive Outcomes

✅ **User Experience**: Clear, actionable error messages
✅ **Availability**: System remains useful during partial outages
✅ **Observability**: Service-specific HTTP codes aid debugging
✅ **Intelligent Retries**: Clients can implement appropriate retry logic
✅ **Educational Value**: Users get chunks even when LLM fails
✅ **Compliance**: 99.9% uptime achievable with graceful degradation
✅ **Trust**: Users see we handle failures professionally

### Tradeoffs & Constraints

⚠️ **More Code**: Multiple error paths (simpler to just throw 500)
⚠️ **Complexity**: Testing all failure scenarios is tedious
⚠️ **Monitoring**: More error types to track and alert on
⚠️ **Partial Data**: Returning chunks without answer may confuse some users
⚠️ **No Automatic Failover**: No backup LLM if Gemini fails (acceptable for MVP)

### Implementation Implications

- Must implement timeout handling at multiple levels (search, generation, total)
- Must catch and map exceptions to HTTP status codes
- Must include Retry-After header for 429 responses
- Must log all errors with context for debugging
- Must return detailed, user-friendly error messages
- Must test all error paths (timeout, offline, rate limit, validation)

---

## Alternatives Considered

### Alternative 1: Automatic Failover to Secondary LLM

**Approach**: If Gemini fails, automatically try OpenAI GPT-4 as fallback

**Pros**:
- Higher availability (very unlikely both fail simultaneously)
- Seamless to user (still returns generated answer)
- Better than returning chunks alone

**Cons**:
- Cost: Have to pay for 2 LLMs even if one is rarely used
- Complexity: Manage credentials for 2 providers
- Latency: Fallover adds 3s timeout to generation time
- Not needed for MVP: Gemini availability is >99.9%

**Decision**: Rejected for MVP; deferred to Phase 5 if Gemini reliability becomes issue

### Alternative 2: Return 503 for All Service Failures (No Chunks)

**Approach**: Strict fail-fast: if any service unavailable, return 503 empty response

**Pros**:
- Simpler code (no fallback logic)
- Clear to user: Service is down
- No partial/confusing responses

**Cons**:
- No value to user (empty error response)
- Vector search wasted (search succeeded, generation failed)
- Lower perceived availability (all users see 503)
- Constitutional violation: "System remains functional"

**Decision**: Rejected; graceful degradation provides better UX

### Alternative 3: Implement Internal Retry Loop with Exponential Backoff

**Approach**: Automatically retry failed LLM calls with 1s, 2s, 4s delays

**Pros**:
- Transient failures recover automatically
- User doesn't see errors from temporary glitches
- Higher success rate

**Cons**:
- Adds latency (failed retry + exponential backoff can be 7 seconds)
- Violates 3-second SLA during retry
- Can exacerbate rate limiting (retries use more quota)
- Gemini already does exponential backoff internally

**Decision**: Rejected; rely on client-side retries (clients have context, can backoff intelligently)

### Alternative 4: Circuit Breaker Pattern for Failing Services

**Approach**: If Gemini fails 5 times in row, stop trying and return 503 immediately

**Pros**:
- Prevents cascading failures
- Faster error response when service is down
- Reduces wasted requests to failing service

**Cons**:
- Complexity: Manage circuit state, recovery
- Overkill for MVP (services usually all-or-nothing)
- Harder to test and debug
- Requires monitoring to detect when to reset

**Decision**: Deferred to Phase 5; add if observability indicates repeated failures

---

## Implementation Strategy

### Phase 1: Basic Error Handling
```python
try:
    # Search
    chunks = await qdrant_search(question)
except TimeoutError:
    raise HTTPException(status_code=503, detail="Vector search timed out")
except Exception as e:
    raise HTTPException(status_code=503, detail="Vector search failed")

try:
    # Generate
    answer = await gemini_generate(context)
except HTTPException as e:
    if e.status_code == 429:
        raise  # Re-raise rate limit
    else:
        # Return chunks as fallback
        return ChatResponse(answer="Generation failed. Here's retrieved content:", sources=chunks)
```

### Phase 2: Request Validation
```python
if not request.question or len(request.question) > 2000:
    raise HTTPException(status_code=400, detail="Question must be 1-2000 characters")
```

### Phase 3: Timeout Handling
```python
try:
    async with asyncio.timeout(total_timeout=5):
        chunks = await qdrant_search()  # 1.5s max
        answer = await gemini_generate()  # 3s max
except asyncio.TimeoutError:
    raise HTTPException(status_code=503, detail="Request processing timed out")
```

### Phase 4: Logging & Monitoring
```python
logger.error("Gemini generation failed", extra={
    "service": "gemini",
    "error_type": type(e).__name__,
    "request_id": request_id,
    "status_code": 503
})
```

### Phase 5: Rate Limit Handling
```python
if isinstance(e, RateLimitError):
    raise HTTPException(
        status_code=429,
        detail="Rate limit exceeded",
        headers={"Retry-After": "1"}
    )
```

---

## Error Response Format

**Successful Response** (200):
```json
{
  "answer": "...",
  "sources": [...],
  "metadata": { "confidence_score": 0.92 }
}
```

**Error Response** (400/429/503):
```json
{
  "detail": "Vector search temporarily unavailable. Please try again."
}
```

**Fallback Response When Gemini Fails** (503):
```json
{
  "answer": "Generation service temporarily unavailable. Retrieved content below:",
  "sources": [
    { "chapter_id": "ch03", "section_title": "Forward Kinematics", "excerpt": "...", "relevance_score": 0.89 }
  ],
  "metadata": { "confidence_score": 0.0, "error_type": "lm_service_unavailable" }
}
```

---

## Monitoring & Success Metrics

**Error Rate Tracking**: <0.1% of requests return 5xx errors (99.9% SLA)
**Error Type Distribution**: Monitor 400/429/503 to understand failure patterns
**User-Facing Clarity**: Error messages must be actionable (not generic)
**Retry Success Rate**: >80% of client retries succeed (indicates transient failures)

---

## Related Decisions

- **ADR-001**: LLM Provider (Gemini API rate limits → 429 handling)
- **ADR-002**: Rate Limiting (429 response implementation)

---

## References

- **Plan**: `specs/004-rag-chatbot-api/plan.md` (Phase 0, section 5: Error Handling)
- **Specification**: `specs/004-rag-chatbot-api/spec.md` (FR-011: Error handling, FR-015: Service failures)
- **Constitution**: `.specify/memory/constitution.md` (I. Production-Grade Quality, IV. Modular Architecture)
- **Implementation**: Task T037, T070 in tasks.md (Error handling and service failure management)

---

## Decision Log

| Date | Status | Author | Notes |
|------|--------|--------|-------|
| 2025-12-05 | Accepted | Architecture Team | Decision finalized during planning phase |

---

## Future Considerations

### When to Add Circuit Breaker
- If Gemini fails consistently (>5 failures/min)
- If observability shows cascading failures downstream

### When to Add Automatic Failover
- If Gemini reliability <99.5%
- If cost allows secondary LLM budget
- If distributed deployment requires high availability

---

**ADR Status**: ✅ Accepted for implementation
