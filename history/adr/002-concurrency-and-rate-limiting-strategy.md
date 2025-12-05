# ADR-002: Concurrency & Rate Limiting Strategy for Gemini API Quota

**Status**: Accepted
**Date**: 2025-12-05
**Feature**: 004-rag-chatbot-api
**Decision ID**: RAG-RATELIMIT-002

---

## Context

The RAG chatbot API must handle multiple concurrent user questions while respecting Gemini API rate limits (5 requests/second per API key). The architectural choice for rate limiting has significant implications for:

- **Throughput**: How many concurrent users can be served
- **Fairness**: Preventing one user from starving others
- **User Experience**: How quickly requests are processed vs queued
- **Complexity**: Implementation difficulty and operational overhead
- **Scaling**: How the system handles burst traffic
- **Reliability**: Graceful degradation under load

The specification requires handling 50+ concurrent requests without errors or degradation (SC-002). Rate limiting is essential for staying within API quota.

---

## Decision

**Chosen Approach**: Python asyncio.Semaphore(5) for in-process rate limiting

**Specific Components**:
1. **Mechanism**: asyncio.Semaphore with capacity 5 (matching 5 req/sec Gemini quota)
2. **Location**: RAGService class initialization
3. **Application**: Guard around `GeminiService.generate_answer()` calls
4. **Behavior When Exceeded**: Return HTTP 429 (Too Many Requests) with Retry-After header
5. **Timeout**: 5 second request timeout (return 503 if semaphore wait exceeds timeout)
6. **Queuing**: No explicit queue; requests either acquire semaphore or get 429
7. **Stateless**: Semaphore is in-process; reset on app restart

---

## Rationale

### Why asyncio.Semaphore(5) Over Alternatives

**asyncio.Semaphore(5)** (Chosen):
- **Simplicity**: 3-4 lines of code to implement
- **Efficiency**: Zero-copy, in-memory synchronization primitive
- **Python Native**: Built-in asyncio, no external dependencies
- **Fast Paths**: Requests within quota acquire immediately (<1ms)
- **Clear Semantics**: Semaphore value matches API quota (5)
- **Async-First**: Perfect for async/await code
- **Adequate Scope**: Sufficient for MVP (single instance deployment)

**Token Bucket Algorithm** (Rejected):
- ❌ More Complex: 20+ lines of code for custom implementation
- ❌ Overkill for MVP: Supports burst capacity (not needed for 5 req/sec flat)
- ❌ Requires Timers: Thread scheduling overhead
- ✅ Better for scaling: Could handle variable rate limits
- **Decision**: Deferred to Phase 5 if scaling requires burst handling

**Redis-Based Rate Limiting** (Rejected):
- ❌ Operational Complexity: Requires Redis instance
- ❌ Latency: Network round-trip for every request check
- ❌ Over-Engineered: MVP runs single instance
- ✅ Perfect for Multi-Instance: Would work with distributed deployment
- **Decision**: Deferred to scaling phase if multi-instance required

**Simple Queue (Queue.Queue)** (Rejected):
- ❌ Less Flexible: Fixed queue size required upfront
- ❌ No Timeout: Hard to implement timeout semantics
- ❌ Thread-Based: asyncio code needs Semaphore, not Queue
- ✅ Thread-Safe: Better if threading were used (not applicable)
- **Decision**: Semaphore is superior for async/await pattern

### Why 429 Response Over Queuing

**Chosen**: Return 429 (Too Many Requests) when semaphore exceeded
- Clients can implement exponential backoff (standard HTTP behavior)
- No server-side queue to manage or monitor
- Clear to client that they're rate limited (not service failure)
- Prevents unbounded queuing (no memory explosion)
- Aligns with REST standards for rate limit responses

**Alternative**: Queue Requests (Rejected)
- ❌ Complex: Maintain queue, dequeue when capacity available
- ❌ Memory Risk: Queue could grow unbounded if Gemini is slow
- ❌ Latency Variance: First user ~0ms, last user ~5000ms (unfair)
- ❌ No Client Signal: Client doesn't know they're waiting
- ✅ Reliability: No user requests dropped

### Why In-Process Semaphore

**Chosen**: asyncio.Semaphore in RAGService (in-process)
- Simple to implement and understand
- No external dependencies
- Perfect latency for MVP
- Suitable for single-instance deployment

**Multi-Instance Alternative** (Deferred):
- Would need Redis-based distributed semaphore
- Adds operational complexity
- Only necessary when scaling to multiple servers
- Deferred to Phase 5 (production scaling)

---

## Consequences

### Positive Outcomes

✅ **Simplicity**: 5-10 lines of code, no external dependencies
✅ **Performance**: Immediate acquisition for requests within quota (<1ms)
✅ **Clear Semantics**: 429 response clearly indicates rate limiting
✅ **Client Compatibility**: Standard HTTP 429 response (clients know how to handle)
✅ **Predictable**: Deterministic behavior (no randomness or complexity)
✅ **Observable**: Can log semaphore state and 429 occurrences

### Tradeoffs & Constraints

⚠️ **No Queuing**: Users at quota limits get 429 (good for fairness, bad for reliability)
⚠️ **Single-Instance Only**: In-process semaphore doesn't work across multiple servers
⚠️ **No Burst Handling**: Cannot exceed 5 req/sec momentarily (strict rate limit)
⚠️ **Reset on Restart**: Quota resets when app restarts (rare, acceptable)
⚠️ **No Time Window Awareness**: Simple count-based (not "5 per second" window)

### Implementation Implications

- Must create asyncio.Semaphore(5) in RAGService.__init__()
- Must acquire semaphore before calling GeminiService.generate_answer()
- Must release semaphore after LLM call (use async context manager)
- Must catch TimeoutError if semaphore.acquire() times out
- Must return HTTP 429 with Retry-After header when semaphore exceeded
- Must log 429 responses for monitoring quota usage

---

## Alternatives Considered

### Alternative 1: Token Bucket Algorithm with Burst Capacity

**Approach**: Implement token bucket to allow burst of 20 requests/sec (4x quota) with recovery to 5 req/sec

**Pros**:
- Better user experience (burst traffic handled without 429)
- Allows momentary overages as long as averaging complies
- Industry standard for rate limiting
- More sophisticated quota management

**Cons**:
- 30+ lines of custom code (vs 5 for Semaphore)
- Requires timer management (background task resetting tokens)
- More complex testing and verification
- Overkill for MVP with steady traffic pattern
- Adds latency for every request (timer check + token decrement)

**Decision**: Rejected for MVP; deferred to Phase 5 if traffic patterns indicate burst handling is needed

### Alternative 2: Distributed Rate Limiting with Redis

**Approach**: Use Redis INCR with expiring keys to track quota across all app instances

**Pros**:
- Works across multiple server instances
- Centralized rate limit tracking
- Flexible quota management (can adjust without restart)
- Industry standard for distributed systems

**Cons**:
- Requires Redis deployment and maintenance
- Network latency for every rate limit check (~5ms)
- Added complexity: Redis client integration, connection pooling
- Cost: Additional service to operate
- Overkill for single-instance MVP deployment

**Decision**: Rejected for MVP; deferred to Phase 5 (production scaling)

### Alternative 3: Request Queuing with Variable Timeout

**Approach**: Queue excess requests, serve them in FIFO order when quota becomes available

**Pros**:
- No requests dropped (better reliability)
- Smooth degradation (users wait instead of failing)
- Could prioritize requests (VIP users served first)

**Cons**:
- Complex implementation (queue management, timeout tracking)
- Unfair: First user ~0ms, last user ~30 seconds (high variance)
- Memory risk: Queue could grow unbounded if Gemini is slow
- Harder to predict/debug (dynamic behavior)
- Users don't know they're waiting (no feedback)

**Decision**: Rejected; 429 response gives clear feedback to clients

### Alternative 4: No Rate Limiting (Trust Gemini Quota)

**Approach**: Don't implement rate limiting, rely on Gemini API to enforce quota

**Pros**:
- Simplest: No code to write
- Let Gemini handle errors (they're responsible)

**Cons**:
- ❌ Error handling: Gemini will return 429, but at API boundary
- ❌ No queueing: Still returns errors to users
- ❌ Harder to control: Can't implement circuit breaker or backoff
- ❌ Poor observability: No log of rate limit events
- ❌ Resource waste: Failed requests consumed compute resources

**Decision**: Rejected; implementing client-side rate limiting gives better control and observability

---

## Implementation Strategy

### Phase 1: Basic Semaphore
```python
# In RAGService.__init__
self.gemini_semaphore = asyncio.Semaphore(5)

# In answer_question method
async with self.gemini_semaphore:
    answer = await self.gemini_service.generate_answer(context, question)
```

### Phase 2: Error Handling
```python
try:
    async with asyncio.timeout(3):  # 3-second max for LLM call
        async with self.gemini_semaphore:
            answer = await self.gemini_service.generate_answer(context, question)
except asyncio.TimeoutError:
    raise HTTPException(status_code=429, headers={"Retry-After": "1"})
```

### Phase 3: Logging & Monitoring
```python
# Log semaphore state periodically
# Alert if 429 rate exceeds threshold (e.g., >50% of requests)
# Dashboard showing quota utilization over time
```

### Phase 4: Metrics & Analysis (Phase 5)
- Track 429 response rate
- Monitor average wait time for quota
- Identify peak traffic patterns
- Determine if scaling to multi-instance needed

---

## Monitoring & Success Metrics

**Rate Limit Compliance**: 100% of requests comply with 5 req/sec quota
**429 Response Rate**: <5% of requests return 429 (buffer for burst)
**Latency Impact**: <1ms overhead for semaphore acquisition (negligible)
**Fairness**: All users experience similar latency (FIFO within quota)
**Observability**: All 429 responses logged with context

---

## Transition Plan (Future Scaling)

### When to Upgrade to Token Bucket
- If sustained request rate exceeds 4 req/sec (touching quota limit)
- If users report frequent 429 responses
- If traffic analysis shows burst patterns (20+ req/sec spikes)

### When to Upgrade to Distributed Rate Limiting
- If deploying multiple app instances (load balancing)
- If quota sharing between instances is required
- If central quota management becomes operational need

---

## Related Decisions

- **ADR-001**: LLM Provider (Gemini with 5 req/sec quota)
- **ADR-003**: Error Handling & Graceful Degradation (429 response handling)

---

## References

- **Plan**: `specs/004-rag-chatbot-api/plan.md` (Phase 0, section 4: Concurrency & Rate Limiting)
- **Specification**: `specs/004-rag-chatbot-api/spec.md` (FR-015: Service failure handling)
- **Implementation**: Task T071-T073 in tasks.md (Rate limiting implementation)

---

## Decision Log

| Date | Status | Author | Notes |
|------|--------|--------|-------|
| 2025-12-05 | Accepted | Architecture Team | Decision finalized during planning phase |

---

## Future Considerations

### Phase 5 Upgrade Path

If traffic analysis indicates need:
1. Replace asyncio.Semaphore with TokenBucket implementation
2. Add distributed rate limiting with Redis when multi-instance deployed
3. Implement circuit breaker pattern if Gemini quota exhausted

---

**ADR Status**: ✅ Accepted for implementation
