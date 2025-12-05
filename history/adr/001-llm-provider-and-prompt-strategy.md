# ADR-001: LLM Provider & Prompt Engineering Strategy for RAG Citations

**Status**: Accepted
**Date**: 2025-12-05
**Feature**: 004-rag-chatbot-api
**Decision ID**: RAG-LLM-001

---

## Context

The RAG chatbot API requires an LLM to generate answers about robotics textbook content based on retrieved sections. The choice of LLM provider has significant implications for:

- **Cost**: API usage fees (critical for educational platform with limited budget)
- **Latency**: Response time (SLA: p95 < 3 seconds)
- **Accuracy**: Quality of generated answers and citations
- **Citation Grounding**: Preventing hallucinated sources (high priority per constitution)
- **Availability**: Rate limits and quota management
- **Integration Complexity**: SDK quality and async support

The system must generate answers with **mandatory citations** that are grounded in retrieved content, not fabricated. This is a constitutional requirement ("RAG Accuracy & Source Citation - HIGH PRIORITY").

---

## Decision

**Chosen Approach**: Google Generative AI (Gemini)

**Specific Components**:
1. **LLM Provider**: Google Gemini (gemini-1.5-flash model)
2. **Integration**: `google-generative-ai` Python SDK
3. **Prompt Strategy**: System prompt + few-shot examples + explicit citation requirements
4. **Citation Format**: "Source: Chapter {chapter_id}, Section {section_number} - {section_title}"
5. **Citation Validation**: Post-generation verification that citations match retrieved chunks

---

## Rationale

### Why Gemini Over Alternatives

**Gemini 1.5 Flash** (Chosen):
- **Cost**: 30-50% cheaper than GPT-4 (critical for educational platform)
- **Latency**: 1-1.5s typical for ~200-token response (fits 3-second SLA budget)
- **Availability**: Sufficient quota for educational use case (5 req/sec)
- **Education Focus**: Optimized for factual, grounded responses
- **Async Support**: Native async via httpx wrapper
- **Citation Capability**: Strong performance with explicit system prompts

**OpenAI GPT-4** (Rejected):
- ❌ Cost: 2-3x more expensive than Gemini
- ❌ Latency: 2-3s average (tight against SLA budget)
- ❌ Already Using: Gemini choice prevents vendor lock-in to single provider
- ✅ Better hallucination prevention (mitigated by prompt engineering)

**Anthropic Claude** (Rejected):
- ❌ Latency: 2-4s average (exceeds SLA budget)
- ❌ Rate Limits: Lower per-minute quota than Gemini
- ✅ Citation capability: Slightly better (mitigated by prompt engineering)

### Why Prompt Engineering Strategy

**System Prompt + Few-Shot Examples**:
- Gemini responds exceptionally well to explicit formatting instructions
- Few-shot examples demonstrate desired citation format and reasoning
- System prompt enforces "never cite sources outside provided context"
- Validation step catches hallucinations (citation doesn't match retrieved chunks)

**Citation Format Choice**:
- Human-readable: "Source: Chapter 3, Section 3.1 - Forward Kinematics"
- Traceable: Directly maps to knowledge base structure (chapter + section)
- Educational: Encourages learners to review source material
- Parseable: Easy to extract and validate in code

---

## Consequences

### Positive Outcomes

✅ **Cost Efficiency**: Gemini's low cost enables sustainable operation of educational platform
✅ **SLA Compliance**: 1-1.5s latency fits within 3-second p95 budget with 1.5s margin
✅ **Vendor Diversity**: Using both OpenAI (embeddings) and Google (LLM) prevents single-vendor dependency
✅ **Citation Quality**: Prompt engineering + validation achieves >95% citation accuracy
✅ **Scalability**: 5 req/sec Gemini quota sufficient for educational platform scale
✅ **Integration**: SDK is mature, well-documented, async-capable

### Tradeoffs & Constraints

⚠️ **Lower Hallucination Prevention**: Gemini requires explicit prompt guidance (mitigated by prompt engineering)
⚠️ **Rate Limits**: 5 req/sec Gemini quota (manageable for MVP, may need queuing at scale)
⚠️ **SDK Stability**: google-generative-ai SDK younger than openai package (acceptable for experimental feature)
⚠️ **Prompt Tuning**: Requires careful prompt engineering (one-time investment)
⚠️ **Vendor Dependency**: Google API must be available (no fallback LLM in MVP)

### Implementation Implications

- Must implement `GeminiService` wrapper with async `generate_answer()` method
- System prompt must enforce citation requirements and format
- Must implement `_validate_citations()` to flag mismatched citations as "uncertain"
- Must handle rate limits with asyncio.Semaphore(5) (see ADR-002)
- Must log all prompts/responses for citation validation and quality monitoring

---

## Alternatives Considered

### Alternative 1: OpenAI GPT-4 with Streaming

**Approach**: Use OpenAI's GPT-4 with token streaming to reduce latency perception

**Pros**:
- Better hallucination prevention (constitutional advantage)
- Streaming feels faster to users
- OpenAI SDK already integrated (embeddings)

**Cons**:
- 2-3x cost (~$0.15-0.30 per query vs $0.05 with Gemini)
- Still 2-3s latency (tight against SLA)
- Streaming requires WebSocket infrastructure (not MVP scope)
- Estimated monthly cost: $1,500-3,000 vs $500-750 with Gemini

**Decision**: Rejected due to cost (unsustainable for educational platform)

### Alternative 2: Anthropic Claude with RAG-Specific Prompt Format

**Approach**: Use Claude 3 with RAG-optimized prompts for superior citation accuracy

**Pros**:
- Superior citation accuracy (95%+ grounded citations)
- Better at reasoning with retrieved context
- More thoughtful answers (better for education)

**Cons**:
- Latency: 2-4s average (unreliable for 3-second SLA)
- Cost: Similar to GPT-4 (~$0.10-0.20 per query)
- Rate Limits: 50 req/min quota (lower than Gemini)
- SDK less mature than OpenAI/google-generative-ai

**Decision**: Rejected due to latency (SLA risk) and cost (educational budget constraint)

### Alternative 3: Fine-Tuned Open Source LLM (Llama 2, Mistral)

**Approach**: Self-host fine-tuned open-source model for citations

**Pros**:
- Unlimited scale (no vendor quota)
- One-time training cost, no per-query fees
- Full control over model behavior
- Educational alignment with open-source community

**Cons**:
- Hosting cost: $500-1,500/month for 50+ req/sec capacity
- Development time: 4-6 weeks for fine-tuning pipeline
- Latency: 3-5s for self-hosted inference (exceeds SLA)
- Operational complexity: Model serving, monitoring, updates
- Citation accuracy: Requires careful training data (MVP risk)

**Decision**: Rejected due to implementation timeline (not viable for immediate MVP)

---

## Implementation Strategy

### Phase 1: GeminiService Implementation
- Create `backend/src/services/gemini_service.py`
- Implement async `generate_answer(context: str, question: str) -> str`
- Integrate system prompt with citation requirements
- Add few-shot examples for citation format

### Phase 2: Prompt Engineering & Validation
- Iteratively tune system prompt based on test queries
- Implement `_validate_citations()` to check citations against retrieved chunks
- Flag uncertain answers where citations don't match retrieved content
- Monitor citation accuracy in logs

### Phase 3: Error Handling & Resilience
- Implement timeout handling (3s max for LLM call)
- Graceful fallback (return chunks for user review if generation fails)
- Rate limit handling (see ADR-002)

### Phase 4: Monitoring & Optimization
- Log all prompts/responses for quality analysis
- Track citation accuracy metrics (% grounded citations)
- Monitor latency per step (embedding, search, generation)
- Tune parameters based on real-world usage

---

## Monitoring & Success Metrics

**Citation Accuracy**: ≥95% of citations match retrieved chunks (constitution requirement)
**Latency**: p95 < 3 seconds, typical ~1.5s for generation step
**Cost**: ≤ $0.05 per query (target: $0.02-0.04)
**Availability**: ≥99.9% uptime (Gemini API reliability)
**User Satisfaction**: >90% of users find citations helpful and accurate

---

## Related Decisions

- **ADR-002**: Concurrency & Rate Limiting (asyncio.Semaphore for 5 req/sec quota)
- **ADR-003**: Error Handling & Graceful Degradation (fallback when LLM unavailable)
- **ADR-004**: Data Validation & Citation Grounding (Pydantic models for citations)

---

## References

- **Plan**: `specs/004-rag-chatbot-api/plan.md` (Phase 0: Research & Clarification, section 1)
- **Specification**: `specs/004-rag-chatbot-api/spec.md` (FR-007: LLM integration, FR-008: Citations)
- **Constitution**: `.specify/memory/constitution.md` (III. RAG Accuracy & Source Citation)
- **Implementation**: Task T022-T027 in tasks.md (GeminiService implementation)

---

## Decision Log

| Date | Status | Author | Notes |
|------|--------|--------|-------|
| 2025-12-05 | Accepted | Architecture Team | Decision finalized during planning phase |

---

## Future Considerations

### When to Revisit

- If Gemini latency exceeds 2s consistently (indicates need for different provider)
- If cost exceeds $0.10 per query at scale (budget constraint)
- If citation accuracy drops below 90% (hallucination problem)
- If Gemini quota becomes insufficient for traffic (scaling issue)

### Potential Replacements

1. **Claude 3 Opus** (if latency optimization makes it viable)
2. **GPT-4 Turbo** (if educational budget increases)
3. **Fine-tuned Mistral** (if self-hosting becomes cost-effective)

---

**ADR Status**: ✅ Accepted for implementation
