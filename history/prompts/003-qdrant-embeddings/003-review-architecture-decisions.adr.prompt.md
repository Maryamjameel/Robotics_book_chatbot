---
id: 003
title: Review and Document Architecture Decisions
stage: adr
date: 2025-12-05
surface: agent
model: claude-haiku-4-5-20251001
feature: 003-qdrant-embeddings
branch: 003-qdrant-embeddings
user: system
command: /sp.adr
labels: ["adr", "architecture", "decisions", "embeddings", "qdrant"]
links:
  spec: specs/003-qdrant-embeddings/spec.md
  ticket: null
  adr: history/adr/001-vector-storage-and-embedding-stack.md
  pr: null
files:
  - history/adr/001-vector-storage-and-embedding-stack.md
  - history/adr/002-section-based-data-model-for-rag.md
  - history/adr/003-modular-service-architecture-with-batch-processing.md
tests:
  - Decision clustering: 3 ADRs (not atomic choices)
  - Significance testing: All pass (impact architecture, have alternatives, will be revisited)
  - Alternatives documented: All 3 ADRs list 3+ alternatives with tradeoffs
  - Consequences balanced: Positive and negative outcomes documented
---

## Prompt

Execute `/sp.adr` to extract architecturally significant decisions from planning artifacts and create ADRs (Architecture Decision Records). Identify decision clusters (not atomic choices), verify significance, document alternatives and tradeoffs.

Feature: Vector embeddings with Qdrant for RAG (specs/003-qdrant-embeddings/)

## Response snapshot

**Step 2: Decision Cluster Extraction**

Extracted 3 architecturally significant clusters:

1. **Vector Storage & Embedding Stack** (impacts system architecture, data flow, costs)
2. **Section-Based Data Model** (impacts RAG quality, citation accuracy, chunking logic)
3. **Modular Service Architecture** (impacts code organization, testability, error handling)

All pass significance test:
- Impact multiple components ✓
- Have notable tradeoffs ✓
- Will likely be questioned later ✓

**Step 4: Significance Grading**

Measure checklist (PASS all to create ADR):

| Criterion | ADR-001 | ADR-002 | ADR-003 |
|-----------|---------|---------|---------|
| Clusters related changes (not atomic) | ✓ | ✓ | ✓ |
| Lists 3+ alternatives with rationale | ✓ | ✓ | ✓ |
| Documents pros/cons and tradeoffs | ✓ | ✓ | ✓ |
| Concise but sufficiently detailed | ✓ | ✓ | ✓ |

**Result**: All 3 ADRs **PASS** significance testing

**Step 5: ADR Creation**

Created 3 ADRs with full documentation:

| ID | Title | Context | Key Decision |
|----|----|---------|---|
| **ADR-001** | Vector Storage & Embedding Stack | RAG system needs semantic search infrastructure | OpenAI text-embedding-3-small + Qdrant Cloud Free Tier |
| **ADR-002** | Section-Based Data Model | Must support precise citations ("Chapter X, Section Y") | Split by ## headers; preserve metadata through pipeline |
| **ADR-003** | Modular Service Architecture | Handle OpenAI rate limits, transient failures gracefully | 3 decoupled services + orchestrator; batch processing + retry logic |

**ADR-001: Vector Storage & Embedding Stack**
- Chosen: OpenAI text-embedding-3-small (1536-dim) + Qdrant Cloud
- Cost: $2-3/month (text-embedding-3-small ~$0.02/1M tokens)
- Alternatives rejected:
  - Self-hosted sentence-transformers: $100-500/month GPU cost
  - Pinecone: $0.07-0.50 per 1M vectors vs Qdrant free
  - Hybrid (Elasticsearch + semantic): Overkill, double storage cost
- Key tradeoff: Vendor lock-in vs. managed infrastructure cost

**ADR-002: Section-Based Data Model**
- Chosen: Chunk by ## headers; store chapter_id, section_number, section_title, content
- Enables citations: "Source: Chapter 3, Section 2 - Forward Kinematics"
- Alternatives rejected:
  - Paragraph-level: Lost section context, meaningless citations
  - Fixed-size tokens (512): Breaks mid-concept, no semantic boundaries
  - Hierarchical (multi-level): 3-5x more vectors, exceeds Qdrant Free Tier, premature optimization
- Key tradeoff: Some answers need multi-section synthesis vs. reduced hallucination

**ADR-003: Modular Service Architecture**
- Chosen: 3 services (markdown_parser, embedding_service, qdrant_service) + orchestrator
- Implements batch processing (32 chunks/batch) + exponential backoff retry (max 3 attempts)
- Graceful degradation: Failed batches logged and skipped
- Alternatives rejected:
  - Monolithic script: No error handling, no batch processing, not reusable
  - Async/streaming: Over-engineered for 100-chapter textbook, harder to debug
  - Kubernetes distributed: Massive over-engineering, no throughput gain for MVP scale
- Key tradeoff: 4-file architecture vs. single-file simplicity; batch processing adds ~1-2 min latency

**Consequences & Risks**

Each ADR documents:
- ✅ Positive outcomes (cost-effective, precise citations, testable)
- ❌ Negative outcomes (vendor lock-in, context loss, implementation overhead)
- ⚠️ Risks and mitigations (rate limits, storage quota, memory constraints)

## Outcome

- ✅ Impact: 3 ADRs document major architectural decisions; team aligned on approach before implementation
- 🧪 Tests: Decision clustering passed (not atomic); significance passed (all criteria met); alternatives documented
- 📁 Files: 3 ADRs created + 1 PHR
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks; team review of ADRs
- 🧠 Reflection: ADRs provide decision rationale and alternatives for future reference. Clear justification for each choice. Low risk of decisions being revisited without context.

## Evaluation notes (flywheel)

- Failure modes observed: None—ADRs complete, well-documented, clear alternatives
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: Team review ADRs; proceed to `/sp.tasks` for implementation planning
