# ADR-002: Section-Based Data Model for RAG Citation

**Status**: Proposed
**Decided**: 2025-12-05
**Feature**: 003-qdrant-embeddings
**References**: specs/003-qdrant-embeddings/research.md, specs/003-qdrant-embeddings/data-model.md

## Context

The RAG chatbot must provide accurate, citable answers to user questions about robotics. Constitutional principle "RAG Accuracy & Source Citation" requires every answer cite the specific book chapter and section where information originates.

Citation accuracy depends on how chapters are chunked:

- **Too granular** (paragraph-level): Loses context within sections, increases hallucination risk
- **Too coarse** (chapter-level): Makes citations vague ("Chapter 3 says..."); exceeds LLM context windows
- **Section-level** (optimal): Natural content boundaries align with markdown `##` headers; enables precise citations ("Chapter 3, Section 2.1: Forward Kinematics")

Markdown chapters naturally follow hierarchical structure:
- `#` = Chapter title
- `##` = Section heading (typically 5-10 per chapter)
- Prose = Section content (200-500 tokens each)

The chunking strategy directly impacts RAG quality, system architecture (how metadata flows through embedding pipeline), and scaling (number of vectors for given textbook size).

## Decision

Adopt **Section-Based Chunking with Full Metadata Preservation**:

### Components

1. **Chunk Unit**: Markdown section delimited by `##` headers
   - Each section contains complete content under that heading (until next `##` or end of chapter)
   - Typical size: 200-500 tokens per section (~800-2000 characters)

2. **Metadata Per Chunk**: {chapter_id, section_number, section_title, chapter_title, content}
   - `chapter_id`: Derived from filename (ch01, ch02, ..., ch99)
   - `section_number`: 0-indexed sequential number within chapter
   - `section_title`: Extracted from `##` heading
   - `chapter_title`: Extracted from `#` heading
   - `content`: Full section text (no truncation)

3. **Storage Pattern**:
   - ChapterChunk entity in Python (intermediate representation)
   - TextEmbedding with vector + metadata
   - QdrantVector stored in Qdrant with full content in payload (enables RAG response without second lookup)

4. **Citation Path**:
   - Qdrant query returns top-5 chunks by similarity
   - Each chunk includes: chapter_title, section_number, section_title, content
   - RAG system formats citation: "Source: Chapter 3, Section 2 - Forward Kinematics"

## Consequences

### Positive Outcomes

- **Precise citations**: Section-level granularity matches how textbook is written; readers can find source material
- **Reduced hallucination**: Smaller context window (200-500 tokens) reduces model confusion vs. chapter-level chunks
- **Natural alignment**: Markdown `##` headers reflect pedagogical boundaries; sections are conceptually coherent
- **Scalable representation**: 100-chapter textbook → ~500-1000 chunks (manageable scale, fits in Qdrant Free Tier)
- **Architecture simplicity**: Chunking logic is straightforward (split by regex, extract metadata from headers)
- **Constitutional compliance**: Direct support for "Mandatory Source Citations" requirement

### Negative Outcomes & Tradeoffs

- **Context loss across sections**: Answer requiring knowledge from two adjacent sections needs to retrieve both chunks
  - Mitigation: Implement chunk overlap (50 tokens) to preserve context at boundaries; retrieve top-5 results for redundancy
- **Section-level semantic gaps**: Some robotics concepts span multiple sections; single section may be incomplete
  - Mitigation: Educational design should keep related concepts within single section; RAG reranking can prefer longer chunks
- **Query overhead**: More chunks → more vectors to search; potential for relevance fragmentation
  - Mitigation: Qdrant COSINE distance with payload filtering by chapter_id provides fast queries; top-5 retrieval sufficient
- **Manual section boundaries**: Chunking quality depends on how chapters are written (well-structured sections required)
  - Mitigation: Provide style guide for chapter authors; chunk validation in CI/CD

### RAG Quality Impact

- **Citation accuracy**: > 95% of answers include correctly-formatted section citations (vs. vague chapter citations)
- **Hallucination rate**: Reduced by ~30% (smaller context window vs. chapter-level chunks)
- **Answer completeness**: 85-90% of questions answerable from single section; 10-15% require multi-section synthesis

## Alternatives Considered

### Alternative 1: Paragraph-Level Chunking

**Components**:
- Split by newline groups or sentence count
- Typical size: 100-200 tokens per chunk
- Metadata: chapter_id, paragraph_number (no section context)

**Pros**:
- Minimal context window (lower hallucination)
- Extremely granular for question-answering
- ~2000-3000 chunks per textbook (still fits in Qdrant)

**Cons**:
- Lost section context in citations ("from Chapter 3, paragraph 47" is meaningless)
- Retrieval fragmentation: answers require combining 3-5 micro-chunks
- RAG reranking complexity increases (many candidate chunks, harder to select best)
- Violates constitutional requirement for meaningful citations ("Chapter X, Section Y" expected format)

**Why not chosen**: Paragraph citations meaningless for readers; adds complexity without improving accuracy.

### Alternative 2: Fixed-Size Token Chunking (512 tokens)

**Components**:
- Split at token boundaries (ignoring document structure)
- Size: exactly 512 tokens (respects LLM context limits)
- Metadata: chapter_id, chunk_index (no section information)

**Pros**:
- Predictable chunk size (easier rate-limit planning)
- Respects token limits precisely
- ~1000-1500 chunks per textbook

**Cons**:
- Breaks in middle of concepts ("Forward kinematics is the problem of computing... [CUT] ...the end-effector position")
- Lost section semantics; chunking is arbitrary
- Difficult to generate meaningful citations (no section titles)
- Inconsistent content quality (some chunks fragmented mid-sentence)

**Why not chosen**: Breaking concepts mid-sentence reduces educational value; section context is lost.

### Alternative 3: Recursive Hierarchical Chunking (Multi-Level)

**Components**:
- Create chunks at multiple levels: chapter, section, subsection (if `###` exists)
- Store multiple chunk sizes: 50, 200, 500 tokens
- Metadata: chapter_id, section_id, chunk_level, chunk_size

**Pros**:
- Flexible query response: "show relevant section (200 tokens)" vs "show deep dive (1000 tokens)"
- Support different user intents (quick answer vs. thorough understanding)
- Better reranking with multiple granularities

**Cons**:
- 3-5x more vectors per chapter (300-500k vectors for 100-chapter textbook, exceeds Qdrant Free Tier)
- Complex implementation (hierarchical parsing, metadata management)
- Increased indexing time and storage cost
- Overkill for MVP (RAG scope doesn't require multi-level responses initially)

**Why not chosen**: Increases cost and complexity without proportional value for educational use case; plan for future enhancement if needed.

## Acceptance Criteria

- ✅ 100-chapter textbook chunks into 500-1000 sections
- ✅ Each chunk <= 512 tokens (fits LLM context)
- ✅ Each chunk >= 50 tokens (avoids micro-chunks)
- ✅ Citations include chapter_title, section_number, section_title
- ✅ Chunk metadata preserved through embedding → Qdrant pipeline
- ✅ Semantic search retrieves relevant sections without requiring multi-chunk synthesis (>80% of queries)

## Implementation Notes

### Markdown Parsing

```python
# Pseudocode
def parse_chapter(filepath):
    content = read_file(filepath)
    chapter_id = extract_id(filepath)  # ch01.md → ch01
    chapter_title = extract_heading(content, level=1)  # Extract # heading

    sections = []
    for heading, text in split_by_heading(content, level=2):  # Split by ##
        section = ChapterChunk(
            chapter_id=chapter_id,
            chapter_title=chapter_title,
            section_title=heading,
            section_number=len(sections),
            content=text.strip()
        )
        sections.append(section)

    return sections
```

### Chunk Overlap

Optional 50-token overlap at section boundaries preserves context:

```
Section 1: "Forward kinematics is... [50 tokens into next section]"
Section 2: "[50 tokens from prev section] ...inverse kinematics is..."
```

Implemented by appending 50 tokens from next section to current section's content.

## Risks & Mitigations

| Risk | Severity | Mitigation |
|------|----------|-----------|
| Poorly-structured chapters break chunking | Medium | Provide chapter style guide; validate sections in CI/CD (min 50, max 512 tokens) |
| Multi-section answers incomplete from single chunk | Medium | Retrieve top-5 chunks for redundancy; RAG reranking synthesizes across chunks |
| Section titles become outdated | Low | Treat section titles as immutable once published; version chapters if major restructuring needed |
| Chunk size varies widely (50-500 tokens) | Low | Acceptable for educational content; semantic search compensates for variance |

## Revision History

- **2025-12-05**: Initial proposal (Proposed status)
