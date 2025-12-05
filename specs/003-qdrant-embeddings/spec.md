# Feature Specification: Vector Embeddings with Qdrant for RAG

**Feature Branch**: `003-qdrant-embeddings`
**Created**: 2025-12-05
**Status**: Draft
**Input**: Set up vector embeddings with Qdrant for RAG with text chunking, embedding generation, and Qdrant insertion pipeline.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Index Chapter Content for Semantic Search (Priority: P1)

Content creators want to make chapter markdown files searchable by semantic meaning. Currently, chapters cannot be found by topic without manual full-text search. This feature enables users to upload chapters once and query them by meaning (e.g., "How do I implement inverse kinematics?" returns relevant sections).

**Why this priority**: Core RAG functionality—without indexed content, the system cannot answer questions. This is the MVP prerequisite.

**Independent Test**: Can be fully tested by uploading a chapter and verifying vectors are stored in Qdrant with correct metadata. Delivers immediate searchability.

**Acceptance Scenarios**:

1. **Given** a markdown chapter file exists in `frontend/docs/chapters/`, **When** the chunking script processes it, **Then** sections are split by `##` headers and stored with chapter_id, title, section_number, and content metadata.
2. **Given** chunked sections exist, **When** the embedding script generates vectors, **Then** each section receives a 768-dimensional vector embedding without errors.
3. **Given** vectors are generated, **When** the insertion script runs, **Then** all vectors and metadata are stored in Qdrant's `chapter_embeddings` collection.
4. **Given** vectors are stored in Qdrant, **When** a semantic query is executed, **Then** relevant sections are returned ranked by COSINE distance similarity.

---

### User Story 2 - Batch Process All Chapters (Priority: P2)

Content administrators need to process multiple chapters in one operation without manual intervention for each file. Currently, processing chapters requires running multiple scripts sequentially with error handling.

**Why this priority**: Operational efficiency—enables administrators to index large textbooks without manual script execution between steps.

**Independent Test**: Can be fully tested by running the end-to-end pipeline and verifying all chapters are indexed with error reporting and graceful failure handling.

**Acceptance Scenarios**:

1. **Given** multiple chapter files exist in `frontend/docs/chapters/`, **When** the pipeline script runs, **Then** all chapters are processed through chunking, embedding, and insertion in sequence.
2. **Given** an error occurs during embedding generation (rate limit, API timeout), **When** the pipeline encounters the error, **Then** it logs the error, retries with exponential backoff, and continues processing remaining chapters.
3. **Given** the pipeline completes, **When** it finishes, **Then** a summary report shows processed chapters, skipped files, and any errors encountered.

---

### User Story 3 - Verify Embedding Quality (Priority: P3)

Data engineers want to validate that embeddings are correctly stored and queryable before users rely on them. Currently, there is no verification mechanism after insertion.

**Why this priority**: Quality assurance—ensures data integrity before production use. Enables debugging of malformed vectors.

**Independent Test**: Can be tested by querying Qdrant with a known vector and verifying correct metadata is returned and payload indexes are working.

**Acceptance Scenarios**:

1. **Given** vectors are inserted into Qdrant, **When** a verification query is run, **Then** a sample of vectors is retrieved and compared against expected metadata.
2. **Given** vectors are retrieved, **When** payload index filtering is tested, **Then** queries can filter by chapter_id, section_number, and title without performance degradation.

---

### Edge Cases

- What happens when a chapter has no `##` headers (flat structure)?
- How does the system handle encoding issues in markdown files (UTF-8, special characters)?
- What occurs if the Qdrant instance is unavailable during insertion?
- How does the system handle duplicate chapters (same content, different files)?
- What happens if embedding API rate limits are exceeded mid-batch?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST read all markdown files from `frontend/docs/chapters/` directory.
- **FR-002**: System MUST split chapter content by `##` section headers, extracting chapter_id, title, section_number, and content as metadata.
- **FR-003**: System MUST generate 768-dimensional text embeddings for each chunk without truncating content.
- **FR-004**: System MUST implement batch processing with configurable batch size for embedding generation.
- **FR-005**: System MUST handle embedding API rate limits with exponential backoff retry logic.
- **FR-006**: System MUST connect to a Qdrant instance and insert vectors with payload containing chapter metadata.
- **FR-007**: System MUST create a Qdrant collection named `chapter_embeddings` with COSINE distance metric.
- **FR-008**: System MUST index payload fields: chapter_id, section_number, title for filtering queries.
- **FR-009**: System MUST orchestrate the full pipeline (chunking → embedding → insertion) with error handling.
- **FR-010**: System MUST log all operations and errors with timestamps for debugging.
- **FR-011**: System MUST verify insertion success and report results (processed count, skipped, errors).

### Key Entities

- **ChapterChunk**: Represents a section of a chapter. Attributes: content (string), chapter_id (string), title (string), section_number (integer), metadata (dict).
- **Vector**: Text embedding generated from chunk content. Attributes: embedding (float array, 768-dim), chunk_id (unique identifier), metadata (chapter_id, section_number, title).
- **QdrantCollection**: Remote vector store for semantic search. Attributes: name ("chapter_embeddings"), vector_size (768), distance_metric (COSINE), payload_indexes (chapter_id, section_number, title).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All chapter markdown files from `frontend/docs/chapters/` are successfully chunked and indexed within 5 minutes for a typical 100-chapter textbook.
- **SC-002**: 99% of embeddings are successfully generated without data loss (failures are logged and retried).
- **SC-003**: Semantic queries return relevant sections in under 500ms (measured from Qdrant response time).
- **SC-004**: Pipeline execution is fully automated with zero manual intervention—administrators run one command and all chapters are indexed.
- **SC-005**: Error logs clearly identify root cause of any failures (file format, API limit, connection issue) for debugging.

## Assumptions

1. Embedding model is externally provided (user will specify during configuration). Default assumed to be OpenAI's text-embedding-3-small or equivalent.
2. `frontend/docs/chapters/` directory structure is flat (all chapters in one directory, not nested).
3. Markdown files follow consistent naming and `##` header structure for section delimitation.
4. Qdrant instance is already running and accessible at a configured endpoint.
5. Batch size and retry configuration are sensible defaults (e.g., 32-token batch, 3 retries with exponential backoff).

## Out of Scope

- User interface for manual chunk editing or embedding verification (handled in future UI feature).
- Real-time indexing of newly added chapters (handled in separate feature).
- Alternative embedding models beyond the configured model (model selection is deployment-time configuration).
- Vector retrieval and ranking for chatbot answers (handled in separate RAG feature).
