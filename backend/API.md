# API Reference

Complete API documentation for the Robotics Book Embeddings Pipeline.

## Table of Contents

1. [Services](#services)
2. [Models](#models)
3. [Configuration](#configuration)
4. [Error Handling](#error-handling)
5. [Examples](#examples)

## Services

### Markdown Parser Service

Module: `src.services.markdown_parser`

#### `parse_chapter(chapter_path: str, chapter_id: str) -> List[ChapterChunk]`

Parse a single markdown chapter file and extract sections.

**Parameters:**
- `chapter_path` (str): Absolute path to the markdown file
- `chapter_id` (str): Unique identifier for the chapter (e.g., "ch01")

**Returns:**
- `List[ChapterChunk]`: List of section chunks extracted from the chapter

**Raises:**
- `FileNotFoundError`: If the chapter file does not exist
- `ValueError`: If the markdown format is invalid or file is empty

**Example:**
```python
from src.services import parse_chapter

chunks = parse_chapter("chapters/ch01.md", "ch01")
for chunk in chunks:
    print(f"Section {chunk.section_number}: {chunk.section_title}")
    print(f"Content: {chunk.content[:100]}...")
```

#### `parse_all_chapters(chapters_dir: str) -> List[ChapterChunk]`

Parse all markdown files in a directory and extract sections.

**Parameters:**
- `chapters_dir` (str): Path to directory containing markdown files

**Returns:**
- `List[ChapterChunk]`: All chunks from all chapters, sorted by chapter ID

**Raises:**
- `FileNotFoundError`: If the directory does not exist
- `ValueError`: If path is not a directory

**Example:**
```python
from src.services import parse_all_chapters

all_chunks = parse_all_chapters("./data/chapters")
print(f"Total sections: {len(all_chunks)}")
```

---

### Embedding Service

Module: `src.services.embedding_service`

#### `embed_chunks(chunks: List[ChapterChunk]) -> List[TextEmbedding]`

Generate embeddings for a list of chapter chunks.

**Parameters:**
- `chunks` (List[ChapterChunk]): Chunks to embed

**Returns:**
- `List[TextEmbedding]`: Embeddings with 1536-dimensional vectors

**Raises:**
- `ValueError`: If chunk list is empty
- `RuntimeError`: If embedding generation fails after all retries

**Features:**
- Exponential backoff retry (2^attempt seconds, max 3 attempts)
- Batch processing (default 32 chunks per API call)
- Graceful error logging

**Example:**
```python
from src.services import parse_all_chapters, embed_chunks

chunks = parse_all_chapters("./chapters")
embeddings = embed_chunks(chunks)
print(f"Generated {len(embeddings)} embeddings")
```

#### `embed_batch(texts: List[str], chunk_ids: List[str], metadata_list: List[dict]) -> List[TextEmbedding]`

Generate embeddings for a batch of texts with exponential backoff retry logic.

**Parameters:**
- `texts` (List[str]): List of text strings to embed
- `chunk_ids` (List[str]): Corresponding chunk identifiers
- `metadata_list` (List[dict]): Corresponding metadata dictionaries

**Returns:**
- `List[TextEmbedding]`: Embeddings with metadata

**Raises:**
- `ValueError`: If lists have different lengths or contain empty values
- `RuntimeError`: If all retry attempts fail

**Retry Logic:**
- Attempt 1: 1 second delay
- Attempt 2: 2 second delay
- Attempt 3: 4 second delay
- After 3 attempts: RuntimeError raised

**Example:**
```python
from src.services import embed_batch

texts = ["Text 1", "Text 2", "Text 3"]
chunk_ids = ["c1", "c2", "c3"]
metadata = [{"id": 1}, {"id": 2}, {"id": 3}]

embeddings = embed_batch(texts, chunk_ids, metadata)
```

---

### Qdrant Service

Module: `src.services.qdrant_service`

#### `initialize_collection(collection_name: Optional[str] = None) -> bool`

Initialize or verify Qdrant collection configuration.

**Parameters:**
- `collection_name` (Optional[str]): Collection name (uses config default if None)

**Returns:**
- `bool`: True if collection is ready

**Collection Configuration:**
- Vector size: 1536 (OpenAI text-embedding-3-small)
- Distance metric: COSINE
- Payload indexed on: chapter_id, section_number, section_title

**Raises:**
- `RuntimeError`: If collection initialization fails

**Example:**
```python
from src.services import initialize_collection

initialize_collection("robotics_chapters")
```

#### `insert_embeddings(embeddings: List[TextEmbedding], collection_name: Optional[str] = None) -> InsertionResult`

Insert embeddings into Qdrant collection using upsert (idempotent).

**Parameters:**
- `embeddings` (List[TextEmbedding]): Embeddings to insert
- `collection_name` (Optional[str]): Collection name (uses config default if None)

**Returns:**
- `InsertionResult`: Statistics on success/failure

**Returns Example:**
```python
{
    "total": 100,
    "inserted": 98,
    "failed": 2,
    "errors": [
        {"chunk_id": "ch01_sec05", "error": "Vector dimension mismatch"}
    ]
}
```

**Raises:**
- `ValueError`: If embeddings list is empty
- `RuntimeError`: If all insertions fail

**Features:**
- Idempotent upsert (safe to run multiple times)
- Graceful degradation (continues on partial failures)
- Error logging for failed chunks

**Example:**
```python
from src.services import embed_chunks, insert_embeddings, parse_all_chapters

chunks = parse_all_chapters("./chapters")
embeddings = embed_chunks(chunks)
result = insert_embeddings(embeddings, "robotics_chapters")
print(f"Inserted: {result.inserted}/{result.total}")
```

#### `verify_insertion(collection_name: Optional[str] = None, sample_size: int = 50) -> VerificationResult`

Verify embeddings in Qdrant collection.

**Parameters:**
- `collection_name` (Optional[str]): Collection name (uses config default if None)
- `sample_size` (int): Number of points to sample (default 50)

**Returns:**
- `VerificationResult`: Validation details

**Verification Checks:**
- Vector dimensions match (1536)
- Payload schema consistency
- Collection statistics

**Returns Example:**
```python
{
    "total_points": 500,
    "sampled": 50,
    "valid": 50,
    "invalid": 0,
    "checks": {
        "vector_dimension": 1536,
        "payload_keys": ["chapter_id", "section_title"],
        "sample_valid_percentage": 100.0
    }
}
```

**Raises:**
- `RuntimeError`: If collection doesn't exist or is empty

**Example:**
```python
from src.services import verify_insertion

result = verify_insertion(sample_size=100)
print(f"Valid: {result.valid}/{result.sampled}")
```

---

### Orchestrator Service

Module: `src.services.orchestrator`

#### `run_pipeline(chapters_dir: str, collection_name: Optional[str] = None, verify: bool = True) -> PipelineResult`

Run the complete embedding pipeline: parse → embed → insert → verify.

**Parameters:**
- `chapters_dir` (str): Directory containing markdown chapter files
- `collection_name` (Optional[str]): Qdrant collection name
- `verify` (bool): Whether to verify insertion results (default True)

**Returns:**
- `PipelineResult`: Execution status and statistics

**Pipeline Steps:**
1. Parse all markdown files in chapters_dir
2. Generate embeddings for each section
3. Initialize Qdrant collection
4. Insert embeddings (upsert)
5. Verify insertion quality (optional)

**Raises:**
- `ValueError`: If chapters_dir is invalid or no chapters found
- `RuntimeError`: If critical pipeline step fails

**Features:**
- Comprehensive error handling with detailed logging
- Graceful partial failures (continues on non-critical errors)
- Full execution statistics and warnings

**Returns Example:**
```python
{
    "status": "success",
    "chapters_parsed": 10,
    "embeddings_generated": 50,
    "embeddings_inserted": 49,
    "embeddings_failed": 1,
    "total_errors": 0,
    "total_warnings": 1,
    "errors": [],
    "warnings": [
        {
            "step": "insert_embeddings",
            "chunk_id": "ch02_sec03",
            "error": "Duplicate point"
        }
    ],
    "verification": {...}  # VerificationResult if verify=True
}
```

**Example:**
```python
from src.services import run_pipeline

result = run_pipeline(
    chapters_dir="./data/chapters",
    collection_name="robotics_chapters",
    verify=True
)

if result.status == "success":
    print(f"✓ Pipeline completed!")
    print(f"  Parsed: {result.chapters_parsed}")
    print(f"  Embedded: {result.embeddings_generated}")
    print(f"  Inserted: {result.embeddings_inserted}")
else:
    print(f"✗ Pipeline failed: {result.status}")
    for error in result.errors:
        print(f"  - {error}")
```

---

### Verification Service

Module: `src.services.verification_service`

#### `verify_embedding_quality(collection_name: Optional[str] = None, sample_size: int = 100) -> EmbeddingQualityReport`

Verify quality of embeddings in Qdrant collection.

**Parameters:**
- `collection_name` (Optional[str]): Collection name
- `sample_size` (int): Number of embeddings to sample

**Returns:**
- `EmbeddingQualityReport`: Quality metrics

**Quality Checks:**
- Vector dimensions (should be 1536)
- Vector magnitudes (normalized vectors ~1.0)
- Payload completeness (required fields present)
- Duplicate detection

**Returns Example:**
```python
{
    "vector_dimension_match": true,
    "vector_dimension": 1536,
    "vector_magnitude_valid": true,
    "average_magnitude": 1.0,
    "magnitude_variance": 0.001,
    "payload_completeness": 100.0,
    "duplicate_vectors": 0,
    "tests_passed": 5,
    "tests_total": 5,
    "pass_rate": 100.0
}
```

**Raises:**
- `RuntimeError`: If collection is empty or unreachable

**Example:**
```python
from src.services import verify_embedding_quality

report = verify_embedding_quality(sample_size=200)
if report.pass_rate == 100.0:
    print("✓ All quality checks passed!")
else:
    print(f"⚠ Pass rate: {report.pass_rate}%")
```

#### `verify_rag_accuracy(collection_name: Optional[str] = None, num_queries: int = 10, top_k: int = 5) -> dict`

Verify RAG accuracy by testing search quality.

**Parameters:**
- `collection_name` (Optional[str]): Collection name
- `num_queries` (int): Number of test queries to perform
- `top_k` (int): Number of results to retrieve per query

**Returns:**
- `dict`: RAG accuracy metrics

**RAG Checks:**
- Search success rate
- Average results per query
- Chapter diversity in results
- Citation coverage (metadata completeness)

**Returns Example:**
```python
{
    "total_searches": 10,
    "successful_searches": 10,
    "search_success_rate": 100.0,
    "average_results_per_search": 5.0,
    "diverse_chapters": 8,
    "results_with_citations": 48,
    "citation_coverage": 96.0
}
```

**Raises:**
- `RuntimeError`: If collection is empty or search fails

**Example:**
```python
from src.services import verify_rag_accuracy

metrics = verify_rag_accuracy(num_queries=20, top_k=10)
print(f"Search success: {metrics['search_success_rate']}%")
print(f"Citation coverage: {metrics['citation_coverage']}%")
```

---

## Models

### ChapterChunk

```python
from src.models import ChapterChunk

chunk = ChapterChunk(
    chapter_id="ch01",
    chapter_title="Introduction to Robotics",
    section_number=1,
    section_title="Overview",
    content="Robotics is an interdisciplinary field...",
    metadata={"source_file": "ch01.md"}
)
```

**Fields:**
- `chapter_id` (str): Unique chapter identifier
- `chapter_title` (str): Chapter title
- `section_number` (int): Sequential section number
- `section_title` (str): Section title
- `content` (str): Text content of section
- `metadata` (Dict): Additional metadata for citations

### TextEmbedding

```python
from src.models import TextEmbedding

embedding = TextEmbedding(
    chunk_id="ch01_sec01",
    vector=[0.001, 0.002, ...],  # 1536 values
    metadata={"chapter_id": "ch01"}
)
```

**Fields:**
- `chunk_id` (str): Unique chunk identifier
- `vector` (List[float]): 1536-dimensional embedding
- `metadata` (Dict): Metadata from source chunk

### InsertionResult

```python
{
    "total": 100,
    "inserted": 98,
    "failed": 2,
    "errors": [...]
}
```

### VerificationResult

```python
{
    "total_points": 500,
    "sampled": 50,
    "valid": 50,
    "invalid": 0,
    "checks": {...}
}
```

### PipelineResult

```python
{
    "status": "success",
    "chapters_parsed": 10,
    "embeddings_generated": 50,
    "embeddings_inserted": 50,
    "embeddings_failed": 0,
    "errors": [],
    "warnings": [],
    "verification": None
}
```

---

## Configuration

### Environment Variables

```env
# Required
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...

# Optional
EMBEDDING_MODEL=text-embedding-3-small
QDRANT_API_KEY=...
COLLECTION_NAME=robotics_chapters
EMBEDDING_BATCH_SIZE=32
MAX_RETRIES=3
INITIAL_BACKOFF_SECONDS=1
LOG_LEVEL=INFO
LOG_FILE=pipeline.log
LOG_TO_CONSOLE=true
```

### Configuration Class

```python
from src.config import config

print(config.openai_api_key)
print(config.qdrant_url)
print(config.vector_size)  # 1536
print(config.distance_metric)  # cosine
```

---

## Error Handling

### Common Exceptions

#### FileNotFoundError
- **When**: Chapter file or directory not found
- **Handle**: Check file paths, ensure files exist
```python
try:
    chunks = parse_chapter("nonexistent.md", "ch01")
except FileNotFoundError as e:
    print(f"File not found: {e}")
```

#### ValueError
- **When**: Invalid input data or format
- **Handle**: Validate inputs before calling services
```python
try:
    embeddings = embed_chunks([])
except ValueError as e:
    print(f"Invalid input: {e}")
```

#### RuntimeError
- **When**: API calls fail after retries or Qdrant errors
- **Handle**: Check credentials, network, and service availability
```python
try:
    embeddings = embed_chunks(chunks)
except RuntimeError as e:
    print(f"Embedding failed: {e}")
```

### Logging

All operations log in JSON format:

```python
from src.config import logger

logger.info(
    "Processing started",
    extra={
        "operation": "parse_chapters",
        "status": "starting",
        "chapters_dir": "./data"
    }
)
```

---

## Examples

### Complete Pipeline

```python
from src.services import run_pipeline

# Run everything in one call
result = run_pipeline("./chapters", verify=True)

print(f"Status: {result.status}")
print(f"Chapters: {result.chapters_parsed}")
print(f"Embedded: {result.embeddings_generated}")
print(f"Inserted: {result.embeddings_inserted}")

if result.verification:
    print(f"Valid: {result.verification.valid}/{result.verification.sampled}")
```

### Step-by-Step Processing

```python
from src.services import (
    parse_all_chapters,
    embed_chunks,
    initialize_collection,
    insert_embeddings,
    verify_insertion,
    verify_embedding_quality,
    verify_rag_accuracy
)

# Step 1: Parse
chunks = parse_all_chapters("./chapters")
print(f"Parsed: {len(chunks)} sections")

# Step 2: Embed
embeddings = embed_chunks(chunks)
print(f"Embedded: {len(embeddings)} vectors")

# Step 3: Initialize
initialize_collection("robotics")

# Step 4: Insert
result = insert_embeddings(embeddings, "robotics")
print(f"Inserted: {result.inserted}/{result.total}")

# Step 5: Verify
quality = verify_embedding_quality(sample_size=100)
print(f"Quality: {quality.pass_rate}%")

rag = verify_rag_accuracy(num_queries=10)
print(f"RAG success: {rag['search_success_rate']}%")
```

### Error Handling

```python
from src.services import run_pipeline

try:
    result = run_pipeline("./chapters", verify=True)

    if result.status == "success":
        print("✓ Pipeline succeeded")
    elif result.status == "failed":
        print("✗ Pipeline failed:")
        for error in result.errors:
            print(f"  - {error['step']}: {error['error']}")

    if result.warnings:
        print("⚠ Warnings:")
        for warning in result.warnings:
            print(f"  - {warning}")

except Exception as e:
    print(f"Unexpected error: {e}")
```
