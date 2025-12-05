# Service Contract: Qdrant Service

**Service**: `backend/src/services/qdrant_service.py`
**Purpose**: Initialize Qdrant collection and insert vector embeddings

## Functions

### `initialize_collection(collection_name: str = "chapter_embeddings") -> bool`

Create Qdrant collection for chapter embeddings if it doesn't exist.

**Input**:
```python
collection_name: str = "chapter_embeddings"  # Name of collection to create
```

**Output**:
```python
bool  # True if created or already exists, False if error
```

**Behavior**:
1. Connect to Qdrant instance (using QDRANT_URL and QDRANT_API_KEY from environment)
2. Check if collection exists
3. If exists: log and return True
4. If not exists:
   a. Create collection with VectorParams:
      - size: 1536 (from embedding model)
      - distance: COSINE
   b. Create indexes on payloads: chapter_id, section_number, title
   c. Log collection created
5. Return True

**Qdrant Configuration**:

```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

client.create_collection(
    collection_name="chapter_embeddings",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
    payload_indexing_options={
        "indexed_fields": ["chapter_id", "section_number", "title"]
    }
)
```

**Error Handling**:
- Connection error: Raise `QdrantConnectionError(f"Cannot connect to Qdrant: {url}")`
- API error: Raise `QdrantAPIError(f"Qdrant API error: {error}")`
- Other errors: Log and return False

**Example**:

```python
from backend.src.services.qdrant_service import initialize_collection

success = initialize_collection()
if success:
    print("Qdrant collection ready")
else:
    print("Failed to initialize collection")
```

---

### `insert_embeddings(embeddings: List[TextEmbedding], collection_name: str = "chapter_embeddings") -> InsertionResult`

Insert embedded chunks into Qdrant collection.

**Input**:
```python
embeddings: List[TextEmbedding]  # Embeddings to insert (from embedding_service)
collection_name: str = "chapter_embeddings"  # Target collection
```

**Output**:
```python
InsertionResult  # {
                  #   "total": 32,
                  #   "inserted": 32,
                  #   "failed": 0,
                  #   "errors": []
                  # }
```

**Behavior**:
1. Connect to Qdrant
2. Create PointStruct objects from TextEmbedding list:
   ```python
   points = [
       PointStruct(
           id=hash(embedding.chunk_id),
           vector=embedding.vector,
           payload={
               "chapter_id": embedding.metadata["chapter_id"],
               "section_number": embedding.metadata["section_number"],
               "title": embedding.metadata["title"],
               "content": embedding.metadata["content"]
           }
       )
       for embedding in embeddings
   ]
   ```
3. Upsert points to collection (upsert = insert or update if exists)
4. Log insertion statistics
5. Return InsertionResult

**Upsert vs Insert**:
- Use upsert: allows re-processing chapters without duplicate errors
- ID strategy: deterministic hash of chunk_id ensures same vector always has same ID

**Error Handling** (per batch):
- Transient errors (rate limit, timeout): Retry once
- Duplicate ID: Update existing point (upsert behavior)
- Invalid payload: Log error, skip point, continue (graceful degradation)
- Collection not found: Raise `CollectionNotFoundError`

**Example**:

```python
from backend.src.services.qdrant_service import insert_embeddings

result = insert_embeddings(embeddings)
print(f"Inserted {result['inserted']}/{result['total']} embeddings")
if result['failed'] > 0:
    print(f"Failed: {result['errors']}")
```

---

### `verify_insertion(sample_size: int = 10, collection_name: str = "chapter_embeddings") -> VerificationResult`

Verify that embeddings are correctly stored and queryable in Qdrant.

**Input**:
```python
sample_size: int = 10  # Number of random points to verify
collection_name: str = "chapter_embeddings"
```

**Output**:
```python
VerificationResult  # {
                     #   "total_points": 520,
                     #   "sampled": 10,
                     #   "valid": 10,
                     #   "invalid": 0,
                     #   "checks": {
                     #     "dimensions": true,
                     #     "metadata": true,
                     #     "payload_indexing": true,
                     #     "query_performance": true
                     #   }
                     # }
```

**Behavior**:
1. Get collection info (total point count)
2. Retrieve `sample_size` random points
3. For each point:
   a. Validate vector dimensions (exactly 1536)
   b. Validate metadata fields present (chapter_id, section_number, title, content)
   c. Validate metadata types (chapter_id string, section_number int, etc.)
4. Test payload filtering: query with chapter_id filter, verify fast response
5. Test semantic search: create random query vector, search, verify results ranked by similarity
6. Return VerificationResult

**Checks Performed**:
- **dimensions**: All vectors exactly 1536-dimensional
- **metadata**: All required payload fields present
- **payload_indexing**: Queries using indexed fields complete in < 100ms
- **query_performance**: Full collection search (no filter) completes in < 1s

**Error Handling**:
- No points found: Raise `NoPointsError("Collection is empty")`
- Invalid vectors: Log details, mark as invalid
- Query timeout: Log warning, mark as failed

**Example**:

```python
from backend.src.services.qdrant_service import verify_insertion

result = verify_insertion(sample_size=20)
if result['invalid'] == 0:
    print("✓ All sampled embeddings verified successfully")
else:
    print(f"✗ {result['invalid']}/{result['sampled']} sampled embeddings invalid")
    print(f"Issues: {result}")
```

---

## Data Contracts

### InsertionResult Model (Pydantic)

```python
from pydantic import BaseModel
from typing import List, Dict, Any

class InsertionResult(BaseModel):
    total: int  # Total embeddings attempted
    inserted: int  # Successfully inserted
    failed: int  # Failed to insert
    errors: List[Dict[str, Any]] = []  # Error details

    class Config:
        json_schema_extra = {
            "example": {
                "total": 520,
                "inserted": 520,
                "failed": 0,
                "errors": []
            }
        }
```

### VerificationResult Model (Pydantic)

```python
from pydantic import BaseModel
from typing import Dict, Any

class VerificationResult(BaseModel):
    total_points: int  # Total points in collection
    sampled: int  # Number sampled
    valid: int  # Passed all checks
    invalid: int  # Failed checks
    checks: Dict[str, bool]  # Status of each check

    class Config:
        json_schema_extra = {
            "example": {
                "total_points": 520,
                "sampled": 10,
                "valid": 10,
                "invalid": 0,
                "checks": {
                    "dimensions": True,
                    "metadata": True,
                    "payload_indexing": True,
                    "query_performance": True
                }
            }
        }
```

---

## Configuration Contract

**Environment Variables** (required):

```bash
QDRANT_URL=https://xxx-xxx-xxx.qdrantcloud.io  # Qdrant Cloud endpoint
QDRANT_API_KEY=xxx...  # Qdrant API key (stored in .env, never committed)
```

**Configuration Defaults**:

```python
DEFAULT_COLLECTION_NAME = "chapter_embeddings"
VECTOR_DIMENSION = 1536
DISTANCE_METRIC = "cosine"
INDEXED_FIELDS = ["chapter_id", "section_number", "title"]
```

---

## Testing Contract

**Unit Tests** (`backend/tests/unit/test_qdrant_service.py`):

1. **test_initialize_collection_creates_new**: Mock Qdrant client, verify collection created with correct schema
2. **test_initialize_collection_existing**: Collection already exists, verify returns True
3. **test_initialize_collection_connection_error**: Connection fails, verify error raised
4. **test_insert_embeddings_success**: Mock 32 embeddings, verify all inserted
5. **test_insert_embeddings_partial_failure**: 1 invalid payload, verify 31 inserted, 1 failed logged
6. **test_insert_embeddings_upsert**: Insert same chunk twice, verify second overwrites (no duplicates)
7. **test_verify_insertion_valid**: 10 sample points, all pass checks
8. **test_verify_insertion_invalid_dimensions**: Mock vector with wrong dimensions, verify detected
9. **test_verify_insertion_missing_metadata**: Mock point missing required field, verify detected
10. **test_verify_insertion_query_performance**: Verify indexed field queries complete fast

**Integration Tests** (`backend/tests/integration/test_end_to_end_pipeline.py`):

- Use test Qdrant instance (local Docker container or Qdrant Cloud test environment)
- Initialize collection
- Insert 50 sample embeddings
- Verify insertion with all checks
- Test semantic search: query with known similar vector, verify results

---

## Performance Expectations

- **Collection initialization**: < 1s (create collection + indexes)
- **Batch insertion (32 points)**: ~200ms
- **1000 points (32 batches)**: ~6s total insertion
- **Verification (10 samples)**: < 1s
- **Query performance** (no filter): < 500ms for full collection
- **Query performance** (with filter): < 100ms (indexed fields)

---

## Qdrant Quota & Limits

**Free Tier Limits**:
- Storage: 1GB
- Vectors: ~100-200k at 1536 dimensions (~200 textbook chapters)
- Requests per second: Unlimited (but Qdrant Cloud may throttle if abused)
- Collections: Multiple (we only use 1)

**Scaling Strategy**:
- Monitor storage usage periodically
- Archive old chapters when storage exceeds 80%
- Plan for paid tier if expansion beyond textbook needed

---

## Backwards Compatibility

This is a new service. No backwards compatibility concerns.

**Future Extensibility**:
- Support multiple collections (separate by domain/subject)
- Implement collection versioning (re-index all chapters when schema changes)
- Add compression for large-scale deployments
