# Quickstart: Vector Embeddings Pipeline

**Purpose**: Get the embedding pipeline running in 15 minutes for local development

## Prerequisites

- Python 3.11+
- OpenAI API key (free tier account sufficient)
- Qdrant Cloud account (free tier sufficient)
- Poetry (Python dependency manager)

## Setup (5 minutes)

### 1. Install Dependencies

```bash
cd backend
poetry install

# Installs:
# - qdrant-client (v2.0+)
# - openai (v1.0+)
# - pydantic (v2)
# - python-dotenv
# - pytest (for testing)
```

### 2. Configure Environment Variables

Create `.env` file in `backend/` directory (never commit this):

```bash
# .env (example)
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxx
QDRANT_URL=https://xxx-xxx-xxx.qdrantcloud.io:6333
QDRANT_API_KEY=xxxxxxxxxxxxx
```

**Getting values**:
- **OPENAI_API_KEY**: Create free account at https://platform.openai.com/account/api-keys
- **QDRANT_URL** and **QDRANT_API_KEY**: Create free account at https://qdrant.io/ and get cluster credentials

### 3. Create Sample Chapters (for testing)

Create `frontend/docs/chapters/test_ch01.md`:

```markdown
# Introduction to Robotics

## Overview

Robotics is an interdisciplinary field that combines mechanical engineering, electrical engineering, computer science, and control theory.

## History

The word "robot" comes from the Czech word "robota" meaning "forced labor". The first industrial robot was installed by Unimation in 1961.

## Applications

Robots are used in manufacturing, healthcare, exploration, and entertainment industries.
```

Create `frontend/docs/chapters/test_ch02.md`:

```markdown
# Forward and Inverse Kinematics

## Forward Kinematics

Forward kinematics is the problem of computing the position and orientation of the end-effector given joint angles.

## Inverse Kinematics

Inverse kinematics solves the reverse problem: given desired end-effector position and orientation, compute required joint angles.

## Denavit-Hartenberg Convention

The DH convention provides a systematic way to assign coordinate frames to robot links.
```

## Running the Pipeline (10 minutes)

### Option 1: Run Step-by-Step (for learning)

```bash
cd backend

# Step 1: Parse markdown chapters
python -m scripts.chunk_chapters \
  --input-dir ../frontend/docs/chapters/ \
  --output chunks.json

# Expected output:
# Processed 2 chapters
# Total chunks: 6
# Saved to chunks.json

# Step 2: Generate embeddings
python -m scripts.generate_embeddings \
  --input chunks.json \
  --output embeddings.json

# Expected output:
# Processing batch 1/1 (6 chunks)
# Successfully embedded 6/6 chunks
# Saved to embeddings.json

# Step 3: Insert into Qdrant
python -m scripts.insert_qdrant \
  --input embeddings.json \
  --collection chapter_embeddings

# Expected output:
# Initialized collection chapter_embeddings
# Inserted 6/6 embeddings
# Verified 6 points in collection

# Step 4: Verify insertion
python -m scripts.verify_qdrant \
  --collection chapter_embeddings

# Expected output:
# Total points: 6
# Sampled 5 random points
# All checks passed ✓
```

### Option 2: Run End-to-End Pipeline (faster)

```bash
cd backend

python -m scripts.process_all \
  --input-dir ../frontend/docs/chapters/ \
  --collection chapter_embeddings

# Expected output:
# Starting vector embedding pipeline...
# [1/2] Processing test_ch01.md → 3 chunks
# [2/2] Processing test_ch02.md → 3 chunks
# Embedding 6 chunks (1 batch)...
# ✓ Successfully embedded 6/6 chunks
# Inserting 6 embeddings into Qdrant...
# ✓ Inserted 6/6 embeddings
# Verifying insertion...
# ✓ All verification checks passed
# Pipeline complete! Processed 2 chapters, 6 chunks
```

## Verify It Works (2 minutes)

### Test Semantic Search

Create `backend/test_search.py`:

```python
from qdrant_client import QdrantClient
import os

# Connect to Qdrant
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Generate query embedding (in production, use OpenAI API)
# For testing, use a fake 1536-dim vector
query_vector = [0.001] * 1536

# Search
results = client.search(
    collection_name="chapter_embeddings",
    query_vector=query_vector,
    limit=3,
    score_threshold=0.0
)

print(f"Found {len(results)} results:")
for hit in results:
    print(f"  - {hit.payload['title']} (score: {hit.score:.4f})")
```

Run test:

```bash
cd backend
python test_search.py

# Expected output:
# Found 3 results:
#   - Introduction to Robotics > Overview (score: 0.5234)
#   - Forward and Inverse Kinematics > Forward Kinematics (score: 0.4891)
#   - Introduction to Robotics > History (score: 0.4567)
```

## Next Steps

### For Development

1. **Study the data model**: Read `data-model.md` to understand entity relationships
2. **Review service contracts**: Check `contracts/` directory for API specifications
3. **Implement services**: Follow `backend/src/services/` structure
4. **Write tests**: Use `backend/tests/` as template for new test cases

### For Production

1. **Create production chapters**: Place full textbook markdown in `frontend/docs/chapters/`
2. **Configure Qdrant Cloud**: Upgrade to paid tier if chapters exceed 1GB
3. **Set up CI/CD**: Automate embedding pipeline to run on chapter updates
4. **Monitor**: Set up observability (logging, metrics) for production pipeline
5. **Scale**: Plan for re-indexing strategy as textbook grows

## Troubleshooting

### OpenAI API Errors

**Error**: `AuthenticationError: Invalid API key`
- **Solution**: Check OPENAI_API_KEY in `.env` is correct (starts with `sk-proj-`)

**Error**: `RateLimitError: Rate limit exceeded`
- **Solution**: Pipeline automatically retries with exponential backoff. Happens after 3 failed attempts.
- **Recovery**: Wait 30 seconds and re-run pipeline

### Qdrant Connection Errors

**Error**: `Connection refused to https://xxx.qdrantcloud.io`
- **Solution**: Check QDRANT_URL is correct and includes full hostname with `:6333` port

**Error**: `API error: Unauthorized`
- **Solution**: Check QDRANT_API_KEY is correct in `.env`

### Markdown Parsing Issues

**Error**: `ValueError: No chapter title found in ch01.md`
- **Solution**: Ensure markdown file starts with `# Chapter Title` on first line

**Error**: `ValueError: No .md files in frontend/docs/chapters/`
- **Solution**: Create sample chapters first using instructions above

## Performance Expectations

- **Single chapter (3 sections)**: ~5 seconds
- **100 chapters (500 sections)**: ~5-10 minutes (including rate limit backoff)
- **Query latency**: < 500ms per semantic search
- **Storage**: ~100-200 chapters fit in Qdrant Free Tier (1GB)

## Architecture Diagram

```
Frontend Markdown Files
  ↓ (chunk_chapters.py)
ChapterChunk list
  ↓ (generate_embeddings.py)
TextEmbedding list (with vectors)
  ↓ (insert_qdrant.py)
Qdrant "chapter_embeddings" collection
  ↓ (RAG queries)
Vector search results
```

## Monitoring

### Logs

Pipeline generates JSON logs:

```json
{"timestamp": "2025-12-05T14:32:10Z", "operation": "chunk", "file": "ch01.md", "chunks": 3, "status": "success"}
{"timestamp": "2025-12-05T14:32:12Z", "operation": "embed", "chunks": 32, "status": "success", "latency_ms": 1850}
```

View logs:

```bash
cd backend
python -m scripts.process_all --input-dir ../frontend/docs/chapters/ 2>&1 | tee pipeline.log
# Logs saved to pipeline.log
```

### Storage Usage

Check Qdrant storage:

```python
from qdrant_client import QdrantClient

client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
info = client.get_collection("chapter_embeddings")
print(f"Points: {info.points_count}")
print(f"Storage: {info.config.vector_size} dims × {info.points_count} points")
```

## Further Reading

- **Data Model**: `specs/003-qdrant-embeddings/data-model.md`
- **Service Contracts**: `specs/003-qdrant-embeddings/contracts/`
- **Research & Decisions**: `specs/003-qdrant-embeddings/research.md`
- **Implementation Plan**: `specs/003-qdrant-embeddings/plan.md`
- **OpenAI API**: https://platform.openai.com/docs/api-reference/embeddings
- **Qdrant Documentation**: https://qdrant.tech/documentation/

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review logs in `pipeline.log`
3. Verify `.env` configuration
4. Consult OpenAI and Qdrant documentation
5. Open GitHub issue with error logs
