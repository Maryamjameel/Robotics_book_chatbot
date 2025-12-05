# Robotics Book Embeddings Pipeline

A production-ready Python service for generating, storing, and verifying vector embeddings of robotics textbook chapters using OpenAI and Qdrant.

## Overview

This pipeline implements a complete RAG (Retrieval-Augmented Generation) system for robotics educational content:

1. **Parse**: Extract sections from markdown chapter files
2. **Embed**: Generate 1536-dimensional embeddings using OpenAI's text-embedding-3-small model
3. **Store**: Insert embeddings into Qdrant vector database with metadata
4. **Verify**: Quality assurance checks on embeddings and RAG accuracy

## Architecture

```
Markdown Files
    ↓
[Parse Service] → ChapterChunk objects
    ↓
[Embedding Service] → TextEmbedding (1536-dim vectors)
    ↓
[Qdrant Service] → Vector storage with COSINE distance
    ↓
[Verification Service] → Quality metrics & RAG accuracy
```

## Quick Start

### Prerequisites

- Python 3.11+
- Poetry (for dependency management)
- OpenAI API key
- Qdrant instance (Cloud or local)

### Installation

```bash
# Install dependencies
poetry install

# Create environment file
cp .env.example .env

# Edit .env with your credentials
nano .env
```

### Configuration

Edit `.env` with your settings:

```env
# OpenAI
OPENAI_API_KEY=sk-...
EMBEDDING_MODEL=text-embedding-3-small

# Qdrant
QDRANT_URL=https://your-qdrant-instance.com
QDRANT_API_KEY=your-api-key
COLLECTION_NAME=robotics_chapters

# Processing
EMBEDDING_BATCH_SIZE=32
MAX_RETRIES=3
INITIAL_BACKOFF_SECONDS=1

# Text chunking
MIN_CHUNK_TOKENS=50
MAX_CHUNK_TOKENS=512

# Logging
LOG_LEVEL=INFO
LOG_FILE=pipeline.log
LOG_TO_CONSOLE=true
```

## Usage

### Complete Pipeline (Recommended)

Run the entire pipeline in one command:

```bash
python scripts/process_all.py /path/to/chapters robotics_chapters --verify
```

This will:
1. Parse all markdown files in `/path/to/chapters`
2. Generate embeddings for each section
3. Initialize Qdrant collection
4. Insert embeddings
5. Verify insertion quality

### Individual Scripts

#### 1. Parse Chapters

Extract sections from markdown files:

```bash
python scripts/chunk_chapters.py /path/to/chapters chapters.json
```

Output: `chapters.json` containing parsed ChapterChunk objects

#### 2. Generate Embeddings

Create embeddings from chapters:

```bash
python scripts/generate_embeddings.py chapters.json embeddings.json
```

Output: `embeddings.json` with 1536-dimensional vectors

#### 3. Insert into Qdrant

Store embeddings in vector database:

```bash
python scripts/insert_qdrant.py embeddings.json robotics_chapters --verify
```

Options:
- `--verify`: Run quality verification after insertion

### Using Python API

```python
from src.services import run_pipeline

# Run complete pipeline
result = run_pipeline(
    chapters_dir="./data/chapters",
    collection_name="robotics_chapters",
    verify=True
)

print(f"Status: {result.status}")
print(f"Parsed: {result.chapters_parsed} chapters")
print(f"Embedded: {result.embeddings_generated} sections")
print(f"Inserted: {result.embeddings_inserted} embeddings")
```

## API Reference

### Services

#### Markdown Parser

```python
from src.services import parse_chapter, parse_all_chapters

# Parse single chapter
chunks = parse_chapter("path/to/chapter.md", "ch01")

# Parse all chapters in directory
chunks = parse_all_chapters("path/to/chapters/")
```

#### Embedding Service

```python
from src.services import embed_chunks

# Generate embeddings
embeddings = embed_chunks(chunks)
```

#### Qdrant Service

```python
from src.services import initialize_collection, insert_embeddings, verify_insertion

# Initialize collection
initialize_collection("robotics_chapters")

# Insert embeddings
result = insert_embeddings(embeddings, "robotics_chapters")

# Verify quality
verification = verify_insertion("robotics_chapters", sample_size=50)
```

#### Verification Service

```python
from src.services import verify_embedding_quality, verify_rag_accuracy

# Check embedding quality metrics
quality = verify_embedding_quality(sample_size=100)

# Check RAG search accuracy
rag_metrics = verify_rag_accuracy(num_queries=10, top_k=5)
```

## Data Models

### ChapterChunk

```python
{
    "chapter_id": "ch01",
    "chapter_title": "Introduction to Robotics",
    "section_number": 1,
    "section_title": "Overview",
    "content": "Robotics is an interdisciplinary field...",
    "metadata": {
        "source_file": "ch01.md",
        "chapter_id": "ch01",
        "section_number": 1,
        "section_title": "Overview"
    }
}
```

### TextEmbedding

```python
{
    "chunk_id": "ch01_sec01",
    "vector": [0.001, 0.002, -0.003, ...],  # 1536 dimensions
    "metadata": {
        "chapter_id": "ch01",
        "section_title": "Overview"
    }
}
```

### Qdrant Point

Points in Qdrant include:
- **Vector**: 1536-dimensional embedding (COSINE distance)
- **Payload**: Metadata (chapter_id, section_title, content, source_file)
- **ID**: Hash-based unique identifier

## Testing

Run all tests:

```bash
pytest
```

Run specific test suites:

```bash
# Unit tests only
pytest -m unit

# Integration tests
pytest -m integration

# Slow tests
pytest -m slow

# With coverage
pytest --cov=src
```

### Test Files

- `tests/test_markdown_parser.py`: Parsing functionality
- `tests/test_embedding_service.py`: Embedding generation
- `tests/test_qdrant_service.py`: Vector storage
- `tests/test_verification_service.py`: Quality assurance
- `tests/test_orchestrator.py`: Pipeline coordination
- `tests/test_integration.py`: End-to-end workflows

## Markdown Format

The parser expects markdown files with this structure:

```markdown
# Chapter Title (required - single H1)

## Section 1 Title
Section content here...

## Section 2 Title
Section content here...
```

**Requirements**:
- Exactly one H1 header (`#`) per file (chapter title)
- One or more H2 headers (`##`) for sections
- Section headers become searchable metadata
- Content between headers becomes embedding text

## Logging

Logs are written in JSON format for easy parsing:

```json
{
    "timestamp": "2024-12-05T10:30:45.123456",
    "level": "INFO",
    "logger": "embedding_pipeline",
    "message": "Embeddings generated successfully",
    "operation": "embed_batch",
    "status": "success",
    "count": 50
}
```

Log files:
- **File**: `logs/pipeline.log` (rotating, 10MB per file)
- **Console**: Optional (controlled by `LOG_TO_CONSOLE`)

## Performance

### Batch Processing
- Batch size: 32 embeddings per API call (configurable)
- Exponential backoff on failures: 1s, 2s, 4s (max 3 retries)
- Graceful degradation: Failed items logged but processing continues

### Qdrant Storage
- Vector dimensions: 1536 (fixed for text-embedding-3-small)
- Distance metric: COSINE
- Index type: Auto-configured by Qdrant
- Upsert semantics: Idempotent updates/inserts

## Troubleshooting

### OpenAI API Errors
- Check API key in `.env`
- Verify rate limits haven't been exceeded
- Monitor logs for retry attempts and backoff timing

### Qdrant Connection Issues
- Verify URL and API key in `.env`
- Ensure Qdrant instance is running
- Check network connectivity to Qdrant URL

### Low Verification Scores
- Check vector magnitudes (should be ~1.0 for normalized embeddings)
- Verify payload completeness (chapter_id, section_title required)
- Sample more points for verification (`--sample-size` parameter)

## Configuration Reference

| Variable | Default | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | Required | OpenAI API authentication token |
| `EMBEDDING_MODEL` | text-embedding-3-small | Model for generating embeddings |
| `QDRANT_URL` | http://localhost:6333 | Qdrant instance URL |
| `QDRANT_API_KEY` | None | Qdrant authentication (optional for local) |
| `COLLECTION_NAME` | robotics_chapters | Default Qdrant collection |
| `EMBEDDING_BATCH_SIZE` | 32 | Texts per API batch |
| `MAX_RETRIES` | 3 | Retry attempts on failure |
| `INITIAL_BACKOFF_SECONDS` | 1 | Initial backoff duration |
| `MIN_CHUNK_TOKENS` | 50 | Minimum chunk size |
| `MAX_CHUNK_TOKENS` | 512 | Maximum chunk size |
| `LOG_LEVEL` | INFO | Logging verbosity |
| `LOG_FILE` | pipeline.log | Log file name |
| `LOG_TO_CONSOLE` | true | Whether to print to stdout |

## Development

### Project Structure

```
backend/
├── src/
│   ├── config.py              # Configuration & logging setup
│   ├── models/                # Pydantic data models
│   │   ├── chunk.py           # ChapterChunk, TextEmbedding, etc.
│   │   └── __init__.py
│   └── services/              # Business logic
│       ├── markdown_parser.py # Markdown parsing
│       ├── embedding_service.py # OpenAI embeddings
│       ├── qdrant_service.py  # Qdrant operations
│       ├── orchestrator.py    # Pipeline coordination
│       ├── verification_service.py # Quality checks
│       └── __init__.py
├── scripts/
│   ├── chunk_chapters.py      # CLI: Parse chapters
│   ├── generate_embeddings.py # CLI: Generate embeddings
│   ├── insert_qdrant.py       # CLI: Insert to Qdrant
│   └── process_all.py         # CLI: Full pipeline
├── tests/
│   ├── conftest.py            # Pytest fixtures & mocks
│   ├── test_markdown_parser.py
│   ├── test_embedding_service.py
│   ├── test_qdrant_service.py
│   ├── test_verification_service.py
│   ├── test_orchestrator.py
│   └── test_integration.py
├── pyproject.toml             # Dependencies & config
├── pytest.ini                 # Pytest configuration
├── .env.example               # Environment template
└── README.md                  # This file
```

### Adding New Features

1. **Create service**: Add function to appropriate service module
2. **Add models**: Update `src/models/chunk.py` if needed
3. **Write tests**: Unit tests required before code review
4. **Update exports**: Add to `src/services/__init__.py`
5. **Document**: Update this README

## Contributing

All pull requests must:
- ✅ Pass all unit tests (`pytest -m unit`)
- ✅ Pass integration tests (`pytest -m integration`)
- ✅ Maintain >80% code coverage
- ✅ Follow existing code style
- ✅ Update relevant documentation

## License

MIT

## Support

For issues, questions, or suggestions:
1. Check existing documentation
2. Review test files for usage examples
3. Examine logs (JSON format in `logs/pipeline.log`)
4. Open an issue with reproduction steps
