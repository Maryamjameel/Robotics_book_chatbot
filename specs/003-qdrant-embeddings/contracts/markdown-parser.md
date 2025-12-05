# Service Contract: Markdown Parser

**Service**: `backend/src/services/markdown_parser.py`
**Purpose**: Extract chapters and sections from markdown files for embedding

## Functions

### `parse_chapter(filepath: str) -> List[ChapterChunk]`

Extract chapter metadata and sections from a markdown file.

**Input**:
```python
filepath: str  # Path to markdown file (e.g., "frontend/docs/chapters/ch01.md")
```

**Output**:
```python
List[ChapterChunk]  # List of section chunks extracted from the file
# Each chunk contains: chapter_id, chapter_title, section_number, section_title, content
```

**Behavior**:
1. Read markdown file from disk
2. Extract first `#` heading as chapter title
3. Extract chapter_id from filename (e.g., "ch01.md" → "ch01")
4. Split by `##` section headers
5. For each section: create ChapterChunk with metadata
6. Return list of chunks in order

**Error Handling**:
- File not found: Raise `FileNotFoundError(f"Chapter file not found: {filepath}")`
- Invalid markdown (no `#` heading): Raise `ValueError(f"No chapter title found in {filepath}")`
- Empty file: Raise `ValueError(f"Empty markdown file: {filepath}")`
- Encoding error: Raise `UnicodeDecodeError` with context

**Example**:

```python
chunks = parse_chapter("frontend/docs/chapters/ch03.md")
# Returns:
# [
#   ChapterChunk(
#     chapter_id="ch03",
#     chapter_title="Kinematics and Motion Planning",
#     section_number=0,
#     section_title="Overview",
#     content="Kinematics is the study of motion without considering forces..."
#   ),
#   ChapterChunk(
#     chapter_id="ch03",
#     chapter_title="Kinematics and Motion Planning",
#     section_number=1,
#     section_title="Forward Kinematics",
#     content="Forward kinematics solves for end-effector position..."
#   ),
#   ...
# ]
```

---

### `parse_all_chapters(directory: str) -> List[ChapterChunk]`

Extract all chapters from a directory of markdown files.

**Input**:
```python
directory: str  # Path to directory containing chapter markdown files
```

**Output**:
```python
List[ChapterChunk]  # All chunks from all chapters in order
```

**Behavior**:
1. List all `.md` files in directory
2. Sort by filename (ensures ch01, ch02, ch03... order)
3. Call `parse_chapter()` on each file
4. Aggregate results into single list
5. Validate no duplicate (chapter_id, section_number) pairs

**Error Handling**:
- Directory not found: Raise `FileNotFoundError(f"Chapter directory not found: {directory}")`
- No markdown files found: Raise `ValueError(f"No .md files in {directory}")`
- Duplicate chunks: Raise `ValueError(f"Duplicate chunk: {chapter_id}_{section_number}")`
- Individual file errors: Log error, skip file, continue (graceful degradation)

**Example**:

```python
all_chunks = parse_all_chapters("frontend/docs/chapters/")
# Returns list of 500+ chunks from all chapters
# Each chunk is a section ready for embedding
```

---

## Data Contracts

### ChapterChunk Model (Pydantic)

```python
from pydantic import BaseModel, Field, validator

class ChapterChunk(BaseModel):
    chapter_id: str = Field(..., min_length=1, regex="^ch[0-9]+$")
    chapter_title: str = Field(..., min_length=1, max_length=200)
    section_number: int = Field(..., ge=0)
    section_title: str = Field(..., min_length=1, max_length=200)
    content: str = Field(..., min_length=50, max_length=8000)
    metadata: Optional[Dict[str, Any]] = None

    @validator('content')
    def content_not_empty(cls, v):
        if not v.strip():
            raise ValueError('content must not be empty or whitespace-only')
        return v.strip()

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": "ch03",
                "chapter_title": "Kinematics and Motion Planning",
                "section_number": 2,
                "section_title": "Forward Kinematics",
                "content": "Forward kinematics is the problem of computing...",
                "metadata": {
                    "source_file": "frontend/docs/chapters/ch03.md",
                    "token_count": 287
                }
            }
        }
```

---

## Testing Contract

**Unit Tests** (`backend/tests/unit/test_markdown_parser.py`):

1. **test_parse_chapter_valid_file**: Parse well-formed chapter, verify chunk count and content
2. **test_parse_chapter_missing_title**: File with no `#` heading raises ValueError
3. **test_parse_chapter_empty_file**: Empty file raises ValueError
4. **test_parse_chapter_encoding_error**: Non-UTF8 file raises UnicodeDecodeError
5. **test_parse_chapter_sequential_sections**: Verify section_number is 0-indexed and sequential
6. **test_parse_all_chapters_valid_directory**: Parse directory with 5 test chapters, verify all chunks extracted
7. **test_parse_all_chapters_missing_directory**: Non-existent directory raises FileNotFoundError
8. **test_parse_all_chapters_duplicate_detection**: Raises error if duplicate (chapter_id, section_number) found
9. **test_parse_all_chapters_skips_failed_files**: One invalid file, others processed (graceful degradation)

**Integration Tests** (`backend/tests/integration/test_end_to_end_pipeline.py`):

- Test with sample 5-chapter markdown directory
- Verify chunk structure matches Qdrant payload schema
- Verify metadata is extracted correctly for citation

---

## Performance Expectations

- **Single file parsing**: < 100ms for typical chapter (20 sections)
- **Directory parsing**: < 1s for 100 chapters (~500 chunks)
- **Memory**: < 50MB for 500 chunks (typical chapter set)
- **Token counting**: Uses `tiktoken` library (< 10ms per chunk)

---

## Backwards Compatibility

This is a new service. No backwards compatibility concerns.

**Future Changes**:
- Support for `.rst` or other markdown variants (extend `parse_chapter()` interface with `format` parameter)
- Custom section delimiters (add optional `section_delimiter` parameter)
