---
name: "vector-embedding"
description: "Generate and manage text embeddings for semantic search. Create embeddings from chapter content, insert into Qdrant, and build efficient RAG pipelines. Use when setting up or updating the vector database."
version: "1.0.0"
---

# Vector Embedding Skill

## When to Use This Skill

- Converting textbook chapters to vector embeddings
- Setting up semantic search for the RAG chatbot
- Inserting embeddings into Qdrant vector database
- Creating embedding pipelines for new content
- Updating embeddings when chapters are modified
- Optimizing embedding strategies for better search

## How This Skill Works

1. **Text Chunking**: Split chapters into semantic chunks (paragraphs, sections)
2. **Embedding Generation**: Convert text to vector embeddings using OpenAI/other models
3. **Metadata Extraction**: Extract chapter ID, title, section info
4. **Vector Storage**: Insert embeddings into Qdrant with metadata
5. **Index Optimization**: Configure Qdrant for efficient search

## Embedding Models

### OpenAI Embeddings (Recommended)
- **Model**: `text-embedding-3-small` or `text-embedding-ada-002`
- **Dimensions**: 1536 (ada-002), 512/1536/3072 (3-small)
- **Cost**: $0.02 per 1M tokens (3-small), $0.10 per 1M tokens (ada-002)
- **Use case**: High-quality semantic search

### Open Source Alternatives
- **sentence-transformers/all-MiniLM-L6-v2**: 384 dimensions, free
- **sentence-transformers/all-mpnet-base-v2**: 768 dimensions, free
- **instructor-xl**: 768 dimensions, instruction-based

## Text Chunking Strategies

### 1. Section-Based Chunking (Recommended for Textbooks)

```python
import re
from typing import List, Dict

def chunk_by_sections(chapter_content: str, chapter_id: int, chapter_title: str) -> List[Dict]:
    """Split chapter into sections based on markdown headers."""
    chunks = []

    # Split by ## headers (sections)
    sections = re.split(r'\n##\s+', chapter_content)

    for idx, section in enumerate(sections):
        if not section.strip():
            continue

        # Extract section title (first line)
        lines = section.split('\n', 1)
        section_title = lines[0].strip()
        section_content = lines[1] if len(lines) > 1 else ""

        chunks.append({
            "chapter_id": chapter_id,
            "chapter_title": chapter_title,
            "section_number": idx,
            "section_title": section_title,
            "content": section_content.strip(),
            "chunk_type": "section"
        })

    return chunks
```

### 2. Fixed-Size Chunking with Overlap

```python
def chunk_by_tokens(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """Split text into fixed-size chunks with overlap."""
    import tiktoken

    encoder = tiktoken.encoding_for_model("text-embedding-ada-002")
    tokens = encoder.encode(text)

    chunks = []
    for i in range(0, len(tokens), chunk_size - overlap):
        chunk_tokens = tokens[i:i + chunk_size]
        chunk_text = encoder.decode(chunk_tokens)
        chunks.append(chunk_text)

    return chunks
```

### 3. Semantic Chunking

```python
from langchain.text_splitter import RecursiveCharacterTextSplitter

def chunk_semantically(text: str) -> List[str]:
    """Split text based on semantic boundaries."""
    splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        separators=["\n## ", "\n### ", "\n\n", "\n", " ", ""]
    )

    chunks = splitter.split_text(text)
    return chunks
```

## Generating Embeddings

### Using OpenAI API

```python
import openai
from typing import List
import os

openai.api_key = os.getenv("OPENAI_API_KEY")

def generate_embeddings(texts: List[str], model: str = "text-embedding-3-small") -> List[List[float]]:
    """Generate embeddings for a list of texts."""
    response = openai.embeddings.create(
        input=texts,
        model=model
    )

    embeddings = [item.embedding for item in response.data]
    return embeddings

# Batch processing for efficiency
def generate_embeddings_batch(texts: List[str], batch_size: int = 100) -> List[List[float]]:
    """Generate embeddings in batches to handle large datasets."""
    all_embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        embeddings = generate_embeddings(batch)
        all_embeddings.extend(embeddings)
        print(f"Processed {i + len(batch)}/{len(texts)} texts")

    return all_embeddings
```

### Using Sentence Transformers (Open Source)

```python
from sentence_transformers import SentenceTransformer

def generate_embeddings_local(texts: List[str]) -> List[List[float]]:
    """Generate embeddings using local model."""
    model = SentenceTransformer('all-MiniLM-L6-v2')
    embeddings = model.encode(texts, show_progress_bar=True)
    return embeddings.tolist()
```

## Storing Embeddings in Qdrant

### Setup Qdrant Collection

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

def setup_qdrant_collection(
    collection_name: str = "chapter_embeddings",
    vector_size: int = 1536
):
    """Create Qdrant collection for chapter embeddings."""
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Check if collection exists
    collections = client.get_collections().collections
    if any(c.name == collection_name for c in collections):
        print(f"Collection '{collection_name}' already exists")
        return

    # Create collection
    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(
            size=vector_size,
            distance=models.Distance.COSINE  # Cosine similarity
        )
    )

    # Create payload indexes for filtering
    client.create_payload_index(
        collection_name=collection_name,
        field_name="chapter_id",
        field_schema=models.PayloadSchemaType.INTEGER
    )

    client.create_payload_index(
        collection_name=collection_name,
        field_name="difficulty_level",
        field_schema=models.PayloadSchemaType.KEYWORD
    )

    print(f"✓ Created collection: {collection_name}")
```

### Insert Embeddings

```python
from qdrant_client.http import models as rest
from typing import List, Dict
import uuid

def insert_chapter_embeddings(
    chunks: List[Dict],
    embeddings: List[List[float]],
    collection_name: str = "chapter_embeddings"
):
    """Insert chapter embeddings into Qdrant."""
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    points = []
    for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        point = rest.PointStruct(
            id=str(uuid.uuid4()),  # Generate unique ID
            vector=embedding,
            payload={
                "chapter_id": chunk["chapter_id"],
                "chapter_title": chunk["chapter_title"],
                "section_number": chunk.get("section_number", 0),
                "section_title": chunk.get("section_title", ""),
                "content": chunk["content"],
                "chunk_type": chunk.get("chunk_type", "section"),
                "keywords": chunk.get("keywords", []),
            }
        )
        points.append(point)

    # Batch insert
    client.upsert(
        collection_name=collection_name,
        points=points
    )

    print(f"✓ Inserted {len(points)} embeddings into '{collection_name}'")
```

## Complete Embedding Pipeline

### End-to-End Example

```python
import os
from pathlib import Path
from typing import List, Dict

def process_chapter(
    chapter_file: Path,
    chapter_id: int,
    chapter_title: str
) -> int:
    """Process a single chapter: chunk, embed, and store."""

    # 1. Read chapter content
    with open(chapter_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # 2. Chunk the content
    chunks = chunk_by_sections(content, chapter_id, chapter_title)
    print(f"Created {len(chunks)} chunks for '{chapter_title}'")

    # 3. Extract text for embedding
    texts = [chunk["content"] for chunk in chunks]

    # 4. Generate embeddings
    embeddings = generate_embeddings_batch(texts)

    # 5. Store in Qdrant
    insert_chapter_embeddings(chunks, embeddings)

    return len(chunks)

def process_all_chapters(chapters_dir: Path):
    """Process all chapters in the directory."""
    chapter_files = sorted(chapters_dir.glob("chapter-*.md"))

    total_chunks = 0
    for chapter_file in chapter_files:
        # Extract chapter number from filename
        chapter_num = int(chapter_file.stem.split('-')[1])
        chapter_title = extract_title_from_file(chapter_file)

        print(f"\nProcessing: {chapter_file.name}")
        chunks_count = process_chapter(chapter_file, chapter_num, chapter_title)
        total_chunks += chunks_count

    print(f"\n✓ Processed {len(chapter_files)} chapters ({total_chunks} total chunks)")

def extract_title_from_file(file_path: Path) -> str:
    """Extract chapter title from markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('# '):
                return line[2:].strip()
    return file_path.stem

# Usage
if __name__ == "__main__":
    # Setup collection
    setup_qdrant_collection()

    # Process all chapters
    chapters_dir = Path("docs/chapters")
    process_all_chapters(chapters_dir)
```

## Searching Embeddings

### Semantic Search

```python
def search_similar_content(
    query: str,
    top_k: int = 5,
    chapter_filter: List[int] = None
) -> List[Dict]:
    """Search for similar content using semantic search."""
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Generate query embedding
    query_embedding = generate_embeddings([query])[0]

    # Build filter
    filter_conditions = []
    if chapter_filter:
        filter_conditions.append(
            models.FieldCondition(
                key="chapter_id",
                match=models.MatchAny(any=chapter_filter)
            )
        )

    query_filter = models.Filter(must=filter_conditions) if filter_conditions else None

    # Search
    results = client.search(
        collection_name="chapter_embeddings",
        query_vector=query_embedding,
        query_filter=query_filter,
        limit=top_k,
        score_threshold=0.7  # Minimum similarity score
    )

    # Format results
    formatted_results = []
    for result in results:
        formatted_results.append({
            "score": result.score,
            "chapter_id": result.payload["chapter_id"],
            "chapter_title": result.payload["chapter_title"],
            "section_title": result.payload["section_title"],
            "content": result.payload["content"][:200] + "...",  # Preview
        })

    return formatted_results

# Example usage
results = search_similar_content("What is inverse kinematics?", top_k=5)
for idx, result in enumerate(results):
    print(f"\n{idx + 1}. Score: {result['score']:.2f}")
    print(f"   Chapter: {result['chapter_title']}")
    print(f"   Section: {result['section_title']}")
    print(f"   Content: {result['content']}")
```

### Hybrid Search (Vector + Keywords)

```python
def hybrid_search(
    query: str,
    keywords: List[str],
    top_k: int = 5
) -> List[Dict]:
    """Combine vector similarity with keyword filtering."""
    client = QdrantClient(url=os.getenv("QDRANT_URL"))

    query_embedding = generate_embeddings([query])[0]

    # Filter by keywords in payload
    filter_conditions = [
        models.FieldCondition(
            key="keywords",
            match=models.MatchAny(any=keywords)
        )
    ]

    results = client.search(
        collection_name="chapter_embeddings",
        query_vector=query_embedding,
        query_filter=models.Filter(must=filter_conditions),
        limit=top_k
    )

    return results
```

## Updating Embeddings

### Update Single Chapter

```python
def update_chapter_embeddings(
    chapter_id: int,
    new_content: str,
    chapter_title: str
):
    """Update embeddings when a chapter is modified."""
    client = QdrantClient(url=os.getenv("QDRANT_URL"))

    # 1. Delete old embeddings
    client.delete(
        collection_name="chapter_embeddings",
        points_selector=models.FilterSelector(
            filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="chapter_id",
                        match=models.MatchValue(value=chapter_id)
                    )
                ]
            )
        )
    )

    # 2. Generate new embeddings
    chunks = chunk_by_sections(new_content, chapter_id, chapter_title)
    texts = [chunk["content"] for chunk in chunks]
    embeddings = generate_embeddings_batch(texts)

    # 3. Insert new embeddings
    insert_chapter_embeddings(chunks, embeddings)

    print(f"✓ Updated embeddings for chapter {chapter_id}")
```

## Performance Optimization

### Batch Processing

```python
async def process_chapters_parallel(chapter_files: List[Path]):
    """Process multiple chapters in parallel."""
    import asyncio

    tasks = [
        asyncio.create_task(process_chapter_async(file))
        for file in chapter_files
    ]

    results = await asyncio.gather(*tasks)
    return results
```

### Caching Embeddings

```python
import pickle
from pathlib import Path

def cache_embeddings(embeddings: List[List[float]], cache_file: Path):
    """Save embeddings to disk cache."""
    with open(cache_file, 'wb') as f:
        pickle.dump(embeddings, f)

def load_cached_embeddings(cache_file: Path) -> List[List[float]]:
    """Load embeddings from disk cache."""
    with open(cache_file, 'rb') as f:
        return pickle.load(f)
```

## Best Practices

- ✓ **Chunk semantically**: Use section-based chunking for structured content
- ✓ **Include metadata**: Store chapter ID, title, section info for filtering
- ✓ **Batch processing**: Generate embeddings in batches for efficiency
- ✓ **Cache embeddings**: Save computed embeddings to avoid reprocessing
- ✓ **Monitor costs**: Track OpenAI API usage and optimize chunk sizes
- ✓ **Version control**: Track embedding model versions and configurations
- ✓ **Quality threshold**: Use score thresholds (e.g., 0.7) to filter low-quality matches

## Output Format

When creating an embedding pipeline, provide:

1. **Chunking Strategy**:
   - Method used (section-based, fixed-size, semantic)
   - Chunk size and overlap parameters
   - Metadata extraction logic

2. **Embedding Configuration**:
   - Model name and version
   - Vector dimensions
   - Batch size for processing

3. **Qdrant Setup**:
   - Collection configuration
   - Index definitions
   - Payload schema

4. **Processing Scripts**:
   - Code to process chapters
   - Error handling
   - Progress tracking

5. **Search Examples**:
   - How to query embeddings
   - Filter strategies
   - Result formatting
