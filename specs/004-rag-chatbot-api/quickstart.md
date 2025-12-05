# RAG Chatbot API - Quick Start Guide

## Overview

This quickstart guides you through implementing the RAG chatbot API feature. The feature adds a `/api/v1/chat/ask` endpoint that answers user questions about robotics textbook content using semantic search and LLM generation.

## Prerequisites

- Python 3.11+
- FastAPI project setup (existing backend directory)
- Qdrant collection pre-indexed with robotics textbook embeddings
- Google Generative AI API key (Gemini access)
- Poetry for dependency management

## Architecture Overview

```
User Question
    ↓
[ChatRequest Validation]
    ↓
[Question Embedding] → OpenAI API
    ↓
[Vector Search] → Qdrant (top-5 similar sections)
    ↓
[Prompt Construction] (question + context)
    ↓
[LLM Generation] → Gemini API
    ↓
[Citation Extraction & Validation]
    ↓
[Response Serialization]
    ↓
[ChatResponse] (answer + sources + metadata)
```

## Implementation Steps

### Phase 1: Setup Dependencies

1. **Add google-generative-ai to pyproject.toml**:
   ```toml
   [tool.poetry.dependencies]
   google-generative-ai = "^0.3.0"
   ```

2. **Update .env.example**:
   ```env
   # Gemini API
   GEMINI_API_KEY=your-key-here
   GEMINI_MODEL=gemini-1.5-flash
   ```

3. **Install dependencies**:
   ```bash
   poetry install
   ```

### Phase 2: Data Models (models/chat.py)

Create Pydantic models for request/response validation:

```python
from pydantic import BaseModel, Field
from typing import List, Optional

class ChatRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000)
    filters: Optional[dict] = None

class Source(BaseModel):
    chapter_id: str
    chapter_title: str
    section_number: int
    section_title: str
    excerpt: str
    relevance_score: float

class ChatResponse(BaseModel):
    answer: str
    sources: List[Source]
    metadata: dict
```

### Phase 3: Gemini Service (services/gemini_service.py)

Async wrapper for Gemini LLM:

```python
import google.generativeai as genai
from typing import Optional
import logging

class GeminiService:
    def __init__(self, api_key: str, model: str = "gemini-1.5-flash"):
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(model)
        self.logger = logging.getLogger(__name__)

    async def generate_answer(self, context: str, question: str) -> str:
        """Generate answer using Gemini with context from vector search"""
        system_prompt = """You are a robotics education assistant.
        Answer questions based ONLY on the provided context.
        Always end your answer with a source citation in format:
        Source: Chapter {chapter_id}, Section {section_number} - {section_title}
        """

        prompt = f"""
        Context:
        {context}

        Question: {question}

        Requirements:
        - Answer based solely on provided context
        - Be concise (100-200 words)
        - If uncertain, start with "I'm not entirely certain, but..."
        """

        try:
            response = self.model.generate_content(
                [system_prompt, prompt],
                generation_config={"temperature": 0.7, "max_output_tokens": 500}
            )
            return response.text
        except Exception as e:
            self.logger.error(f"Gemini API error: {e}")
            raise
```

### Phase 4: RAG Service (services/rag_service.py)

Orchestrate the RAG pipeline:

```python
from services.embedding_service import embed_chunks
from services.qdrant_service import search_chunks
from services.gemini_service import GeminiService
from models.chat import ChatResponse, Source
import time

class RAGService:
    def __init__(self, gemini_service: GeminiService):
        self.gemini = gemini_service

    async def answer_question(self, question: str, filters: Optional[dict] = None) -> ChatResponse:
        start_time = time.time()

        # Step 1: Embed question
        embed_start = time.time()
        question_embedding = await embed_question(question)
        embed_time = time.time() - embed_start

        # Step 2: Search vector database
        search_start = time.time()
        search_results = await search_chunks(
            question_embedding,
            top_k=5,
            filters=filters
        )
        search_time = time.time() - search_start

        if not search_results:
            return ChatResponse(
                answer="No relevant content found in the textbook.",
                sources=[],
                metadata={"confidence_score": 0.0, "num_sources_retrieved": 0}
            )

        # Step 3: Construct context from chunks
        context = self._format_context(search_results)

        # Step 4: Generate answer
        gen_start = time.time()
        answer = await self.gemini.generate_answer(context, question)
        gen_time = time.time() - gen_start

        # Step 5: Extract and validate citations
        sources = self._extract_sources(search_results)

        # Step 6: Calculate confidence and metadata
        avg_relevance = sum(r['score'] for r in search_results) / len(search_results)
        confidence = self._calculate_confidence(avg_relevance)

        total_time = time.time() - start_time

        return ChatResponse(
            answer=answer,
            sources=sources,
            metadata={
                "question_embedding_time_ms": embed_time * 1000,
                "vector_search_time_ms": search_time * 1000,
                "llm_generation_time_ms": gen_time * 1000,
                "total_time_ms": total_time * 1000,
                "confidence_score": confidence,
                "avg_relevance_score": avg_relevance,
                "num_sources_retrieved": len(sources)
            }
        )

    def _format_context(self, search_results) -> str:
        """Format retrieved chunks as context for LLM"""
        context_parts = []
        for result in search_results:
            context_parts.append(
                f"Chapter {result['chapter_id']}, Section {result['section_number']}: {result['section_title']}\n"
                f"{result['content']}\n"
            )
        return "\n---\n".join(context_parts)

    def _extract_sources(self, search_results) -> List[Source]:
        """Convert search results to Source objects"""
        sources = []
        for result in search_results:
            sources.append(Source(
                chapter_id=result['chapter_id'],
                chapter_title=result['chapter_title'],
                section_number=result['section_number'],
                section_title=result['section_title'],
                excerpt=result['content'][:200],
                relevance_score=result['score']
            ))
        return sources

    def _calculate_confidence(self, avg_relevance: float) -> float:
        """Calculate confidence score from relevance"""
        if avg_relevance < 0.7:
            return 0.5
        elif avg_relevance < 0.85:
            return 0.7
        else:
            return 0.9
```

### Phase 5: Chat Endpoint (api/v1/routes/chat.py)

FastAPI endpoint handler:

```python
from fastapi import APIRouter, Depends, HTTPException, status
from models.chat import ChatRequest, ChatResponse
from services.rag_service import RAGService

router = APIRouter(prefix="/api/v1", tags=["chat"])

@router.post("/chat/ask", response_model=ChatResponse)
async def ask_question(request: ChatRequest, rag_service: RAGService = Depends()) -> ChatResponse:
    """
    Answer a question about robotics textbook content
    """
    try:
        response = await rag_service.answer_question(
            question=request.question,
            filters=request.filters
        )
        return response
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except TimeoutError:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Vector search service timed out. Please try again."
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="LLM service temporarily unavailable. Please try again."
        )
```

### Phase 6: Integration with FastAPI App (main.py)

Add endpoint to FastAPI application:

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api.v1.routes import chat

app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://yourdomain.github.io"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include chat routes
app.include_router(chat.router)

@app.get("/health")
async def health():
    return {"status": "ok"}
```

## Testing

### Unit Tests (tests/unit/test_rag_service.py)

```python
import pytest
from services.rag_service import RAGService
from services.gemini_service import GeminiService
from unittest.mock import AsyncMock, patch

@pytest.mark.asyncio
async def test_rag_service_generates_answer():
    """Test RAG pipeline returns answer with sources"""
    mock_gemini = AsyncMock(spec=GeminiService)
    mock_gemini.generate_answer.return_value = "Forward kinematics is..."

    rag_service = RAGService(mock_gemini)

    with patch('services.qdrant_service.search_chunks') as mock_search:
        mock_search.return_value = [
            {
                'chapter_id': 'ch03',
                'chapter_title': 'Kinematics',
                'section_number': 1,
                'section_title': 'Forward Kinematics',
                'content': 'Forward kinematics...',
                'score': 0.89
            }
        ]

        response = await rag_service.answer_question("What is forward kinematics?")

        assert response.answer is not None
        assert len(response.sources) == 1
        assert response.metadata['confidence_score'] > 0
```

### Integration Tests (tests/integration/test_rag_pipeline.py)

```python
import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_chat_endpoint_returns_answer():
    """Test /api/v1/chat/ask endpoint"""
    response = client.post(
        "/api/v1/chat/ask",
        json={"question": "What is forward kinematics?"}
    )

    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert "metadata" in data
    assert data['metadata']['total_time_ms'] < 3000  # <3 seconds SLA
```

## Running the API

```bash
# Start FastAPI server
poetry run uvicorn main:app --reload --port 8000

# Test the endpoint
curl -X POST http://localhost:8000/api/v1/chat/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is forward kinematics?"}'

# View OpenAPI docs
open http://localhost:8000/api/v1/docs
```

## Deployment Checklist

- [ ] All unit tests passing (80%+ coverage)
- [ ] Integration tests passing
- [ ] Environment variables configured (GEMINI_API_KEY, Qdrant credentials)
- [ ] Error handling tested (Gemini down, Qdrant down, invalid input)
- [ ] Latency validated (p95 < 3 seconds)
- [ ] Rate limiting configured (5 req/sec for Gemini)
- [ ] Logging configured (JSON format with request_id)
- [ ] Health check endpoint working
- [ ] OpenAPI documentation generated
- [ ] CORS configured for frontend origin

## Next Steps

After implementation:
1. Run `/sp.tasks` to generate detailed task breakdown
2. Implement features in priority order (P1 → P2 → P3)
3. Create code review checklist
4. Set up CI/CD pipeline for testing
5. Deploy to staging for end-to-end testing
