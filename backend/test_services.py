"""Test script to debug service issues."""

import asyncio
import sys
from src.config import config, logger


async def test_embedding():
    """Test embedding service."""
    try:
        from src.services.embedding_service import embed_question
        print("[OK] Embedding service imported")

        embedding = await embed_question("What is robotics?")
        print(f"[OK] Embedding generated: {len(embedding)} dimensions")
        return True
    except Exception as e:
        print(f"[FAIL] Embedding service failed: {type(e).__name__}: {str(e)}")
        return False


async def test_qdrant():
    """Test Qdrant service."""
    try:
        from src.services.qdrant_service import search_chunks
        print("[OK] Qdrant service imported")

        # Create a dummy embedding
        dummy_embedding = [0.1] * 768
        result = search_chunks(
            question_embedding=dummy_embedding,
            top_k=2,
            relevance_threshold=0.3
        )
        print(f"[OK] Qdrant search works: {len(result.get('results', []))} results")
        return True
    except Exception as e:
        print(f"[FAIL] Qdrant service failed: {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


async def test_rag():
    """Test RAG service."""
    try:
        from src.services.rag_service import RAGService
        print("[OK] RAG service imported")

        rag = RAGService()
        # Test with dummy chunks
        chunks = [
            {
                "chapter_id": "ch1",
                "section_number": 1,
                "section_title": "Introduction",
                "excerpt": "Robotics is the study of robots.",
                "relevance_score": 0.9
            }
        ]
        response = await rag.answer_question(
            question="What is robotics?",
            retrieved_chunks=chunks,
            request_id="test-123"
        )
        print(f"[OK] RAG service works: answer length = {len(response.answer)}")
        return True
    except Exception as e:
        print(f"[FAIL] RAG service failed: {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    """Run all tests."""
    print("=" * 50)
    print("Testing RAG Pipeline Services")
    print("=" * 50)

    print("\n1. Testing Configuration...")
    try:
        config.validate()
        print("[OK] Config valid")
        print(f"  - Gemini API Key: {'present' if config.gemini_api_key else 'MISSING'}")
        print(f"  - Qdrant URL: {config.qdrant_url}")
        print(f"  - Collection: {config.collection_name}")
    except Exception as e:
        print(f"[FAIL] Config invalid: {str(e)}")
        sys.exit(1)

    print("\n2. Testing Embedding Service...")
    embedding_ok = await test_embedding()

    print("\n3. Testing Qdrant Service...")
    qdrant_ok = await test_qdrant()

    print("\n4. Testing RAG Service...")
    rag_ok = await test_rag()

    print("\n" + "=" * 50)
    print("Test Summary:")
    print(f"  Embedding: {'PASS' if embedding_ok else 'FAIL'}")
    print(f"  Qdrant:    {'PASS' if qdrant_ok else 'FAIL'}")
    print(f"  RAG:       {'PASS' if rag_ok else 'FAIL'}")
    print("=" * 50)

    if all([embedding_ok, qdrant_ok, rag_ok]):
        print("\n[OK] All services working! The issue may be elsewhere.")
        sys.exit(0)
    else:
        print("\n[FAIL] Some services failed. Check errors above.")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
