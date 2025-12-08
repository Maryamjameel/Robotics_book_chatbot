"""Test the chatbot with a real ROS question."""

import asyncio
import json
from src.models import ChatRequest
from src.api.v1.routes.chat import ask_question
from unittest.mock import Mock


async def test_real_question():
    """Test with the actual question 'What is ROS?'"""
    print("=" * 60)
    print("Testing Chatbot with Real Question")
    print("=" * 60)

    # Create request
    request = ChatRequest(
        question="What is ROS?",
        selected_text=None,
        chapter_context=None,
        filters=None
    )

    # Mock HTTP request
    http_request = Mock()
    http_request.url.path = "/api/v1/chat/ask"

    print(f"\nQuestion: '{request.question}'")
    print("\nProcessing...")

    try:
        # Call the chat endpoint
        response = await ask_question(request, http_request)

        print("\n" + "=" * 60)
        print("RESPONSE")
        print("=" * 60)
        print(f"\nAnswer:\n{response.answer}\n")
        print(f"Confidence: {response.confidence:.1%}")
        print(f"\nSources ({len(response.sources)}):")

        for i, source in enumerate(response.sources, 1):
            print(f"\n{i}. {source.chapter_title}")
            print(f"   Chapter: {source.chapter_id}")
            print(f"   Section {source.section_number}: {source.section_title}")
            print(f"   Relevance: {source.relevance_score:.1%}")
            print(f"   Excerpt: {source.excerpt[:100]}...")

        print("\n" + "=" * 60)
        print("Metadata:")
        print(f"  Search latency: {response.metadata.search_latency_ms:.0f}ms")
        print(f"  Generation latency: {response.metadata.generation_latency_ms:.0f}ms")
        print(f"  Total latency: {response.metadata.total_latency_ms:.0f}ms")
        print("=" * 60)

        # Check for success
        if response.sources and len(response.sources) > 0:
            print("\n[SUCCESS] Chatbot is working correctly!")
            print(f"  - Found {len(response.sources)} relevant sources")
            print(f"  - Generated answer with {response.confidence:.1%} confidence")
            return 0
        else:
            print("\n[WARNING] No sources found, but chatbot is functional")
            return 1

    except Exception as e:
        print(f"\n[ERROR] Chatbot failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(test_real_question())
    exit(exit_code)
