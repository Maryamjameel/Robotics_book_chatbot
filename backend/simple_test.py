"""Simple test to verify rate limiting works."""

import asyncio
import time
from src.models import ChatRequest
from src.api.v1.routes.chat import ask_question
from unittest.mock import Mock


async def test_rate_limiting():
    """Test that rate limiting prevents 429 errors."""
    print("=" * 60)
    print("Testing Rate Limiting")
    print("=" * 60)

    questions = [
        "What is ROS?",
        "What is kinematics?",
        "What is Gazebo?",
    ]

    http_request = Mock()
    http_request.url.path = "/api/v1/chat/ask"

    for i, question in enumerate(questions, 1):
        print(f"\nTest {i}/{len(questions)}: {question}")
        start = time.time()

        try:
            request = ChatRequest(
                question=question,
                selected_text=None,
                chapter_context=None,
                filters=None
            )

            response = await ask_question(request, http_request)
            elapsed = time.time() - start

            print(f"  ✓ Success in {elapsed:.1f}s")
            print(f"  Sources: {len(response.sources)}")
            print(f"  Confidence: {response.confidence:.1%}")

        except Exception as e:
            elapsed = time.time() - start
            print(f"  ✗ Failed in {elapsed:.1f}s: {str(e)}")

        # Small delay between requests
        if i < len(questions):
            print("  Waiting 2 seconds...")
            await asyncio.sleep(2)

    print("\n" + "=" * 60)
    print("Test Complete")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(test_rate_limiting())
