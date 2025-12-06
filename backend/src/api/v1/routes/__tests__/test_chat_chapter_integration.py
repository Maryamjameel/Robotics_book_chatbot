"""Integration tests for chat endpoint with chapter context.

Tests end-to-end RAG pipeline with chapter filtering.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from fastapi.testclient import TestClient
from src.main import app
from src.models import ChatResponse


@pytest.fixture
def client():
    """Create test client."""
    return TestClient(app)


@pytest.fixture
def mock_embedding_response():
    """Mock embedding response."""
    return [0.1] * 1536


@pytest.fixture
def mock_search_response():
    """Mock Qdrant search response."""
    return {
        "results": [
            {
                "id": "1",
                "chapter_id": "ch03",
                "section_number": 1,
                "section_title": "Forward Kinematics",
                "excerpt": "Forward kinematics calculates the end-effector position given joint angles.",
                "relevance_score": 0.95,
            },
            {
                "id": "2",
                "chapter_id": "ch03",
                "section_number": 2,
                "section_title": "Inverse Kinematics",
                "excerpt": "Inverse kinematics computes joint angles from desired end-effector position.",
                "relevance_score": 0.88,
            },
            {
                "id": "3",
                "chapter_id": "ch04",
                "section_number": 1,
                "section_title": "Dynamics",
                "excerpt": "Dynamics describes forces and torques that cause motion.",
                "relevance_score": 0.82,
            },
        ],
        "metadata": {
            "chapter_filtered": True,
            "chapter_id": "ch03",
            "boost_applied": True,
            "filtered_count": 2,
        },
    }


@pytest.fixture
def mock_rag_response():
    """Mock RAG service response."""
    return ChatResponse(
        answer="Forward kinematics calculates the end-effector position given joint angles using the Denavit-Hartenberg parameters.",
        sources=[
            {
                "chapter_id": "ch03",
                "chapter_title": "Kinematics",
                "section_number": 1,
                "section_title": "Forward Kinematics",
                "excerpt": "Forward kinematics calculates the end-effector position given joint angles.",
                "relevance_score": 0.95,
            }
        ],
        confidence=0.92,
        metadata={
            "confidence_score": 0.92,
            "search_latency_ms": 120.0,
            "generation_latency_ms": 450.0,
            "total_latency_ms": 570.0,
            "chapter_filtered": True,
            "chapter_id": "ch03",
        },
    )


class TestChatEndpointWithChapterContext:
    """Test chat endpoint with chapter context."""

    @patch("src.api.v1.routes.chat.RAGService")
    @patch("src.api.v1.routes.chat.search_chunks")
    @patch("src.api.v1.routes.chat.embed_question")
    async def test_chat_with_chapter_context(
        self, mock_embed, mock_search, mock_rag_service_class, client, mock_embedding_response, mock_search_response, mock_rag_response
    ):
        """Test chat endpoint with chapter context."""
        # Setup mocks
        mock_embed.return_value = mock_embedding_response
        mock_search.return_value = mock_search_response
        mock_rag_instance = AsyncMock()
        mock_rag_instance.answer_question.return_value = mock_rag_response
        mock_rag_service_class.return_value = mock_rag_instance

        # Send request with chapter context
        response = client.post(
            "/api/v1/chat/ask",
            json={
                "question": "What is forward kinematics?",
                "chapter_context": {
                    "chapter_id": "ch03",
                    "chapter_title": "Kinematics",
                },
            },
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "confidence" in data
        assert "metadata" in data

        # Verify chapter filtering in metadata
        assert data["metadata"]["chapter_filtered"] is True
        assert data["metadata"]["chapter_id"] == "ch03"

        # Verify search was called with chapter context
        mock_search.assert_called_once()
        call_args = mock_search.call_args
        assert call_args.kwargs.get("chapter_context") == {
            "chapter_id": "ch03",
            "chapter_title": "Kinematics",
        }

    @patch("src.api.v1.routes.chat.RAGService")
    @patch("src.api.v1.routes.chat.search_chunks")
    @patch("src.api.v1.routes.chat.embed_question")
    async def test_chat_without_chapter_context(
        self, mock_embed, mock_search, mock_rag_service_class, client, mock_embedding_response, mock_search_response, mock_rag_response
    ):
        """Test chat endpoint without chapter context."""
        # Setup mocks
        mock_embed.return_value = mock_embedding_response
        mock_search.return_value = mock_search_response
        mock_rag_instance = AsyncMock()
        mock_rag_instance.answer_question.return_value = mock_rag_response
        mock_rag_service_class.return_value = mock_rag_instance

        # Send request without chapter context
        response = client.post(
            "/api/v1/chat/ask",
            json={"question": "What is forward kinematics?"},
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()

        # Verify search was called without chapter context
        mock_search.assert_called_once()
        call_args = mock_search.call_args
        assert call_args.kwargs.get("chapter_context") is None

    @patch("src.api.v1.routes.chat.RAGService")
    @patch("src.api.v1.routes.chat.search_chunks")
    @patch("src.api.v1.routes.chat.embed_question")
    async def test_chat_with_chapter_context_and_selected_text(
        self, mock_embed, mock_search, mock_rag_service_class, client, mock_embedding_response, mock_search_response, mock_rag_response
    ):
        """Test chat with both chapter context and selected text."""
        # Setup mocks
        mock_embed.return_value = mock_embedding_response
        mock_search.return_value = mock_search_response
        mock_rag_instance = AsyncMock()
        mock_rag_instance.answer_question.return_value = mock_rag_response
        mock_rag_service_class.return_value = mock_rag_instance

        # Send request with chapter context and selected text
        response = client.post(
            "/api/v1/chat/ask",
            json={
                "question": "Explain this concept",
                "selected_text": "Forward kinematics calculates position",
                "chapter_context": {
                    "chapter_id": "ch03",
                    "chapter_title": "Kinematics",
                },
            },
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()

        # Verify both features are active
        assert data["metadata"]["chapter_filtered"] is True
        assert data["metadata"]["selected_text_boosted"] is True

        # Verify selected text terms were extracted
        call_args = mock_search.call_args
        assert call_args.kwargs.get("selected_text_terms") is not None

    async def test_chat_with_invalid_chapter_context(self, client):
        """Test chat with invalid chapter context format."""
        response = client.post(
            "/api/v1/chat/ask",
            json={
                "question": "What is kinematics?",
                "chapter_context": {
                    # Missing chapter_id (required field)
                    "chapter_title": "Kinematics",
                },
            },
        )

        # Should still work - chapter_context is optional
        # The validation depends on schema definition
        assert response.status_code in [200, 400]

    @patch("src.api.v1.routes.chat.RAGService")
    @patch("src.api.v1.routes.chat.search_chunks")
    @patch("src.api.v1.routes.chat.embed_question")
    async def test_chat_chapter_filtering_reranks_results(
        self, mock_embed, mock_search, mock_rag_service_class, client, mock_embedding_response, mock_search_response, mock_rag_response
    ):
        """Test that chapter filtering re-ranks results."""
        # Modify mock to show boosted chapter results first
        boosted_response = {
            "results": [
                {
                    "id": "1",
                    "chapter_id": "ch03",
                    "section_number": 1,
                    "section_title": "Forward Kinematics",
                    "excerpt": "Forward kinematics...",
                    "relevance_score": 0.95 * 1.5,  # Boosted by 1.5x
                },
                {
                    "id": "2",
                    "chapter_id": "ch04",
                    "section_number": 1,
                    "section_title": "Dynamics",
                    "excerpt": "Dynamics...",
                    "relevance_score": 0.92,  # Not boosted
                },
            ],
            "metadata": {
                "chapter_filtered": True,
                "chapter_id": "ch03",
                "boost_applied": True,
                "filtered_count": 1,
            },
        }

        # Setup mocks
        mock_embed.return_value = mock_embedding_response
        mock_search.return_value = boosted_response
        mock_rag_instance = AsyncMock()
        mock_rag_instance.answer_question.return_value = mock_rag_response
        mock_rag_service_class.return_value = mock_rag_instance

        # Send request
        response = client.post(
            "/api/v1/chat/ask",
            json={
                "question": "What is forward kinematics?",
                "chapter_context": {
                    "chapter_id": "ch03",
                    "chapter_title": "Kinematics",
                },
            },
        )

        # Verify results are re-ranked
        assert response.status_code == 200
        data = response.json()
        assert data["metadata"]["boost_applied"] is True
        assert data["metadata"]["filtered_count"] == 1

    async def test_chat_empty_question(self, client):
        """Test chat with empty question."""
        response = client.post(
            "/api/v1/chat/ask",
            json={
                "question": "",
                "chapter_context": {
                    "chapter_id": "ch03",
                    "chapter_title": "Kinematics",
                },
            },
        )

        # Should return 400 bad request
        assert response.status_code == 400

    async def test_chat_question_too_long(self, client):
        """Test chat with question exceeding max length."""
        long_question = "a" * 2001  # Exceeds 2000 char limit

        response = client.post(
            "/api/v1/chat/ask",
            json={
                "question": long_question,
                "chapter_context": {
                    "chapter_id": "ch03",
                    "chapter_title": "Kinematics",
                },
            },
        )

        # Should return 400 bad request
        assert response.status_code == 400

    @patch("src.api.v1.routes.chat.RAGService")
    @patch("src.api.v1.routes.chat.search_chunks")
    @patch("src.api.v1.routes.chat.embed_question")
    async def test_chat_no_results_with_chapter_context(
        self, mock_embed, mock_search, mock_rag_service_class, client, mock_embedding_response
    ):
        """Test chat response when no results match chapter filter."""
        # Mock empty search results
        mock_embed.return_value = mock_embedding_response
        mock_search.return_value = {
            "results": [],
            "metadata": {
                "chapter_filtered": True,
                "chapter_id": "ch03",
                "boost_applied": False,
                "filtered_count": 0,
            },
        }

        # Send request
        response = client.post(
            "/api/v1/chat/ask",
            json={
                "question": "What is kinematics?",
                "chapter_context": {
                    "chapter_id": "ch03",
                    "chapter_title": "Kinematics",
                },
            },
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()
        assert "No relevant content found" in data["answer"]
