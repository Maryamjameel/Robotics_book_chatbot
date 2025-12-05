"""RAG service orchestrating vector search and LLM generation."""

import asyncio
import re
import time
from typing import List, Optional

from src.config import logger
from src.models import ChatResponse, RAGMetadata, Source
from src.services.gemini_service import GeminiService


class RAGService:
    """Service orchestrating RAG pipeline: search -> generate -> validate."""

    def __init__(self):
        """Initialize RAG service with dependencies."""
        self.gemini_service = GeminiService()
        # Rate limiting: 5 requests per second (Gemini quota)
        self.gemini_semaphore = asyncio.Semaphore(5)
        logger.info(
            "RAGService initialized",
            extra={"operation": "rag_init", "status": "success"},
        )

    def _extract_sources(
        self, retrieved_chunks: List[dict]
    ) -> List[Source]:
        """Convert search results to Source objects.

        Args:
            retrieved_chunks: List of dictionaries from Qdrant search

        Returns:
            List of Source objects with metadata
        """
        sources = []
        for chunk in retrieved_chunks:
            payload = chunk.get("payload", {})
            source = Source(
                chapter_id=payload.get("chapter_id", "unknown"),
                chapter_title=payload.get("chapter_title", "Unknown"),
                section_number=payload.get("section_number", 0),
                section_title=payload.get("section_title", "Unknown"),
                excerpt=payload.get("text", "")[:500],  # Truncate excerpt to 500 chars
                relevance_score=chunk.get("score", 0.0),
            )
            sources.append(source)
        return sources

    def _format_context(self, retrieved_chunks: List[dict]) -> str:
        """Format retrieved chunks for LLM context.

        Args:
            retrieved_chunks: List of dictionaries from Qdrant search

        Returns:
            Formatted context string for LLM input
        """
        context_parts = []
        for i, chunk in enumerate(retrieved_chunks, 1):
            payload = chunk.get("payload", {})
            chapter_id = payload.get("chapter_id", "unknown")
            section_num = payload.get("section_number", 0)
            section_title = payload.get("section_title", "Unknown")
            text = payload.get("text", "")
            score = chunk.get("score", 0.0)

            context_parts.append(
                f"[{i}] Chapter {chapter_id}, Section {section_num} - {section_title} (relevance: {score:.2f})\n{text}"
            )

        return "\n\n".join(context_parts)

    def _calculate_confidence(
        self, average_relevance: float, citations_valid: bool
    ) -> float:
        """Calculate confidence score based on relevance and citation validity.

        Args:
            average_relevance: Average relevance score from search (0-1)
            citations_valid: Whether citations are grounded in chunks

        Returns:
            Confidence score (0-1)
        """
        # Base confidence from relevance
        if average_relevance < 0.7:
            base_confidence = 0.5
        elif average_relevance < 0.85:
            base_confidence = 0.7
        else:
            base_confidence = 0.9

        # Adjust for citation validity
        if citations_valid:
            confidence = min(1.0, base_confidence + 0.1)
        else:
            confidence = max(0.0, base_confidence - 0.2)

        return round(confidence, 2)

    def _validate_citations(
        self, answer: str, retrieved_chunks: List[dict]
    ) -> bool:
        """Validate that answer citations match retrieved chunks.

        Args:
            answer: Generated answer text
            retrieved_chunks: Retrieved chunks from search

        Returns:
            True if citations are grounded, False if hallucinated
        """
        # Extract citations from answer (format: "Chapter X, Section Y")
        citation_pattern = r"Chapter\s+([a-zA-Z0-9]+),?\s+Section\s+(\d+)"
        citations = re.findall(citation_pattern, answer)

        if not citations:
            # No citations found - mark as uncertain
            logger.warning(
                "No citations found in answer",
                extra={
                    "operation": "validate_citations",
                    "answer_length": len(answer),
                    "status": "uncertain",
                },
            )
            return False

        # Build set of retrieved (chapter_id, section_number) tuples
        retrieved_ids = set()
        for chunk in retrieved_chunks:
            payload = chunk.get("payload", {})
            chapter_id = payload.get("chapter_id", "")
            section_num = payload.get("section_number", 0)
            retrieved_ids.add((chapter_id, str(section_num)))

        # Check if all citations are grounded
        for chapter_id, section_num in citations:
            if (chapter_id, section_num) not in retrieved_ids:
                logger.warning(
                    f"Citation not found in retrieved chunks: Chapter {chapter_id}, Section {section_num}",
                    extra={
                        "operation": "validate_citations",
                        "cited_chapter": chapter_id,
                        "cited_section": section_num,
                        "status": "ungrounded",
                    },
                )
                return False

        logger.info(
            f"Citations validated: {len(citations)} citations grounded",
            extra={
                "operation": "validate_citations",
                "citation_count": len(citations),
                "status": "valid",
            },
        )
        return True

    async def answer_question(
        self,
        question: str,
        retrieved_chunks: List[dict],
        request_id: Optional[str] = None,
    ) -> ChatResponse:
        """Orchestrate RAG pipeline: format context -> generate -> validate -> score.

        Args:
            question: User's question
            retrieved_chunks: Retrieved chunks from vector search
            request_id: Optional request ID for tracing

        Returns:
            ChatResponse with answer, sources, and metadata
        """
        pipeline_start = time.time()

        try:
            # Step 1: Extract and format sources
            sources = self._extract_sources(retrieved_chunks)
            context = self._format_context(retrieved_chunks)

            # Step 2: Calculate average relevance
            relevance_scores = [
                chunk.get("score", 0.0) for chunk in retrieved_chunks
            ]
            average_relevance = (
                sum(relevance_scores) / len(relevance_scores)
                if relevance_scores
                else 0.0
            )

            # Step 3: Generate answer with rate limiting
            generation_start = time.time()
            async with self.gemini_semaphore:
                answer = await self.gemini_service.generate_answer(
                    context, question, request_id
                )
            generation_latency_ms = (time.time() - generation_start) * 1000

            # Step 4: Validate citations
            citations_valid = self._validate_citations(answer, retrieved_chunks)

            # Step 5: Calculate confidence
            confidence = self._calculate_confidence(
                average_relevance, citations_valid
            )

            # Step 6: If citations invalid, append note to answer
            if not citations_valid:
                answer += "\n\n(Note: This answer could not be fully verified against the source material. Please review the sources below.)"

            # Calculate total latency
            total_latency_ms = (time.time() - pipeline_start) * 1000

            # Create metadata
            metadata = RAGMetadata(
                confidence_score=confidence,
                search_latency_ms=0,  # Will be set by caller
                generation_latency_ms=generation_latency_ms,
                total_latency_ms=total_latency_ms,
            )

            # Log successful completion
            logger.info(
                "RAG pipeline completed successfully",
                extra={
                    "operation": "rag_pipeline",
                    "request_id": request_id,
                    "question_length": len(question),
                    "answer_length": len(answer),
                    "source_count": len(sources),
                    "confidence": confidence,
                    "citations_valid": citations_valid,
                    "total_latency_ms": total_latency_ms,
                    "status": "success",
                },
            )

            return ChatResponse(answer=answer, sources=sources, metadata=metadata)

        except Exception as e:
            total_latency_ms = (time.time() - pipeline_start) * 1000
            logger.error(
                f"RAG pipeline failed: {str(e)}",
                extra={
                    "operation": "rag_pipeline",
                    "request_id": request_id,
                    "total_latency_ms": total_latency_ms,
                    "error": str(e),
                    "status": "failed",
                },
            )
            raise
