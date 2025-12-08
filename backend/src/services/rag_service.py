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
            # Chunks are now direct dicts (not wrapped in payload)
            source = Source(
                chapter_id=chunk.get("chapter_id", "unknown"),
                chapter_title=chunk.get("chapter_title", "Unknown"),
                section_number=chunk.get("section_number", 0),
                section_title=chunk.get("section_title", "Unknown"),
                excerpt=chunk.get("excerpt", "")[:500],  # Truncate excerpt to 500 chars
                relevance_score=chunk.get("relevance_score", 0.0),
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
            # Chunks are now direct dicts (not wrapped in payload)
            chapter_id = chunk.get("chapter_id", "unknown")
            chapter_title = chunk.get("chapter_title", "Unknown")
            section_num = chunk.get("section_number", 0)
            section_title = chunk.get("section_title", "Unknown")
            text = chunk.get("excerpt", "")  # Use 'excerpt' field
            score = chunk.get("relevance_score", 0.0)

            context_parts.append(
                f"[{i}] {chapter_title} - Chapter {chapter_id}, Section {section_num}: {section_title} (relevance: {score:.2f})\n{text}"
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

        Uses flexible matching to account for different citation formats.
        Patterns supported:
        - "Chapter X, Section Y"
        - "Section Y"
        - "Chapter X"

        Args:
            answer: Generated answer text
            retrieved_chunks: Retrieved chunks from search

        Returns:
            True if at least one citation is grounded, False if none found or all hallucinated
        """
        # Build set of retrieved (chapter_id, section_number) tuples
        retrieved_ids = {}  # Dict for more flexible lookup
        retrieved_chapters = set()
        retrieved_sections = set()

        for chunk in retrieved_chunks:
            payload = chunk.get("payload", {})
            chapter_id = payload.get("chapter_id", "").lower()
            section_num = payload.get("section_number", 0)
            section_title = payload.get("section_title", "").lower()

            if chapter_id:
                retrieved_chapters.add(chapter_id)
                retrieved_ids[(chapter_id, str(section_num))] = True
            if section_title:
                retrieved_sections.add(section_title)

        answer_lower = answer.lower()

        # Try to find citations in order of specificity
        # Pattern 1: "Chapter X, Section Y"
        citation_pattern_specific = r"Chapter\s+([a-zA-Z0-9]+),?\s+Section\s+(\d+)"
        specific_citations = re.findall(citation_pattern_specific, answer_lower)

        if specific_citations:
            for chapter_id, section_num in specific_citations:
                if (chapter_id, section_num) in retrieved_ids:
                    logger.info(
                        f"Citation validated: Chapter {chapter_id}, Section {section_num}",
                        extra={
                            "operation": "validate_citations",
                            "citation_type": "specific",
                            "chapter_id": chapter_id,
                            "section_num": section_num,
                            "status": "valid",
                        },
                    )
                    return True

        # Pattern 2: "Section Y" or just section numbers mentioned
        section_pattern = r"Section\s+(\d+)"
        section_citations = re.findall(section_pattern, answer_lower)

        if section_citations:
            for section_num in section_citations:
                # Check if this section exists in any retrieved chunk
                for (ch_id, sec_num) in retrieved_ids.keys():
                    if sec_num == section_num:
                        logger.info(
                            f"Citation validated: Section {section_num}",
                            extra={
                                "operation": "validate_citations",
                                "citation_type": "section_only",
                                "section_num": section_num,
                                "status": "valid",
                            },
                        )
                        return True

        # Pattern 3: Chapter reference
        chapter_pattern = r"Chapter\s+([a-zA-Z0-9]+)"
        chapter_citations = re.findall(chapter_pattern, answer_lower)

        if chapter_citations:
            for chapter_id in chapter_citations:
                if chapter_id in retrieved_chapters:
                    logger.info(
                        f"Citation validated: Chapter {chapter_id}",
                        extra={
                            "operation": "validate_citations",
                            "citation_type": "chapter_only",
                            "chapter_id": chapter_id,
                            "status": "valid",
                        },
                    )
                    return True

        # If no citations found, still allow answer but mark as uncertain
        if not (specific_citations or section_citations or chapter_citations):
            logger.warning(
                "No citations found in answer",
                extra={
                    "operation": "validate_citations",
                    "answer_length": len(answer),
                    "status": "uncertain",
                },
            )
            return False

        # Citations found but not grounded in retrieved chunks
        logger.warning(
            "Citations found but not grounded in retrieved chunks",
            extra={
                "operation": "validate_citations",
                "answer_length": len(answer),
                "status": "ungrounded",
            },
        )
        return False

    async def answer_question(
        self,
        question: str,
        retrieved_chunks: List[dict],
        request_id: Optional[str] = None,
    ) -> ChatResponse:
        """Orchestrate RAG pipeline: format context -> generate -> validate -> score.

        Always returns a ChatResponse, even if LLM generation fails.
        In case of LLM failure, returns the sources with a helpful fallback message.

        Args:
            question: User's question
            retrieved_chunks: Retrieved chunks from vector search
            request_id: Optional request ID for tracing

        Returns:
            ChatResponse with answer, sources, and metadata
        """
        pipeline_start = time.time()

        # Step 1: Extract and format sources (always do this first)
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

        try:
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

            # Create metadata with all fields populated
            metadata = RAGMetadata(
                confidence_score=confidence,
                search_latency_ms=0,  # Will be set by caller
                generation_latency_ms=generation_latency_ms,
                total_latency_ms=total_latency_ms,
                selected_text_boosted=False,  # Will be updated by caller if applicable
                boost_factor=1.0,  # Default no boost
                selected_text_terms=[],  # Will be updated by caller if applicable
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

            return ChatResponse(
                answer=answer,
                sources=sources,
                confidence=confidence,
                metadata=metadata
            )

        except Exception as e:
            # LLM generation failed - return fallback response with sources
            generation_latency_ms = (time.time() - pipeline_start) * 1000
            total_latency_ms = generation_latency_ms

            logger.warning(
                f"LLM generation failed, returning fallback response with sources: {str(e)}",
                extra={
                    "operation": "rag_pipeline",
                    "request_id": request_id,
                    "total_latency_ms": total_latency_ms,
                    "error": str(e),
                    "source_count": len(sources),
                    "status": "fallback",
                },
            )

            # Create a helpful fallback message with source information
            fallback_answer = self._create_fallback_answer(sources, question)

            # Create metadata with lower confidence since we couldn't generate an answer
            metadata = RAGMetadata(
                confidence_score=0.6,  # Medium-low confidence for fallback
                search_latency_ms=0,  # Will be set by caller
                generation_latency_ms=generation_latency_ms,
                total_latency_ms=total_latency_ms,
                selected_text_boosted=False,
                boost_factor=1.0,
                selected_text_terms=[],
            )

            return ChatResponse(
                answer=fallback_answer,
                sources=sources,
                confidence=0.6,
                metadata=metadata
            )

    def _create_fallback_answer(self, sources: List[Source], question: str) -> str:
        """Create a helpful fallback answer when LLM generation fails.

        Args:
            sources: List of relevant sources found
            question: User's question

        Returns:
            Formatted fallback answer with source information
        """
        if not sources:
            return (
                "I apologize, but I'm experiencing technical difficulties generating a complete answer right now. "
                "No relevant sources were found for your question. Please try rephrasing your question or try again in a moment."
            )

        # Build a helpful response with source excerpts
        fallback_parts = [
            "I apologize, but I'm experiencing technical difficulties generating a complete answer right now. "
            "However, I found the following relevant content from the robotics textbook that may help answer your question:\n"
        ]

        for i, source in enumerate(sources[:3], 1):  # Show top 3 sources
            fallback_parts.append(
                f"\n**{i}. {source.chapter_title} - Chapter {source.chapter_id}, Section {source.section_number}**"
            )
            if source.section_title:
                fallback_parts.append(f"\n*{source.section_title}*")

            # Include a preview of the content
            excerpt_preview = source.excerpt[:300] + "..." if len(source.excerpt) > 300 else source.excerpt
            fallback_parts.append(f"\n{excerpt_preview}\n")

        fallback_parts.append(
            "\nPlease review the sources above for detailed information. "
            "You can also try asking your question again in a moment, or rephrase it for better results."
        )

        return "".join(fallback_parts)
