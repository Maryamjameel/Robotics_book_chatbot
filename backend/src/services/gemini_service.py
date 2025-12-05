"""Gemini LLM service for RAG answer generation."""

import json
import logging
import time
from typing import Optional

import google.generativeai as genai

from src.config import config, logger


class GeminiService:
    """Service for generating answers using Google Gemini LLM."""

    def __init__(self):
        """Initialize Gemini service with API configuration."""
        self.api_key = config.gemini_api_key
        self.model_name = config.gemini_model

        # Configure Gemini API
        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel(self.model_name)

        # System prompt for RAG citation requirements
        self.system_prompt = """You are a helpful educational assistant for robotics textbook questions.

CRITICAL REQUIREMENTS:
1. Always cite your sources in the format: Source: Chapter X, Section Y - Section Title
2. Base your answer ONLY on the provided context from textbook sections
3. If you cannot answer from the provided context, say "I cannot find this information in the textbook"
4. Do not hallucinate sources or information not in the provided context
5. Be clear and educational in your explanations

Format citations as: Source: Chapter {id}, Section {number} - {title}
Example: "Source: Chapter 3, Section 1 - Forward Kinematics"

You MUST cite at least one source for every factual claim in your answer."""

        logger.info(
            "GeminiService initialized",
            extra={
                "operation": "gemini_init",
                "model": self.model_name,
                "status": "success",
            },
        )

    async def generate_answer(
        self, context: str, question: str, request_id: Optional[str] = None
    ) -> str:
        """Generate answer using Gemini LLM.

        Args:
            context: Formatted context from retrieved chunks
            question: User's question
            request_id: Optional request ID for tracing

        Returns:
            Generated answer with embedded citations

        Raises:
            ValueError: If API key is missing
            RuntimeError: If LLM generation fails
        """
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY not configured")

        start_time = time.time()
        attempt = 0
        max_attempts = 2

        while attempt < max_attempts:
            try:
                attempt += 1

                # Construct the full prompt
                prompt = f"""{self.system_prompt}

CONTEXT FROM TEXTBOOK:
{context}

QUESTION: {question}

ANSWER:"""

                # Generate response using Gemini
                response = self.model.generate_content(prompt)

                if not response or not response.text:
                    raise RuntimeError("Empty response from Gemini API")

                answer = response.text.strip()
                latency_ms = (time.time() - start_time) * 1000

                # Log successful generation
                logger.info(
                    "Gemini generation successful",
                    extra={
                        "operation": "gemini_generate",
                        "request_id": request_id,
                        "model": self.model_name,
                        "question_length": len(question),
                        "answer_length": len(answer),
                        "latency_ms": latency_ms,
                        "status": "success",
                    },
                )

                return answer

            except (
                json.JSONDecodeError,
                KeyError,
                AttributeError,
            ) as e:
                latency_ms = (time.time() - start_time) * 1000
                logger.warning(
                    f"Attempt {attempt} failed: {type(e).__name__}",
                    extra={
                        "operation": "gemini_generate",
                        "request_id": request_id,
                        "model": self.model_name,
                        "attempt": attempt,
                        "latency_ms": latency_ms,
                        "error": str(e),
                        "status": "retry",
                    },
                )

                if attempt >= max_attempts:
                    raise RuntimeError(f"Gemini API failed after {max_attempts} attempts: {str(e)}")

            except Exception as e:
                latency_ms = (time.time() - start_time) * 1000

                # Log specific error types
                error_type = type(e).__name__
                if "429" in str(e) or "quota" in str(e).lower():
                    logger.error(
                        "Rate limit exceeded",
                        extra={
                            "operation": "gemini_generate",
                            "request_id": request_id,
                            "model": self.model_name,
                            "latency_ms": latency_ms,
                            "error_type": "rate_limit",
                            "status": "failed",
                        },
                    )
                    raise RuntimeError("Rate limit exceeded. Please try again later.") from e

                elif "timeout" in str(e).lower():
                    logger.error(
                        "Generation timeout",
                        extra={
                            "operation": "gemini_generate",
                            "request_id": request_id,
                            "model": self.model_name,
                            "latency_ms": latency_ms,
                            "error_type": "timeout",
                            "status": "failed",
                        },
                    )
                    raise RuntimeError("Generation request timed out") from e

                else:
                    logger.error(
                        f"Gemini API error: {error_type}",
                        extra={
                            "operation": "gemini_generate",
                            "request_id": request_id,
                            "model": self.model_name,
                            "latency_ms": latency_ms,
                            "error_type": error_type,
                            "error": str(e),
                            "status": "failed",
                        },
                    )
                    raise RuntimeError(f"LLM service failed: {str(e)}") from e
