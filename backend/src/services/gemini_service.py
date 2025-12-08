"""Gemini LLM service for RAG answer generation."""

import asyncio
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
        self.model_name = config.gemini_chat_model

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
        """Generate answer using Gemini LLM with robust retry logic.

        Args:
            context: Formatted context from retrieved chunks
            question: User's question
            request_id: Optional request ID for tracing

        Returns:
            Generated answer with embedded citations

        Raises:
            ValueError: If API key is missing
            RuntimeError: If LLM generation fails after all retries
        """
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY not configured")

        start_time = time.time()
        attempt = 0
        max_attempts = 5  # Increased retries for better resilience
        last_error = None

        while attempt < max_attempts:
            try:
                # Import and acquire global rate limiter BEFORE API call
                from .rate_limiter import gemini_rate_limiter
                await gemini_rate_limiter.acquire()

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
                    last_error = "Empty response from Gemini API"
                    if attempt < max_attempts:
                        wait_time = min(2 ** (attempt - 1), 8)  # Cap at 8 seconds
                        logger.warning(
                            f"Empty response, retrying in {wait_time}s (attempt {attempt}/{max_attempts})",
                            extra={
                                "operation": "gemini_generate",
                                "request_id": request_id,
                                "attempt": attempt,
                                "wait_time": wait_time,
                                "status": "empty_response_retry",
                            },
                        )
                        await asyncio.sleep(wait_time)
                        continue
                    raise RuntimeError(last_error)

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
                        "attempts": attempt,
                        "status": "success",
                    },
                )

                return answer

            except (json.JSONDecodeError, KeyError, AttributeError) as e:
                last_error = e
                latency_ms = (time.time() - start_time) * 1000

                if attempt < max_attempts:
                    wait_time = min(2 ** (attempt - 1), 8)  # Exponential backoff, capped at 8s
                    logger.warning(
                        f"Attempt {attempt} failed: {type(e).__name__}, retrying in {wait_time}s",
                        extra={
                            "operation": "gemini_generate",
                            "request_id": request_id,
                            "model": self.model_name,
                            "attempt": attempt,
                            "latency_ms": latency_ms,
                            "error": str(e),
                            "wait_time": wait_time,
                            "status": "retry",
                        },
                    )
                    await asyncio.sleep(wait_time)
                    continue

            except Exception as e:
                last_error = e
                latency_ms = (time.time() - start_time) * 1000
                error_type = type(e).__name__
                error_str = str(e).lower()

                # Check for non-retryable errors (configuration/authentication issues)
                if "404" in str(e) or "not found" in error_str:
                    logger.error(
                        "Model not found - configuration error, not retrying",
                        extra={
                            "operation": "gemini_generate",
                            "request_id": request_id,
                            "model": self.model_name,
                            "latency_ms": latency_ms,
                            "error": str(e),
                            "status": "model_not_found",
                        },
                    )
                    raise RuntimeError(f"Model configuration error: {str(e)}")

                # Check for authentication errors
                if "401" in str(e) or "403" in str(e) or "unauthorized" in error_str or "forbidden" in error_str or "api key" in error_str:
                    logger.error(
                        "Authentication error - invalid API key, not retrying",
                        extra={
                            "operation": "gemini_generate",
                            "request_id": request_id,
                            "model": self.model_name,
                            "latency_ms": latency_ms,
                            "error": str(e),
                            "status": "auth_error",
                        },
                    )
                    raise RuntimeError(f"Authentication error: Invalid or missing API key")

                # Check for rate limit errors
                if "429" in str(e) or "quota" in error_str or "resource" in error_str or "rate" in error_str:
                    if attempt < max_attempts:
                        wait_time = min(2 ** attempt, 16)  # Longer backoff for rate limits, cap at 16s
                        logger.warning(
                            f"Rate limit hit, retrying in {wait_time}s (attempt {attempt}/{max_attempts})",
                            extra={
                                "operation": "gemini_generate",
                                "request_id": request_id,
                                "model": self.model_name,
                                "attempt": attempt,
                                "wait_time": wait_time,
                                "status": "rate_limit_retry",
                            },
                        )
                        await asyncio.sleep(wait_time)
                        continue

                # Check for timeout errors
                elif "timeout" in error_str:
                    if attempt < max_attempts:
                        wait_time = min(2 ** (attempt - 1), 8)
                        logger.warning(
                            f"Timeout error, retrying in {wait_time}s (attempt {attempt}/{max_attempts})",
                            extra={
                                "operation": "gemini_generate",
                                "request_id": request_id,
                                "model": self.model_name,
                                "attempt": attempt,
                                "wait_time": wait_time,
                                "status": "timeout_retry",
                            },
                        )
                        await asyncio.sleep(wait_time)
                        continue

                # Check for temporary service errors
                elif any(keyword in error_str for keyword in ["503", "unavailable", "overloaded", "capacity"]):
                    if attempt < max_attempts:
                        wait_time = min(2 ** attempt, 16)
                        logger.warning(
                            f"Service unavailable, retrying in {wait_time}s (attempt {attempt}/{max_attempts})",
                            extra={
                                "operation": "gemini_generate",
                                "request_id": request_id,
                                "model": self.model_name,
                                "attempt": attempt,
                                "wait_time": wait_time,
                                "status": "service_unavailable_retry",
                            },
                        )
                        await asyncio.sleep(wait_time)
                        continue

                # For other errors, retry with shorter backoff
                if attempt < max_attempts:
                    wait_time = min(2 ** (attempt - 1), 8)
                    logger.warning(
                        f"{error_type} error, retrying in {wait_time}s (attempt {attempt}/{max_attempts})",
                        extra={
                            "operation": "gemini_generate",
                            "request_id": request_id,
                            "model": self.model_name,
                            "attempt": attempt,
                            "error": str(e),
                            "wait_time": wait_time,
                            "status": "generic_retry",
                        },
                    )
                    await asyncio.sleep(wait_time)
                    continue

        # All retries exhausted
        latency_ms = (time.time() - start_time) * 1000
        logger.error(
            f"All {max_attempts} retry attempts exhausted",
            extra={
                "operation": "gemini_generate",
                "request_id": request_id,
                "model": self.model_name,
                "latency_ms": latency_ms,
                "last_error": str(last_error) if last_error else "Unknown",
                "status": "all_retries_failed",
            },
        )
        raise RuntimeError(f"LLM service failed after {max_attempts} attempts: {str(last_error)}")
