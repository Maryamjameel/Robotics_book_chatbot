/**
 * API Service - Backend communication with RAG API
 *
 * Handles all HTTP requests to the backend RAG endpoint.
 * Implements timeout handling with AbortController and error recovery.
 * Uses native fetch API (no external dependencies).
 */

import { RAGRequest, RAGResponse } from '../types/chatkit.types';
import { apiConfig } from '../../../config/api';
import { chatKitConfig } from '../../../config/chatkit.config';

// Rate limiting: Track last request time to prevent hitting Gemini API limits
// Backend handles retry logic, so frontend only needs minimal throttling
let lastRequestTime = 0;
const MIN_REQUEST_INTERVAL_MS = 1000; // 1 second between requests (backend handles heavier rate limiting)

/**
 * Send a question to the RAG backend and receive an answer
 * Implements 30-second timeout per FR-010 specification
 * Supports optional chapter context for filtering and re-ranking results
 * RATE LIMITED: Enforces 1-second minimum interval between requests
 *
 * @param request - RAGRequest with question, optional context:
 *   - question: User question (required, 1-2000 chars)
 *   - selectedText: Optional selected text from page (for TF-IDF boosting)
 *   - pageContext: Optional page metadata (URL, pathname, confidence)
 *   - chapter_context: Optional chapter context for filtering search results
 *     - chapter_id: Chapter identifier (e.g., 'ch03')
 *     - chapter_title: Chapter title (e.g., 'Kinematics')
 *   - sessionId: Optional session ID for conversation tracking
 * @param timeoutMs - Request timeout in milliseconds (default: 30000)
 * @returns RAGResponse with answer, sources, confidence, metadata including:
 *   - metadata.chapter_filtered: Whether results were filtered by chapter
 *   - metadata.chapter_id: Chapter ID used for filtering (if applicable)
 *   - metadata.boost_factor: TF-IDF boost factor applied (1.0 = no boost)
 * @throws Error on network failure, timeout, invalid response, or rate limiting
 */
export async function sendQuestion(
  request: RAGRequest,
  timeoutMs: number = chatKitConfig.requestTimeoutMs
): Promise<RAGResponse> {
  // Rate limiting check: Enforce minimum interval between requests
  const now = Date.now();
  const timeSinceLastRequest = now - lastRequestTime;

  if (lastRequestTime > 0 && timeSinceLastRequest < MIN_REQUEST_INTERVAL_MS) {
    const waitTimeSeconds = Math.ceil((MIN_REQUEST_INTERVAL_MS - timeSinceLastRequest) / 1000);
    const timeUnit = waitTimeSeconds === 1 ? 'second' : 'seconds';
    throw new Error(
      `Please wait ${waitTimeSeconds} ${timeUnit} before asking another question.`
    );
  }

  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeoutMs);

  try {
    // Validate request
    if (!request.question || request.question.trim().length === 0) {
      throw new Error('Question cannot be empty');
    }

    if (request.question.length > 2000) {
      throw new Error('Question exceeds maximum length of 2000 characters');
    }

    // Send POST request to backend using configured API endpoint
    // Uses apiConfig for environment-based endpoint selection (dev/prod/staging)
    const chatEndpoint = `${apiConfig.baseURL}/v1/chat/ask`;
    const response = await fetch(chatEndpoint, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
      signal: controller.signal,
    });

    // Handle HTTP error responses
    if (!response.ok) {
      const statusCode = response.status;

      // Parse error response if available
      let errorMessage = `API returned status ${statusCode}`;
      try {
        const errorData = await response.json();
        if (errorData.error?.message) {
          errorMessage = errorData.error.message;
        }
      } catch {
        // If error body isn't JSON, use status message
        errorMessage = response.statusText || errorMessage;
      }

      // Map specific HTTP status codes to user-friendly messages
      if (statusCode === 429) {
        throw new Error(
          'Too many requests. The backend is rate limited. Please wait a moment and try again.'
        );
      } else if (statusCode === 503 || statusCode === 502 || statusCode === 504) {
        throw new Error(
          'Service unavailable. Please try again later.'
        );
      } else if (statusCode === 400) {
        throw new Error('Invalid question format. Please check and try again.');
      } else if (statusCode >= 500) {
        throw new Error('Server error. Please try again later.');
      } else {
        throw new Error(errorMessage);
      }
    }

    // Parse response
    let responseData: RAGResponse;
    try {
      responseData = await response.json();
    } catch (error) {
      throw new Error('Failed to parse response from server');
    }

    // Validate response structure
    if (
      !responseData.answer ||
      !Array.isArray(responseData.sources) ||
      typeof responseData.confidence !== 'number' ||
      !responseData.metadata
    ) {
      throw new Error('Invalid response format from server');
    }

    // Update last request time on successful response
    lastRequestTime = Date.now();

    return responseData;
  } catch (error) {
    // Handle AbortError (timeout)
    if (error instanceof DOMException && error.name === 'AbortError') {
      throw new Error(
        `Your request timed out. Please try again.`
      );
    }

    // Handle network errors
    if (error instanceof TypeError && error.message.includes('fetch')) {
      throw new Error(
        'Network connection error. Please check your internet connection.'
      );
    }

    // Re-throw our own errors, wrap unknown errors
    if (error instanceof Error) {
      throw error;
    }

    throw new Error('An unexpected error occurred');
  } finally {
    // Always clean up timeout
    clearTimeout(timeoutId);
  }
}

/**
 * Health check - ping the backend to verify it's available
 * Useful for pre-flight checks before user interaction
 *
 * @returns true if backend is available, false otherwise
 */
export async function healthCheck(): Promise<boolean> {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 5000); // 5 second timeout

    const response = await fetch(getHealthUrl(), {
      method: 'GET',
      signal: controller.signal,
    });

    clearTimeout(timeoutId);
    return response.ok;
  } catch {
    return false;
  }
}

function getHealthUrl(): string {
  return `${apiConfig.baseURL}/health`;
}

export const apiService = {
  sendQuestion,
  healthCheck,
};
