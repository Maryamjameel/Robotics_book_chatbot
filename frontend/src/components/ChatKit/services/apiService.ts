/**
 * API Service - Backend communication with RAG API
 *
 * Handles all HTTP requests to the backend RAG endpoint.
 * Implements timeout handling with AbortController and error recovery.
 * Uses native fetch API (no external dependencies).
 */

import { RAGRequest, RAGResponse } from '../types/chatkit.types';
import { chatKitConfig } from '../../config/chatkit.config';

/**
 * Send a question to the RAG backend and receive an answer
 * Implements 30-second timeout per FR-010 specification
 *
 * @param request - RAGRequest with question, optional context
 * @param timeoutMs - Request timeout in milliseconds (default: 30000)
 * @returns RAGResponse with answer, sources, confidence, metadata
 * @throws Error on network failure, timeout, or invalid response
 */
export async function sendQuestion(
  request: RAGRequest,
  timeoutMs: number = chatKitConfig.requestTimeoutMs
): Promise<RAGResponse> {
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

    // Send POST request to backend
    const response = await fetch(chatKitConfig.apiEndpoint, {
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
      if (statusCode === 503 || statusCode === 502 || statusCode === 504) {
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

    const response = await fetch(`${chatKitConfig.apiEndpoint}/health`, {
      method: 'GET',
      signal: controller.signal,
    });

    clearTimeout(timeoutId);
    return response.ok;
  } catch {
    return false;
  }
}

export const apiService = {
  sendQuestion,
  healthCheck,
};
