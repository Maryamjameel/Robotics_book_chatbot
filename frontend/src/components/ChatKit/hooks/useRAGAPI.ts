/**
 * useRAGAPI Hook - Backend communication with loading and error states
 *
 * Manages async communication with the RAG backend API.
 * Handles loading states, error messages, and timeout recovery.
 * Provides a clean interface for components to send questions and receive answers.
 */

import { useState, useCallback } from 'react';
import { RAGRequest, RAGResponse, PageContext, ChapterContext } from '../types/chatkit.types';
import { apiService } from '../services/apiService';

interface UseRAGAPIReturn {
  /** Send a question to the backend and get a response */
  sendQuestion: (
    question: string,
    selectedText?: string,
    pageContext?: PageContext,
    chapterContext?: ChapterContext
  ) => Promise<RAGResponse | null>;

  /** Whether a request is currently in flight */
  isLoading: boolean;

  /** Error message if the last request failed */
  error: string | null;
}

/**
 * Hook for RAG API communication
 * Manages loading state, errors, and request lifecycle
 *
 * @returns Object with sendQuestion function, isLoading, and error state
 */
export function useRAGAPI(): UseRAGAPIReturn {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendQuestion = useCallback(
    async (
      question: string,
      selectedText?: string,
      pageContext?: PageContext,
      chapterContext?: ChapterContext
    ): Promise<RAGResponse | null> => {
      // Reset error state before new request
      setError(null);
      setIsLoading(true);

      try {
        // Validate input
        if (!question || question.trim().length === 0) {
          setError('Please enter a question');
          setIsLoading(false);
          return null;
        }

        if (question.length > 2000) {
          setError('Question too long (max 2000 characters)');
          setIsLoading(false);
          return null;
        }

        // Build request with all optional context
        const request: RAGRequest = {
          question,
          ...(selectedText && { selectedText }),
          ...(pageContext && { pageContext }),
          ...(chapterContext && {
            chapter_context: {
              chapter_id: chapterContext.chapterId,
              chapter_title: chapterContext.chapterTitle,
            },
          }),
        };

        // Send to backend
        const response = await apiService.sendQuestion(request);

        setIsLoading(false);
        return response;
      } catch (err) {
        // Extract error message
        let errorMessage = 'An unexpected error occurred';

        if (err instanceof Error) {
          errorMessage = err.message;
        } else if (typeof err === 'string') {
          errorMessage = err;
        }

        // Map specific error messages to user-friendly ones
        if (
          errorMessage.includes('timeout') ||
          errorMessage.includes('timed out')
        ) {
          errorMessage = 'Your request timed out. Please try again.';
        } else if (
          errorMessage.includes('ServiceUnavailable') ||
          errorMessage.includes('503')
        ) {
          errorMessage = 'Service unavailable. Please try again later.';
        } else if (errorMessage.includes('Network')) {
          errorMessage =
            'Network connection error. Please check your internet connection.';
        }

        setError(errorMessage);
        setIsLoading(false);
        return null;
      }
    },
    []
  );

  return {
    sendQuestion,
    isLoading,
    error,
  };
}
