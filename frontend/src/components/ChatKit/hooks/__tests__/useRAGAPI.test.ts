/**
 * Unit tests for useRAGAPI hook with chapter context payload
 * Tests that chapter context is correctly included in RAG requests
 */

import { renderHook, act, waitFor } from '@testing-library/react';
import { useRAGAPI } from '../useRAGAPI';
import * as apiService from '../../services/apiService';
import { ChapterContext, RAGResponse } from '../../types/chatkit.types';

// Mock apiService
jest.mock('../../services/apiService');

describe('useRAGAPI - Chapter Context Payload', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('sendQuestion with chapter context', () => {
    it('should include chapter_context in request when provided', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      const chapterContext: ChapterContext = {
        chapterId: 'ch03',
        chapterTitle: 'Kinematics',
        chapterSlug: 'kinematics',
        confidence: 'high',
      };

      await act(async () => {
        await result.current.sendQuestion('What is forward kinematics?', undefined, undefined, chapterContext);
      });

      // Verify sendQuestion was called with chapter_context
      expect(apiService.sendQuestion).toHaveBeenCalledWith(
        expect.objectContaining({
          question: 'What is forward kinematics?',
          chapter_context: {
            chapter_id: 'ch03',
            chapter_title: 'Kinematics',
          },
        })
      );
    });

    it('should not include chapter_context when not provided', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      await act(async () => {
        await result.current.sendQuestion('What is forward kinematics?');
      });

      // Verify sendQuestion was called without chapter_context
      const call = (apiService.sendQuestion as jest.Mock).mock.calls[0][0];
      expect(call).not.toHaveProperty('chapter_context');
    });

    it('should map ChapterContext fields to request format correctly', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      const chapterContext: ChapterContext = {
        chapterId: 'chapter-3-kinematics',
        chapterTitle: 'Advanced Kinematics',
        chapterSlug: 'advanced-kinematics',
        confidence: 'medium',
      };

      await act(async () => {
        await result.current.sendQuestion('Test question', undefined, undefined, chapterContext);
      });

      // Verify field mapping
      const call = (apiService.sendQuestion as jest.Mock).mock.calls[0][0];
      expect(call.chapter_context).toEqual({
        chapter_id: 'chapter-3-kinematics',
        chapter_title: 'Advanced Kinematics',
      });

      // Verify chapterSlug is not sent (only in frontend)
      expect(call.chapter_context).not.toHaveProperty('chapterSlug');
      expect(call.chapter_context).not.toHaveProperty('confidence');
    });
  });

  describe('sendQuestion with all context types', () => {
    it('should include chapter_context along with selectedText and pageContext', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      const selectedText = 'Forward kinematics calculates position';
      const pageContext = {
        url: 'https://example.com/docs/chapter-3',
        pathname: '/docs/chapter-3',
        chapter: 'Kinematics',
        section: 'Forward Kinematics',
        confidence: 'high' as const,
      };
      const chapterContext: ChapterContext = {
        chapterId: 'ch03',
        chapterTitle: 'Kinematics',
        chapterSlug: 'kinematics',
        confidence: 'high',
      };

      await act(async () => {
        await result.current.sendQuestion('Question', selectedText, pageContext, chapterContext);
      });

      const call = (apiService.sendQuestion as jest.Mock).mock.calls[0][0];
      expect(call).toEqual({
        question: 'Question',
        selectedText: selectedText,
        pageContext: pageContext,
        chapter_context: {
          chapter_id: 'ch03',
          chapter_title: 'Kinematics',
        },
      });
    });
  });

  describe('chapter_context in response handling', () => {
    it('should handle response with chapter filtering metadata', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Answer about kinematics',
        sources: [],
        confidence: 0.95,
        metadata: {
          confidence_score: 0.95,
          search_latency_ms: 120,
          generation_latency_ms: 450,
          total_latency_ms: 570,
          chapter_filtered: true,
          chapter_id: 'ch03',
          boost_factor: 1.5,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      const chapterContext: ChapterContext = {
        chapterId: 'ch03',
        chapterTitle: 'Kinematics',
        chapterSlug: 'kinematics',
        confidence: 'high',
      };

      let response: RAGResponse | null = null;
      await act(async () => {
        response = await result.current.sendQuestion('Question', undefined, undefined, chapterContext);
      });

      // Verify response includes chapter filtering metadata
      expect(response?.metadata.chapter_filtered).toBe(true);
      expect(response?.metadata.chapter_id).toBe('ch03');
      expect(response?.metadata.boost_factor).toBe(1.5);
    });
  });

  describe('error handling with chapter context', () => {
    it('should handle API errors when sending chapter context', async () => {
      const errorMessage = 'API error with chapter context';
      (apiService.sendQuestion as jest.Mock).mockRejectedValue(new Error(errorMessage));

      const { result } = renderHook(() => useRAGAPI());

      const chapterContext: ChapterContext = {
        chapterId: 'ch03',
        chapterTitle: 'Kinematics',
        chapterSlug: 'kinematics',
        confidence: 'high',
      };

      await act(async () => {
        const response = await result.current.sendQuestion('Question', undefined, undefined, chapterContext);
        expect(response).toBeNull();
      });

      await waitFor(() => {
        expect(result.current.error).toBe(errorMessage);
      });
    });

    it('should set loading state correctly with chapter context', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      expect(result.current.isLoading).toBe(false);

      const chapterContext: ChapterContext = {
        chapterId: 'ch03',
        chapterTitle: 'Kinematics',
        chapterSlug: 'kinematics',
        confidence: 'high',
      };

      let loadingDuringRequest = false;
      act(() => {
        const promise = result.current.sendQuestion('Question', undefined, undefined, chapterContext);
        if (result.current.isLoading) {
          loadingDuringRequest = true;
        }
        return promise;
      });

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });
    });
  });

  describe('chapter context with different confidence levels', () => {
    it('should send chapter context with high confidence', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      const chapterContext: ChapterContext = {
        chapterId: 'ch03',
        chapterTitle: 'Kinematics',
        chapterSlug: 'kinematics',
        confidence: 'high',
      };

      await act(async () => {
        await result.current.sendQuestion('Question', undefined, undefined, chapterContext);
      });

      const call = (apiService.sendQuestion as jest.Mock).mock.calls[0][0];
      expect(call.chapter_context.chapter_id).toBe('ch03');
    });

    it('should send chapter context with medium confidence', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      const chapterContext: ChapterContext = {
        chapterId: 'ch04',
        chapterTitle: 'Dynamics',
        chapterSlug: 'dynamics',
        confidence: 'medium',
      };

      await act(async () => {
        await result.current.sendQuestion('Question', undefined, undefined, chapterContext);
      });

      const call = (apiService.sendQuestion as jest.Mock).mock.calls[0][0];
      expect(call.chapter_context.chapter_id).toBe('ch04');
    });

    it('should send chapter context with low confidence', async () => {
      const mockResponse: RAGResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          confidence_score: 0.9,
          search_latency_ms: 100,
          generation_latency_ms: 500,
          total_latency_ms: 600,
        },
      };

      (apiService.sendQuestion as jest.Mock).mockResolvedValue(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      const chapterContext: ChapterContext = {
        chapterId: 'unknown',
        chapterTitle: 'Unknown Chapter',
        chapterSlug: '',
        confidence: 'low',
      };

      await act(async () => {
        await result.current.sendQuestion('Question', undefined, undefined, chapterContext);
      });

      const call = (apiService.sendQuestion as jest.Mock).mock.calls[0][0];
      expect(call.chapter_context.chapter_id).toBe('unknown');
    });
  });
});
