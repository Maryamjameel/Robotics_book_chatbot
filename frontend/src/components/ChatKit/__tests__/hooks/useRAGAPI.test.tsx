/**
 * Unit Tests for useRAGAPI Hook
 * Tests backend API communication via React hook
 */

import { describe, it, expect, beforeEach, vi } from 'vitest';
import { renderHook, act, waitFor } from '@testing-library/react';
import { useRAGAPI } from '../../hooks/useRAGAPI';
import * as apiService from '../../services/apiService';

vi.mock('../../services/apiService');

describe('useRAGAPI Hook', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  describe('sendQuestion', () => {
    it('should initialize with loading false and error null', () => {
      const { result } = renderHook(() => useRAGAPI());

      expect(result.current.isLoading).toBe(false);
      expect(result.current.error).toBeNull();
    });

    it('should successfully send question and return response', async () => {
      const mockResponse = {
        answer: 'Test answer',
        sources: [],
        confidence: 0.9,
        metadata: {
          searchLatencyMs: 100,
          generationLatencyMs: 500,
          totalLatencyMs: 600,
          chunksRetrieved: 5,
          chunksUsed: 3,
          model: 'test-model',
        },
      };

      vi.mocked(apiService.sendQuestion).mockResolvedValueOnce(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      let response;
      await act(async () => {
        response = await result.current.sendQuestion('Test question');
      });

      expect(response).toEqual(mockResponse);
      expect(result.current.isLoading).toBe(false);
      expect(result.current.error).toBeNull();
    });

    it('should set error on API failure', async () => {
      vi.mocked(apiService.sendQuestion).mockRejectedValueOnce(
        new Error('API error')
      );

      const { result } = renderHook(() => useRAGAPI());

      await act(async () => {
        await result.current.sendQuestion('Test question');
      });

      expect(result.current.isLoading).toBe(false);
      expect(result.current.error).toBe('API error');
    });

    it('should reject empty question', async () => {
      const { result } = renderHook(() => useRAGAPI());

      await act(async () => {
        await result.current.sendQuestion('');
      });

      expect(result.current.error).toContain('Please enter a question');
    });

    it('should reject question over 2000 characters', async () => {
      const { result } = renderHook(() => useRAGAPI());

      await act(async () => {
        await result.current.sendQuestion('a'.repeat(2001));
      });

      expect(result.current.error).toContain('too long');
    });

    it('should handle timeout errors', async () => {
      vi.mocked(apiService.sendQuestion).mockRejectedValueOnce(
        new Error('timeout')
      );

      const { result } = renderHook(() => useRAGAPI());

      await act(async () => {
        await result.current.sendQuestion('Test question');
      });

      expect(result.current.error).toContain('timed out');
    });
  });

  describe('loading state', () => {
    it('should set isLoading to false after request completes', async () => {
      const mockResponse = {
        answer: 'Test',
        sources: [],
        confidence: 0.9,
        metadata: {
          searchLatencyMs: 0,
          generationLatencyMs: 0,
          totalLatencyMs: 0,
          chunksRetrieved: 0,
          chunksUsed: 0,
          model: 'test',
        },
      };

      vi.mocked(apiService.sendQuestion).mockResolvedValueOnce(mockResponse);

      const { result } = renderHook(() => useRAGAPI());

      expect(result.current.isLoading).toBe(false);

      await act(async () => {
        await result.current.sendQuestion('Test');
      });

      expect(result.current.isLoading).toBe(false);
    });
  });
});
