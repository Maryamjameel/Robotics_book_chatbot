/**
 * Unit Tests for apiService
 * Tests backend API communication and error handling
 */

import { describe, it, expect, beforeEach, vi } from 'vitest';
import { sendQuestion } from '../../services/apiService';
import { RAGRequest, RAGResponse } from '../../types/chatkit.types';

describe('apiService', () => {
  const mockRequest: RAGRequest = {
    question: 'What is forward kinematics?',
  };

  const mockResponse: RAGResponse = {
    answer: 'Forward kinematics is...',
    sources: [
      {
        id: 'chunk-1',
        title: 'Chapter 3',
        snippet: 'Forward kinematics...',
        url: 'http://example.com',
        similarity: 0.92,
      },
    ],
    confidence: 0.87,
    metadata: {
      searchLatencyMs: 100,
      generationLatencyMs: 500,
      totalLatencyMs: 600,
      chunksRetrieved: 5,
      chunksUsed: 3,
      model: 'gpt-4o-mini',
    },
  };

  beforeEach(() => {
    vi.clearAllMocks();
    global.fetch = vi.fn();
  });

  describe('sendQuestion', () => {
    it('should successfully send question and receive response', async () => {
      global.fetch = vi.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const result = await sendQuestion(mockRequest);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('api/v1/chat/ask'),
        expect.objectContaining({
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
        })
      );
    });

    it('should throw error on empty question', async () => {
      const emptyRequest: RAGRequest = { question: '' };

      await expect(sendQuestion(emptyRequest)).rejects.toThrow(
        'Question cannot be empty'
      );
    });

    it('should throw error on question exceeding 2000 chars', async () => {
      const longRequest: RAGRequest = {
        question: 'a'.repeat(2001),
      };

      await expect(sendQuestion(longRequest)).rejects.toThrow(
        'exceeds maximum length'
      );
    });

    it('should handle timeout error', async () => {
      global.fetch = vi.fn().mockRejectedValueOnce(
        new DOMException('AbortError')
      );

      await expect(sendQuestion(mockRequest, 1000)).rejects.toThrow(
        'timed out'
      );
    });

    it('should handle 503 service unavailable', async () => {
      global.fetch = vi.fn().mockResolvedValueOnce({
        ok: false,
        status: 503,
        statusText: 'Service Unavailable',
      });

      await expect(sendQuestion(mockRequest)).rejects.toThrow(
        'Service unavailable'
      );
    });

    it('should handle invalid JSON response', async () => {
      global.fetch = vi.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => {
          throw new Error('Invalid JSON');
        },
      });

      await expect(sendQuestion(mockRequest)).rejects.toThrow(
        'parse response'
      );
    });
  });
});
