/**
 * Unit Tests for useChatHistory Hook
 * Tests session persistence and message management
 */

import { describe, it, expect, beforeEach, vi } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useChatHistory } from '../../hooks/useChatHistory';
import { ChatMessage } from '../../types/chatkit.types';
import * as storageService from '../../services/storageService';

vi.mock('../../services/storageService');

describe('useChatHistory Hook', () => {
  beforeEach(() => {
    vi.clearAllMocks();
    localStorage.clear();
    vi.mocked(storageService.getActiveSessionId).mockReturnValueOnce(null);
  });

  describe('session initialization', () => {
    it('should create new session on first render', async () => {
      const { result } = renderHook(() => useChatHistory());

      // Wait for useEffect
      await new Promise(resolve => setTimeout(resolve, 0));

      expect(result.current.session).not.toBeNull();
      expect(result.current.session?.messages).toEqual([]);
      expect(result.current.session?.sessionId).toBeDefined();
    });

    it('should load existing session if available', async () => {
      const mockSession = {
        sessionId: 'test-123',
        createdAt: Date.now(),
        lastMessageAt: Date.now(),
        messages: [],
        isActive: true,
      };

      vi.mocked(storageService.getActiveSessionId).mockReturnValueOnce(
        'test-123'
      );
      vi.mocked(storageService.loadSession).mockReturnValueOnce(mockSession);

      const { result } = renderHook(() => useChatHistory());

      await new Promise(resolve => setTimeout(resolve, 0));

      expect(result.current.session).toEqual(mockSession);
    });
  });

  describe('addMessage', () => {
    it('should add message to session', async () => {
      const { result } = renderHook(() => useChatHistory());

      await new Promise(resolve => setTimeout(resolve, 0));

      const testMessage: ChatMessage = {
        id: 'msg-1',
        role: 'user',
        content: 'Test question',
        timestamp: Date.now(),
        sessionId: result.current.session!.sessionId,
        status: 'sent',
      };

      act(() => {
        result.current.addMessage(testMessage);
      });

      expect(result.current.session?.messages).toContain(testMessage);
    });

    it('should update lastMessageAt timestamp', async () => {
      const { result } = renderHook(() => useChatHistory());

      await new Promise(resolve => setTimeout(resolve, 0));

      const originalTime = result.current.session!.lastMessageAt;

      await new Promise(resolve => setTimeout(resolve, 10));

      const testMessage: ChatMessage = {
        id: 'msg-1',
        role: 'user',
        content: 'Test',
        timestamp: Date.now(),
        sessionId: result.current.session!.sessionId,
        status: 'sent',
      };

      act(() => {
        result.current.addMessage(testMessage);
      });

      expect(result.current.session!.lastMessageAt).toBeGreaterThan(
        originalTime
      );
    });

    it('should persist to localStorage after adding message', async () => {
      const { result } = renderHook(() => useChatHistory());

      await new Promise(resolve => setTimeout(resolve, 0));

      const testMessage: ChatMessage = {
        id: 'msg-1',
        role: 'user',
        content: 'Test',
        timestamp: Date.now(),
        sessionId: result.current.session!.sessionId,
        status: 'sent',
      };

      act(() => {
        result.current.addMessage(testMessage);
      });

      expect(storageService.saveSession).toHaveBeenCalled();
    });
  });

  describe('clearHistory', () => {
    it('should clear all history and create new session', async () => {
      const { result } = renderHook(() => useChatHistory());

      await new Promise(resolve => setTimeout(resolve, 0));

      const oldSessionId = result.current.session!.sessionId;

      act(() => {
        result.current.clearHistory();
      });

      const newSessionId = result.current.session!.sessionId;

      expect(storageService.clearHistory).toHaveBeenCalled();
      expect(newSessionId).not.toBe(oldSessionId);
      expect(result.current.session?.messages).toEqual([]);
    });
  });
});
