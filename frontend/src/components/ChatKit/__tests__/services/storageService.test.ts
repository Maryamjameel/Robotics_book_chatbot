/**
 * Unit Tests for storageService
 * Tests localStorage persistence operations
 */

import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import {
  saveSession,
  loadSession,
  getActiveSessionId,
  clearHistory,
} from '../../services/storageService';
import { ChatSession } from '../../types/chatkit.types';

describe('storageService', () => {
  const mockSession: ChatSession = {
    sessionId: 'test-session-123',
    createdAt: Date.now(),
    lastMessageAt: Date.now(),
    messages: [],
    isActive: true,
  };

  beforeEach(() => {
    // Clear localStorage before each test
    localStorage.clear();
  });

  afterEach(() => {
    localStorage.clear();
  });

  describe('saveSession', () => {
    it('should save session to localStorage', () => {
      saveSession(mockSession);
      const stored = localStorage.getItem(
        `chatkit-session-${mockSession.sessionId}`
      );
      expect(stored).toBeDefined();
      expect(JSON.parse(stored!)).toEqual(mockSession);
    });

    it('should set active session ID', () => {
      saveSession(mockSession);
      const activeId = localStorage.getItem('chatkit-active-session');
      expect(activeId).toBe(mockSession.sessionId);
    });
  });

  describe('loadSession', () => {
    it('should load session from localStorage', () => {
      saveSession(mockSession);
      const loaded = loadSession(mockSession.sessionId);
      expect(loaded).toEqual(mockSession);
    });

    it('should return null for non-existent session', () => {
      const loaded = loadSession('non-existent');
      expect(loaded).toBeNull();
    });
  });

  describe('getActiveSessionId', () => {
    it('should return active session ID', () => {
      saveSession(mockSession);
      const id = getActiveSessionId();
      expect(id).toBe(mockSession.sessionId);
    });

    it('should return null when no active session', () => {
      const id = getActiveSessionId();
      expect(id).toBeNull();
    });
  });

  describe('clearHistory', () => {
    it('should remove all chatkit keys from localStorage', () => {
      saveSession(mockSession);
      localStorage.setItem('other-key', 'value');

      clearHistory();

      expect(localStorage.getItem(`chatkit-session-${mockSession.sessionId}`))
        .toBeNull();
      expect(localStorage.getItem('chatkit-active-session')).toBeNull();
      expect(localStorage.getItem('other-key')).toBe('value');
    });
  });
});
