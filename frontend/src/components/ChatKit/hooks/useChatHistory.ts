/**
 * useChatHistory Hook - Session persistence and message management
 *
 * Manages chat session state and persistence to localStorage.
 * Loads existing sessions on mount, creates new sessions as needed.
 * Provides interface to add/remove messages and clear history.
 */

import { useState, useEffect, useCallback } from 'react';
import { ChatSession, ChatMessage } from '../types/chatkit.types';
import { storageService } from '../services/storageService';

interface UseChatHistoryReturn {
  /** Current active session (null until loaded) */
  session: ChatSession | null;

  /** Add a message to the current session */
  addMessage: (message: ChatMessage) => void;

  /** Clear all history and create new session */
  clearHistory: () => void;
}

/**
 * Generate a UUID v4 for session and message IDs
 * Simple implementation without external dependency
 */
function generateSessionId(): string {
  // Simple UUID v4 generator (not cryptographically secure, but sufficient for session IDs)
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Create a new empty session
 */
function createNewSession(): ChatSession {
  const now = Date.now();
  return {
    sessionId: generateSessionId(),
    createdAt: now,
    lastMessageAt: now,
    messages: [],
    isActive: true,
  };
}

/**
 * Hook for managing chat history and session persistence
 * Automatically loads existing session on mount or creates new one
 *
 * @returns Object with session, addMessage, and clearHistory functions
 */
export function useChatHistory(): UseChatHistoryReturn {
  const [session, setSession] = useState<ChatSession | null>(null);
  const [isInitialized, setIsInitialized] = useState(false);

  // Load or create session on mount
  useEffect(() => {
    const initializeSession = () => {
      // Try to load existing active session
      const existingSessionId = storageService.getActiveSessionId();
      let loadedSession: ChatSession | null = null;

      if (existingSessionId) {
        loadedSession = storageService.loadSession(existingSessionId);
      }

      // Create new session if none exists
      const activeSession = loadedSession || createNewSession();
      setSession(activeSession);
      storageService.saveSession(activeSession);
      setIsInitialized(true);
    };

    initializeSession();
  }, []);

  const addMessage = useCallback((message: ChatMessage) => {
    setSession(prevSession => {
      if (!prevSession) {
        // Shouldn't happen but handle gracefully
        console.warn('Session is null, cannot add message');
        return null;
      }

      const updatedSession: ChatSession = {
        ...prevSession,
        messages: [...prevSession.messages, message],
        lastMessageAt: message.timestamp,
      };

      // Persist to localStorage
      try {
        storageService.saveSession(updatedSession);
      } catch (error) {
        console.error('Error persisting session to localStorage', error);
      }

      return updatedSession;
    });
  }, []);

  const clearHistory = useCallback(() => {
    // Clear all localStorage entries
    storageService.clearHistory();

    // Create brand new session
    const newSession = createNewSession();
    setSession(newSession);
    storageService.saveSession(newSession);
  }, []);

  return {
    session,
    addMessage,
    clearHistory,
  };
}
