/**
 * Storage Service - localStorage persistence for chat sessions
 *
 * Handles all localStorage operations with graceful error handling.
 * Centralizes localStorage access to a single service module.
 */

import { ChatSession } from '../types/chatkit.types';

const CHATKIT_SESSION_PREFIX = 'chatkit-session-';
const CHATKIT_ACTIVE_SESSION_KEY = 'chatkit-active-session';

/**
 * Check if localStorage is available (handles SSR and disabled cases)
 */
function isLocalStorageAvailable(): boolean {
  try {
    if (typeof window === 'undefined') return false;
    const test = '__test__';
    window.localStorage.setItem(test, test);
    window.localStorage.removeItem(test);
    return true;
  } catch {
    return false;
  }
}

/**
 * Save a chat session to localStorage
 * @param session - The ChatSession object to save
 */
export function saveSession(session: ChatSession): void {
  if (!isLocalStorageAvailable()) {
    console.warn('localStorage not available, session will not persist');
    return;
  }

  try {
    const key = `${CHATKIT_SESSION_PREFIX}${session.sessionId}`;
    const serialized = JSON.stringify(session);

    // Check if we're exceeding localStorage quota (10KB limit roughly 25-50 messages)
    const estimatedSize = new Blob([serialized]).size;
    if (estimatedSize > 10 * 1024) {
      console.warn(
        'Session data approaching localStorage quota limit',
        { size: estimatedSize }
      );
    }

    window.localStorage.setItem(key, serialized);
    window.localStorage.setItem(CHATKIT_ACTIVE_SESSION_KEY, session.sessionId);
  } catch (error) {
    if (error instanceof Error && error.name === 'QuotaExceededError') {
      console.error(
        'localStorage quota exceeded, unable to save session',
        error
      );
    } else {
      console.error('Error saving session to localStorage', error);
    }
  }
}

/**
 * Load a chat session from localStorage
 * @param sessionId - The UUID of the session to load
 * @returns ChatSession object or null if not found/error
 */
export function loadSession(sessionId: string): ChatSession | null {
  if (!isLocalStorageAvailable()) {
    return null;
  }

  try {
    const key = `${CHATKIT_SESSION_PREFIX}${sessionId}`;
    const serialized = window.localStorage.getItem(key);

    if (!serialized) {
      return null;
    }

    const session = JSON.parse(serialized) as ChatSession;
    return session;
  } catch (error) {
    console.error('Error loading session from localStorage', error);
    return null;
  }
}

/**
 * Get the ID of the currently active session
 * @returns Session ID string or null if no active session
 */
export function getActiveSessionId(): string | null {
  if (!isLocalStorageAvailable()) {
    return null;
  }

  try {
    const sessionId = window.localStorage.getItem(CHATKIT_ACTIVE_SESSION_KEY);
    return sessionId;
  } catch (error) {
    console.error('Error getting active session ID', error);
    return null;
  }
}

/**
 * Clear all ChatKit history from localStorage
 * Removes all keys starting with 'chatkit-'
 */
export function clearHistory(): void {
  if (!isLocalStorageAvailable()) {
    return;
  }

  try {
    const keys: string[] = [];

    // Collect all chatkit keys first (safer than modifying during iteration)
    for (let i = 0; i < window.localStorage.length; i++) {
      const key = window.localStorage.key(i);
      if (key && key.startsWith('chatkit-')) {
        keys.push(key);
      }
    }

    // Remove all collected keys
    keys.forEach(key => {
      window.localStorage.removeItem(key);
    });
  } catch (error) {
    console.error('Error clearing chat history from localStorage', error);
  }
}

/**
 * Get the current localStorage quota usage
 * Returns object with used and available bytes
 * Useful for monitoring and debugging storage issues
 */
export function getStorageInfo(): { used: number; available: number } | null {
  if (!isLocalStorageAvailable()) {
    return null;
  }

  try {
    let totalSize = 0;

    for (let i = 0; i < window.localStorage.length; i++) {
      const key = window.localStorage.key(i);
      if (key) {
        const value = window.localStorage.getItem(key);
        if (value) {
          totalSize += key.length + value.length;
        }
      }
    }

    // Rough estimate of localStorage quota (typically 5-10MB)
    const estimatedQuota = 5 * 1024 * 1024; // 5MB
    return {
      used: totalSize,
      available: estimatedQuota - totalSize,
    };
  } catch (error) {
    console.error('Error getting storage info', error);
    return null;
  }
}

export const storageService = {
  saveSession,
  loadSession,
  getActiveSessionId,
  clearHistory,
  getStorageInfo,
};
