/**
 * ChatKit Widget Configuration
 *
 * Central configuration for ChatKit Docusaurus integration.
 * All settings can be overridden via environment variables.
 */

export interface ChatKitConfig {
  apiEndpoint: string;
  position: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';
  defaultOpen: boolean;
  darkMode: boolean;
  enableSelectedText: boolean;
  maxHistoryMessages: number;
  requestTimeoutMs: number;
}

export const chatKitConfig: ChatKitConfig = {
  // Backend RAG API endpoint
  // Override via: REACT_APP_RAG_API_URL environment variable
  apiEndpoint:
    process.env.REACT_APP_RAG_API_URL ||
    'http://localhost:8000/api/v1/chat/ask',

  // Widget positioning on the page
  position: 'bottom-right',

  // Whether widget opens automatically on page load
  defaultOpen: false,

  // Enable dark mode styling (will auto-sync with Docusaurus theme)
  darkMode: false,

  // Allow users to select text and include it in questions
  enableSelectedText: true,

  // Maximum chat history messages to keep in localStorage (FR-009)
  maxHistoryMessages: 100,

  // Request timeout in milliseconds (FR-010: 30 seconds)
  requestTimeoutMs: 30000,
};

/**
 * Feature flags for gradual rollout
 */
export const featureFlags = {
  // Enable page context detection and passing to backend (FR-005, FR-006)
  enablePageContext: true,

  // Enable selected text context passing (FR-011, FR-012)
  enableSelectedTextContext: true,

  // Enable right-click context menu for quick questions (P3 feature)
  enableContextMenu: false,

  // Enable response citations and source highlighting (FR-007)
  enableSourceCitations: true,

  // Enable message persistence via localStorage (FR-009)
  enableHistoryPersistence: true,
};

/**
 * Validation constraints (from spec and contracts)
 */
export const constraints = {
  // Maximum question length in characters (per spec)
  maxQuestionLength: 2000,

  // Minimum localStorage quota (10KB estimate for 20-25 messages)
  minLocalStorageKB: 10,

  // Maximum sources to display per response
  maxSourcesPerResponse: 10,

  // Confidence threshold for "uncertain" flag display
  uncertaintyThreshold: 0.8,
};
