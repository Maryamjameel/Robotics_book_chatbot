/**
 * ChatKit Type Definitions
 * Shared types for theme and configuration integration
 */

/**
 * Theme configuration with CSS variable mappings
 */
export interface ThemeConfig {
  isDarkMode: boolean;
  cssVariables: {
    primaryColor: string;
    backgroundColor: string;
    fontColorBase: string;
    borderColor: string;
  };
}

/**
 * Chapter context information detected from current page
 */
export interface ChapterContext {
  id: string;
  title: string;
  url: string;
  confidence: 'high' | 'medium' | 'low';
}

/**
 * RAG Search result with source information
 */
export interface RAGSearchResult {
  id: string;
  content: string;
  confidence: number;
  source: {
    chapter: string;
    section?: string;
    page?: number;
  };
  metadata?: Record<string, unknown>;
}

/**
 * Chat message in the conversation
 */
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: Date;
  sources?: RAGSearchResult[];
  confidence?: number;
}

/**
 * ChatKit widget configuration
 */
export interface ChatKitConfig {
  apiEndpoint: string;
  theme: ThemeConfig;
  maxMessages: number;
  enableChapterContext: boolean;
  enableSourceDisplay: boolean;
}

/**
 * Widget state enumeration
 */
export enum ChatKitState {
  Idle = 'idle',
  Loading = 'loading',
  Streaming = 'streaming',
  Error = 'error',
  Success = 'success',
}
