/**
 * ChatKit TypeScript Types and Interfaces
 *
 * Central type definitions for all ChatKit entities and API contracts.
 * All types are derived from the data model specification.
 */

/**
 * Chat Message - represents a single message in a conversation
 * Can be from user or assistant
 */
export interface ChatMessage {
  /** Unique identifier (UUID v4) */
  id: string;

  /** Message sender: 'user' for questions, 'assistant' for answers */
  role: 'user' | 'assistant';

  /** Message content text */
  content: string;

  /** Unix timestamp in milliseconds when message was created */
  timestamp: number;

  /** Session ID this message belongs to */
  sessionId: string;

  /** Selected text context from the page (optional, P3 feature) */
  selectedText?: string;

  /** Page context where question was asked (optional) */
  pageContext?: PageContext;

  /** Retrieved source chunks from vector database (assistant messages only) */
  sources?: SourceReference[];

  /** Confidence score 0-1 for assistant responses */
  confidence?: number;

  /** Message delivery status */
  status: 'pending' | 'sent' | 'received' | 'error';

  /** Error information if status is 'error' */
  error?: {
    code: string;
    message: string;
  };
}

/**
 * Chat Session - container for a conversation
 * Groups related messages and persists to localStorage
 */
export interface ChatSession {
  /** Unique session identifier (UUID v4) */
  sessionId: string;

  /** When session was created (Unix timestamp) */
  createdAt: number;

  /** When last message was added (Unix timestamp) */
  lastMessageAt: number;

  /** All messages in this session */
  messages: ChatMessage[];

  /** Whether this session is currently active */
  isActive: boolean;
}

/**
 * Page Context - metadata about the current Docusaurus page
 * Used to provide contextual answers from the documentation
 */
export interface PageContext {
  /** Full page URL including protocol and host */
  url: string;

  /** URL pathname without domain (e.g., /docs/chapter-3) */
  pathname: string;

  /** Extracted chapter title if available */
  chapter?: string;

  /** Extracted section title if available */
  section?: string;

  /** Confidence in extraction: 'high' if from breadcrumb, 'medium' if from URL, 'low' if guessed */
  confidence: 'high' | 'medium' | 'low';
}

/**
 * RAG Request - payload sent to backend /api/v1/chat/ask endpoint
 * Conforms to chat-request.schema.json
 */
export interface RAGRequest {
  /** User question (1-2000 characters) */
  question: string;

  /** Optional selected text from the page context */
  selectedText?: string;

  /** Optional page context where question was asked */
  pageContext?: PageContext;

  /** Optional session ID for tracking conversation */
  sessionId?: string;
}

/**
 * RAG Response - payload returned from backend /api/v1/chat/ask endpoint
 * Conforms to chat-response.schema.json
 */
export interface RAGResponse {
  /** LLM-generated answer with optional citations */
  answer: string;

  /** Retrieved source chunks with metadata */
  sources: SourceReference[];

  /** Confidence score 0-1: <0.8 means uncertain */
  confidence: number;

  /** Operational metrics for the response */
  metadata: ResponseMetadata;

  /** Error information (only if request failed) */
  error?: {
    code: string;
    message: string;
  };
}

/**
 * Source Reference - individual chunk retrieved from vector database
 * Includes snippet text and URL for clicking through to full source
 */
export interface SourceReference {
  /** Unique chunk ID from vector database */
  id: string;

  /** Source title, e.g., "Chapter 3, Section 3.2 - Forward Kinematics" */
  title: string;

  /** Relevant excerpt (200-400 characters) */
  snippet: string;

  /** Clickable URL to the source document */
  url: string;

  /** Cosine similarity score 0-1 (higher = more relevant) */
  similarity: number;
}

/**
 * Response Metadata - operational metrics for debugging and monitoring
 * Captured from backend to understand performance characteristics
 */
export interface ResponseMetadata {
  /** Time spent on vector search in milliseconds */
  searchLatencyMs: number;

  /** Time spent on LLM generation in milliseconds */
  generationLatencyMs: number;

  /** Total end-to-end latency in milliseconds */
  totalLatencyMs: number;

  /** Number of chunks retrieved from vector database */
  chunksRetrieved: number;

  /** Number of chunks passed to LLM context window */
  chunksUsed: number;

  /** LLM model name used for generation */
  model: string;

  /** Approximate total tokens used (optional) */
  tokensUsed?: number;
}

/**
 * ChatKit Widget Configuration
 * Matches configuration structure from chatkit.config.ts
 */
export interface ChatKitWidgetConfig {
  /** Backend RAG API endpoint URL */
  apiEndpoint: string;

  /** Widget positioning on page */
  position: 'bottom-right' | 'bottom-left' | 'top-right' | 'top-left';

  /** Auto-open widget on page load */
  defaultOpen: boolean;

  /** Enable dark mode styling */
  darkMode: boolean;

  /** Allow selected text context in questions */
  enableSelectedText: boolean;

  /** Maximum messages to keep in history */
  maxHistoryMessages: number;

  /** Request timeout in milliseconds */
  requestTimeoutMs: number;
}

/**
 * Service Error - standardized error type for API failures
 * Provides consistent error handling across services
 */
export interface ServiceError {
  code: 'NETWORK_ERROR' | 'TIMEOUT' | 'PARSE_ERROR' | 'SERVER_ERROR' | 'UNKNOWN';
  message: string;
  originalError?: Error;
}
