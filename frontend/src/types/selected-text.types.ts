/**
 * Selected Text Context Types
 * Defines TypeScript interfaces for text selection detection and tooltip management
 */

/**
 * Represents a text selection made by the user on the page
 */
export interface TextSelection {
  /** The actual selected text (max 500 chars) */
  text: string;

  /** Left coordinate relative to viewport (pixels) */
  x: number;

  /** Top coordinate relative to viewport (pixels) */
  y: number;

  /** Timestamp when selection was made (milliseconds since epoch) */
  timestamp: number;
}

/**
 * Represents the state and positioning of the selection tooltip
 */
export interface SelectionTooltipState {
  /** Whether tooltip is currently visible */
  isVisible: boolean;

  /** The current text selection */
  selection: TextSelection | null;

  /** Whether user has dismissed the tooltip */
  isDismissed: boolean;

  /** Tooltip position coordinates */
  position: {
    x: number;
    y: number;
  };
}

/**
 * Represents coordinates for tooltip positioning
 */
export interface SelectionCoordinates {
  /** Horizontal position (pixels from left) */
  x: number;

  /** Vertical position (pixels from top) */
  y: number;

  /** Width of the tooltip (optional, for collision detection) */
  width?: number;

  /** Height of the tooltip (optional, for collision detection) */
  height?: number;
}

/**
 * Callback function type for when user clicks "Ask about this" button
 */
export type OnAskCallback = (selectedText: string) => void;

/**
 * Callback function type for when user dismisses the tooltip
 */
export type OnDismissCallback = () => void;

/**
 * Props for SelectionTooltip component
 */
export interface SelectionTooltipProps {
  /** Whether tooltip should be visible */
  isVisible: boolean;

  /** Position of the tooltip on screen */
  position: SelectionCoordinates;

  /** Selected text to display */
  selectedText: string;

  /** Callback when "Ask about this" button is clicked */
  onAsk: OnAskCallback;

  /** Callback when tooltip is dismissed */
  onDismiss: OnDismissCallback;

  /** Optional CSS class name for styling */
  className?: string;

  /** Z-index for tooltip (default 1000) */
  zIndex?: number;
}

/**
 * Hook return type for useTextSelection
 */
export interface UseTextSelectionReturn {
  /** Current text selection, or null if nothing selected */
  selection: TextSelection | null;

  /** Whether selection detection is active */
  isActive: boolean;

  /** Any errors from selection detection */
  error: Error | null;
}

/**
 * Hook return type for useSelectionTooltip
 */
export interface UseSelectionTooltipReturn {
  /** Current tooltip state */
  state: SelectionTooltipState;

  /** Show tooltip at given coordinates */
  show: (selection: TextSelection) => void;

  /** Hide tooltip */
  hide: () => void;

  /** Dismiss tooltip (prevents reappearing) */
  dismiss: () => void;

  /** Reset dismissed state (allow tooltip to reappear) */
  reset: () => void;
}

/**
 * Props for ChatKitWidget with selected text support
 */
export interface ChatKitWidgetProps {
  /** Optional pre-filled selected text */
  selectedText?: string;

  /** Callback when selected text is cleared */
  onClearSelectedText?: () => void;

  /** Other ChatKit props... */
  [key: string]: any;
}

/**
 * RAG Request with selected text context
 */
export interface RAGRequest {
  /** User's question (1-2000 chars) */
  question: string;

  /** Optional selected text from page (0-500 chars) */
  selected_text?: string;

  /** Optional page context where question was asked */
  pageContext?: {
    url?: string;
    pathname?: string;
    chapter?: string;
    section?: string;
    confidence?: 'high' | 'medium' | 'low';
  };

  /** Optional session ID for tracking conversation */
  sessionId?: string;
}

/**
 * RAG Response with boosting metadata
 */
export interface RAGResponse {
  /** LLM-generated answer */
  answer: string;

  /** Retrieved source chunks */
  sources: Array<{
    id: string;
    title: string;
    snippet: string;
    url: string;
    similarity: number;
  }>;

  /** Overall confidence in answer (0-1) */
  confidence: number;

  /** Operational metrics and search boosting information */
  metadata: {
    searchLatencyMs: number;
    generationLatencyMs: number;
    totalLatencyMs: number;
    chunksRetrieved?: number;
    chunksUsed?: number;
    model?: string;
    tokensUsed?: number;

    /** Whether selected_text parameter was used for boosting */
    selectedTextBoosted?: boolean;

    /** Terms extracted from selected_text used for boosting */
    selectedTextTerms?: string[];

    /** Multiplier applied to cosine similarity scores */
    boostFactor?: number;

    /** Term frequency scores for selected_text terms */
    termFrequency?: Record<string, number>;
  };

  /** Error information if request failed */
  error?: {
    code: string;
    message: string;
  };
}

/**
 * Search result with boosting information
 */
export interface SearchResult {
  /** Unique chunk ID from vector database */
  chunk_id: string;

  /** Text content of the chunk */
  text: string;

  /** Original cosine similarity score (0-1) */
  original_score: number;

  /** Boosted score after applying selected text weighting */
  boosted_score?: number;

  /** Term frequency score for selected text terms */
  tf_score?: number;

  /** Title of the source */
  title?: string;

  /** URL to the source */
  url?: string;
}
