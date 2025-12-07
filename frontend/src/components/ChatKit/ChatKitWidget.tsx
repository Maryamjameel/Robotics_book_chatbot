/**
 * ChatKitWidget - Main chat interface component
 *
 * Displays a fixed chatbot widget with message list, input field, and controls.
 * Integrates with hooks for backend communication, history persistence, and page context.
 * Responsive design supports mobile (320px+) and desktop screens.
 */

import React, { useState, useCallback, useRef, useEffect } from 'react';
import { ChatMessage } from './types/chatkit.types';
import { useChatHistory } from './hooks/useChatHistory';
import { usePageContext } from './hooks/usePageContext';
import { useChapterContext } from './hooks/useChapterContext';
import { useThemeContext } from './hooks/useThemeContext';
import { useRAGAPI } from './hooks/useRAGAPI';
import { chatKitConfig, constraints } from '../../config/chatkit.config';
import './styles/chatkit.css';

interface ChatKitWidgetProps {
  /** Optional CSS class for styling */
  className?: string;
}

/**
 * Generate a unique ID for messages
 * Simple UUID v4 generator for message IDs
 */
function generateMessageId(): string {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * ChatKitWidget Component
 * Fixed-position chat widget for Docusaurus documentation
 */
export function ChatKitWidget({ className }: ChatKitWidgetProps): JSX.Element {
  // State management
  const [isOpen, setIsOpen] = useState(false);
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [inputError, setInputError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Hooks
  const { session, addMessage, clearHistory } = useChatHistory();
  const pageContext = usePageContext();
  const chapterContext = useChapterContext();
  // Theme hook detects Docusaurus theme changes and provides CSS variable fallbacks
  // Styling is applied via CSS variables in chatkit.css (--chatkit-primary, --chatkit-bg, etc.)
  const theme = useThemeContext();
  const { sendQuestion, isLoading, error: apiError } = useRAGAPI();

  // Auto-scroll to latest message
  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [session?.messages.length, scrollToBottom]);

  /**
   * Listen for selection:ask events from SelectionTooltip
   * Pre-fill input with selected text when user clicks "Ask about this"
   */
  useEffect(() => {
    const handleSelectionAsk = (event: Event) => {
      const customEvent = event as CustomEvent;
      const selectedText = customEvent.detail?.selectedText;

      if (selectedText && typeof selectedText === 'string') {
        // Open the chat widget
        setIsOpen(true);

        // Pre-fill input and selected text state
        setInput(selectedText);
        setSelectedText(selectedText);

        // Focus the input field for immediate typing/submission
        setTimeout(() => {
          const inputElement = document.querySelector('.chatkit-input') as HTMLTextAreaElement;
          if (inputElement) {
            inputElement.focus();
            // Move cursor to end
            inputElement.selectionStart = inputElement.selectionEnd = inputElement.value.length;
          }
        }, 100);
      }
    };

    document.addEventListener('selection:ask', handleSelectionAsk);
    return () => {
      document.removeEventListener('selection:ask', handleSelectionAsk);
    };
  }, []);

  /**
   * Validate and send question to backend
   */
  const handleSendQuestion = useCallback(async () => {
    // Clear previous input error
    setInputError(null);

    // Validate input
    const trimmedQuestion = input.trim();
    if (!trimmedQuestion) {
      setInputError('Please enter a question');
      return;
    }

    if (trimmedQuestion.length > constraints.maxQuestionLength) {
      setInputError(
        `Question too long (max ${constraints.maxQuestionLength} characters)`
      );
      return;
    }

    // Ensure session exists
    if (!session) {
      console.error('Session is not initialized');
      return;
    }

    // Create user message
    const userMessage: ChatMessage = {
      id: generateMessageId(),
      role: 'user',
      content: trimmedQuestion,
      timestamp: Date.now(),
      sessionId: session.sessionId,
      ...(selectedText && { selectedText }),
      ...(pageContext && { pageContext }),
      status: 'sent',
    };

    // Add to chat history
    addMessage(userMessage);

    // Clear input field
    setInput('');
    setSelectedText(null);

    // Send to backend with chapter context
    const response = await sendQuestion(
      trimmedQuestion,
      selectedText || undefined,
      pageContext || undefined,
      chapterContext || undefined
    );

    // Handle response
    if (response) {
      const assistantMessage: ChatMessage = {
        id: generateMessageId(),
        role: 'assistant',
        content: response.answer,
        timestamp: Date.now(),
        sessionId: session.sessionId,
        sources: response.sources,
        confidence: response.confidence,
        status: 'received',
      };
      addMessage(assistantMessage);
    } else if (apiError) {
      // Create error message
      const errorMessage: ChatMessage = {
        id: generateMessageId(),
        role: 'assistant',
        content: apiError,
        timestamp: Date.now(),
        sessionId: session.sessionId,
        status: 'error',
        error: {
          code: 'API_ERROR',
          message: apiError,
        },
      };
      addMessage(errorMessage);
    }
  }, [input, selectedText, pageContext, session, addMessage, sendQuestion, apiError]);

  /**
   * Handle Enter key to send message
   */
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
      if (e.key === 'Enter' && !e.shiftKey && !isLoading) {
        e.preventDefault();
        handleSendQuestion();
      }
    },
    [handleSendQuestion, isLoading]
  );

  /**
   * Render chapter context badge
   * Displays current chapter name and extraction confidence
   */
  const renderChapterBadge = () => {
    if (!chapterContext) {
      return null;
    }

    // Map confidence to color and label
    const confidenceStyles: Record<'high' | 'medium' | 'low', { color: string; icon: string }> = {
      high: { color: '#10b981', icon: '✓' },      // Green
      medium: { color: '#f59e0b', icon: '◐' },    // Amber
      low: { color: '#ef4444', icon: '?' },       // Red
    };

    const style = confidenceStyles[chapterContext.confidence];

    return (
      <div className="chatkit-chapter-badge" style={{ borderLeftColor: style.color }}>
        <span className="chapter-badge-icon" style={{ color: style.color }}>
          {style.icon}
        </span>
        <span className="chapter-badge-text">
          Chapter: <strong>{chapterContext.chapterTitle}</strong>
        </span>
        <span className="chapter-badge-confidence" style={{ color: style.color }}>
          ({chapterContext.confidence})
        </span>
      </div>
    );
  };

  /**
   * Render a single chat message
   */
  const renderMessage = (msg: ChatMessage) => {
    const isUser = msg.role === 'user';

    return (
      <div key={msg.id} className={`chatkit-message message-${msg.role}`}>
        <div className="message-content">
          {msg.content}

          {/* Display confidence warning if low */}
          {msg.confidence !== undefined && msg.confidence < constraints.uncertaintyThreshold && (
            <div className="message-uncertainty">
              ⚠️ Low confidence answer
            </div>
          )}
        </div>

        {/* Display sources for assistant messages */}
        {msg.role === 'assistant' && msg.sources && msg.sources.length > 0 && (
          <div className="message-sources">
            <div className="sources-label">Sources:</div>
            <div className="sources-list">
              {msg.sources.slice(0, constraints.maxSourcesPerResponse).map((source, idx) => (
                <div
                  key={source.id || idx}
                  className="source-link"
                  title={`${source.chapter_title}, Section ${source.section_number} - ${source.section_title} (${(source.relevance_score * 100).toFixed(0)}% match)`}
                >
                  {source.chapter_title}, Section {source.section_number}: {source.section_title}
                  <span className="source-relevance"> ({(source.relevance_score * 100).toFixed(0)}%)</span>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Display error details */}
        {msg.status === 'error' && msg.error && (
          <div className="message-error">
            Error: {msg.error.message}
          </div>
        )}
      </div>
    );
  };

  // Toggle chat open/closed
  const toggleChat = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  // Don't render if session not loaded yet
  if (!session) {
    return (
      <div className="chatkit-container">
        <button
          className="chatkit-toggle-button"
          onClick={toggleChat}
          aria-label="Open chat"
          title="Chat with AI Assistant"
        >
          💬
        </button>
      </div>
    );
  }

  return (
    <div className={`chatkit-container ${className || ''}`}>
      {/* Toggle Button - Always visible */}
      <button
        className={`chatkit-toggle-button ${isOpen ? 'chatkit-toggle-open' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
        title={isOpen ? 'Close chat' : 'Chat with AI Assistant'}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Widget - Only visible when open */}
      {isOpen && (
        <div className="chatkit-widget">
          {/* Header */}
          <div className="chatkit-header">
            <span className="chatkit-header-title">AI Assistant</span>
            <button
              className="chatkit-close-button"
              onClick={toggleChat}
              aria-label="Close chat"
              title="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Chapter Context Badge */}
          {renderChapterBadge()}

          {/* Messages Container */}
          <div className="chatkit-messages">
            {session.messages.length === 0 ? (
              <div className="chatkit-empty">
                <div className="empty-icon">💬</div>
                <div className="empty-text">Ask me anything about the documentation</div>
              </div>
            ) : (
              <>
                {session.messages.map(renderMessage)}
                <div ref={messagesEndRef} />
              </>
            )}

            {isLoading && (
              <div className="chatkit-loading-indicator">
                <div className="loading-spinner"></div>
                <span>Thinking...</span>
              </div>
            )}
          </div>

          {/* Input Error Display */}
          {inputError && (
            <div className="chatkit-input-error">
              ⚠️ {inputError}
            </div>
          )}

          {/* Input Container */}
          <div className="chatkit-input-container">
            <textarea
              className="chatkit-input"
              value={input}
              onChange={e => {
                setInput(e.target.value);
                setInputError(null);
              }}
              onKeyDown={handleKeyDown}
              placeholder="Ask a question... (Shift+Enter for newline)"
              disabled={isLoading}
              maxLength={constraints.maxQuestionLength}
            />

            <div className="chatkit-controls">
              <button
                className="chatkit-button chatkit-send-button"
                onClick={handleSendQuestion}
                disabled={isLoading || !input.trim()}
                title="Send question (Enter)"
                aria-label="Send question"
              >
                {isLoading ? '⏳' : '→'}
              </button>

              <button
                className="chatkit-button chatkit-clear-button"
                onClick={clearHistory}
                disabled={isLoading || session.messages.length === 0}
                title="Clear chat history"
                aria-label="Clear chat history"
              >
                🗑️
              </button>
            </div>
          </div>

          {/* Character count indicator */}
          <div className="chatkit-char-count">
            {input.length} / {constraints.maxQuestionLength}
          </div>
        </div>
      )}
    </div>
  );
}
