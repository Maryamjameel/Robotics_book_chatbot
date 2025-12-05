# ChatKit Docusaurus Integration - Developer Quickstart

**Feature**: ChatKit Docusaurus Integration (005-chatkit-integration)
**Date**: 2025-12-06
**Status**: Design Document (Pre-Implementation)

## Quick Summary

This guide helps developers understand the ChatKit integration architecture and get started with implementation.

**What**: Integrate ChatKit SDK into Docusaurus to embed a RAG-powered chatbot widget in the documentation

**Where**: `frontend/src/components/ChatKit/` (new component directory)

**Integration Point**: `frontend/src/theme/Root.tsx` (wrap app with ChatKitProvider)

**Backend**: Existing RAG API at `http://localhost:8000/api/v1/chat/ask` (feature 004)

## Setup & Installation

### 1. Install Dependencies

```bash
cd frontend
npm install @anthropic/chatkit
npm install --save-dev vitest @testing-library/react @testing-library/user-event
```

### 2. Verify TypeScript Configuration

Ensure `tsconfig.json` has strict mode enabled:

```json
{
  "compilerOptions": {
    "strict": true,
    "noImplicitAny": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true
  }
}
```

### 3. Create ChatKit Config

File: `frontend/src/config/chatkit.config.ts`

```typescript
export const chatKitConfig = {
  apiEndpoint: process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000/api/v1/chat/ask',
  position: 'bottom-right' as const,
  defaultOpen: false,
  darkMode: false,
  enableSelectedText: true,
  maxHistoryMessages: 100,
};
```

### 4. Create Widget Component Structure

```bash
mkdir -p frontend/src/components/ChatKit/{hooks,services,types,styles,__tests__}
```

## Architecture Overview

### Component Hierarchy

```
Root.tsx (wraps entire app with ChatKitProvider)
  ├── ChatKitProvider
  │   └── ChatKitWidget
  │       ├── MessageList (displays messages)
  │       ├── InputBox (user question input)
  │       ├── SourceLinks (displays retrieved sources)
  │       └── ErrorBoundary (handles errors)
```

### State Management

```
localStorage
  ├── chatkit-session-{uuid}     ← ChatSession object (persistent across page reload)
  └── chatkit-active-session      ← Current session UUID

React Context (session state)
  ├── currentSessionId
  ├── messages[]
  ├── isLoading
  └── error?
```

### Data Flow

```
User Input
   ↓
InputBox Component
   ↓
useRAGAPI Hook (handles API call)
   ↓
apiService.sendQuestion()
   ↓
Fetch POST /api/v1/chat/ask
   ↓
Backend RAG Pipeline
   ↓
RAGResponse (JSON)
   ↓
useChatHistory Hook (saves to localStorage)
   ↓
MessageList Component (renders new message + sources)
   ↓
User sees answer with clickable source links
```

## Core Files to Implement

### 1. Types File
**File**: `frontend/src/components/ChatKit/types/chatkit.types.ts`

Define all TypeScript interfaces matching data-model.md:
- `ChatMessage`
- `ChatSession`
- `PageContext`
- `SourceReference`
- `RAGRequest`
- `RAGResponse`

```typescript
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
  sessionId: string;
  selectedText?: string;
  pageContext?: PageContext;
  sources?: SourceReference[];
  confidence?: number;
  status: 'pending' | 'sent' | 'received' | 'error';
  error?: { code: string; message: string };
}

export interface ChatSession {
  sessionId: string;
  createdAt: number;
  lastMessageAt: number;
  messages: ChatMessage[];
  isActive: boolean;
}

// ... additional interfaces
```

### 2. Storage Service
**File**: `frontend/src/components/ChatKit/services/storageService.ts`

Implements localStorage persistence:

```typescript
export const storageService = {
  saveSession(session: ChatSession): void {
    localStorage.setItem(`chatkit-session-${session.sessionId}`, JSON.stringify(session));
    localStorage.setItem('chatkit-active-session', session.sessionId);
  },

  loadSession(sessionId: string): ChatSession | null {
    const data = localStorage.getItem(`chatkit-session-${sessionId}`);
    return data ? JSON.parse(data) : null;
  },

  getActiveSessionId(): string | null {
    return localStorage.getItem('chatkit-active-session');
  },

  clearHistory(): void {
    Object.keys(localStorage)
      .filter(key => key.startsWith('chatkit-'))
      .forEach(key => localStorage.removeItem(key));
  },
};
```

### 3. API Service
**File**: `frontend/src/components/ChatKit/services/apiService.ts`

Handles backend communication with timeout and error handling:

```typescript
export const apiService = {
  async sendQuestion(request: RAGRequest, timeoutMs = 30000): Promise<RAGResponse> {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), timeoutMs);

    try {
      const response = await fetch(chatKitConfig.apiEndpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
        signal: controller.signal,
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      if (error instanceof DOMException && error.name === 'AbortError') {
        throw new Error('Request timeout - please try again');
      }
      throw error;
    } finally {
      clearTimeout(timeoutId);
    }
  },
};
```

### 4. Page Context Service
**File**: `frontend/src/components/ChatKit/services/pageContextService.ts`

Extracts chapter/section from Docusaurus:

```typescript
export const pageContextService = {
  getPageContext(): PageContext {
    // Extract from Docusaurus location object
    const url = window.location.href;
    const pathname = window.location.pathname;

    // Query sidebar for chapter/section (Docusaurus-specific)
    // Implementation depends on Docusaurus structure

    return {
      url,
      pathname,
      chapter: extractedChapter,
      section: extractedSection,
      confidence: 'high' | 'medium' | 'low',
    };
  },
};
```

### 5. Hooks

#### `useChatHistory` Hook
**File**: `frontend/src/components/ChatKit/hooks/useChatHistory.ts`

Manages chat session persistence:

```typescript
export function useChatHistory() {
  const [session, setSession] = useState<ChatSession | null>(null);

  useEffect(() => {
    // Load active session on mount
    const sessionId = storageService.getActiveSessionId();
    if (sessionId) {
      const loaded = storageService.loadSession(sessionId);
      setSession(loaded);
    } else {
      // Create new session
      const newSession = createNewSession();
      setSession(newSession);
    }
  }, []);

  const addMessage = (message: ChatMessage) => {
    setSession(prev => {
      if (!prev) return null;
      const updated = {
        ...prev,
        messages: [...prev.messages, message],
        lastMessageAt: message.timestamp,
      };
      storageService.saveSession(updated);
      return updated;
    });
  };

  const clearHistory = () => {
    storageService.clearHistory();
    setSession(createNewSession());
  };

  return { session, addMessage, clearHistory };
}
```

#### `usePageContext` Hook
**File**: `frontend/src/components/ChatKit/hooks/usePageContext.ts`

Tracks current page:

```typescript
export function usePageContext() {
  const [context, setContext] = useState<PageContext | null>(null);

  useEffect(() => {
    const context = pageContextService.getPageContext();
    setContext(context);
  }, [window.location.pathname]);

  return context;
}
```

#### `useRAGAPI` Hook
**File**: `frontend/src/components/ChatKit/hooks/useRAGAPI.ts`

Handles API communication:

```typescript
export function useRAGAPI() {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendQuestion = async (
    question: string,
    selectedText?: string,
    pageContext?: PageContext
  ): Promise<RAGResponse | null> => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await apiService.sendQuestion({
        question,
        selectedText,
        pageContext,
      });
      return response;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      return null;
    } finally {
      setIsLoading(false);
    }
  };

  return { sendQuestion, isLoading, error };
}
```

### 6. Main Widget Component
**File**: `frontend/src/components/ChatKit/ChatKitWidget.tsx`

Main component rendering the chat interface:

```typescript
export function ChatKitWidget() {
  const { session, addMessage, clearHistory } = useChatHistory();
  const pageContext = usePageContext();
  const { sendQuestion, isLoading, error } = useRAGAPI();
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState<string | null>(null);

  const handleSendQuestion = async () => {
    if (!input.trim()) return;

    // Add user message
    const userMessage: ChatMessage = {
      id: generateUUID(),
      role: 'user',
      content: input,
      timestamp: Date.now(),
      sessionId: session!.sessionId,
      selectedText: selectedText || undefined,
      pageContext: pageContext || undefined,
      status: 'sent',
    };
    addMessage(userMessage);
    setInput('');
    setSelectedText(null);

    // Get response from backend
    const response = await sendQuestion(input, selectedText || undefined, pageContext || undefined);

    if (response) {
      const assistantMessage: ChatMessage = {
        id: generateUUID(),
        role: 'assistant',
        content: response.answer,
        timestamp: Date.now(),
        sessionId: session!.sessionId,
        sources: response.sources,
        confidence: response.confidence,
        status: 'received',
      };
      addMessage(assistantMessage);
    } else {
      // Error message
      const errorMessage: ChatMessage = {
        id: generateUUID(),
        role: 'assistant',
        content: 'Sorry, I encountered an error while processing your question.',
        timestamp: Date.now(),
        sessionId: session!.sessionId,
        status: 'error',
        error: { code: 'ERROR', message: error || 'Unknown error' },
      };
      addMessage(errorMessage);
    }
  };

  return (
    <div className="chatkit-widget">
      {/* Message list */}
      <div className="messages">
        {session?.messages.map(msg => (
          <div key={msg.id} className={`message message-${msg.role}`}>
            {msg.content}
            {msg.role === 'assistant' && msg.sources && (
              <div className="sources">
                {msg.sources.map(source => (
                  <a key={source.id} href={source.url} target="_blank" rel="noopener noreferrer">
                    {source.title}
                  </a>
                ))}
              </div>
            )}
          </div>
        ))}
      </div>

      {/* Input area */}
      <input
        type="text"
        value={input}
        onChange={e => setInput(e.target.value)}
        placeholder="Ask a question..."
        disabled={isLoading}
      />
      <button onClick={handleSendQuestion} disabled={isLoading || !input.trim()}>
        {isLoading ? 'Sending...' : 'Send'}
      </button>

      {/* Clear history button */}
      <button onClick={clearHistory}>Clear History</button>
    </div>
  );
}
```

### 7. Provider Component
**File**: `frontend/src/components/ChatKit/ChatKitProvider.tsx`

Wraps app with ChatKit context:

```typescript
export const ChatKitContext = createContext<ChatKitContextType | null>(null);

export function ChatKitProvider({ children }: { children: React.ReactNode }) {
  return (
    <ChatKitContext.Provider value={{}}>
      {children}
      <ChatKitWidget />
    </ChatKitContext.Provider>
  );
}
```

### 8. Root Integration
**File**: `frontend/src/theme/Root.tsx` (MODIFY)

```typescript
import { ChatKitProvider } from '@site/src/components/ChatKit/ChatKitProvider';

export default function Root({ children }) {
  return (
    <ChatKitProvider>
      {children}
    </ChatKitProvider>
  );
}
```

### 9. Styling
**File**: `frontend/src/components/ChatKit/styles/chatkit.css`

Responsive styles with dark mode support:

```css
:root {
  --chatkit-primary: #0066cc;
  --chatkit-bg: #ffffff;
  --chatkit-text: #333333;
  --chatkit-border: #cccccc;
}

[data-theme='dark'] {
  --chatkit-primary: #3399ff;
  --chatkit-bg: #1a1a1a;
  --chatkit-text: #ffffff;
  --chatkit-border: #444444;
}

.chatkit-widget {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 350px;
  max-height: 500px;
  background: var(--chatkit-bg);
  border: 1px solid var(--chatkit-border);
  border-radius: 8px;
  box-shadow: 0 2px 12px rgba(0, 0, 0, 0.15);
  display: flex;
  flex-direction: column;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto;
  z-index: 1000;
}

@media (max-width: 640px) {
  .chatkit-widget {
    width: 100%;
    max-width: 100%;
    bottom: 0;
    right: 0;
    border-radius: 0;
  }
}
```

## Testing Strategy

### Unit Tests (Vitest)
- Services (storage, API, pageContext)
- Utility functions (UUID generation, validation)
- Hook logic (state updates, side effects)

### Component Tests (React Testing Library)
- Input validation and error display
- Message rendering
- Source link display
- Button interactions

### E2E Tests (Playwright)
- Full user flow: open widget → ask question → see answer
- Dark mode toggle
- Mobile responsiveness
- Backend error scenarios

## Development Checklist

- [ ] Install dependencies (@anthropic/chatkit, vitest, testing-library)
- [ ] Verify TypeScript strict mode
- [ ] Create config file (chatkit.config.ts)
- [ ] Create component directory structure
- [ ] Implement types (chatkit.types.ts)
- [ ] Implement services (storage, API, pageContext)
- [ ] Implement hooks (useChatHistory, usePageContext, useRAGAPI)
- [ ] Implement ChatKitWidget component
- [ ] Implement ChatKitProvider
- [ ] Modify Root.tsx to wrap app with ChatKitProvider
- [ ] Create styles with dark mode support
- [ ] Create unit tests for services and hooks
- [ ] Create component tests for Widget
- [ ] Create E2E tests for user flows
- [ ] Test with backend RAG API running
- [ ] Verify dark mode toggle
- [ ] Test on mobile (320px+ width)
- [ ] Run accessibility checks (axe)
- [ ] Document any deviations from plan

## Common Issues & Solutions

### Issue: "localStorage is not defined" (SSR)
**Solution**: Wrap localStorage calls in `typeof window !== 'undefined'` check

### Issue: "ChatKit context not available"
**Solution**: Ensure Root.tsx wraps entire app with ChatKitProvider

### Issue: "Backend API timeout"
**Solution**: Check REACT_APP_RAG_API_URL env variable, ensure backend is running

### Issue: "Messages not persisting"
**Solution**: Verify localStorage is not disabled, check browser console for quota errors

## Next Steps After Implementation

1. Run full test suite: `npm run test`
2. Build Docusaurus: `npm run build`
3. Deploy to staging
4. Perform manual E2E testing
5. Gather user feedback
6. Iterate on P2/P3 features if needed

## References

- **Specification**: `specs/005-chatkit-integration/spec.md`
- **Data Model**: `specs/005-chatkit-integration/data-model.md`
- **API Contracts**: `specs/005-chatkit-integration/contracts/`
- **Implementation Plan**: `specs/005-chatkit-integration/plan.md`
- **RAG API Backend**: `backend/README.md` (feature 004)
- **Docusaurus Docs**: https://docusaurus.io/docs
- **React Testing Library**: https://testing-library.com/docs/react-testing-library/intro
- **Playwright**: https://playwright.dev
