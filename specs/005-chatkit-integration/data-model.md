# Data Model: ChatKit Docusaurus Integration

**Feature**: ChatKit Docusaurus Integration (005-chatkit-integration)
**Date**: 2025-12-06
**Status**: Design Document

## Entity Definitions

### ChatMessage

Represents a single message in the conversation (user question or bot response).

```typescript
interface ChatMessage {
  // Identification
  id: string;                           // UUID, auto-generated

  // Content
  role: 'user' | 'assistant';           // Who sent the message
  content: string;                      // Message text (1-2000 chars for user)

  // Metadata
  timestamp: number;                    // Unix timestamp (ms)
  sessionId: string;                    // Reference to parent ChatSession

  // User message specific
  selectedText?: string;                // Text highlighted by user (optional)
  pageContext?: PageContext;            // Page metadata at time of question

  // Assistant message specific
  sources?: SourceReference[];          // Retrieved chunks with links
  confidence?: number;                  // 0-1 confidence score
  metadata?: ResponseMetadata;          // Latency, token counts, etc.

  // State
  status: 'pending' | 'sent' | 'received' | 'error';  // Message delivery state
  error?: {
    code: string;                       // e.g., 'BACKEND_TIMEOUT', 'NETWORK_ERROR'
    message: string;                    // User-friendly error message
  };
}
```

**Validation Rules**:
- `id`: Must be UUID v4 format
- `content`: 1-2000 characters for user messages, unlimited for assistant
- `role`: Only two allowed values
- `timestamp`: Must be valid Unix timestamp (ms) in reasonable range
- `selectedText`: If provided, must be non-empty string
- `confidence`: If provided, must be number between 0 and 1
- `status`: Must be one of four allowed values

**Storage**: localStorage as JSON string, serialized as complete ChatSession

### ChatSession

Container for a conversation with multiple messages.

```typescript
interface ChatSession {
  // Identification
  sessionId: string;                    // UUID, auto-generated
  createdAt: number;                    // Unix timestamp (ms)
  lastMessageAt: number;                // Unix timestamp of last message

  // Content
  messages: ChatMessage[];              // Array of messages in order

  // State
  isActive: boolean;                    // True if session open, false if user closed
  userAgent?: string;                   // Browser user agent (optional, for debugging)
}
```

**Validation Rules**:
- `sessionId`: Must be UUID v4 format
- `createdAt`: Must be valid Unix timestamp (ms)
- `lastMessageAt`: Must be >= createdAt
- `messages`: Array of valid ChatMessage objects, ordered by timestamp

**Storage**: localStorage key format: `chatkit-session-{sessionId}`

### PageContext

Metadata about the page where the question was asked.

```typescript
interface PageContext {
  // Page identification
  url: string;                          // Full page URL
  pathname: string;                     // Path component of URL (e.g., /docs/chapter-1)

  // Docusaurus-specific metadata
  chapter?: string;                     // Chapter title (e.g., "Chapter 1 - Introduction")
  section?: string;                     // Section title (e.g., "1.1 Overview")
  docId?: string;                       // Internal Docusaurus doc ID

  // Detection confidence
  confidence: 'high' | 'medium' | 'low'; // How confident we are in the extraction
}
```

**Validation Rules**:
- `url`: Must be valid URL
- `pathname`: Must start with /
- `confidence`: Must be one of three values
- At least `url` must be present (required)

**Extraction Strategy**:
- Extract from Docusaurus location.pathname + window.location.href
- Query sidebar tree for chapter/section matching current doc
- If extraction fails, set confidence='low' with partial data

### RAGRequest

Request payload sent to backend RAG endpoint.

```typescript
interface RAGRequest {
  // Required
  question: string;                     // User's question (1-2000 chars)

  // Optional context
  selectedText?: string;                // Highlighted text from page
  pageContext?: PageContext;            // Chapter/section metadata

  // Session tracking
  sessionId?: string;                   // For correlating related questions
}
```

**Validation Rules**:
- `question`: 1-2000 characters, non-empty
- `selectedText`: Non-empty if provided
- All fields are optional except `question`

**Serialization**: JSON POST body to `/api/v1/chat/ask`

**Example**:
```json
{
  "question": "What is forward kinematics?",
  "selectedText": "The forward kinematics problem is to determine...",
  "pageContext": {
    "url": "http://localhost:3000/docs/chapter-1-intro",
    "pathname": "/docs/chapter-1-intro",
    "chapter": "Chapter 1 - Introduction to ROS 2",
    "section": "1.2 Kinematics Fundamentals",
    "confidence": "high"
  },
  "sessionId": "550e8400-e29b-41d4-a716-446655440000"
}
```

### RAGResponse

Response from backend RAG endpoint.

```typescript
interface RAGResponse {
  // Generated answer
  answer: string;                       // LLM-generated answer text

  // Source attribution
  sources: SourceReference[];           // Array of retrieved chunks

  // Quality indicators
  confidence: number;                   // 0-1 confidence score

  // Operational metrics
  metadata: ResponseMetadata;
}

interface SourceReference {
  // Identification
  id: string;                           // Chunk ID from vector DB

  // Content
  title: string;                        // Chapter/section title
  snippet: string;                      // Relevant text excerpt (200-400 chars)

  // Navigation
  url: string;                          // Link to relevant documentation section

  // Relevance
  similarity: number;                   // 0-1 cosine similarity score
}

interface ResponseMetadata {
  // Timing
  searchLatencyMs: number;              // Time for vector search
  generationLatencyMs: number;          // Time for LLM generation
  totalLatencyMs: number;               // Total request time

  // Operational
  chunksRetrieved: number;              // Number of chunks found
  chunksUsed: number;                   // Number passed to LLM
  model: string;                        // Which LLM generated answer
  tokensUsed?: number;                  // Approximate tokens (if available)
}
```

**Validation Rules**:
- `answer`: Non-empty string
- `sources`: Array of valid SourceReference objects
- `confidence`: 0-1 number
- `similarity` in sources: 0-1 number
- All metadata fields present and >= 0

**Deserialization**: Parsed from JSON response of `/api/v1/chat/ask`

**Example**:
```json
{
  "answer": "Forward kinematics is the process of determining the position and orientation of the end-effector of a robotic arm given the joint angles. See Chapter 3 for detailed derivations.",
  "sources": [
    {
      "id": "chunk-3-2-1",
      "title": "Chapter 3, Section 3.2 - Forward Kinematics",
      "snippet": "Forward kinematics is the process of determining the spatial position and orientation of the end-effector (also called tool or hand) of a robotic manipulator...",
      "url": "http://localhost:3000/docs/chapter-3-kinematics#forward-kinematics",
      "similarity": 0.92
    }
  ],
  "confidence": 0.87,
  "metadata": {
    "searchLatencyMs": 145,
    "generationLatencyMs": 1230,
    "totalLatencyMs": 1375,
    "chunksRetrieved": 5,
    "chunksUsed": 3,
    "model": "gemini-1.5-flash",
    "tokensUsed": 450
  }
}
```

### Widget Configuration

ChatKit widget configuration options.

```typescript
interface ChatKitWidgetConfig {
  // Backend integration
  apiEndpoint: string;                  // Backend RAG endpoint URL

  // Behavior
  position: 'bottom-right' | 'bottom-left'; // Widget position on page (default: bottom-right)
  defaultOpen: boolean;                 // Auto-open on page load (default: false)

  // Theming
  darkMode: boolean;                    // Current dark mode state
  colorScheme?: {
    primary?: string;                   // Primary color (CSS color value)
    secondary?: string;                 // Secondary color
    error?: string;                     // Error state color
  };

  // Features
  enableSelectedText: boolean;          // Enable right-click "Ask about this" (default: true)
  maxHistoryMessages?: number;          // Max messages to store (default: 100)
}
```

**Validation Rules**:
- `apiEndpoint`: Must be valid URL
- `position`: One of two allowed values
- Color values: Valid CSS colors (hex, rgb, named colors)
- `maxHistoryMessages`: 1-1000 range

## State Machine: Message Status

```
┌─────────────┐
│   pending   │  Initial state when user sends message
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    sent     │  Message successfully sent to server
└──────┬──────┘
       │
    ┌──┴──────────────────────────┐
    │                             │
    ▼                             ▼
┌──────────────┐          ┌───────────────┐
│   received   │          │     error     │  Network error, timeout, etc.
└──────────────┘          └─────────────────┘
    (Assistant)                (User action needed)
    (Do not retry)             (Allow retry)
```

## Entity Relationships

```
┌──────────────────────────────────────────┐
│         ChatSession (Root)               │
│  sessionId, createdAt, lastMessageAt     │
└──────────────────────────────────────────┘
             │ contains
             ▼
    ┌─────────────────────────────────────────────────────┐
    │            ChatMessage (array)                       │
    │  id, role, content, timestamp, status               │
    │                                                      │
    │  IF role='user':                                    │
    │  ├─ selectedText (optional)                         │
    │  └─ pageContext → PageContext                       │
    │                                                      │
    │  IF role='assistant':                               │
    │  ├─ sources → SourceReference[] (from RAGResponse)  │
    │  ├─ confidence                                      │
    │  └─ metadata → ResponseMetadata                     │
    └─────────────────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│       RAGRequest (sent to backend)       │
│  question, selectedText?, pageContext?   │
└──────────────────────────────────────────┘

┌──────────────────────────────────────────┐
│      RAGResponse (received from backend) │
│  answer, sources[], confidence, metadata │
└──────────────────────────────────────────┘
```

## Storage Strategy

### localStorage Keys

```
chatkit-session-{sessionId}     // Current session object (JSON)
chatkit-active-session          // UUID of currently active session
chatkit-config                  // Widget configuration
```

### Size Estimation

- Single ChatMessage: ~300-500 bytes (with sources)
- Single ChatSession: 300 bytes + (messages count × avg message size)
- Full capacity: 10KB localStorage ≈ 20-25 messages with sources

**Cleanup Strategy**:
- Keep only current session in localStorage
- Archive older sessions to memory (lost on page reload)
- Allow user to clear history with button (FR-014)

### localStorage Persistence Code Pattern

```typescript
// Save session
localStorage.setItem(
  `chatkit-session-${session.sessionId}`,
  JSON.stringify(session)
);

// Load session
const session = JSON.parse(
  localStorage.getItem(`chatkit-session-${sessionId}`)
);

// Clear all
Object.keys(localStorage)
  .filter(key => key.startsWith('chatkit-'))
  .forEach(key => localStorage.removeItem(key));
```

## Type Safety & Validation

All entities use TypeScript interfaces with:
- **Required fields**: Marked without `?`
- **Optional fields**: Marked with `?`
- **Enums**: Union types for fixed values (`'high' | 'medium' | 'low'`)
- **Ranges**: JSDoc comments for numeric bounds

## References to Specification

- **ChatMessage**: Maps to User Story 1 (P1), Core Entity
- **PageContext**: Maps to User Story 2 (P2), P2-specific context
- **Selected Text**: Maps to User Story 3 (P3), Optional in ChatMessage
- **RAGRequest/Response**: Maps to FR-003 (backend communication), FR-004 (message format), FR-005 (sources)
- **Storage**: Maps to FR-007 (history persistence), FR-014 (clear history)
