---
name: "component-generation"
description: "Generate React/TypeScript UI components for Docusaurus. Creates buttons, forms, modals, widgets, and interactive elements with proper TypeScript types and styling. Use when building frontend UI components."
version: "1.0.0"
---

# Component Generation Skill

## When to Use This Skill

- Creating new React components for Docusaurus
- Building interactive buttons (Personalize, Translate, Chat)
- Generating authentication forms (Signup, Signin)
- Creating modal dialogs and popups
- Building chatbot widgets and floating interfaces
- Designing responsive UI components
- Adding TypeScript type definitions

## How This Skill Works

1. **Define Component Purpose**: Understand what the component does
2. **Design Props Interface**: Create TypeScript interface for props
3. **Implement Component Logic**: Write React component with hooks
4. **Add Styling**: Include CSS or Tailwind classes
5. **Create Tests**: Write component tests with React Testing Library
6. **Document Usage**: Provide usage examples

## Component Templates

### 1. Button Component

```typescript
// src/components/PersonalizeButton.tsx
import React, { useState } from 'react';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalized?: (content: string) => void;
  className?: string;
}

export function PersonalizeButton({
  chapterId,
  onPersonalized,
  className
}: PersonalizeButtonProps): JSX.Element {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleClick = async () => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`/api/v1/personalization/personalize`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ chapter_id: chapterId })
      });

      if (!response.ok) {
        throw new Error('Personalization failed');
      }

      const data = await response.json();
      onPersonalized?.(data.personalizedContent);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={className}>
      <button
        onClick={handleClick}
        disabled={isLoading}
        className={`${styles.button} ${styles.primary}`}
        aria-label="Personalize chapter content"
      >
        {isLoading ? (
          <>
            <span className={styles.spinner} />
            Personalizing...
          </>
        ) : (
          <>
            <span className={styles.icon}>✨</span>
            Personalize Chapter
          </>
        )}
      </button>
      {error && <div className={styles.error}>{error}</div>}
    </div>
  );
}
```

**Styling** (`PersonalizeButton.module.css`):
```css
.button {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.75rem 1.5rem;
  border: none;
  border-radius: 0.5rem;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s ease;
}

.button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.primary {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

.primary:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
}

.spinner {
  display: inline-block;
  width: 1rem;
  height: 1rem;
  border: 2px solid rgba(255, 255, 255, 0.3);
  border-top-color: white;
  border-radius: 50%;
  animation: spin 0.6s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.error {
  margin-top: 0.5rem;
  color: var(--ifm-color-danger);
  font-size: 0.875rem;
}

.icon {
  font-size: 1.25rem;
}
```

### 2. Form Component (Signup)

```typescript
// src/components/AuthForms/SignupForm.tsx
import React, { useState } from 'react';
import { useNavigate } from '@docusaurus/router';
import styles from './SignupForm.module.css';

interface SignupFormData {
  email: string;
  password: string;
  name: string;
  softwareBackground: string;
  hardwareBackground: string;
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
}

interface SignupFormProps {
  onSuccess?: () => void;
}

export function SignupForm({ onSuccess }: SignupFormProps): JSX.Element {
  const navigate = useNavigate();
  const [formData, setFormData] = useState<SignupFormData>({
    email: '',
    password: '',
    name: '',
    softwareBackground: '',
    hardwareBackground: '',
    experienceLevel: 'beginner'
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>
  ) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/v1/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(formData)
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.error || 'Signup failed');
      }

      onSuccess?.();
      navigate('/dashboard');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.form}>
      <h2 className={styles.title}>Create Your Account</h2>

      <div className={styles.formGroup}>
        <label htmlFor="email" className={styles.label}>
          Email Address *
        </label>
        <input
          type="email"
          id="email"
          name="email"
          value={formData.email}
          onChange={handleChange}
          required
          className={styles.input}
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="password" className={styles.label}>
          Password *
        </label>
        <input
          type="password"
          id="password"
          name="password"
          value={formData.password}
          onChange={handleChange}
          required
          minLength={8}
          className={styles.input}
        />
        <span className={styles.hint}>At least 8 characters</span>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="name" className={styles.label}>
          Full Name
        </label>
        <input
          type="text"
          id="name"
          name="name"
          value={formData.name}
          onChange={handleChange}
          className={styles.input}
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="softwareBackground" className={styles.label}>
          Software Background
          <span className={styles.hint}>e.g., Python, JavaScript, C++</span>
        </label>
        <textarea
          id="softwareBackground"
          name="softwareBackground"
          value={formData.softwareBackground}
          onChange={handleChange}
          rows={3}
          className={styles.textarea}
          placeholder="List programming languages and frameworks you know"
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="hardwareBackground" className={styles.label}>
          Hardware Background
          <span className={styles.hint}>e.g., Arduino, Raspberry Pi</span>
        </label>
        <textarea
          id="hardwareBackground"
          name="hardwareBackground"
          value={formData.hardwareBackground}
          onChange={handleChange}
          rows={3}
          className={styles.textarea}
          placeholder="List hardware platforms you've worked with"
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="experienceLevel" className={styles.label}>
          Experience Level
        </label>
        <select
          id="experienceLevel"
          name="experienceLevel"
          value={formData.experienceLevel}
          onChange={handleChange}
          className={styles.select}
        >
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
        </select>
      </div>

      {error && <div className={styles.errorMessage}>{error}</div>}

      <button
        type="submit"
        disabled={isLoading}
        className={styles.submitButton}
      >
        {isLoading ? 'Creating Account...' : 'Sign Up'}
      </button>
    </form>
  );
}
```

### 3. Chatbot Widget Component

```typescript
// src/components/ChatbotWidget.tsx
import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
}

export function ChatbotWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput('');
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      // Get selected text if any
      const selectedText = window.getSelection()?.toString() || null;

      const response = await fetch('/api/v1/chatbot/ask', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          question: userMessage,
          selected_text: selectedText
        })
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.answer }]);
    } catch (error) {
      setMessages(prev => [
        ...prev,
        { role: 'assistant', content: 'Sorry, I encountered an error. Please try again.' }
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <>
      {/* Toggle Button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={styles.toggleButton}
        aria-label="Toggle chatbot"
      >
        💬
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.header}>
            <h3 className={styles.headerTitle}>Ask about the textbook</h3>
            <button
              onClick={() => setIsOpen(false)}
              className={styles.closeButton}
              aria-label="Close chatbot"
            >
              ✕
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                👋 Hi! Select text and ask questions about the robotics textbook.
              </div>
            ) : (
              messages.map((msg, idx) => (
                <div
                  key={idx}
                  className={`${styles.message} ${styles[`message${msg.role.charAt(0).toUpperCase() + msg.role.slice(1)}`]}`}
                >
                  {msg.content}
                </div>
              ))
            )}
            {isLoading && (
              <div className={`${styles.message} ${styles.messageLoading}`}>
                <span className={styles.loadingDots}>Thinking</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              disabled={isLoading}
              className={styles.input}
            />
            <button
              onClick={handleSendMessage}
              disabled={isLoading || !input.trim()}
              className={styles.sendButton}
              aria-label="Send message"
            >
              ➤
            </button>
          </div>
        </div>
      )}
    </>
  );
}
```

**Styling** (`ChatbotWidget.module.css`):
```css
.toggleButton {
  position: fixed;
  bottom: 2rem;
  right: 2rem;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  font-size: 1.5rem;
  border: none;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
  transition: transform 0.2s;
}

.toggleButton:hover {
  transform: scale(1.1);
}

.chatWindow {
  position: fixed;
  bottom: 100px;
  right: 2rem;
  width: 400px;
  height: 600px;
  background: white;
  border-radius: 1rem;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
  display: flex;
  flex-direction: column;
  z-index: 999;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border-radius: 1rem 1rem 0 0;
}

.headerTitle {
  margin: 0;
  font-size: 1.125rem;
  font-weight: 600;
}

.closeButton {
  background: none;
  border: none;
  color: white;
  font-size: 1.5rem;
  cursor: pointer;
  padding: 0;
  width: 2rem;
  height: 2rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.messagesContainer {
  flex: 1;
  overflow-y: auto;
  padding: 1rem;
  background: #f8f9fa;
}

.message {
  margin-bottom: 1rem;
  padding: 0.75rem 1rem;
  border-radius: 0.75rem;
  max-width: 80%;
  line-height: 1.5;
}

.messageUser {
  background: #667eea;
  color: white;
  margin-left: auto;
  border-bottom-right-radius: 0.25rem;
}

.messageAssistant {
  background: white;
  color: #333;
  border: 1px solid #e0e0e0;
  border-bottom-left-radius: 0.25rem;
}

.messageLoading {
  background: white;
  color: #666;
  font-style: italic;
}

.inputContainer {
  display: flex;
  gap: 0.5rem;
  padding: 1rem;
  border-top: 1px solid #e0e0e0;
  background: white;
  border-radius: 0 0 1rem 1rem;
}

.input {
  flex: 1;
  padding: 0.75rem;
  border: 1px solid #e0e0e0;
  border-radius: 0.5rem;
  font-size: 0.9375rem;
}

.sendButton {
  padding: 0.75rem 1rem;
  background: #667eea;
  color: white;
  border: none;
  border-radius: 0.5rem;
  cursor: pointer;
  font-size: 1.25rem;
}

.sendButton:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

@media (max-width: 768px) {
  .chatWindow {
    width: calc(100vw - 2rem);
    right: 1rem;
    left: 1rem;
  }
}
```

### 4. Modal Component

```typescript
// src/components/shared/Modal.tsx
import React, { useEffect } from 'react';
import ReactDOM from 'react-dom';
import styles from './Modal.module.css';

interface ModalProps {
  isOpen: boolean;
  onClose: () => void;
  title?: string;
  children: React.ReactNode;
  size?: 'small' | 'medium' | 'large';
}

export function Modal({
  isOpen,
  onClose,
  title,
  children,
  size = 'medium'
}: ModalProps): JSX.Element | null {
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'unset';
    }

    return () => {
      document.body.style.overflow = 'unset';
    };
  }, [isOpen]);

  if (!isOpen) return null;

  return ReactDOM.createPortal(
    <div className={styles.overlay} onClick={onClose}>
      <div
        className={`${styles.modal} ${styles[size]}`}
        onClick={(e) => e.stopPropagation()}
      >
        {title && (
          <div className={styles.header}>
            <h2 className={styles.title}>{title}</h2>
            <button onClick={onClose} className={styles.closeButton}>
              ✕
            </button>
          </div>
        )}
        <div className={styles.content}>{children}</div>
      </div>
    </div>,
    document.body
  );
}
```

## Component Testing

```typescript
// src/components/__tests__/PersonalizeButton.test.tsx
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { PersonalizeButton } from '../PersonalizeButton';

describe('PersonalizeButton', () => {
  it('renders button with correct text', () => {
    render(<PersonalizeButton chapterId="1" />);
    expect(screen.getByText('Personalize Chapter')).toBeInTheDocument();
  });

  it('shows loading state when clicked', async () => {
    render(<PersonalizeButton chapterId="1" />);
    const button = screen.getByRole('button');

    fireEvent.click(button);

    await waitFor(() => {
      expect(screen.getByText('Personalizing...')).toBeInTheDocument();
    });
  });

  it('calls onPersonalized callback on success', async () => {
    const onPersonalized = jest.fn();

    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ personalizedContent: 'test content' })
      })
    ) as jest.Mock;

    render(
      <PersonalizeButton chapterId="1" onPersonalized={onPersonalized} />
    );

    fireEvent.click(screen.getByRole('button'));

    await waitFor(() => {
      expect(onPersonalized).toHaveBeenCalledWith('test content');
    });
  });
});
```

## Best Practices

- ✓ **Use TypeScript** for type safety
- ✓ **Component modularity** - keep components focused on single responsibility
- ✓ **Proper error handling** - show user-friendly error messages
- ✓ **Loading states** - provide feedback during async operations
- ✓ **Accessibility** - include ARIA labels and keyboard navigation
- ✓ **Responsive design** - work on all screen sizes
- ✓ **CSS Modules** - scope styles to avoid conflicts
- ✓ **Test components** - write unit tests with React Testing Library

## Output Format

When generating a component, provide:

1. **Component File**: Full TypeScript/JSX implementation
2. **Props Interface**: TypeScript interface for props
3. **Styling**: CSS Module or Tailwind classes
4. **Usage Example**: How to use the component
5. **Tests**: Unit tests for component behavior
