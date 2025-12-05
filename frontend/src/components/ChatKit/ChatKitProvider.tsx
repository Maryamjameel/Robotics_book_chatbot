/**
 * ChatKitProvider - Context provider and widget wrapper
 *
 * Wraps the entire Docusaurus application with ChatKit context.
 * Renders the ChatKitWidget at the app root level.
 * Manages global ChatKit state if needed in future versions.
 */

import React, { createContext, ReactNode } from 'react';

/**
 * ChatKit Context Type
 * Can be extended in future for global state management
 */
export interface ChatKitContextValue {
  // Placeholder for future context properties
  // Currently the widget manages its own state via hooks
}

/**
 * Create context for ChatKit
 */
export const ChatKitContext = createContext<ChatKitContextValue | undefined>(
  undefined
);

interface ChatKitProviderProps {
  children: ReactNode;
}

/**
 * ChatKitProvider Component
 * Wraps the app with ChatKit provider and renders the widget
 *
 * @param children - The wrapped application content
 */
export function ChatKitProvider({ children }: ChatKitProviderProps): JSX.Element {
  // Context value - can be extended with global state
  const contextValue: ChatKitContextValue = {};

  return (
    <ChatKitContext.Provider value={contextValue}>
      {children}
      {/* Widget rendered at root level */}
      <ChatKitWidget />
    </ChatKitContext.Provider>
  );
}

// Lazy import to avoid circular dependencies
// This allows the widget to be loaded only after the provider is set up
import { ChatKitWidget } from './ChatKitWidget';
