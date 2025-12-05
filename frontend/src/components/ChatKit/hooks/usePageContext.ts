/**
 * usePageContext Hook - Track current Docusaurus page context
 *
 * Extracts and monitors the current documentation page (chapter, section).
 * Updates when user navigates to different pages.
 * Used to provide contextually relevant RAG answers.
 */

import { useState, useEffect } from 'react';
import { PageContext } from '../types/chatkit.types';
import { pageContextService } from '../services/pageContextService';

/**
 * Hook for tracking page context
 * Initializes context on mount and updates when pathname changes
 *
 * @returns PageContext object or null until initialized
 */
export function usePageContext(): PageContext | null {
  const [context, setContext] = useState<PageContext | null>(null);

  useEffect(() => {
    // Initial page context extraction
    const initialContext = pageContextService.getPageContext();
    setContext(initialContext);

    // Watch for route changes and update context
    const unsubscribe = pageContextService.watchPageContextChanges(newContext => {
      setContext(newContext);
    });

    // Cleanup subscription on unmount
    return () => {
      unsubscribe();
    };
  }, []);

  return context;
}
