/**
 * Page Context Service - Extract Docusaurus metadata
 *
 * Detects current documentation page context (chapter, section) to provide
 * contextually relevant answers from the RAG backend.
 * Works with Docusaurus' native sidebar and breadcrumb data structures.
 */

import { PageContext } from '../types/chatkit.types';

/**
 * Extract chapter/section from Docusaurus breadcrumb/sidebar metadata
 * Attempts multiple extraction strategies with confidence scoring
 *
 * @returns PageContext with URL, chapter, section, and confidence level
 */
export function getPageContext(): PageContext {
  // Always available in browser context
  const url = typeof window !== 'undefined' ? window.location.href : '';
  const pathname = typeof window !== 'undefined' ? window.location.pathname : '';

  // If not in browser, return minimal context
  if (typeof window === 'undefined') {
    return {
      url,
      pathname,
      confidence: 'low',
    };
  }

  let chapter: string | undefined;
  let section: string | undefined;
  let confidence: 'high' | 'medium' | 'low' = 'low';

  // Strategy 1: Extract from Docusaurus breadcrumb element
  try {
    const breadcrumb = document.querySelector(
      'nav[aria-label="Docs breadcrumb"], [class*="breadcrumb"]'
    );

    if (breadcrumb) {
      const breadcrumbLinks = breadcrumb.querySelectorAll('a');
      const breadcrumbTexts = Array.from(breadcrumbLinks)
        .map(link => link.textContent?.trim())
        .filter(Boolean) as string[];

      if (breadcrumbTexts.length > 0) {
        // Docusaurus breadcrumb structure: Home > Chapter > Section > Current Page
        if (breadcrumbTexts.length >= 2) {
          // Second item is usually chapter
          chapter = breadcrumbTexts[1];
          // Third item is section if available
          if (breadcrumbTexts.length >= 3) {
            section = breadcrumbTexts[2];
            confidence = 'high';
          } else {
            confidence = 'high';
          }
        }
      }
    }
  } catch (error) {
    console.debug('Error extracting breadcrumb context', error);
  }

  // Strategy 2: Extract from Docusaurus sidebar active item
  if (!chapter && confidence === 'low') {
    try {
      // Look for active sidebar item with aria-current="page"
      const activeItem = document.querySelector(
        '[aria-current="page"], [class*="active"]'
      );

      if (activeItem) {
        const text = activeItem.textContent?.trim();
        if (text) {
          chapter = text;
          confidence = 'medium';

          // Try to find parent (section)
          const parent = activeItem.closest('li')?.parentElement?.closest('li');
          if (parent) {
            const parentText = parent.textContent?.trim();
            if (parentText) {
              section = parentText;
            }
          }
        }
      }
    } catch (error) {
      console.debug('Error extracting sidebar context', error);
    }
  }

  // Strategy 3: Extract from page title/heading
  if (!chapter && confidence === 'low') {
    try {
      // Get main heading content
      const heading = document.querySelector('h1, h2');
      if (heading) {
        const text = heading.textContent?.trim();
        if (text && text.length > 0 && text.length < 100) {
          chapter = text;
          confidence = 'low'; // Low confidence for title-based extraction
        }
      }
    } catch (error) {
      console.debug('Error extracting heading context', error);
    }
  }

  // Strategy 4: Extract from URL path segments
  if (!chapter && confidence === 'low') {
    try {
      const segments = pathname
        .split('/')
        .filter(seg => seg.length > 0 && !seg.includes('.'));

      if (segments.length > 0) {
        // Last segment is usually the page name
        const lastSegment = segments[segments.length - 1];
        // Convert kebab-case to Title Case
        chapter = lastSegment
          .split('-')
          .map(word => word.charAt(0).toUpperCase() + word.slice(1))
          .join(' ');

        confidence = 'low';
      }
    } catch (error) {
      console.debug('Error extracting URL context', error);
    }
  }

  return {
    url,
    pathname,
    chapter,
    section,
    confidence,
  };
}

/**
 * Watch for route changes and update page context
 * Used by usePageContext hook to detect navigation
 *
 * @param callback - Called when pathname changes
 * @returns Cleanup function to stop watching
 */
export function watchPageContextChanges(
  callback: (context: PageContext) => void
): () => void {
  let previousPathname = typeof window !== 'undefined' ? window.location.pathname : '';

  const handlePopState = () => {
    const newPathname = window.location.pathname;
    if (newPathname !== previousPathname) {
      previousPathname = newPathname;
      callback(getPageContext());
    }
  };

  // Listen for history changes (browser back/forward)
  window.addEventListener('popstate', handlePopState);

  // Also watch for Docusaurus navigation events (SPA routing)
  // Docusaurus uses window.history.pushState internally
  const originalPushState = window.history.pushState;
  window.history.pushState = function(...args) {
    originalPushState.apply(window.history, args);
    const newPathname = window.location.pathname;
    if (newPathname !== previousPathname) {
      previousPathname = newPathname;
      callback(getPageContext());
    }
    return undefined;
  };

  // Return cleanup function
  return () => {
    window.removeEventListener('popstate', handlePopState);
    window.history.pushState = originalPushState;
  };
}

export const pageContextService = {
  getPageContext,
  watchPageContextChanges,
};
