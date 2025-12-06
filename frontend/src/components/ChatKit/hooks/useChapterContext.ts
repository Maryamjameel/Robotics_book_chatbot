/**
 * useChapterContext Hook
 * Extracts chapter context from the current page URL and DOM
 * Returns chapter metadata for search filtering and display
 */

import { useLocation } from 'react-router-dom';
import { ChapterContext } from '../types/chatkit.types';

/**
 * Extract chapter ID from URL pathname
 * Matches patterns: /docs/chapter-3-kinematics, /chapter/3, /ch-03
 */
function extractChapterIdFromUrl(pathname: string): string | null {
  // Pattern: /chapter[_-]?(\w+)
  const match = pathname.match(/chapter[_-]?(\w+)/i);
  return match ? match[1].toLowerCase() : null;
}

/**
 * Extract chapter title from page h1 element
 */
function extractTitleFromDOM(): string | null {
  try {
    const h1 = document.querySelector('h1');
    const text = h1?.textContent?.trim();
    return text && text.length > 0 ? text : null;
  } catch {
    return null;
  }
}

/**
 * Convert title to URL-safe slug
 * Example: "Inverse Kinematics" → "inverse-kinematics"
 */
function titleToSlug(title: string): string {
  return title
    .toLowerCase()
    .replace(/\s+/g, '-')
    .replace(/[^a-z0-9\-]/g, '')
    .replace(/\-+/g, '-')
    .replace(/^\-|\-$/g, '');
}

/**
 * Determine extraction confidence level
 * "high": both URL and h1 match
 * "medium": only URL or only h1 match
 * "low": uncertain extraction
 */
function determineConfidence(
  chapterId: string | null,
  chapterTitle: string | null,
  urlMatched: boolean,
  domMatched: boolean
): 'high' | 'medium' | 'low' {
  if (chapterId && chapterTitle && urlMatched && domMatched) {
    return 'high'; // Both sources confirm
  }
  if ((chapterId && urlMatched) || (chapterTitle && domMatched)) {
    return 'medium'; // One source confirms
  }
  return 'low'; // Uncertain
}

/**
 * useChapterContext Hook
 * Extracts chapter context from current page
 * Returns null if not on a chapter page
 *
 * @returns ChapterContext object or null
 *
 * @example
 * const chapterContext = useChapterContext();
 * if (chapterContext) {
 *   console.log(`On chapter: ${chapterContext.chapterTitle}`);
 * }
 */
export function useChapterContext(): ChapterContext | null {
  const location = useLocation();

  try {
    // Extract chapter ID from URL
    const chapterId = extractChapterIdFromUrl(location.pathname);
    const urlMatched = !!chapterId;

    // Extract chapter title from DOM
    const chapterTitle = extractTitleFromDOM();
    const domMatched = !!chapterTitle;

    // If neither source found chapter context, return null
    if (!chapterId && !chapterTitle) {
      return null;
    }

    // Generate slug from title
    const chapterSlug = chapterTitle ? titleToSlug(chapterTitle) : '';

    // Determine confidence level
    const confidence = determineConfidence(
      chapterId,
      chapterTitle,
      urlMatched,
      domMatched
    );

    // Fallback values for missing fields
    const finalChapterId = chapterId || chapterSlug || 'unknown';
    const finalChapterTitle = chapterTitle || 'Unknown Chapter';

    return {
      chapterId: finalChapterId,
      chapterTitle: finalChapterTitle,
      chapterSlug: chapterSlug,
      confidence,
    };
  } catch (error) {
    console.error('Error extracting chapter context:', error);
    return null;
  }
}

export default useChapterContext;
