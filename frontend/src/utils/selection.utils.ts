/**
 * Utility functions for text selection validation and manipulation
 */

import { MAX_SELECTED_TEXT_LENGTH } from '../constants/selection.constants';

/**
 * Validates if selected text is valid (not empty/whitespace)
 * @param text - The text to validate
 * @returns true if text is valid, false otherwise
 */
export function validateSelection(text: string): boolean {
  if (!text || typeof text !== 'string') {
    return false;
  }

  // Check if it's only whitespace
  return text.trim().length > 0;
}

/**
 * Truncates selected text to maximum length
 * @param text - The text to truncate
 * @param maxLength - Maximum length (default: MAX_SELECTED_TEXT_LENGTH)
 * @returns Truncated text
 */
export function truncateSelection(text: string, maxLength: number = MAX_SELECTED_TEXT_LENGTH): string {
  if (!text) {
    return '';
  }

  if (text.length <= maxLength) {
    return text;
  }

  return text.substring(0, maxLength).trim();
}

/**
 * Normalizes text by trimming whitespace and collapsing multiple spaces
 * @param text - The text to normalize
 * @returns Normalized text
 */
export function normalizeText(text: string): string {
  if (!text) {
    return '';
  }

  // Trim and collapse multiple whitespace into single space
  return text.trim().replace(/\s+/g, ' ');
}

/**
 * Checks if text is only whitespace
 * @param text - The text to check
 * @returns true if text is only whitespace or empty
 */
export function isWhitespaceOnly(text: string | null | undefined): boolean {
  if (!text || typeof text !== 'string') {
    return true;
  }

  return text.trim().length === 0;
}

/**
 * Gets a preview of the text truncated to specified length
 * @param text - The text to preview
 * @param maxLength - Maximum length of preview
 * @returns Preview text with ellipsis if truncated
 */
export function getTextPreview(text: string, maxLength: number = 50): string {
  if (!text) {
    return '';
  }

  const normalized = normalizeText(text);

  if (normalized.length <= maxLength) {
    return normalized;
  }

  return normalized.substring(0, maxLength).trim() + '...';
}

/**
 * Sanitizes text for safe display (prevents XSS)
 * @param text - The text to sanitize
 * @returns Sanitized text safe for HTML display
 */
export function sanitizeText(text: string): string {
  if (!text) {
    return '';
  }

  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

/**
 * Extracts individual terms from selected text for boosting
 * @param text - The text to extract terms from
 * @returns Array of unique terms
 */
export function extractTerms(text: string): string[] {
  if (!text) {
    return [];
  }

  // Split by whitespace and non-alphanumeric characters (except hyphens)
  const terms = text
    .toLowerCase()
    .split(/[\s\-]+/)
    .filter((term) => term.length > 0)
    .filter((term, index, arr) => arr.indexOf(term) === index); // Remove duplicates

  return terms;
}

/**
 * Calculates text length in characters
 * @param text - The text to measure
 * @returns Character count
 */
export function getTextLength(text: string | null | undefined): number {
  if (!text || typeof text !== 'string') {
    return 0;
  }

  return text.length;
}

/**
 * Checks if text exceeds maximum length
 * @param text - The text to check
 * @param maxLength - Maximum allowed length
 * @returns true if text exceeds maximum length
 */
export function exceedsMaxLength(text: string | null | undefined, maxLength: number): boolean {
  return getTextLength(text) > maxLength;
}
