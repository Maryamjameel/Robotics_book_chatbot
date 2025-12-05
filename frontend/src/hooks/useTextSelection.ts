/**
 * useTextSelection Hook
 * Detects text selection on the page and returns selection coordinates for tooltip positioning
 */

import { useEffect, useRef, useState, useCallback } from 'react';
import type { TextSelection, UseTextSelectionReturn } from '../types/selected-text.types';
import { validateSelection, normalizeText } from '../utils/selection.utils';
import { DEBOUNCE_DELAY_MS } from '../constants/selection.constants';

/**
 * Hook to detect text selection and return selection data with viewport coordinates
 * @returns Object with current selection, active state, and error information
 */
export function useTextSelection(): UseTextSelectionReturn {
  const [selection, setSelection] = useState<TextSelection | null>(null);
  const [isActive, setIsActive] = useState<boolean>(true);
  const [error, setError] = useState<Error | null>(null);
  const debounceTimer = useRef<NodeJS.Timeout | null>(null);

  /**
   * Handle selection events (mouseup, touchend)
   * Detects selected text and calculates its bounding coordinates
   */
  const handleSelectionChange = useCallback(() => {
    // Clear any pending debounce
    if (debounceTimer.current) {
      clearTimeout(debounceTimer.current);
    }

    // Debounce the selection update
    debounceTimer.current = setTimeout(() => {
      try {
        // Check if window.getSelection is supported
        if (!window.getSelection) {
          setError(new Error('Text selection not supported in this browser'));
          setIsActive(false);
          return;
        }

        const selectedText = window.getSelection();

        // If nothing selected, clear state
        if (!selectedText || selectedText.toString().length === 0) {
          setSelection(null);
          return;
        }

        // Get selected text and validate it
        const text = normalizeText(selectedText.toString());

        if (!validateSelection(text)) {
          // Empty or whitespace-only selection
          setSelection(null);
          return;
        }

        // Get the bounding rectangle of the selection
        const range = selectedText.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Create selection object with viewport coordinates
        const newSelection: TextSelection = {
          text,
          x: Math.round(rect.left),
          y: Math.round(rect.top),
          timestamp: Date.now(),
        };

        setSelection(newSelection);
        setError(null);
      } catch (err) {
        const error = err instanceof Error ? err : new Error('Unknown selection error');
        setError(error);
        setSelection(null);
        console.error('Error detecting text selection:', error);
      }
    }, DEBOUNCE_DELAY_MS);
  }, []);

  /**
   * Set up event listeners for selection detection
   */
  useEffect(() => {
    try {
      // Check if window.getSelection is supported
      if (!window.getSelection) {
        setError(new Error('Text selection not supported in this browser'));
        setIsActive(false);
        return;
      }

      // Add event listeners for mouseup and touchend
      document.addEventListener('mouseup', handleSelectionChange);
      document.addEventListener('touchend', handleSelectionChange);

      setIsActive(true);

      return () => {
        // Cleanup: remove event listeners
        document.removeEventListener('mouseup', handleSelectionChange);
        document.removeEventListener('touchend', handleSelectionChange);

        // Clear any pending debounce timer
        if (debounceTimer.current) {
          clearTimeout(debounceTimer.current);
        }
      };
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Unknown error setting up selection detection');
      setError(error);
      setIsActive(false);
      console.error('Error setting up text selection detection:', error);
    }
  }, [handleSelectionChange]);

  return {
    selection,
    isActive,
    error,
  };
}
