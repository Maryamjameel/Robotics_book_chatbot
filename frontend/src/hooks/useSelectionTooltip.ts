/**
 * useSelectionTooltip Hook
 * Manages tooltip visibility, positioning, and dismissal logic
 */

import { useEffect, useState, useCallback, useRef } from 'react';
import type { SelectionTooltipState, TextSelection, SelectionCoordinates } from '../types/selected-text.types';
import { TOOLTIP_OFFSET, TOOLTIP_AUTO_DISMISS_DELAY_MS, REPOSITION_DEBOUNCE_DELAY_MS } from '../constants/selection.constants';

/**
 * Hook to manage tooltip state and dismissal logic
 * @returns Tooltip state and control functions
 */
export function useSelectionTooltip() {
  const [state, setState] = useState<SelectionTooltipState>({
    isVisible: false,
    selection: null,
    isDismissed: false,
    position: { x: 0, y: 0 },
  });

  const autoDismissTimer = useRef<NodeJS.Timeout | null>(null);
  const repositionDebounceTimer = useRef<NodeJS.Timeout | null>(null);

  /**
   * Show tooltip at specified position
   */
  const show = useCallback((selection: TextSelection) => {
    // Clear any pending timers
    if (autoDismissTimer.current) {
      clearTimeout(autoDismissTimer.current);
    }
    if (repositionDebounceTimer.current) {
      clearTimeout(repositionDebounceTimer.current);
    }

    setState((prev) => ({
      ...prev,
      isVisible: true,
      selection,
      isDismissed: false,
      position: {
        x: selection.x + TOOLTIP_OFFSET,
        y: selection.y - TOOLTIP_OFFSET,
      },
    }));

    // Set up auto-dismiss if configured
    if (TOOLTIP_AUTO_DISMISS_DELAY_MS > 0) {
      autoDismissTimer.current = setTimeout(() => {
        dismiss();
      }, TOOLTIP_AUTO_DISMISS_DELAY_MS);
    }
  }, []);

  /**
   * Hide tooltip (user can select again)
   */
  const hide = useCallback(() => {
    if (autoDismissTimer.current) {
      clearTimeout(autoDismissTimer.current);
    }

    setState((prev) => ({
      ...prev,
      isVisible: false,
    }));
  }, []);

  /**
   * Dismiss tooltip (prevent reappearing for this selection)
   */
  const dismiss = useCallback(() => {
    if (autoDismissTimer.current) {
      clearTimeout(autoDismissTimer.current);
    }

    setState((prev) => ({
      ...prev,
      isVisible: false,
      isDismissed: true,
    }));
  }, []);

  /**
   * Reset dismissed state (allow tooltip to reappear)
   */
  const reset = useCallback(() => {
    if (autoDismissTimer.current) {
      clearTimeout(autoDismissTimer.current);
    }

    setState({
      isVisible: false,
      selection: null,
      isDismissed: false,
      position: { x: 0, y: 0 },
    });
  }, []);

  /**
   * Update tooltip position for viewport constraints
   */
  const updatePosition = useCallback((newPosition: SelectionCoordinates) => {
    if (repositionDebounceTimer.current) {
      clearTimeout(repositionDebounceTimer.current);
    }

    repositionDebounceTimer.current = setTimeout(() => {
      setState((prev) => ({
        ...prev,
        position: {
          x: newPosition.x,
          y: newPosition.y,
        },
      }));
    }, REPOSITION_DEBOUNCE_DELAY_MS);
  }, []);

  /**
   * Handle Escape key to dismiss tooltip
   */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape' && state.isVisible) {
        dismiss();
      }
    };

    if (state.isVisible) {
      document.addEventListener('keydown', handleKeyDown);

      return () => {
        document.removeEventListener('keydown', handleKeyDown);
      };
    }
  }, [state.isVisible, dismiss]);

  /**
   * Handle click outside tooltip to dismiss
   */
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (!state.isVisible) {
        return;
      }

      const target = event.target as HTMLElement;

      // Check if click was on tooltip or its children
      const tooltip = document.querySelector('[data-testid="selection-tooltip"]');
      if (tooltip && tooltip.contains(target)) {
        return; // Click was inside tooltip, don't dismiss
      }

      // Check if click was on "Ask about this" button in ChatKit
      if (target.closest('[data-testid="ask-button"]')) {
        return; // This is the intended action
      }

      dismiss();
    };

    if (state.isVisible) {
      // Use setTimeout to avoid immediately catching the selection click
      const timer = setTimeout(() => {
        document.addEventListener('click', handleClickOutside);
      }, 100);

      return () => {
        clearTimeout(timer);
        document.removeEventListener('click', handleClickOutside);
      };
    }
  }, [state.isVisible, dismiss]);

  /**
   * Handle scroll to dismiss tooltip
   */
  useEffect(() => {
    const handleScroll = () => {
      if (state.isVisible) {
        dismiss();
      }
    };

    if (state.isVisible) {
      document.addEventListener('scroll', handleScroll, true);

      return () => {
        document.removeEventListener('scroll', handleScroll, true);
      };
    }
  }, [state.isVisible, dismiss]);

  /**
   * Cleanup timers on unmount
   */
  useEffect(() => {
    return () => {
      if (autoDismissTimer.current) {
        clearTimeout(autoDismissTimer.current);
      }
      if (repositionDebounceTimer.current) {
        clearTimeout(repositionDebounceTimer.current);
      }
    };
  }, []);

  return {
    state,
    show,
    hide,
    dismiss,
    reset,
    updatePosition,
  };
}
