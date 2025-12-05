/**
 * SelectionTooltip Component
 * Displays a tooltip near selected text with "Ask about this" and dismiss buttons
 */

import React, { useEffect, useRef } from 'react';
import type { SelectionTooltipProps } from '../../types/selected-text.types';
import { getTextPreview } from '../../utils/selection.utils';
import { TOOLTIP_Z_INDEX } from '../../constants/selection.constants';
import './SelectionTooltip.css';

/**
 * SelectionTooltip Component
 *
 * Displays a fixed-position tooltip near the user's text selection with:
 * - Preview of selected text (truncated to 50 chars)
 * - "💬 Ask about this" button to trigger question
 * - "×" dismiss button to close tooltip
 * - Keyboard accessible (Escape to dismiss, Tab to navigate)
 * - WCAG AA compliant (color contrast, tap targets)
 *
 * @component
 * @example
 * const [tooltip, setTooltip] = useState({isVisible: false, position: {x: 0, y: 0}});
 * return (
 *   <SelectionTooltip
 *     isVisible={tooltip.isVisible}
 *     position={tooltip.position}
 *     selectedText="forward kinematics"
 *     onAsk={(text) => chatKit.open(text)}
 *     onDismiss={() => setTooltip({...tooltip, isVisible: false})}
 *   />
 * );
 */
export const SelectionTooltip = React.memo(
  ({
    isVisible,
    position,
    selectedText,
    onAsk,
    onDismiss,
    className = '',
    zIndex = TOOLTIP_Z_INDEX,
  }: SelectionTooltipProps) => {
    const tooltipRef = useRef<HTMLDivElement>(null);
    const askButtonRef = useRef<HTMLButtonElement>(null);
    const touchStartRef = useRef<{ x: number; y: number } | null>(null);

    /**
     * Handle "Ask about this" button click
     */
    const handleAskClick = () => {
      if (selectedText) {
        onAsk(selectedText);
      }
    };

    /**
     * Handle dismiss button click
     */
    const handleDismissClick = () => {
      onDismiss();
    };

    /**
     * Handle touch start for swipe gesture detection (mobile)
     */
    const handleTouchStart = (e: React.TouchEvent<HTMLDivElement>) => {
      if (e.touches.length === 1) {
        const touch = e.touches[0];
        touchStartRef.current = { x: touch.clientX, y: touch.clientY };
      }
    };

    /**
     * Handle touch end for swipe-to-dismiss gesture (mobile)
     * Swipe right or left dismisses the tooltip
     */
    const handleTouchEnd = (e: React.TouchEvent<HTMLDivElement>) => {
      if (!touchStartRef.current || e.changedTouches.length !== 1) return;

      const touch = e.changedTouches[0];
      const deltaX = touch.clientX - touchStartRef.current.x;
      const deltaY = touch.clientY - touchStartRef.current.y;

      // Horizontal swipe detection (>50px movement)
      // Ignore if vertical movement is greater (scrolling)
      if (Math.abs(deltaX) > 50 && Math.abs(deltaX) > Math.abs(deltaY)) {
        onDismiss();
      }

      touchStartRef.current = null;
    };

    /**
     * Handle Escape key press to dismiss tooltip
     */
    const handleKeyDown = (e: React.KeyboardEvent<HTMLDivElement>) => {
      if (e.key === 'Escape') {
        e.preventDefault();
        onDismiss();
      }
    };

    /**
     * Focus the ask button when tooltip becomes visible
     * for keyboard navigation accessibility
     */
    useEffect(() => {
      if (isVisible && askButtonRef.current) {
        // Small delay to ensure element is mounted and visible
        const timer = setTimeout(() => {
          askButtonRef.current?.focus();
        }, 50);

        return () => clearTimeout(timer);
      }
    }, [isVisible]);

    if (!isVisible) {
      return null;
    }

    const preview = getTextPreview(selectedText, 50);
    const tooltipStyle: React.CSSProperties = {
      position: 'fixed',
      left: `${position.x}px`,
      top: `${position.y}px`,
      zIndex: zIndex,
    };

    return (
      <div
        ref={tooltipRef}
        className={`selection-tooltip ${className}`}
        style={tooltipStyle}
        role="dialog"
        aria-label="Text selection action menu"
        data-testid="selection-tooltip"
        onTouchStart={handleTouchStart}
        onTouchEnd={handleTouchEnd}
        onKeyDown={handleKeyDown}
        tabIndex={-1}
      >
        {/* Tooltip content container */}
        <div className="selection-tooltip__content">
          {/* Selected text preview */}
          <div className="selection-tooltip__preview" title={selectedText}>
            {preview}
          </div>

          {/* Button container */}
          <div className="selection-tooltip__buttons">
            {/* Ask about this button */}
            <button
              ref={askButtonRef}
              className="selection-tooltip__button selection-tooltip__button--primary"
              onClick={handleAskClick}
              aria-label={`Ask about: ${selectedText}`}
              title="Ask ChatKit about this selection"
              data-testid="ask-button"
            >
              <span className="selection-tooltip__button-icon">💬</span>
              <span className="selection-tooltip__button-text">Ask</span>
            </button>

            {/* Dismiss button */}
            <button
              className="selection-tooltip__button selection-tooltip__button--secondary"
              onClick={handleDismissClick}
              aria-label="Dismiss tooltip"
              title="Close this tooltip"
              data-testid="dismiss-button"
            >
              <span className="selection-tooltip__button-icon">×</span>
            </button>
          </div>
        </div>

        {/* Arrow pointing to selection */}
        <div className="selection-tooltip__arrow" aria-hidden="true" />
      </div>
    );
  }
);

SelectionTooltip.displayName = 'SelectionTooltip';
