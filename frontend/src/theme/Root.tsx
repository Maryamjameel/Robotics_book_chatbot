/**
 * Root Layout Component
 *
 * Wraps the entire Docusaurus application with global providers.
 * This is where ChatKitProvider is integrated to make the chat widget available
 * to all pages in the documentation.
 *
 * Also integrates:
 * - SelectionTooltip: Displays tooltip when user selects text
 * - useTextSelection & useSelectionTooltip: Manage text selection and tooltip state
 */

import React, { ReactNode, useCallback } from 'react';
import { ChatKitProvider } from '@site/src/components/ChatKit/ChatKitProvider';
import { SelectionTooltip } from '@site/src/components/SelectionTooltip';
import { useTextSelection } from '@site/src/hooks/useTextSelection';
import { useSelectionTooltip } from '@site/src/hooks/useSelectionTooltip';
import { getBestTooltipPosition } from '@site/src/utils/positioning.utils';
import type { TextSelection } from '@site/src/types/selected-text.types';

interface RootProps {
  children: ReactNode;
}

/**
 * Root Component
 * Top-level layout component for the entire Docusaurus site
 *
 * @param children - The page content from Docusaurus router
 */
export default function Root({ children }: RootProps): JSX.Element {
  // Detect text selection on the page
  const { selection } = useTextSelection();

  // Manage tooltip visibility and state
  const { state: tooltipState, show: showTooltip, dismiss: dismissTooltip } = useSelectionTooltip();

  /**
   * Handle new text selection - show tooltip at selection coordinates
   */
  React.useEffect(() => {
    if (selection && !tooltipState.isDismissed) {
      // Calculate optimal position for tooltip based on selection location
      const bestPosition = getBestTooltipPosition(
        new DOMRect(selection.x, selection.y, 0, 0),
        selection.text.length
      );

      showTooltip({
        ...selection,
        x: bestPosition.x,
        y: bestPosition.y,
      });
    }
  }, [selection, tooltipState.isDismissed, showTooltip]);

  /**
   * Handle "Ask about this" button click
   * Passes selected text to ChatKit widget for context
   */
  const handleAskAboutSelection = useCallback((selectedText: string) => {
    // Dismiss the tooltip after clicking
    dismissTooltip();

    // Dispatch custom event that ChatKit will listen to
    // This allows ChatKit to pre-fill with the selected text
    const event = new CustomEvent('selection:ask', {
      detail: { selectedText },
    });
    document.dispatchEvent(event);
  }, [dismissTooltip]);

  /**
   * Handle tooltip dismiss
   */
  const handleDismissTooltip = useCallback(() => {
    dismissTooltip();
  }, [dismissTooltip]);

  return (
    <ChatKitProvider>
      {/* Selection tooltip appears when user selects text */}
      <SelectionTooltip
        isVisible={tooltipState.isVisible}
        position={tooltipState.position}
        selectedText={tooltipState.selection?.text || ''}
        onAsk={handleAskAboutSelection}
        onDismiss={handleDismissTooltip}
      />

      {/* Page content */}
      {children}
    </ChatKitProvider>
  );
}
