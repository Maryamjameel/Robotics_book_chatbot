/**
 * Utility functions for tooltip positioning and viewport calculations
 */

import type { SelectionCoordinates } from '../types/selected-text.types';
import {
  TOOLTIP_OFFSET,
  TOOLTIP_MIN_WIDTH,
  TOOLTIP_MAX_WIDTH,
  TOOLTIP_ARROW_HEIGHT,
  VIEWPORT_THRESHOLD,
  MOBILE_BREAKPOINT,
} from '../constants/selection.constants';

/**
 * Calculates tooltip position relative to text selection
 * Positions above selection if there's enough space, otherwise below
 * @param rect - DOMRect of the selected text
 * @param viewportHeight - Height of the viewport
 * @param tooltipHeight - Height of the tooltip (default 60 for estimation)
 * @returns Coordinates {x, y} for tooltip positioning
 */
export function calculateTooltipPosition(
  rect: DOMRect,
  viewportHeight: number,
  tooltipHeight: number = 60
): SelectionCoordinates {
  const tooltipWidth = TOOLTIP_MIN_WIDTH; // Use minimum for calculation

  // Calculate position above selection
  let x = Math.round(rect.left + rect.width / 2 - tooltipWidth / 2);
  let y = Math.round(rect.top - TOOLTIP_OFFSET - tooltipHeight);

  // Check if there's enough space above
  const spaceAbove = rect.top - tooltipHeight - TOOLTIP_OFFSET;

  if (spaceAbove < VIEWPORT_THRESHOLD) {
    // Not enough space above, position below selection
    y = Math.round(rect.bottom + TOOLTIP_OFFSET);
  }

  return {
    x,
    y,
    width: tooltipWidth,
    height: tooltipHeight,
  };
}

/**
 * Checks if tooltip would be clipped at viewport edges
 * @param x - Left position of tooltip
 * @param y - Top position of tooltip
 * @param viewportWidth - Width of the viewport
 * @param viewportHeight - Height of the viewport
 * @param tooltipWidth - Width of the tooltip
 * @param tooltipHeight - Height of the tooltip
 * @returns true if tooltip would be clipped, false otherwise
 */
export function isTooltipClipped(
  x: number,
  y: number,
  viewportWidth: number,
  viewportHeight: number,
  tooltipWidth: number = TOOLTIP_MIN_WIDTH,
  tooltipHeight: number = 60
): boolean {
  // Check left edge
  if (x < 0) return true;

  // Check right edge
  if (x + tooltipWidth > viewportWidth) return true;

  // Check top edge
  if (y < 0) return true;

  // Check bottom edge
  if (y + tooltipHeight > viewportHeight) return true;

  return false;
}

/**
 * Adjusts tooltip position to keep it within viewport bounds
 * Moves tooltip to ensure it's visible and not clipped
 * @param x - Initial left position
 * @param y - Initial top position
 * @param tooltipWidth - Width of the tooltip
 * @param tooltipHeight - Height of the tooltip
 * @param padding - Padding from viewport edges (default 10px)
 * @returns Adjusted coordinates {x, y}
 */
export function adjustTooltipForViewport(
  x: number,
  y: number,
  tooltipWidth: number = TOOLTIP_MIN_WIDTH,
  tooltipHeight: number = 60,
  padding: number = 10
): SelectionCoordinates {
  let adjustedX = x;
  let adjustedY = y;

  const viewportWidth = window.innerWidth;
  const viewportHeight = window.innerHeight;

  // Adjust for left edge clipping
  if (adjustedX < padding) {
    adjustedX = padding;
  }

  // Adjust for right edge clipping
  if (adjustedX + tooltipWidth > viewportWidth - padding) {
    adjustedX = viewportWidth - tooltipWidth - padding;
  }

  // Adjust for top edge clipping
  if (adjustedY < padding) {
    adjustedY = padding;
  }

  // Adjust for bottom edge clipping
  if (adjustedY + tooltipHeight > viewportHeight - padding) {
    adjustedY = viewportHeight - tooltipHeight - padding;
  }

  return {
    x: adjustedX,
    y: adjustedY,
    width: tooltipWidth,
    height: tooltipHeight,
  };
}

/**
 * Determines if we should show tooltip above or below selection
 * @param rect - DOMRect of the selected text
 * @param tooltipHeight - Height of the tooltip
 * @returns true if should position above, false for below
 */
export function shouldTooltipBeAbove(rect: DOMRect, tooltipHeight: number = 60): boolean {
  const spaceAbove = rect.top - tooltipHeight - TOOLTIP_OFFSET;
  const spaceBelow = window.innerHeight - rect.bottom - TOOLTIP_OFFSET;

  return spaceAbove > spaceBelow && spaceAbove > VIEWPORT_THRESHOLD;
}

/**
 * Calculates tooltip dimensions based on screen size
 * Adjusts width for mobile viewports
 * @param selectedTextLength - Length of selected text
 * @returns {width, height} dimensions
 */
export function calculateTooltipDimensions(selectedTextLength: number): {
  width: number;
  height: number;
} {
  const viewportWidth = window.innerWidth;
  const isMobile = viewportWidth < MOBILE_BREAKPOINT;

  // Calculate width based on text length and available space
  let width = Math.min(
    TOOLTIP_MAX_WIDTH,
    Math.max(TOOLTIP_MIN_WIDTH, selectedTextLength * 8 + 60)
  );

  // Constrain to mobile viewport if needed
  if (isMobile) {
    width = Math.min(width, viewportWidth * 0.9);
  }

  // Estimate height (rough calculation)
  const height = isMobile ? 70 : 60;

  return { width, height };
}

/**
 * Gets the best position for tooltip considering multiple constraints
 * @param rect - DOMRect of the selected text
 * @param selectedTextLength - Length of selected text
 * @returns Optimal coordinates for tooltip
 */
export function getBestTooltipPosition(
  rect: DOMRect,
  selectedTextLength: number
): SelectionCoordinates {
  // Calculate dimensions based on content
  const { width, height } = calculateTooltipDimensions(selectedTextLength);

  // Calculate initial position
  let position = calculateTooltipPosition(rect, window.innerHeight, height);

  // Adjust for viewport constraints
  position = adjustTooltipForViewport(position.x, position.y, width, height);

  return position;
}

/**
 * Checks if tooltip is off-screen (for debugging/logging)
 * @param x - Left position
 * @param y - Top position
 * @param width - Tooltip width
 * @param height - Tooltip height
 * @returns true if tooltip is completely off-screen
 */
export function isTooltipOffScreen(
  x: number,
  y: number,
  width: number = TOOLTIP_MIN_WIDTH,
  height: number = 60
): boolean {
  const viewportWidth = window.innerWidth;
  const viewportHeight = window.innerHeight;

  // Check if completely off-screen
  return (
    x + width < 0 ||
    x > viewportWidth ||
    y + height < 0 ||
    y > viewportHeight
  );
}

/**
 * Calculates distance between tooltip center and viewport center
 * Useful for centering tooltip on small screens
 * @param x - Left position
 * @param y - Top position
 * @param width - Tooltip width
 * @param height - Tooltip height
 * @returns Distance in pixels
 */
export function calculateCenterDistance(
  x: number,
  y: number,
  width: number,
  height: number
): number {
  const tooltipCenterX = x + width / 2;
  const tooltipCenterY = y + height / 2;

  const viewportCenterX = window.innerWidth / 2;
  const viewportCenterY = window.innerHeight / 2;

  const dx = tooltipCenterX - viewportCenterX;
  const dy = tooltipCenterY - viewportCenterY;

  return Math.sqrt(dx * dx + dy * dy);
}
