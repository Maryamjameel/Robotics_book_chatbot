/**
 * Constants for selected text feature
 */

/** Maximum length of selected text (characters) */
export const MAX_SELECTED_TEXT_LENGTH = 500;

/** Maximum length of text preview shown in tooltip (characters) */
export const MAX_PREVIEW_LENGTH = 50;

/** Delay before showing tooltip after selection (milliseconds) */
export const TOOLTIP_DISPLAY_DELAY_MS = 0;

/** Debounce delay for selection updates (milliseconds) */
export const DEBOUNCE_DELAY_MS = 50;

/** Debounce delay for repositioning tooltip (milliseconds) */
export const REPOSITION_DEBOUNCE_DELAY_MS = 100;

/** Delay before auto-dismissing tooltip (milliseconds, 0 = no auto-dismiss) */
export const TOOLTIP_AUTO_DISMISS_DELAY_MS = 0;

/** Tooltip z-index */
export const TOOLTIP_Z_INDEX = 1000;

/** Minimum tooltip width (pixels) */
export const TOOLTIP_MIN_WIDTH = 200;

/** Maximum tooltip width (pixels) */
export const TOOLTIP_MAX_WIDTH = 350;

/** Tooltip padding (pixels) */
export const TOOLTIP_PADDING = 12;

/** Minimum button height (pixels, WCAG AA requirement) */
export const MIN_BUTTON_HEIGHT = 48;

/** Minimum button width (pixels, WCAG AA requirement) */
export const MIN_BUTTON_WIDTH = 48;

/** Tooltip arrow height (pixels) */
export const TOOLTIP_ARROW_HEIGHT = 6;

/** Offset from selection to tooltip (pixels) */
export const TOOLTIP_OFFSET = 8;

/** Viewport threshold for repositioning tooltip (pixels) */
export const VIEWPORT_THRESHOLD = 150;

/** Breakpoint for mobile viewport (pixels) */
export const MOBILE_BREAKPOINT = 640;

/** Minimum question length (characters) */
export const MIN_QUESTION_LENGTH = 1;

/** Maximum question length (characters) */
export const MAX_QUESTION_LENGTH = 2000;

/** Color for tooltip background (dark mode) */
export const TOOLTIP_BG_COLOR_DARK = '#1f2937';

/** Color for tooltip background (light mode) */
export const TOOLTIP_BG_COLOR_LIGHT = '#ffffff';

/** Color for tooltip text (dark mode) */
export const TOOLTIP_TEXT_COLOR_DARK = '#f3f4f6';

/** Color for tooltip text (light mode) */
export const TOOLTIP_TEXT_COLOR_LIGHT = '#111827';

/** Color for tooltip border */
export const TOOLTIP_BORDER_COLOR = '#d1d5db';

/** Color for tooltip shadow */
export const TOOLTIP_SHADOW_COLOR = 'rgba(0, 0, 0, 0.1)';

/** Accessibility: WCAG AA minimum contrast ratio */
export const MIN_CONTRAST_RATIO = 4.5;

/** Keyboard key codes */
export const KEY_CODES = {
  ESCAPE: 'Escape',
  ENTER: 'Enter',
  TAB: 'Tab',
} as const;

/** Events for selection detection */
export const SELECTION_EVENTS = ['mouseup', 'touchend'] as const;

/** Events for tooltip dismissal */
export const DISMISSAL_EVENTS = ['keydown', 'click', 'scroll'] as const;

/** Animation duration (milliseconds) */
export const ANIMATION_DURATION_MS = 200;

/** Toast/notification timeout (milliseconds) */
export const NOTIFICATION_TIMEOUT_MS = 3000;

/** Default boost factor for search results */
export const DEFAULT_BOOST_FACTOR = 1.5;

/** Maximum boost factor for search results */
export const MAX_BOOST_FACTOR = 2.0;

/** Minimum boost factor for search results */
export const MIN_BOOST_FACTOR = 1.0;

/** Search latency threshold (milliseconds) */
export const SEARCH_LATENCY_THRESHOLD_MS = 5000;

/** Performance metric collection enabled */
export const ENABLE_METRICS = true;

/** Debug logging enabled */
export const DEBUG_LOGGING = false;
