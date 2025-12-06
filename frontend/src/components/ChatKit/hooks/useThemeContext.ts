import { useState, useEffect } from 'react';

export interface ThemeConfig {
  isDarkMode: boolean;
  cssVariables: {
    primaryColor: string;
    backgroundColor: string;
    fontColorBase: string;
    borderColor: string;
  };
}

const FALLBACK_COLORS = {
  primaryColor: '#3578e5',
  backgroundColor: '#ffffff',
  fontColorBase: '#000000',
  borderColor: '#e8e8e8',
};

const DARK_MODE_FALLBACK_COLORS = {
  primaryColor: '#3578e5',
  backgroundColor: '#1a1a1a',
  fontColorBase: '#ffffff',
  borderColor: '#404040',
};

/**
 * Detects and resolves CSS variables from Docusaurus theme
 */
function resolveCSSVariables(isDarkMode: boolean): ThemeConfig['cssVariables'] {
  if (typeof window === 'undefined') {
    return isDarkMode ? DARK_MODE_FALLBACK_COLORS : FALLBACK_COLORS;
  }

  const htmlElement = document.documentElement;
  const computedStyle = getComputedStyle(htmlElement);

  const cssVariables: ThemeConfig['cssVariables'] = {
    primaryColor: computedStyle.getPropertyValue('--ifm-color-primary').trim() || FALLBACK_COLORS.primaryColor,
    backgroundColor: computedStyle.getPropertyValue('--ifm-background-color').trim() || (isDarkMode ? DARK_MODE_FALLBACK_COLORS.backgroundColor : FALLBACK_COLORS.backgroundColor),
    fontColorBase: computedStyle.getPropertyValue('--ifm-font-color-base').trim() || (isDarkMode ? DARK_MODE_FALLBACK_COLORS.fontColorBase : FALLBACK_COLORS.fontColorBase),
    borderColor: computedStyle.getPropertyValue('--ifm-color-emphasis-200').trim() || (isDarkMode ? DARK_MODE_FALLBACK_COLORS.borderColor : FALLBACK_COLORS.borderColor),
  };

  return cssVariables;
}

/**
 * Detects current theme mode from Docusaurus
 * Listens to html[data-theme] attribute changes
 */
function detectThemeMode(): boolean {
  if (typeof window === 'undefined') {
    return false;
  }
  const htmlElement = document.documentElement;
  const theme = htmlElement.getAttribute('data-theme');
  return theme === 'dark';
}

/**
 * Hook for detecting and responding to Docusaurus theme changes
 * Uses MutationObserver to detect when the theme attribute changes
 *
 * @returns Current theme configuration (isDarkMode + CSS variables)
 */
export function useThemeContext(): ThemeConfig {
  const [isDarkMode, setIsDarkMode] = useState(() => detectThemeMode());
  const [cssVariables, setCSSVariables] = useState(() =>
    resolveCSSVariables(detectThemeMode())
  );

  useEffect(() => {
    if (typeof window === 'undefined') {
      return;
    }

    const htmlElement = document.documentElement;

    // Create MutationObserver to listen for theme attribute changes
    const observer = new MutationObserver(() => {
      const newIsDarkMode = detectThemeMode();
      setIsDarkMode(newIsDarkMode);
      setCSSVariables(resolveCSSVariables(newIsDarkMode));
    });

    // Start observing the html element for attribute changes
    observer.observe(htmlElement, {
      attributes: true,
      attributeFilter: ['data-theme'],
    });

    return () => {
      observer.disconnect();
    };
  }, []);

  return {
    isDarkMode,
    cssVariables,
  };
}

/**
 * Apply theme colors to an element
 * Utility function for applying theme CSS variables to elements
 */
export function applyThemeToElement(
  element: HTMLElement,
  theme: ThemeConfig
): void {
  if (!element) {
    return;
  }

  element.style.setProperty('--theme-primary-color', theme.cssVariables.primaryColor);
  element.style.setProperty('--theme-background-color', theme.cssVariables.backgroundColor);
  element.style.setProperty('--theme-font-color', theme.cssVariables.fontColorBase);
  element.style.setProperty('--theme-border-color', theme.cssVariables.borderColor);
}
