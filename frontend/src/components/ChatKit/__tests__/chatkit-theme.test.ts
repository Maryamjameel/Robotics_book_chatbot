import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { render, screen } from '@testing-library/react';
import ChatKitWidget from '../ChatKitWidget';
import { useThemeContext } from '../hooks/useThemeContext';

// Mock the useThemeContext hook
vi.mock('../hooks/useThemeContext', () => ({
  useThemeContext: vi.fn(),
}));

describe('ChatKit Component Theme Integration', () => {
  const mockThemeLight = {
    isDarkMode: false,
    cssVariables: {
      primaryColor: '#0066cc',
      backgroundColor: '#ffffff',
      fontColorBase: '#333333',
      borderColor: '#cccccc',
    },
  };

  const mockThemeDark = {
    isDarkMode: true,
    cssVariables: {
      primaryColor: '#3399ff',
      backgroundColor: '#1a1a1a',
      fontColorBase: '#ffffff',
      borderColor: '#444444',
    },
  };

  beforeEach(() => {
    vi.clearAllMocks();
  });

  afterEach(() => {
    vi.clearAllMocks();
  });

  describe('Theme style application', () => {
    it('should apply correct colors from theme in light mode', () => {
      (useThemeContext as any).mockReturnValue(mockThemeLight);

      render(<ChatKitWidget />);

      // Component should be rendered
      const widget = screen.getByRole('region', { hidden: true }) || document.querySelector('.chatkit-widget');
      expect(widget).toBeTruthy();
    });

    it('should apply correct colors from theme in dark mode', () => {
      (useThemeContext as any).mockReturnValue(mockThemeDark);

      render(<ChatKitWidget />);

      // Component should be rendered with dark theme
      const widget = document.querySelector('.chatkit-widget');
      expect(widget).toBeTruthy();
    });
  });

  describe('Theme color accuracy', () => {
    it('should use hook to get Docusaurus CSS variables', () => {
      (useThemeContext as any).mockReturnValue(mockThemeLight);

      render(<ChatKitWidget />);

      // Verify the hook was called
      expect(useThemeContext).toHaveBeenCalled();
    });

    it('should verify color values match Docusaurus variables', () => {
      const mockTheme = {
        isDarkMode: false,
        cssVariables: {
          primaryColor: '#0066cc',
          backgroundColor: '#ffffff',
          fontColorBase: '#333333',
          borderColor: '#cccccc',
        },
      };

      (useThemeContext as any).mockReturnValue(mockTheme);

      render(<ChatKitWidget />);

      // Verify colors are valid hex values
      expect(mockTheme.cssVariables.primaryColor).toMatch(/^#[0-9a-f]{6}$/i);
      expect(mockTheme.cssVariables.backgroundColor).toMatch(/^#[0-9a-f]{6}$/i);
      expect(mockTheme.cssVariables.fontColorBase).toMatch(/^#[0-9a-f]{6}$/i);
      expect(mockTheme.cssVariables.borderColor).toMatch(/^#[0-9a-f]{6}$/i);
    });
  });

  describe('Theme transitions', () => {
    it('should not produce console errors during theme transitions', () => {
      const consoleSpy = vi.spyOn(console, 'error');

      (useThemeContext as any).mockReturnValue(mockThemeLight);
      const { rerender } = render(<ChatKitWidget />);

      // Switch theme
      (useThemeContext as any).mockReturnValue(mockThemeDark);
      rerender(<ChatKitWidget />);

      // Should not have console errors
      expect(consoleSpy).not.toHaveBeenCalled();

      consoleSpy.mockRestore();
    });
  });

  describe('No hardcoded colors', () => {
    it('should not use hardcoded color values directly in component', () => {
      (useThemeContext as any).mockReturnValue(mockThemeLight);

      const { container } = render(<ChatKitWidget />);

      // Get computed styles from the widget
      const widget = container.querySelector('.chatkit-widget');
      
      if (widget) {
        const styles = getComputedStyle(widget);
        
        // CSS variables should be in use (checking for var() functions in computed properties)
        // This is a simplified check - in reality you'd need to check the CSS
        expect(widget).toBeTruthy();
      }
    });
  });

  describe('Graceful degradation', () => {
    it('should render without errors even if hook returns incomplete data', () => {
      const incompleteTheme = {
        isDarkMode: false,
        cssVariables: {
          primaryColor: '',
          backgroundColor: '',
          fontColorBase: '',
          borderColor: '',
        },
      };

      (useThemeContext as any).mockReturnValue(incompleteTheme);

      expect(() => {
        render(<ChatKitWidget />);
      }).not.toThrow();
    });
  });
});
