/**
 * DOM extraction edge case tests for useChapterContext hook
 * Tests DOM-related boundary conditions and error scenarios
 */

import { renderHook } from '@testing-library/react';
import { BrowserRouter } from 'react-router-dom';
import { useChapterContext } from '../useChapterContext';

jest.mock('react-router-dom', () => ({
  ...jest.requireActual('react-router-dom'),
  useLocation: jest.fn(),
}));

import { useLocation } from 'react-router-dom';

const mockUseLocation = useLocation as jest.MockedFunction<typeof useLocation>;

function renderHookWithRouter(callback: () => any) {
  return renderHook(callback, {
    wrapper: BrowserRouter,
  });
}

describe('useChapterContext - DOM Extraction Edge Cases', () => {
  beforeEach(() => {
    document.body.innerHTML = '';
    jest.clearAllMocks();
  });

  describe('Missing or malformed h1 tags', () => {
    it('should handle h1 with textContent property missing', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      // Don't set textContent, leave it undefined
      Object.defineProperty(h1, 'textContent', {
        value: undefined,
        writable: false,
      });
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });

    it('should handle h1 with null textContent', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = null;
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });

    it('should handle h1 that is comment node', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const comment = document.createComment('This is a comment');
      document.body.appendChild(comment);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });
  });

  describe('H1 content with nested elements', () => {
    it('should handle h1 with nested span elements', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      const span = document.createElement('span');
      span.textContent = 'Kinematics';
      h1.appendChild(span);
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Kinematics');
    });

    it('should handle h1 with multiple nested elements', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      const span1 = document.createElement('span');
      span1.textContent = 'Forward ';
      const span2 = document.createElement('span');
      span2.textContent = 'Kinematics';
      h1.appendChild(span1);
      h1.appendChild(span2);
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Forward Kinematics');
    });

    it('should handle h1 with hidden child elements', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      const visible = document.createElement('span');
      visible.textContent = 'Kinematics';
      const hidden = document.createElement('span');
      hidden.style.display = 'none';
      hidden.textContent = ' (Hidden)';
      h1.appendChild(visible);
      h1.appendChild(hidden);
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toContain('Kinematics');
    });

    it('should handle h1 with script elements inside', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      const script = document.createElement('script');
      script.textContent = 'console.log("test")';
      h1.appendChild(script);
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      // textContent should still return proper value
      expect(result.current?.chapterTitle).toBe('Kinematics');
    });
  });

  describe('H1 visibility and accessibility', () => {
    it('should handle hidden h1 element', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      h1.style.display = 'none';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      // Should still extract even if hidden (textContent is available)
      expect(result.current?.chapterTitle).toBe('Kinematics');
    });

    it('should handle h1 with aria-hidden attribute', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      h1.setAttribute('aria-hidden', 'true');
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      // Should extract regardless of aria-hidden
      expect(result.current?.chapterTitle).toBe('Kinematics');
    });

    it('should handle h1 with zero opacity', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      h1.style.opacity = '0';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Kinematics');
    });
  });

  describe('Large and unusual h1 content', () => {
    it('should handle h1 with 10000+ characters', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'K'.repeat(10000);
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('K'.repeat(10000));
    });

    it('should handle h1 with only whitespace variations', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '\u00A0\u2003\u2009\u200B';  // Various whitespace chars
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      // These are whitespace-like but may not be trimmed by trim()
      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });

    it('should handle h1 with line breaks', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Forward\nKinematics\nAlgorithm';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Forward\nKinematics\nAlgorithm');
    });
  });

  describe('H1 in different document positions', () => {
    it('should find h1 at document root', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Root Heading';
      document.documentElement.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Root Heading');
    });

    it('should find h1 deep in nested structure', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const div1 = document.createElement('div');
      const div2 = document.createElement('div');
      const div3 = document.createElement('div');
      const h1 = document.createElement('h1');
      h1.textContent = 'Deeply Nested';
      div3.appendChild(h1);
      div2.appendChild(div3);
      div1.appendChild(div2);
      document.body.appendChild(div1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Deeply Nested');
    });

    it('should use first h1 when multiple exist at different levels', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1_first = document.createElement('h1');
      h1_first.textContent = 'First H1';
      document.body.appendChild(h1_first);

      const div = document.createElement('div');
      const h1_nested = document.createElement('h1');
      h1_nested.textContent = 'Nested H1';
      div.appendChild(h1_nested);
      document.body.appendChild(div);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('First H1');
    });
  });

  describe('Document mutation scenarios', () => {
    it('should handle h1 being removed after initial render', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Original Title';
      document.body.appendChild(h1);

      const { result: result1 } = renderHookWithRouter(() => useChapterContext());
      expect(result1.current?.chapterTitle).toBe('Original Title');

      // Remove h1 and render again
      document.body.removeChild(h1);
      const { result: result2 } = renderHookWithRouter(() => useChapterContext());
      expect(result2.current?.chapterTitle).toBe('Unknown Chapter');
    });

    it('should handle h1 text content change', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Original';
      document.body.appendChild(h1);

      const { result: result1 } = renderHookWithRouter(() => useChapterContext());
      expect(result1.current?.chapterTitle).toBe('Original');

      // Change h1 content
      h1.textContent = 'Updated Title';
      const { result: result2 } = renderHookWithRouter(() => useChapterContext());
      expect(result2.current?.chapterTitle).toBe('Updated Title');
    });

    it('should handle h1 being replaced with new element', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1_old = document.createElement('h1');
      h1_old.textContent = 'Old Title';
      document.body.appendChild(h1_old);

      document.body.removeChild(h1_old);

      const h1_new = document.createElement('h1');
      h1_new.textContent = 'New Title';
      document.body.appendChild(h1_new);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('New Title');
    });
  });

  describe('DOM error handling', () => {
    it('should handle querySelector throwing error', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const originalQuerySelector = document.querySelector;
      document.querySelector = jest.fn(() => {
        throw new Error('DOM query failed');
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');

      document.querySelector = originalQuerySelector;
    });

    it('should handle textContent getter throwing error', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      Object.defineProperty(h1, 'textContent', {
        get: () => {
          throw new Error('textContent access failed');
        },
      });
      document.body.appendChild(h1);

      const originalQuerySelector = document.querySelector;
      document.querySelector = jest.fn().mockReturnValue(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');

      document.querySelector = originalQuerySelector;
    });
  });
});
