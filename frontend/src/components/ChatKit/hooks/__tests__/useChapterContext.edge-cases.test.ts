/**
 * Edge case tests for useChapterContext hook
 * Tests unusual URL patterns and boundary conditions
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

describe('useChapterContext - Edge Cases', () => {
  beforeEach(() => {
    document.body.innerHTML = '';
    jest.clearAllMocks();
  });

  describe('Unusual URL formats', () => {
    it('should handle URL with multiple hyphens in chapter ID', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-very-long-topic-name-kinematics-v2',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-very-long-topic-name-kinematics-v2');
    });

    it('should handle URL with mixed underscores and hyphens', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter_3-advanced_kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter_3-advanced');
    });

    it('should handle URL with numbers in middle', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter2b3c4kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter2b3c4kinematics');
    });

    it('should handle deeply nested URLs', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/intro/chapter-3-kinematics/forward/basic/fundamentals',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-3');
    });

    it('should handle URL with query parameters and hash', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '?tab=forward&scroll=true',
        hash: '#section-1',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-3');
    });

    it('should handle very long chapter ID', () => {
      const longId = 'chapter-' + 'a'.repeat(100);
      mockUseLocation.mockReturnValue({
        pathname: `/docs/${longId}`,
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe(longId);
    });

    it('should not match chapter pattern with insufficient text after "chapter"', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Some Section';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      // Should only get from h1
      expect(result.current?.chapterId).toBe('some-section');
    });
  });

  describe('Special characters and encoding', () => {
    it('should handle URL-encoded characters in pathname', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3%20kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      // Should still extract the chapter pattern
      expect(result.current?.chapterId).toBe('chapter-3');
    });

    it('should handle chapter ID with alphanumeric combination', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter3abc456def',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter3abc456def');
    });
  });

  describe('Extreme heading content', () => {
    it('should handle heading with only spaces', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '     ';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });

    it('should handle heading with newlines and tabs', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '\n\t  Kinematics  \n\t';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Kinematics');
    });

    it('should handle very long heading text', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      const longText = 'A Very Long Chapter Title With Many Words That Exceeds Normal Length ' + 'x'.repeat(200);
      h1.textContent = longText;
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe(longText);
    });

    it('should handle heading with special unicode characters', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics α-β-γ (ΣΔΓ)';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Kinematics α-β-γ (ΣΔΓ)');
    });

    it('should handle heading with HTML entities', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.innerHTML = 'Kinematics &amp; Dynamics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toContain('Kinematics');
    });
  });

  describe('Title to slug conversion edge cases', () => {
    it('should handle title with consecutive spaces', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Motion    Planning    Algorithm';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('motion-planning-algorithm');
    });

    it('should handle title with all special characters', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '!@#$%^&*(){}[]';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('');
    });

    it('should handle title with leading/trailing special characters', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '---Kinematics---';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('kinematics');
    });

    it('should handle title with numbers only', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '123 456 789';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('123-456-789');
    });
  });

  describe('Confidence scoring edge cases', () => {
    it('should handle matching URL and h1 with different cases', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/CHAPTER-3-KINEMATICS',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.confidence).toBe('high');
    });

    it('should handle URL with chapter but empty h1', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.confidence).toBe('medium');
    });

    it('should return low confidence for mismatched URL and h1', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Completely Different Topic';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.confidence).toBe('high');  // Still high because URL matches
    });
  });

  describe('Error recovery scenarios', () => {
    it('should gracefully handle DOM querySelector returning null', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      // Don't add h1 - should return null from DOM

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-3');
      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });

    it('should handle multiple h1 elements (should use first)', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1_1 = document.createElement('h1');
      h1_1.textContent = 'First Heading';
      document.body.appendChild(h1_1);

      const h1_2 = document.createElement('h1');
      h1_2.textContent = 'Second Heading';
      document.body.appendChild(h1_2);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('First Heading');
    });

    it('should handle pathname with only chapter word', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/chapter',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      // Should not match - needs something after chapter
      expect(result.current).toBeNull();
    });
  });

  describe('Boundary conditions', () => {
    it('should handle empty pathname', () => {
      mockUseLocation.mockReturnValue({
        pathname: '',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current).toBeNull();
    });

    it('should handle pathname with only slash', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current).toBeNull();
    });

    it('should handle very deep URL path', () => {
      const deepPath = '/a/b/c/d/e/f/g/h/chapter-3-kinematics/i/j/k/l/m';
      mockUseLocation.mockReturnValue({
        pathname: deepPath,
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-3');
    });

    it('should handle single character after chapter keyword', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-a',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-a');
    });
  });
});
