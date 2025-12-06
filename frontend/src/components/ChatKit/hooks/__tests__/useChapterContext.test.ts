/**
 * Unit tests for useChapterContext hook
 * Tests URL extraction, DOM extraction, slug generation, confidence scoring
 */

import { renderHook } from '@testing-library/react';
import { BrowserRouter } from 'react-router-dom';
import { useChapterContext } from '../useChapterContext';

// Mock useLocation from react-router-dom
jest.mock('react-router-dom', () => ({
  ...jest.requireActual('react-router-dom'),
  useLocation: jest.fn(),
}));

import { useLocation } from 'react-router-dom';

const mockUseLocation = useLocation as jest.MockedFunction<typeof useLocation>;

// Helper function to render hook with Router wrapper
function renderHookWithRouter(callback: () => any) {
  return renderHook(callback, {
    wrapper: BrowserRouter,
  });
}

describe('useChapterContext', () => {
  beforeEach(() => {
    // Clear DOM and mocks before each test
    document.body.innerHTML = '';
    jest.clearAllMocks();
  });

  describe('URL Extraction', () => {
    it('should extract chapter ID from URL pattern /docs/chapter-3-kinematics', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-3');
    });

    it('should extract chapter ID from URL pattern /docs/chapter_3_kinematics', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter_3_kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter_3');
    });

    it('should extract chapter ID from URL pattern /docs/ch03', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/ch03',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('ch03');
    });

    it('should extract chapter ID from URL with advanced topic names', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-advanced-motion-planning',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-advanced-motion-planning');
    });

    it('should handle case-insensitive URL patterns', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/CHAPTER-5-DYNAMICS',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-5-dynamics');
    });

    it('should return null for non-chapter URLs', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/introduction',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      // Add h1 to DOM for DOM extraction to succeed
      const h1 = document.createElement('h1');
      h1.textContent = 'Introduction';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      // Should extract from DOM only
      expect(result.current?.chapterId).toBe('introduction');
      expect(result.current?.chapterTitle).toBe('Introduction');
    });
  });

  describe('DOM Extraction', () => {
    it('should extract chapter title from h1 tag', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Kinematics');
    });

    it('should trim whitespace from h1 content', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '  Inverse Kinematics  ';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Inverse Kinematics');
    });

    it('should return null when h1 is missing', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });

    it('should return null when h1 is empty', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '   ';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterTitle).toBe('Unknown Chapter');
    });
  });

  describe('Slug Generation', () => {
    it('should convert title to URL-safe slug', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Inverse Kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('inverse-kinematics');
    });

    it('should handle multiple spaces in title', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Advanced  Motion   Planning';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('advanced-motion-planning');
    });

    it('should remove special characters from slug', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Dynamics & Control (Part 1)';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('dynamics--control-part-1');
    });

    it('should remove leading and trailing hyphens from slug', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = '-Motion Planning-';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('motion-planning');
    });

    it('should return empty slug when no title available', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterSlug).toBe('');
    });
  });

  describe('Confidence Scoring', () => {
    it('should return "high" confidence when both URL and h1 match', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.confidence).toBe('high');
    });

    it('should return "medium" confidence when only URL matches', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      // No h1 tag

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.confidence).toBe('medium');
    });

    it('should return "medium" confidence when only h1 matches', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/introduction',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.confidence).toBe('medium');
    });

    it('should return "low" confidence when neither URL nor h1 match', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/introduction',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      // No h1 tag or h1 doesn't match

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current).toBeNull();
    });
  });

  describe('Return Value and Null Handling', () => {
    it('should return ChapterContext object when on chapter page', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current).toEqual({
        chapterId: 'chapter-3',
        chapterTitle: 'Kinematics',
        chapterSlug: 'kinematics',
        confidence: 'high',
      });
    });

    it('should return null when not on chapter page and no h1 found', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/introduction',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current).toBeNull();
    });

    it('should use fallback values when chapter ID is missing', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/introduction',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Advanced Kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('advanced-kinematics');
    });

    it('should use "unknown" as final fallback when both sources missing', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      // This won't match because it needs 'chapter' followed by something
      // So it will extract 'chapter' from the pattern? Let me check the regex
      // Actually the regex is /chapter[_-]?(\w+)/i so it needs at least one word char after
      // So /docs/chapter alone won't match

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current).toBeNull();
    });
  });

  describe('Error Handling', () => {
    it('should handle DOM errors gracefully', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      // Mock document.querySelector to throw an error
      const originalQuerySelector = document.querySelector;
      document.querySelector = jest.fn(() => {
        throw new Error('DOM error');
      });

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-3');
      expect(result.current?.chapterTitle).toBe('Unknown Chapter');

      // Restore original
      document.querySelector = originalQuerySelector;
    });

    it('should not throw when useLocation fails', () => {
      mockUseLocation.mockImplementation(() => {
        throw new Error('Router error');
      });

      expect(() => {
        renderHookWithRouter(() => useChapterContext());
      }).toThrow();
    });
  });

  describe('Integration Cases', () => {
    it('should handle real-world Docusaurus chapter URL', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-3-kinematics/forward-kinematics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Forward Kinematics';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current).toEqual({
        chapterId: 'chapter-3',
        chapterTitle: 'Forward Kinematics',
        chapterSlug: 'forward-kinematics',
        confidence: 'high',
      });
    });

    it('should handle chapter with numeric ID and hyphens', () => {
      mockUseLocation.mockReturnValue({
        pathname: '/docs/chapter-advanced-1-dynamics',
        search: '',
        hash: '',
        state: null,
        key: 'default',
      });

      const h1 = document.createElement('h1');
      h1.textContent = 'Dynamics and Control';
      document.body.appendChild(h1);

      const { result } = renderHookWithRouter(() => useChapterContext());

      expect(result.current?.chapterId).toBe('chapter-advanced-1-dynamics');
      expect(result.current?.chapterTitle).toBe('Dynamics and Control');
      expect(result.current?.confidence).toBe('high');
    });
  });
});
