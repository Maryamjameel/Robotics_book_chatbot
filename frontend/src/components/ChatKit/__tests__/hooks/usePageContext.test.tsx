/**
 * Unit Tests for usePageContext Hook
 * Tests page context extraction and pathname change detection
 */

import { describe, it, expect, beforeEach, vi } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { usePageContext } from '../../hooks/usePageContext';
import * as pageContextService from '../../services/pageContextService';

vi.mock('../../services/pageContextService');

describe('usePageContext Hook', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  describe('initialization', () => {
    it('should extract page context on mount', () => {
      const mockContext = {
        url: 'http://localhost:3000/docs/chapter-3',
        pathname: '/docs/chapter-3',
        chapter: 'Chapter 3',
        section: 'Kinematics',
        confidence: 'high' as const,
      };

      vi.mocked(pageContextService.getPageContext).mockReturnValueOnce(
        mockContext
      );
      vi.mocked(pageContextService.watchPageContextChanges).mockReturnValueOnce(
        () => {}
      );

      const { result } = renderHook(() => usePageContext());

      expect(result.current).toEqual(mockContext);
    });

    it('should set up watch for context changes', () => {
      const mockContext = {
        url: 'http://localhost:3000/docs',
        pathname: '/docs',
        confidence: 'low' as const,
      };

      vi.mocked(pageContextService.getPageContext).mockReturnValueOnce(
        mockContext
      );
      vi.mocked(pageContextService.watchPageContextChanges).mockReturnValueOnce(
        () => {}
      );

      renderHook(() => usePageContext());

      expect(pageContextService.watchPageContextChanges).toHaveBeenCalled();
    });
  });

  describe('context updates', () => {
    it('should update context when pathname changes', () => {
      const initialContext = {
        url: 'http://localhost:3000/docs/chapter-1',
        pathname: '/docs/chapter-1',
        chapter: 'Chapter 1',
        confidence: 'high' as const,
      };

      const updatedContext = {
        url: 'http://localhost:3000/docs/chapter-2',
        pathname: '/docs/chapter-2',
        chapter: 'Chapter 2',
        confidence: 'high' as const,
      };

      let changeCallback: ((context: any) => void) | null = null;

      vi.mocked(pageContextService.getPageContext).mockReturnValueOnce(
        initialContext
      );
      vi.mocked(pageContextService.watchPageContextChanges).mockImplementationOnce(
        (callback) => {
          changeCallback = callback;
          return () => {};
        }
      );

      const { result, rerender } = renderHook(() => usePageContext());

      expect(result.current).toEqual(initialContext);

      // Simulate pathname change
      if (changeCallback) {
        act(() => {
          changeCallback!(updatedContext);
        });
      }

      expect(result.current).toEqual(updatedContext);
    });
  });

  describe('cleanup', () => {
    it('should call unsubscribe cleanup on unmount', () => {
      const mockUnsubscribe = vi.fn();
      const mockContext = {
        url: 'http://localhost:3000/docs',
        pathname: '/docs',
        confidence: 'low' as const,
      };

      vi.mocked(pageContextService.getPageContext).mockReturnValueOnce(
        mockContext
      );
      vi.mocked(pageContextService.watchPageContextChanges).mockReturnValueOnce(
        mockUnsubscribe
      );

      const { unmount } = renderHook(() => usePageContext());

      unmount();

      expect(mockUnsubscribe).toHaveBeenCalled();
    });
  });
});
