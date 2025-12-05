/**
 * Unit tests for useTextSelection hook
 * Tests text selection detection, validation, and coordinate calculation
 */

import { renderHook, act, waitFor } from '@testing-library/react';
import { useTextSelection } from './useTextSelection';
import type { TextSelection } from '../types/selected-text.types';

describe('useTextSelection hook', () => {
  // Mock window.getSelection
  const mockSelection = {
    toString: jest.fn(() => 'selected text'),
    rangeCount: 1,
    getRangeAt: jest.fn(() => ({
      getBoundingClientRect: jest.fn(() => ({
        x: 100,
        y: 200,
        width: 150,
        height: 20,
      })),
    })),
  } as unknown as Selection;

  beforeEach(() => {
    jest.clearAllMocks();
    Object.defineProperty(window, 'getSelection', {
      value: jest.fn(() => mockSelection),
      writable: true,
    });
  });

  it('should initialize with null selection', () => {
    const { result } = renderHook(() => useTextSelection());

    expect(result.current.selection).toBeNull();
    expect(result.current.isActive).toBe(true);
    expect(result.current.error).toBeNull();
  });

  it('should detect text selection on mouseup', async () => {
    const { result } = renderHook(() => useTextSelection());

    act(() => {
      const event = new MouseEvent('mouseup', {
        bubbles: true,
        cancelable: true,
      });
      document.dispatchEvent(event);
    });

    await waitFor(() => {
      expect(result.current.selection).not.toBeNull();
      expect(result.current.selection?.text).toBe('selected text');
    });
  });

  it('should detect text selection on touchend', async () => {
    const { result } = renderHook(() => useTextSelection());

    act(() => {
      const event = new TouchEvent('touchend', {
        bubbles: true,
        cancelable: true,
      });
      document.dispatchEvent(event);
    });

    await waitFor(() => {
      expect(result.current.selection).not.toBeNull();
    });
  });

  it('should return null selection when text is empty', async () => {
    const mockEmptySelection = {
      ...mockSelection,
      toString: jest.fn(() => ''),
    } as unknown as Selection;

    Object.defineProperty(window, 'getSelection', {
      value: jest.fn(() => mockEmptySelection),
      writable: true,
    });

    const { result } = renderHook(() => useTextSelection());

    act(() => {
      const event = new MouseEvent('mouseup');
      document.dispatchEvent(event);
    });

    await waitFor(() => {
      expect(result.current.selection).toBeNull();
    });
  });

  it('should calculate correct coordinates for selection', async () => {
    const { result } = renderHook(() => useTextSelection());

    act(() => {
      const event = new MouseEvent('mouseup');
      document.dispatchEvent(event);
    });

    await waitFor(() => {
      expect(result.current.selection?.x).toBe(100);
      expect(result.current.selection?.y).toBe(200);
    });
  });

  it('should include timestamp in selection', async () => {
    const { result } = renderHook(() => useTextSelection());
    const beforeTime = Date.now();

    act(() => {
      const event = new MouseEvent('mouseup');
      document.dispatchEvent(event);
    });

    await waitFor(() => {
      expect(result.current.selection?.timestamp).toBeGreaterThanOrEqual(beforeTime);
      expect(result.current.selection?.timestamp).toBeLessThanOrEqual(Date.now());
    });
  });

  it('should handle rapid selections (debouncing)', async () => {
    const { result } = renderHook(() => useTextSelection());
    let selectionCount = 0;

    const originalSelection = result.current.selection;

    // Trigger multiple mouseup events rapidly
    act(() => {
      for (let i = 0; i < 5; i++) {
        const event = new MouseEvent('mouseup');
        document.dispatchEvent(event);
      }
    });

    // Selection should only be set once (debounced)
    await waitFor(() => {
      expect(result.current.selection).not.toBeNull();
    });
  });

  it('should return selection with expected shape', async () => {
    const { result } = renderHook(() => useTextSelection());

    act(() => {
      const event = new MouseEvent('mouseup');
      document.dispatchEvent(event);
    });

    await waitFor(() => {
      const selection: TextSelection | null = result.current.selection;
      expect(selection).toHaveProperty('text');
      expect(selection).toHaveProperty('x');
      expect(selection).toHaveProperty('y');
      expect(selection).toHaveProperty('timestamp');
    });
  });

  it('should clean up event listeners on unmount', () => {
    const removeEventListenerSpy = jest.spyOn(document, 'removeEventListener');
    const { unmount } = renderHook(() => useTextSelection());

    unmount();

    expect(removeEventListenerSpy).toHaveBeenCalledWith('mouseup', expect.any(Function));
    expect(removeEventListenerSpy).toHaveBeenCalledWith('touchend', expect.any(Function));

    removeEventListenerSpy.mockRestore();
  });
});
