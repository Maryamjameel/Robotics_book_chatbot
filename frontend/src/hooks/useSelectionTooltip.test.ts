/**
 * Unit tests for useSelectionTooltip hook
 * Tests tooltip visibility, positioning, and dismissal logic
 */

import { renderHook, act } from '@testing-library/react';
import { useSelectionTooltip } from './useSelectionTooltip';
import type { TextSelection, SelectionTooltipState } from '../types/selected-text.types';

describe('useSelectionTooltip hook', () => {
  const mockSelection: TextSelection = {
    text: 'selected text',
    x: 100,
    y: 200,
    timestamp: Date.now(),
  };

  it('should initialize with hidden state', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    expect(result.current.state.isVisible).toBe(false);
    expect(result.current.state.isDismissed).toBe(false);
    expect(result.current.state.selection).toBeNull();
    expect(result.current.state.position).toEqual({ x: 0, y: 0 });
  });

  it('should show tooltip with selection and position', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.show(mockSelection);
    });

    expect(result.current.state.isVisible).toBe(true);
    expect(result.current.state.selection).toEqual(mockSelection);
    expect(result.current.state.position).toEqual({ x: 100, y: 200 });
  });

  it('should hide tooltip', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.show(mockSelection);
    });

    expect(result.current.state.isVisible).toBe(true);

    act(() => {
      result.current.hide();
    });

    expect(result.current.state.isVisible).toBe(false);
    expect(result.current.state.selection).toEqual(mockSelection);
  });

  it('should dismiss tooltip and prevent reappearing', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.show(mockSelection);
    });

    expect(result.current.state.isVisible).toBe(true);
    expect(result.current.state.isDismissed).toBe(false);

    act(() => {
      result.current.dismiss();
    });

    expect(result.current.state.isVisible).toBe(false);
    expect(result.current.state.isDismissed).toBe(true);
  });

  it('should reset all state', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.show(mockSelection);
    });

    expect(result.current.state.isVisible).toBe(true);

    act(() => {
      result.current.reset();
    });

    expect(result.current.state.isVisible).toBe(false);
    expect(result.current.state.isDismissed).toBe(false);
    expect(result.current.state.selection).toBeNull();
    expect(result.current.state.position).toEqual({ x: 0, y: 0 });
  });

  it('should update tooltip position', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.show(mockSelection);
    });

    const newPosition = { x: 150, y: 250 };

    act(() => {
      result.current.updatePosition(newPosition);
    });

    expect(result.current.state.position).toEqual(newPosition);
    expect(result.current.state.isVisible).toBe(true);
  });

  it('should not show tooltip if dismissed', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.dismiss();
    });

    act(() => {
      result.current.show(mockSelection);
    });

    // Tooltip should still be considered "dismissed" state
    expect(result.current.state.isDismissed).toBe(true);
  });

  it('should return correct state shape', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.show(mockSelection);
    });

    const state: SelectionTooltipState = result.current.state;
    expect(state).toHaveProperty('isVisible');
    expect(state).toHaveProperty('isDismissed');
    expect(state).toHaveProperty('selection');
    expect(state).toHaveProperty('position');
  });

  it('should handle multiple show calls', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    const selection1: TextSelection = { ...mockSelection, text: 'first', x: 100, y: 200 };
    const selection2: TextSelection = { ...mockSelection, text: 'second', x: 150, y: 250 };

    act(() => {
      result.current.show(selection1);
    });

    expect(result.current.state.selection?.text).toBe('first');
    expect(result.current.state.position).toEqual({ x: 100, y: 200 });

    act(() => {
      result.current.show(selection2);
    });

    expect(result.current.state.selection?.text).toBe('second');
    expect(result.current.state.position).toEqual({ x: 150, y: 250 });
  });

  it('should maintain visibility after position update', () => {
    const { result } = renderHook(() => useSelectionTooltip());

    act(() => {
      result.current.show(mockSelection);
    });

    act(() => {
      result.current.updatePosition({ x: 200, y: 300 });
    });

    expect(result.current.state.isVisible).toBe(true);
    expect(result.current.state.position).toEqual({ x: 200, y: 300 });
  });
});
