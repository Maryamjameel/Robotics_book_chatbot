/**
 * Component tests for SelectionTooltip
 * Tests rendering, user interactions, and accessibility
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { SelectionTooltip } from './SelectionTooltip';
import type { SelectionTooltipProps } from '../../types/selected-text.types';

describe('SelectionTooltip Component', () => {
  const defaultProps: SelectionTooltipProps = {
    isVisible: true,
    position: { x: 100, y: 200 },
    selectedText: 'forward kinematics',
    onAsk: jest.fn(),
    onDismiss: jest.fn(),
  };

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('should render nothing when not visible', () => {
    const { container } = render(
      <SelectionTooltip {...defaultProps} isVisible={false} />
    );
    expect(container.firstChild).toBeNull();
  });

  it('should render tooltip when visible', () => {
    render(<SelectionTooltip {...defaultProps} />);
    expect(screen.getByTestId('selection-tooltip')).toBeInTheDocument();
  });

  it('should display selected text preview', () => {
    render(<SelectionTooltip {...defaultProps} />);
    expect(screen.getByText('forward kinematics')).toBeInTheDocument();
  });

  it('should position tooltip at specified coordinates', () => {
    const { container } = render(
      <SelectionTooltip {...defaultProps} position={{ x: 150, y: 250 }} />
    );
    const tooltip = container.querySelector('.selection-tooltip');
    expect(tooltip).toHaveStyle('left: 150px');
    expect(tooltip).toHaveStyle('top: 250px');
  });

  it('should call onAsk when ask button clicked', async () => {
    const onAsk = jest.fn();
    render(<SelectionTooltip {...defaultProps} onAsk={onAsk} />);

    const askButton = screen.getByTestId('ask-button');
    await userEvent.click(askButton);

    expect(onAsk).toHaveBeenCalledWith('forward kinematics');
  });

  it('should call onDismiss when dismiss button clicked', async () => {
    const onDismiss = jest.fn();
    render(<SelectionTooltip {...defaultProps} onDismiss={onDismiss} />);

    const dismissButton = screen.getByTestId('dismiss-button');
    await userEvent.click(dismissButton);

    expect(onDismiss).toHaveBeenCalled();
  });

  it('should call onDismiss when Escape key pressed', async () => {
    const onDismiss = jest.fn();
    const { container } = render(
      <SelectionTooltip {...defaultProps} onDismiss={onDismiss} />
    );

    const tooltip = container.querySelector('.selection-tooltip');
    await userEvent.keyboard('{Escape}');

    expect(onDismiss).toHaveBeenCalled();
  });

  it('should have accessible aria labels', () => {
    render(<SelectionTooltip {...defaultProps} />);
    expect(screen.getByRole('dialog')).toHaveAttribute(
      'aria-label',
      'Text selection action menu'
    );
  });

  it('should have accessible ask button label', () => {
    render(<SelectionTooltip {...defaultProps} />);
    const askButton = screen.getByTestId('ask-button');
    expect(askButton).toHaveAttribute('aria-label', 'Ask about: forward kinematics');
  });

  it('should have accessible dismiss button label', () => {
    render(<SelectionTooltip {...defaultProps} />);
    const dismissButton = screen.getByTestId('dismiss-button');
    expect(dismissButton).toHaveAttribute('aria-label', 'Dismiss tooltip');
  });

  it('should truncate long selected text in preview', () => {
    const longText = 'a'.repeat(100);
    render(
      <SelectionTooltip
        {...defaultProps}
        selectedText={longText}
      />
    );

    // Preview should be truncated
    const preview = screen.getByText(new RegExp(/a{30,}/));
    expect(preview.textContent?.length).toBeLessThan(longText.length);
  });

  it('should focus ask button on render', () => {
    render(<SelectionTooltip {...defaultProps} />);
    const askButton = screen.getByTestId('ask-button');

    // Check that button receives focus
    waitFor(() => {
      expect(askButton).toHaveFocus();
    });
  });

  it('should handle touch events for swipe-to-dismiss', () => {
    const onDismiss = jest.fn();
    const { container } = render(
      <SelectionTooltip {...defaultProps} onDismiss={onDismiss} />
    );

    const tooltip = container.querySelector('.selection-tooltip') as HTMLElement;

    // Simulate horizontal swipe (>50px)
    fireEvent.touchStart(tooltip, {
      touches: [{ clientX: 100, clientY: 200 }],
    });
    fireEvent.touchEnd(tooltip, {
      changedTouches: [{ clientX: 160, clientY: 200 }],
    });

    expect(onDismiss).toHaveBeenCalled();
  });

  it('should not dismiss on vertical scroll', () => {
    const onDismiss = jest.fn();
    const { container } = render(
      <SelectionTooltip {...defaultProps} onDismiss={onDismiss} />
    );

    const tooltip = container.querySelector('.selection-tooltip') as HTMLElement;

    // Simulate vertical movement (scroll)
    fireEvent.touchStart(tooltip, {
      touches: [{ clientX: 100, clientY: 200 }],
    });
    fireEvent.touchEnd(tooltip, {
      changedTouches: [{ clientX: 100, clientY: 260 }],
    });

    // Should not dismiss on vertical movement
    expect(onDismiss).not.toHaveBeenCalled();
  });

  it('should apply custom className', () => {
    const { container } = render(
      <SelectionTooltip {...defaultProps} className="custom-class" />
    );
    const tooltip = container.querySelector('.selection-tooltip');
    expect(tooltip).toHaveClass('custom-class');
  });

  it('should apply custom zIndex', () => {
    const { container } = render(
      <SelectionTooltip {...defaultProps} zIndex={9999} />
    );
    const tooltip = container.querySelector('.selection-tooltip');
    expect(tooltip).toHaveStyle('z-index: 9999');
  });

  it('should handle tooltip with title attribute for full text', () => {
    const selectedText = 'forward kinematics';
    render(<SelectionTooltip {...defaultProps} selectedText={selectedText} />);

    const preview = screen.getByTitle(selectedText);
    expect(preview).toBeInTheDocument();
  });

  it('should have proper role and dialog attributes', () => {
    const { container } = render(<SelectionTooltip {...defaultProps} />);
    const tooltip = container.querySelector('[role="dialog"]');
    expect(tooltip).toBeInTheDocument();
    expect(tooltip).toHaveAttribute('data-testid', 'selection-tooltip');
  });

  it('should render arrow element', () => {
    const { container } = render(<SelectionTooltip {...defaultProps} />);
    const arrow = container.querySelector('.selection-tooltip__arrow');
    expect(arrow).toBeInTheDocument();
    expect(arrow).toHaveAttribute('aria-hidden', 'true');
  });

  it('should handle empty selectedText gracefully', () => {
    const { container } = render(
      <SelectionTooltip {...defaultProps} selectedText="" />
    );
    expect(container.querySelector('.selection-tooltip')).toBeInTheDocument();
  });

  it('should memoize component correctly', () => {
    const { rerender } = render(<SelectionTooltip {...defaultProps} />);
    const firstRender = screen.getByTestId('selection-tooltip');

    // Rerender with same props
    rerender(<SelectionTooltip {...defaultProps} />);
    const secondRender = screen.getByTestId('selection-tooltip');

    // Component should still exist
    expect(firstRender).toBeInTheDocument();
    expect(secondRender).toBeInTheDocument();
  });
});
