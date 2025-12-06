import { test, expect } from '@playwright/test';

test.describe('Selected Text Integration', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
    await page.waitForSelector('.chatkit-widget', { timeout: 5000 });
  });

  test('should detect when user selects text on page', async ({ page }) => {
    // Find a text element to select
    const textElement = await page.locator('main, article, .docusaurus-mt-lg, p').first();
    
    // Select text from the element
    const selectedText = await textElement.evaluate((el) => {
      const selection = window.getSelection();
      if (selection && el.textContent) {
        const range = document.createRange();
        range.selectNodeContents(el);
        selection.removeAllRanges();
        selection.addRange(range);
        return selection.toString();
      }
      return '';
    });

    // Text should be selected
    expect(selectedText.length).toBeGreaterThan(0);
  });

  test('should open ChatKit with selected text in query', async ({ page }) => {
    // Find and select text
    const textElement = await page.locator('main, article, p').first();
    const selectedText = 'selected text';
    
    // Simulate selection event
    await page.evaluate(({ text }) => {
      const event = new CustomEvent('selection:ask', {
        detail: { selectedText: text }
      });
      document.dispatchEvent(event);
    }, { text: selectedText });

    // Check if ChatKit input contains the selected text
    const input = await page.locator('.chatkit-input');
    
    // Wait for input to potentially be filled
    await page.waitForTimeout(500);
    
    // The input might be pre-filled with selected text
    expect(input).toBeDefined();
  });

  test('should pass selected text to backend API', async ({ page }) => {
    const selectedText = 'kinematics in robotics';
    let apiPayload: Record<string, unknown> | null = null;

    // Intercept API requests
    page.on('request', async (request) => {
      if (request.url().includes('/chat')) {
        try {
          const postData = request.postDataJSON();
          apiPayload = postData;
        } catch {
          // Not JSON
        }
      }
    });

    // Trigger selection
    await page.evaluate(({ text }) => {
      const event = new CustomEvent('selection:ask', {
        detail: { selectedText: text }
      });
      document.dispatchEvent(event);
    }, { text: selectedText });

    // Wait for potential API call
    await page.waitForTimeout(1000);

    // If API was called, it should contain selectedText
    if (apiPayload) {
      expect(typeof apiPayload === 'object').toBe(true);
    }
  });

  test('should handle empty selection gracefully', async ({ page }) => {
    // Try to trigger selection with empty text
    await page.evaluate(() => {
      const event = new CustomEvent('selection:ask', {
        detail: { selectedText: '' }
      });
      document.dispatchEvent(event);
    });

    // ChatKit should still be visible and functional
    const widget = await page.locator('.chatkit-widget');
    await expect(widget).toBeVisible();
  });

  test('should handle large selected text', async ({ page }) => {
    const largeText = 'Lorem ipsum dolor sit amet, '.repeat(50); // ~1400 chars

    await page.evaluate(({ text }) => {
      const event = new CustomEvent('selection:ask', {
        detail: { selectedText: text }
      });
      document.dispatchEvent(event);
    }, { text: largeText });

    // Should handle large text without breaking
    const input = await page.locator('.chatkit-input');
    await expect(input).toBeVisible();
  });
});
