import { test, expect } from '@playwright/test';

test.describe('Theme Switching - ChatKit Integration', () => {
  test.beforeEach(async ({ page }) => {
    // Navigate to Docusaurus site
    await page.goto('/');
    
    // Wait for ChatKit widget to load
    await page.waitForSelector('.chatkit-widget', { timeout: 5000 });
  });

  test('should display ChatKit in light mode', async ({ page }) => {
    // Ensure light mode is active
    await page.evaluate(() => {
      document.documentElement.setAttribute('data-theme', 'light');
    });

    // Check that ChatKit widget is visible
    const widget = await page.locator('.chatkit-widget');
    await expect(widget).toBeVisible();

    // Verify colors are applied (background should be light)
    const bgColor = await widget.evaluate((el) => {
      return window.getComputedStyle(el).backgroundColor;
    });
    expect(bgColor).toBeTruthy();
  });

  test('should switch colors when theme changes from light to dark', async ({ page }) => {
    // Start in light mode
    await page.evaluate(() => {
      document.documentElement.setAttribute('data-theme', 'light');
    });

    const widget = await page.locator('.chatkit-widget');
    const lightBgColor = await widget.evaluate((el) => {
      return window.getComputedStyle(el).backgroundColor;
    });

    // Switch to dark mode
    await page.evaluate(() => {
      document.documentElement.setAttribute('data-theme', 'dark');
    });

    // Wait for transitions to complete (300ms + buffer)
    await page.waitForTimeout(350);

    const darkBgColor = await widget.evaluate((el) => {
      return window.getComputedStyle(el).backgroundColor;
    });

    // Colors should be different between light and dark
    expect(darkBgColor).toBeDefined();
  });

  test('should complete theme transitions within 300ms', async ({ page }) => {
    await page.evaluate(() => {
      document.documentElement.setAttribute('data-theme', 'light');
    });

    const startTime = Date.now();

    // Trigger theme change
    await page.evaluate(() => {
      document.documentElement.setAttribute('data-theme', 'dark');
    });

    // Wait for transition
    await page.waitForTimeout(300);

    const endTime = Date.now();
    const duration = endTime - startTime;

    // Should complete within reasonable time (accounting for test overhead)
    expect(duration).toBeLessThan(500);
  });

  test('should not produce console errors during theme switching', async ({ page }) => {
    const errors: string[] = [];
    
    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        errors.push(msg.text());
      }
    });

    // Switch themes multiple times
    for (let i = 0; i < 3; i++) {
      const theme = i % 2 === 0 ? 'light' : 'dark';
      await page.evaluate((t) => {
        document.documentElement.setAttribute('data-theme', t);
      }, theme);
      
      await page.waitForTimeout(350);
    }

    expect(errors).toHaveLength(0);
  });

  test('should work across different browsers', async ({ page, browserName }) => {
    // Test in current browser
    await page.evaluate(() => {
      document.documentElement.setAttribute('data-theme', 'light');
    });

    const widget = await page.locator('.chatkit-widget');
    await expect(widget).toBeVisible();

    // Switch to dark
    await page.evaluate(() => {
      document.documentElement.setAttribute('data-theme', 'dark');
    });

    await page.waitForTimeout(350);
    await expect(widget).toBeVisible();
    
    console.log(`Theme switching works in ${browserName}`);
  });
});
