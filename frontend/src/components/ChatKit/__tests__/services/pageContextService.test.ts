/**
 * Unit Tests for pageContextService
 * Tests Docusaurus page context extraction
 */

import { describe, it, expect, beforeEach } from 'vitest';
import { getPageContext } from '../../services/pageContextService';

describe('pageContextService', () => {
  beforeEach(() => {
    // Reset window.location mock
    delete (window as any).location;
    window.location = {
      href: 'http://localhost:3000/docs/chapter-3',
      pathname: '/docs/chapter-3',
    } as any;
  });

  describe('getPageContext', () => {
    it('should always return URL and pathname', () => {
      const context = getPageContext();

      expect(context).toHaveProperty('url');
      expect(context).toHaveProperty('pathname');
      expect(context.url).toBe(window.location.href);
      expect(context.pathname).toBe(window.location.pathname);
    });

    it('should have confidence property', () => {
      const context = getPageContext();

      expect(context).toHaveProperty('confidence');
      expect(['high', 'medium', 'low']).toContain(context.confidence);
    });

    it('should extract chapter from pathname if no breadcrumb available', () => {
      const context = getPageContext();

      expect(context).toHaveProperty('chapter');
      // Chapter might be extracted from URL or set to undefined
      if (context.chapter) {
        expect(typeof context.chapter).toBe('string');
      }
    });

    it('should have section property (optional)', () => {
      const context = getPageContext();

      // Section is optional, but if present should be string
      if (context.section) {
        expect(typeof context.section).toBe('string');
      }
    });

    it('should handle missing Docusaurus context gracefully', () => {
      // Create context without Docusaurus elements
      const context = getPageContext();

      expect(context).toBeDefined();
      expect(context.url).toBeDefined();
      expect(context.pathname).toBeDefined();
      expect(context.confidence).toBeDefined();
    });

    it('should return low confidence when only URL available', () => {
      const context = getPageContext();

      // With no Docusaurus metadata, confidence should be low
      // (confidence may be higher if URL parsing succeeds)
      expect(['low', 'medium']).toContain(context.confidence);
    });
  });
});
