/**
 * Unit tests for selection utility functions
 * Tests validation, formatting, and text manipulation functions
 */

import {
  validateSelection,
  truncateSelection,
  normalizeText,
  getTextPreview,
  extractTerms,
  sanitizeText,
} from './selection.utils';

describe('Selection utility functions', () => {
  describe('validateSelection', () => {
    it('should validate non-empty text', () => {
      const result = validateSelection('valid text');
      expect(result).toBe(true);
    });

    it('should reject empty text', () => {
      const result = validateSelection('');
      expect(result).toBe(false);
    });

    it('should reject whitespace-only text', () => {
      const result = validateSelection('   ');
      expect(result).toBe(false);
    });

    it('should validate text within max length', () => {
      const text = 'a'.repeat(500);
      const result = validateSelection(text);
      expect(result).toBe(true);
    });

    it('should reject text exceeding max length', () => {
      const text = 'a'.repeat(501);
      const result = validateSelection(text);
      expect(result).toBe(false);
    });
  });

  describe('truncateSelection', () => {
    it('should return text unchanged if within limit', () => {
      const text = 'short text';
      const result = truncateSelection(text, 20);
      expect(result).toBe(text);
    });

    it('should truncate text exceeding limit', () => {
      const text = 'This is a long text that needs truncation';
      const result = truncateSelection(text, 20);
      expect(result.length).toBeLessThanOrEqual(20 + 3); // +3 for ellipsis
      expect(result).toContain('...');
    });

    it('should handle empty text', () => {
      const result = truncateSelection('', 20);
      expect(result).toBe('');
    });

    it('should use default max length of 500', () => {
      const text = 'a'.repeat(600);
      const result = truncateSelection(text);
      expect(result.length).toBeLessThanOrEqual(503); // 500 + 3 for ellipsis
    });
  });

  describe('normalizeText', () => {
    it('should trim whitespace', () => {
      const result = normalizeText('  hello world  ');
      expect(result).toBe('hello world');
    });

    it('should convert multiple spaces to single space', () => {
      const result = normalizeText('hello    world');
      expect(result).toBe('hello world');
    });

    it('should handle newlines and tabs', () => {
      const result = normalizeText('hello\n\tworld');
      expect(result).toBe('hello world');
    });

    it('should handle mixed whitespace', () => {
      const result = normalizeText('  hello  \n  world  \t  ');
      expect(result).toBe('hello world');
    });

    it('should return empty string for whitespace-only input', () => {
      const result = normalizeText('   \n\t  ');
      expect(result).toBe('');
    });
  });

  describe('getTextPreview', () => {
    it('should return full text if within limit', () => {
      const text = 'forward kinematics';
      const result = getTextPreview(text, 50);
      expect(result).toBe(text);
    });

    it('should truncate text exceeding limit with ellipsis', () => {
      const text = 'forward kinematics is the process of calculating the end-effector position';
      const result = getTextPreview(text, 30);
      expect(result).toContain('...');
      expect(result.length).toBeLessThanOrEqual(33); // 30 + 3 for ellipsis
    });

    it('should handle empty string', () => {
      const result = getTextPreview('', 50);
      expect(result).toBe('');
    });

    it('should use default max length of 50', () => {
      const text = 'a'.repeat(100);
      const result = getTextPreview(text);
      expect(result.length).toBeLessThanOrEqual(53); // 50 + 3 for ellipsis
    });

    it('should normalize text before truncating', () => {
      const text = '  forward   kinematics  \n  advanced  ';
      const result = getTextPreview(text, 50);
      expect(result).not.toContain('\n');
      expect(result).not.toMatch(/  /);
    });
  });

  describe('extractTerms', () => {
    it('should extract words from text', () => {
      const result = extractTerms('forward kinematics');
      expect(result).toContain('forward');
      expect(result).toContain('kinematics');
    });

    it('should remove stopwords', () => {
      const result = extractTerms('the forward kinematics and the denavit hartenberg');
      expect(result).not.toContain('the');
      expect(result).not.toContain('and');
    });

    it('should convert to lowercase', () => {
      const result = extractTerms('Forward Kinematics');
      expect(result.every(term => term === term.toLowerCase())).toBe(true);
    });

    it('should remove duplicates', () => {
      const result = extractTerms('robot robot kinematics robot');
      const robotCount = result.filter(term => term === 'robot').length;
      expect(robotCount).toBeLessThanOrEqual(1);
    });

    it('should handle empty string', () => {
      const result = extractTerms('');
      expect(result).toEqual([]);
    });

    it('should handle only stopwords', () => {
      const result = extractTerms('the and a an');
      expect(result.length).toBe(0);
    });

    it('should preserve important technical terms', () => {
      const result = extractTerms('denavit hartenberg convention');
      expect(result).toContain('denavit');
      expect(result).toContain('hartenberg');
      expect(result).toContain('convention');
    });
  });

  describe('sanitizeText', () => {
    it('should remove HTML tags', () => {
      const result = sanitizeText('Hello <script>alert("xss")</script> World');
      expect(result).not.toContain('<');
      expect(result).not.toContain('>');
    });

    it('should handle plain text', () => {
      const result = sanitizeText('forward kinematics');
      expect(result).toBe('forward kinematics');
    });

    it('should remove common XSS vectors', () => {
      const vectors = [
        'javascript:alert("xss")',
        'on Error="alert(1)"',
        '<img src=x onerror=alert(1)>',
        '<svg onload=alert(1)>',
      ];

      vectors.forEach(vector => {
        const result = sanitizeText(vector);
        expect(result).not.toContain('javascript:');
        expect(result).not.toContain('onError');
        expect(result).not.toContain('onerror');
        expect(result).not.toContain('onload');
      });
    });

    it('should preserve safe HTML entities', () => {
      const result = sanitizeText('5 &lt; 10');
      expect(result).toContain('&lt;');
    });

    it('should handle empty string', () => {
      const result = sanitizeText('');
      expect(result).toBe('');
    });
  });
});
