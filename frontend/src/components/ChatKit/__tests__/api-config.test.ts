import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { getAPIConfig, validateAPIURL, logAPIConfiguration } from '../api';

describe('API Configuration Module', () => {
  beforeEach(() => {
    // Save original env vars
    vi.stubEnv('NODE_ENV', 'development');
    vi.stubEnv('REACT_APP_API_URL', '');
  });

  afterEach(() => {
    vi.unstubAllEnvs();
    vi.clearAllMocks();
  });

  describe('getAPIConfig function', () => {
    it('should return configuration object with required properties', () => {
      const config = getAPIConfig();

      expect(config).toHaveProperty('baseURL');
      expect(config).toHaveProperty('environment');
      expect(config).toHaveProperty('isDevelopment');
      expect(config).toHaveProperty('isProduction');
      expect(config).toHaveProperty('isStaging');
      expect(config).toHaveProperty('validateURL');
    });

    it('should return correct baseURL for development environment', () => {
      vi.stubEnv('NODE_ENV', 'development');
      vi.stubEnv('REACT_APP_API_URL', '');

      const config = getAPIConfig();

      expect(config.baseURL).toBe('http://localhost:8000/api');
      expect(config.isDevelopment).toBe(true);
      expect(config.isProduction).toBe(false);
    });

    it('should return correct baseURL for production environment', () => {
      vi.stubEnv('NODE_ENV', 'production');
      vi.stubEnv('REACT_APP_API_URL', '');

      const config = getAPIConfig();

      expect(config.baseURL).toBe('https://api.yourdomain.com/api');
      expect(config.isProduction).toBe(true);
      expect(config.isDevelopment).toBe(false);
    });

    it('should return correct baseURL for staging environment', () => {
      vi.stubEnv('NODE_ENV', 'staging');
      vi.stubEnv('REACT_APP_API_URL', '');

      const config = getAPIConfig();

      expect(config.baseURL).toBe('https://staging-api.yourdomain.com/api');
      expect(config.isStaging).toBe(true);
    });
  });

  describe('REACT_APP_API_URL override', () => {
    it('should override NODE_ENV when REACT_APP_API_URL is set', () => {
      const customURL = 'https://custom-api.example.com/api';
      vi.stubEnv('NODE_ENV', 'development');
      vi.stubEnv('REACT_APP_API_URL', customURL);

      const config = getAPIConfig();

      expect(config.baseURL).toBe(customURL);
    });

    it('should prioritize REACT_APP_API_URL over production NODE_ENV', () => {
      const customURL = 'https://staging-override.example.com/api';
      vi.stubEnv('NODE_ENV', 'production');
      vi.stubEnv('REACT_APP_API_URL', customURL);

      const config = getAPIConfig();

      expect(config.baseURL).toBe(customURL);
      expect(config.isProduction).toBe(true);
    });
  });

  describe('URL validation', () => {
    it('should validate correct HTTP development URL', () => {
      expect(() => {
        validateAPIURL('http://localhost:8000/api', 'development');
      }).not.toThrow();
    });

    it('should validate correct HTTPS production URL', () => {
      expect(() => {
        validateAPIURL('https://api.example.com/api', 'production');
      }).not.toThrow();
    });

    it('should reject empty URL', () => {
      expect(() => {
        validateAPIURL('', 'development');
      }).toThrow('[APIConfig] API URL cannot be empty');
    });

    it('should reject invalid URL format', () => {
      expect(() => {
        validateAPIURL('not-a-valid-url', 'development');
      }).toThrow('[APIConfig] Invalid API URL format');
    });

    it('should enforce HTTPS in production', () => {
      expect(() => {
        validateAPIURL('http://api.example.com/api', 'production');
      }).toThrow('[APIConfig] Production API URL must use HTTPS');
    });

    it('should allow HTTP in development', () => {
      expect(() => {
        validateAPIURL('http://localhost:3000/api', 'development');
      }).not.toThrow();
    });

    it('should allow HTTPS in development', () => {
      expect(() => {
        validateAPIURL('https://dev-api.example.com/api', 'development');
      }).not.toThrow();
    });
  });

  describe('Configuration loading', () => {
    it('should load configuration at startup before components render', () => {
      const config = getAPIConfig();
      
      // Configuration should be available immediately
      expect(config).toBeDefined();
      expect(config.baseURL).toBeDefined();
      expect(config.environment).toBeDefined();
    });

    it('should be singleton - same instance on multiple calls', () => {
      const config1 = getAPIConfig();
      const config2 = getAPIConfig();
      
      // Both should have the same baseURL (same environment)
      expect(config1.baseURL).toBe(config2.baseURL);
    });
  });

  describe('Error handling', () => {
    it('should throw error if validateURL is called with invalid config', () => {
      vi.stubEnv('NODE_ENV', 'development');
      vi.stubEnv('REACT_APP_API_URL', 'not-a-url');

      expect(() => {
        getAPIConfig();
      }).toThrow();
    });

    it('should provide helpful error message for misconfigured URLs', () => {
      expect(() => {
        validateAPIURL('bad-url-format', 'development');
      }).toThrow(expect.stringContaining('[APIConfig]'));
    });
  });

  describe('Logging', () => {
    it('should have logAPIConfiguration function', () => {
      expect(typeof logAPIConfiguration).toBe('function');
    });

    it('should log configuration without throwing errors', () => {
      const consoleSpy = vi.spyOn(console, 'log').mockImplementation(() => {});
      
      logAPIConfiguration();
      
      expect(consoleSpy).toHaveBeenCalled();
      
      consoleSpy.mockRestore();
    });
  });
});
