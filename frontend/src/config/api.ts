/**
 * API Configuration Module
 */
type Environment = 'development' | 'production' | 'staging';

export interface APIConfiguration {
  baseURL: string;
  environment: Environment;
  isDevelopment: boolean;
  isProduction: boolean;
  isStaging: boolean;
  validateURL(): void;
}

export function getAPIConfig(): APIConfiguration {
  const customURL = typeof process !== 'undefined' ? process.env?.REACT_APP_API_URL : undefined;
  const nodeEnv = (typeof process !== 'undefined' ? process.env?.NODE_ENV : 'development') as Environment;

  let baseURL: string;
  if (customURL) {
    baseURL = customURL;
  } else {
    switch (nodeEnv) {
      case 'production':
        baseURL = 'https://api.yourdomain.com/api';
        console.warn('[APIConfig] PRODUCTION mode detected but REACT_APP_API_URL not set.');
        break;
      case 'staging':
        baseURL = 'https://staging-api.yourdomain.com/api';
        break;
      case 'development':
      default:
        baseURL = 'http://localhost:8000/api';
        break;
    }
  }

  const config: APIConfiguration = {
    baseURL,
    environment: nodeEnv,
    isDevelopment: nodeEnv === 'development',
    isProduction: nodeEnv === 'production',
    isStaging: nodeEnv === 'staging',
    validateURL: function() {
      validateAPIURL(this.baseURL, this.environment);
    },
  };

  config.validateURL();
  return config;
}

export function validateAPIURL(url: string, environment: Environment): void {
  if (!url || url.trim() === '') {
    throw new Error('[APIConfig] API URL cannot be empty');
  }

  try {
    new URL(url);
  } catch (error) {
    throw new Error(
      `[APIConfig] Invalid API URL format: "${url}". Expected valid URL.`
    );
  }

  if (environment === 'production' && !url.startsWith('https://')) {
    throw new Error(
      `[APIConfig] Production API URL must use HTTPS. Got: "${url}".`
    );
  }

  if (environment === 'development') {
    console.debug(`[APIConfig] API URL validated: ${url}`);
  }
}

export const apiConfig: APIConfiguration = getAPIConfig();

export function logAPIConfiguration(): void {
  console.log('[APIConfig] Configuration loaded:', {
    environment: apiConfig.environment,
    baseURL: apiConfig.baseURL,
    isDevelopment: apiConfig.isDevelopment,
    isProduction: apiConfig.isProduction,
    isStaging: apiConfig.isStaging,
    customURLSet: typeof process !== 'undefined' ? !!process.env?.REACT_APP_API_URL : false,
  });
}
