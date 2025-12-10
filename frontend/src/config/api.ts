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
  const nodeEnv = (typeof process !== 'undefined' ? process.env?.NODE_ENV : 'development') as Environment;

  console.log('[APIConfig] Environment:', nodeEnv);

  // Always use Hugging Face Spaces backend (deployed backend)
  const baseURL = 'https://maryamjamil-robotics-chatbot-api.hf.space/api';
  console.log('[APIConfig] Using Hugging Face Spaces backend:', baseURL);

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
