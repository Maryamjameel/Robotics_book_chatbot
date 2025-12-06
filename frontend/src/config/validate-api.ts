/**
 * API URL Validation Utility
 */

export interface ValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
}

/**
 * Validate API URL format and requirements
 */
export function validateURLFormat(url: string, environment: string = 'development'): ValidationResult {
  const errors: string[] = [];
  const warnings: string[] = [];

  if (!url || url.trim() === '') {
    errors.push('API URL cannot be empty');
    return { isValid: false, errors, warnings };
  }

  try {
    new URL(url);
  } catch (error) {
    errors.push(
      `Invalid URL format: "${url}". Expected format: http(s)://host:port/path`
    );
    return { isValid: false, errors, warnings };
  }

  if (environment === 'production' && !url.startsWith('https://')) {
    errors.push(
      `Production API URL must use HTTPS. Got: "${url}"`
    );
  }

  if (environment === 'production' && url.includes('localhost')) {
    warnings.push(
      'Localhost URL detected in production. Ensure this is intentional.'
    );
  }

  if (environment === 'production') {
    const urlObj = new URL(url);
    const port = urlObj.port;
    if (port && port !== '443' && port !== '80') {
      warnings.push(
        `Non-standard port ${port} detected. Ensure firewall rules allow this port.`
      );
    }
  }

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
  };
}

/**
 * Check if URL is reachable
 */
export async function isURLReachable(url: string, timeout: number = 5000): Promise<boolean> {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), timeout);

    const response = await fetch(url, {
      method: 'HEAD',
      signal: controller.signal,
    }).catch(async () => {
      return fetch(url, {
        method: 'GET',
        signal: controller.signal,
      });
    });

    clearTimeout(timeoutId);
    return response.ok || response.status !== 0;
  } catch (error) {
    return false;
  }
}

/**
 * Validate domain matches expected patterns
 */
export function validateDomain(url: string, expectedDomains: string[]): boolean {
  try {
    const urlObj = new URL(url);
    return expectedDomains.some(domain => 
      urlObj.hostname === domain || urlObj.hostname.endsWith(`.${domain}`)
    );
  } catch {
    return false;
  }
}

/**
 * Complete validation with all checks
 */
export async function validateAPIURL(
  url: string,
  environment: string = 'development',
  options: {
    checkReachability?: boolean;
    timeout?: number;
    allowedDomains?: string[];
  } = {}
): Promise<ValidationResult> {
  const formatResult = validateURLFormat(url, environment);

  if (!formatResult.isValid) {
    return formatResult;
  }

  const errors = [...formatResult.errors];
  const warnings = [...formatResult.warnings];

  if (options.checkReachability) {
    const isReachable = await isURLReachable(url, options.timeout || 5000);
    if (!isReachable) {
      warnings.push(
        `API URL may not be reachable: "${url}". Please verify endpoint is available.`
      );
    }
  }

  if (options.allowedDomains && options.allowedDomains.length > 0) {
    const domainValid = validateDomain(url, options.allowedDomains);
    if (!domainValid) {
      const hostname = new URL(url).hostname;
      errors.push(
        `API domain must be one of: ${options.allowedDomains.join(', ')}. Got: "${hostname}"`
      );
    }
  }

  return {
    isValid: errors.length === 0,
    errors,
    warnings,
  };
}

/**
 * Log validation results
 */
export function logValidationResult(url: string, result: ValidationResult): void {
  if (result.isValid) {
    console.debug(`[APIValidation] Valid URL: ${url}`);
  } else {
    console.error(`[APIValidation] Invalid URL: ${url}`);
    result.errors.forEach(err => console.error(`  - ${err}`));
  }

  if (result.warnings.length > 0) {
    console.warn(`[APIValidation] Warnings for: ${url}`);
    result.warnings.forEach(warn => console.warn(`  - ${warn}`));
  }
}
