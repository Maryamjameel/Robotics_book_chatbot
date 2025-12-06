/**
 * API Configuration Logging Utility
 * Logs API configuration at application startup for debugging
 */

export interface LogEntry {
  timestamp: Date;
  level: 'debug' | 'info' | 'warn' | 'error';
  message: string;
  data?: Record<string, unknown>;
}

/**
 * API Configuration Logger
 */
export class APIConfigLogger {
  private logs: LogEntry[] = [];
  private maxLogs = 100;

  /**
   * Log API configuration at startup
   */
  logStartup(data: {
    environment: string;
    baseURL: string;
    isDevelopment: boolean;
    isProduction: boolean;
    isStaging: boolean;
    customURLSet: boolean;
  }): void {
    const message = '[APIConfig] Configuration loaded at startup';
    
    this.info(message, data);

    // Log details based on environment
    if (data.isDevelopment) {
      this.debug('[APIConfig] Development mode - using localhost endpoint', {
        baseURL: data.baseURL,
      });
    } else if (data.isProduction) {
      this.info('[APIConfig] Production mode - using production endpoint', {
        baseURL: data.baseURL,
        customURLSet: data.customURLSet,
      });
    }

    if (data.customURLSet) {
      this.info('[APIConfig] Custom API URL override detected', {
        baseURL: data.baseURL,
      });
    }
  }

  /**
   * Log API request
   */
  logRequest(method: string, url: string, data?: unknown): void {
    this.debug('[APIRequest] Outgoing request', {
      method,
      url,
      timestamp: new Date().toISOString(),
    });
  }

  /**
   * Log API response
   */
  logResponse(
    method: string,
    url: string,
    status: number,
    duration: number
  ): void {
    const level = status >= 400 ? 'warn' : 'debug';
    this.log(level, '[APIResponse] Response received', {
      method,
      url,
      status,
      durationMs: duration,
      timestamp: new Date().toISOString(),
    });
  }

  /**
   * Log API error
   */
  logError(method: string, url: string, error: unknown): void {
    const errorMessage = error instanceof Error ? error.message : String(error);
    this.error('[APIError] Request failed', {
      method,
      url,
      error: errorMessage,
      timestamp: new Date().toISOString(),
    });
  }

  /**
   * Log configuration validation result
   */
  logValidation(isValid: boolean, errors: string[] = [], warnings: string[] = []): void {
    if (isValid) {
      this.debug('[APIValidation] Configuration validation passed');
    } else {
      this.error('[APIValidation] Configuration validation failed', {
        errors,
        warnings,
      });
    }
  }

  /**
   * Internal logging methods
   */
  private debug(message: string, data?: Record<string, unknown>): void {
    this.log('debug', message, data);
  }

  private info(message: string, data?: Record<string, unknown>): void {
    this.log('info', message, data);
  }

  private warn(message: string, data?: Record<string, unknown>): void {
    this.log('warn', message, data);
  }

  private error(message: string, data?: Record<string, unknown>): void {
    this.log('error', message, data);
  }

  private log(
    level: 'debug' | 'info' | 'warn' | 'error',
    message: string,
    data?: Record<string, unknown>
  ): void {
    const entry: LogEntry = {
      timestamp: new Date(),
      level,
      message,
      data,
    };

    // Add to in-memory log
    this.logs.push(entry);
    if (this.logs.length > this.maxLogs) {
      this.logs.shift();
    }

    // Console output
    const consoleFn = console[level];
    if (consoleFn) {
      if (data) {
        consoleFn(message, data);
      } else {
        consoleFn(message);
      }
    }
  }

  /**
   * Get all logs
   */
  getLogs(): LogEntry[] {
    return [...this.logs];
  }

  /**
   * Clear logs
   */
  clearLogs(): void {
    this.logs = [];
  }

  /**
   * Get logs by level
   */
  getLogsByLevel(level: LogEntry['level']): LogEntry[] {
    return this.logs.filter(log => log.level === level);
  }

  /**
   * Export logs as JSON
   */
  exportLogs(): string {
    return JSON.stringify(this.logs, null, 2);
  }
}

/**
 * Singleton logger instance
 */
export const apiLogger = new APIConfigLogger();
