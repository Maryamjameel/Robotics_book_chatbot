"""Global rate limiter for Gemini API calls."""

import asyncio
import time
from collections import deque


class GlobalRateLimiter:
    """Global rate limiter using token bucket algorithm."""

    def __init__(self, max_requests_per_second: int = 2):
        """Initialize rate limiter.

        Args:
            max_requests_per_second: Maximum requests allowed per second
        """
        self.max_requests = max_requests_per_second
        self.window = 1.0  # 1 second window
        self.requests = deque()
        self.lock = asyncio.Lock()

    async def acquire(self):
        """Wait until we can make a request without exceeding rate limit."""
        async with self.lock:
            now = time.time()

            # Remove old requests outside the time window
            while self.requests and now - self.requests[0] > self.window:
                self.requests.popleft()

            # If we're at the limit, wait until the oldest request expires
            if len(self.requests) >= self.max_requests:
                sleep_time = self.requests[0] + self.window - now + 0.1  # Add 100ms buffer
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)

                # Clean up again after sleeping
                now = time.time()
                while self.requests and now - self.requests[0] > self.window:
                    self.requests.popleft()

            # Add current request
            self.requests.append(time.time())


# Global singleton instance - shared across all requests
# Limit to 2 requests per second to be safe (well below Gemini's 5 req/sec limit)
gemini_rate_limiter = GlobalRateLimiter(max_requests_per_second=2)
