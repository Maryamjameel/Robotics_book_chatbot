"""Rate limiting middleware for API endpoints."""

import asyncio
import time
from collections import deque
from typing import Dict, Optional

from fastapi import HTTPException, Request
from starlette.middleware.base import BaseHTTPMiddleware


class RateLimiter:
    """Token bucket rate limiter for controlling request rate."""

    def __init__(self, max_requests: int = 5, time_window: float = 1.0):
        """Initialize rate limiter.

        Args:
            max_requests: Maximum number of requests allowed in time window
            time_window: Time window in seconds
        """
        self.max_requests = max_requests
        self.time_window = time_window
        self.requests: deque = deque()
        self.lock = asyncio.Lock()

    async def acquire(self) -> bool:
        """Try to acquire a token for making a request.

        Returns:
            True if request is allowed, False if rate limited
        """
        async with self.lock:
            current_time = time.time()

            # Remove requests outside the time window
            while self.requests and current_time - self.requests[0] > self.time_window:
                self.requests.popleft()

            # Check if we can make a new request
            if len(self.requests) < self.max_requests:
                self.requests.append(current_time)
                return True

            return False

    async def wait_if_needed(self, max_wait: float = 5.0) -> None:
        """Wait until a token is available or max_wait is reached.

        Args:
            max_wait: Maximum time to wait in seconds

        Raises:
            HTTPException: If max_wait is exceeded
        """
        start_time = time.time()

        while True:
            if await self.acquire():
                return

            # Check if we've exceeded max wait time
            elapsed = time.time() - start_time
            if elapsed >= max_wait:
                raise HTTPException(
                    status_code=429,
                    detail="Rate limit exceeded. Please try again later.",
                    headers={"Retry-After": "1"},
                )

            # Wait a bit before retrying
            await asyncio.sleep(0.1)


class RequestQueue:
    """Queue for managing concurrent requests with rate limiting."""

    def __init__(self, max_concurrent: int = 3):
        """Initialize request queue.

        Args:
            max_concurrent: Maximum number of concurrent requests
        """
        self.semaphore = asyncio.Semaphore(max_concurrent)
        self.active_requests = 0
        self.lock = asyncio.Lock()

    async def process_request(self, request_func, *args, **kwargs):
        """Process a request with concurrency control.

        Args:
            request_func: Async function to execute
            *args: Positional arguments for request_func
            **kwargs: Keyword arguments for request_func

        Returns:
            Result from request_func
        """
        async with self.semaphore:
            async with self.lock:
                self.active_requests += 1

            try:
                result = await request_func(*args, **kwargs)
                return result
            finally:
                async with self.lock:
                    self.active_requests -= 1


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Middleware for rate limiting API requests."""

    def __init__(self, app, requests_per_second: int = 5):
        """Initialize rate limit middleware.

        Args:
            app: FastAPI application
            requests_per_second: Maximum requests per second
        """
        super().__init__(app)
        self.rate_limiter = RateLimiter(
            max_requests=requests_per_second, time_window=1.0
        )

    async def dispatch(self, request: Request, call_next):
        """Process request with rate limiting.

        Args:
            request: Incoming HTTP request
            call_next: Next middleware in chain

        Returns:
            HTTP response
        """
        # Only apply rate limiting to /api/v1/chat/ask endpoint
        if request.url.path == "/api/v1/chat/ask":
            try:
                # Wait for rate limiter to allow request (max 5 seconds)
                await self.rate_limiter.wait_if_needed(max_wait=5.0)
            except HTTPException as e:
                # Rate limit exceeded
                from fastapi.responses import JSONResponse

                return JSONResponse(
                    status_code=e.status_code,
                    content={"detail": e.detail},
                    headers=e.headers,
                )

        # Process the request
        response = await call_next(request)
        return response


# Global rate limiter and queue instances
embedding_rate_limiter = RateLimiter(max_requests=5, time_window=1.0)
request_queue = RequestQueue(max_concurrent=3)
