"""Rate limiting middleware."""

import logging
import time
from typing import Callable

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

logger = logging.getLogger(__name__)


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Rate limiting middleware (10 requests per minute per IP)."""

    def __init__(self, app: Callable, requests_per_minute: int = 10) -> None:
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.request_times: dict[str, list[float]] = {}

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Check rate limit before processing request."""
        # Get client IP
        client_ip = request.client.host if request.client else "unknown"

        # Skip rate limiting for health checks
        if request.url.path == "/health":
            return await call_next(request)

        current_time = time.time()

        # Initialize IP if not seen before
        if client_ip not in self.request_times:
            self.request_times[client_ip] = []

        # Remove old requests (older than 1 minute)
        self.request_times[client_ip] = [
            t for t in self.request_times[client_ip] if current_time - t < 60
        ]

        # Check if limit exceeded
        if len(self.request_times[client_ip]) >= self.requests_per_minute:
            logger.warning(f"Rate limit exceeded for {client_ip}")
            return Response(
                content='{"error": "Rate limit exceeded", "details": "10 requests per minute limit"}',
                status_code=429,
                media_type="application/json",
                headers={"Retry-After": "60"},
            )

        # Record this request
        self.request_times[client_ip].append(current_time)

        # Process request
        response = await call_next(request)
        return response
