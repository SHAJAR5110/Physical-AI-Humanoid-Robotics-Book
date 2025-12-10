"""Structured logging middleware."""

import json
import logging
import time
from typing import Callable

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

logger = logging.getLogger(__name__)


class LoggingMiddleware(BaseHTTPMiddleware):
    """Structured logging middleware for requests and responses."""

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Log request and response."""
        client_ip = request.client.host if request.client else "unknown"
        start_time = time.time()

        # Log request
        logger.info(
            json.dumps(
                {
                    "event": "request",
                    "method": request.method,
                    "path": request.url.path,
                    "client_ip": client_ip,
                    "timestamp": start_time,
                }
            )
        )

        # Process request
        try:
            response = await call_next(request)
        except Exception as e:
            logger.error(
                json.dumps(
                    {
                        "event": "error",
                        "path": request.url.path,
                        "error": str(e),
                        "timestamp": time.time(),
                    }
                )
            )
            raise

        # Log response
        duration = time.time() - start_time
        logger.info(
            json.dumps(
                {
                    "event": "response",
                    "method": request.method,
                    "path": request.url.path,
                    "status_code": response.status_code,
                    "duration_ms": int(duration * 1000),
                    "timestamp": time.time(),
                }
            )
        )

        return response
