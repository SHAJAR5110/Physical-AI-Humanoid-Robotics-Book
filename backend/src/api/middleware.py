"""
Middleware for Physical AI Book backend.

Handles authentication, error handling, logging, and request/response processing.
"""

import logging
import time
from typing import Callable
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)


class AuthMiddleware:
    """Middleware for protecting endpoints and injecting user context."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, request: Request, call_next: Callable):
        """Process request and validate auth token if required."""
        # Skip auth for public endpoints
        public_paths = ["/", "/docs", "/openapi.json", "/health"]
        if request.url.path in public_paths or request.url.path.startswith("/api/auth/"):
            return await call_next(request)

        # Extract token from Authorization header
        auth_header = request.headers.get("Authorization")
        if not auth_header:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Missing Authorization header",
            )

        try:
            scheme, token = auth_header.split()
            if scheme.lower() != "bearer":
                raise ValueError("Invalid auth scheme")
        except ValueError:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid Authorization header",
            )

        # Validate token
        from services.auth_service import AuthService
        user_id = AuthService.validate_token(token)

        if not user_id:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid or expired token",
            )

        # Inject user context into request
        request.state.user_id = user_id
        response = await call_next(request)
        return response


class LoggingMiddleware:
    """Middleware for logging request/response metrics."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, request: Request, call_next: Callable):
        """Log request and response."""
        start_time = time.time()

        # Log request
        logger.info(f"{request.method} {request.url.path}")

        try:
            response = await call_next(request)
            process_time = time.time() - start_time

            # Log response
            logger.info(
                f"{request.method} {request.url.path} - {response.status_code} "
                f"({process_time:.3f}s)"
            )

            return response
        except Exception as e:
            process_time = time.time() - start_time
            logger.error(
                f"{request.method} {request.url.path} - ERROR ({process_time:.3f}s): {e}"
            )
            raise


class ErrorHandler:
    """Error handler for consistent error responses."""

    @staticmethod
    async def http_exception_handler(request: Request, exc: HTTPException):
        """Handle HTTP exceptions."""
        return JSONResponse(
            status_code=exc.status_code,
            content={
                "error": exc.detail,
                "status": exc.status_code,
                "path": request.url.path,
            },
        )

    @staticmethod
    async def general_exception_handler(request: Request, exc: Exception):
        """Handle general exceptions."""
        logger.error(f"Unhandled exception: {exc}")
        return JSONResponse(
            status_code=500,
            content={
                "error": "Internal server error",
                "status": 500,
                "path": request.url.path,
            },
        )
