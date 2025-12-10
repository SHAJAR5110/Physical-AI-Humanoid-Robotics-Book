"""Error handling utility for converting exceptions to user-friendly responses."""

import logging
from typing import Any, Optional

from pydantic import ValidationError

from src.models import ErrorResponse
from src.services.generation_service import GenerationError
from src.services.retrieval_service import RetrievalError

logger = logging.getLogger(__name__)


class ErrorHandler:
    """Handles exception conversion to user-friendly error responses."""

    # User-friendly error messages
    ERROR_MESSAGES = {
        "validation_error": {
            "title": "Validation failed",
            "message": "Your input is invalid. Please check and try again.",
        },
        "retrieval_error": {
            "title": "No relevant content found",
            "message": "I couldn't find information on this topic. Try rephrasing your question or browse the chapters.",
        },
        "generation_error": {
            "title": "Generation failed",
            "message": "I'm having trouble generating a response. Please try again.",
        },
        "service_unavailable": {
            "title": "Service unavailable",
            "message": "Chat service is temporarily unavailable. Please try again in a moment.",
        },
        "rate_limit": {
            "title": "Rate limit exceeded",
            "message": "You have exceeded 10 requests per minute. Please wait before asking another question.",
        },
        "internal_error": {
            "title": "Internal server error",
            "message": "Something went wrong. Please try again later.",
        },
    }

    @staticmethod
    def handle_validation_error(error: ValidationError) -> tuple[ErrorResponse, int]:
        """
        Handle Pydantic validation errors.

        Args:
            error: ValidationError from Pydantic

        Returns:
            Tuple of (ErrorResponse, HTTP status code)
        """
        # Extract specific validation error
        if error.errors():
            first_error = error.errors()[0]
            field = ".".join(str(x) for x in first_error["loc"])
            msg = first_error["msg"]
            details = f"{field}: {msg}"
        else:
            details = "Validation failed"

        logger.warning(f"Validation error: {details}")
        return (
            ErrorResponse(
                error=ErrorHandler.ERROR_MESSAGES["validation_error"]["title"],
                details=details,
            ),
            400,
        )

    @staticmethod
    def handle_retrieval_error(error: RetrievalError) -> tuple[ErrorResponse, int]:
        """
        Handle retrieval errors.

        Args:
            error: RetrievalError

        Returns:
            Tuple of (ErrorResponse, HTTP status code)
        """
        logger.warning(f"Retrieval error: {str(error)}")
        return (
            ErrorResponse(
                error=ErrorHandler.ERROR_MESSAGES["retrieval_error"]["title"],
                details=str(error),
            ),
            400,
        )

    @staticmethod
    def handle_generation_error(error: GenerationError) -> tuple[ErrorResponse, int]:
        """
        Handle generation errors.

        Args:
            error: GenerationError

        Returns:
            Tuple of (ErrorResponse, HTTP status code)
        """
        logger.error(f"Generation error: {str(error)}")
        return (
            ErrorResponse(
                error=ErrorHandler.ERROR_MESSAGES["generation_error"]["title"],
                details=str(error),
            ),
            503,
        )

    @staticmethod
    def handle_service_unavailable() -> tuple[ErrorResponse, int]:
        """
        Handle service unavailable errors.

        Returns:
            Tuple of (ErrorResponse, HTTP status code)
        """
        logger.error("Service unavailable")
        return (
            ErrorResponse(
                error=ErrorHandler.ERROR_MESSAGES["service_unavailable"]["title"],
                details=ErrorHandler.ERROR_MESSAGES["service_unavailable"]["message"],
            ),
            503,
        )

    @staticmethod
    def handle_rate_limit() -> tuple[ErrorResponse, int]:
        """
        Handle rate limit errors.

        Returns:
            Tuple of (ErrorResponse, HTTP status code)
        """
        logger.warning("Rate limit exceeded")
        return (
            ErrorResponse(
                error=ErrorHandler.ERROR_MESSAGES["rate_limit"]["title"],
                details=ErrorHandler.ERROR_MESSAGES["rate_limit"]["message"],
            ),
            429,
        )

    @staticmethod
    def handle_generic_error(error: Exception) -> tuple[ErrorResponse, int]:
        """
        Handle generic/unexpected errors.

        Args:
            error: Exception

        Returns:
            Tuple of (ErrorResponse, HTTP status code)
        """
        logger.error(f"Unexpected error: {str(error)}", exc_info=True)
        return (
            ErrorResponse(
                error=ErrorHandler.ERROR_MESSAGES["internal_error"]["title"],
                details=ErrorHandler.ERROR_MESSAGES["internal_error"]["message"],
            ),
            500,
        )

    @staticmethod
    def log_error_with_context(
        error: Exception,
        context: Optional[dict] = None,
    ) -> None:
        """
        Log error with context for debugging.

        Args:
            error: Exception to log
            context: Optional context dictionary (redacted of sensitive data)
        """
        context_str = ""
        if context:
            # Redact sensitive data
            safe_context = {
                k: v for k, v in context.items() if k not in ["api_key", "password", "token"]
            }
            context_str = f" Context: {safe_context}"

        logger.error(f"{type(error).__name__}: {str(error)}{context_str}")
