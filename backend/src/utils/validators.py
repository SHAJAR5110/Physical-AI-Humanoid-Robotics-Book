"""Input validation utilities."""

import logging
import re

logger = logging.getLogger(__name__)


class InputValidator:
    """Validates user input for security and correctness."""

    MAX_QUESTION_LENGTH = 1000
    MIN_QUESTION_LENGTH = 3
    MAX_SELECTED_TEXT_LENGTH = 5000

    @staticmethod
    def validate_question(question: str) -> tuple[bool, str]:
        """Validate question input."""
        if not question or not isinstance(question, str):
            return False, "Question must be a non-empty string"

        q = question.strip()
        if len(q) < InputValidator.MIN_QUESTION_LENGTH:
            return False, f"Question must be at least {InputValidator.MIN_QUESTION_LENGTH} characters"

        if len(q) > InputValidator.MAX_QUESTION_LENGTH:
            return False, f"Question must be at most {InputValidator.MAX_QUESTION_LENGTH} characters"

        # Check for injection patterns
        dangerous = ["<script", "DROP ", "DELETE ", "INSERT ", "UPDATE "]
        if any(pattern in q.upper() for pattern in dangerous):
            return False, "Question contains invalid characters"

        return True, ""

    @staticmethod
    def validate_selected_text(text: str | None) -> tuple[bool, str]:
        """Validate selected text input."""
        if text is None:
            return True, ""

        if not isinstance(text, str):
            return False, "Selected text must be a string"

        t = text.strip()
        if not t:
            return True, ""  # Empty is OK (just ignore it)

        if len(t) > InputValidator.MAX_SELECTED_TEXT_LENGTH:
            return False, f"Selected text must be at most {InputValidator.MAX_SELECTED_TEXT_LENGTH} characters"

        return True, ""

    @staticmethod
    def sanitize_input(text: str) -> str:
        """Basic input sanitization."""
        # Remove null bytes
        text = text.replace("\x00", "")
        # Normalize whitespace
        text = re.sub(r"\s+", " ", text).strip()
        return text
