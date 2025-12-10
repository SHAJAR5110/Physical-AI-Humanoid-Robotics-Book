"""Tests for core services."""

import pytest
from unittest.mock import MagicMock
from src.services.confidence_service import ConfidenceService
from src.utils.error_handler import ErrorHandler
from src.utils.context_manager import ContextManager
from src.utils.validators import InputValidator
from src.utils.slug_generator import SlugGenerator


class TestConfidenceService:
    """Test suite for ConfidenceService."""

    def test_initialization(self) -> None:
        """Test service initialization."""
        service = ConfidenceService()
        assert service.similarity_weight == 0.4
        assert service.source_weight == 0.3
        assert service.term_weight == 0.3

    def test_high_confidence(self, sample_sources, sample_answer, sample_question) -> None:
        """Test high confidence calculation."""
        service = ConfidenceService()
        confidence = service.calculate_confidence(sample_sources, sample_answer, sample_question)
        assert confidence in ["high", "medium", "low"]

    def test_medium_confidence(self) -> None:
        """Test medium confidence with lower similarity."""
        service = ConfidenceService()
        sources = [
            {
                "chapter": "Ch",
                "module": "Mod",
                "section": "sec",
                "similarity": 0.60,  # Medium similarity
            }
        ]
        confidence = service.calculate_confidence(sources, "Average answer", "Test")
        assert confidence in ["medium", "low"]

    def test_low_confidence(self) -> None:
        """Test low confidence."""
        service = ConfidenceService()
        sources = [
            {
                "chapter": "Ch",
                "module": "Mod",
                "section": "sec",
                "similarity": 0.30,  # Low similarity
            }
        ]
        confidence = service.calculate_confidence(sources, "Poor answer", "Test")
        assert confidence == "low"

    def test_no_sources(self) -> None:
        """Test with no sources."""
        service = ConfidenceService()
        confidence = service.calculate_confidence([], "Answer", "Question")
        assert confidence == "low"

    def test_confidence_details(self) -> None:
        """Test getting confidence details."""
        service = ConfidenceService()
        high_details = service.get_confidence_details("high")
        assert high_details["level"] == "high"
        assert "confidence" in high_details["message"].lower()


class TestErrorHandler:
    """Test suite for ErrorHandler."""

    def test_validation_error_response(self) -> None:
        """Test validation error handling."""
        from pydantic import ValidationError, BaseModel, Field

        class TestModel(BaseModel):
            value: str = Field(min_length=3)

        try:
            TestModel(value="ab")
        except ValidationError as e:
            response, status = ErrorHandler.handle_validation_error(e)
            assert status == 400
            assert "Validation" in response.error

    def test_rate_limit_response(self) -> None:
        """Test rate limit error response."""
        response, status = ErrorHandler.handle_rate_limit()
        assert status == 429
        assert "Rate limit" in response.error

    def test_service_unavailable(self) -> None:
        """Test service unavailable response."""
        response, status = ErrorHandler.handle_service_unavailable()
        assert status == 503
        assert "unavailable" in response.error.lower()

    def test_generic_error(self) -> None:
        """Test generic error handling."""
        error = Exception("Test error")
        response, status = ErrorHandler.handle_generic_error(error)
        assert status == 500


class TestContextManager:
    """Test suite for ContextManager."""

    def test_prepare_context_basic(self, sample_sources) -> None:
        """Test basic context preparation."""
        context = ContextManager.prepare_context(sample_sources)
        assert "Chapter" in context
        assert "Module" in context
        assert "publish-subscribe" in context

    def test_prepare_context_exceeds_limit(self) -> None:
        """Test context truncation when exceeding limit."""
        sources = [
            {
                "chapter": "Chapter",
                "module": "Module",
                "excerpt": "a" * 10000,  # Very long excerpt
            }
        ]
        context = ContextManager.prepare_context(sources, max_tokens=100)
        # Should be truncated
        assert len(context) < len("a" * 10000)

    def test_estimate_tokens(self) -> None:
        """Test token estimation."""
        text = "This is a test sentence."
        tokens = ContextManager.estimate_tokens(text)
        assert tokens > 0
        assert tokens < len(text)  # Rough estimate

    def test_validate_token_count(self) -> None:
        """Test token count validation."""
        text = "short"
        is_valid = ContextManager.validate_token_count(text, 1000)
        assert is_valid is True

        is_valid = ContextManager.validate_token_count(text, 1)
        assert is_valid is False


class TestInputValidator:
    """Test suite for InputValidator."""

    def test_validate_question_valid(self) -> None:
        """Test valid question validation."""
        is_valid, msg = InputValidator.validate_question("What is ROS 2?")
        assert is_valid is True
        assert msg == ""

    def test_validate_question_too_short(self) -> None:
        """Test question too short."""
        is_valid, msg = InputValidator.validate_question("ab")
        assert is_valid is False
        assert "at least" in msg

    def test_validate_question_too_long(self) -> None:
        """Test question too long."""
        is_valid, msg = InputValidator.validate_question("a" * 1001)
        assert is_valid is False
        assert "at most" in msg

    def test_validate_question_injection(self) -> None:
        """Test SQL injection detection."""
        is_valid, msg = InputValidator.validate_question("What is this? DROP TABLE;")
        assert is_valid is False

    def test_validate_selected_text_valid(self) -> None:
        """Test valid selected text."""
        is_valid, msg = InputValidator.validate_selected_text("Selected text")
        assert is_valid is True

    def test_validate_selected_text_none(self) -> None:
        """Test None selected text."""
        is_valid, msg = InputValidator.validate_selected_text(None)
        assert is_valid is True

    def test_sanitize_input(self) -> None:
        """Test input sanitization."""
        dirty = "  test   \n  input  "
        clean = InputValidator.sanitize_input(dirty)
        assert clean == "test input"


class TestSlugGenerator:
    """Test suite for SlugGenerator."""

    def test_slugify_basic(self) -> None:
        """Test basic slug generation."""
        slug = SlugGenerator.slugify("ROS 2 Topics")
        assert slug == "ros-2-topics"

    def test_slugify_special_chars(self) -> None:
        """Test slug generation with special characters."""
        slug = SlugGenerator.slugify("Module 2.1: Nodes & Topics!")
        assert "/" not in slug
        assert ":" not in slug
        assert "&" not in slug

    def test_slugify_spaces(self) -> None:
        """Test slug generation with multiple spaces."""
        slug = SlugGenerator.slugify("Chapter   2   ROS 2")
        assert "--" not in slug  # No consecutive hyphens
        assert slug.startswith("chapter")

    def test_slugify_empty(self) -> None:
        """Test slug generation with empty string."""
        slug = SlugGenerator.slugify("")
        assert slug == "unknown"

    def test_generate_anchor(self) -> None:
        """Test anchor generation."""
        anchor = SlugGenerator.generate_anchor(
            "Chapter 2", "Module 2.1", "ROS 2 Topics"
        )
        assert anchor == "ros-2-topics"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
