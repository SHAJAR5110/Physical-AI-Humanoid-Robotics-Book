"""Tests for Pydantic models."""

import pytest
from pydantic import ValidationError
from src.models import ChatRequest, ChatResponse, SourceRef, ErrorResponse


class TestChatRequest:
    """Test suite for ChatRequest model."""

    def test_valid_request_minimal(self) -> None:
        """Test minimal valid request."""
        request = ChatRequest(question="What is ROS 2?")
        assert request.question == "What is ROS 2?"
        assert request.selected_text is None

    def test_valid_request_with_selected_text(self) -> None:
        """Test request with selected text."""
        request = ChatRequest(
            question="What is this?",
            selected_text="Selected passage from book",
        )
        assert request.question == "What is this?"
        assert request.selected_text == "Selected passage from book"

    def test_question_too_short(self) -> None:
        """Test question shorter than minimum."""
        with pytest.raises(ValidationError):
            ChatRequest(question="ab")

    def test_question_too_long(self) -> None:
        """Test question longer than maximum."""
        with pytest.raises(ValidationError):
            ChatRequest(question="a" * 1001)

    def test_question_empty(self) -> None:
        """Test empty question."""
        with pytest.raises(ValidationError):
            ChatRequest(question="")

    def test_question_injection_attempt(self) -> None:
        """Test SQL injection attempt in question."""
        with pytest.raises(ValidationError):
            ChatRequest(question="What is this? DROP TABLE books;")

    def test_question_script_injection(self) -> None:
        """Test script injection attempt."""
        with pytest.raises(ValidationError):
            ChatRequest(question="<script>alert('xss')</script>")

    def test_selected_text_too_long(self) -> None:
        """Test selected text exceeding max length."""
        with pytest.raises(ValidationError):
            ChatRequest(
                question="Valid question",
                selected_text="a" * 5001,
            )

    def test_selected_text_whitespace_only(self) -> None:
        """Test selected text with only whitespace."""
        request = ChatRequest(
            question="Valid question",
            selected_text="   ",
        )
        assert request.selected_text is None

    def test_question_whitespace_stripping(self) -> None:
        """Test question is stripped."""
        request = ChatRequest(question="  test question  ")
        assert request.question == "test question"


class TestSourceRef:
    """Test suite for SourceRef model."""

    def test_valid_source(self) -> None:
        """Test valid source reference."""
        source = SourceRef(
            chapter="Chapter 2: ROS 2",
            module="Module 2.1",
            section="ros-2-topics",
            excerpt="ROS 2 is a middleware...",
        )
        assert source.chapter == "Chapter 2: ROS 2"
        assert source.module == "Module 2.1"
        assert source.section == "ros-2-topics"
        assert source.excerpt == "ROS 2 is a middleware..."

    def test_source_with_similarity(self) -> None:
        """Test source with similarity score."""
        source = SourceRef(
            chapter="Chapter 2",
            module="Module 2.1",
            section="ros-2",
            excerpt="content",
            similarity=0.92,
        )
        assert source.similarity == 0.92

    def test_source_similarity_out_of_range(self) -> None:
        """Test similarity score validation."""
        with pytest.raises(ValidationError):
            SourceRef(
                chapter="Chapter",
                module="Module",
                section="section",
                excerpt="content",
                similarity=1.5,  # Out of 0-1 range
            )


class TestChatResponse:
    """Test suite for ChatResponse model."""

    def test_valid_response(self) -> None:
        """Test valid chat response."""
        response = ChatResponse(
            answer="ROS 2 is a middleware for robotics.",
            sources=[
                SourceRef(
                    chapter="Chapter 2",
                    module="Module 2.1",
                    section="ros-2",
                    excerpt="ROS 2...",
                )
            ],
            confidence="high",
            processing_time_ms=1850,
        )
        assert response.confidence == "high"
        assert len(response.sources) == 1
        assert response.processing_time_ms == 1850

    def test_answer_too_short(self) -> None:
        """Test answer shorter than minimum."""
        with pytest.raises(ValidationError):
            ChatResponse(
                answer="short",
                sources=[
                    SourceRef(
                        chapter="Ch",
                        module="Mod",
                        section="sec",
                        excerpt="content",
                    )
                ],
                confidence="high",
                processing_time_ms=100,
            )

    def test_answer_too_long(self) -> None:
        """Test answer longer than maximum."""
        with pytest.raises(ValidationError):
            ChatResponse(
                answer="a" * 2001,
                sources=[
                    SourceRef(
                        chapter="Ch",
                        module="Mod",
                        section="sec",
                        excerpt="content",
                    )
                ],
                confidence="high",
                processing_time_ms=100,
            )

    def test_no_sources(self) -> None:
        """Test response without sources."""
        with pytest.raises(ValidationError):
            ChatResponse(
                answer="Valid answer here with enough length.",
                sources=[],
                confidence="high",
                processing_time_ms=100,
            )

    def test_invalid_confidence(self) -> None:
        """Test invalid confidence level."""
        with pytest.raises(ValidationError):
            ChatResponse(
                answer="Valid answer here with enough length.",
                sources=[
                    SourceRef(
                        chapter="Ch",
                        module="Mod",
                        section="sec",
                        excerpt="content",
                    )
                ],
                confidence="invalid",  # Not high/medium/low
                processing_time_ms=100,
            )

    def test_negative_processing_time(self) -> None:
        """Test negative processing time."""
        with pytest.raises(ValidationError):
            ChatResponse(
                answer="Valid answer here with enough length.",
                sources=[
                    SourceRef(
                        chapter="Ch",
                        module="Mod",
                        section="sec",
                        excerpt="content",
                    )
                ],
                confidence="high",
                processing_time_ms=-100,
            )


class TestErrorResponse:
    """Test suite for ErrorResponse model."""

    def test_valid_error(self) -> None:
        """Test valid error response."""
        error = ErrorResponse(
            error="Validation failed",
            details="Question must be at least 3 characters",
        )
        assert error.error == "Validation failed"
        assert "Question" in error.details


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
