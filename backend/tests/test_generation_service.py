"""Tests for generation service."""

import pytest
from unittest.mock import MagicMock, patch
from src.services.generation_service import GenerationService, GenerationError


class TestGenerationService:
    """Test suite for GenerationService."""

    def test_initialization(self) -> None:
        """Test service initialization."""
        service = GenerationService()
        assert service.model_name == "mixtral-8x7b-32768"
        assert service.temperature == 0.3
        assert service.client is None

    def test_generate_answer_success(self, mock_groq_client) -> None:
        """Test successful answer generation."""
        service = GenerationService()
        service.client = mock_groq_client

        question = "What is ROS 2?"
        context = "ROS 2 is a middleware that enables robot communication through topics and services."

        answer = service.generate_answer(
            question=question, context=context, max_tokens=512
        )

        assert isinstance(answer, str)
        assert len(answer) > 0
        assert "ROS 2" in answer or "middleware" in answer

    def test_generate_answer_respects_max_tokens(self, mock_groq_client) -> None:
        """Test that answer respects max_tokens limit."""
        service = GenerationService()
        service.client = mock_groq_client

        question = "What is ROS 2?"
        context = "ROS 2 is a middleware..."

        # Mock to return a message within token limit
        message_mock = MagicMock()
        message_mock.content = "Short answer."
        choice_mock = MagicMock()
        choice_mock.message = message_mock
        response_mock = MagicMock()
        response_mock.choices = [choice_mock]

        mock_groq_client.chat.completions.create.return_value = response_mock

        answer = service.generate_answer(
            question=question, context=context, max_tokens=50
        )

        # Verify the API was called with correct max_tokens
        call_args = mock_groq_client.chat.completions.create.call_args
        assert call_args[1]["max_tokens"] <= 50 + 10  # Some buffer

    def test_generate_answer_uses_system_prompt(self, mock_groq_client) -> None:
        """Test that generation uses correct system prompt."""
        service = GenerationService()
        service.client = mock_groq_client

        question = "What is ROS 2?"
        context = "ROS 2 is a middleware..."

        answer = service.generate_answer(question=question, context=context)

        # Verify system prompt was set
        call_args = mock_groq_client.chat.completions.create.call_args
        messages = call_args[1]["messages"]

        # Check that first message has system role
        assert messages[0]["role"] == "system"
        assert "context" in messages[0]["content"].lower()

    def test_generate_answer_api_failure(self, mock_groq_client) -> None:
        """Test error handling for API failures."""
        service = GenerationService()
        service.client = mock_groq_client

        # Mock API failure
        mock_groq_client.chat.completions.create.side_effect = Exception(
            "API Error"
        )

        question = "What is ROS 2?"
        context = "ROS 2 is a middleware..."

        with pytest.raises(GenerationError):
            service.generate_answer(question=question, context=context)

    def test_generate_answer_empty_context(self, mock_groq_client) -> None:
        """Test answer generation with empty context."""
        service = GenerationService()
        service.client = mock_groq_client

        question = "What is ROS 2?"
        context = ""

        # Should still attempt to generate answer
        answer = service.generate_answer(question=question, context=context)

        assert isinstance(answer, str)

    def test_generate_answer_long_question(self, mock_groq_client) -> None:
        """Test with very long question."""
        service = GenerationService()
        service.client = mock_groq_client

        question = "What " * 100  # Very long question
        context = "ROS 2 is a middleware..."

        answer = service.generate_answer(question=question, context=context)

        assert isinstance(answer, str)

    def test_health_check(self, mock_groq_client) -> None:
        """Test health check method."""
        service = GenerationService()
        service.client = mock_groq_client

        # Mock models list
        model_mock = MagicMock()
        model_mock.id = "mixtral-8x7b-32768"
        mock_groq_client.models.list.return_value = [model_mock]

        is_healthy = service.health_check()

        assert is_healthy is True

    def test_health_check_no_client(self) -> None:
        """Test health check when client not initialized."""
        service = GenerationService()
        service.client = None

        is_healthy = service.health_check()

        assert is_healthy is False

    def test_get_model_info(self) -> None:
        """Test getting model information."""
        service = GenerationService()

        info = service.get_model_info()

        assert info["model"] == "mixtral-8x7b-32768"
        assert "temperature" in info
        assert info["temperature"] == 0.3


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
