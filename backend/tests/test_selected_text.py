"""Tests for selected_text handling in retrieval and generation services."""

import pytest
from unittest.mock import MagicMock, patch
from src.services.retrieval_service import RetrievalService, RetrievalError
from src.services.generation_service import GenerationService


class TestRetrievalServiceSelectedText:
    """Test suite for RetrievalService with selected_text prioritization."""

    def test_search_with_selected_text_prioritizes_matching(
        self, mock_qdrant_client
    ) -> None:
        """Test that selected_text prioritizes passages containing it."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536
        selected_text = "publish-subscribe pattern"

        # Create mock results: one matches selected_text, one doesn't
        result1 = MagicMock()
        result1.score = 0.88
        result1.payload = {
            "chapter": "Chapter 2",
            "module": "Module 2.1",
            "section": "topics",
            "text": "ROS 2 uses a publish-subscribe pattern for communication.",
        }

        result2 = MagicMock()
        result2.score = 0.90  # Higher score but doesn't contain selected_text
        result2.payload = {
            "chapter": "Chapter 3",
            "module": "Module 3.1",
            "section": "gazebo",
            "text": "Gazebo is a simulation platform for robotics.",
        }

        mock_qdrant_client.search.return_value = [result1, result2]

        # Search with selected_text
        results = service.search(
            embedding=embedding, top_k=5, selected_text=selected_text
        )

        # Verify that result1 (matching selected_text) comes first
        assert results[0]["chapter"] == "Chapter 2"
        assert results[0]["similarity"] > 0.88  # Boosted from 0.88

    def test_search_with_selected_text_boosts_similarity(
        self, mock_qdrant_client
    ) -> None:
        """Test that selected_text causes similarity boost."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536
        selected_text = "ROS 2 nodes"

        result = MagicMock()
        result.score = 0.87
        result.payload = {
            "chapter": "Chapter 2: ROS 2 Fundamentals",
            "module": "Module 2.1: Nodes and Topics",
            "section": "ros-2-nodes",
            "text": "ROS 2 nodes are processes that perform computation. Nodes communicate using ROS 2 nodes topics...",
        }

        mock_qdrant_client.search.return_value = [result]

        results = service.search(
            embedding=embedding, top_k=5, selected_text=selected_text
        )

        # Similarity should be boosted by 0.1
        assert results[0]["similarity"] == pytest.approx(0.97, abs=0.01)

    def test_search_with_selected_text_case_insensitive(
        self, mock_qdrant_client
    ) -> None:
        """Test that selected_text matching is case-insensitive."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536
        selected_text = "PUBLISH-SUBSCRIBE"  # All caps

        result = MagicMock()
        result.score = 0.89
        result.payload = {
            "chapter": "Chapter 2",
            "module": "Module 2.1",
            "section": "pubsub",
            "text": "The publish-subscribe pattern is fundamental to ROS 2.",  # Lower case
        }

        mock_qdrant_client.search.return_value = [result]

        results = service.search(
            embedding=embedding, top_k=5, selected_text=selected_text
        )

        # Should find match despite case difference
        assert results[0]["similarity"] > 0.89  # Boosted

    def test_search_without_selected_text_no_boost(
        self, mock_qdrant_client
    ) -> None:
        """Test that search without selected_text doesn't boost similarity."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536

        result = MagicMock()
        result.score = 0.91
        result.payload = {
            "chapter": "Chapter 2",
            "module": "Module 2.1",
            "section": "topics",
            "text": "ROS 2 topics are used for communication.",
        }

        mock_qdrant_client.search.return_value = [result]

        results = service.search(embedding=embedding, top_k=5, selected_text=None)

        # Similarity should remain unchanged (no boost)
        assert results[0]["similarity"] == 0.91

    def test_search_selected_text_empty_string_ignored(
        self, mock_qdrant_client
    ) -> None:
        """Test that empty selected_text is treated as None."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536

        result = MagicMock()
        result.score = 0.89
        result.payload = {
            "chapter": "Ch", "module": "Mod", "section": "sec", "text": "Text here"
        }

        mock_qdrant_client.search.return_value = [result]

        # Empty string should not boost
        results = service.search(embedding=embedding, top_k=5, selected_text="")

        assert results[0]["similarity"] == 0.89  # No boost

    def test_search_selected_text_partial_match(
        self, mock_qdrant_client
    ) -> None:
        """Test that selected_text with partial matches is found."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536
        selected_text = "ROS 2"

        # Multiple results, some containing selected_text
        result1 = MagicMock()
        result1.score = 0.85
        result1.payload = {
            "chapter": "Ch1", "module": "Mod1", "section": "s1", "text": "ROS 2 is great"
        }

        result2 = MagicMock()
        result2.score = 0.87
        result2.payload = {
            "chapter": "Ch2", "module": "Mod2", "section": "s2", "text": "Gazebo simulation"
        }

        result3 = MagicMock()
        result3.score = 0.88
        result3.payload = {
            "chapter": "Ch3",
            "module": "Mod3",
            "section": "s3",
            "text": "ROS 2 is a middleware",
        }

        mock_qdrant_client.search.return_value = [result1, result2, result3]

        results = service.search(
            embedding=embedding, top_k=3, selected_text=selected_text
        )

        # Results containing "ROS 2" should be boosted and come first
        found_ros2_first = results[0]["chapter"] in ["Ch1", "Ch3"]
        assert found_ros2_first


class TestGenerationServiceSelectedText:
    """Test suite for GenerationService with selected_text."""

    def test_generate_answer_with_selected_text(
        self, mock_groq_client
    ) -> None:
        """Test that selected_text is included in the prompt."""
        service = GenerationService(api_key="test-key")
        service.client = mock_groq_client

        question = "What is publish-subscribe?"
        context = "ROS 2 uses pub/sub for messaging."
        selected_text = "The publish-subscribe pattern enables decoupled communication."

        answer = service.generate_answer(
            question=question,
            context=context,
            selected_text=selected_text,
        )

        # Verify the call was made
        assert mock_groq_client.chat.completions.create.called

        # Check that selected_text was included in the user message
        call_args = mock_groq_client.chat.completions.create.call_args
        messages = call_args.kwargs["messages"]

        # Find the user message
        user_message = next(msg for msg in messages if msg["role"] == "user")
        assert "Student selected this text" in user_message["content"]
        assert selected_text in user_message["content"]

    def test_generate_answer_without_selected_text(
        self, mock_groq_client
    ) -> None:
        """Test that generation works without selected_text."""
        service = GenerationService(api_key="test-key")
        service.client = mock_groq_client

        question = "What is ROS 2?"
        context = "ROS 2 is a middleware for robotics."

        answer = service.generate_answer(
            question=question, context=context, selected_text=None
        )

        # Verify the call was made
        assert mock_groq_client.chat.completions.create.called

        # Check that selected_text block is not in message
        call_args = mock_groq_client.chat.completions.create.call_args
        messages = call_args.kwargs["messages"]

        user_message = next(msg for msg in messages if msg["role"] == "user")
        assert "Student selected this text" not in user_message["content"]

    def test_generate_answer_selected_text_acknowledged(
        self, mock_groq_client
    ) -> None:
        """Test that prompt asks LLM to acknowledge selected text."""
        service = GenerationService(api_key="test-key")
        service.client = mock_groq_client

        question = "How does this work?"
        context = "Context here"
        selected_text = "Some selected text"

        service.generate_answer(
            question=question,
            context=context,
            selected_text=selected_text,
        )

        call_args = mock_groq_client.chat.completions.create.call_args
        messages = call_args.kwargs["messages"]

        user_message = next(msg for msg in messages if msg["role"] == "user")
        assert "answer based on the selected text" in user_message["content"]

    def test_generate_answer_selected_text_long(
        self, mock_groq_client
    ) -> None:
        """Test generation with long selected_text."""
        service = GenerationService(api_key="test-key")
        service.client = mock_groq_client

        question = "What is this about?"
        context = "Short context"
        selected_text = "A" * 1000  # Long selected text

        answer = service.generate_answer(
            question=question,
            context=context,
            selected_text=selected_text,
        )

        # Should succeed despite long selected_text
        assert mock_groq_client.chat.completions.create.called


class TestSelectedTextValidation:
    """Test validation of selected_text in chat request."""

    def test_chat_request_with_valid_selected_text(self) -> None:
        """Test ChatRequest accepts valid selected_text."""
        from src.models import ChatRequest

        request = ChatRequest(
            question="What is this?",
            selected_text="Some selected passage from the book",
        )

        assert request.selected_text == "Some selected passage from the book"

    def test_chat_request_selected_text_whitespace_stripped(self) -> None:
        """Test that selected_text is stripped of leading/trailing whitespace."""
        from src.models import ChatRequest

        request = ChatRequest(
            question="What is this?",
            selected_text="  Some text  ",
        )

        assert request.selected_text == "Some text"

    def test_chat_request_selected_text_empty_becomes_none(self) -> None:
        """Test that empty selected_text becomes None."""
        from src.models import ChatRequest

        request = ChatRequest(
            question="What is this?",
            selected_text="   ",  # Only whitespace
        )

        assert request.selected_text is None

    def test_chat_request_selected_text_too_long(self) -> None:
        """Test that selected_text > 5000 chars is rejected."""
        from src.models import ChatRequest
        import pytest

        with pytest.raises(ValueError):
            ChatRequest(
                question="What is this?",
                selected_text="A" * 5001,  # Over the limit
            )

    def test_chat_request_selected_text_optional(self) -> None:
        """Test that selected_text is optional."""
        from src.models import ChatRequest

        # Should work without selected_text
        request = ChatRequest(question="What is this?")

        assert request.selected_text is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
