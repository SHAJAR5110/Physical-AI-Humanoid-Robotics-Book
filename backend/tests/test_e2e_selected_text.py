"""End-to-end integration tests for selected_text flow in Q&A pipeline."""

import pytest
from unittest.mock import MagicMock, patch, AsyncMock

from src.models import ChatRequest, ChatResponse
from src.services.chat_pipeline_service import ChatPipelineService


class TestSelectedTextPipeline:
    """Test suite for selected_text flow through the full Q&A pipeline."""

    @pytest.mark.asyncio
    async def test_e2e_pipeline_with_selected_text(
        self, mock_embedding_service, mock_qdrant_client, mock_groq_client
    ) -> None:
        """Test end-to-end Q&A pipeline with selected_text."""
        # Request with both question and selected_text
        request = ChatRequest(
            question="How does this pattern work?",
            selected_text="The publish-subscribe pattern enables decoupled communication.",
        )

        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        # Set up mock returns
                        mock_embed.return_value = [0.1] * 1536

                        # Mocked search that returns results
                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 2: ROS 2 Fundamentals",
                                "module": "Module 2.1: Nodes and Topics",
                                "section": "pubsub-pattern",
                                "excerpt": "The publish-subscribe pattern enables decoupled communication between nodes.",
                                "similarity": 0.95,  # Boosted due to selected_text match
                            }
                        ]

                        mock_generate.return_value = (
                            "Based on the selected text, the publish-subscribe pattern allows nodes to communicate "
                            "without direct knowledge of each other, using topics as intermediaries."
                        )

                        mock_confidence.return_value = "high"

                        # Process question with selected_text
                        response = await pipeline.process_question(request)

                        # Verify response
                        assert isinstance(response, ChatResponse)
                        assert len(response.answer) > 0
                        assert len(response.sources) >= 1
                        assert response.confidence == "high"
                        assert response.processing_time_ms > 0

                        # Verify that selected_text was passed to retrieval service
                        mock_search.assert_called_once()
                        call_kwargs = mock_search.call_args.kwargs
                        assert call_kwargs.get("selected_text") == request.selected_text

                        # Verify that selected_text was passed to generation service
                        mock_generate.assert_called_once()
                        gen_call_kwargs = mock_generate.call_args.kwargs
                        assert gen_call_kwargs.get("selected_text") == request.selected_text

    @pytest.mark.asyncio
    async def test_e2e_pipeline_selected_text_prioritizes_results(
        self,
    ) -> None:
        """Test that selected_text prioritizes relevant results in the pipeline."""
        request = ChatRequest(
            question="What about Isaac?",
            selected_text="NVIDIA Isaac platform",
        )

        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        # Return a result that matches the selected_text
                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 4: NVIDIA Isaac Platform",
                                "module": "Module 4.1: Isaac Introduction",
                                "section": "isaac-overview",
                                "excerpt": "NVIDIA Isaac platform provides tools for building AI robots.",
                                "similarity": 0.96,
                            }
                        ]

                        mock_generate.return_value = "NVIDIA Isaac is a comprehensive robotics platform."

                        mock_confidence.return_value = "high"

                        # Process
                        response = await pipeline.process_question(request)

                        # Verify selected_text was used
                        search_call_kwargs = mock_search.call_args.kwargs
                        assert search_call_kwargs.get("selected_text") == "NVIDIA Isaac platform"

                        # Verify result contains information matching selected_text
                        assert "Isaac" in response.sources[0]["chapter"]

    @pytest.mark.asyncio
    async def test_e2e_pipeline_without_selected_text_still_works(
        self,
    ) -> None:
        """Test that pipeline still works when selected_text is not provided."""
        request = ChatRequest(
            question="What is ROS 2?",
            selected_text=None,
        )

        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 2",
                                "module": "Module 2.1",
                                "section": "ros-2",
                                "excerpt": "ROS 2 is middleware.",
                                "similarity": 0.90,
                            }
                        ]

                        mock_generate.return_value = "ROS 2 is a middleware platform."

                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify selected_text=None is passed
                        search_call_kwargs = mock_search.call_args.kwargs
                        assert search_call_kwargs.get("selected_text") is None

                        # Pipeline should still return valid response
                        assert isinstance(response, ChatResponse)
                        assert response.answer

    @pytest.mark.asyncio
    async def test_e2e_pipeline_selected_text_empty_treated_as_none(
        self,
    ) -> None:
        """Test that empty selected_text is treated as None in pipeline."""
        request = ChatRequest(
            question="What is this?",
            selected_text="   ",  # Only whitespace, becomes None
        )

        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        mock_search.return_value = [
                            {
                                "chapter": "Ch1",
                                "module": "Mod1",
                                "section": "sec1",
                                "excerpt": "Text",
                                "similarity": 0.90,
                            }
                        ]

                        mock_generate.return_value = "Answer"
                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify selected_text becomes None
                        assert request.selected_text is None
                        search_call_kwargs = mock_search.call_args.kwargs
                        assert search_call_kwargs.get("selected_text") is None

    @pytest.mark.asyncio
    async def test_e2e_pipeline_response_includes_sources(
        self,
    ) -> None:
        """Test that selected_text answers include accurate source attribution."""
        request = ChatRequest(
            question="How do I use this?",
            selected_text="Use the Isaac Sim API for interfacing with the simulation.",
        )

        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 4: NVIDIA Isaac Platform",
                                "module": "Module 4.3: Isaac Sim API",
                                "section": "isaac-sim-api",
                                "excerpt": "Use the Isaac Sim API for interfacing with the simulation. The API provides methods for setting object poses.",
                                "similarity": 0.97,
                            },
                            {
                                "chapter": "Chapter 4: NVIDIA Isaac Platform",
                                "module": "Module 4.2: Isaac Basics",
                                "section": "isaac-overview",
                                "excerpt": "Isaac Sim is a GPU-accelerated simulation platform.",
                                "similarity": 0.88,
                            },
                        ]

                        mock_generate.return_value = "The Isaac Sim API provides methods for interacting with the simulation environment."

                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify sources are present
                        assert len(response.sources) >= 1

                        # Verify source details
                        first_source = response.sources[0]
                        assert first_source["chapter"]
                        assert first_source["module"]
                        assert first_source["section"]
                        assert first_source["excerpt"]
                        assert first_source["similarity"] >= 0.85

    @pytest.mark.asyncio
    async def test_e2e_pipeline_selected_text_long(
        self,
    ) -> None:
        """Test that pipeline handles long selected_text (near 5000 char limit)."""
        long_text = "Lorem ipsum dolor sit amet. " * 100  # ~2800 chars
        request = ChatRequest(
            question="What about this long passage?",
            selected_text=long_text,
        )

        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536
                        mock_search.return_value = [
                            {
                                "chapter": "Ch",
                                "module": "Mod",
                                "section": "sec",
                                "excerpt": "Content",
                                "similarity": 0.90,
                            }
                        ]
                        mock_generate.return_value = "Answer"
                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Should handle long selected_text successfully
                        assert isinstance(response, ChatResponse)

                        # Verify long selected_text was passed through
                        gen_call_kwargs = mock_generate.call_args.kwargs
                        assert gen_call_kwargs.get("selected_text") == long_text


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
