"""End-to-end integration tests for source attribution in Q&A pipeline."""

import pytest
from unittest.mock import MagicMock, patch, AsyncMock

from src.models import ChatRequest, ChatResponse
from src.services.chat_pipeline_service import ChatPipelineService
from src.utils.response_formatter import ResponseFormatter


class TestSourceAttributionPipeline:
    """Test suite for end-to-end source attribution in the Q&A pipeline."""

    @pytest.mark.asyncio
    async def test_e2e_pipeline_includes_source_metadata(
        self, mock_embedding_service, mock_qdrant_client, mock_groq_client
    ) -> None:
        """Test that E2E pipeline includes complete source metadata."""
        request = ChatRequest(question="What is ROS 2?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        # Return comprehensive source information
                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 2: ROS 2 Fundamentals",
                                "module": "Module 2.1: Nodes and Topics",
                                "section": "ros-2-topics",
                                "excerpt": "ROS 2 is a middleware platform that uses publish-subscribe for communication.",
                                "similarity": 0.92,
                            }
                        ]

                        mock_generate.return_value = (
                            "ROS 2 is a middleware platform for building robot applications "
                            "that uses publish-subscribe messaging for inter-node communication."
                        )

                        mock_confidence.return_value = "high"

                        # Process question
                        response = await pipeline.process_question(request)

                        # Verify response
                        assert isinstance(response, ChatResponse)
                        assert len(response.sources) >= 1

                        # Verify source metadata is complete
                        source = response.sources[0]
                        assert source.chapter == "Chapter 2: ROS 2 Fundamentals"
                        assert source.module == "Module 2.1: Nodes and Topics"
                        assert source.section == "ros-2-topics"
                        assert source.excerpt  # Should have excerpt
                        assert source.similarity == 0.92

    @pytest.mark.asyncio
    async def test_e2e_pipeline_source_links_are_valid(
        self,
    ) -> None:
        """Test that generated source links are valid and clickable."""
        request = ChatRequest(question="How does publish-subscribe work?")
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
                                "chapter": "Chapter 2: ROS 2 Fundamentals",
                                "module": "Module 2.1: Nodes and Topics",
                                "section": "publish-subscribe",
                                "excerpt": "Publish-subscribe enables decoupled communication.",
                                "similarity": 0.94,
                            }
                        ]

                        mock_generate.return_value = "Publish-subscribe decouples senders from receivers."

                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify sources can generate valid links
                        source = response.sources[0]
                        link = ResponseFormatter.generate_source_link(
                            chapter=source.chapter,
                            module=source.module,
                            section=source.section
                        )

                        # Verify link format
                        assert link.startswith("/docs/")
                        assert "#" in link
                        assert "publish-subscribe" in link

    @pytest.mark.asyncio
    async def test_e2e_pipeline_multiple_sources_with_metadata(
        self,
    ) -> None:
        """Test that multiple sources are returned with correct metadata."""
        request = ChatRequest(question="What is a ROS node?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        # Return multiple sources
                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 2: ROS 2 Fundamentals",
                                "module": "Module 2.1: Nodes and Topics",
                                "section": "ros-2-nodes",
                                "excerpt": "ROS 2 nodes are processes that perform computation.",
                                "similarity": 0.96,
                            },
                            {
                                "chapter": "Chapter 2: ROS 2 Fundamentals",
                                "module": "Module 2.2: Services",
                                "section": "ros-2-services",
                                "excerpt": "Nodes communicate through services for request-response patterns.",
                                "similarity": 0.88,
                            }
                        ]

                        mock_generate.return_value = (
                            "A ROS 2 node is a computational unit that can send and receive messages "
                            "through topics and services."
                        )

                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify we have multiple sources
                        assert len(response.sources) == 2

                        # Verify first source (highest similarity)
                        assert response.sources[0].section == "ros-2-nodes"
                        assert response.sources[0].similarity == 0.96

                        # Verify second source
                        assert response.sources[1].section == "ros-2-services"
                        assert response.sources[1].similarity == 0.88

                        # Verify both sources have valid metadata
                        for source in response.sources:
                            assert source.chapter
                            assert source.module
                            assert source.section
                            assert source.excerpt

    @pytest.mark.asyncio
    async def test_e2e_pipeline_excerpt_from_retrieval(
        self,
    ) -> None:
        """Test that excerpts from retrieval are preserved accurately."""
        long_excerpt = (
            "ROS 2 is a middleware platform that provides tools and libraries for building "
            "robot applications. It is the successor to ROS (Robot Operating System) and "
            "includes improvements for production robotics."
        )

        request = ChatRequest(question="What is ROS 2?")
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
                                "section": "intro",
                                "excerpt": long_excerpt,
                                "similarity": 0.92,
                            }
                        ]

                        mock_generate.return_value = "ROS 2 is middleware for robotics."

                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify excerpt is preserved without truncation
                        assert response.sources[0].excerpt == long_excerpt
                        assert "middleware" in response.sources[0].excerpt
                        assert "improvements" in response.sources[0].excerpt

    @pytest.mark.asyncio
    async def test_e2e_pipeline_source_section_slugified(
        self,
    ) -> None:
        """Test that source sections are properly slugified for URLs."""
        request = ChatRequest(question="What is NVIDIA Isaac?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        # Section is already slugified from retrieval
                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 4: NVIDIA Isaac Platform",
                                "module": "Module 4.1: Isaac Overview",
                                "section": "isaac-overview",  # Slugified
                                "excerpt": "NVIDIA Isaac is a GPU-accelerated robotics platform.",
                                "similarity": 0.91,
                            }
                        ]

                        mock_generate.return_value = "NVIDIA Isaac provides simulation and tools."

                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify section is valid URL-safe slug
                        section = response.sources[0].section
                        assert section == "isaac-overview"
                        assert "-" in section or section.isalnum()
                        assert not any(c in section for c in [" ", "/", "\\", "?", "#"])

    @pytest.mark.asyncio
    async def test_e2e_pipeline_source_accuracy(
        self,
    ) -> None:
        """Test that sources accurately match retrieved content."""
        request = ChatRequest(question="How do I use topics?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        expected_excerpt = (
                            "Topics are the primary mechanism for data exchange between ROS 2 nodes. "
                            "A node publishes a message to a topic, and any node subscribed to that "
                            "topic receives the message."
                        )

                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 2: ROS 2 Fundamentals",
                                "module": "Module 2.1: Topics",
                                "section": "ros-2-topics",
                                "excerpt": expected_excerpt,
                                "similarity": 0.95,
                            }
                        ]

                        mock_generate.return_value = "Topics use publish-subscribe for asynchronous communication."

                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify source excerpt matches exactly
                        source = response.sources[0]
                        assert source.excerpt == expected_excerpt
                        assert source.chapter == "Chapter 2: ROS 2 Fundamentals"
                        assert source.module == "Module 2.1: Topics"


class TestSourceAttributionErrors:
    """Test error handling in source attribution."""

    @pytest.mark.asyncio
    async def test_e2e_pipeline_handles_missing_metadata(
        self,
    ) -> None:
        """Test that pipeline handles sources with missing optional metadata gracefully."""
        request = ChatRequest(question="What is ROS 2?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(
                        pipeline.confidence_service, "calculate_confidence"
                    ) as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536

                        # Source with minimal metadata
                        mock_search.return_value = [
                            {
                                "chapter": "Unknown",
                                "module": "Unknown",
                                "section": "unknown",
                                "excerpt": "Some content",
                                # No similarity provided
                            }
                        ]

                        mock_generate.return_value = "Answer"

                        mock_confidence.return_value = "low"

                        response = await pipeline.process_question(request)

                        # Should still return valid response
                        assert isinstance(response, ChatResponse)
                        assert len(response.sources) >= 1
                        assert response.sources[0].chapter == "Unknown"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
