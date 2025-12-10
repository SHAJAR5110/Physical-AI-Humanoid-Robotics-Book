"""End-to-end integration tests for full Q&A pipeline."""

import pytest
from unittest.mock import MagicMock, patch, AsyncMock
import numpy as np

from src.models import ChatRequest, ChatResponse
from src.services.chat_pipeline_service import ChatPipelineService


class TestQAPipeline:
    """Test suite for full Q&A pipeline integration."""

    @pytest.mark.asyncio
    async def test_e2e_qa_pipeline_success(
        self, mock_embedding_service, mock_qdrant_client, mock_groq_client
    ) -> None:
        """Test end-to-end Q&A pipeline with mocked services."""
        # Note: This is a simplified integration test
        # In real scenario, we would patch the services properly

        request = ChatRequest(question="What is ROS 2?")

        # Create pipeline
        pipeline = ChatPipelineService()

        # Mock the services
        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(pipeline.confidence_service, "calculate_confidence") as mock_confidence:

                        # Set up mock returns
                        mock_embed.return_value = [0.1] * 1536

                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 2: ROS 2 Fundamentals",
                                "module": "Module 2.1: Nodes and Topics",
                                "section": "ros-2-topics",
                                "excerpt": "ROS 2 is a middleware for robotics communication.",
                                "similarity": 0.92,
                            }
                        ]

                        mock_generate.return_value = "ROS 2 is a middleware platform for building robot applications."

                        mock_confidence.return_value = "high"

                        # Process question
                        response = await pipeline.process_question(request)

                        # Verify response
                        assert isinstance(response, ChatResponse)
                        assert len(response.answer) > 0
                        assert len(response.sources) >= 1
                        assert response.confidence in ["high", "medium", "low"]
                        assert response.processing_time_ms > 0

    @pytest.mark.asyncio
    async def test_e2e_pipeline_response_schema(
        self, mock_embedding_service, mock_qdrant_client, mock_groq_client
    ) -> None:
        """Test that pipeline response matches ChatResponse schema."""
        request = ChatRequest(question="What is Gazebo?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(pipeline.confidence_service, "calculate_confidence") as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536
                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 3",
                                "module": "Module 3.1",
                                "section": "gazebo-intro",
                                "excerpt": "Gazebo is a simulator...",
                                "similarity": 0.85,
                            }
                        ]
                        mock_generate.return_value = "Gazebo is a 3D robotics simulator."
                        mock_confidence.return_value = "medium"

                        response = await pipeline.process_question(request)

                        # Verify schema
                        assert hasattr(response, "answer")
                        assert hasattr(response, "sources")
                        assert hasattr(response, "confidence")
                        assert hasattr(response, "processing_time_ms")

                        # Verify types
                        assert isinstance(response.answer, str)
                        assert isinstance(response.sources, list)
                        assert isinstance(response.confidence, str)
                        assert isinstance(response.processing_time_ms, int)

    @pytest.mark.asyncio
    async def test_e2e_pipeline_source_reference(
        self, mock_embedding_service, mock_qdrant_client, mock_groq_client
    ) -> None:
        """Test that pipeline includes source references in response."""
        request = ChatRequest(question="What is NVIDIA Isaac?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(pipeline.confidence_service, "calculate_confidence") as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536
                        mock_search.return_value = [
                            {
                                "chapter": "Chapter 4: NVIDIA Isaac Platform",
                                "module": "Module 4.1: Isaac Sim",
                                "section": "isaac-sim-intro",
                                "excerpt": "NVIDIA Isaac Sim is a physics-based simulation platform.",
                                "similarity": 0.89,
                            }
                        ]
                        mock_generate.return_value = "NVIDIA Isaac is a suite of tools for robotics simulation and deployment."
                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Verify sources are included
                        assert len(response.sources) > 0
                        source = response.sources[0]
                        assert source.chapter == "Chapter 4: NVIDIA Isaac Platform"
                        assert source.module == "Module 4.1: Isaac Sim"
                        assert "Isaac" in source.excerpt

    @pytest.mark.asyncio
    async def test_e2e_pipeline_processing_time(
        self, mock_embedding_service, mock_qdrant_client, mock_groq_client
    ) -> None:
        """Test that processing time is tracked."""
        request = ChatRequest(question="Question?")
        pipeline = ChatPipelineService()

        with patch.object(pipeline.embedding_service, "embed_text") as mock_embed:
            with patch.object(pipeline.retrieval_service, "search") as mock_search:
                with patch.object(pipeline.generation_service, "generate_answer") as mock_generate:
                    with patch.object(pipeline.confidence_service, "calculate_confidence") as mock_confidence:

                        mock_embed.return_value = [0.1] * 1536
                        mock_search.return_value = [
                            {
                                "chapter": "Ch",
                                "module": "M",
                                "section": "S",
                                "excerpt": "E",
                                "similarity": 0.9,
                            }
                        ]
                        mock_generate.return_value = "Answer"
                        mock_confidence.return_value = "high"

                        response = await pipeline.process_question(request)

                        # Processing time should be positive
                        assert response.processing_time_ms > 0
                        # Should be reasonable (less than 30 seconds for mocked services)
                        assert response.processing_time_ms < 30000

    @pytest.mark.asyncio
    async def test_e2e_pipeline_input_validation(
        self, mock_embedding_service, mock_qdrant_client, mock_groq_client
    ) -> None:
        """Test that pipeline validates input."""
        pipeline = ChatPipelineService()

        # Test with invalid question (too short)
        with pytest.raises(ValueError):
            invalid_request = ChatRequest(question="ab")
            await pipeline.process_question(invalid_request)

        # Test with invalid question (too long)
        with pytest.raises(ValueError):
            invalid_request = ChatRequest(question="a" * 1001)
            await pipeline.process_question(invalid_request)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
