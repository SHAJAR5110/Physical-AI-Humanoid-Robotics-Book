"""Tests for retrieval service."""

import pytest
import numpy as np
from unittest.mock import MagicMock, patch
from src.services.retrieval_service import RetrievalService, RetrievalError


class TestRetrievalService:
    """Test suite for RetrievalService."""

    def test_initialization(self) -> None:
        """Test service initialization."""
        service = RetrievalService()
        assert service.similarity_threshold == 0.85
        assert service.client is None

    def test_search_valid_embedding(self, mock_qdrant_client) -> None:
        """Test search with valid embedding returns filtered results."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        # Create a mock embedding (1536 dimensions)
        embedding = [0.1] * 1536

        # Mock search to return results
        search_result = MagicMock()
        search_result.score = 0.92
        search_result.payload = {
            "chapter": "Chapter 2: ROS 2 Fundamentals",
            "module": "Module 2.1: Nodes and Topics",
            "section": "ros-2-topics",
            "text": "ROS 2 is a middleware for robotics...",
        }
        mock_qdrant_client.search.return_value = [search_result]

        # Search
        results = service.search(embedding=embedding, top_k=5)

        # Verify
        assert len(results) == 1
        assert results[0]["chapter"] == "Chapter 2: ROS 2 Fundamentals"
        assert results[0]["similarity"] == 0.92

    def test_search_similarity_threshold_enforcement(self, mock_qdrant_client) -> None:
        """Test similarity threshold enforcement - no results < 0.85."""
        service = RetrievalService()
        service.client = mock_qdrant_client
        service.similarity_threshold = 0.85

        embedding = [0.1] * 1536

        # Mock search to return low-similarity result
        search_result = MagicMock()
        search_result.score = 0.70  # Below threshold
        search_result.payload = {
            "chapter": "Chapter 1",
            "module": "Module 1.1",
            "section": "intro",
            "text": "Introduction...",
        }
        mock_qdrant_client.search.return_value = [search_result]

        # Search should filter by threshold
        with pytest.raises(RetrievalError):
            service.search(embedding=embedding, top_k=5, similarity_threshold=0.85)

    def test_search_no_matching_results(self, mock_qdrant_client) -> None:
        """Test with no matching results raises RetrievalError."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536

        # Mock search to return empty list
        mock_qdrant_client.search.return_value = []

        # Should raise error
        with pytest.raises(RetrievalError):
            service.search(embedding=embedding, top_k=5)

    def test_search_metadata_extraction(self, mock_qdrant_client) -> None:
        """Test metadata extraction (chapter, module, section)."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536

        # Mock search result with all metadata fields
        search_result = MagicMock()
        search_result.score = 0.92
        search_result.payload = {
            "chapter": "Chapter 3: Gazebo Simulation",
            "module": "Module 3.2: Physics Engine",
            "section": "gazebo-physics",
            "text": "Gazebo uses ODE physics engine for realistic simulation...",
        }
        mock_qdrant_client.search.return_value = [search_result]

        results = service.search(embedding=embedding, top_k=1)

        assert results[0]["chapter"] == "Chapter 3: Gazebo Simulation"
        assert results[0]["module"] == "Module 3.2: Physics Engine"
        assert results[0]["section"] == "gazebo-physics"
        assert "Gazebo" in results[0]["excerpt"]

    def test_search_multiple_results_ordered(self, mock_qdrant_client) -> None:
        """Test multiple results returned in order of relevance."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536

        # Mock multiple search results in descending relevance order
        result1 = MagicMock()
        result1.score = 0.95
        result1.payload = {
            "chapter": "Ch1", "module": "Mod1", "section": "sec1", "text": "Text1"
        }

        result2 = MagicMock()
        result2.score = 0.88
        result2.payload = {
            "chapter": "Ch2", "module": "Mod2", "section": "sec2", "text": "Text2"
        }

        result3 = MagicMock()
        result3.score = 0.86
        result3.payload = {
            "chapter": "Ch3", "module": "Mod3", "section": "sec3", "text": "Text3"
        }

        mock_qdrant_client.search.return_value = [result1, result2, result3]

        results = service.search(embedding=embedding, top_k=3)

        assert len(results) == 3
        assert results[0]["similarity"] == 0.95  # Most relevant first
        assert results[1]["similarity"] == 0.88
        assert results[2]["similarity"] == 0.86

    def test_search_custom_threshold(self, mock_qdrant_client) -> None:
        """Test custom similarity threshold."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536

        search_result = MagicMock()
        search_result.score = 0.75
        search_result.payload = {
            "chapter": "Ch", "module": "Mod", "section": "sec", "text": "Text"
        }
        mock_qdrant_client.search.return_value = [search_result]

        # With custom lower threshold, should accept the result
        results = service.search(
            embedding=embedding, top_k=5, similarity_threshold=0.70
        )

        assert len(results) == 1

    def test_search_top_k_limit(self, mock_qdrant_client) -> None:
        """Test that search respects top_k limit."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        embedding = [0.1] * 1536

        # Create 10 mock results
        results_list = []
        for i in range(10):
            result = MagicMock()
            result.score = 0.95 - (i * 0.01)  # Decreasing scores
            result.payload = {
                "chapter": f"Ch{i}",
                "module": f"Mod{i}",
                "section": f"sec{i}",
                "text": f"Text{i}",
            }
            results_list.append(result)

        mock_qdrant_client.search.return_value = results_list

        # Request only top 3
        results = service.search(embedding=embedding, top_k=3)

        # Should only return 3 results even though 10 available
        assert len(results) <= 3

    def test_health_check(self, mock_qdrant_client) -> None:
        """Test health check method."""
        service = RetrievalService()
        service.client = mock_qdrant_client

        # Mock get_collection to return collection info
        collection_mock = MagicMock()
        collection_mock.points_count = 500
        mock_qdrant_client.get_collection.return_value = collection_mock

        is_healthy = service.health_check()

        assert is_healthy is True

    def test_health_check_no_client(self) -> None:
        """Test health check when client not initialized."""
        service = RetrievalService()
        service.client = None

        is_healthy = service.health_check()

        # Should handle gracefully (return False or handle exception)
        assert is_healthy is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
