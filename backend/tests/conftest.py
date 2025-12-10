"""Pytest configuration and fixtures."""

import pytest
from unittest.mock import MagicMock, patch


@pytest.fixture
def mock_embedding_service():
    """Mock embedding service for testing."""
    with patch("src.services.embedding_service.SentenceTransformer") as mock:
        service_mock = MagicMock()
        service_mock.encode.return_value = [0.1] * 1536  # Mock 1536-dim vector
        service_mock.get_sentence_embedding_dimension.return_value = 1536
        mock.return_value = service_mock
        yield service_mock


@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client for testing."""
    with patch("src.services.retrieval_service.QdrantClient") as mock:
        client_mock = MagicMock()

        # Mock search results
        search_result = MagicMock()
        search_result.score = 0.92
        search_result.payload = {
            "chapter": "Chapter 2: ROS 2 Fundamentals",
            "module": "Module 2.1: Nodes and Topics",
            "section": "ros-2-topics",
            "text": "ROS 2 is a middleware for robotics...",
        }

        client_mock.search.return_value = [search_result]
        client_mock.get_collection.return_value = MagicMock(points_count=100)
        client_mock.get_collections.return_value = []

        mock.return_value = client_mock
        yield client_mock


@pytest.fixture
def mock_groq_client():
    """Mock Groq client for testing."""
    with patch("src.services.generation_service.Groq") as mock:
        client_mock = MagicMock()

        # Mock completion response
        message_mock = MagicMock()
        message_mock.content = "ROS 2 is a middleware that enables robot communication through topics and services."

        choice_mock = MagicMock()
        choice_mock.message = message_mock

        response_mock = MagicMock()
        response_mock.choices = [choice_mock]

        client_mock.chat.completions.create.return_value = response_mock
        client_mock.models.list.return_value = []

        mock.return_value = client_mock
        yield client_mock


@pytest.fixture
def sample_sources():
    """Sample source data for testing."""
    return [
        {
            "chapter": "Chapter 2: ROS 2 Fundamentals",
            "module": "Module 2.1: Nodes and Topics",
            "section": "ros-2-topics",
            "excerpt": "ROS 2 uses a publish-subscribe pattern...",
            "similarity": 0.92,
        }
    ]


@pytest.fixture
def sample_question():
    """Sample question for testing."""
    return "What is ROS 2?"


@pytest.fixture
def sample_answer():
    """Sample generated answer for testing."""
    return "ROS 2 is a middleware for robotics that provides tools and libraries for building robot applications."
