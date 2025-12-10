"""Tests for confidence scoring service."""

import pytest
from src.services.confidence_service import ConfidenceService


class TestConfidenceScoring:
    """Test suite for ConfidenceService confidence calculation."""

    def test_high_confidence_with_high_similarity(self) -> None:
        """Test high confidence with high similarity sources."""
        service = ConfidenceService()

        sources = [
            {
                "chapter": "Chapter 2",
                "module": "Module 2.1",
                "section": "ros-2",
                "similarity": 0.95,
                "excerpt": "ROS 2 is a middleware...",
            }
        ]
        answer = "ROS 2 is a middleware for robotics communication."
        question = "What is ROS 2?"

        confidence = service.calculate_confidence(sources, answer, question)

        # High similarity should result in high confidence
        assert confidence == "high"

    def test_medium_confidence_with_medium_similarity(self) -> None:
        """Test medium confidence with medium similarity sources."""
        service = ConfidenceService()

        sources = [
            {
                "chapter": "Chapter 2",
                "module": "Module 2.1",
                "section": "ros-2",
                "similarity": 0.65,  # Medium similarity
                "excerpt": "ROS 2 information...",
            }
        ]
        answer = "Some information about ROS 2"
        question = "What is ROS 2?"

        confidence = service.calculate_confidence(sources, answer, question)

        # Medium similarity should result in medium or low confidence
        assert confidence in ["medium", "low"]

    def test_low_confidence_with_low_similarity(self) -> None:
        """Test low confidence with low similarity sources."""
        service = ConfidenceService()

        sources = [
            {
                "chapter": "Chapter 1",
                "module": "Module 1.1",
                "section": "intro",
                "similarity": 0.40,  # Low similarity
                "excerpt": "General information...",
            }
        ]
        answer = "Generic answer"
        question = "What is ROS 2?"

        confidence = service.calculate_confidence(sources, answer, question)

        # Low similarity should result in low confidence
        assert confidence == "low"

    def test_confidence_with_multiple_sources(self) -> None:
        """Test confidence calculation with multiple sources."""
        service = ConfidenceService()

        sources = [
            {
                "chapter": "Chapter 2",
                "module": "Module 2.1",
                "section": "ros-2-topics",
                "similarity": 0.92,
                "excerpt": "ROS 2 uses publish-subscribe...",
            },
            {
                "chapter": "Chapter 2",
                "module": "Module 2.2",
                "section": "ros-2-services",
                "similarity": 0.88,
                "excerpt": "ROS 2 services for request-reply...",
            },
            {
                "chapter": "Chapter 2",
                "module": "Module 2.3",
                "section": "ros-2-actions",
                "similarity": 0.85,
                "excerpt": "ROS 2 actions for long-running tasks...",
            },
        ]
        answer = "ROS 2 provides multiple communication patterns including topics, services, and actions."
        question = "How does ROS 2 communication work?"

        confidence = service.calculate_confidence(sources, answer, question)

        # Multiple high-similarity sources should give high confidence
        assert confidence == "high"

    def test_confidence_with_single_low_similarity_source(self) -> None:
        """Test confidence with only one low-similarity source."""
        service = ConfidenceService()

        sources = [
            {
                "chapter": "Chapter 3",
                "module": "Module 3.1",
                "section": "gazebo",
                "similarity": 0.50,  # Below typical threshold
                "excerpt": "Gazebo is a simulator...",
            }
        ]
        answer = "Gazebo information"
        question = "Tell me about ROS 2"

        confidence = service.calculate_confidence(sources, answer, question)

        # Should be low confidence due to low similarity
        assert confidence == "low"

    def test_confidence_returns_valid_level(self) -> None:
        """Test that confidence always returns valid level."""
        service = ConfidenceService()

        # Test with various sources
        test_cases = [
            ([{"similarity": 0.99, "chapter": "C", "module": "M", "section": "S", "excerpt": "E"}], "ROS 2"),
            ([{"similarity": 0.70, "chapter": "C", "module": "M", "section": "S", "excerpt": "E"}], "Mid"),
            ([{"similarity": 0.30, "chapter": "C", "module": "M", "section": "S", "excerpt": "E"}], "Low"),
        ]

        for sources, answer in test_cases:
            confidence = service.calculate_confidence(sources, answer, "question")
            assert confidence in ["high", "medium", "low"]

    def test_confidence_with_no_sources(self) -> None:
        """Test confidence with no sources."""
        service = ConfidenceService()

        sources = []
        answer = "Some answer"
        question = "Question"

        confidence = service.calculate_confidence(sources, answer, question)

        # No sources should always be low confidence
        assert confidence == "low"

    def test_confidence_with_matching_terms(self) -> None:
        """Test confidence calculation with matching terms in answer."""
        service = ConfidenceService()

        sources = [
            {
                "chapter": "Chapter 2",
                "module": "Module 2.1",
                "section": "ros-2-topics",
                "similarity": 0.70,
                "excerpt": "ROS 2 topics are publish-subscribe communication pattern",
            }
        ]
        # Answer contains matching terms from excerpt
        answer = "ROS 2 topics use the publish-subscribe communication pattern for decoupled communication."
        question = "What are ROS 2 topics?"

        confidence = service.calculate_confidence(sources, answer, question)

        # Should boost confidence due to term matching
        assert confidence in ["medium", "high"]

    def test_confidence_weights_similarity_most(self) -> None:
        """Test that similarity has highest weight in confidence calculation."""
        service = ConfidenceService()

        # High similarity should dominate
        high_sim_sources = [
            {
                "chapter": "Ch",
                "module": "M",
                "section": "S",
                "similarity": 0.95,
                "excerpt": "Content",
            }
        ]
        high_sim_confidence = service.calculate_confidence(
            high_sim_sources, "short answer", "question"
        )

        # Low similarity should dominate
        low_sim_sources = [
            {
                "chapter": "Ch",
                "module": "M",
                "section": "S",
                "similarity": 0.30,
                "excerpt": "Content",
            }
        ]
        low_sim_confidence = service.calculate_confidence(
            low_sim_sources, "detailed answer with many words", "question"
        )

        # High similarity source should result in higher or equal confidence
        confidence_levels = {"high": 3, "medium": 2, "low": 1}
        assert (
            confidence_levels[high_sim_confidence]
            >= confidence_levels[low_sim_confidence]
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
