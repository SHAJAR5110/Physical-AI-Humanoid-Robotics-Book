"""Tests for source attribution and slug generation."""

import pytest
from unittest.mock import MagicMock, patch

from src.utils.slug_generator import SlugGenerator
from src.utils.response_formatter import ResponseFormatter
from src.models import SourceRef


class TestSlugGenerator:
    """Test suite for slug generation utilities."""

    def test_slugify_basic(self) -> None:
        """Test basic slugification."""
        assert SlugGenerator.slugify("ROS 2 Topics") == "ros-2-topics"
        assert SlugGenerator.slugify("NVIDIA Isaac Platform") == "nvidia-isaac-platform"

    def test_slugify_special_characters(self) -> None:
        """Test slugification with special characters."""
        assert SlugGenerator.slugify("What is publish-subscribe?") == "what-is-publish-subscribe"
        assert SlugGenerator.slugify("C++/CUDA Performance") == "ccuda-performance"

    def test_slugify_multiple_spaces(self) -> None:
        """Test slugification with multiple consecutive spaces."""
        assert SlugGenerator.slugify("ROS   2   Topics") == "ros-2-topics"

    def test_slugify_case_insensitive(self) -> None:
        """Test that slugification is case-insensitive."""
        assert SlugGenerator.slugify("ROS 2") == "ros-2"
        assert SlugGenerator.slugify("ros 2") == "ros-2"
        assert SlugGenerator.slugify("RoS 2") == "ros-2"

    def test_slugify_leading_trailing_hyphens(self) -> None:
        """Test that leading/trailing hyphens are removed."""
        assert SlugGenerator.slugify("-ROS 2-") == "ros-2"
        assert SlugGenerator.slugify("---ROS 2---") == "ros-2"

    def test_slugify_consecutive_hyphens(self) -> None:
        """Test that consecutive hyphens are collapsed."""
        assert SlugGenerator.slugify("ROS--2--Topics") == "ros-2-topics"

    def test_slugify_numbers(self) -> None:
        """Test slugification with numbers."""
        assert SlugGenerator.slugify("Chapter 2") == "chapter-2"
        assert SlugGenerator.slugify("Module 2.1") == "module-21"

    def test_slugify_empty_string(self) -> None:
        """Test slugification of empty string."""
        result = SlugGenerator.slugify("")
        assert result == "unknown"

    def test_slugify_only_special_chars(self) -> None:
        """Test slugification of text with only special characters."""
        result = SlugGenerator.slugify("!@#$%^&*()")
        assert result == "unknown"

    def test_generate_anchor_with_section(self) -> None:
        """Test anchor generation uses section when available."""
        anchor = SlugGenerator.generate_anchor(
            chapter="Chapter 2: ROS 2 Fundamentals",
            module="Module 2.1: Nodes and Topics",
            section="ROS 2 Topics"
        )
        assert anchor == "ros-2-topics"

    def test_generate_anchor_without_section(self) -> None:
        """Test anchor generation falls back to module."""
        anchor = SlugGenerator.generate_anchor(
            chapter="Chapter 2: ROS 2 Fundamentals",
            module="Module 2.1: Nodes and Topics",
            section=""
        )
        assert anchor == "module-21-nodes-and-topics"

    def test_generate_anchor_only_chapter(self) -> None:
        """Test anchor generation falls back to chapter."""
        anchor = SlugGenerator.generate_anchor(
            chapter="Chapter 2: ROS 2 Fundamentals",
            module="",
            section=""
        )
        assert anchor == "chapter-2-ros-2-fundamentals"


class TestResponseFormatter:
    """Test suite for response formatting with source links."""

    def test_generate_source_link_basic(self) -> None:
        """Test basic source link generation."""
        link = ResponseFormatter.generate_source_link(
            chapter="Chapter 2: ROS 2 Fundamentals",
            module="Module 2.1: Nodes and Topics",
            section="ros-2-topics"
        )
        assert link == "/docs/chapter-2-ros-2-fundamentals#ros-2-topics"

    def test_generate_source_link_with_special_chars(self) -> None:
        """Test source link generation with special characters."""
        link = ResponseFormatter.generate_source_link(
            chapter="Chapter 4: NVIDIA Isaac Platform",
            module="Module 4.1: Basics",
            section="isaac-intro"
        )
        assert link.startswith("/docs/")
        assert "#isaac-intro" in link

    def test_generate_source_link_empty_section(self) -> None:
        """Test source link generation with empty section."""
        link = ResponseFormatter.generate_source_link(
            chapter="Chapter 1",
            module="Module 1.1",
            section=""
        )
        assert link == "/docs/chapter-1#unknown"

    def test_get_source_links_multiple(self) -> None:
        """Test generating links for multiple sources."""
        sources = [
            {
                "chapter": "Chapter 2: ROS 2 Fundamentals",
                "module": "Module 2.1: Nodes",
                "section": "ros-2-nodes",
                "excerpt": "ROS 2 nodes are processes...",
                "similarity": 0.92
            },
            {
                "chapter": "Chapter 2: ROS 2 Fundamentals",
                "module": "Module 2.2: Topics",
                "section": "ros-2-topics",
                "excerpt": "Topics enable pub/sub...",
                "similarity": 0.88
            }
        ]

        links = ResponseFormatter.get_source_links(sources)

        assert len(links) == 2
        assert all("/docs/" in link for link in links)
        assert all("#" in link for link in links)

    def test_get_source_links_empty(self) -> None:
        """Test generating links for empty source list."""
        links = ResponseFormatter.get_source_links([])
        assert links == []

    def test_format_chat_response_with_sources(self) -> None:
        """Test formatting chat response with sources."""
        sources = [
            {
                "chapter": "Chapter 2",
                "module": "Module 2.1",
                "section": "ros-2-topics",
                "excerpt": "ROS 2 is middleware...",
                "similarity": 0.92
            }
        ]

        response = ResponseFormatter.format_chat_response(
            answer="ROS 2 is a middleware platform for robotics.",
            sources=sources,
            confidence="high",
            processing_time_ms=1850
        )

        assert response.answer == "ROS 2 is a middleware platform for robotics."
        assert len(response.sources) == 1
        assert response.sources[0].chapter == "Chapter 2"
        assert response.sources[0].similarity == 0.92
        assert response.confidence == "high"
        assert response.processing_time_ms == 1850

    def test_format_chat_response_multiple_sources(self) -> None:
        """Test formatting with multiple sources."""
        sources = [
            {
                "chapter": "Chapter 2",
                "module": "Module 2.1",
                "section": "topic1",
                "excerpt": "Excerpt 1",
                "similarity": 0.95
            },
            {
                "chapter": "Chapter 3",
                "module": "Module 3.1",
                "section": "topic2",
                "excerpt": "Excerpt 2",
                "similarity": 0.87
            },
            {
                "chapter": "Chapter 4",
                "module": "Module 4.1",
                "section": "topic3",
                "excerpt": "Excerpt 3",
                "similarity": 0.85
            }
        ]

        response = ResponseFormatter.format_chat_response(
            answer="Complex answer from multiple sources.",
            sources=sources,
            confidence="medium",
            processing_time_ms=2100
        )

        assert len(response.sources) == 3
        # Verify all sources have required fields
        for source in response.sources:
            assert source.chapter
            assert source.module
            assert source.section
            assert source.excerpt
            assert source.similarity is not None


class TestSourceRefModel:
    """Test suite for SourceRef Pydantic model."""

    def test_source_ref_creation(self) -> None:
        """Test creating a SourceRef model."""
        source = SourceRef(
            chapter="Chapter 2: ROS 2",
            module="Module 2.1",
            section="ros-2-nodes",
            excerpt="ROS 2 nodes are processes...",
            similarity=0.92
        )

        assert source.chapter == "Chapter 2: ROS 2"
        assert source.module == "Module 2.1"
        assert source.section == "ros-2-nodes"
        assert source.excerpt == "ROS 2 nodes are processes..."
        assert source.similarity == 0.92

    def test_source_ref_without_similarity(self) -> None:
        """Test SourceRef works without similarity score."""
        source = SourceRef(
            chapter="Chapter 2",
            module="Module 2.1",
            section="sec",
            excerpt="Text"
        )

        assert source.similarity is None

    def test_source_ref_json_serialization(self) -> None:
        """Test SourceRef serializes to JSON correctly."""
        source = SourceRef(
            chapter="Chapter 2",
            module="Module 2.1",
            section="ros-2",
            excerpt="Content",
            similarity=0.85
        )

        json_data = source.model_dump()

        assert json_data["chapter"] == "Chapter 2"
        assert json_data["section"] == "ros-2"
        assert json_data["similarity"] == 0.85

    def test_source_ref_similarity_validation(self) -> None:
        """Test SourceRef validates similarity range."""
        # Valid similarity (0.0-1.0)
        source = SourceRef(
            chapter="Ch", module="Mod", section="sec", excerpt="Text", similarity=0.5
        )
        assert source.similarity == 0.5

        # Out of range similarity should fail validation
        import pytest
        with pytest.raises(ValueError):
            SourceRef(
                chapter="Ch", module="Mod", section="sec", excerpt="Text", similarity=1.5
            )


class TestSourceAttributionIntegration:
    """Integration tests for source attribution flow."""

    def test_full_source_attribution_flow(self) -> None:
        """Test complete source attribution flow."""
        # Simulate retrieval results
        raw_sources = [
            {
                "chapter": "Chapter 2: ROS 2 Fundamentals",
                "module": "Module 2.1: Nodes and Topics",
                "section": "ros-2-topics",
                "text": "ROS 2 uses a publish-subscribe pattern...",
                "similarity": 0.92
            }
        ]

        # Format response
        response = ResponseFormatter.format_chat_response(
            answer="ROS 2 uses publish-subscribe for communication.",
            sources=raw_sources,
            confidence="high",
            processing_time_ms=1850
        )

        # Verify source link can be generated
        source = response.sources[0]
        link = ResponseFormatter.generate_source_link(
            chapter=source.chapter,
            module=source.module,
            section=source.section
        )

        # Verify link structure
        assert link.startswith("/docs/")
        assert "#" in link
        assert source.section in link

    def test_source_attribution_with_special_chapter_names(self) -> None:
        """Test source attribution with complex chapter names."""
        sources = [
            {
                "chapter": "Chapter 5: Advanced C++/CUDA Programming",
                "module": "Module 5.2: GPU Optimization",
                "section": "cuda-optimization",
                "text": "CUDA optimization techniques...",
                "similarity": 0.90
            }
        ]

        response = ResponseFormatter.format_chat_response(
            answer="CUDA optimization involves memory coalescing.",
            sources=sources,
            confidence="high",
            processing_time_ms=1950
        )

        # Verify link is generated correctly despite special chars
        source = response.sources[0]
        link = ResponseFormatter.generate_source_link(
            chapter=source.chapter,
            module=source.module,
            section=source.section
        )

        assert link  # Link should be generated
        assert "/docs/" in link
        assert "cuda-optimization" in link


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
