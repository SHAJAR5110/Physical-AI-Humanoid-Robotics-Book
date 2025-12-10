"""Response formatting utilities."""

import logging
from typing import Any

from src.models import ChatResponse
from src.utils.slug_generator import SlugGenerator

logger = logging.getLogger(__name__)


class ResponseFormatter:
    """Formats responses for API endpoints."""

    @staticmethod
    def generate_source_link(
        chapter: str,
        module: str,
        section: str,
    ) -> str:
        """
        Generate a clickable documentation link for a source.

        Links use Docusaurus anchor format: /docs/chapter-page#section-anchor

        Args:
            chapter: Chapter name
            module: Module name
            section: Section anchor (already slugified)

        Returns:
            Full URL path to the source (e.g., "/docs/chapter-2#ros-2-topics")
        """
        # Generate chapter slug from chapter name
        chapter_slug = SlugGenerator.slugify(chapter)

        # Section is already provided as slugified anchor
        section_slug = section if section else "unknown"

        # Format as /docs/chapter-slug#section-slug
        link = f"/docs/{chapter_slug}#{section_slug}"

        logger.debug(f"Generated source link: {link}")
        return link

    @staticmethod
    def format_chat_response(
        answer: str,
        sources: list[dict],
        confidence: str,
        processing_time_ms: int,
    ) -> ChatResponse:
        """
        Format chat response with validation and source links.

        Args:
            answer: Generated answer
            sources: List of source references
            confidence: Confidence level
            processing_time_ms: Processing time

        Returns:
            Validated ChatResponse with source links
        """
        # Convert sources to SourceRef models
        from src.models import SourceRef

        source_refs = [
            SourceRef(
                chapter=s.get("chapter", "Unknown"),
                module=s.get("module", "Unknown"),
                section=s.get("section", "unknown"),
                excerpt=s.get("excerpt", ""),
                similarity=s.get("similarity"),
            )
            for s in sources
        ]

        response = ChatResponse(
            answer=answer,
            sources=source_refs,
            confidence=confidence,
            processing_time_ms=processing_time_ms,
        )

        logger.debug(f"Formatted response: {confidence} confidence, {len(source_refs)} sources")
        return response

    @staticmethod
    def get_source_links(sources: list[dict]) -> list[str]:
        """
        Generate documentation links for all sources.

        Args:
            sources: List of source references with chapter/module/section

        Returns:
            List of clickable links to documentation
        """
        links = []
        for source in sources:
            link = ResponseFormatter.generate_source_link(
                chapter=source.get("chapter", "Unknown"),
                module=source.get("module", "Unknown"),
                section=source.get("section", "unknown"),
            )
            links.append(link)

        logger.debug(f"Generated {len(links)} source links")
        return links

    @staticmethod
    def format_error_response(error: str, details: str) -> dict[str, Any]:
        """Format error response."""
        from src.models import ErrorResponse

        return ErrorResponse(error=error, details=details).model_dump()

    @staticmethod
    def format_health_response(
        status: str,
        app: str,
        version: str,
        environment: str,
    ) -> dict[str, Any]:
        """Format health check response."""
        from src.models import HealthResponse

        return HealthResponse(
            status=status,
            app=app,
            version=version,
            environment=environment,
        ).model_dump()
