"""Utility for generating URL-safe slugs from text."""

import re
import logging

logger = logging.getLogger(__name__)


class SlugGenerator:
    """Generates URL-safe slugs from chapter/module/section names."""

    @staticmethod
    def slugify(text: str) -> str:
        """
        Convert text to URL-safe slug.

        Args:
            text: Text to slugify

        Returns:
            URL-safe slug
        """
        if not text:
            return "unknown"

        # Convert to lowercase
        slug = text.lower()

        # Replace spaces with hyphens
        slug = re.sub(r'\s+', '-', slug)

        # Remove special characters, keep only alphanumeric and hyphens
        slug = re.sub(r'[^a-z0-9\-]', '', slug)

        # Remove consecutive hyphens
        slug = re.sub(r'-+', '-', slug)

        # Remove leading/trailing hyphens
        slug = slug.strip('-')

        # Ensure not empty
        if not slug:
            slug = "unknown"

        logger.debug(f"Slugified '{text}' to '{slug}'")
        return slug

    @staticmethod
    def generate_anchor(chapter: str, module: str, section: str) -> str:
        """
        Generate full anchor path for documentation link.

        Args:
            chapter: Chapter name
            module: Module name
            section: Section name

        Returns:
            Full anchor path (e.g., "ros-2-topics")
        """
        # For now, use section as primary anchor
        # In future could combine with chapter/module
        return SlugGenerator.slugify(section)
