"""
Embedding service for Physical AI Book backend.

Handles chapter embedding generation and vector storage in Qdrant.
"""

import logging
import uuid
from typing import List, Optional
from anthropic import Anthropic
from sqlalchemy.orm import Session

from config import settings
from db import get_qdrant
from models.chapter import ChapterEmbedding
from db.connection import SessionLocal

logger = logging.getLogger(__name__)

# Anthropic client
client = Anthropic()


class EmbeddingService:
    """
    Service for embedding chapter content and storing vectors in Qdrant.

    Handles:
    - Chapter embedding generation via Claude API
    - Vector storage in Qdrant
    - Semantic search queries
    """

    @staticmethod
    def embed_text(text: str) -> Optional[List[float]]:
        """
        Generate embedding for text using Claude embeddings API.

        Args:
            text: Text to embed

        Returns:
            Vector embedding or None if failed
        """
        try:
            response = client.messages.create(
                model=settings.anthropic_embedding_model,
                max_tokens=1024,
                messages=[
                    {
                        "role": "user",
                        "content": f"Generate an embedding vector for this text:\n\n{text}",
                    }
                ],
            )

            # Extract vector from response (placeholder - actual API response varies)
            # In production, use: from anthropic import Embeddings
            embedding = [0.1] * 1024  # Placeholder: 1024-dimensional vector

            logger.debug(f"Generated embedding for {len(text)} characters")
            return embedding
        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            return None

    @staticmethod
    def split_chapter(content: str, section_size: int = 500) -> List[Dict[str, str]]:
        """
        Split chapter content into sections for embedding.

        Args:
            content: Full chapter markdown content
            section_size: Target words per section

        Returns:
            List of sections with title and content
        """
        sections = []
        lines = content.split("\n")
        current_section = ""
        current_title = "Introduction"

        for line in lines:
            # Detect section headers (markdown # ## ###)
            if line.startswith("#"):
                if current_section.strip():
                    sections.append({
                        "title": current_title,
                        "content": current_section.strip(),
                    })
                current_section = ""
                current_title = line.lstrip("#").strip()
            else:
                current_section += line + "\n"

                # Split by word count if section too large
                word_count = len(current_section.split())
                if word_count > section_size:
                    sections.append({
                        "title": current_title,
                        "content": current_section.strip(),
                    })
                    current_section = ""

        # Add final section
        if current_section.strip():
            sections.append({
                "title": current_title,
                "content": current_section.strip(),
            })

        logger.info(f"Split chapter into {len(sections)} sections")
        return sections

    @staticmethod
    def embed_chapter(chapter_id: int, content: str, db: Optional[Session] = None) -> bool:
        """
        Embed and store chapter in Qdrant.

        Args:
            chapter_id: Chapter database ID
            content: Chapter markdown content
            db: Database session

        Returns:
            True if successful
        """
        if db is None:
            db = SessionLocal()

        try:
            qdrant = get_qdrant()

            # Ensure collection exists
            qdrant.create_collection(
                collection_name=settings.qdrant_collection_name,
                vector_size=1024,
            )

            # Split chapter into sections
            sections = EmbeddingService.split_chapter(content)

            # Embed each section
            for idx, section in enumerate(sections, 1):
                # Generate embedding (placeholder)
                embedding = [0.1] * 1024  # Replace with actual embedding

                # Create point ID
                point_id = str(uuid.uuid4())

                # Add to Qdrant
                payload = {
                    "chapter_id": chapter_id,
                    "section_title": section["title"],
                    "section_index": idx,
                    "excerpt": section["content"][:200],  # Store excerpt
                }

                qdrant.add_point(
                    collection_name=settings.qdrant_collection_name,
                    point_id=point_id,
                    vector=embedding,
                    payload=payload,
                )

                # Store metadata in Postgres
                embedding_record = ChapterEmbedding(
                    id=uuid.uuid4(),
                    chapter_id=chapter_id,
                    section_title=section["title"],
                    section_index=idx,
                    content_excerpt=section["content"],
                    qdrant_point_id=point_id,
                )
                db.add(embedding_record)

            db.commit()
            logger.info(f"Embedded chapter {chapter_id} into {len(sections)} sections")
            return True

        except Exception as e:
            db.rollback()
            logger.error(f"Chapter embedding failed: {e}")
            return False

    @staticmethod
    def search_embeddings(
        query: str,
        limit: int = 5,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chapter sections.

        Args:
            query: Search query text
            limit: Maximum results

        Returns:
            List of matching sections
        """
        try:
            # Generate query embedding (placeholder)
            query_embedding = [0.1] * 1024  # Replace with actual embedding

            qdrant = get_qdrant()
            results = qdrant.search(
                collection_name=settings.qdrant_collection_name,
                query_vector=query_embedding,
                limit=limit,
            )

            logger.info(f"Search returned {len(results)} results for query: {query}")
            return results

        except Exception as e:
            logger.error(f"Search failed: {e}")
            return []


# Create singleton instance
_embedding_service = EmbeddingService()


def get_embedding_service() -> EmbeddingService:
    """Get embedding service instance."""
    return _embedding_service
