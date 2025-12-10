"""Retrieval service for semantic search in Qdrant vector database."""

import logging
from typing import Optional

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

from src.config import settings

logger = logging.getLogger(__name__)


class RetrievalError(Exception):
    """Raised when retrieval fails or no relevant content found."""

    pass


class RetrievalService:
    """Service for semantic search using Qdrant vector database."""

    def __init__(
        self,
        qdrant_url: str,
        api_key: str,
        collection_name: str = "book_passages",
        embedding_dimension: int = 384,
    ) -> None:
        """
        Initialize retrieval service with Qdrant connection.

        Args:
            qdrant_url: Qdrant server URL
            api_key: Qdrant API key
            collection_name: Name of the collection
            embedding_dimension: Dimension of embeddings (384 for all-minilm-l6-v2)
        """
        self.qdrant_url = qdrant_url
        self.api_key = api_key
        self.collection_name = collection_name
        self.embedding_dimension = embedding_dimension
        self.client: Optional[QdrantClient] = None

    def connect(self) -> None:
        """Connect to Qdrant server."""
        try:
            logger.info(f"Connecting to Qdrant at {self.qdrant_url}")
            self.client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.api_key,
                prefer_grpc=False,  # Use HTTP for simplicity
            )
            logger.info("Connected to Qdrant successfully")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

    def ensure_collection(self) -> None:
        """Ensure collection exists; create if not."""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")

        try:
            # Try to get collection info
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' exists")
        except Exception:
            # Collection doesn't exist, create it
            logger.info(f"Creating collection '{self.collection_name}'")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.embedding_dimension,
                    distance=Distance.COSINE,
                ),
            )
            logger.info(f"Collection '{self.collection_name}' created")

    def search(
        self,
        embedding: list[float],
        top_k: int = 5,
        similarity_threshold: float = 0.85,
        selected_text: Optional[str] = None,
    ) -> list[dict]:
        """
        Search for similar passages in Qdrant.

        Args:
            embedding: Query embedding vector (384 dimensions)
            top_k: Number of top results to return
            similarity_threshold: Minimum cosine similarity (0-1)
            selected_text: Optional selected text to prioritize in retrieval

        Returns:
            List of source references with metadata

        Raises:
            RetrievalError: If no relevant content found or search fails
        """
        if not self.client:
            raise RuntimeError("Qdrant client not connected")

        try:
            # Search for similar passages
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=embedding,
                limit=top_k * 2,  # Get more than needed, filter by threshold
                with_payload=True,
            ).points

            # Filter by similarity threshold
            filtered_results = []
            for result in results:
                if result.score >= similarity_threshold:
                    filtered_results.append(
                        {
                            "chapter": result.payload.get("chapter", "Unknown"),
                            "module": result.payload.get("module", "Unknown"),
                            "section": result.payload.get("section", "unknown"),
                            "excerpt": result.payload.get("text", ""),
                            "similarity": result.score,
                        }
                    )

            if not filtered_results:
                raise RetrievalError(
                    "I couldn't find information on this topic. "
                    "Try rephrasing your question or browse the chapters."
                )

            # If selected_text is provided, prioritize passages matching it
            if selected_text:
                selected_text_lower = selected_text.lower()
                # Boost similarity for passages containing selected text
                for result in filtered_results:
                    excerpt_lower = result["excerpt"].lower()
                    if selected_text_lower in excerpt_lower:
                        # Boost similarity score for matching passages
                        result["similarity"] += 0.1

                # Re-sort by boosted similarity
                filtered_results.sort(key=lambda x: x["similarity"], reverse=True)
                logger.info(
                    f"Prioritized results based on selected text: {selected_text[:50]}..."
                )

            logger.info(f"Retrieved {len(filtered_results)} passages for query")
            return filtered_results[:top_k]  # Return top_k after boosting

        except RetrievalError:
            raise
        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}")
            raise RetrievalError(
                "Chat service is temporarily unavailable. Please try again in a moment."
            )

    def health_check(self) -> bool:
        """
        Verify Qdrant connection is healthy.

        Returns:
            True if healthy, False otherwise
        """
        if not self.client:
            return False

        try:
            self.client.get_collections()
            return True
        except Exception as e:
            logger.warning(f"Qdrant health check failed: {e}")
            return False

    def get_collection_stats(self) -> dict:
        """Get statistics about the collection."""
        if not self.client:
            raise RuntimeError("Qdrant client not connected")

        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "points_count": collection_info.points_count,
                "vectors_count": collection_info.vectors_count,
            }
        except Exception as e:
            logger.error(f"Error getting collection stats: {e}")
            return {
                "name": self.collection_name,
                "error": str(e),
            }


# Singleton instance (lazy-loaded at startup)
_retrieval_service: Optional[RetrievalService] = None


def get_retrieval_service() -> RetrievalService:
    """Get or initialize the retrieval service."""
    global _retrieval_service
    if _retrieval_service is None:
        _retrieval_service = RetrievalService(
            qdrant_url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection_name,
        )
        _retrieval_service.connect()
        _retrieval_service.ensure_collection()
    return _retrieval_service
