"""
Qdrant vector database client for Physical AI Book backend.

Handles semantic search, embedding storage, and vector operations.
"""

import logging
from typing import Optional, List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import uuid

from config import settings

logger = logging.getLogger(__name__)


class QdrantManager:
    """
    Manager for Qdrant vector database operations.

    Handles:
    - Connection management
    - Collection creation/deletion
    - Vector storage and retrieval
    - Semantic search
    """

    def __init__(self):
        """Initialize Qdrant client."""
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key if settings.qdrant_api_key else None,
            )
            logger.info(f"Connected to Qdrant at {settings.qdrant_url}")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

    def health_check(self) -> bool:
        """Check Qdrant server health."""
        try:
            self.client.get_collections()
            logger.info("Qdrant health check passed")
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False

    def create_collection(
        self,
        collection_name: str = settings.qdrant_collection_name,
        vector_size: int = 1024,
        distance: Distance = Distance.COSINE,
    ) -> bool:
        """
        Create a vector collection in Qdrant.

        Args:
            collection_name: Name of the collection
            vector_size: Dimension of vectors (1024 for Claude embeddings)
            distance: Distance metric (COSINE, EUCLID, DOT)

        Returns:
            True if creation successful, False otherwise
        """
        try:
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=vector_size, distance=distance),
            )
            logger.info(f"Created Qdrant collection: {collection_name}")
            return True
        except Exception as e:
            if "already exists" in str(e):
                logger.warning(f"Collection {collection_name} already exists")
                return True
            logger.error(f"Failed to create collection: {e}")
            return False

    def add_point(
        self,
        collection_name: str,
        point_id: str,
        vector: List[float],
        payload: Dict[str, Any],
    ) -> bool:
        """
        Add a vector point to collection.

        Args:
            collection_name: Name of the collection
            point_id: Unique point identifier
            vector: Vector embedding (list of floats)
            payload: Metadata associated with the vector

        Returns:
            True if successful
        """
        try:
            point = PointStruct(id=point_id, vector=vector, payload=payload)
            self.client.upsert(
                collection_name=collection_name,
                points=[point],
            )
            logger.debug(f"Added point {point_id} to {collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to add point: {e}")
            return False

    def search(
        self,
        collection_name: str,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: float = 0.0,
    ) -> List[Dict[str, Any]]:
        """
        Semantic search in Qdrant collection.

        Args:
            collection_name: Name of the collection
            query_vector: Query embedding vector
            limit: Maximum number of results
            score_threshold: Minimum similarity score

        Returns:
            List of matching points with metadata
        """
        try:
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
            )

            search_results = []
            for result in results:
                search_results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                })

            logger.debug(f"Search returned {len(search_results)} results")
            return search_results
        except Exception as e:
            logger.error(f"Search failed: {e}")
            return []

    def delete_point(self, collection_name: str, point_id: str) -> bool:
        """
        Delete a point from collection.

        Args:
            collection_name: Name of the collection
            point_id: Point identifier to delete

        Returns:
            True if successful
        """
        try:
            self.client.delete(
                collection_name=collection_name,
                points_selector=[point_id],
            )
            logger.debug(f"Deleted point {point_id} from {collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to delete point: {e}")
            return False

    def delete_collection(self, collection_name: str) -> bool:
        """
        Delete entire collection.

        Args:
            collection_name: Name of the collection to delete

        Returns:
            True if successful
        """
        try:
            self.client.delete_collection(collection_name=collection_name)
            logger.info(f"Deleted collection: {collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to delete collection: {e}")
            return False

    def get_collection_info(self, collection_name: str) -> Optional[Dict[str, Any]]:
        """
        Get collection information.

        Args:
            collection_name: Name of the collection

        Returns:
            Collection info or None if not found
        """
        try:
            info = self.client.get_collection(collection_name=collection_name)
            return {
                "name": collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "config": str(info.config),
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            return None


# Global Qdrant manager instance
_qdrant_manager: Optional[QdrantManager] = None


def get_qdrant() -> QdrantManager:
    """Get or initialize Qdrant manager singleton."""
    global _qdrant_manager
    if _qdrant_manager is None:
        _qdrant_manager = QdrantManager()
    return _qdrant_manager


def close_qdrant() -> None:
    """Close Qdrant connection."""
    global _qdrant_manager
    if _qdrant_manager is not None:
        _qdrant_manager = None
        logger.info("Closed Qdrant connection")
