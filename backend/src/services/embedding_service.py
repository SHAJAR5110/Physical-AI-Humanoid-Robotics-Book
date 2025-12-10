"""Embedding service using sentence-transformers for semantic search."""

import logging
from typing import Optional

import numpy as np
from sentence_transformers import SentenceTransformer

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Service for generating text embeddings using sentence-transformers."""

    def __init__(self, model_name: str = "all-minilm-l6-v2") -> None:
        """
        Initialize embedding service with specified model.

        Args:
            model_name: HuggingFace model name (default: all-minilm-l6-v2)
                       - 384 dimensions
                       - 22M parameters
                       - Fast on CPU
        """
        self.model_name = model_name
        self.model: Optional[SentenceTransformer] = None
        self.dimension = 384

    def load(self) -> None:
        """Load the embedding model. Called at startup."""
        try:
            logger.info(f"Loading embedding model: {self.model_name}")
            self.model = SentenceTransformer(self.model_name)
            logger.info(
                f"Model loaded successfully. Dimensions: {self.model.get_sentence_embedding_dimension()}"
            )
        except Exception as e:
            logger.error(f"Failed to load embedding model: {e}")
            raise

    def embed_text(self, text: str) -> list[float]:
        """
        Embed text using the loaded model.

        Args:
            text: Text to embed (will be stripped)

        Returns:
            384-dimensional embedding vector

        Raises:
            ValueError: If text is empty
            RuntimeError: If model not loaded
        """
        if not self.model:
            raise RuntimeError("Embedding model not loaded. Call load() first.")

        # Validate input
        text = text.strip()
        if not text:
            raise ValueError("Cannot embed empty text")

        if len(text) > 100000:
            logger.warning(f"Text length {len(text)} exceeds recommended max")
            text = text[:100000]

        try:
            # Generate embedding
            embedding = self.model.encode(
                text,
                convert_to_numpy=True,
                show_progress_bar=False,
            )

            # Convert to list[float]
            if isinstance(embedding, np.ndarray):
                return embedding.tolist()
            return list(embedding)

        except Exception as e:
            logger.error(f"Error embedding text: {e}")
            raise

    def embed_texts_batch(self, texts: list[str]) -> list[list[float]]:
        """
        Embed multiple texts efficiently in batch.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors

        Raises:
            ValueError: If texts list is empty
            RuntimeError: If model not loaded
        """
        if not self.model:
            raise RuntimeError("Embedding model not loaded. Call load() first.")

        if not texts:
            raise ValueError("Cannot embed empty texts list")

        try:
            # Clean and validate texts
            clean_texts = [t.strip() for t in texts if t.strip()]

            if not clean_texts:
                raise ValueError("All texts were empty")

            # Generate embeddings in batch
            embeddings = self.model.encode(
                clean_texts,
                convert_to_numpy=True,
                show_progress_bar=False,
                batch_size=32,
            )

            # Convert to list of lists
            if isinstance(embeddings, np.ndarray):
                return embeddings.tolist()
            return [list(e) for e in embeddings]

        except Exception as e:
            logger.error(f"Error batch embedding texts: {e}")
            raise

    def get_model_info(self) -> dict:
        """
        Get information about the loaded model.

        Returns:
            Dictionary with model metadata
        """
        if not self.model:
            return {
                "model": self.model_name,
                "loaded": False,
                "dimensions": self.dimension,
            }

        return {
            "model": self.model_name,
            "loaded": True,
            "dimensions": self.model.get_sentence_embedding_dimension(),
            "device": str(self.model.device),
        }


# Singleton instance (lazy-loaded at startup)
_embedding_service: Optional[EmbeddingService] = None


def get_embedding_service() -> EmbeddingService:
    """Get or initialize the embedding service."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
        _embedding_service.load()
    return _embedding_service
