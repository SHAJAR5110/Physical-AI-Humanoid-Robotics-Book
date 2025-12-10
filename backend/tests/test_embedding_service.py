"""Tests for embedding service."""

import pytest
import numpy as np
from src.services.embedding_service import EmbeddingService


class TestEmbeddingService:
    """Test suite for EmbeddingService."""

    def test_initialization(self) -> None:
        """Test service initialization."""
        service = EmbeddingService()
        assert service.model_name == "all-minilm-l6-v2"
        assert service.dimension == 1536
        assert service.model is None

    def test_get_model_info_not_loaded(self) -> None:
        """Test model info before loading."""
        service = EmbeddingService()
        info = service.get_model_info()
        assert info["loaded"] is False
        assert info["dimensions"] == 1536
        assert info["model"] == "all-minilm-l6-v2"

    def test_embed_text_not_loaded(self) -> None:
        """Test embedding fails if model not loaded."""
        service = EmbeddingService()
        with pytest.raises(RuntimeError, match="not loaded"):
            service.embed_text("test text")

    def test_embed_text_empty(self, mock_embedding_service) -> None:
        """Test embedding empty text raises error."""
        service = EmbeddingService()
        service.model = mock_embedding_service

        with pytest.raises(ValueError, match="empty"):
            service.embed_text("")

    def test_embed_text_valid(self, mock_embedding_service) -> None:
        """Test successful text embedding."""
        service = EmbeddingService()
        service.model = mock_embedding_service

        # Mock the encode method to return a numpy array
        mock_embedding_service.encode.return_value = np.array([0.1] * 1536)

        result = service.embed_text("What is ROS 2?")

        assert isinstance(result, list)
        assert len(result) == 1536
        assert all(isinstance(x, (int, float)) for x in result)

    def test_embed_text_whitespace_stripping(self, mock_embedding_service) -> None:
        """Test text is stripped of whitespace."""
        service = EmbeddingService()
        service.model = mock_embedding_service
        mock_embedding_service.encode.return_value = np.array([0.1] * 1536)

        service.embed_text("  question text  ")
        mock_embedding_service.encode.assert_called()

    def test_embed_text_very_long(self, mock_embedding_service) -> None:
        """Test handling of very long text."""
        service = EmbeddingService()
        service.model = mock_embedding_service
        mock_embedding_service.encode.return_value = np.array([0.1] * 1536)

        long_text = "a" * 100001
        service.embed_text(long_text)
        mock_embedding_service.encode.assert_called()

    def test_embed_texts_batch_empty(self, mock_embedding_service) -> None:
        """Test batch embedding with empty list."""
        service = EmbeddingService()
        service.model = mock_embedding_service

        with pytest.raises(ValueError, match="empty"):
            service.embed_texts_batch([])

    def test_embed_texts_batch_all_empty(self, mock_embedding_service) -> None:
        """Test batch with all empty strings."""
        service = EmbeddingService()
        service.model = mock_embedding_service

        with pytest.raises(ValueError, match="empty"):
            service.embed_texts_batch(["", "  ", "\n"])

    def test_embed_texts_batch_valid(self, mock_embedding_service) -> None:
        """Test successful batch embedding."""
        service = EmbeddingService()
        service.model = mock_embedding_service

        # Mock batch encode
        mock_embedding_service.encode.return_value = np.array(
            [[0.1] * 1536, [0.2] * 1536, [0.3] * 1536]
        )

        texts = ["text 1", "text 2", "text 3"]
        results = service.embed_texts_batch(texts)

        assert len(results) == 3
        assert all(len(r) == 1536 for r in results)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
