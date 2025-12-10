"""Service for calculating confidence scores on generated answers."""

import logging
from typing import Literal

logger = logging.getLogger(__name__)


class ConfidenceService:
    """Service for calculating and classifying answer confidence levels."""

    def __init__(self) -> None:
        """Initialize confidence service."""
        # Weights for confidence calculation
        self.similarity_weight = 0.4  # Max cosine similarity
        self.source_weight = 0.3  # Number of relevant passages
        self.term_weight = 0.3  # Key term matching

    def calculate_confidence(
        self,
        sources: list[dict],
        answer: str,
        question: str,
    ) -> Literal["high", "medium", "low"]:
        """
        Calculate confidence level of generated answer.

        Args:
            sources: List of source references with similarity scores
            answer: Generated answer text
            question: Original question

        Returns:
            Confidence level: "high", "medium", or "low"
        """
        try:
            # Component 1: Max similarity from sources (40% weight)
            max_similarity = 0.0
            if sources:
                similarities = [s.get("similarity", 0.85) for s in sources]
                max_similarity = max(similarities)

            similarity_score = max_similarity * self.similarity_weight

            # Component 2: Number of relevant sources (30% weight)
            # Normalized to 0-1 scale (max 5 sources)
            num_sources = min(len(sources), 5)
            source_score = (num_sources / 5) * self.source_weight

            # Component 3: Key term matching (30% weight)
            # Check if important words from question appear in answer
            question_terms = set(
                word.lower() for word in question.split() if len(word) > 3
            )
            answer_lower = answer.lower()

            matching_terms = sum(
                1 for term in question_terms if term in answer_lower
            )

            if question_terms:
                term_score = (matching_terms / len(question_terms)) * self.term_weight
            else:
                term_score = self.term_weight

            # Total confidence score (0-1 scale)
            total_score = similarity_score + source_score + term_score

            # Classify into confidence levels
            confidence = self._classify_confidence(total_score)

            logger.debug(
                f"Confidence calculation: "
                f"similarity={similarity_score:.2f}, "
                f"sources={source_score:.2f}, "
                f"terms={term_score:.2f}, "
                f"total={total_score:.2f} → {confidence}"
            )

            return confidence

        except Exception as e:
            logger.error(f"Error calculating confidence: {e}")
            return "medium"  # Default to medium on error

    def _classify_confidence(self, score: float) -> Literal["high", "medium", "low"]:
        """
        Classify confidence score into level.

        Args:
            score: Confidence score (0-1)

        Returns:
            Confidence level
        """
        if score >= 0.8:
            return "high"
        elif score >= 0.5:
            return "medium"
        else:
            return "low"

    def get_confidence_details(self, confidence: Literal["high", "medium", "low"]) -> dict:
        """
        Get details about a confidence level.

        Args:
            confidence: Confidence level

        Returns:
            Dictionary with details and interpretation
        """
        details = {
            "high": {
                "level": "high",
                "emoji": "✅",
                "message": "High confidence answer based on relevant content",
                "score_range": "≥ 0.8",
            },
            "medium": {
                "level": "medium",
                "emoji": "⚠️",
                "message": "Medium confidence answer; may need verification",
                "score_range": "0.5 - 0.8",
            },
            "low": {
                "level": "low",
                "emoji": "❓",
                "message": "Low confidence answer; limited content match",
                "score_range": "< 0.5",
            },
        }
        return details.get(confidence, details["medium"])


# Singleton instance
_confidence_service: ConfidenceService | None = None


def get_confidence_service() -> ConfidenceService:
    """Get or initialize the confidence service."""
    global _confidence_service
    if _confidence_service is None:
        _confidence_service = ConfidenceService()
    return _confidence_service
