"""Generation service using Groq LLM for answer synthesis."""

import logging
from typing import Optional

from groq import Groq

from src.config import settings

logger = logging.getLogger(__name__)


class GenerationError(Exception):
    """Raised when generation fails."""

    pass


class GenerationService:
    """Service for generating answers using Groq LLM."""

    def __init__(self, api_key: str, model: str = "mixtral-8x7b-32768") -> None:
        """
        Initialize generation service with Groq API.

        Args:
            api_key: Groq API key
            model: Model to use (default: mixtral-8x7b-32768)
                  - Fast inference
                  - Free tier: 30,000 tokens/minute
        """
        self.api_key = api_key
        self.model = model
        self.client: Optional[Groq] = None

    def connect(self) -> None:
        """Initialize Groq client."""
        try:
            logger.info(f"Initializing Groq client with model: {self.model}")
            self.client = Groq(api_key=self.api_key)
            logger.info("Groq client initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Groq client: {e}")
            raise

    def generate_answer(
        self,
        question: str,
        context: str,
        max_tokens: int = 512,
        temperature: float = 0.3,
        selected_text: Optional[str] = None,
    ) -> str:
        """
        Generate answer using Groq LLM.

        Args:
            question: User's question
            context: Retrieved context passages (formatted)
            max_tokens: Maximum tokens in response
            temperature: Creativity (0.0 = deterministic, 1.0 = creative)
            selected_text: Optional selected text to acknowledge in answer

        Returns:
            Generated answer string

        Raises:
            GenerationError: If generation fails
        """
        if not self.client:
            raise RuntimeError("Groq client not initialized. Call connect() first.")

        system_prompt = (
            "You are a knowledgeable tutor for a Physical AI & Humanoid Robotics textbook. "
            "Answer student questions based on the provided context from the book. "
            "Be comprehensive but concise (2-4 sentences). "
            "If the exact answer isn't in the context, use the provided information to give the best answer you can. "
            "Only say 'I don't have information' if there is truly no relevant context provided."
        )

        # Build user message, including selected_text if provided
        if selected_text:
            user_message = (
                f"Student selected this text:\n---\n{selected_text}\n---\n\n"
                f"Context:\n{context}\n\n"
                f"Question: {question}\n\n"
                f"Please answer based on the selected text and context provided:"
            )
        else:
            user_message = f"Context:\n{context}\n\nQuestion: {question}\n\nAnswer:"

        try:
            logger.debug(f"Generating answer for question: {question[:50]}...")
            if selected_text:
                logger.debug(
                    f"Using selected text: {selected_text[:50]}..."
                )

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message},
                ],
                max_tokens=max_tokens,
                temperature=temperature,
            )

            answer = response.choices[0].message.content.strip()
            logger.debug(f"Generated answer ({len(answer)} chars)")
            return answer

        except Exception as e:
            logger.error(f"Error generating answer with Groq: {e}")
            raise GenerationError(
                "I'm having trouble generating a response. Please try again."
            )

    def health_check(self) -> bool:
        """
        Verify Groq API is accessible.

        Returns:
            True if healthy, False otherwise
        """
        if not self.client:
            return False

        try:
            # Try to list models (lightweight check)
            self.client.models.list()
            return True
        except Exception as e:
            logger.warning(f"Groq health check failed: {e}")
            return False


# Singleton instance (lazy-loaded at startup)
_generation_service: Optional[GenerationService] = None


def get_generation_service() -> GenerationService:
    """Get or initialize the generation service."""
    global _generation_service
    if _generation_service is None:
        _generation_service = GenerationService(api_key=settings.groq_api_key)
        _generation_service.connect()
    return _generation_service
