"""Chat Pipeline Service - orchestrates the full Q&A workflow."""

import time
from typing import List

from src.config import settings
from src.models import ChatRequest, ChatResponse, SourceRef
from src.services.embedding_service import EmbeddingService
from src.services.retrieval_service import RetrievalService, RetrievalError
from src.services.generation_service import GenerationService, GenerationError
from src.services.confidence_service import ConfidenceService
from src.utils.context_manager import ContextManager
from src.utils.response_formatter import ResponseFormatter
from src.utils.error_handler import ErrorHandler


class ChatPipelineService:
    """Orchestrates the complete Q&A pipeline."""

    def __init__(self) -> None:
        """Initialize pipeline with all required services."""
        self.embedding_service = EmbeddingService()
        self.embedding_service.load()
        self.retrieval_service = RetrievalService(
            qdrant_url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection_name,
        )
        self.retrieval_service.connect()
        self.generation_service = GenerationService(
            api_key=settings.groq_api_key,
            model=settings.groq_model,
        )
        self.generation_service.connect()
        self.confidence_service = ConfidenceService()
        self.context_manager = ContextManager()
        self.response_formatter = ResponseFormatter()

    async def process_question(self, request: ChatRequest) -> ChatResponse:
        """
        Process a question through the full RAG pipeline.

        Pipeline Steps:
        1. Validate input (ChatRequest)
        2. Embed question using EmbeddingService
        3. Retrieve passages using RetrievalService
        4. Prepare context using ContextManager
        5. Generate answer using GenerationService
        6. Calculate confidence using ConfidenceService
        7. Format response using ResponseFormatter

        Args:
            request: ChatRequest with question and optional selected_text

        Returns:
            ChatResponse with answer, sources, confidence, and processing_time_ms

        Raises:
            ValueError: If input validation fails
            RetrievalError: If no relevant passages found
            GenerationError: If LLM generation fails
        """
        start_time = time.time()

        try:
            # Step 1: Validate input
            self._validate_input(request)

            # Step 2: Embed question
            question_embedding = self.embedding_service.embed_text(request.question)

            # Step 3: Retrieve passages
            try:
                sources = self.retrieval_service.search(
                    embedding=question_embedding,
                    top_k=5,
                    similarity_threshold=0.3,  # Semantic search threshold - actual similarity ~0.5-0.55
                    selected_text=request.selected_text,  # Pass selected_text for prioritization
                )
            except RetrievalError:
                # No relevant passages found
                raise RetrievalError(
                    "I couldn't find information on this topic in the book. "
                    "Try rephrasing your question or checking the table of contents."
                )

            # Step 4: Prepare context
            context = self.context_manager.prepare_context(
                sources=sources,
                max_tokens=6000,
            )

            # Step 5: Generate answer
            try:
                answer_text = self.generation_service.generate_answer(
                    question=request.question,
                    context=context,
                    max_tokens=512,
                    selected_text=request.selected_text,  # Pass selected_text for LLM acknowledgment
                )
            except GenerationError:
                raise GenerationError(
                    "I'm having trouble generating a response right now. "
                    "Please try again in a moment."
                )

            # Step 6: Calculate confidence
            confidence_level = self.confidence_service.calculate_confidence(
                sources=sources,
                answer=answer_text,
                question=request.question,
            )

            # Step 7: Format response
            processing_time_ms = int((time.time() - start_time) * 1000)
            response = ChatResponse(
                answer=answer_text,
                sources=sources,
                confidence=confidence_level,
                processing_time_ms=processing_time_ms,
            )

            return response

        except (RetrievalError, GenerationError, ValueError) as e:
            # Convert service errors to HTTP-appropriate exceptions
            raise e
        except Exception as e:
            # Unexpected error
            raise GenerationError(f"Unexpected error: {str(e)}")

    def _validate_input(self, request: ChatRequest) -> None:
        """
        Validate ChatRequest input.

        Args:
            request: ChatRequest to validate

        Raises:
            ValueError: If validation fails
        """
        if not request.question or len(request.question.strip()) < 3:
            raise ValueError("Question must be at least 3 characters long")

        if len(request.question) > 1000:
            raise ValueError("Question must be at most 1000 characters long")

        if request.selected_text:
            if len(request.selected_text) > 5000:
                raise ValueError("Selected text must be at most 5000 characters")
            if not request.selected_text.strip():
                request.selected_text = None
