"""Chat API router - handles question-answering requests."""

from typing import Optional
from fastapi import APIRouter, HTTPException, status
from src.models import ChatRequest, ChatResponse, ErrorResponse
from src.utils.error_handler import ErrorHandler

router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/chat", response_model=ChatResponse, status_code=200)
async def chat_endpoint(request: ChatRequest) -> ChatResponse:
    """
    Main chat endpoint for Q&A requests.

    **Request**:
    - `question` (str): The student's question (3-1000 characters)
    - `selected_text` (optional str): Selected passage for context-focused answers

    **Response**:
    - `answer` (str): Synthesized answer from LLM
    - `sources` (list): Source references with chapter, module, section, excerpt
    - `confidence` (str): Answer confidence level (high/medium/low)
    - `processing_time_ms` (int): Total processing time in milliseconds

    **Errors**:
    - 400: Invalid request (validation error)
    - 429: Rate limit exceeded
    - 503: Service unavailable (LLM or retrieval error)
    """
    try:
        # Import here to avoid circular imports
        from src.services.chat_pipeline_service import ChatPipelineService

        # Initialize pipeline service
        pipeline = ChatPipelineService()

        # Process question through pipeline
        response = await pipeline.process_question(request)

        return response

    except ValueError as e:
        # Validation errors
        error_response, status_code = ErrorHandler.handle_validation_error(e)
        raise HTTPException(status_code=status_code, detail=error_response.dict())

    except Exception as e:
        # Generic error handling
        error_response, status_code = ErrorHandler.handle_generic_error(e)
        raise HTTPException(status_code=status_code, detail=error_response.dict())
