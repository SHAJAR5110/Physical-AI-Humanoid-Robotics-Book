"""Pydantic models for request and response validation."""

from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator


class SourceRef(BaseModel):
    """Reference to a source passage in the textbook."""

    chapter: str = Field(..., description="Chapter name")
    module: str = Field(..., description="Module name")
    section: str = Field(..., description="Section anchor (slugified)")
    excerpt: str = Field(..., description="Relevant text excerpt")
    similarity: Optional[float] = Field(None, ge=0.0, le=1.0, description="Cosine similarity score")

    class Config:
        """Pydantic configuration."""
        json_schema_extra = {
            "example": {
                "chapter": "Chapter 2: ROS 2 Fundamentals",
                "module": "Module 2.1: Nodes and Topics",
                "section": "ros-2-topics",
                "excerpt": "ROS 2 is built on a publish-subscribe architecture...",
                "similarity": 0.92,
            }
        }


class ChatRequest(BaseModel):
    """Request model for the chat endpoint."""

    question: str = Field(
        ...,
        min_length=3,
        max_length=1000,
        description="User's question about book content",
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=5000,
        description="Optional selected text from the book",
    )

    @field_validator("question")
    @classmethod
    def validate_question(cls, v: str) -> str:
        """Validate question for injection attacks and sanitization."""
        # Check for obvious injection patterns
        dangerous_patterns = ["<script", "DROP ", "DELETE ", "INSERT ", "UPDATE "]
        v_upper = v.upper()

        for pattern in dangerous_patterns:
            if pattern in v_upper:
                raise ValueError("Invalid characters in question")

        return v.strip()

    @field_validator("selected_text")
    @classmethod
    def validate_selected_text(cls, v: Optional[str]) -> Optional[str]:
        """Validate selected text."""
        if v is None:
            return None

        v = v.strip()
        if not v:
            return None

        return v

    class Config:
        """Pydantic configuration."""
        json_schema_extra = {
            "example": {
                "question": "What is ROS 2?",
                "selected_text": None,
            }
        }


class ChatResponse(BaseModel):
    """Response model for the chat endpoint."""

    answer: str = Field(
        ...,
        min_length=10,
        max_length=2000,
        description="Generated answer to the question",
    )
    sources: list[SourceRef] = Field(
        ...,
        min_length=1,
        max_length=5,
        description="Source references for the answer",
    )
    confidence: Literal["high", "medium", "low"] = Field(
        ...,
        description="Confidence level of the answer",
    )
    processing_time_ms: int = Field(
        ...,
        ge=0,
        description="Time taken to generate answer in milliseconds",
    )

    class Config:
        """Pydantic configuration."""
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 is a middleware for robotics that provides tools and libraries for building robot applications...",
                "sources": [
                    {
                        "chapter": "Chapter 2: ROS 2 Fundamentals",
                        "module": "Module 2.1: Nodes and Topics",
                        "section": "ros-2-topics",
                        "excerpt": "ROS 2 is built on a publish-subscribe architecture...",
                        "similarity": 0.92,
                    }
                ],
                "confidence": "high",
                "processing_time_ms": 1850,
            }
        }


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(..., description="Error type/title")
    details: str = Field(..., description="Detailed error message")

    class Config:
        """Pydantic configuration."""
        json_schema_extra = {
            "example": {
                "error": "Validation failed",
                "details": "question must be at least 3 characters",
            }
        }


class HealthResponse(BaseModel):
    """Health check response model."""

    status: str = Field(..., description="Health status")
    app: str = Field(..., description="Application name")
    version: str = Field(..., description="Application version")
    environment: str = Field(..., description="Environment (development/production)")

    class Config:
        """Pydantic configuration."""
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "app": "RAG Chatbot API",
                "version": "1.0.0",
                "environment": "development",
            }
        }
