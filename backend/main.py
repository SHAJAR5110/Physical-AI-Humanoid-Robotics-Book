"""Main FastAPI application for RAG Chatbot."""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.config import settings
from src.routers import chat

# Configure logging
logging.basicConfig(
    level=settings.log_level.upper(),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan context manager."""
    # Startup
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Allowed origins: {settings.allowed_origins_list}")

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API...")


# Initialize FastAPI app
app = FastAPI(
    title=settings.app_name,
    description="RAG Chatbot for Physical AI & Humanoid Robotics Textbook",
    version=settings.app_version,
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
)

# Register routers
app.include_router(chat.router)


@app.get("/health")
async def health_check() -> dict:
    """Health check endpoint for monitoring and deployment verification."""
    return {
        "status": "healthy",
        "app": settings.app_name,
        "version": settings.app_version,
        "environment": settings.environment,
    }


@app.get("/")
async def root() -> dict:
    """Root endpoint with API information."""
    return {
        "message": "RAG Chatbot API",
        "docs": "/api/docs",
        "health": "/health",
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.reload,
        log_level=settings.log_level.lower(),
    )
