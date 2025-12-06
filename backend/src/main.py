"""
FastAPI application entry point for Physical AI Book backend.

Initializes the application, configures middleware, and includes all routes.
"""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
import uvicorn

from config import settings
from db.connection import init_db
from db import models  # Import models for Alembic to detect them

# Configure logging
logging.basicConfig(
    level=settings.log_level,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan context manager.

    Runs startup logic before app starts, cleanup after app shuts down.
    """
    # Startup
    logger.info("Starting Physical AI Book API...")
    init_db()
    logger.info("Database initialized")

    yield

    # Shutdown
    logger.info("Shutting down Physical AI Book API...")


# Create FastAPI application
app = FastAPI(
    title=settings.title,
    version=settings.version,
    description="REST API for Physical AI & Humanoid Robotics Book MVP",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.get_cors_origins(),
    allow_credentials=settings.cors_credentials,
    allow_methods=settings.get_cors_methods(),
    allow_headers=settings.get_cors_headers(),
)

# Add trusted host middleware
app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=["localhost", "127.0.0.1", "*.render.com"],
)


@app.get("/")
async def root():
    """Root endpoint - API welcome message."""
    return {
        "message": "Welcome to Physical AI Book API",
        "docs": "/docs",
        "openapi": "/openapi.json",
    }


@app.get("/health")
async def health_check():
    """
    Health check endpoint.

    Verifies database, Qdrant, and Claude API connectivity.
    """
    from db.connection import health_check as db_health_check

    db_status = db_health_check()

    return {
        "status": "ok" if db_status else "degraded",
        "services": {
            "database": "ok" if db_status else "down",
            "qdrant": "ok",  # TODO: Implement Qdrant health check
            "claude_api": "ok",  # TODO: Implement Claude API health check
        },
        "version": settings.version,
        "environment": settings.environment,
    }


# TODO: Import and include routers
# from api.routes import auth, chapters, personalize, translate, chat
# app.include_router(auth.router, prefix=f"{settings.api_prefix}/auth", tags=["auth"])
# app.include_router(chapters.router, prefix=f"{settings.api_prefix}/chapters", tags=["chapters"])
# app.include_router(personalize.router, prefix=f"{settings.api_prefix}", tags=["personalization"])
# app.include_router(translate.router, prefix=f"{settings.api_prefix}", tags=["translation"])
# app.include_router(chat.router, prefix=f"{settings.api_prefix}", tags=["chat"])


if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.reload,
        log_level=settings.log_level.lower(),
    )
