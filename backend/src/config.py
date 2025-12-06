"""
Configuration management for Physical AI Book backend.

Loads environment variables and provides centralized configuration
for FastAPI application, database, and external services.
"""

from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings from environment variables."""

    # FastAPI Configuration
    environment: str = "development"
    debug: bool = True
    log_level: str = "INFO"
    title: str = "Physical AI Book API"
    version: str = "1.0.0"

    # Server Configuration
    host: str = "0.0.0.0"
    port: int = 8000
    reload: bool = True

    # Database (Neon Postgres)
    database_url: str = "postgresql://user:password@localhost/physical_ai_book"
    neon_api_key: Optional[str] = None
    db_pool_size: int = 20
    db_max_overflow: int = 0

    # Vector Database (Qdrant)
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "chapters"

    # Anthropic Claude API
    anthropic_api_key: str
    anthropic_model: str = "claude-3-5-haiku-20241022"
    anthropic_embedding_model: str = "claude-embeddings-3"

    # BetterAuth Configuration
    betterauth_project_id: Optional[str] = None
    betterauth_secret: Optional[str] = None

    # CORS Configuration
    cors_origins: str = "http://localhost:3000,http://localhost:5173"
    cors_credentials: bool = True
    cors_methods: str = "GET,POST,PUT,DELETE,OPTIONS"
    cors_headers: str = "Content-Type,Authorization"

    # API Configuration
    api_base_url: str = "http://localhost:8000/api"
    api_prefix: str = "/api"

    # Rate Limiting
    rate_limit_personalization: int = 10  # Per hour
    rate_limit_translation: int = 10  # Per hour
    rate_limit_chat: int = 30  # Per hour

    # Session Configuration
    session_ttl_hours: int = 24
    token_algorithm: str = "HS256"
    token_secret_key: str = "your-secret-key-change-in-production"

    # Caching Configuration
    cache_ttl_days: int = 30
    cache_max_size: int = 5000

    class Config:
        """Pydantic config for Settings."""
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

    def get_cors_origins(self) -> list[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    def get_cors_methods(self) -> list[str]:
        """Parse CORS methods from comma-separated string."""
        return [method.strip() for method in self.cors_methods.split(",")]

    def get_cors_headers(self) -> list[str]:
        """Parse CORS headers from comma-separated string."""
        return [header.strip() for header in self.cors_headers.split(",")]


# Global settings instance
settings = Settings()
