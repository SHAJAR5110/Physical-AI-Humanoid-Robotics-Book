"""Configuration management for RAG Chatbot API."""

from typing import Optional

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    app_name: str = "RAG Chatbot API"
    app_version: str = "1.0.0"
    environment: str = "development"
    debug: bool = True

    # Server
    host: str = "0.0.0.0"
    port: int = 8000
    reload: bool = True

    # Logging
    log_level: str = "debug"

    # CORS
    allowed_origins: str = "http://localhost:3000,http://localhost:8000"

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_passages"

    # Groq LLM Configuration
    groq_api_key: str
    groq_model: str = "llama-3.3-70b-versatile"

    # Optional: Database
    database_url: Optional[str] = None

    # Optional: Sentry
    sentry_dsn: Optional[str] = None

    class Config:
        """Pydantic configuration."""

        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

    @property
    def allowed_origins_list(self) -> list[str]:
        """Parse comma-separated origins into list."""
        return [origin.strip() for origin in self.allowed_origins.split(",")]

    def get_qdrant_config(self) -> dict:
        """Get Qdrant client configuration."""
        return {
            "url": self.qdrant_url,
            "api_key": self.qdrant_api_key,
            "prefer_grpc": False,  # Use HTTP for simplicity
        }

    def get_groq_config(self) -> dict:
        """Get Groq client configuration."""
        return {
            "api_key": self.groq_api_key,
        }


# Singleton instance
settings = Settings()
