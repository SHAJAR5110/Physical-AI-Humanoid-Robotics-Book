"""
User and Session models for Physical AI Book backend.

Represents user profiles and authenticated sessions.
"""

from sqlalchemy import Column, String, Boolean, DateTime, UUID, Text, ForeignKey, Index
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from datetime import datetime
import uuid

from db.connection import Base


class User(Base):
    """
    User profile model.

    Stores user registration data including hardware profile,
    experience level, and robotics background for personalization.
    """

    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    name = Column(String(255), nullable=False)
    password_hash = Column(String(255), nullable=False)  # BetterAuth will handle actual passwords

    # User profile for personalization
    os = Column(String(50), nullable=True)  # 'linux', 'macos', 'windows'
    gpu = Column(String(100), nullable=True)  # 'nvidia-rtx-4090', 'apple-m3'
    experience_level = Column(String(50), nullable=True)  # 'beginner', 'intermediate', 'advanced'
    robotics_background = Column(Boolean, default=False)

    # Metadata
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationships
    sessions = relationship("Session", back_populates="user", cascade="all, delete-orphan")
    personalized_contents = relationship(
        "PersonalizedContent", back_populates="user", cascade="all, delete-orphan"
    )
    chat_messages = relationship("ChatMessage", back_populates="user", cascade="all, delete-orphan")

    def __repr__(self) -> str:
        return f"<User(id={self.id}, email={self.email}, name={self.name})>"


class Session(Base):
    """
    User session model.

    Represents an authenticated session with JWT token and expiration.
    """

    __tablename__ = "sessions"

    session_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    token_hash = Column(String(255), unique=True, nullable=False, index=True)
    expires_at = Column(DateTime(timezone=True), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)

    # Relationships
    user = relationship("User", back_populates="sessions")

    # Indexes
    __table_args__ = (
        Index("idx_sessions_user_id", "user_id"),
        Index("idx_sessions_expires_at", "expires_at"),
    )

    def __repr__(self) -> str:
        return f"<Session(session_id={self.session_id}, user_id={self.user_id}, expires_at={self.expires_at})>"

    def is_valid(self) -> bool:
        """Check if session is still valid (not expired)."""
        return datetime.utcnow() < self.expires_at
