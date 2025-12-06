"""
Chapter and related models for Physical AI Book backend.

Represents book chapters, embeddings, and personalized content.
"""

from sqlalchemy import Column, String, Text, Integer, DateTime, UUID, ForeignKey, Index
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from datetime import datetime
import uuid

from db.connection import Base


class Chapter(Base):
    """
    Book chapter model.

    Stores chapter metadata and content for the 6-chapter Physical AI Book.
    """

    __tablename__ = "chapters"

    id = Column(Integer, primary_key=True)
    title = Column(String(255), nullable=False)
    slug = Column(String(255), unique=True, nullable=False, index=True)
    content = Column(Text, nullable=False)
    summary = Column(Text, nullable=True)
    author = Column(String(255), nullable=True)
    version = Column(String(10), default="1.0")
    page_count = Column(Integer, nullable=True)

    # Metadata
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationships
    embeddings = relationship("ChapterEmbedding", back_populates="chapter", cascade="all, delete-orphan")
    personalized_contents = relationship(
        "PersonalizedContent", back_populates="chapter", cascade="all, delete-orphan"
    )
    chat_messages = relationship("ChatMessage", back_populates="chapter", cascade="all, delete-orphan")

    def __repr__(self) -> str:
        return f"<Chapter(id={self.id}, title={self.title}, slug={self.slug})>"


class ChapterEmbedding(Base):
    """
    Chapter embedding metadata model.

    Stores metadata for chapter embeddings (actual vectors stored in Qdrant).
    """

    __tablename__ = "chapter_embeddings"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(Integer, ForeignKey("chapters.id", ondelete="CASCADE"), nullable=False, index=True)
    section_title = Column(String(255), nullable=True)
    section_index = Column(Integer, nullable=True)
    content_excerpt = Column(Text, nullable=False)
    embedding_model = Column(String(50), default="claude-embeddings-3")
    vector_dimensions = Column(Integer, default=1024)
    qdrant_point_id = Column(UUID(as_uuid=True), nullable=True, index=True)

    # Metadata
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationships
    chapter = relationship("Chapter", back_populates="embeddings")

    # Unique constraint: max 1 embedding per section
    __table_args__ = (
        Index("idx_chapter_embeddings_chapter_id", "chapter_id"),
        Index("idx_chapter_embeddings_qdrant_point_id", "qdrant_point_id"),
    )

    def __repr__(self) -> str:
        return f"<ChapterEmbedding(chapter_id={self.chapter_id}, section={self.section_title})>"


class PersonalizedContent(Base):
    """
    Cached personalized chapter content model.

    Stores Claude API outputs for chapter rewrites based on user profile.
    """

    __tablename__ = "personalized_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id", ondelete="CASCADE"), nullable=False, index=True)
    personalized_text = Column(Text, nullable=False)
    model_version = Column(String(50), default="claude-3-5-haiku-20241022")
    cache_source = Column(String(50), default="claude-api")  # 'claude-api' or 'database'

    # Cache expiration (TTL)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    expires_at = Column(DateTime(timezone=True), nullable=False)

    # Relationships
    user = relationship("User", back_populates="personalized_contents")
    chapter = relationship("Chapter", back_populates="personalized_contents")

    # Unique constraint: max 1 personalized version per user per chapter
    __table_args__ = (
        Index("idx_personalized_content_user_id", "user_id"),
        Index("idx_personalized_content_chapter_id", "chapter_id"),
        Index("idx_personalized_content_user_chapter", "user_id", "chapter_id"),
        Index("idx_personalized_content_expires_at", "expires_at"),
    )

    def __repr__(self) -> str:
        return f"<PersonalizedContent(user_id={self.user_id}, chapter_id={self.chapter_id})>"

    def is_expired(self) -> bool:
        """Check if cache has expired."""
        return datetime.utcnow() >= self.expires_at
