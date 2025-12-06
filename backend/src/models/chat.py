"""
Chat message model for Physical AI Book backend.

Represents user questions and assistant responses with citations.
"""

from sqlalchemy import Column, String, Text, Integer, DateTime, UUID, ForeignKey, Index
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from datetime import datetime
import uuid

from db.connection import Base


class ChatMessage(Base):
    """
    Chat message model for RAG chatbot.

    Stores user questions, assistant responses, and citations to source chapters.
    """

    __tablename__ = "chat_messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id", ondelete="CASCADE"), nullable=False, index=True)

    # Message content
    user_message = Column(Text, nullable=False)
    assistant_response = Column(Text, nullable=False)

    # Citations and metadata
    citations = Column(Text, nullable=True)  # JSON array: [{"chapter_id": 1, "section": "...", "excerpt": "..."}]
    source_chapters = Column(Text, nullable=True)  # JSON array: [1, 2, 3]
    tokens_used = Column(Integer, nullable=True)

    # Metadata
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False, index=True)

    # Relationships
    user = relationship("User", back_populates="chat_messages")
    chapter = relationship("Chapter", back_populates="chat_messages")

    # Indexes
    __table_args__ = (
        Index("idx_chat_messages_user_id", "user_id"),
        Index("idx_chat_messages_chapter_id", "chapter_id"),
        Index("idx_chat_messages_created_at", "created_at"),
    )

    def __repr__(self) -> str:
        return f"<ChatMessage(id={self.id}, user_id={self.user_id}, chapter_id={self.chapter_id})>"
