"""
Session management service for Physical AI Book backend.

Handles session lifecycle: creation, validation, and expiration.
"""

from typing import Optional
from uuid import UUID
from datetime import datetime, timedelta
import hashlib

from sqlalchemy.orm import Session
from sqlalchemy import select

from models.user import Session as SessionModel


class SessionService:
    """Service for managing user sessions and tokens."""

    @staticmethod
    def create_session(
        db: Session,
        user_id: UUID,
        token: str,
        ttl_hours: int = 24,
    ) -> SessionModel:
        """
        Create a new user session.

        Args:
            db: Database session
            user_id: User ID for the session
            token: JWT token (will be hashed)
            ttl_hours: Time-to-live in hours (default 24)

        Returns:
            Created Session object
        """
        # Hash token for storage (never store plaintext tokens)
        token_hash = hashlib.sha256(token.encode()).hexdigest()

        # Calculate expiration
        expires_at = datetime.utcnow() + timedelta(hours=ttl_hours)

        session = SessionModel(
            user_id=user_id,
            token_hash=token_hash,
            expires_at=expires_at,
        )

        db.add(session)
        db.commit()
        db.refresh(session)

        return session

    @staticmethod
    def validate_token(
        db: Session,
        token: str,
        user_id: Optional[UUID] = None,
    ) -> bool:
        """
        Validate a JWT token against stored session.

        Checks:
        1. Token hash exists in database
        2. Session not expired
        3. (Optionally) matches expected user_id

        Args:
            db: Database session
            token: JWT token to validate
            user_id: (Optional) expected user_id for additional check

        Returns:
            True if token is valid, False otherwise
        """
        # Hash incoming token
        token_hash = hashlib.sha256(token.encode()).hexdigest()

        # Query for session
        query = db.query(SessionModel).filter(
            SessionModel.token_hash == token_hash,
        )

        # Optionally filter by user_id
        if user_id:
            query = query.filter(SessionModel.user_id == user_id)

        session = query.first()

        if not session:
            return False

        # Check expiration
        if not session.is_valid():
            return False

        return True

    @staticmethod
    def get_session_by_token(
        db: Session,
        token: str,
    ) -> Optional[SessionModel]:
        """
        Get session by token.

        Args:
            db: Database session
            token: JWT token

        Returns:
            Session object if valid, None otherwise
        """
        token_hash = hashlib.sha256(token.encode()).hexdigest()

        session = db.query(SessionModel).filter(
            SessionModel.token_hash == token_hash,
        ).first()

        if not session or not session.is_valid():
            return None

        return session

    @staticmethod
    def get_active_sessions(
        db: Session,
        user_id: UUID,
    ) -> list[SessionModel]:
        """
        Get all active sessions for a user.

        Args:
            db: Database session
            user_id: User ID

        Returns:
            List of active Session objects
        """
        now = datetime.utcnow()

        sessions = db.query(SessionModel).filter(
            SessionModel.user_id == user_id,
            SessionModel.expires_at > now,
        ).all()

        return sessions

    @staticmethod
    def expire_session(
        db: Session,
        session_id: UUID,
    ) -> bool:
        """
        Manually expire a session (logout).

        Args:
            db: Database session
            session_id: Session ID to expire

        Returns:
            True if expired, False if not found
        """
        session = db.query(SessionModel).filter(
            SessionModel.session_id == session_id,
        ).first()

        if not session:
            return False

        # Set expiration to now
        session.expires_at = datetime.utcnow()
        db.commit()

        return True

    @staticmethod
    def expire_all_user_sessions(
        db: Session,
        user_id: UUID,
    ) -> int:
        """
        Expire all sessions for a user (logout all devices).

        Args:
            db: Database session
            user_id: User ID

        Returns:
            Number of sessions expired
        """
        now = datetime.utcnow()

        sessions = db.query(SessionModel).filter(
            SessionModel.user_id == user_id,
            SessionModel.expires_at > now,
        ).all()

        count = len(sessions)

        for session in sessions:
            session.expires_at = now
            db.add(session)

        db.commit()

        return count

    @staticmethod
    def cleanup_expired_sessions(db: Session) -> int:
        """
        Delete expired sessions from database.

        Should be run periodically (e.g., daily).

        Args:
            db: Database session

        Returns:
            Number of sessions deleted
        """
        now = datetime.utcnow()

        expired = db.query(SessionModel).filter(
            SessionModel.expires_at <= now,
        ).delete()

        db.commit()

        return expired
