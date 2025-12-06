"""
User management service for Physical AI Book backend.

Handles user CRUD operations, profile management, and personalization.
"""

from typing import Optional
from uuid import UUID
from sqlalchemy.orm import Session
from sqlalchemy import select

from models.user import User
from config import settings


class UserService:
    """Service for managing user accounts and profiles."""

    @staticmethod
    def create_user(
        db: Session,
        email: str,
        name: str,
        password_hash: str,
        os: Optional[str] = None,
        gpu: Optional[str] = None,
        experience_level: Optional[str] = None,
        robotics_background: bool = False,
    ) -> User:
        """
        Create a new user account.

        Args:
            db: Database session
            email: User email (must be unique)
            name: User's display name
            password_hash: Bcrypt-hashed password
            os: Operating system ('linux', 'macos', 'windows')
            gpu: GPU type ('nvidia-rtx-4090', 'apple-m3', etc.)
            experience_level: 'beginner', 'intermediate', 'advanced'
            robotics_background: Whether user has robotics background

        Returns:
            Created User object

        Raises:
            ValueError: If email already exists
        """
        # Check for existing email
        existing = db.query(User).filter(User.email == email).first()
        if existing:
            raise ValueError(f"User with email {email} already exists")

        user = User(
            email=email,
            name=name,
            password_hash=password_hash,
            os=os,
            gpu=gpu,
            experience_level=experience_level,
            robotics_background=robotics_background,
        )

        db.add(user)
        db.commit()
        db.refresh(user)

        return user

    @staticmethod
    def get_user_by_id(db: Session, user_id: UUID) -> Optional[User]:
        """Get user by ID."""
        return db.query(User).filter(User.id == user_id).first()

    @staticmethod
    def get_user_by_email(db: Session, email: str) -> Optional[User]:
        """Get user by email."""
        return db.query(User).filter(User.email == email).first()

    @staticmethod
    def update_user(
        db: Session,
        user_id: UUID,
        **kwargs,
    ) -> Optional[User]:
        """
        Update user profile.

        Args:
            db: Database session
            user_id: User ID to update
            **kwargs: Fields to update (name, os, gpu, experience_level, robotics_background)

        Returns:
            Updated User object or None if not found
        """
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            return None

        # Whitelist updatable fields
        allowed_fields = {
            "name",
            "os",
            "gpu",
            "experience_level",
            "robotics_background",
        }

        for key, value in kwargs.items():
            if key in allowed_fields:
                setattr(user, key, value)

        db.commit()
        db.refresh(user)

        return user

    @staticmethod
    def delete_user(db: Session, user_id: UUID) -> bool:
        """
        Delete user account.

        Cascades to sessions, personalized content, and chat messages.

        Args:
            db: Database session
            user_id: User ID to delete

        Returns:
            True if deleted, False if not found
        """
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            return False

        db.delete(user)
        db.commit()

        return True

    @staticmethod
    def get_user_profile(db: Session, user_id: UUID) -> Optional[dict]:
        """
        Get user profile for personalization.

        Returns user's profile as dict for use in personalization/translation services.

        Args:
            db: Database session
            user_id: User ID

        Returns:
            Profile dict with keys: id, email, name, os, gpu, experience_level, robotics_background
            None if user not found
        """
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            return None

        return {
            "id": str(user.id),
            "email": user.email,
            "name": user.name,
            "os": user.os,
            "gpu": user.gpu,
            "experience_level": user.experience_level,
            "robotics_background": user.robotics_background,
        }

    @staticmethod
    def update_last_active(db: Session, user_id: UUID) -> Optional[User]:
        """
        Update user's last active timestamp.

        Used for analytics and activity tracking.

        Args:
            db: Database session
            user_id: User ID

        Returns:
            Updated User object or None if not found
        """
        user = db.query(User).filter(User.id == user_id).first()
        if user:
            # SQLAlchemy will auto-update via onupdate=func.now()
            # Just trigger a save
            db.commit()
            db.refresh(user)

        return user
