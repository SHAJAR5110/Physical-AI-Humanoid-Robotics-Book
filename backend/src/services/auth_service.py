"""
Authentication service for Physical AI Book backend.

Handles BetterAuth integration, JWT token validation, and session management.
"""

import logging
from typing import Optional, Dict, Any
from datetime import datetime, timedelta
from jose import JWTError, jwt
from passlib.context import CryptContext
import uuid
from sqlalchemy.orm import Session

from config import settings
from db.connection import SessionLocal
from models.user import User, Session as SessionModel

logger = logging.getLogger(__name__)

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class AuthService:
    """
    Authentication service for user signup, signin, signout, and session management.

    Integrates with BetterAuth for password handling and JWT tokens.
    """

    @staticmethod
    def hash_password(password: str) -> str:
        """Hash password using bcrypt."""
        return pwd_context.hash(password)

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify plain password against hashed password."""
        return pwd_context.verify(plain_password, hashed_password)

    @staticmethod
    def create_access_token(user_id: str, expires_delta: Optional[timedelta] = None) -> str:
        """
        Create JWT access token.

        Args:
            user_id: User UUID
            expires_delta: Token expiration delta (default: 24 hours)

        Returns:
            JWT token string
        """
        if expires_delta is None:
            expires_delta = timedelta(hours=settings.session_ttl_hours)

        expire = datetime.utcnow() + expires_delta
        to_encode = {"sub": str(user_id), "exp": expire}

        encoded_jwt = jwt.encode(
            to_encode,
            settings.token_secret_key,
            algorithm=settings.token_algorithm,
        )
        return encoded_jwt

    @staticmethod
    def verify_token(token: str) -> Optional[str]:
        """
        Verify JWT token and return user_id.

        Args:
            token: JWT token

        Returns:
            User ID if valid, None otherwise
        """
        try:
            payload = jwt.decode(
                token,
                settings.token_secret_key,
                algorithms=[settings.token_algorithm],
            )
            user_id: str = payload.get("sub")
            if user_id is None:
                return None
            return user_id
        except JWTError:
            return None

    @staticmethod
    def signup(
        email: str,
        password: str,
        name: str,
        os: Optional[str] = None,
        gpu: Optional[str] = None,
        experience_level: Optional[str] = None,
        robotics_background: bool = False,
        db: Optional[Session] = None,
    ) -> Dict[str, Any]:
        """
        Create new user account.

        Args:
            email: User email
            password: Plain text password
            name: User display name
            os: Operating system
            gpu: GPU model
            experience_level: Skill level
            robotics_background: Robotics experience flag
            db: Database session

        Returns:
            Dict with user_id, session_token, expires_at, or error message
        """
        if db is None:
            db = SessionLocal()

        try:
            # Check if email already exists
            existing_user = db.query(User).filter_by(email=email).first()
            if existing_user:
                logger.warning(f"Signup failed: email {email} already registered")
                return {
                    "success": False,
                    "error": "Email already registered",
                    "status": 400,
                }

            # Hash password
            password_hash = AuthService.hash_password(password)

            # Create user
            new_user = User(
                id=uuid.uuid4(),
                email=email,
                name=name,
                password_hash=password_hash,
                os=os,
                gpu=gpu,
                experience_level=experience_level,
                robotics_background=robotics_background,
            )
            db.add(new_user)
            db.commit()
            db.refresh(new_user)

            logger.info(f"User created: {new_user.id} ({email})")

            # Create session
            session_result = AuthService.create_session(new_user.id, db)
            if not session_result["success"]:
                return session_result

            return {
                "success": True,
                "user_id": str(new_user.id),
                "email": new_user.email,
                "name": new_user.name,
                "session_token": session_result["session_token"],
                "expires_at": session_result["expires_at"],
            }
        except Exception as e:
            db.rollback()
            logger.error(f"Signup error: {e}")
            return {
                "success": False,
                "error": "Signup failed",
                "status": 500,
            }

    @staticmethod
    def signin(
        email: str,
        password: str,
        db: Optional[Session] = None,
    ) -> Dict[str, Any]:
        """
        Authenticate user and create session.

        Args:
            email: User email
            password: Plain text password
            db: Database session

        Returns:
            Dict with session_token, expires_at, or error message
        """
        if db is None:
            db = SessionLocal()

        try:
            user = db.query(User).filter_by(email=email).first()
            if not user or not AuthService.verify_password(password, user.password_hash):
                logger.warning(f"Signin failed: invalid credentials for {email}")
                return {
                    "success": False,
                    "error": "Invalid email or password",
                    "status": 401,
                }

            # Create session
            session_result = AuthService.create_session(user.id, db)
            if not session_result["success"]:
                return session_result

            logger.info(f"User signin: {user.id} ({email})")

            return {
                "success": True,
                "user_id": str(user.id),
                "session_token": session_result["session_token"],
                "expires_at": session_result["expires_at"],
                "user": {
                    "id": str(user.id),
                    "email": user.email,
                    "name": user.name,
                    "os": user.os,
                    "gpu": user.gpu,
                    "experience_level": user.experience_level,
                    "robotics_background": user.robotics_background,
                }
            }
        except Exception as e:
            logger.error(f"Signin error: {e}")
            return {
                "success": False,
                "error": "Signin failed",
                "status": 500,
            }

    @staticmethod
    def create_session(user_id: str, db: Optional[Session] = None) -> Dict[str, Any]:
        """
        Create authenticated session for user.

        Args:
            user_id: User UUID
            db: Database session

        Returns:
            Dict with session_token, expires_at
        """
        if db is None:
            db = SessionLocal()

        try:
            # Create JWT token
            token = AuthService.create_access_token(user_id)
            token_hash = AuthService.hash_password(token)

            # Calculate expiration
            expires_at = datetime.utcnow() + timedelta(hours=settings.session_ttl_hours)

            # Store session in database
            session = SessionModel(
                session_id=uuid.uuid4(),
                user_id=uuid.UUID(user_id),
                token_hash=token_hash,
                expires_at=expires_at,
            )
            db.add(session)
            db.commit()

            logger.info(f"Session created for user {user_id}")

            return {
                "success": True,
                "session_token": token,
                "expires_at": expires_at.isoformat(),
            }
        except Exception as e:
            db.rollback()
            logger.error(f"Session creation error: {e}")
            return {
                "success": False,
                "error": "Session creation failed",
                "status": 500,
            }

    @staticmethod
    def validate_token(token: str, db: Optional[Session] = None) -> Optional[str]:
        """
        Validate token and return user_id.

        Args:
            token: Bearer token
            db: Database session

        Returns:
            User ID if token valid and not expired, None otherwise
        """
        if db is None:
            db = SessionLocal()

        try:
            # Verify JWT
            user_id = AuthService.verify_token(token)
            if not user_id:
                return None

            # Check if user exists
            user = db.query(User).filter_by(id=uuid.UUID(user_id)).first()
            if not user:
                return None

            # Verify session hasn't expired
            session = db.query(SessionModel).filter_by(user_id=uuid.UUID(user_id)).first()
            if not session or session.expires_at < datetime.utcnow():
                return None

            return user_id
        except Exception as e:
            logger.error(f"Token validation error: {e}")
            return None

    @staticmethod
    def signout(user_id: str, db: Optional[Session] = None) -> Dict[str, Any]:
        """
        Sign out user by invalidating session.

        Args:
            user_id: User UUID
            db: Database session

        Returns:
            Success message
        """
        if db is None:
            db = SessionLocal()

        try:
            # Delete user's sessions
            db.query(SessionModel).filter_by(user_id=uuid.UUID(user_id)).delete()
            db.commit()

            logger.info(f"User signout: {user_id}")

            return {
                "success": True,
                "message": "Successfully signed out",
            }
        except Exception as e:
            db.rollback()
            logger.error(f"Signout error: {e}")
            return {
                "success": False,
                "error": "Signout failed",
                "status": 500,
            }


# Create singleton instance
_auth_service = AuthService()


def get_auth_service() -> AuthService:
    """Get auth service instance."""
    return _auth_service
