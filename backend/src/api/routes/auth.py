"""
Authentication API routes for Physical AI Book backend.

Endpoints for signup, signin, signout, and user profile management.
"""

import logging
from typing import Optional
from uuid import UUID
from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, EmailStr, Field

from db.connection import get_db
from services.auth_service import AuthService
from services.user_service import UserService
from sqlalchemy.orm import Session

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/auth", tags=["auth"])


# ============================================================================
# Request/Response Models
# ============================================================================


class SignUpRequest(BaseModel):
    """Signup request model."""

    email: EmailStr
    password: str = Field(..., min_length=8, max_length=255)
    name: str = Field(..., min_length=1, max_length=255)
    os: Optional[str] = Field(None, description="Operating system: linux, macos, windows")
    gpu: Optional[str] = Field(None, description="GPU model")
    experience_level: Optional[str] = Field(
        None, description="Skill level: beginner, intermediate, advanced"
    )
    robotics_background: bool = Field(False, description="Has robotics experience")


class SignInRequest(BaseModel):
    """Signin request model."""

    email: EmailStr
    password: str


class SignOutRequest(BaseModel):
    """Signout request model."""

    user_id: str


class ProfileUpdateRequest(BaseModel):
    """Profile update request model."""

    name: Optional[str] = None
    os: Optional[str] = None
    gpu: Optional[str] = None
    experience_level: Optional[str] = None
    robotics_background: Optional[bool] = None


class UserProfileResponse(BaseModel):
    """User profile response model."""

    id: str
    email: str
    name: str
    os: Optional[str]
    gpu: Optional[str]
    experience_level: Optional[str]
    robotics_background: bool


class SignUpResponse(BaseModel):
    """Signup response model."""

    success: bool
    user_id: str
    email: str
    name: str
    session_token: str
    expires_at: str


class SignInResponse(BaseModel):
    """Signin response model."""

    success: bool
    user_id: str
    session_token: str
    expires_at: str
    user: UserProfileResponse


class ErrorResponse(BaseModel):
    """Error response model."""

    success: bool = False
    error: str
    status: int


# ============================================================================
# Authentication Middleware Helper
# ============================================================================


async def get_current_user_id(
    authorization: Optional[str] = None,
    db: Session = Depends(get_db),
) -> str:
    """
    Extract and validate bearer token from Authorization header.

    Args:
        authorization: Authorization header value
        db: Database session

    Returns:
        User ID if valid

    Raises:
        HTTPException: If token invalid or missing
    """
    if not authorization:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing Authorization header",
        )

    try:
        scheme, token = authorization.split(" ")
        if scheme.lower() != "bearer":
            raise ValueError("Invalid auth scheme")
    except (ValueError, IndexError):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid Authorization header format",
        )

    user_id = AuthService.validate_token(token, db)
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    return user_id


# ============================================================================
# API Endpoints
# ============================================================================


@router.post("/signup", response_model=SignUpResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    request: SignUpRequest,
    db: Session = Depends(get_db),
) -> SignUpResponse:
    """
    Create new user account.

    **Request Body**:
    - email: User email (must be unique)
    - password: Password (min 8 chars)
    - name: User display name
    - os: (Optional) Operating system
    - gpu: (Optional) GPU model
    - experience_level: (Optional) Skill level
    - robotics_background: (Optional) Robotics experience

    **Responses**:
    - 201: Account created, returns user_id and session token
    - 400: Email already registered
    - 422: Invalid request format
    - 500: Server error

    **Example**:
    ```bash
    curl -X POST http://localhost:8000/api/auth/signup \
      -H "Content-Type: application/json" \
      -d '{
        "email": "alice@example.com",
        "password": "SecurePassword123",
        "name": "Alice",
        "os": "linux",
        "gpu": "nvidia-rtx-4090",
        "experience_level": "intermediate",
        "robotics_background": true
      }'
    ```

    **Success Response**:
    ```json
    {
      "success": true,
      "user_id": "550e8400-e29b-41d4-a716-446655440000",
      "email": "alice@example.com",
      "name": "Alice",
      "session_token": "eyJhbGciOiJIUzI1NiIs...",
      "expires_at": "2025-12-07T12:00:00"
    }
    ```
    """
    try:
        result = AuthService.signup(
            email=request.email,
            password=request.password,
            name=request.name,
            os=request.os,
            gpu=request.gpu,
            experience_level=request.experience_level,
            robotics_background=request.robotics_background,
            db=db,
        )

        if not result["success"]:
            raise HTTPException(
                status_code=result.get("status", 400),
                detail=result.get("error", "Signup failed"),
            )

        return SignUpResponse(**result)

    except Exception as e:
        logger.error(f"Signup error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Signup failed",
        )


@router.post("/signin", response_model=SignInResponse)
async def signin(
    request: SignInRequest,
    db: Session = Depends(get_db),
) -> SignInResponse:
    """
    Sign in with email and password.

    **Request Body**:
    - email: User email
    - password: User password

    **Responses**:
    - 200: Auth successful, returns session token
    - 401: Invalid credentials
    - 422: Invalid request format
    - 500: Server error

    **Example**:
    ```bash
    curl -X POST http://localhost:8000/api/auth/signin \
      -H "Content-Type: application/json" \
      -d '{
        "email": "alice@example.com",
        "password": "SecurePassword123"
      }'
    ```

    **Success Response**:
    ```json
    {
      "success": true,
      "user_id": "550e8400-e29b-41d4-a716-446655440000",
      "session_token": "eyJhbGciOiJIUzI1NiIs...",
      "expires_at": "2025-12-07T12:00:00",
      "user": {
        "id": "550e8400-e29b-41d4-a716-446655440000",
        "email": "alice@example.com",
        "name": "Alice",
        "os": "linux",
        "gpu": "nvidia-rtx-4090",
        "experience_level": "intermediate",
        "robotics_background": true
      }
    }
    ```
    """
    try:
        result = AuthService.signin(
            email=request.email,
            password=request.password,
            db=db,
        )

        if not result["success"]:
            raise HTTPException(
                status_code=result.get("status", 401),
                detail=result.get("error", "Signin failed"),
            )

        return SignInResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signin error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Signin failed",
        )


@router.post("/signout")
async def signout(
    authorization: Optional[str] = None,
    db: Session = Depends(get_db),
) -> JSONResponse:
    """
    Sign out user (invalidate session).

    **Headers**:
    - Authorization: Bearer <session_token>

    **Responses**:
    - 200: Signout successful
    - 401: Invalid token
    - 500: Server error

    **Example**:
    ```bash
    curl -X POST http://localhost:8000/api/auth/signout \
      -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIs..."
    ```

    **Success Response**:
    ```json
    {
      "success": true,
      "message": "Successfully signed out"
    }
    ```
    """
    try:
        user_id = await get_current_user_id(authorization, db)

        result = AuthService.signout(user_id, db)

        if not result["success"]:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=result.get("error", "Signout failed"),
            )

        return JSONResponse(
            status_code=status.HTTP_200_OK,
            content=result,
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signout error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Signout failed",
        )


@router.get("/profile", response_model=UserProfileResponse)
async def get_profile(
    authorization: Optional[str] = None,
    db: Session = Depends(get_db),
) -> UserProfileResponse:
    """
    Get current user's profile.

    **Headers**:
    - Authorization: Bearer <session_token>

    **Responses**:
    - 200: Profile returned
    - 401: Invalid token
    - 404: User not found
    - 500: Server error

    **Example**:
    ```bash
    curl -X GET http://localhost:8000/api/auth/profile \
      -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIs..."
    ```

    **Success Response**:
    ```json
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "email": "alice@example.com",
      "name": "Alice",
      "os": "linux",
      "gpu": "nvidia-rtx-4090",
      "experience_level": "intermediate",
      "robotics_background": true
    }
    ```
    """
    try:
        user_id = await get_current_user_id(authorization, db)

        profile = UserService.get_user_profile(db, UUID(user_id))
        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found",
            )

        return UserProfileResponse(**profile)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get profile error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Get profile failed",
        )


@router.patch("/profile", response_model=UserProfileResponse)
async def update_profile(
    request: ProfileUpdateRequest,
    authorization: Optional[str] = None,
    db: Session = Depends(get_db),
) -> UserProfileResponse:
    """
    Update user profile.

    **Headers**:
    - Authorization: Bearer <session_token>

    **Request Body**:
    - name: (Optional) User display name
    - os: (Optional) Operating system
    - gpu: (Optional) GPU model
    - experience_level: (Optional) Skill level
    - robotics_background: (Optional) Robotics experience

    **Responses**:
    - 200: Profile updated
    - 401: Invalid token
    - 404: User not found
    - 500: Server error

    **Example**:
    ```bash
    curl -X PATCH http://localhost:8000/api/auth/profile \
      -H "Authorization: Bearer eyJhbGciOiJIUzI1NiIs..." \
      -H "Content-Type: application/json" \
      -d '{
        "experience_level": "advanced",
        "gpu": "nvidia-rtx-4090"
      }'
    ```
    """
    try:
        user_id = await get_current_user_id(authorization, db)

        update_data = request.dict(exclude_unset=True)
        updated_user = UserService.update_user(db, UUID(user_id), **update_data)

        if not updated_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found",
            )

        profile = UserService.get_user_profile(db, UUID(user_id))
        return UserProfileResponse(**profile)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Update profile error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Profile update failed",
        )
