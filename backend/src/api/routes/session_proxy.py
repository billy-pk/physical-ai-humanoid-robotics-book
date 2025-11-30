"""
Session proxy endpoint for validating Better Auth sessions.

This endpoint receives a session token from the frontend and validates it
with Better Auth, returning the user information if valid.
"""

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
import httpx
from ...core.config import settings
from ...core.logging import logger

router = APIRouter(prefix="/api/session", tags=["session"])


class SessionValidateRequest(BaseModel):
    """Request to validate a session token."""
    token: str


class SessionValidateResponse(BaseModel):
    """Response with validated user information."""
    user_id: str
    email: str
    name: str | None = None


@router.post("/validate", response_model=SessionValidateResponse)
async def validate_session(request: SessionValidateRequest):
    """
    Validate a Better Auth session token.
    
    This endpoint acts as a proxy to Better Auth's get-session endpoint,
    allowing the frontend to validate sessions across different ports.
    
    Args:
        request: Contains the session token to validate
        
    Returns:
        User information if session is valid
        
    Raises:
        401: If session is invalid or expired
    """
    if not settings.BETTER_AUTH_SERVICE_URL:
        logger.error("BETTER_AUTH_SERVICE_URL not configured")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Authentication service not configured"
        )
    
    try:
        # Call Better Auth's get-session endpoint with the token
        async with httpx.AsyncClient() as client:
            cookie_header = f"better-auth.session_token={request.token}"
            
            response = await client.get(
                f"{settings.BETTER_AUTH_SERVICE_URL}/api/auth/get-session",
                headers={"Cookie": cookie_header},
                timeout=5.0
            )
            
            if response.status_code != 200:
                logger.warning(f"Better Auth returned {response.status_code}")
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Invalid or expired session"
                )
            
            data = response.json()
            
            if not data or not data.get("user"):
                logger.warning("Better Auth returned invalid session data")
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Invalid session"
                )
            
            user = data["user"]
            
            return SessionValidateResponse(
                user_id=user["id"],
                email=user["email"],
                name=user.get("name")
            )
            
    except httpx.RequestError as e:
        logger.error(f"Failed to connect to Better Auth: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Authentication service unavailable"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error validating session: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to validate session"
        )

