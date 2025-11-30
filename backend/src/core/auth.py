"""
Authentication utilities for Better Auth integration.

This module provides functions to:
- Validate Better Auth sessions
- Extract user information from requests
- Query user data from the shared database
"""

from typing import Optional
import httpx
from fastapi import HTTPException, status, Header
from ..core.config import settings
from ..core.logging import logger


async def get_user_from_session(session_token: Optional[str] = None) -> Optional[dict]:
    """
    Validate Better Auth session and return user information.
    
    Args:
        session_token: Better Auth session token from cookie or Authorization header
        
    Returns:
        User dict with id, email, name, etc. or None if not authenticated
    """
    if not session_token:
        return None
    
    if not settings.BETTER_AUTH_SERVICE_URL:
        logger.warning("BETTER_AUTH_SERVICE_URL not configured, skipping auth validation")
        return None
    
    try:
        # Call Better Auth's get-session endpoint
        async with httpx.AsyncClient() as client:
            # Better Auth expects the cookie in a specific format
            cookie_header = f"better-auth.session_token={session_token}"
            response = await client.get(
                f"{settings.BETTER_AUTH_SERVICE_URL}/api/auth/get-session",
                headers={"Cookie": cookie_header},
                timeout=5.0,
                follow_redirects=True
            )
            
            if response.status_code == 200:
                try:
                    data = response.json()
                    if data and isinstance(data, dict) and data.get("user"):
                        return data["user"]
                    else:
                        logger.warning(f"Invalid session response format: {data}")
                        return None
                except Exception as json_error:
                    logger.error(f"Error parsing JSON response from Better Auth: {json_error}")
                    logger.debug(f"Response text: {response.text[:200]}")
                    return None
            elif response.status_code == 401:
                logger.debug("Better Auth returned 401 - session invalid")
                return None
            else:
                logger.warning(f"Unexpected response from Better Auth: {response.status_code}")
                logger.debug(f"Response text: {response.text[:200]}")
                return None
                
    except Exception as e:
        logger.error(f"Error validating session with Better Auth: {e}", exc_info=True)
        return None


async def get_user_id_from_request(
    authorization: Optional[str] = Header(None),
    cookie: Optional[str] = None
) -> Optional[str]:
    """
    Extract user ID from request headers or cookies.
    
    Args:
        authorization: Authorization header (Bearer token or session token)
        cookie: Cookie header containing Better Auth session
        
    Returns:
        User ID string or None if not authenticated
    """
    session_token = None
    
    # Try to get token from Authorization header
    if authorization:
        if authorization.startswith("Bearer "):
            session_token = authorization[7:]
        elif authorization.startswith("Session "):
            session_token = authorization[8:]
    
    # Try to get token from cookie header
    if not session_token and cookie:
        # Parse cookie header for better-auth.session_token
        # Cookie header format: "cookie1=value1; cookie2=value2; better-auth.session_token=token"
        import urllib.parse
        for part in cookie.split(";"):
            part = part.strip()
            if part.startswith("better-auth.session_token="):
                session_token = part.split("=", 1)[1]
                # URL decode if needed (handles %3D for =, etc.)
                session_token = urllib.parse.unquote(session_token)
                break
    
    if not session_token:
        return None
    
    user = await get_user_from_session(session_token)
    return user.get("id") if user else None


def require_auth(user_id: Optional[str]) -> str:
    """
    Require authentication - raise 401 if user_id is None.
    
    Args:
        user_id: User ID from request (may be None)
        
    Returns:
        User ID string
        
    Raises:
        HTTPException: 401 if not authenticated
    """
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )
    return user_id
