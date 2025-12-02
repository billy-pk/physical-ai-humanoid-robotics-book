"""
Authentication utilities for Better Auth integration.

This module provides functions to:
- Validate Better Auth sessions
- Extract user information from requests
- Query user data from the shared database
"""

from typing import Optional, TypedDict
from fastapi import HTTPException, status, Header, Request, Depends
import httpx
from ..core.config import settings
from ..core.logging import logger


class User(TypedDict):
    """User type from Better Auth session."""
    id: str
    email: str
    name: Optional[str]


async def get_user_from_session(session_token: Optional[str] = None) -> Optional[dict]:
    """
    Validate Better Auth session and return user information.
    
    This function queries the session table directly from the shared database
    instead of calling Better Auth's API, which is more reliable for server-to-server requests.
    
    Args:
        session_token: Better Auth session token from cookie or Authorization header
        
    Returns:
        User dict with id, email, name, etc. or None if not authenticated
    """
    logger.info(f"get_user_from_session called with token: {session_token[:20] if session_token else 'None'}...")
    
    if not session_token:
        logger.warning("No session token provided")
        return None
    
    try:
        # Query the session table directly from the shared database
        from datetime import datetime, timezone
        from sqlalchemy import text
        from ..database import async_session_maker
        
        now = datetime.now(timezone.utc)
        logger.info(f"Validating session token: {session_token[:20]}..., current time: {now}")
        
        async with async_session_maker() as session:
            # Query session table to validate token and get user
            # Use simple parameter binding
            query = text("""
                SELECT 
                    s."userId",
                    s."expiresAt",
                    u.id,
                    u.email,
                    u.name,
                    u."emailVerified",
                    u.image,
                    u."createdAt",
                    u."updatedAt"
                FROM session s
                INNER JOIN "user" u ON u.id = s."userId"
                WHERE s.token = :token
                AND s."expiresAt" > :now
            """)
            
            logger.debug(f"Executing session query with token: {session_token[:20]}...")
            result = await session.execute(
                query,
                {"token": session_token, "now": now}
            )
            row = result.fetchone()
            
            if row:
                logger.info(f"Session validated for user: {row.id}, email: {row.email}")
                return {
                    "id": row.id,
                    "email": row.email,
                    "name": row.name,
                    "emailVerified": row.emailVerified,
                    "image": row.image,
                    "createdAt": row.createdAt.isoformat() if row.createdAt else None,
                    "updatedAt": row.updatedAt.isoformat() if row.updatedAt else None,
                }
            else:
                logger.warning(f"Session not found or expired for token: {session_token[:20]}...")
                # Try to see if session exists but expired
                check_query = text("""
                    SELECT s.token, s."expiresAt", s."userId"
                    FROM session s
                    WHERE s.token = :token
                """)
                check_result = await session.execute(check_query, {"token": session_token})
                check_row = check_result.fetchone()
                if check_row:
                    logger.warning(f"Session exists but expired. ExpiresAt: {check_row.expiresAt}, Now: {now}")
                else:
                    logger.warning("Session token not found in database")
                return None
                
    except Exception as e:
        logger.error(f"Error validating session from database: {e}", exc_info=True)
        # Fallback to Better Auth API if database query fails
        logger.info("Falling back to Better Auth API validation")
        return await _get_user_from_better_auth_api(session_token)


async def _get_user_from_better_auth_api(session_token: Optional[str] = None) -> Optional[dict]:
    """
    Fallback: Validate session via Better Auth API.
    
    This is used as a fallback if direct database query fails.
    """
    if not settings.BETTER_AUTH_SERVICE_URL:
        logger.warning("BETTER_AUTH_SERVICE_URL not configured")
        return None
    
    try:
        async with httpx.AsyncClient() as client:
            cookie_header = f"better-auth.session_token={session_token}"
            url = f"{settings.BETTER_AUTH_SERVICE_URL}/api/auth/get-session"
            
            response = await client.get(
                url,
                headers={"Cookie": cookie_header},
                timeout=5.0,
                follow_redirects=True
            )
            
            if response.status_code == 200:
                data = response.json()
                if data and isinstance(data, dict) and data.get("user"):
                    return data["user"]
            return None
                
    except Exception as e:
        logger.error(f"Error validating session with Better Auth API: {e}", exc_info=True)
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
    logger.info(f"get_user_id_from_request called")
    logger.info(f"Authorization header: {authorization[:50] if authorization else 'None'}...")
    logger.info(f"Cookie header: {cookie[:100] if cookie else 'None'}...")
    
    session_token = None
    
    # Try to get token from Authorization header
    if authorization:
        if authorization.startswith("Bearer "):
            session_token = authorization[7:]
            logger.info(f"Extracted Bearer token: {session_token[:20]}...")
        elif authorization.startswith("Session "):
            session_token = authorization[8:]
            logger.info(f"Extracted Session token: {session_token[:20]}...")
    
    # Try to get token from cookie header
    if not session_token and cookie:
        # Parse cookie header for better-auth.session_token
        # Cookie header format: "cookie1=value1; cookie2=value2; better-auth.session_token=token"
        import urllib.parse
        logger.info(f"Parsing cookie header for session token")
        for part in cookie.split(";"):
            part = part.strip()
            if part.startswith("better-auth.session_token="):
                session_token = part.split("=", 1)[1]
                # URL decode if needed (handles %3D for =, etc.)
                session_token = urllib.parse.unquote(session_token)
                logger.info(f"Found session token in cookie: {session_token[:50]}...")
                break
        if not session_token:
            logger.warning("Cookie header present but no better-auth.session_token found")
    
    if not session_token:
        logger.warning("No session token found in request")
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


async def get_current_active_user(request: Request) -> User:
    """
    FastAPI dependency to get the current authenticated user.
    
    Args:
        request: FastAPI Request object
        
    Returns:
        User dict with id, email, name, etc.
        
    Raises:
        HTTPException: 401 if not authenticated
    """
    # Get session token from request
    cookie_header = request.headers.get("cookie", "")
    # Try both lowercase and original case for authorization header
    authorization_header = request.headers.get("authorization") or request.headers.get("Authorization")
    
    logger.info(f"get_current_active_user - Authorization header: {authorization_header[:50] if authorization_header else 'None'}...")
    logger.info(f"get_current_active_user - Cookie header: {cookie_header[:100] if cookie_header else 'None'}...")
    
    # Also check request.cookies dict (FastAPI's cookie parser)
    if not cookie_header:
        cookies_dict = dict(request.cookies)
        logger.info(f"get_current_active_user - Request cookies dict: {list(cookies_dict.keys())}")
        if "better-auth.session_token" in cookies_dict:
            cookie_header = f"better-auth.session_token={cookies_dict['better-auth.session_token']}"
    
    # Extract session token
    session_token = None
    if authorization_header:
        if authorization_header.startswith("Bearer "):
            session_token = authorization_header[7:]
            logger.info(f"get_current_active_user - Extracted Bearer token: {session_token[:20]}...")
        elif authorization_header.startswith("Session "):
            session_token = authorization_header[8:]
            logger.info(f"get_current_active_user - Extracted Session token: {session_token[:20]}...")
    
    if not session_token and cookie_header:
        import urllib.parse
        for part in cookie_header.split(";"):
            part = part.strip()
            if part.startswith("better-auth.session_token="):
                session_token = part.split("=", 1)[1]
                session_token = urllib.parse.unquote(session_token)
                logger.info(f"get_current_active_user - Extracted token from cookie: {session_token[:20]}...")
                break
    
    if not session_token:
        logger.warning("get_current_active_user - No session token found in request")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required"
        )
    
    logger.info(f"get_current_active_user - Validating session token with Better Auth...")
    user = await get_user_from_session(session_token)
    if not user:
        logger.warning("get_current_active_user - Session validation failed")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session"
        )
    
    logger.info(f"get_current_active_user - User authenticated: {user.get('id')}")
    # Return user dict (TypedDict is just for type checking)
    return {
        "id": user.get("id", ""),
        "email": user.get("email", ""),
        "name": user.get("name")
    }
