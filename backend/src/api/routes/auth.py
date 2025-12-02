"""
Authentication and user profile API routes.

Endpoints:
- GET /api/auth/profile - Get current user's profile
- POST /api/auth/profile/background - Save user background questionnaire
- PUT /api/auth/profile - Update user profile
"""

from fastapi import APIRouter, HTTPException, status, Request, Depends
from typing import Dict, Any, List
from ...core.logging import logger
from ...core.auth import get_user_id_from_request, get_current_active_user, User
from ...models.user import UserProfileResponse, UserProfileCreate, PersonalizationPreferences
from ...services.user_profile import UserProfileService

router = APIRouter(prefix="/api/auth", tags=["auth"])


@router.get("/profile", response_model=UserProfileResponse, status_code=status.HTTP_200_OK)
async def get_profile(request: Request):
    """
    Get current user's profile.
    
    Requires authentication via Better Auth session token.
    """
    try:
        # Get user ID from request (cookie or Authorization header)
        # FastAPI's Request object provides cookies via request.cookies dict
        # But Better Auth uses a cookie header, so we check both
        cookie_header = request.headers.get("cookie", "")
        authorization_header = request.headers.get("authorization")
        
        # Also check request.cookies dict (FastAPI's cookie parser)
        if not cookie_header:
            cookies_dict = dict(request.cookies)
            if "better-auth.session_token" in cookies_dict:
                cookie_header = f"better-auth.session_token={cookies_dict['better-auth.session_token']}"
        
        user_id = await get_user_id_from_request(
            authorization=authorization_header,
            cookie=cookie_header
        )
        
        if not user_id:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Authentication required"
            )
        
        # Get profile
        profile = await UserProfileService.get_profile(user_id)
        
        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Profile not found"
            )
        
        return UserProfileResponse(
            user_id=profile.user_id,
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            experience_level=profile.experience_level,
            learning_goals=profile.learning_goals,
            has_robotics_projects=profile.has_robotics_projects,
            robotics_projects_description=profile.robotics_projects_description,
            programming_years=profile.programming_years,
            learning_style=profile.learning_style,
            questionnaire_completed=profile.questionnaire_completed,
            questionnaire_completed_at=profile.questionnaire_completed_at,
            preferences=profile.preferences,
            created_at=profile.created_at,
            updated_at=profile.updated_at
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting user profile: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve profile"
        )


@router.get("/profile/preferences", response_model=PersonalizationPreferences, summary="Get user personalization preferences")
async def get_user_preferences(
    user: User = Depends(get_current_active_user),
) -> PersonalizationPreferences:
    """
    Retrieve the current user's personalization preferences.
    """
    preferences = await UserProfileService.get_personalization_preferences(user["id"])
    if not preferences:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Personalization preferences not found for this user."
        )
    return preferences


@router.post("/profile/preferences", response_model=PersonalizationPreferences, summary="Update user personalization preferences")
async def update_user_preferences(
    preferences: PersonalizationPreferences,
    user: User = Depends(get_current_active_user),
) -> PersonalizationPreferences:
    """
    Update the current user's personalization preferences.
    """
    try:
        updated_profile = await UserProfileService.update_personalization_preferences(user["id"], preferences)
        if not updated_profile.preferences:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to retrieve updated preferences after update."
            )
        # Re-validate the stored preferences against the Pydantic model to ensure consistency
        return PersonalizationPreferences.model_validate(updated_profile.preferences)
    except Exception as e:
        logger.error(f"Error updating personalization preferences for user {user['id']}: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update personalization preferences: {e}"
        )



@router.post("/profile/background", response_model=UserProfileResponse, status_code=status.HTTP_200_OK)
async def save_background_questionnaire(
    profile_data: UserProfileCreate,
    request: Request
):
    """
    Save user background questionnaire answers.
    
    This endpoint accepts questionnaire data and creates/updates the user profile.
    Requires authentication via Better Auth session token.
    """
    try:
        # Get user ID from request
        # Try multiple authentication methods:
        # 1. Better Auth cookie (if request came from same domain as Better Auth)
        # 2. X-Session-Token header (for cross-origin requests where we manually pass the signed cookie)
        # 3. Authorization Bearer header (for API tokens)
        
        authorization_header = request.headers.get("authorization")
        cookie_header = request.headers.get("cookie", "")
        session_token_header = request.headers.get("x-session-token")  # For passing signed cookie value
        
        logger.info(f"Profile background request - Auth: {bool(authorization_header)}, Cookie: {bool(cookie_header)}, X-Session-Token: {bool(session_token_header)}")
        
        # If we have X-Session-Token header, use that (it contains the signed cookie value)
        if session_token_header and not cookie_header:
            cookie_header = f"better-auth.session_token={session_token_header}"
            logger.info(f"Using X-Session-Token header as cookie: {session_token_header[:30]}...")
        
        # Check request.cookies dict as fallback
        if not cookie_header and not authorization_header and not session_token_header:
            cookies_dict = dict(request.cookies)
            logger.info(f"Request cookies dict: {list(cookies_dict.keys())}")
            if "better-auth.session_token" in cookies_dict:
                cookie_header = f"better-auth.session_token={cookies_dict['better-auth.session_token']}"
                logger.info("Using session token from request.cookies")
        
        user_id = await get_user_id_from_request(
            authorization=authorization_header,
            cookie=cookie_header
        )
        
        logger.info(f"Retrieved user_id: {user_id}")
        
        if not user_id:
            logger.warning("Authentication failed - no valid user_id retrieved")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Authentication required"
            )
        
        # Validate required fields
        if not profile_data.experience_level:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="experience_level is required"
            )
        
        if profile_data.has_robotics_projects and not profile_data.robotics_projects_description:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="robotics_projects_description is required when has_robotics_projects is true"
            )
        
        # Save profile
        profile = await UserProfileService.create_or_update_profile(user_id, profile_data)
        
        return UserProfileResponse(
            user_id=profile.user_id,
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            experience_level=profile.experience_level,
            learning_goals=profile.learning_goals,
            has_robotics_projects=profile.has_robotics_projects,
            robotics_projects_description=profile.robotics_projects_description,
            programming_years=profile.programming_years,
            learning_style=profile.learning_style,
            questionnaire_completed=profile.questionnaire_completed,
            questionnaire_completed_at=profile.questionnaire_completed_at,
            preferences=profile.preferences,
            created_at=profile.created_at,
            updated_at=profile.updated_at
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error saving background questionnaire: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to save questionnaire"
        )


@router.put("/profile", response_model=UserProfileResponse, status_code=status.HTTP_200_OK)
async def update_profile(
    profile_data: UserProfileCreate,
    request: Request
):
    """
    Update user profile.
    
    Requires authentication via Better Auth session token.
    """
    try:
        # Get user ID from request
        # Check both cookie header and FastAPI's cookies dict
        cookie_header = request.headers.get("cookie", "")
        authorization_header = request.headers.get("authorization")
        
        # Also check request.cookies dict (FastAPI's cookie parser)
        if not cookie_header:
            cookies_dict = dict(request.cookies)
            if "better-auth.session_token" in cookies_dict:
                cookie_header = f"better-auth.session_token={cookies_dict['better-auth.session_token']}"
        
        user_id = await get_user_id_from_request(
            authorization=authorization_header,
            cookie=cookie_header
        )
        
        if not user_id:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Authentication required"
            )
        
        # Update profile
        profile = await UserProfileService.create_or_update_profile(user_id, profile_data)
        
        return UserProfileResponse(
            user_id=profile.user_id,
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            experience_level=profile.experience_level,
            learning_goals=profile.learning_goals,
            has_robotics_projects=profile.has_robotics_projects,
            robotics_projects_description=profile.robotics_projects_description,
            programming_years=profile.programming_years,
            learning_style=profile.learning_style,
            questionnaire_completed=profile.questionnaire_completed,
            questionnaire_completed_at=profile.questionnaire_completed_at,
            preferences=profile.preferences,
            created_at=profile.created_at,
            updated_at=profile.updated_at
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating profile: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update profile"
        )
