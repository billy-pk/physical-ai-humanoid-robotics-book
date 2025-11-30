"""
Authentication and user profile API routes.

Endpoints:
- GET /api/auth/profile - Get current user's profile
- POST /api/auth/profile/background - Save user background questionnaire
- PUT /api/auth/profile - Update user profile
"""

from fastapi import APIRouter, HTTPException, status, Request
from typing import Optional
from ...core.logging import logger
from ...core.auth import get_user_id_from_request
from ...models.user import UserProfileResponse, UserProfileCreate
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
