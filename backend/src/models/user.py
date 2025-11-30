"""
User models for authentication and profile management.

These models represent user data stored in the database:
- User (from Better Auth's user table)
- UserProfile (from our user_profiles table)
- UserBackgroundQuestionnaire (from user_background_questionnaire table)
"""

from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid


class UserProfile(BaseModel):
    """User profile model with background questionnaire data."""
    user_id: str = Field(..., description="User ID from Better Auth (text)")
    software_background: Optional[List[str]] = Field(None, description="List of software skills")
    hardware_background: Optional[List[str]] = Field(None, description="List of hardware skills")
    experience_level: Optional[str] = Field(None, description="beginner/intermediate/advanced")
    learning_goals: Optional[str] = Field(None, max_length=500, description="User's learning goals")
    has_robotics_projects: Optional[bool] = Field(None, description="Whether user has prior robotics projects")
    robotics_projects_description: Optional[str] = Field(None, description="Description of robotics projects")
    programming_years: Optional[int] = Field(None, ge=0, le=50, description="Years of programming experience")
    learning_style: Optional[str] = Field(None, description="visual/hands-on/theoretical/mixed")
    questionnaire_completed: bool = Field(default=False, description="Whether questionnaire is completed")
    questionnaire_completed_at: Optional[datetime] = Field(None, description="When questionnaire was completed")
    preferences: Optional[Dict[str, Any]] = Field(None, description="Additional user preferences")
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class UserBackgroundQuestionnaire(BaseModel):
    """Individual questionnaire answer entry."""
    id: uuid.UUID
    user_id: str = Field(..., description="User ID from Better Auth")
    question_id: str = Field(..., max_length=100, description="Question identifier")
    answer: Optional[Dict[str, Any]] = Field(None, description="Answer data (JSON)")
    answered_at: datetime

    class Config:
        from_attributes = True


class UserProfileCreate(BaseModel):
    """Request model for creating/updating user profile."""
    software_background: Optional[List[str]] = None
    hardware_background: Optional[List[str]] = None
    experience_level: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced)$")
    learning_goals: Optional[str] = Field(None, max_length=500)
    has_robotics_projects: Optional[bool] = None
    robotics_projects_description: Optional[str] = None
    programming_years: Optional[int] = Field(None, ge=0, le=50)
    learning_style: Optional[str] = Field(None, pattern="^(visual|hands-on|theoretical|mixed)$")
    preferences: Optional[Dict[str, Any]] = None


class UserProfileResponse(BaseModel):
    """Response model for user profile."""
    user_id: str
    software_background: Optional[List[str]] = None
    hardware_background: Optional[List[str]] = None
    experience_level: Optional[str] = None
    learning_goals: Optional[str] = None
    has_robotics_projects: Optional[bool] = None
    robotics_projects_description: Optional[str] = None
    programming_years: Optional[int] = None
    learning_style: Optional[str] = None
    questionnaire_completed: bool
    questionnaire_completed_at: Optional[datetime] = None
    preferences: Optional[Dict[str, Any]] = None
    created_at: datetime
    updated_at: datetime
