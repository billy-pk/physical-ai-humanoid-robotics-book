"""
User profile service for managing user profiles and background questionnaires.

This service handles:
- Creating and updating user profiles
- Saving questionnaire answers
- Retrieving user profile data
"""

from typing import Optional
from datetime import datetime
import psycopg
import json
import asyncio
from ..core.config import settings
from ..core.logging import logger
from ..models.user import UserProfile, UserProfileCreate, UserBackgroundQuestionnaire, PersonalizationPreferences


class UserProfileService:
    """Service for user profile operations."""
    
    @staticmethod
    async def get_profile(user_id: str) -> Optional[UserProfile]:
        """
        Get user profile by user ID.
        
        Args:
            user_id: User ID from Better Auth
            
        Returns:
            UserProfile if found, None otherwise
        """
        def _get_profile_sync():
            with psycopg.connect(settings.NEON_DATABASE_URL) as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT user_id, software_background, hardware_background, experience_level,
                               learning_goals, has_robotics_projects, robotics_projects_description,
                               programming_years, learning_style, questionnaire_completed,
                               questionnaire_completed_at, preferences, created_at, updated_at
                        FROM user_profiles
                        WHERE user_id = %s
                        """,
                        (user_id,)
                    )
                    row = cur.fetchone()
                    
                    if row:
                        # psycopg returns JSONB columns as Python objects (dict/list), not JSON strings
                        # So we don't need to json.loads() them
                        def parse_jsonb(value):
                            if value is None:
                                return None
                            if isinstance(value, (dict, list)):
                                return value  # Already parsed by psycopg
                            if isinstance(value, str):
                                return json.loads(value)  # If it's a string, parse it
                            return value
                        
                        return UserProfile(
                            user_id=row[0],
                            software_background=parse_jsonb(row[1]),
                            hardware_background=parse_jsonb(row[2]),
                            experience_level=row[3],
                            learning_goals=row[4],
                            has_robotics_projects=row[5],
                            robotics_projects_description=row[6],
                            programming_years=row[7],
                            learning_style=row[8],
                            questionnaire_completed=row[9],
                            questionnaire_completed_at=row[10],
                            preferences=parse_jsonb(row[11]),
                            created_at=row[12],
                            updated_at=row[13]
                        )
                    return None
        
        try:
            return await asyncio.to_thread(_get_profile_sync)
        except Exception as e:
            logger.error(f"Error getting user profile: {e}", user_id=user_id)
            raise
    
    @staticmethod
    async def create_or_update_profile(user_id: str, profile_data: UserProfileCreate) -> UserProfile:
        """
        Create or update user profile.
        
        Args:
            user_id: User ID from Better Auth
            profile_data: Profile data to save
            
        Returns:
            Updated UserProfile
        """
        def _create_or_update_sync():
            with psycopg.connect(settings.NEON_DATABASE_URL) as conn:
                with conn.cursor() as cur:
                    # Check if profile exists
                    cur.execute("SELECT user_id FROM user_profiles WHERE user_id = %s", (user_id,))
                    exists = cur.fetchone()
                    
                    now = datetime.utcnow()
                    completed_at = now if profile_data.experience_level else None
                    
                    if exists:
                        # Update existing profile
                        cur.execute(
                            """
                            UPDATE user_profiles
                            SET software_background = %s,
                                hardware_background = %s,
                                experience_level = %s,
                                learning_goals = %s,
                                has_robotics_projects = %s,
                                robotics_projects_description = %s,
                                programming_years = %s,
                                learning_style = %s,
                                preferences = %s,
                                questionnaire_completed = %s,
                                questionnaire_completed_at = %s,
                                updated_at = %s
                            WHERE user_id = %s
                            """,
                            (
                                json.dumps(profile_data.software_background) if profile_data.software_background else None,
                                json.dumps(profile_data.hardware_background) if profile_data.hardware_background else None,
                                profile_data.experience_level,
                                profile_data.learning_goals,
                                profile_data.has_robotics_projects,
                                profile_data.robotics_projects_description,
                                profile_data.programming_years,
                                profile_data.learning_style,
                                json.dumps(profile_data.preferences) if profile_data.preferences else None,
                                bool(profile_data.experience_level),  # Completed if experience_level is set
                                completed_at,
                                now,
                                user_id
                            )
                        )
                    else:
                        # Create new profile
                        cur.execute(
                            """
                            INSERT INTO user_profiles (
                                user_id, software_background, hardware_background, experience_level,
                                learning_goals, has_robotics_projects, robotics_projects_description,
                                programming_years, learning_style, preferences, questionnaire_completed,
                                questionnaire_completed_at, created_at, updated_at
                            )
                            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                            """,
                            (
                                user_id,
                                json.dumps(profile_data.software_background) if profile_data.software_background else None,
                                json.dumps(profile_data.hardware_background) if profile_data.hardware_background else None,
                                profile_data.experience_level,
                                profile_data.learning_goals,
                                profile_data.has_robotics_projects,
                                profile_data.robotics_projects_description,
                                profile_data.programming_years,
                                profile_data.learning_style,
                                json.dumps(profile_data.preferences) if profile_data.preferences else None,
                                bool(profile_data.experience_level),
                                completed_at,
                                now,
                                now
                            )
                        )
                    
                    conn.commit()
        
        try:
            await asyncio.to_thread(_create_or_update_sync)
            # Return updated profile
            return await UserProfileService.get_profile(user_id)
        except Exception as e:
            logger.error(f"Error creating/updating user profile: {e}", user_id=user_id)
            raise

    @staticmethod
    async def get_personalization_preferences(user_id: str) -> Optional[PersonalizationPreferences]:
        """
        Get user's personalization preferences by user ID.
        
        If preferences JSONB column is null but profile has questionnaire data,
        creates basic preferences from questionnaire data.

        Args:
            user_id: User ID from Better Auth

        Returns:
            PersonalizationPreferences if found or can be created from questionnaire, None otherwise
        """
        profile = await UserProfileService.get_profile(user_id)
        if not profile:
            return None
            
        # If preferences JSONB column exists and is valid, use it
        if profile.preferences:
            try:
                return PersonalizationPreferences.model_validate(profile.preferences)
            except Exception as e:
                logger.error(f"Error validating personalization preferences for user {user_id}: {e}")
                # Fall through to create from questionnaire data
        
        # If preferences JSONB is null but we have questionnaire data, create basic preferences
        if profile.experience_level and profile.learning_goals:
            logger.info(f"Creating preferences from questionnaire data for user {user_id}")
            try:
                # Create basic preferences from questionnaire data
                # Use default values for missing fields
                basic_preferences = PersonalizationPreferences(
                    experience_level=profile.experience_level,
                    learning_topics=["Physical AI", "Humanoid Robotics"],  # Default topics
                    learning_goals=profile.learning_goals,
                    content_mode="full",  # Default to full content
                    urdu_translation_enabled=False,
                    preferences_version=1
                )
                
                # Save these preferences back to the database
                await UserProfileService.update_personalization_preferences(user_id, basic_preferences)
                
                return basic_preferences
            except Exception as e:
                logger.error(f"Error creating preferences from questionnaire data for user {user_id}: {e}")
                return None
        
        return None

    @staticmethod
    async def update_personalization_preferences(user_id: str,
                                                 preferences: PersonalizationPreferences) -> UserProfile:
        """
        Update user's personalization preferences.

        Args:
            user_id: User ID from Better Auth
            preferences: PersonalizationPreferences object with updated data

        Returns:
            Updated UserProfile
        """
        profile = await UserProfileService.get_profile(user_id)
        if not profile:
            # If no profile exists, create a basic one before updating preferences
            # This handles cases where a user might directly hit the preference update
            # without a full profile existing yet (e.g., first login experience)
            profile_data = UserProfileCreate(
                experience_level=preferences.experience_level,
                learning_goals=preferences.learning_goals,
                preferences=preferences.model_dump(mode='json')
            )
            profile = await UserProfileService.create_or_update_profile(user_id, profile_data)
            # Fetch again to ensure all fields are loaded
            profile = await UserProfileService.get_profile(user_id)
            if not profile:
                raise ValueError("Failed to create or retrieve user profile.")


        # Update preferences with version increment and timestamp
        current_preferences_dict = preferences.model_dump(mode='json')
        # Increment version: if profile.preferences exists and has a version, use it; otherwise, start at 1
        current_preferences_dict['preferences_version'] = (profile.preferences.get('preferences_version', 0) if profile.preferences else 0) + 1
        current_preferences_dict['preferences_last_updated_at'] = datetime.utcnow().isoformat()
        
        # Set preferences_submitted_at only if it's the first submission
        if not profile.preferences or 'preferences_submitted_at' not in profile.preferences or not profile.preferences['preferences_submitted_at']:
            current_preferences_dict['preferences_submitted_at'] = datetime.utcnow().isoformat()


        profile_update_data = UserProfileCreate(
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            experience_level=current_preferences_dict.get('experience_level'), # Use new experience level
            learning_goals=current_preferences_dict.get('learning_goals'), # Use new learning goals
            has_robotics_projects=profile.has_robotics_projects,
            robotics_projects_description=profile.robotics_projects_description,
            programming_years=profile.programming_years,
            learning_style=profile.learning_style,
            preferences=current_preferences_dict # Assign the updated preferences dict here
        )
        
        # Note: questionnaire_completed and questionnaire_completed_at are automatically
        # set by create_or_update_profile based on experience_level, so we don't need to set them here

        updated_profile = await UserProfileService.create_or_update_profile(user_id, profile_update_data)
        return updated_profile

