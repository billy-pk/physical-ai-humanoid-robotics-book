# User Preferences Database Table

## Table Name: `user_profiles`

User preferences are stored in the **`user_profiles`** table, specifically in the **`preferences`** column, which is a JSONB/JSON type column.

## Table Structure

The `user_profiles` table was created by the migration: `21c54f44e3e4_create_user_profiles.py`

```sql
CREATE TABLE user_profiles (
    user_id TEXT PRIMARY KEY,                    -- References user.id from Better Auth
    software_background JSON,                    -- Array of software skills
    hardware_background JSON,                    -- Array of hardware skills
    experience_level VARCHAR(20),                -- beginner/intermediate/advanced
    learning_goals TEXT,                         -- Free text, max 500 chars
    has_robotics_projects BOOLEAN,
    robotics_projects_description TEXT,
    programming_years INTEGER,                   -- 0-50
    learning_style VARCHAR(20),                  -- visual/hands-on/theoretical/mixed
    questionnaire_completed BOOLEAN DEFAULT false,
    questionnaire_completed_at TIMESTAMP WITH TIME ZONE,
    preferences JSON,                            -- ◄── USER PREFERENCES STORED HERE
    created_at TIMESTAMP WITH TIME ZONE DEFAULT now(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT now(),
    FOREIGN KEY (user_id) REFERENCES user(id) ON DELETE CASCADE
);
```

## Preferences JSON Structure

The `preferences` column stores a JSON object with the following structure:

```json
{
  "experience_level": "beginner|intermediate|advanced",
  "learning_topics": ["ROS 2", "Computer Vision", "Path Planning", ...],
  "learning_goals": "String up to 500 characters",
  "content_mode": "full|personalized",
  "urdu_translation_enabled": true|false,
  "preferences_submitted_at": "ISO 8601 timestamp",
  "preferences_last_updated_at": "ISO 8601 timestamp",
  "preferences_version": 1
}
```

## Example Query

To view user preferences in PostgreSQL:

```sql
-- View preferences for a specific user
SELECT user_id, preferences 
FROM user_profiles 
WHERE user_id = 'your-user-id';

-- View all users with preferences
SELECT user_id, preferences 
FROM user_profiles 
WHERE preferences IS NOT NULL;

-- Query specific preference fields (PostgreSQL JSONB syntax)
SELECT 
    user_id,
    preferences->>'experience_level' AS experience_level,
    preferences->>'content_mode' AS content_mode,
    preferences->'learning_topics' AS learning_topics
FROM user_profiles
WHERE preferences IS NOT NULL;
```

## Related Files

- **Migration**: `backend/alembic/versions/21c54f44e3e4_create_user_profiles.py`
- **Model**: `backend/src/models/user.py` (PersonalizationPreferences class)
- **Service**: `backend/src/services/user_profile.py` (update_personalization_preferences method)
- **API Route**: `backend/src/api/routes/auth.py` (`/api/auth/profile/preferences` endpoint)

## Notes

1. The `preferences` column is nullable - users don't have to set preferences
2. Preferences are stored as JSONB in PostgreSQL (allows JSON querying and indexing)
3. The table has a foreign key constraint linking to the `user` table (Better Auth)
4. The `user_id` is the primary key, ensuring one profile per user

