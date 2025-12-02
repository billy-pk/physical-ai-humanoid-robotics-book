# Data Model: User Personalization Module

**Phase**: 1 - Design & Contracts
**Date**: 2025-12-01
**Purpose**: Define entities, relationships, and data structures for personalization feature

## Overview

This document defines the data model for the user personalization module, including preference storage, content caching, and relationships with existing entities. The design extends the existing `user_profiles` table and adds a new caching table.

## Entity Relationship Diagram

```
┌─────────────────────┐
│      user           │  (Existing - Better Auth)
│─────────────────────│
│ id: TEXT PK         │
│ email: TEXT         │
│ name: TEXT          │
│ created_at          │
└──────────┬──────────┘
           │
           │ 1:1
           │
┌──────────▼──────────────────────────────────┐
│      user_profiles                          │  (Existing - Extended)
│─────────────────────────────────────────────│
│ user_id: TEXT PK FK                         │
│ software_background: JSONB                  │
│ hardware_background: JSONB                  │
│ experience_level: VARCHAR(20)               │
│ learning_goals: TEXT                        │
│ has_robotics_projects: BOOLEAN              │
│ robotics_projects_description: TEXT         │
│ programming_years: INTEGER                  │
│ learning_style: VARCHAR(20)                 │
│ questionnaire_completed: BOOLEAN            │
│ questionnaire_completed_at: TIMESTAMP       │
│ preferences: JSONB  ◄── EXTENDED            │
│   ├─ experience_level: string               │
│   ├─ learning_topics: string[]              │
│   ├─ learning_goals: string                 │
│   ├─ content_mode: string                   │
│   ├─ urdu_translation_enabled: boolean      │
│   ├─ preferences_submitted_at: timestamp    │
│   ├─ preferences_last_updated_at: timestamp │
│   └─ preferences_version: integer           │
│ created_at: TIMESTAMP                       │
│ updated_at: TIMESTAMP                       │
└──────────┬──────────────────────────────────┘
           │
           │ 1:N
           │
┌──────────▼──────────────────────────────────┐
│  personalized_content_cache                 │  (NEW)
│─────────────────────────────────────────────│
│ cache_id: UUID PK                           │
│ user_id: TEXT FK                            │
│ chapter_id: VARCHAR(255)                    │
│ content_type: VARCHAR(20)                   │
│ user_preferences_hash: VARCHAR(64)          │
│ chapter_content_hash: VARCHAR(64)           │
│ target_language: VARCHAR(10) NULL           │
│ generated_content: TEXT                     │
│ generation_metadata: JSONB                  │
│ created_at: TIMESTAMP                       │
│ last_accessed_at: TIMESTAMP                 │
│ access_count: INTEGER                       │
│ UNIQUE(user_id, chapter_id, content_type,  │
│        user_preferences_hash,               │
│        chapter_content_hash, target_language)│
└─────────────────────────────────────────────┘
```

## Entities

### 1. UserProfile (Extended Existing Entity)

**Description**: Extended existing user profile entity to include personalization preferences in the `preferences` JSONB column.

**Table**: `user_profiles` (existing table)

**Primary Key**: `user_id` (TEXT, references `user.id`)

**New/Modified Fields**:

| Field Name | Type | Constraints | Description |
|------------|------|-------------|-------------|
| `preferences` | JSONB | Nullable | **EXTENDED**: Now contains personalization preferences object |

**Preferences JSONB Structure**:
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

**Field Validation Rules**:
- `experience_level`: Must be one of: "beginner", "intermediate", "advanced"
- `learning_topics`: Array of strings, each 1-50 characters, max 10 topics
- `learning_goals`: String, max 500 characters
- `content_mode`: Must be one of: "full", "personalized"
- `urdu_translation_enabled`: Boolean
- `preferences_version`: Positive integer, incremented on each update

**Relationships**:
- Belongs to `user` (Better Auth) via `user_id`
- Has many `personalized_content_cache` entries

**Business Rules**:
- Preferences are optional; default to `null` if not provided
- When preferences are updated, `preferences_last_updated_at` must be set to current timestamp
- `preferences_version` must increment on each update for cache invalidation
- If `content_mode` is "full", personalization service is bypassed

---

### 2. PersonalizedContentCache (New Entity)

**Description**: Caches personalized and translated content to reduce API costs and latency. Keyed by user ID, chapter ID, content type, and hashes of preferences and source content.

**Table**: `personalized_content_cache` (new table)

**Primary Key**: `cache_id` (UUID, auto-generated)

**Fields**:

| Field Name | Type | Constraints | Description |
|------------|------|-------------|-------------|
| `cache_id` | UUID | PRIMARY KEY, NOT NULL, DEFAULT gen_random_uuid() | Unique identifier for cache entry |
| `user_id` | TEXT | NOT NULL, FK to user.id | Owner of cached content |
| `chapter_id` | VARCHAR(255) | NOT NULL | Identifier of the chapter (e.g., "module-1/chapter-3") |
| `content_type` | VARCHAR(20) | NOT NULL | Type of cached content: "personalized" or "translated" |
| `user_preferences_hash` | VARCHAR(64) | NOT NULL | SHA256 hash of user preferences JSON (for cache key) |
| `chapter_content_hash` | VARCHAR(64) | NOT NULL | SHA256 hash of original chapter content (for invalidation) |
| `target_language` | VARCHAR(10) | NULLABLE | Target language code (e.g., "ur" for Urdu). NULL for personalized content |
| `generated_content` | TEXT | NOT NULL | The generated personalized or translated content (markdown) |
| `generation_metadata` | JSONB | NULLABLE | Metadata about generation: model, tokens_used, generation_time_ms, etc. |
| `created_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | When cache entry was created |
| `last_accessed_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last time cache was accessed (updated on read) |
| `access_count` | INTEGER | NOT NULL, DEFAULT 1 | Number of times cache has been accessed |

**Unique Constraint**:
```sql
UNIQUE(user_id, chapter_id, content_type, user_preferences_hash, chapter_content_hash, target_language)
```

**Indexes**:
```sql
CREATE INDEX idx_personalized_cache_lookup ON personalized_content_cache(
    user_id, chapter_id, content_type, user_preferences_hash
);
CREATE INDEX idx_personalized_cache_cleanup ON personalized_content_cache(last_accessed_at);
```

**Field Validation Rules**:
- `content_type`: Must be "personalized" or "translated"
- `target_language`: Required if `content_type` is "translated", NULL otherwise
- `user_preferences_hash`: 64-character hex string (SHA256)
- `chapter_content_hash`: 64-character hex string (SHA256)
- `generated_content`: Non-empty string

**Relationships**:
- Belongs to `user` via `user_id`
- Indirectly related to `user_profiles` via `user_id`

**Business Rules**:
- Cache is automatically invalidated when either `user_preferences_hash` or `chapter_content_hash` changes (natural cache miss via unique constraint)
- `last_accessed_at` is updated on every read to track cache freshness
- Entries not accessed for 90+ days are candidates for cleanup (configurable retention policy)
- `generation_metadata` stores diagnostic info:
  ```json
  {
    "model": "gpt-4o-mini",
    "tokens_used": 4500,
    "generation_time_ms": 8234,
    "generated_at": "2025-12-01T10:30:00Z",
    "api_version": "v1"
  }
  ```

---

## Data Access Patterns

### 1. Fetch User Preferences

**Use Case**: Load user personalization settings when rendering content or generating personalized views.

**Query**:
```sql
SELECT preferences
FROM user_profiles
WHERE user_id = $1;
```

**Expected Result**: JSONB object with personalization preferences or NULL

**Frequency**: High (every page load for authenticated users)

**Caching Strategy**: Cache in-memory on backend with 5-minute TTL (similar to existing session cache pattern)

---

### 2. Update User Preferences

**Use Case**: User submits or updates their personalization preferences.

**Query**:
```sql
UPDATE user_profiles
SET
    preferences = $2,
    updated_at = NOW()
WHERE user_id = $1
RETURNING preferences, updated_at;
```

**Side Effects**:
- Increments `preferences_version` within JSONB
- Sets `preferences_last_updated_at` within JSONB
- Triggers cache invalidation for personalized content (via hash mismatch)

**Frequency**: Low (once at signup, occasional updates)

---

### 3. Check Cache for Personalized Content

**Use Case**: Before generating personalized content, check if cached version exists.

**Query**:
```sql
UPDATE personalized_content_cache
SET last_accessed_at = NOW(), access_count = access_count + 1
WHERE user_id = $1
  AND chapter_id = $2
  AND content_type = $3
  AND user_preferences_hash = $4
  AND chapter_content_hash = $5
  AND (target_language = $6 OR (target_language IS NULL AND $6 IS NULL))
RETURNING generated_content, generation_metadata;
```

**Expected Result**: Cached content if found, NULL otherwise

**Frequency**: High (every content request in personalized mode)

**Performance**: Index on `(user_id, chapter_id, content_type, user_preferences_hash)` ensures <10ms lookup

---

### 4. Store Generated Content in Cache

**Use Case**: After generating personalized or translated content, store in cache for future use.

**Query**:
```sql
INSERT INTO personalized_content_cache (
    user_id, chapter_id, content_type, user_preferences_hash,
    chapter_content_hash, target_language, generated_content,
    generation_metadata
) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
ON CONFLICT (user_id, chapter_id, content_type, user_preferences_hash,
             chapter_content_hash, target_language)
DO UPDATE SET
    generated_content = EXCLUDED.generated_content,
    generation_metadata = EXCLUDED.generation_metadata,
    last_accessed_at = NOW();
```

**Side Effects**: Updates existing cache if preferences/content match, creates new entry otherwise

**Frequency**: Medium (once per cache miss, ~10-30% of requests initially)

---

### 5. Cleanup Stale Cache Entries

**Use Case**: Periodic maintenance to remove old, unused cache entries.

**Query**:
```sql
DELETE FROM personalized_content_cache
WHERE last_accessed_at < NOW() - INTERVAL '90 days';
```

**Frequency**: Daily cron job

**Expected Impact**: Reclaim storage, improve query performance

---

## Migration Strategy

### Phase 1: Extend Existing user_profiles (No Migration Needed)

**Action**: Use existing `user_profiles.preferences` JSONB column for personalization preferences

**Rationale**:
- Column already exists (created in migration `21c54f44e3e4_create_user_profiles.py`)
- Backend already has JSONB read/write logic in `backend/src/services/user_profile.py`
- No schema change required, just extend Pydantic models

**Implementation**:
```python
# backend/src/models/user.py
class PersonalizationPreferences(BaseModel):
    experience_level: str  # 'beginner' | 'intermediate' | 'advanced'
    learning_topics: List[str]
    learning_goals: str
    content_mode: str  # 'full' | 'personalized'
    urdu_translation_enabled: bool = False
    preferences_submitted_at: Optional[datetime] = None
    preferences_last_updated_at: Optional[datetime] = None
    preferences_version: int = 1

class UserProfile(BaseModel):
    user_id: str
    # ... existing fields ...
    preferences: Optional[PersonalizationPreferences] = None  # JSONB
    created_at: datetime
    updated_at: datetime
```

---

### Phase 2: Create personalized_content_cache Table

**Migration**: Create new Alembic migration

**File**: `backend/alembic/versions/[timestamp]_create_personalized_content_cache.py`

**Migration Up**:
```python
def upgrade():
    op.create_table(
        'personalized_content_cache',
        sa.Column('cache_id', postgresql.UUID(as_uuid=True), primary_key=True,
                  server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', sa.TEXT(), nullable=False),
        sa.Column('chapter_id', sa.VARCHAR(255), nullable=False),
        sa.Column('content_type', sa.VARCHAR(20), nullable=False),
        sa.Column('user_preferences_hash', sa.VARCHAR(64), nullable=False),
        sa.Column('chapter_content_hash', sa.VARCHAR(64), nullable=False),
        sa.Column('target_language', sa.VARCHAR(10), nullable=True),
        sa.Column('generated_content', sa.TEXT(), nullable=False),
        sa.Column('generation_metadata', postgresql.JSONB(), nullable=True),
        sa.Column('created_at', sa.TIMESTAMP(), server_default=sa.func.now(), nullable=False),
        sa.Column('last_accessed_at', sa.TIMESTAMP(), server_default=sa.func.now(), nullable=False),
        sa.Column('access_count', sa.INTEGER(), server_default='1', nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['user.id'], ondelete='CASCADE')
    )

    # Unique constraint for cache key
    op.create_unique_constraint(
        'uq_personalized_cache',
        'personalized_content_cache',
        ['user_id', 'chapter_id', 'content_type', 'user_preferences_hash',
         'chapter_content_hash', 'target_language']
    )

    # Indexes
    op.create_index(
        'idx_personalized_cache_lookup',
        'personalized_content_cache',
        ['user_id', 'chapter_id', 'content_type', 'user_preferences_hash']
    )
    op.create_index(
        'idx_personalized_cache_cleanup',
        'personalized_content_cache',
        ['last_accessed_at']
    )

def downgrade():
    op.drop_table('personalized_content_cache')
```

---

## Data Validation Rules

### Preference Validation

**Pydantic Model** (Python backend):
```python
from pydantic import BaseModel, Field, field_validator
from typing import List, Optional
from datetime import datetime

class PersonalizationPreferences(BaseModel):
    experience_level: str = Field(..., pattern="^(beginner|intermediate|advanced)$")
    learning_topics: List[str] = Field(..., min_length=1, max_length=10)
    learning_goals: str = Field(..., max_length=500)
    content_mode: str = Field(..., pattern="^(full|personalized)$")
    urdu_translation_enabled: bool = False
    preferences_submitted_at: Optional[datetime] = None
    preferences_last_updated_at: Optional[datetime] = None
    preferences_version: int = Field(default=1, ge=1)

    @field_validator('learning_topics')
    def validate_topics(cls, v):
        if not all(1 <= len(topic) <= 50 for topic in v):
            raise ValueError('Each topic must be 1-50 characters')
        return v
```

**TypeScript Type** (Frontend):
```typescript
export interface PersonalizationPreferences {
  experience_level: 'beginner' | 'intermediate' | 'advanced';
  learning_topics: string[];  // 1-10 items, each 1-50 chars
  learning_goals: string;  // max 500 chars
  content_mode: 'full' | 'personalized';
  urdu_translation_enabled: boolean;
  preferences_submitted_at?: string;  // ISO 8601
  preferences_last_updated_at?: string;  // ISO 8601
  preferences_version: number;
}
```

---

## Storage Size Estimates

### user_profiles.preferences JSONB

**Average Size per User**:
```json
{
  "experience_level": "intermediate",  // ~15 bytes
  "learning_topics": ["ROS 2", "Computer Vision", "Path Planning"],  // ~60 bytes
  "learning_goals": "250 char string...",  // ~250 bytes
  "content_mode": "personalized",  // ~15 bytes
  "urdu_translation_enabled": true,  // ~5 bytes
  "preferences_submitted_at": "2025-12-01T10:00:00Z",  // ~30 bytes
  "preferences_last_updated_at": "2025-12-01T10:00:00Z",  // ~30 bytes
  "preferences_version": 1  // ~5 bytes
}
```

**Total**: ~410 bytes per user
**For 1000 users**: ~410 KB (negligible)

---

### personalized_content_cache Table

**Average Size per Cache Entry**:
- `cache_id`: 16 bytes (UUID)
- `user_id`: ~20 bytes (TEXT)
- `chapter_id`: ~30 bytes (VARCHAR)
- `content_type`: ~15 bytes (VARCHAR)
- `user_preferences_hash`: 64 bytes
- `chapter_content_hash`: 64 bytes
- `target_language`: 10 bytes
- `generated_content`: ~10 KB average (personalized chapter)
- `generation_metadata`: ~200 bytes (JSONB)
- Timestamps + access_count: ~30 bytes

**Total**: ~10.5 KB per cache entry

**Estimated Entries**:
- 1000 users × 50 chapters × 1.2 (some users have both personalized + translated) = 60,000 entries
- Total storage: 60,000 × 10.5 KB = **630 MB**

**With 90-day retention**: Assuming 50% of cache is stale after 90 days, steady-state ~315 MB

---

## Summary

✅ **Existing Infrastructure Leveraged**:
- `user_profiles.preferences` JSONB column (no migration needed for preferences)
- Existing user authentication and session management via Better Auth

✅ **New Infrastructure Added**:
- `personalized_content_cache` table for API cost and latency optimization

✅ **Data Access Performance**:
- Indexed lookups for cache queries (<10ms)
- In-memory caching for user preferences (5-minute TTL)

✅ **Storage Efficiency**:
- Hash-based cache keys enable automatic invalidation
- Periodic cleanup of stale entries prevents unbounded growth
- Estimated storage: ~315 MB steady-state for 1000 users

➡️ **Next**: Generate API contracts in /contracts/ directory
