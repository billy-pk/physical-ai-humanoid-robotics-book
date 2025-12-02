# Quickstart: User Personalization Module

**Phase**: 1 - Design & Contracts
**Date**: 2025-12-01
**Purpose**: Developer onboarding guide for implementing the user personalization feature

## Overview

This quickstart guide helps developers understand, implement, and test the user personalization module. The feature adds preference collection, personalized content generation, and Urdu translation capabilities to the existing Physical AI & Humanoid Robotics Book platform.

## Prerequisites

Before starting, ensure you have:

- ✅ Existing project running locally (both frontend and backend)
- ✅ Better Auth authentication system operational
- ✅ OpenAI API key with access to GPT-4o-mini
- ✅ Database access (Neon Postgres)
- ✅ Python 3.12+ and Node.js 20+
- ✅ Familiarity with FastAPI, React, and TypeScript

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        User Browser                          │
│                    (http://localhost:3000)                   │
└───────────────────┬──────────────────────────────────────────┘
                    │
         ┌──────────┼──────────┐
         │                     │
         ▼                     ▼
  ┌─────────────┐      ┌────────────────┐
  │  Frontend   │      │    Backend     │
  │  (React)    │      │   (FastAPI)    │
  │             │      │                │
  │ - /popup    │◄────►│ - Preferences  │
  │ - Content   │      │   API          │
  │   Modes     │      │ - Content Gen  │
  └─────────────┘      │ - Translation  │
                       └────────┬───────┘
                                │
                       ┌────────┼────────┐
                       │                 │
                       ▼                 ▼
               ┌──────────────┐  ┌────────────┐
               │   Postgres   │  │  OpenAI    │
               │   (Cache)    │  │   Agents   │
               └──────────────┘  └────────────┘
```

## Development Workflow

### Phase 1: Database Setup (30 minutes)

#### 1.1 Create Database Migration

Create a new Alembic migration for the content cache table:

```bash
cd backend
alembic revision -m "create personalized content cache table"
```

Edit the generated migration file in `backend/alembic/versions/[timestamp]_create_personalized_content_cache.py`:

```python
"""create personalized content cache table

Revision ID: [generated]
Revises: 954501217eb6
Create Date: [generated]
"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = '[generated]'
down_revision = '954501217eb6'
branch_labels = None
depends_on = None

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

    # Unique constraint
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

#### 1.2 Run Migration

```bash
alembic upgrade head
```

**Expected Output:**
```
INFO  [alembic.runtime.migration] Running upgrade 954501217eb6 -> [new_id], create personalized content cache table
```

---

### Phase 2: Backend Implementation (4-6 hours)

#### 2.1 Extend User Models

Edit `backend/src/models/user.py` to add personalization preference models:

```python
from pydantic import BaseModel, Field, field_validator
from typing import List, Optional
from datetime import datetime

class PersonalizationPreferences(BaseModel):
    """User personalization preferences (stored in user_profiles.preferences JSONB)"""
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

class PersonalizedContentCacheEntry(BaseModel):
    """Cache entry for personalized/translated content"""
    cache_id: str
    user_id: str
    chapter_id: str
    content_type: str  # 'personalized' or 'translated'
    generated_content: str
    generation_metadata: Optional[dict] = None
    cached: bool = True
```

#### 2.2 Create Personalization Service

Create `backend/src/services/personalization/content_generator.py`:

```python
"""
Content personalization service using OpenAI Agents SDK
"""
from openai_agents import Agent, function_tool, Runner
from typing import List, Dict
import logging

logger = logging.getLogger(__name__)

@function_tool
async def fetch_chapter_content(chapter_id: str) -> str:
    """Fetch original chapter content from filesystem or database"""
    # TODO: Implement chapter content fetching logic
    # For now, read from frontend/docs/ directory
    chapter_path = f"frontend/docs/{chapter_id}.md"
    with open(chapter_path, 'r') as f:
        return f.read()

personalization_agent = Agent(
    model="gpt-4o-mini",
    instructions="""
    You are an educational content adapter for a Physical AI & Humanoid Robotics course.

    When given:
    - Original chapter content
    - User experience level (beginner/intermediate/advanced)
    - User learning topics
    - User learning goals

    Generate adapted content that:
    1. Adjusts explanation depth for the user's level
    2. Emphasizes concepts related to their topics of interest
    3. Provides examples aligned with their goals
    4. Maintains technical accuracy and cites original sections
    5. Preserves all code blocks exactly as provided
    6. Uses markdown formatting

    Level guidelines:
    - Beginner: Simple language, analogies, step-by-step, foundational concepts
    - Intermediate: Assume basic knowledge, focus on application and integration
    - Advanced: Emphasize edge cases, optimizations, architectural patterns

    IMPORTANT: Do NOT modify code blocks. Preserve them exactly as provided.
    """,
    tools=[fetch_chapter_content]
)

async def generate_personalized_content(
    chapter_id: str,
    user_level: str,
    user_topics: List[str],
    user_goals: str
) -> Dict:
    """Generate personalized content for a chapter"""
    try:
        prompt = f"""
        Personalize the content for chapter '{chapter_id}' for a user with:
        - Experience level: {user_level}
        - Topics of interest: {', '.join(user_topics)}
        - Learning goals: {user_goals}

        Use the fetch_chapter_content tool to get the original content, then adapt it.
        """

        result = await Runner.run(personalization_agent, prompt)

        return {
            "content": result.final_output,
            "model": "gpt-4o-mini",
            "tokens_used": getattr(result, 'tokens_used', 0),
        }
    except Exception as e:
        logger.error(f"Personalization failed for chapter {chapter_id}: {e}")
        raise
```

#### 2.3 Create Translation Service

Create `backend/src/services/personalization/translator.py`:

```python
"""
Translation service using OpenAI Agents SDK
"""
import re
from openai_agents import Agent, Runner
from typing import Dict
import logging

logger = logging.getLogger(__name__)

translation_agent = Agent(
    model="gpt-4o-mini",
    instructions="""
    You are a technical translator for educational robotics content.

    Translate English text to Urdu while:
    1. Maintaining technical accuracy
    2. Using appropriate Urdu terminology when it exists
    3. Transliterating or keeping English terms when standard Urdu translations don't exist
    4. Preserving all {{CODE_BLOCK_N}} placeholders exactly
    5. Maintaining markdown formatting
    6. Using right-to-left text flow for Urdu

    Provide only the translated text, no explanations.
    """
)

async def translate_to_urdu(chapter_content: str) -> Dict:
    """Translate chapter content to Urdu, preserving code blocks"""
    try:
        # Step 1: Extract code blocks
        code_blocks = []
        def replace_code(match):
            code_blocks.append(match.group(0))
            return f"{{{{CODE_BLOCK_{len(code_blocks)}}}}}"

        content_with_placeholders = re.sub(
            r'```[\s\S]*?```|`[^`]+`',
            replace_code,
            chapter_content
        )

        # Step 2: Translate
        prompt = f"Translate the following text to Urdu:\n\n{content_with_placeholders}"
        result = await Runner.run(translation_agent, prompt)
        translated_text = result.final_output

        # Step 3: Restore code blocks
        for i, code_block in enumerate(code_blocks, 1):
            placeholder = f"{{{{CODE_BLOCK_{i}}}}}"
            translated_text = translated_text.replace(placeholder, code_block)

        # Validate code blocks preserved
        original_code_count = len(re.findall(r'```[\s\S]*?```|`[^`]+`', chapter_content))
        translated_code_count = len(re.findall(r'```[\s\S]*?```|`[^`]+`', translated_text))

        if original_code_count != translated_code_count:
            logger.error(f"Code block count mismatch: {original_code_count} != {translated_code_count}")
            raise ValueError("Code block preservation failed")

        return {
            "content": translated_text,
            "model": "gpt-4o-mini",
            "tokens_used": getattr(result, 'tokens_used', 0),
        }
    except Exception as e:
        logger.error(f"Translation failed: {e}")
        raise
```

#### 2.4 Create Cache Service

Create `backend/src/services/personalization/cache.py`:

```python
"""
Caching service for personalized/translated content
"""
import hashlib
import json
from typing import Optional, Dict
import logging

logger = logging.getLogger(__name__)

class PersonalizationCache:
    def __init__(self, db_pool):
        self.db = db_pool

    def _compute_preferences_hash(self, preferences: dict) -> str:
        """Deterministic hash of user preferences"""
        prefs_json = json.dumps(preferences, sort_keys=True)
        return hashlib.sha256(prefs_json.encode()).hexdigest()

    def _compute_content_hash(self, content: str) -> str:
        """Hash of original chapter content"""
        return hashlib.sha256(content.encode()).hexdigest()

    async def get_cached(
        self,
        user_id: str,
        chapter_id: str,
        content_type: str,
        user_preferences: dict,
        chapter_content: str,
        target_language: Optional[str] = None
    ) -> Optional[Dict]:
        """Retrieve cached content if exists"""
        prefs_hash = self._compute_preferences_hash(user_preferences)
        content_hash = self._compute_content_hash(chapter_content)

        async with self.db.acquire() as conn:
            result = await conn.fetchrow("""
                UPDATE personalized_content_cache
                SET last_accessed_at = NOW(), access_count = access_count + 1
                WHERE user_id = $1 AND chapter_id = $2 AND content_type = $3
                  AND user_preferences_hash = $4 AND chapter_content_hash = $5
                  AND (target_language = $6 OR (target_language IS NULL AND $6 IS NULL))
                RETURNING generated_content, generation_metadata
            """, user_id, chapter_id, content_type, prefs_hash, content_hash, target_language)

            if result:
                return {
                    "content": result['generated_content'],
                    "metadata": result['generation_metadata'],
                    "cached": True
                }
            return None

    async def set_cached(
        self,
        user_id: str,
        chapter_id: str,
        content_type: str,
        user_preferences: dict,
        chapter_content: str,
        generated_content: str,
        metadata: dict,
        target_language: Optional[str] = None
    ):
        """Store generated content in cache"""
        prefs_hash = self._compute_preferences_hash(user_preferences)
        content_hash = self._compute_content_hash(chapter_content)

        async with self.db.acquire() as conn:
            await conn.execute("""
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
                    last_accessed_at = NOW()
            """, user_id, chapter_id, content_type, prefs_hash, content_hash,
                target_language, generated_content, json.dumps(metadata))
```

#### 2.5 Create API Routes

Create `backend/src/api/routes/content.py`:

```python
"""
API routes for content personalization and translation
"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
from src.services.personalization.content_generator import generate_personalized_content
from src.services.personalization.translator import translate_to_urdu
from src.services.personalization.cache import PersonalizationCache
from src.core.auth import get_current_user

router = APIRouter(prefix="/api/content", tags=["content"])

class PersonalizeRequest(BaseModel):
    chapter_id: str
    force_regenerate: bool = False

class TranslateRequest(BaseModel):
    chapter_id: str
    target_language: str
    force_regenerate: bool = False

@router.post("/personalize")
async def personalize_content(
    request: PersonalizeRequest,
    current_user: dict = Depends(get_current_user)
):
    """Generate personalized content for a chapter"""
    # TODO: Implement full logic with cache check
    try:
        result = await generate_personalized_content(
            request.chapter_id,
            user_level="intermediate",  # Get from user preferences
            user_topics=["ROS 2"],
            user_goals="Learn navigation"
        )
        return {"success": True, "chapter_id": request.chapter_id, **result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/translate")
async def translate_content(
    request: TranslateRequest,
    current_user: dict = Depends(get_current_user)
):
    """Translate chapter content to target language"""
    if request.target_language != "ur":
        raise HTTPException(status_code=400, detail="Only Urdu (ur) supported")

    try:
        # TODO: Fetch chapter content and check cache
        chapter_content = "# Sample chapter content..."
        result = await translate_to_urdu(chapter_content)
        return {"success": True, "chapter_id": request.chapter_id, **result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

#### 2.6 Register Routes in Main App

Edit `backend/src/main.py` to include the new routes:

```python
from src.api.routes import content

app.include_router(content.router)
```

---

### Phase 3: Frontend Implementation (3-4 hours)

#### 3.1 Create Preference Form Component

Create `frontend/src/components/Auth/PreferenceForm.tsx`:

```typescript
import React, { useState } from 'react';
import { useRouter } from '@docusaurus/router';

export interface PersonalizationPreferences {
  experience_level: 'beginner' | 'intermediate' | 'advanced';
  learning_topics: string[];
  learning_goals: string;
  content_mode: 'full' | 'personalized';
  urdu_translation_enabled: boolean;
}

export function PreferenceForm() {
  const router = useRouter();
  const [formData, setFormData] = useState<PersonalizationPreferences>({
    experience_level: 'beginner',
    learning_topics: [],
    learning_goals: '',
    content_mode: 'full',
    urdu_translation_enabled: false,
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const topicOptions = [
    'ROS 2', 'Computer Vision', 'Path Planning', 'SLAM',
    'Manipulation', 'Sensors', 'Actuators', 'Simulation'
  ];

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/profile/preferences', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(formData),
      });

      if (!response.ok) throw new Error('Failed to save preferences');

      router.push('/');
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="preference-form">
      <h2>Personalize Your Learning Experience</h2>

      <div className="form-group">
        <label>Experience Level</label>
        <select
          value={formData.experience_level}
          onChange={(e) => setFormData({ ...formData, experience_level: e.target.value as any })}
        >
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
        </select>
      </div>

      <div className="form-group">
        <label>Topics of Interest</label>
        {topicOptions.map(topic => (
          <label key={topic}>
            <input
              type="checkbox"
              checked={formData.learning_topics.includes(topic)}
              onChange={(e) => {
                const topics = e.target.checked
                  ? [...formData.learning_topics, topic]
                  : formData.learning_topics.filter(t => t !== topic);
                setFormData({ ...formData, learning_topics: topics });
              }}
            />
            {topic}
          </label>
        ))}
      </div>

      <div className="form-group">
        <label>Learning Goals</label>
        <textarea
          value={formData.learning_goals}
          onChange={(e) => setFormData({ ...formData, learning_goals: e.target.value })}
          maxLength={500}
          placeholder="What do you want to achieve?"
        />
      </div>

      <div className="form-group">
        <label>Content Mode</label>
        <select
          value={formData.content_mode}
          onChange={(e) => setFormData({ ...formData, content_mode: e.target.value as any })}
        >
          <option value="full">Full Content (Original)</option>
          <option value="personalized">Personalized Content</option>
        </select>
      </div>

      <div className="form-group">
        <label>
          <input
            type="checkbox"
            checked={formData.urdu_translation_enabled}
            onChange={(e) => setFormData({ ...formData, urdu_translation_enabled: e.target.checked })}
          />
          Enable Urdu Translation
        </label>
      </div>

      {error && <div className="error">{error}</div>}

      <button type="submit" disabled={loading}>
        {loading ? 'Saving...' : 'Save Preferences'}
      </button>
    </form>
  );
}
```

#### 3.2 Create Popup Page

Create `frontend/src/pages/popup.tsx`:

```typescript
import React from 'react';
import Layout from '@theme/Layout';
import { PreferenceForm } from '@site/src/components/Auth/PreferenceForm';

export default function PreferencesPopup() {
  return (
    <Layout title="Set Your Preferences" description="Personalize your learning experience">
      <div className="container margin-vert--lg">
        <PreferenceForm />
      </div>
    </Layout>
  );
}
```

#### 3.3 Create Personalization Context

Create `frontend/src/contexts/PersonalizationContext.tsx`:

```typescript
import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth } from '@site/src/hooks/useAuth';

interface PersonalizationContextType {
  preferences: any | null;
  loading: boolean;
  updatePreferences: (prefs: any) => Promise<void>;
}

const PersonalizationContext = createContext<PersonalizationContextType>(null);

export function PersonalizationProvider({ children }) {
  const { session } = useAuth();
  const [preferences, setPreferences] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (session?.user) {
      fetchPreferences();
    }
  }, [session]);

  const fetchPreferences = async () => {
    try {
      const response = await fetch('/api/auth/profile/preferences', {
        credentials: 'include'
      });
      if (response.ok) {
        const data = await response.json();
        setPreferences(data.preferences);
      }
    } catch (error) {
      console.error('Failed to fetch preferences:', error);
    } finally {
      setLoading(false);
    }
  };

  const updatePreferences = async (prefs: any) => {
    const response = await fetch('/api/auth/profile/preferences', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(prefs),
    });
    if (response.ok) {
      const data = await response.json();
      setPreferences(data.preferences);
    }
  };

  return (
    <PersonalizationContext.Provider value={{ preferences, loading, updatePreferences }}>
      {children}
    </PersonalizationContext.Provider>
  );
}

export const usePersonalization = () => useContext(PersonalizationContext);
```

---

### Phase 4: Testing (2-3 hours)

#### 4.1 Backend Unit Tests

Create `backend/tests/unit/test_content_generator.py`:

```python
import pytest
from src.services.personalization.content_generator import generate_personalized_content

@pytest.mark.asyncio
async def test_generate_personalized_content():
    result = await generate_personalized_content(
        chapter_id="module-1/introduction",
        user_level="beginner",
        user_topics=["ROS 2"],
        user_goals="Learn basics"
    )

    assert "content" in result
    assert len(result["content"]) > 0
    assert result["model"] == "gpt-4o-mini"
```

#### 4.2 Frontend Component Tests

Create `frontend/tests/components/PreferenceForm.test.tsx`:

```typescript
import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import { PreferenceForm } from '@site/src/components/Auth/PreferenceForm';

describe('PreferenceForm', () => {
  it('renders all form fields', () => {
    render(<PreferenceForm />);

    expect(screen.getByLabelText(/Experience Level/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/Topics of Interest/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/Learning Goals/i)).toBeInTheDocument();
  });

  it('submits form data', async () => {
    render(<PreferenceForm />);

    // TODO: Complete test implementation
  });
});
```

---

## Local Testing

### 1. Start Backend

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

**Expected**: Server starts on http://localhost:8000

### 2. Start Frontend

```bash
cd frontend
npm start
```

**Expected**: Frontend starts on http://localhost:3000

### 3. Manual Test Flow

1. **Sign up** for a new account
2. **Verify redirect** to `/popup` after login
3. **Submit preferences** with various options
4. **Navigate** to a chapter and verify content mode
5. **Test translation** by enabling Urdu

---

## Troubleshooting

### Issue: Migration Fails

**Error**: `relation "user" does not exist`

**Solution**: Ensure Better Auth migrations have been applied first. Better Auth auto-creates the `user` table.

---

### Issue: OpenAI API Errors

**Error**: `401 Unauthorized`

**Solution**: Verify `OPENAI_API_KEY` is set in `.env`:

```bash
echo $OPENAI_API_KEY
```

---

### Issue: Preferences Not Saving

**Error**: `JSONB validation error`

**Solution**: Check Pydantic model validation. Ensure all fields match schema.

---

## Next Steps

After completing this quickstart:

1. ✅ Run full test suite: `pytest backend/tests && npm test --prefix frontend`
2. ✅ Review performance metrics (cache hit rate, API latency)
3. ✅ Test with real chapter content (not sample data)
4. ✅ Implement error handling and fallbacks
5. ✅ Add monitoring and logging
6. ➡️ Proceed to `/sp.tasks` for detailed implementation tasks

## Resources

- [OpenAPI Contract](./contracts/personalization-api.openapi.yaml)
- [Data Model Specification](./data-model.md)
- [Research Decisions](./research.md)
- [Feature Specification](./spec.md)

---

**Questions or Issues?** Check the research.md for design decisions and rationale.
