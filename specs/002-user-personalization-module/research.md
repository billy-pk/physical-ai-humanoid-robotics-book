# Research: User Personalization Module

**Phase**: 0 - Outline & Research
**Date**: 2025-12-01
**Purpose**: Resolve technical unknowns and document best practices for personalization implementation

## Overview

This document consolidates research findings for implementing the user personalization module, covering OpenAI Agents SDK integration patterns, translation strategies, caching mechanisms, and database schema design.

## Research Areas

### 1. OpenAI Agents SDK for Content Personalization

**Context**: Need to generate personalized educational content based on user preferences (level, topics, goals) while maintaining technical accuracy.

**Decision**: Use OpenAI Agents SDK with function tools pattern (similar to existing RAG agent)

**Rationale**:
- **Existing Pattern**: Project already uses OpenAI Agents SDK successfully in `backend/src/services/rag/rag_agent.py` with `@function_tool` decorator and `Runner.run()` execution
- **Structured Control**: Function tools allow controlled content generation with specific inputs (user preferences, chapter content, target level)
- **Model Selection**: `gpt-4o-mini` balances cost and quality (same model as RAG agent) - sufficient for educational content adaptation
- **Streaming Support**: Agents SDK supports streaming responses for better UX during long content generation

**Implementation Approach**:
```python
# Pattern based on existing rag_agent.py
from openai_agents import Agent, function_tool, Runner

@function_tool
async def fetch_chapter_content(chapter_id: str) -> str:
    """Retrieve original chapter content for personalization"""
    # Read from Docusaurus docs or database
    pass

personalization_agent = Agent(
    model="gpt-4o-mini",
    instructions="""
    You are an educational content adapter for a robotics course.

    When given:
    - Original chapter content
    - User experience level (beginner/intermediate/advanced)
    - User learning topics (e.g., "ROS 2 navigation", "computer vision")
    - User learning goals (free text)

    Generate adapted content that:
    1. Adjusts explanation depth for the user's level
    2. Emphasizes concepts related to their topics of interest
    3. Provides examples aligned with their goals
    4. Maintains technical accuracy and cites original sections
    5. Preserves all code blocks exactly as provided
    6. Uses markdown formatting

    For beginners: Use simple language, more analogies, step-by-step explanations
    For intermediate: Assume foundational knowledge, focus on application
    For advanced: Emphasize edge cases, optimizations, and architectural patterns
    """,
    tools=[fetch_chapter_content]
)

async def generate_personalized_content(
    chapter_id: str,
    user_level: str,
    user_topics: List[str],
    user_goals: str
) -> str:
    prompt = f"""
    Personalize the content for chapter {chapter_id} for a user with:
    - Experience level: {user_level}
    - Topics of interest: {', '.join(user_topics)}
    - Learning goals: {user_goals}

    Use the fetch_chapter_content tool to get the original content, then adapt it.
    """
    result = await Runner.run(personalization_agent, prompt)
    return result.final_output
```

**Alternatives Considered**:
- **Direct Chat Completion API**: More flexible but loses structured tool-use benefits and consistency with existing RAG pattern
- **Fine-tuned Model**: Too expensive and time-consuming for initial MVP; can consider after validating personalization value
- **Template-based Approach**: Limited adaptability; doesn't leverage LLM's language understanding for true personalization

**Best Practices**:
- **Chunking for Long Chapters**: If chapter exceeds context window (~100k tokens for gpt-4o-mini), split into sections and personalize each
- **Code Block Preservation**: In agent instructions, explicitly state "preserve all code blocks exactly as provided" and use regex post-processing validation
- **Caching**: Cache personalized content by (user_id, chapter_id, preferences_hash) to avoid regeneration costs
- **Error Handling**: Implement fallback to original content if OpenAI API fails

---

### 2. Urdu Translation with Code Block Preservation

**Context**: Need to translate educational content to Urdu while keeping code blocks in original language (English).

**Decision**: Use OpenAI Agents SDK with pre-processing to mark code blocks, translation, then post-processing to restore code

**Rationale**:
- **GPT-4 Translation Quality**: OpenAI models have strong Urdu support (Urdu is one of the top 50 languages in training data)
- **Technical Terminology**: Educational robotics content uses many English technical terms; GPT can intelligently transliterate or retain terms where appropriate
- **Code Preservation Strategy**:
  1. Pre-process: Extract code blocks, replace with placeholders like `{{CODE_BLOCK_1}}`
  2. Translate: Pass text with placeholders to translation agent
  3. Post-process: Restore code blocks at placeholder positions

**Implementation Approach**:
```python
import re
from openai_agents import Agent, Runner

translation_agent = Agent(
    model="gpt-4o-mini",  # or gpt-4o for better quality
    instructions="""
    You are a technical translator specializing in educational robotics content.

    Translate the given English text to Urdu while:
    1. Maintaining technical accuracy
    2. Using appropriate Urdu technical terminology when it exists
    3. Transliterating or keeping English terms when standard Urdu translations don't exist
       (e.g., "robot", "sensor", "ROS", "Python" often kept in English/transliterated)
    4. Preserving all {{CODE_BLOCK_N}} placeholders exactly as they appear
    5. Maintaining markdown formatting (headings, lists, bold, italic)
    6. Keeping right-to-left text flow for Urdu while left-to-right for code placeholders

    Provide only the translated text, no explanations.
    """
)

async def translate_to_urdu(chapter_content: str) -> str:
    # Step 1: Extract code blocks
    code_blocks = []
    def replace_code(match):
        code_blocks.append(match.group(0))
        return f"{{{{CODE_BLOCK_{len(code_blocks)}}}}}"

    # Match both inline code and code blocks
    content_with_placeholders = re.sub(
        r'```[\s\S]*?```|`[^`]+`',
        replace_code,
        chapter_content
    )

    # Step 2: Translate content with placeholders
    prompt = f"Translate the following text to Urdu:\n\n{content_with_placeholders}"
    result = await Runner.run(translation_agent, prompt)
    translated_text = result.final_output

    # Step 3: Restore code blocks
    for i, code_block in enumerate(code_blocks, 1):
        placeholder = f"{{{{CODE_BLOCK_{i}}}}}"
        translated_text = translated_text.replace(placeholder, code_block)

    return translated_text
```

**Alternatives Considered**:
- **Specialized Translation API** (Google Translate, DeepL): Lower cost but poor handling of technical content and no context awareness
- **Manual Translation**: High quality but not scalable; could be used for validation/comparison
- **Bilingual Model Training**: Too expensive and complex for MVP; OpenAI's multilingual capabilities are sufficient

**Best Practices**:
- **Validation**: Use regex to verify all code blocks are preserved character-for-character after translation
- **Caching**: Cache translations by (chapter_id, chapter_version_hash, target_language) - translations are deterministic per content version
- **User Feedback**: Allow users to report translation issues for continuous improvement
- **Fallback**: If translation fails, provide English original with error message

---

### 3. Caching Strategy for Generated Content

**Context**: Personalized content and translations are expensive (API cost, latency). Need efficient caching to minimize regeneration.

**Decision**: Use database-backed caching with cache keys based on content identity and user preferences

**Rationale**:
- **Cost Reduction**: Personalized content generation costs ~$0.01-0.05 per chapter (depending on length), caching saves 90%+ of these costs for returning users
- **Latency**: Cached content serves in <100ms vs 5-15s for generation
- **Invalidation Strategy**: Cache must invalidate when either source content or user preferences change

**Implementation Approach**:

**Database Schema** (extend existing Postgres):
```sql
CREATE TABLE personalized_content_cache (
    cache_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id TEXT NOT NULL REFERENCES "user"(id),
    chapter_id VARCHAR(255) NOT NULL,
    content_type VARCHAR(20) NOT NULL,  -- 'personalized' or 'translated'

    -- Cache key components
    user_preferences_hash VARCHAR(64) NOT NULL,  -- SHA256 of user preferences JSON
    chapter_content_hash VARCHAR(64) NOT NULL,   -- SHA256 of original chapter content
    target_language VARCHAR(10),                 -- e.g., 'ur' for Urdu, NULL for personalized

    -- Cached content
    generated_content TEXT NOT NULL,
    generation_metadata JSONB,  -- model, tokens, generation_time, etc.

    -- Timestamps
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    last_accessed_at TIMESTAMP NOT NULL DEFAULT NOW(),
    access_count INTEGER DEFAULT 1,

    -- Indexes for fast lookup
    UNIQUE(user_id, chapter_id, content_type, user_preferences_hash, chapter_content_hash, target_language)
);

CREATE INDEX idx_personalized_cache_lookup ON personalized_content_cache(
    user_id, chapter_id, content_type, user_preferences_hash
);
CREATE INDEX idx_personalized_cache_cleanup ON personalized_content_cache(last_accessed_at);
```

**Cache Service**:
```python
import hashlib
import json
from typing import Optional

class PersonalizationCache:
    def __init__(self, db_pool):
        self.db = db_pool

    def _compute_preferences_hash(self, preferences: dict) -> str:
        """Deterministic hash of user preferences"""
        # Sort keys for deterministic JSON
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
    ) -> Optional[str]:
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
                RETURNING generated_content
            """, user_id, chapter_id, content_type, prefs_hash, content_hash, target_language)

            return result['generated_content'] if result else None

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

**Cache Invalidation Strategy**:
- **Automatic**: When chapter_content_hash changes (chapter updated), old cache entries naturally miss
- **Manual**: When user updates preferences, preferences_hash changes, triggering new generation
- **Cleanup**: Periodic job to delete cache entries not accessed in 90 days (configurable)

**Alternatives Considered**:
- **Redis Cache**: Faster but adds infrastructure complexity; Postgres is sufficient for MVP with proper indexing
- **No Caching**: Unacceptable due to cost and latency for repeated content requests
- **Client-Side Caching**: Doesn't work across devices; database cache works everywhere

**Best Practices**:
- **Cache Hit Metrics**: Log cache hit rate to monitor effectiveness
- **Preemptive Generation**: Consider background job to pre-generate personalized content for active users' unvisited chapters
- **Partial Cache**: For very long chapters, cache by section to enable partial updates

---

### 4. Database Schema for Personalization Preferences

**Context**: Need to store user personalization preferences, extending existing user_profiles table.

**Decision**: Extend existing `user_profiles.preferences` JSONB column rather than creating new table

**Rationale**:
- **Existing Infrastructure**: `user_profiles` table already has `preferences` JSONB column (from migration `21c54f44e3e4_create_user_profiles.py`)
- **Flexibility**: JSONB allows schema evolution without migrations for preference additions
- **Single Source**: Keeps all user profile data in one place; no joins needed
- **Existing Pattern**: Backend already reads/writes `user_profiles.preferences` in `backend/src/services/user_profile.py`

**Schema Design**:

**Extend `user_profiles` table** (no new migration needed initially, use existing structure):
```python
# Pydantic model in backend/src/models/user.py
class PersonalizationPreferences(BaseModel):
    """Stored in user_profiles.preferences JSONB column"""

    # Preference collection fields
    experience_level: str  # 'beginner' | 'intermediate' | 'advanced'
    learning_topics: List[str]  # e.g., ["ROS 2", "Computer Vision", "Path Planning"]
    learning_goals: str  # Free text, max 500 chars
    content_mode: str  # 'full' | 'personalized'
    urdu_translation_enabled: bool = False

    # Metadata
    preferences_submitted_at: Optional[datetime] = None
    preferences_last_updated_at: Optional[datetime] = None

    # Version for cache invalidation
    preferences_version: int = 1  # Increment when user updates preferences

class UserProfile(BaseModel):
    user_id: str
    # ... existing fields ...
    preferences: Optional[PersonalizationPreferences] = None  # JSONB column
    created_at: datetime
    updated_at: datetime
```

**Migration Strategy**:
- **Phase 1 (MVP)**: Use existing `preferences` JSONB column; no migration needed
- **Phase 2 (If JSONB becomes problematic)**: Create dedicated `personalization_preferences` table with foreign key to user_profiles

**API Contract**:
```python
# POST /api/auth/profile/preferences
{
    "experience_level": "beginner",
    "learning_topics": ["ROS 2", "Computer Vision"],
    "learning_goals": "I want to build autonomous navigation systems",
    "content_mode": "personalized",
    "urdu_translation_enabled": false
}

# Response: 200 OK
{
    "success": true,
    "preferences": { ... },
    "preferences_version": 1
}
```

**Alternatives Considered**:
- **Separate Table**: More normalized but adds join overhead; JSONB sufficient for current scale
- **Multiple Columns**: Less flexible for adding new preference types; JSONB allows schema evolution
- **External Service**: Over-engineering for MVP; keep preferences with user data

**Best Practices**:
- **Validation**: Use Pydantic models to validate preference structure before storing in JSONB
- **Versioning**: Include `preferences_version` field for cache invalidation
- **Default Values**: Provide sensible defaults (e.g., `content_mode='full'`) for users who skip optional fields
- **Audit Trail**: Log preference changes to `user_background_questionnaire` table for analytics

---

### 5. Redirect Flow After Login

**Context**: Need to redirect users to `/popup` after login if they haven't completed preferences, otherwise to main content.

**Decision**: Use frontend route guard with session check

**Rationale**:
- **Existing Auth Pattern**: Project already has `QuestionnaireGuard` component (in `frontend/src/components/Auth/QuestionnaireGuard.tsx`) that checks background questionnaire completion
- **Extend Pattern**: Add similar check for personalization preferences
- **User Experience**: Non-blocking; users can skip and configure later if desired

**Implementation Approach**:

**Frontend Route Guard**:
```typescript
// frontend/src/components/Auth/PersonalizationGuard.tsx
import { useEffect } from 'react';
import { useRouter } from '@docusaurus/router';
import { useAuth } from '@site/src/hooks/useAuth';

export function PersonalizationGuard({ children }) {
    const router = useRouter();
    const { session, loading } = useAuth();

    useEffect(() => {
        if (loading) return;

        if (session?.user) {
            // Check if user has completed personalization preferences
            // by fetching from /api/auth/profile
            fetch('/api/auth/profile', {
                credentials: 'include'
            })
            .then(res => res.json())
            .then(data => {
                const hasPersonalizationPrefs = data.preferences?.experience_level
                    && data.preferences?.content_mode;

                if (!hasPersonalizationPrefs && router.pathname !== '/popup') {
                    router.push('/popup');
                }
            });
        }
    }, [session, loading, router]);

    return <>{children}</>;
}
```

**Integration with Root Component**:
```typescript
// frontend/src/theme/Root.tsx (extend existing Root)
export default function Root({ children }) {
    return (
        <AuthProvider>
            <PersonalizationProvider>
                <PersonalizationGuard>
                    {children}
                </PersonalizationGuard>
            </PersonalizationProvider>
        </AuthProvider>
    );
}
```

**Alternatives Considered**:
- **Backend Redirect**: More control but requires coordinating between Better Auth service and frontend; increases complexity
- **Modal Instead of Page**: Less disruptive but harder to ensure completion; dedicated page is clearer
- **Blocking vs Non-Blocking**: Made non-blocking (user can skip) for better UX; blocking may frustrate users

**Best Practices**:
- **Skip Option**: Allow users to dismiss/skip preference setup and access content immediately
- **Reminder**: Show subtle reminder banner if preferences not completed after multiple sessions
- **Edit Access**: Always allow users to edit preferences later via profile/settings page

---

## Summary of Key Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Content Personalization | OpenAI Agents SDK with function tools | Consistent with existing RAG pattern, structured control |
| Translation | GPT-4 with pre/post-processing for code preservation | Strong Urdu support, intelligent terminology handling |
| Caching | Postgres-backed with hash-based keys | Cost/latency reduction, simple invalidation strategy |
| Preferences Storage | Extend existing `user_profiles.preferences` JSONB | Leverage existing infrastructure, flexible schema |
| Redirect Flow | Frontend route guard (extend QuestionnaireGuard) | Consistent with existing auth pattern, non-blocking UX |
| Model Choice | gpt-4o-mini for both personalization and translation | Cost-effective, sufficient quality, 128k context window |
| Code Block Preservation | Regex extraction → placeholder → restoration | Deterministic, testable, language-agnostic |
| Cache Invalidation | Hash-based (preferences + content) | Automatic on changes, no manual management needed |

## Risk Mitigation

| Risk | Mitigation Strategy |
|------|---------------------|
| OpenAI API rate limits | Implement request queuing, cache aggressively, monitor usage |
| Translation quality issues | Provide user feedback mechanism, fallback to English, manual review for critical content |
| Cache storage growth | Periodic cleanup of stale entries (>90 days), monitor database size |
| Context window limits | Chunking strategy for long chapters, section-based generation |
| Cost overrun | Set budget alerts, cache hit rate monitoring, consider tier upgrades |

## Next Steps

✅ Research complete - all technical unknowns resolved
➡️ Proceed to Phase 1: Design & Contracts
- Create data-model.md with entity definitions
- Generate API contracts in /contracts/ directory
- Create quickstart.md for developer onboarding
