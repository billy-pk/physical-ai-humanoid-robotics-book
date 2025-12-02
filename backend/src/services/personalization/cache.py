import hashlib
import json
from typing import Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import text
from ...models.user import PersonalizedContentCacheEntry


class PersonalizationCache:
    def __init__(self, db_session: AsyncSession):
        self.db_session = db_session

    def _compute_preferences_hash(self, preferences: Dict[str, Any]) -> str:
        """Computes a SHA256 hash of user preferences."""
        # Ensure consistent order for hashing
        sorted_preferences = json.dumps(preferences, sort_keys=True).encode('utf-8')
        return hashlib.sha256(sorted_preferences).hexdigest()

    def _compute_content_hash(self, content: str) -> str:
        """Computes a SHA256 hash of content."""
        return hashlib.sha256(content.encode('utf-8')).hexdigest()

    async def get_cached(self, user_id: str, chapter_id: str, content_type: str,
                         user_preferences_hash: str, chapter_content_hash: str,
                         target_language: Optional[str] = None) -> Optional[PersonalizedContentCacheEntry]:
        """
        Retrieves cached personalized or translated content from the database.
        Updates last_accessed_at and access_count on retrieval.
        """
        query = text("""
            UPDATE personalized_content_cache
            SET last_accessed_at = NOW(), access_count = access_count + 1
            WHERE user_id = :user_id
              AND chapter_id = :chapter_id
              AND content_type = :content_type
              AND user_preferences_hash = :user_preferences_hash
              AND chapter_content_hash = :chapter_content_hash
              AND (target_language = :target_language OR (target_language IS NULL AND :target_language IS NULL))
            RETURNING cache_id, user_id, chapter_id, content_type, user_preferences_hash,
                      chapter_content_hash, target_language, generated_content,
                      generation_metadata, created_at, last_accessed_at, access_count;
        """)

        result = await self.db_session.execute(query, {
            "user_id": user_id,
            "chapter_id": chapter_id,
            "content_type": content_type,
            "user_preferences_hash": user_preferences_hash,
            "chapter_content_hash": chapter_content_hash,
            "target_language": target_language
        })
        row = result.first()
        if row:
            # Convert row to dictionary to pass to Pydantic model
            return PersonalizedContentCacheEntry.model_validate(row._asdict())
        return None

    async def set_cached(self, user_id: str, chapter_id: str, content_type: str,
                         user_preferences_hash: str, chapter_content_hash: str,
                         generated_content: str, generation_metadata: Dict[str, Any],
                         target_language: Optional[str] = None) -> None:
        """
        Stores generated personalized or translated content in the cache.
        Uses upsert logic to handle conflicts.
        """
        query = text("""
            INSERT INTO personalized_content_cache (
                user_id, chapter_id, content_type, user_preferences_hash,
                chapter_content_hash, target_language, generated_content,
                generation_metadata
            ) VALUES (
                :user_id, :chapter_id, :content_type, :user_preferences_hash,
                :chapter_content_hash, :target_language, :generated_content,
                :generation_metadata
            )
            ON CONFLICT (user_id, chapter_id, content_type, user_preferences_hash,
                         chapter_content_hash, target_language)
            DO UPDATE SET
                generated_content = EXCLUDED.generated_content,
                generation_metadata = EXCLUDED.generation_metadata,
                last_accessed_at = NOW(),
                access_count = personalized_content_cache.access_count + 1;
        """)
        
        await self.db_session.execute(query, {
            "user_id": user_id,
            "chapter_id": chapter_id,
            "content_type": content_type,
            "user_preferences_hash": user_preferences_hash,
            "chapter_content_hash": chapter_content_hash,
            "target_language": target_language,
            "generated_content": generated_content,
            "generation_metadata": generation_metadata
        })
        await self.db_session.commit()
