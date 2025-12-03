from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import StreamingResponse # Import StreamingResponse
from typing import Optional, AsyncIterator, List
# For Pydantic models in request body
from pydantic import BaseModel, Field
from datetime import datetime
import re # Import re module

from ...core.auth import get_current_active_user, User
from ...services.user_profile import UserProfileService
from ...services.content_loader import get_chapter_content
from ...services.personalization.content_generator import generate_personalized_content
from ...services.personalization.cache import PersonalizationCache
from ...core.logging import logger
from ...database import get_db_session
from sqlalchemy.ext.asyncio import AsyncSession
from ...models.user import PersonalizationPreferences
from ...services.personalization.translator import translate_to_urdu

async def async_iterator(items):
    for item in items:
        yield item

class PersonalizeContentRequest(BaseModel):
    chapter_id: str
    user_level: str = Field(..., pattern="^(beginner|intermediate|advanced)$")
    learning_topics: List[str] = Field(..., min_length=1, max_length=10)
    learning_goals: str = Field(..., max_length=500)
    force_regenerate: bool = False # To bypass cache

class TranslateContentRequest(BaseModel):
    chapter_id: str
    target_language: str = Field(..., pattern="^ur$") # Only Urdu for now
    force_regenerate: bool = False # To bypass cache

# Dependency to get PersonalizationCache instance
async def get_personalization_cache(db_session: AsyncSession = Depends(get_db_session)) -> PersonalizationCache:
    return PersonalizationCache(db_session=db_session)

router = APIRouter()

@router.get("/chapters/{chapter_id:path}", summary="Get chapter content based on user preferences")
async def get_chapter(
    chapter_id: str,
    user: User = Depends(get_current_active_user),
    personalization_cache: PersonalizationCache = Depends(get_personalization_cache) # Inject cache service
) -> StreamingResponse:
    """
    Retrieve chapter content, either full or personalized based on user preferences.
    Also applies Urdu translation if enabled in preferences.
    """
    preferences: Optional[PersonalizationPreferences] = await UserProfileService.get_personalization_preferences(user["id"])
    
    # Debug logging for preferences
    logger.info(f"User {user['id']} preferences: {preferences.model_dump() if preferences else None}")
    logger.info(f"Urdu translation enabled: {preferences.urdu_translation_enabled if preferences else None}")

    async def generate_response_stream(content_iterator: AsyncIterator[str]):
        async for chunk in content_iterator:
            yield chunk

    # Determine base content first (full or personalized)
    base_content_stream: AsyncIterator[str]
    full_base_content: str
    is_personalized = False # Flag to know if content is personalized

    if preferences and preferences.content_mode == "personalized":
        is_personalized = True
        original_content = await get_chapter_content(chapter_id)
        if not original_content:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Chapter not found for personalization")
        try:
            # Generate personalized content and collect it into a single string
            personalized_content_stream_raw = generate_personalized_content(
                chapter_id=chapter_id,
                user_level=preferences.experience_level,
                learning_topics=preferences.learning_topics,
                learning_goals=preferences.learning_goals
            )
            base_content_chunks = []
            async for chunk in personalized_content_stream_raw:
                base_content_chunks.append(chunk)
            full_base_content = "".join(base_content_chunks)
            base_content_stream = async_iterator([full_base_content]) # Re-wrap for streaming
        except Exception as e:
            logger.error(f"Error generating personalized content for {chapter_id}: {e}")
            full_base_content = original_content # Fallback
            base_content_stream = async_iterator([full_base_content]) # Re-wrap for streaming
    else:
        # Default to full content
        full_base_content = await get_chapter_content(chapter_id)
        if not full_base_content:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Chapter not found")
        base_content_stream = async_iterator([full_base_content])


    # Now handle translation if enabled
    logger.info(f"[TRANSLATION CHECK] preferences exists: {preferences is not None}, urdu_translation_enabled: {preferences.urdu_translation_enabled if preferences else 'N/A'}")
    if preferences and preferences.urdu_translation_enabled:
        logger.info(f"[TRANSLATION] Translation enabled for user {user['id']}, chapter {chapter_id}")
        chapter_content_hash = personalization_cache._compute_content_hash(full_base_content)
        translation_cache_key_data = {
            "user_id": user["id"],
            "target_language": "ur",
            "chapter_content_hash": chapter_content_hash,
            "is_personalized_base": is_personalized # Include if base was personalized
        }
        translation_cache_hash = personalization_cache._compute_preferences_hash(translation_cache_key_data)

        # Check cache for translated content
        cached_content_entry = await personalization_cache.get_cached(
            user_id=user["id"],
            chapter_id=chapter_id,
            content_type="translated",
            user_preferences_hash=translation_cache_hash,
            chapter_content_hash=chapter_content_hash,
            target_language="ur"
        )
        if cached_content_entry:
            logger.info(f"Cache hit for translated content: {chapter_id} (ur) for user {user["id"]}")
            return StreamingResponse(generate_response_stream(async_iterator([cached_content_entry.generated_content])), media_type="text/markdown")

        logger.info(f"Cache miss for translated content: {chapter_id} (ur) for user {user["id"]}. Translating...")
        try:
            translation_stream_raw, extracted_code_blocks = await translate_to_urdu(full_base_content)
            translated_chunks = []
            async for chunk in translation_stream_raw:
                translated_chunks.append(chunk)
            full_translated_content_with_placeholders = "".join(translated_chunks)

            # Post-processing: Restore code blocks
            def restore_code_blocks(text: str) -> str:
                def replacer(match):
                    placeholder_id = match.group(1)
                    return extracted_code_blocks.get(placeholder_id, f"{{{{{placeholder_id}}}}} (Error: Code not found)")
                return re.sub(r"\{\{(CODE_BLOCK_\d+)\}\}", replacer, text)
            
            final_translated_content = restore_code_blocks(full_translated_content_with_placeholders)

            # Explicit validation for T079
            reinserted_code_block_count = len(re.findall(r"\{\{(CODE_BLOCK_\d+)\}\}", full_translated_content_with_placeholders))
            # `original_code_block_count` needs to be `len(extracted_code_blocks)` which is available.
            if len(extracted_code_blocks) != reinserted_code_block_count:
                logger.warning(f"Code block count mismatch after translation for chapter {chapter_id}. Original: {len(extracted_code_blocks)}, Reinserted: {reinserted_code_block_count}")
            
            generation_metadata = {
                "model": "gpt-4o-mini",
                "generated_at": datetime.utcnow().isoformat(),
                "tokens_used": None,
                "generation_time_ms": None
            }

            await personalization_cache.set_cached(
                user_id=user["id"],
                chapter_id=chapter_id,
                content_type="translated",
                user_preferences_hash=translation_cache_hash,
                chapter_content_hash=chapter_content_hash,
                generated_content=final_translated_content,
                generation_metadata=generation_metadata,
                target_language="ur"
            )
            return StreamingResponse(generate_response_stream(async_iterator([final_translated_content])), media_type="text/markdown")
        except Exception as e:
            logger.error(f"Error generating translated content for {chapter_id} for user {user["id"]}: {e}")
            # Fallback to base content stream (non-translated)
            return StreamingResponse(generate_response_stream(async_iterator([full_base_content])), media_type="text/markdown")
    else:
        # If translation not enabled, just return the base content stream
        logger.info(f"[TRANSLATION] Translation NOT enabled, returning base content for user {user['id']}, chapter {chapter_id}")
        return StreamingResponse(generate_response_stream(base_content_stream), media_type="text/markdown")


@router.post("/content/personalize", summary="Generate and retrieve personalized chapter content")
async def personalize_chapter(
    request: PersonalizeContentRequest,
    user: User = Depends(get_current_active_user),
    personalization_cache: PersonalizationCache = Depends(get_personalization_cache)
) -> StreamingResponse:
    """
    Generates personalized chapter content based on user preferences and caches the result.
    """
    user_preferences = PersonalizationPreferences(
        experience_level=request.user_level,
        learning_topics=request.learning_topics,
        learning_goals=request.learning_goals,
        content_mode="personalized", # This endpoint is always for personalized content
        urdu_translation_enabled=False # Not relevant for personalization endpoint
    )

    user_preferences_hash = personalization_cache._compute_preferences_hash(user_preferences.model_dump(mode='json'))
    
    original_content = await get_chapter_content(request.chapter_id)
    if not original_content:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Chapter not found for personalization")
    
    chapter_content_hash = personalization_cache._compute_content_hash(original_content)

    if not request.force_regenerate:
        cached_content_entry = await personalization_cache.get_cached(
            user_id=user["id"],
            chapter_id=request.chapter_id,
            content_type="personalized",
            user_preferences_hash=user_preferences_hash,
            chapter_content_hash=chapter_content_hash
        )
        if cached_content_entry:
            logger.info(f"Cache hit for personalized content: {request.chapter_id} for user {user["id"]}")
            return StreamingResponse(async_iterator([cached_content_entry.generated_content]), media_type="text/markdown")

    logger.info(f"Cache miss or force regenerate for personalized content: {request.chapter_id} for user {user["id"]}. Generating...")
    
    # Generate content if cache miss or force_regenerate is true
    generated_chunks = []
    try:
        personalized_content_stream = generate_personalized_content(
            chapter_id=request.chapter_id,
            user_level=request.user_level,
            learning_topics=request.learning_topics,
            learning_goals=request.learning_goals
        )
        # Collect stream to save to cache, then return as stream
        async for chunk in personalized_content_stream:
            generated_chunks.append(chunk)
            # This yield is wrong. The outer function is not an async generator
            # Need to collect all chunks first and then stream
            # The StreamingResponse will handle the yielding from the iterator passed to it.
            # So, we accumulate chunks, then pass the accumulated list to async_iterator
            # or yield here if `personalize_chapter` itself is an async generator (which it is not here)
            # Correct approach:
            # yield chunk # This would make personalize_chapter an async generator.
            # Then the return type should be AsyncIterator[str] rather than StreamingResponse
            # Let's adjust the logic slightly. The yield chunk is valid if the endpoint
            # itself is an async generator, but here it's meant to collect for caching then return.
            pass # Remove direct yield here, accumulate instead
            
        full_generated_content = "".join(generated_chunks)
        # TODO: Get generation_metadata from OpenAI response if available (Task T061)
        generation_metadata = {
            "model": "gpt-4o-mini",
            "generated_at": datetime.utcnow().isoformat(),
            "tokens_used": None, # To be captured from OpenAI response if available
            "generation_time_ms": None # To be calculated
        }

        await personalization_cache.set_cached(
            user_id=user["id"],
            chapter_id=request.chapter_id,
            content_type="personalized",
            user_preferences_hash=user_preferences_hash,
            chapter_content_hash=chapter_content_hash,
            generated_content=full_generated_content,
            generation_metadata=generation_metadata
        )
        return StreamingResponse(async_iterator([full_generated_content]), media_type="text/markdown")
    except Exception as e:
        logger.error(f"Error generating personalized content for {request.chapter_id} for user {user["id"]}: {e}")
        # On failure, return original content as fallback
        return StreamingResponse(async_iterator([original_content]), media_type="text/markdown")


@router.post("/content/translate", summary="Translate chapter content to a target language")
async def translate_chapter(
    request: TranslateContentRequest,
    user: User = Depends(get_current_active_user),
    personalization_cache: PersonalizationCache = Depends(get_personalization_cache)
) -> StreamingResponse:
    """
    Translates chapter content to the target language (currently Urdu) based on user preferences and caches the result.
    """
    original_content = await get_chapter_content(request.chapter_id)
    if not original_content:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Chapter not found for translation")

    chapter_content_hash = personalization_cache._compute_content_hash(original_content)
    
    # For translation, user preferences hash is simplified to include target_language and user_id
    translation_cache_key_data = {
        "user_id": user["id"],
        "target_language": request.target_language,
        "chapter_content_hash": chapter_content_hash # Also include content hash
    }
    translation_cache_hash = personalization_cache._compute_preferences_hash(translation_cache_key_data)


    if not request.force_regenerate:
        cached_content_entry = await personalization_cache.get_cached(
            user_id=user["id"],
            chapter_id=request.chapter_id,
            content_type="translated",
            user_preferences_hash=translation_cache_hash, # Use specific hash for translation cache
            chapter_content_hash=chapter_content_hash,
            target_language=request.target_language
        )
        if cached_content_entry:
            logger.info(f"Cache hit for translated content: {request.chapter_id} ({request.target_language}) for user {user["id"]}")
            return StreamingResponse(async_iterator([cached_content_entry.generated_content]), media_type="text/markdown")

    logger.info(f"Cache miss or force regenerate for translated content: {request.chapter_id} ({request.target_language}) for user {user["id"]}. Translating...")

    # Translate content if cache miss or force_regenerate is true
    try:
        # translate_to_urdu now returns stream and extracted_code_blocks
        translation_stream, extracted_code_blocks = await translate_to_urdu(original_content)
        
        # Count original code blocks
        original_code_block_count = len(extracted_code_blocks)

        translated_chunks = []
        async for chunk in translation_stream:
            translated_chunks.append(chunk)

        full_translated_content_with_placeholders = "".join(translated_chunks)

        # Post-processing: Restore code blocks
        def restore_code_blocks(text: str) -> str:
            def replacer(match):
                placeholder_id = match.group(1)
                return extracted_code_blocks.get(placeholder_id, f"{{{{{placeholder_id}}}}} (Error: Code not found)")
            return re.sub(r"\{\{(CODE_BLOCK_\d+)\}\}", replacer, text)

        final_translated_content = restore_code_blocks(full_translated_content_with_placeholders)

        # Explicit validation for T079
        reinserted_code_block_count = len(re.findall(r"\{\{(CODE_BLOCK_\d+)\}\}", full_translated_content_with_placeholders))
        if original_code_block_count != reinserted_code_block_count:
            logger.warning(f"Code block count mismatch after translation for chapter {request.chapter_id}. Original: {original_code_block_count}, Reinserted: {reinserted_code_block_count}")
            # Potentially raise HTTPException or log a critical error, but for now, just warning

        # TODO: Get generation_metadata from OpenAI response if available (Task T061)
        generation_metadata = {
            "model": "gpt-4o-mini",
            "generated_at": datetime.utcnow().isoformat(),
            "tokens_used": None,
            "generation_time_ms": None
        }

        await personalization_cache.set_cached(
            user_id=user["id"],
            chapter_id=request.chapter_id,
            content_type="translated",
            user_preferences_hash=translation_cache_hash,
            chapter_content_hash=chapter_content_hash,
            generated_content=final_translated_content,
            generation_metadata=generation_metadata,
            target_language=request.target_language
        )
        return StreamingResponse(async_iterator([final_translated_content]), media_type="text/markdown")
    except Exception as e:
        logger.error(f"Error generating translated content for {request.chapter_id} for user {user["id"]}: {e}")
        # On failure, return original content as fallback
        return StreamingResponse(async_iterator([original_content]), media_type="text/markdown")