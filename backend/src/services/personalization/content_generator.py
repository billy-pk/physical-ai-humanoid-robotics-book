import asyncio
from typing import Optional, List, Dict, Any, AsyncIterator
import json

from ..content_loader import get_chapter_content
from openai import AsyncOpenAI


async def fetch_chapter_content(chapter_id: str) -> Optional[str]:
    """
    Fetches the full, original content of a chapter.
    This function acts as a tool for the OpenAI agent to access raw chapter data.

    Args:
        chapter_id: The ID of the chapter (e.g., "module-1/chapter-3").

    Returns:
        The full markdown content of the chapter, or None if the chapter is not found.
    """
    return await get_chapter_content(chapter_id)


async def generate_personalized_content(
    chapter_id: str,
    user_level: str, # beginner, intermediate, advanced
    learning_topics: List[str],
    learning_goals: str
) -> AsyncIterator[str]: # AsyncIterator from typing for async generator
    """
    Generates personalized chapter content using an OpenAI model.
    """
    client = AsyncOpenAI() # Assumes API key is in environment variables

    original_content = await fetch_chapter_content(chapter_id)
    if not original_content:
        # Yield an error message if content not found
        yield "Chapter content could not be retrieved."
        return

    # Define the system prompt for content adaptation
    system_prompt = f"""
    You are an expert educational assistant specializing in robotics and AI. Your goal is to adapt educational content to a user's specific learning profile.

    User Profile:
    - Experience Level: {user_level}
    - Learning Topics: {', '.join(learning_topics)}
    - Learning Goals: {learning_goals}

    Instructions for content adaptation:
    1.  Adapt the provided original chapter content based on the user's experience level, focusing on their learning topics and goals.
        -   **Beginner**: Simplify complex concepts, provide analogies, explain jargon. Emphasize foundational knowledge.
        -   **Intermediate**: Focus on practical applications, common challenges, and integration patterns. Assume basic understanding of core concepts.
        -   **Advanced**: Discuss nuances, optimization, cutting-edge research, and deeper theoretical underpinnings. Assume strong prior knowledge.
    2.  Integrate the user's learning topics and goals into the explanation where relevant, providing examples or context that align with them.
    3.  Maintain the overall structure and flow of the original content.
    4.  Preserve all code blocks and code snippets EXACTLY as they appear in the original content. Do not translate, modify, or reformat code.
    5.  Ensure the output is in Markdown format.
    6.  Cite original chapter sections if you significantly rephrase or summarize a large block of text (e.g., "[Original Section 3.1]").
    7.  If the provided content is empty, respond with "Chapter content could not be processed."
    """

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": f"Original chapter content for personalization:\n\n{original_content}"}
    ]

    response_stream = await client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        stream=True
    )
    
    async for chunk in response_stream:
        yield chunk.choices[0].delta.content or ""