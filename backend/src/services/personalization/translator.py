import asyncio
import re
from typing import Optional, AsyncIterator, Tuple, List, Dict
import json

from openai import AsyncOpenAI
from ...core.config import settings
from ...core.logging import logger


CODE_BLOCK_REGEX = re.compile(r"(```.*?```)", re.DOTALL)
# INLINE_CODE_REGEX = re.compile(r"(`[^`]*`)") # Not used in current preprocessing


async def translate_to_urdu(chapter_content: str) -> Tuple[AsyncIterator[str], Dict[str, str]]:
    """
    Translates chapter content to Urdu using an OpenAI model.
    Pre-processes content to preserve code blocks and returns them for post-translation restoration.

    Returns:
        A tuple containing:
        - An AsyncIterator of translated content chunks (with placeholders).
        - A dictionary mapping placeholder IDs to original code blocks.
    """
    client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

    # Step 1: Extract code blocks and replace with placeholders
    extracted_code_blocks: Dict[str, str] = {}
    
    def replace_code_blocks_with_placeholders(text: str) -> str:
        # Function to replace code blocks with placeholders
        def replacer(match):
            full_match = match.group(0)
            placeholder_id = f"CODE_BLOCK_{len(extracted_code_blocks)}"
            extracted_code_blocks[placeholder_id] = full_match
            return f"{{{{{placeholder_id}}}}}"
        return CODE_BLOCK_REGEX.sub(replacer, text)
    
    # Process full code blocks first
    content_with_block_placeholders = replace_code_blocks_with_placeholders(chapter_content)

    system_prompt = """
    You are an expert translator specializing in technical and educational content.
    Your task is to translate the provided English text into Urdu.

    Critical rules to follow:
    1.  Placeholders like `{{CODE_BLOCK_N}}` represent original code snippets. DO NOT translate, modify, remove, or reformat these placeholders or the content they represent. Keep them EXACTLY as they are.
    2.  Translate all natural language text accurately and idiomatically into Urdu.
    3.  Handle technical terminology appropriately:
        -   If an established Urdu translation exists, use it.
        -   If not, transliterate the English term into Urdu script.
        -   If a term is universally used in English within the technical context, keep it in English (e.g., "ROS", "AI", "CPU").
    4.  Ensure the output is in Markdown format.
    5.  If the provided content is empty, respond with "Content could not be translated."
    """

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": f"Translate the following chapter content to Urdu:\n\n{content_with_block_placeholders}"}
    ]

    response_stream = await client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        stream=True
    )

    # Create async generator to process stream and yield text chunks
    async def process_stream():
        async for chunk in response_stream:
            # Extract text content from each chunk
            content = chunk.choices[0].delta.content
            if content:
                yield content

    return process_stream(), extracted_code_blocks
