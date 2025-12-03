import os
import re
from pathlib import Path
from typing import Optional
from ..core.logging import logger

# Assuming the backend is run from the project root or its own directory
# This path might need adjustment based on deployment
# Adjusting path to correctly point to frontend/docs from backend/src/services
FRONTEND_DOCS_PATH = Path(__file__).parent.parent.parent.parent / "frontend" / "docs"

async def get_chapter_content(chapter_id: str) -> Optional[str]:
    """
    Reads chapter markdown content from the frontend/docs directory.

    Args:
        chapter_id: The ID of the chapter (e.g., "module-1/chapter-3").

    Returns:
        The content of the markdown file as a string, or None if not found.
    """
    logger.info(f"[ContentLoader] Requested chapter_id: {chapter_id}")
    logger.info(f"[ContentLoader] FRONTEND_DOCS_PATH: {FRONTEND_DOCS_PATH}")
    logger.info(f"[ContentLoader] FRONTEND_DOCS_PATH exists: {FRONTEND_DOCS_PATH.exists()}")

    # Sanitize chapter_id to prevent directory traversal attacks
    # Allow alphanumeric characters and hyphens (no special chars, no .., etc.)
    parts = chapter_id.split('/')
    logger.info(f"[ContentLoader] Split parts: {parts}")

    for part in parts:
        # Check if part contains only alphanumeric and hyphens
        is_valid = all(c.isalnum() or c == '-' for c in part) and len(part) > 0
        logger.info(f"[ContentLoader] Part '{part}' valid: {is_valid}")
        if not is_valid:
            logger.warning(f"[ContentLoader] Invalid part '{part}' in chapter_id: {chapter_id}")
            return None

    chapter_file_path = FRONTEND_DOCS_PATH / f"{chapter_id}.md"
    logger.info(f"[ContentLoader] Looking for file: {chapter_file_path}")
    logger.info(f"[ContentLoader] File exists: {chapter_file_path.exists()}")

    if not chapter_file_path.exists():
        logger.warning(f"[ContentLoader] Chapter file not found: {chapter_file_path}")
        return None

    try:
        # Read the file content
        content = chapter_file_path.read_text(encoding='utf-8')
        logger.info(f"[ContentLoader] Successfully read {len(content)} characters from {chapter_file_path}")

        # Strip YAML frontmatter (metadata between --- delimiters)
        # Frontmatter pattern: starts with ---, contains metadata, ends with ---
        frontmatter_pattern = r'^---\s*\n.*?\n---\s*\n'
        content_without_frontmatter = re.sub(frontmatter_pattern, '', content, flags=re.DOTALL)

        if len(content_without_frontmatter) < len(content):
            logger.info(f"[ContentLoader] Stripped frontmatter, new length: {len(content_without_frontmatter)} characters")

        return content_without_frontmatter
    except Exception as e:
        logger.error(f"[ContentLoader] Error reading chapter file {chapter_file_path}: {e}")
        return None