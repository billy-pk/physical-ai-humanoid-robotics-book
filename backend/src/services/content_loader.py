import os
from pathlib import Path
from typing import Optional

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
    # Sanitize chapter_id to prevent directory traversal attacks
    # Allow alphanumeric characters, hyphens, and forward slashes for paths
    if not all(part.isalnum() or part == '-' for part in chapter_id.split('/')):
        return None

    chapter_file_path = FRONTEND_DOCS_PATH / f"{chapter_id}.md"

    if not chapter_file_path.exists():
        return None

    try:
        # Read the file content
        return chapter_file_path.read_text(encoding='utf-8')
    except Exception as e:
        # Log the error if necessary
        # logger.error(f"Error reading chapter file {chapter_file_path}: {e}")
        return None