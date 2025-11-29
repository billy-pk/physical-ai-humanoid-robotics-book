#!/usr/bin/env python3
"""
Content ingestion script for Physical AI book.

This script:
1. Reads all markdown files from frontend/docs/module-*
2. Chunks the content into semantically meaningful pieces
3. Generates embeddings using OpenAI text-embedding-3-large
4. Uploads chunks to Qdrant vector database

Usage:
    uv run python ingest_content.py
"""

import sys
import asyncio
import re
from pathlib import Path
from typing import List, Dict, Tuple
from uuid import uuid4

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent))

from src.core.config import settings
from src.core.logging import logger
from src.services.llm.openai_client import generate_embeddings
from src.services.vectordb.qdrant_client import upsert_embeddings


def extract_frontmatter(content: str) -> Tuple[Dict[str, str], str]:
    """
    Extract YAML frontmatter from markdown content.

    Args:
        content: Raw markdown content

    Returns:
        Tuple of (frontmatter_dict, content_without_frontmatter)
    """
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n(.*)$'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if not match:
        return {}, content

    frontmatter_text = match.group(1)
    content_body = match.group(2)

    # Parse simple YAML (title, sidebar_position, etc.)
    frontmatter = {}
    for line in frontmatter_text.split('\n'):
        if ':' in line:
            key, value = line.split(':', 1)
            frontmatter[key.strip()] = value.strip().strip('"\'')

    return frontmatter, content_body


def chunk_markdown_by_sections(
    content: str,
    chapter_title: str,
    module_name: str,
    file_path: str
) -> List[Dict[str, str]]:
    """
    Chunk markdown content by sections (headers).

    Strategy:
    - Each ## (h2) section becomes a chunk
    - If section is too large (>1500 chars), split by ### (h3) subsections
    - Include chapter context in each chunk

    Args:
        content: Markdown content (without frontmatter)
        chapter_title: Title of the chapter
        module_name: Module identifier (e.g., "module-1")
        file_path: Path to the source file

    Returns:
        List of chunk dictionaries with content and metadata
    """
    chunks = []

    # Split by h2 headers (##)
    h2_pattern = r'^##\s+(.+?)$'
    sections = re.split(h2_pattern, content, flags=re.MULTILINE)

    # First element is content before first h2 (if any)
    intro_text = sections[0].strip()
    if intro_text and len(intro_text) > 50:
        chunks.append({
            "content": f"**{chapter_title}**\n\n{intro_text}",
            "section_title": "Introduction",
            "chapter_title": chapter_title,
            "module": module_name,
            "source_file": file_path
        })

    # Process h2 sections
    for i in range(1, len(sections), 2):
        if i + 1 >= len(sections):
            break

        section_title = sections[i].strip()
        section_content = sections[i + 1].strip()

        # If section is too large, split by h3
        if len(section_content) > 1500:
            h3_pattern = r'^###\s+(.+?)$'
            subsections = re.split(h3_pattern, section_content, flags=re.MULTILINE)

            # First part (before first h3)
            if subsections[0].strip():
                chunks.append({
                    "content": f"**{chapter_title} - {section_title}**\n\n{subsections[0].strip()}",
                    "section_title": section_title,
                    "chapter_title": chapter_title,
                    "module": module_name,
                    "source_file": file_path
                })

            # Process h3 subsections
            for j in range(1, len(subsections), 2):
                if j + 1 >= len(subsections):
                    break
                subsection_title = subsections[j].strip()
                subsection_content = subsections[j + 1].strip()

                if subsection_content:
                    chunks.append({
                        "content": f"**{chapter_title} - {section_title} - {subsection_title}**\n\n{subsection_content}",
                        "section_title": f"{section_title} - {subsection_title}",
                        "chapter_title": chapter_title,
                        "module": module_name,
                        "source_file": file_path
                    })
        else:
            # Section is small enough, add as single chunk
            chunks.append({
                "content": f"**{chapter_title} - {section_title}**\n\n{section_content}",
                "section_title": section_title,
                "chapter_title": chapter_title,
                "module": module_name,
                "source_file": file_path
            })

    return chunks


async def process_markdown_file(file_path: Path, module_name: str) -> List[Dict]:
    """
    Process a single markdown file: read, chunk, generate embeddings.

    Args:
        file_path: Path to markdown file
        module_name: Module identifier (e.g., "module-1")

    Returns:
        List of chunk dictionaries with embeddings and metadata
    """
    logger.info(f"Processing file: {file_path}")

    # Read file
    with open(file_path, 'r', encoding='utf-8') as f:
        raw_content = f.read()

    # Extract frontmatter
    frontmatter, content = extract_frontmatter(raw_content)
    chapter_title = frontmatter.get('title', file_path.stem.replace('-', ' ').title())

    # Chunk content
    chunks = chunk_markdown_by_sections(
        content=content,
        chapter_title=chapter_title,
        module_name=module_name,
        file_path=str(file_path)
    )

    logger.info(f"Created {len(chunks)} chunks from {file_path.name}")

    # Generate embeddings for all chunks
    processed_chunks = []
    for chunk_data in chunks:
        # Generate embedding for this chunk
        embedding = await generate_embeddings(chunk_data["content"])

        processed_chunks.append({
            "id": str(uuid4()),
            "vector": embedding,
            "payload": {
                "content": chunk_data["content"],
                "chapter_title": chunk_data["chapter_title"],
                "section_title": chunk_data["section_title"],
                "module": chunk_data["module"],
                "source_file": chunk_data["source_file"],
                "chunk_length": len(chunk_data["content"])
            }
        })

    return processed_chunks


async def ingest_all_modules():
    """
    Main ingestion function: process all modules and upload to Qdrant.
    """
    # Define base paths
    project_root = Path(__file__).parent.parent
    docs_dir = project_root / "frontend" / "docs"

    logger.info("Starting content ingestion...")
    logger.info(f"Docs directory: {docs_dir}")

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        return

    # Find all module directories
    module_dirs = sorted([d for d in docs_dir.iterdir() if d.is_dir() and d.name.startswith('module-')])

    if not module_dirs:
        logger.error("No module directories found in docs/")
        return

    logger.info(f"Found {len(module_dirs)} modules: {[d.name for d in module_dirs]}")

    all_chunks = []

    # Process each module
    for module_dir in module_dirs:
        module_name = module_dir.name
        logger.info(f"\n{'='*60}")
        logger.info(f"Processing {module_name}...")
        logger.info(f"{'='*60}")

        # Find all markdown files in this module
        md_files = sorted(module_dir.glob('*.md'))

        if not md_files:
            logger.warning(f"No markdown files found in {module_dir}")
            continue

        logger.info(f"Found {len(md_files)} files in {module_name}")

        # Process each file
        for md_file in md_files:
            try:
                chunks = await process_markdown_file(md_file, module_name)
                all_chunks.extend(chunks)
                logger.info(f"✅ Processed {md_file.name}: {len(chunks)} chunks")
            except Exception as e:
                logger.error(f"❌ Failed to process {md_file.name}: {e}")
                continue

    # Upload all chunks to Qdrant
    if all_chunks:
        logger.info(f"\n{'='*60}")
        logger.info(f"Uploading {len(all_chunks)} total chunks to Qdrant...")
        logger.info(f"{'='*60}")

        try:
            await upsert_embeddings(all_chunks)
            logger.info(f"✅ Successfully uploaded {len(all_chunks)} chunks to Qdrant!")
        except Exception as e:
            logger.error(f"❌ Failed to upload to Qdrant: {e}")
            raise
    else:
        logger.warning("No chunks were created. Nothing to upload.")

    # Print summary
    logger.info(f"\n{'='*60}")
    logger.info("INGESTION SUMMARY")
    logger.info(f"{'='*60}")
    logger.info(f"Total chunks created: {len(all_chunks)}")
    logger.info(f"Modules processed: {len(module_dirs)}")

    # Group by module
    by_module = {}
    for chunk in all_chunks:
        module = chunk["payload"]["module"]
        by_module[module] = by_module.get(module, 0) + 1

    for module, count in sorted(by_module.items()):
        logger.info(f"  {module}: {count} chunks")

    logger.info(f"{'='*60}\n")


if __name__ == "__main__":
    asyncio.run(ingest_all_modules())
