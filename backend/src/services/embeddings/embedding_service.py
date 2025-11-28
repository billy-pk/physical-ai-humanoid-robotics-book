import hashlib
import uuid
from typing import List, Optional
from datetime import datetime

from backend.src.core.logging import logger
from backend.src.services.llm.openai_client import generate_embeddings
from backend.src.services.vectordb.qdrant_client import upsert_embeddings
from backend.src.models.embeddings import EmbeddingChunk


def compute_content_hash(content: str) -> str:
    """
    Compute SHA-256 hash of content for deduplication.

    Args:
        content: Text content to hash

    Returns:
        Hexadecimal hash string
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def chunk_text(
    text: str,
    chunk_size: int = 1000,
    overlap: int = 200
) -> List[str]:
    """
    Split text into overlapping chunks for embedding.

    Args:
        text: Full text content to chunk
        chunk_size: Target size of each chunk in characters
        overlap: Number of overlapping characters between chunks

    Returns:
        List of text chunks
    """
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # Try to break at sentence boundary if possible
        if end < len(text):
            # Look for period, question mark, or exclamation within last 100 chars
            last_sentence = text[max(start, end - 100):end].rfind('.')
            if last_sentence != -1:
                end = max(start, end - 100) + last_sentence + 1

        chunk = text[start:end].strip()
        if chunk:
            chunks.append(chunk)

        start = end - overlap

    logger.info("Text chunked", total_chunks=len(chunks), chunk_size=chunk_size)
    return chunks


async def create_embeddings_for_chapter(
    chapter_title: str,
    chapter_id: str,
    content: str,
    section_title: Optional[str] = None,
    source_url: str = "",
    metadata: Optional[dict] = None
) -> List[EmbeddingChunk]:
    """
    Create embeddings for a book chapter.

    Args:
        chapter_title: Title of the chapter
        chapter_id: Unique identifier for the chapter
        content: Full chapter text content
        section_title: Optional section title within chapter
        source_url: URL to the chapter in the documentation
        metadata: Additional metadata to store

    Returns:
        List of EmbeddingChunk objects created
    """
    # Split content into chunks
    text_chunks = chunk_text(content)

    embedding_chunks = []

    for idx, chunk_text in enumerate(text_chunks):
        # Generate embedding
        vector = await generate_embeddings(chunk_text)

        # Estimate token count (rough approximation: 1 token ≈ 4 characters)
        token_count = len(chunk_text) // 4

        # Create EmbeddingChunk
        chunk = EmbeddingChunk(
            content=chunk_text,
            content_hash=compute_content_hash(chunk_text),
            vector=vector,
            chapter_id=chapter_id,
            chapter_title=chapter_title,
            section_title=section_title,
            chunk_index=idx,
            token_count=token_count,
            metadata=metadata or {}
        )

        embedding_chunks.append(chunk)

    logger.info(
        "Created embeddings for chapter",
        chapter_id=chapter_id,
        chunks_count=len(embedding_chunks)
    )

    return embedding_chunks


async def store_embeddings(embeddings: List[EmbeddingChunk]) -> None:
    """
    Store embedding chunks in Qdrant vector database.

    Args:
        embeddings: List of EmbeddingChunk objects to store
    """
    # Convert EmbeddingChunk objects to Qdrant format
    chunks_data = []
    for emb in embeddings:
        chunks_data.append({
            "id": str(emb.id),
            "vector": emb.vector,
            "payload": {
                "content": emb.content,
                "content_hash": emb.content_hash,
                "chapter_id": emb.chapter_id,
                "chapter_title": emb.chapter_title,
                "section_title": emb.section_title,
                "chunk_index": emb.chunk_index,
                "token_count": emb.token_count,
                "created_at": emb.created_at.isoformat(),
                "metadata": emb.metadata
            }
        })

    await upsert_embeddings(chunks_data)

    logger.info("Stored embeddings in Qdrant", count=len(chunks_data))


async def process_and_store_chapter(
    chapter_title: str,
    chapter_id: str,
    content: str,
    section_title: Optional[str] = None,
    source_url: str = "",
    metadata: Optional[dict] = None
) -> int:
    """
    End-to-end processing: chunk, embed, and store chapter content.

    Args:
        chapter_title: Title of the chapter
        chapter_id: Unique identifier for the chapter
        content: Full chapter text content
        section_title: Optional section title within chapter
        source_url: URL to the chapter in the documentation
        metadata: Additional metadata to store

    Returns:
        Number of chunks created and stored
    """
    logger.info("Processing chapter", chapter_id=chapter_id, chapter_title=chapter_title)

    # Create embeddings
    embeddings = await create_embeddings_for_chapter(
        chapter_title=chapter_title,
        chapter_id=chapter_id,
        content=content,
        section_title=section_title,
        source_url=source_url,
        metadata=metadata
    )

    # Store in Qdrant
    await store_embeddings(embeddings)

    logger.info("Chapter processing complete", chapter_id=chapter_id, chunks=len(embeddings))

    return len(embeddings)
