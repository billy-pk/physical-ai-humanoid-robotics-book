from qdrant_client import QdrantClient, models
from ...core.config import settings
from ...models.embeddings import VectorSearchResult
from typing import List, Optional
from ...core.logging import logger


def get_qdrant_client() -> QdrantClient:
    """
    Create and return a Qdrant client instance.

    Returns:
        QdrantClient: Configured Qdrant client
    """
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
    return client


def search_book_embeddings(
    query_vector: List[float],
    limit: int = 5,
    score_threshold: float = 0.7,
) -> List[VectorSearchResult]:
    """
    Search for similar book content chunks using vector similarity.

    Note: This function is synchronous since QdrantClient is synchronous.

    Args:
        query_vector: Embedding vector of the user's query (3072 dimensions)
        limit: Maximum number of results to return
        score_threshold: Minimum similarity score (0.0 to 1.0)

    Returns:
        List of VectorSearchResult objects with matching content
    """
    client = get_qdrant_client()

    try:
        # Use query_points for vector search (newer Qdrant API)
        search_result = client.query_points(
            collection_name="book_embeddings",
            query=query_vector,
            limit=limit,
            score_threshold=score_threshold,
            with_payload=True,
        )

        results = []
        for point in search_result.points:
            payload = point.payload or {}
            results.append(
                VectorSearchResult(
                    chunk_id=str(point.id),
                    content=payload.get("content", ""),
                    score=point.score,
                    chapter_title=payload.get("chapter_title", ""),
                    section_title=payload.get("section_title"),
                    source_url=payload.get("source_url", ""),
                    metadata=payload
                )
            )

        logger.info(
            "Vector search completed",
            results_count=len(results),
            limit=limit,
            threshold=score_threshold
        )

        return results

    except Exception as e:
        logger.error("Vector search failed", error=str(e))
        raise


async def create_book_embeddings_collection():
    """
    Create or recreate the book_embeddings collection in Qdrant.

    This sets up a collection with:
    - Vector size: 3072 (OpenAI text-embedding-3-large)
    - Distance metric: Cosine similarity
    """
    client = get_qdrant_client()

    try:
        client.recreate_collection(
            collection_name="book_embeddings",
            vectors_config=models.VectorParams(
                size=3072,  # OpenAI text-embedding-3-large dimension
                distance=models.Distance.COSINE
            ),
        )
        logger.info("Created book_embeddings collection")
    except Exception as e:
        logger.error("Failed to create collection", error=str(e))
        raise


async def upsert_embeddings(
    chunks: List[dict],
    batch_size: int = 50,
) -> None:
    """
    Insert or update embedding chunks in Qdrant in batches.

    Args:
        chunks: List of dictionaries with 'id', 'vector', and 'payload' keys
        batch_size: Number of chunks to upload per batch (default: 50)
    """
    client = get_qdrant_client()

    try:
        total_chunks = len(chunks)
        total_batches = (total_chunks + batch_size - 1) // batch_size
        
        logger.info(
            "Uploading embeddings in batches",
            total_chunks=total_chunks,
            batch_size=batch_size,
            total_batches=total_batches
        )
        
        for i in range(0, total_chunks, batch_size):
            batch = chunks[i:i + batch_size]
            batch_num = (i // batch_size) + 1
            
            points = [
                models.PointStruct(
                    id=chunk["id"],
                    vector=chunk["vector"],
                    payload=chunk["payload"]
                )
                for chunk in batch
            ]

            client.upsert(
                collection_name="book_embeddings",
                points=points
            )

            logger.info(
                "Batch uploaded",
                batch=f"{batch_num}/{total_batches}",
                chunks_in_batch=len(batch),
                total_uploaded=min(i + batch_size, total_chunks)
            )

        logger.info("Successfully upserted all embeddings", total_count=total_chunks)
    except Exception as e:
        logger.error("Failed to upsert embeddings", error=str(e))
        raise
