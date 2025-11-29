#!/usr/bin/env python3
"""Test vector search for ROS query."""

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from src.services.llm.openai_client import generate_embeddings
from src.services.vectordb.qdrant_client import search_book_embeddings


async def test_ros_search():
    """Test searching for ROS content."""

    queries = [
        "what is ros",
        "ROS robot operating system",
        "ros and simulation",
        "introduction to ros",
    ]

    for query in queries:
        print(f"\n{'='*60}")
        print(f"Query: {query}")
        print(f"{'='*60}")

        # Generate embedding
        query_vector = await generate_embeddings(query)

        # Search with different thresholds
        for threshold in [0.3, 0.4, 0.5, 0.6]:
            results = search_book_embeddings(
                query_vector=query_vector,
                limit=3,
                score_threshold=threshold
            )

            print(f"\nThreshold {threshold}: Found {len(results)} results")
            for i, result in enumerate(results, 1):
                print(f"  [{i}] Score: {result.score:.3f} | {result.chapter_title}")
                print(f"      {result.content[:100]}...")


if __name__ == "__main__":
    asyncio.run(test_ros_search())
