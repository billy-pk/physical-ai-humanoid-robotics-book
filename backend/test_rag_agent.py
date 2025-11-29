#!/usr/bin/env python3
"""
Test script for RAG agent with vector search tool.

This script tests the agent's ability to:
1. Use the search_book_content tool
2. Retrieve relevant information from Qdrant
3. Generate grounded responses with citations
"""

import asyncio
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent))

from src.services.rag.rag_agent import ask_question
from src.core.logging import logger


async def test_rag_agent():
    """Test the RAG agent with various questions."""

    print("\n" + "="*60)
    print("Testing RAG Agent with Vector Search Tool")
    print("="*60 + "\n")

    # Test questions
    test_questions = [
        "What is artificial intelligence?",
        "Explain supervised learning",
        "What is computer vision used for in robotics?",
        "How do you cook pasta?",  # Out of scope question
    ]

    for i, question in enumerate(test_questions, 1):
        print(f"\n{'─'*60}")
        print(f"Question {i}: {question}")
        print(f"{'─'*60}\n")

        try:
            answer, tokens = await ask_question(question)

            print(f"Answer:\n{answer}\n")
            print(f"Estimated tokens: {tokens}")

        except Exception as e:
            print(f"❌ Error: {e}")
            logger.error("Test question failed", question=question, error=str(e))

        print(f"\n{'─'*60}\n")

    print("\n" + "="*60)
    print("Test Complete")
    print("="*60 + "\n")


if __name__ == "__main__":
    asyncio.run(test_rag_agent())
