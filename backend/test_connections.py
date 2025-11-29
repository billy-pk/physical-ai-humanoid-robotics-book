#!/usr/bin/env python3
"""Test database connections for Neon Postgres and Qdrant Cloud."""

import sys
import asyncio
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from src.core.config import settings


async def test_postgres():
    """Test Neon Postgres connection."""
    print("🔍 Testing Neon Postgres connection...")
    try:
        import psycopg

        # Parse connection string
        conn_str = settings.NEON_DATABASE_URL

        # Connect synchronously for simple test
        with psycopg.connect(conn_str) as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT version();")
                version = cur.fetchone()
                print(f"✅ Postgres connected! Version: {version[0][:50]}...")

                # Check if tables exist
                cur.execute("""
                    SELECT table_name
                    FROM information_schema.tables
                    WHERE table_schema = 'public'
                """)
                tables = cur.fetchall()
                if tables:
                    print(f"📊 Found {len(tables)} tables: {[t[0] for t in tables]}")
                else:
                    print("⚠️  No tables found - migrations need to be run")

        return True
    except Exception as e:
        print(f"❌ Postgres connection failed: {e}")
        return False


async def test_qdrant():
    """Test Qdrant Cloud connection."""
    print("\n🔍 Testing Qdrant Cloud connection...")
    try:
        from qdrant_client import QdrantClient

        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
        )

        # Test connection by getting collections
        collections = client.get_collections()
        print(f"✅ Qdrant connected! Found {len(collections.collections)} collections")

        # Check if book_embeddings collection exists
        collection_names = [c.name for c in collections.collections]
        if "book_embeddings" in collection_names:
            info = client.get_collection("book_embeddings")
            print(f"📚 Collection 'book_embeddings' exists with {info.points_count} points")
        else:
            print("⚠️  Collection 'book_embeddings' not found - needs to be created")
            print(f"   Existing collections: {collection_names}")

        return True
    except Exception as e:
        print(f"❌ Qdrant connection failed: {e}")
        return False


async def test_openai():
    """Test OpenAI API key."""
    print("\n🔍 Testing OpenAI API connection...")
    try:
        from openai import OpenAI

        client = OpenAI(api_key=settings.OPENAI_API_KEY)

        # Simple test - list models
        models = client.models.list()
        print(f"✅ OpenAI API connected! Found {len(models.data)} models available")

        return True
    except Exception as e:
        print(f"❌ OpenAI API connection failed: {e}")
        return False


async def main():
    """Run all connection tests."""
    print("=" * 60)
    print("🚀 Testing Database Connections")
    print("=" * 60)

    results = await asyncio.gather(
        test_postgres(),
        test_qdrant(),
        test_openai(),
    )

    print("\n" + "=" * 60)
    if all(results):
        print("✅ All connections successful!")
    else:
        print("⚠️  Some connections failed. Check errors above.")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())
