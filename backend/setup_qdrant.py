#!/usr/bin/env python3
"""Setup Qdrant collection for book embeddings."""

import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from src.core.config import settings
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams


def setup_qdrant_collection():
    """Create book_embeddings collection with proper configuration."""

    print("=" * 60)
    print("🚀 Setting up Qdrant Collection")
    print("=" * 60)

    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )

    collection_name = "book_embeddings"

    # Check if collection already exists
    collections = client.get_collections()
    collection_names = [c.name for c in collections.collections]

    if collection_name in collection_names:
        print(f"⚠️  Collection '{collection_name}' already exists")
        response = input("Do you want to delete and recreate it? (yes/no): ")
        if response.lower() == 'yes':
            client.delete_collection(collection_name)
            print(f"🗑️  Deleted existing collection '{collection_name}'")
        else:
            print("❌ Aborting setup")
            return False

    # Create collection with specifications from tasks.md (T012)
    print(f"\n📚 Creating collection '{collection_name}'...")
    print(f"   - Vector size: 3072 dimensions (text-embedding-3-large)")
    print(f"   - Distance metric: Cosine")

    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=3072,  # text-embedding-3-large model dimension
            distance=Distance.COSINE,  # Cosine similarity for semantic search
        ),
    )

    # Verify creation
    info = client.get_collection(collection_name)
    print(f"\n✅ Collection '{collection_name}' created successfully!")
    print(f"   - Points count: {info.points_count}")
    print(f"   - Vector size: {info.config.params.vectors.size}")
    print(f"   - Distance: {info.config.params.vectors.distance}")

    # Optionally configure HNSW parameters for optimal search performance (from T085)
    print(f"\n🔧 Configuring HNSW parameters for optimal search...")
    client.update_collection(
        collection_name=collection_name,
        hnsw_config={
            "m": 16,  # Number of edges per node
            "ef_construct": 100,  # Size of the dynamic candidate list
        },
    )
    print(f"✅ HNSW parameters configured (m=16, ef_construct=100)")

    print("\n" + "=" * 60)
    print("✅ Qdrant setup complete!")
    print("=" * 60)

    return True


if __name__ == "__main__":
    try:
        success = setup_qdrant_collection()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n❌ Error setting up Qdrant: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
