# Vector Database Management Guide

**Last Updated**: 2025-11-30  
**Status**: Production

---

## Overview

The RAG (Retrieval Augmented Generation) chatbot uses a vector database to search and retrieve relevant book content. This guide covers how to manage, update, and troubleshoot the vector database.

### System Architecture

```
Book Content (Markdown)
    ↓
Chunking (by H2/H3 sections)
    ↓
OpenAI Embeddings (text-embedding-3-large, 3072 dims)
    ↓
Qdrant Vector Database
    ↓
RAG Agent (GPT-4o-mini with search tool)
    ↓
User Queries
```

---

## When to Update Vector Database

Update the vector database whenever you:
- ✅ Add new chapters to any module
- ✅ Modify existing chapter content
- ✅ Create new modules
- ✅ Fix technical content or code examples
- ✅ Update learning objectives or explanations

**Note**: You do NOT need to update for:
- ❌ Frontend UI changes
- ❌ Authentication code changes
- ❌ Backend API changes (unless they affect documentation)
- ❌ Styling or CSS updates

---

## Prerequisites

### Required Environment Variables

Ensure `backend/.env` contains:

```bash
OPENAI_API_KEY=sk-...              # For generating embeddings
QDRANT_URL=https://...             # Qdrant Cloud endpoint
QDRANT_API_KEY=...                 # Qdrant authentication
NEON_DATABASE_URL=postgresql://... # Database connection
```

### Check Configuration

```bash
cd backend
grep -E "OPENAI_API_KEY|QDRANT" .env
```

All keys should be present and non-empty.

---

## Updating Vector Database

### Step 1: Run Ingestion Script

```bash
cd backend
uv run python ingest_content.py
```

### What Happens

The script performs these operations:

1. **Discovery**: Finds all `frontend/docs/module-*` directories
2. **Reading**: Reads markdown files (`.md`)
3. **Chunking**: 
   - Splits by H2 headers (`##`)
   - Further splits large sections by H3 headers (`###`)
   - Preserves chapter context in each chunk
4. **Embedding**: 
   - Calls OpenAI API for each chunk
   - Generates 3072-dimensional vectors
5. **Upload**: 
   - Batches chunks (50 per batch)
   - Uploads to Qdrant collection `book_embeddings`

### Expected Output

```
2025-11-30 13:42:47 [info] Found 5 modules: ['module-0', 'module-1', ...]
2025-11-30 13:42:47 [info] Processing module-0...
2025-11-30 13:42:47 [info] Found 4 files in module-0
...
2025-11-30 13:57:14 [info] ✅ Successfully uploaded 539 chunks to Qdrant!
2025-11-30 13:57:14 [info] INGESTION SUMMARY
2025-11-30 13:57:14 [info] Total chunks created: 539
2025-11-30 13:57:14 [info]   module-0: 80 chunks
2025-11-30 13:57:14 [info]   module-1: 107 chunks
2025-11-30 13:57:14 [info]   module-2: 110 chunks
2025-11-30 13:57:14 [info]   module-3: 114 chunks
2025-11-30 13:57:14 [info]   module-4: 128 chunks
```

### Processing Time

- **Small updates** (1-2 chapters): 1-2 minutes
- **Single module**: 2-4 minutes
- **All modules** (5 modules, ~28 files): 7-10 minutes

---

## Verification

### Step 2: Verify Upload Success

Run this command to check the collection:

```bash
cd backend
uv run python -c "
from src.services.vectordb.qdrant_client import get_qdrant_client

client = get_qdrant_client()
collection_info = client.get_collection('book_embeddings')

print(f'✓ Collection: book_embeddings')
print(f'✓ Dimensions: {collection_info.config.params.vectors.size}')
print(f'✓ Total vectors: {collection_info.points_count}')
print(f'✓ Status: {collection_info.status}')
"
```

**Expected output**:
```
✓ Collection: book_embeddings
✓ Dimensions: 3072
✓ Total vectors: 539
✓ Status: green
```

### Step 3: Test RAG Search

Test that the chatbot can find your new content:

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

Or test via the frontend chat widget at:
http://localhost:3000/physical-ai-humanoid-robotics-book/

---

## Troubleshooting

### Error: "Payload too large (36MB > 32MB limit)"

**Cause**: Trying to upload all chunks at once exceeds Qdrant's payload limit.

**Solution**: The script now automatically batches uploads (50 chunks per batch). If you still see this error, reduce batch size:

Edit `backend/src/services/vectordb/qdrant_client.py`:
```python
await upsert_embeddings(all_chunks, batch_size=25)  # Reduce from 50 to 25
```

### Error: "OpenAI API rate limit exceeded"

**Cause**: Too many embedding requests in a short time.

**Solution**: 
1. Wait 1 minute and retry
2. For large updates, process modules individually

**Process single module**:
Edit `backend/ingest_content.py` and comment out modules:
```python
module_dirs = sorted([
    d for d in docs_dir.iterdir() 
    if d.is_dir() and d.name == 'module-3'  # Process only module-3
])
```

### Error: "Collection 'book_embeddings' does not exist"

**Cause**: First-time setup or collection was deleted.

**Solution**: Create the collection:
```bash
cd backend
uv run python setup_qdrant.py
```

Then re-run the ingestion script.

### Error: "Out of memory"

**Cause**: Processing too many files at once.

**Solution**: Process modules one at a time (see "Process single module" above).

### Warning: Very slow processing

**Possible causes**:
1. Network latency to OpenAI API
2. Large files with many chunks
3. API rate limiting (auto-retries happening)

**Normal speeds**:
- 1-2 seconds per chunk (embedding generation)
- Most time is spent on API calls, not local processing

---

## Advanced Operations

### Rebuild Entire Collection

To completely recreate the vector database:

1. **Delete old collection** (optional):
```bash
cd backend
uv run python -c "
from src.services.vectordb.qdrant_client import get_qdrant_client
client = get_qdrant_client()
client.delete_collection('book_embeddings')
print('Collection deleted')
"
```

2. **Create fresh collection**:
```bash
uv run python setup_qdrant.py
```

3. **Ingest all content**:
```bash
uv run python ingest_content.py
```

### Update Specific Module Only

Edit `ingest_content.py` temporarily:

```python
# Line ~220: Modify module filter
module_dirs = sorted([
    d for d in docs_dir.iterdir() 
    if d.is_dir() and d.name == 'module-4'  # Only process module-4
])
```

Then run:
```bash
uv run python ingest_content.py
```

**Note**: This adds/updates chunks but doesn't delete old chunks. For a clean update, delete and recreate the collection.

### Check Chunk Details

Inspect what's stored in a chunk:

```bash
cd backend
uv run python -c "
from src.services.vectordb.qdrant_client import get_qdrant_client

client = get_qdrant_client()

# Search for a specific term
from src.services.llm.openai_client import generate_embeddings
import asyncio

async def search():
    query_vec = await generate_embeddings('ROS 2')
    results = client.query_points(
        collection_name='book_embeddings',
        query=query_vec,
        limit=3,
        with_payload=True
    )
    for i, point in enumerate(results.points, 1):
        print(f'\n--- Result {i} (score: {point.score:.3f}) ---')
        print(f'Chapter: {point.payload[\"chapter_title\"]}')
        print(f'Section: {point.payload[\"section_title\"]}')
        print(f'Content preview: {point.payload[\"content\"][:200]}...')

asyncio.run(search())
"
```

---

## Batch Operations

### Update All Modules (Production)

For production deployments:

```bash
cd backend

# 1. Backup current state (optional)
uv run python -c "
from src.services.vectordb.qdrant_client import get_qdrant_client
client = get_qdrant_client()
info = client.get_collection('book_embeddings')
with open('vector_db_backup_info.txt', 'w') as f:
    f.write(f'Points: {info.points_count}\n')
    f.write(f'Status: {info.status}\n')
print('Backup info saved')
"

# 2. Run full ingestion
uv run python ingest_content.py

# 3. Verify
uv run python -c "from src.services.vectordb.qdrant_client import get_qdrant_client; print(f'Vectors: {get_qdrant_client().get_collection(\"book_embeddings\").points_count}')"
```

---

## Monitoring & Maintenance

### Check Collection Health

```bash
cd backend
uv run python -c "
from src.services.vectordb.qdrant_client import get_qdrant_client

client = get_qdrant_client()
collections = client.get_collections()

print('Available collections:')
for col in collections.collections:
    info = client.get_collection(col.name)
    print(f'  - {col.name}: {info.points_count} vectors, status: {info.status}')
"
```

### Estimate Costs

**OpenAI Embedding Costs** (as of 2024):
- `text-embedding-3-large`: ~$0.13 per 1M tokens
- Average chunk: ~500 tokens
- 539 chunks ≈ 270K tokens
- **Cost**: ~$0.04 per full ingestion

**Qdrant Cloud**:
- Free tier: 1GB storage
- Current usage: ~539 vectors × 3072 dimensions × 4 bytes ≈ 6.6MB
- Well within free tier limits

---

## Quick Reference

| Task | Command |
|------|---------|
| Update all modules | `cd backend && uv run python ingest_content.py` |
| Check vector count | `cd backend && uv run python -c "from src.services.vectordb.qdrant_client import get_qdrant_client; print(get_qdrant_client().get_collection('book_embeddings').points_count)"` |
| Test RAG search | `curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query":"test"}'` |
| Create collection | `cd backend && uv run python setup_qdrant.py` |
| Delete collection | `cd backend && uv run python -c "from src.services.vectordb.qdrant_client import get_qdrant_client; get_qdrant_client().delete_collection('book_embeddings')"` |

---

## File References

- **Ingestion Script**: `backend/ingest_content.py`
- **Qdrant Client**: `backend/src/services/vectordb/qdrant_client.py`
- **Embedding Service**: `backend/src/services/llm/openai_client.py`
- **RAG Agent**: `backend/src/services/rag/rag_agent.py`
- **Content Location**: `frontend/docs/module-*/`

---

## Support

For issues or questions:
1. Check `TROUBLESHOOTING.md`
2. Check `TESTING-GUIDE.md` for RAG testing procedures
3. Review backend logs for error details
4. Verify all environment variables are set correctly

