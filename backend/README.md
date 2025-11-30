# Backend

This directory contains the FastAPI backend for the AI-Course-Book platform.

## Setup

1. Navigate into the `backend/` directory.
2. Initialize `uv` if not already done: `uv init .`
3. Install dependencies: `uv sync`
4. Set environment variables in a `.env` file (see `.env.example`).

## Running the application

`uv run uvicorn src.main:app --host 0.0.0.0 --port 8000`

---

## Vector Database & RAG Chatbot

### Overview

The backend uses a RAG (Retrieval Augmented Generation) system to power the AI chatbot:
- **Embeddings**: OpenAI `text-embedding-3-large` (3072 dimensions)
- **Vector Database**: Qdrant Cloud
- **LLM Agent**: OpenAI Agents SDK with GPT-4o-mini

### Updating Vector Database

When you add or modify book content in `frontend/docs/module-*`, you need to regenerate embeddings and update the vector database.

#### Prerequisites

Ensure these environment variables are set in `.env`:
- `OPENAI_API_KEY` - For generating embeddings
- `QDRANT_URL` - Your Qdrant Cloud endpoint
- `QDRANT_API_KEY` - Qdrant authentication key
- `NEON_DATABASE_URL` - Database connection

#### Run Ingestion Script

```bash
cd backend
uv run python ingest_content.py
```

**What it does**:
1. Scans all `frontend/docs/module-*` directories
2. Chunks markdown files by H2/H3 sections
3. Generates embeddings for each chunk
4. Uploads to Qdrant in batches (50 chunks per batch)

**Expected output**:
- Processing time: ~5-10 minutes (depends on content size)
- Creates ~150-250 chunks per module
- Uploads in batches to avoid payload size limits

#### Verify Upload

Check that embeddings were uploaded successfully:

```bash
cd backend
uv run python -c "
from src.services.vectordb.qdrant_client import get_qdrant_client
client = get_qdrant_client()
info = client.get_collection('book_embeddings')
print(f'Total vectors: {info.points_count}')
print(f'Status: {info.status}')
"
```

#### Troubleshooting

**Payload too large error**:
- The script automatically batches uploads (50 chunks per batch)
- If you still see errors, reduce batch size in `src/services/vectordb/qdrant_client.py`

**Out of memory**:
- Process modules individually by modifying `ingest_content.py`
- Comment out modules you don't need to reprocess

**API rate limits**:
- OpenAI has rate limits on embeddings API
- Script handles this automatically with retries
- Very large updates may take 10-15 minutes
