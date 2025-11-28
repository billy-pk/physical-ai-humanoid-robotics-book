# Data Model: AI-Course-Book Platform

**Date**: 2025-11-28
**Feature**: 001-ai-course-book-platform

## Overview

This document defines the data models for the AI-Course-Book Platform, covering both application entities (chat sessions, messages, embeddings) and database schemas (Postgres for metadata, Qdrant for vectors).

## Entity Definitions

### 1. Documentation Chapter

**Description**: Represents a single book chapter with learning content

**Attributes**:
- `id`: UUID - Unique identifier
- `title`: String - Chapter title
- `module`: String - Module/category this chapter belongs to
- `content`: Markdown Text - Full chapter content
- `learning_outcomes`: List[String] - Learning objectives for the chapter
- `created_at`: Timestamp - Creation timestamp
- `updated_at`: Timestamp - Last modification timestamp
- `version`: String - Chapter version (for tracking updates)
- `author`: String - Chapter author/generator

**Relationships**:
- Has many `Embedding Chunks` (one chapter → many chunks)

**Validation Rules**:
- Title is required and must be unique
- Module is required
- Content must be non-empty
- Must include at least one learning outcome

**State Transitions**:
- Draft → Published → Updated → Archived

**Storage**: Git repository as Markdown files in `frontend/docs/`

---

### 2. Chat Session

**Description**: Represents a conversation between a user and the chatbot

**Attributes**:
- `id`: UUID - Unique identifier
- `created_at`: Timestamp - Session start time
- `last_activity`: Timestamp - Last message timestamp
- `user_context`: JSON - Optional user context (page URL, highlighted text)
- `metadata`: JSON - Additional session metadata

**Relationships**:
- Has many `Chat Messages` (one session → many messages)

**Validation Rules**:
- ID must be unique
- Timestamps must be valid
- last_activity >= created_at

**State Transitions**:
- Active → Inactive (after timeout)

**Storage**: Neon Postgres (`chat_sessions` table)

**Database Schema**:
```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_activity TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    user_context JSONB DEFAULT '{}',
    metadata JSONB DEFAULT '{}',
    CONSTRAINT valid_activity_time CHECK (last_activity >= created_at)
);

CREATE INDEX idx_chat_sessions_last_activity ON chat_sessions(last_activity);
```

---

### 3. Chat Message

**Description**: Represents a single message in a chat conversation (user question or bot response)

**Attributes**:
- `id`: UUID - Unique identifier
- `session_id`: UUID - Reference to chat session
- `role`: Enum['user', 'assistant'] - Message sender role
- `content`: Text - Message content
- `citations`: List[Citation] - Source references (assistant messages only)
- `highlighted_context`: Text (optional) - User-selected text (user messages only)
- `created_at`: Timestamp - Message timestamp
- `tokens_used`: Integer (optional) - Token count for cost tracking

**Relationships**:
- Belongs to one `Chat Session`
- References multiple `Embedding Chunks` through citations

**Validation Rules**:
- Role must be 'user' or 'assistant'
- Content is required and non-empty
- Citations only present for assistant messages
- Highlighted context only for user messages

**Storage**: Neon Postgres (`chat_messages` table)

**Database Schema**:
```sql
CREATE TYPE message_role AS ENUM ('user', 'assistant');

CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role message_role NOT NULL,
    content TEXT NOT NULL,
    citations JSONB DEFAULT '[]',
    highlighted_context TEXT,
    tokens_used INTEGER,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    CONSTRAINT non_empty_content CHECK (LENGTH(content) > 0),
    CONSTRAINT valid_role_citations CHECK (
        (role = 'assistant' AND citations IS NOT NULL) OR
        (role = 'user' AND citations = '[]')
    )
);

CREATE INDEX idx_chat_messages_session ON chat_messages(session_id, created_at);
CREATE INDEX idx_chat_messages_role ON chat_messages(role);
```

**Citation Structure** (JSON):
```json
{
  "chunk_id": "uuid",
  "chapter_title": "Introduction to Physical AI",
  "section": "What is Physical AI?",
  "relevance_score": 0.95,
  "source_url": "/docs/intro#what-is-physical-ai"
}
```

---

### 4. Embedding Chunk

**Description**: A segment of documentation that has been vectorized for similarity search

**Attributes**:
- `id`: UUID - Unique identifier (Qdrant point ID)
- `content`: Text - Chunk text content
- `content_hash`: String - SHA-256 hash for deduplication
- `vector`: Float[3072] - Embedding vector from text-embedding-3-large
- `chapter_id`: String - Source chapter identifier
- `chapter_title`: String - Source chapter title
- `section_title`: String (optional) - Section within chapter
- `chunk_index`: Integer - Sequential position within chapter
- `token_count`: Integer - Number of tokens in chunk
- `created_at`: Timestamp - Embedding creation time
- `metadata`: JSON - Additional chunk metadata

**Relationships**:
- Belongs to one `Documentation Chapter`
- Referenced by `Chat Messages` through citations

**Validation Rules**:
- Content is required and non-empty
- Vector dimension must be exactly 3072
- Chunk index >= 0
- Content hash must match SHA-256(content)

**Storage**: Qdrant vector database

**Qdrant Collection Configuration**:
```python
from qdrant_client.models import Distance, VectorParams, PointStruct

collection_config = {
    "vectors": VectorParams(
        size=3072,
        distance=Distance.COSINE
    ),
    "optimizers_config": {
        "default_segment_number": 2
    },
    "hnsw_config": {
        "m": 16,
        "ef_construct": 100
    }
}

# Point structure
point = PointStruct(
    id=str(uuid.uuid4()),
    vector=embedding_vector,  # List[float] with length 3072
    payload={
        "content": "chunk text content...",
        "content_hash": "sha256_hash_value",
        "chapter_id": "intro",
        "chapter_title": "Introduction to Physical AI",
        "section_title": "What is Physical AI?",
        "chunk_index": 0,
        "token_count": 450,
        "source_url": "/docs/intro#what-is-physical-ai",
        "created_at": "2025-11-28T10:00:00Z",
        "metadata": {}
    }
)
```

**Deduplication Strategy**:
1. Compute SHA-256 hash of chunk content
2. Query Qdrant for existing chunks with same hash
3. Skip embedding if hash exists
4. Store hash in payload for future lookups

---

### 5. Vector Search Result

**Description**: Result from Qdrant similarity search (transient, not persisted)

**Attributes**:
- `chunk_id`: UUID - Embedding chunk identifier
- `content`: Text - Chunk content
- `score`: Float [0.0-1.0] - Similarity/relevance score
- `chapter_title`: String - Source chapter
- `section_title`: String - Source section
- `source_url`: String - Link to documentation
- `metadata`: JSON - Additional context

**Relationships**:
- Maps to `Embedding Chunk` by ID
- Used to construct `Citation` objects in chat messages

**Validation Rules**:
- Score must be between 0.0 and 1.0
- Chunk ID must reference valid embedding

**Storage**: Temporary (API response only, not persisted)

**Python Model**:
```python
from pydantic import BaseModel, Field

class VectorSearchResult(BaseModel):
    chunk_id: str
    content: str
    score: float = Field(ge=0.0, le=1.0)
    chapter_title: str
    section_title: str | None = None
    source_url: str
    metadata: dict = {}
```

---

### 6. API Metrics

**Description**: Observability metrics for monitoring API performance

**Attributes**:
- `id`: BigSerial - Auto-incrementing identifier
- `timestamp`: Timestamp - Request timestamp
- `endpoint`: String - API endpoint path
- `method`: String - HTTP method
- `status_code`: Integer - HTTP response status
- `response_time_ms`: Integer - Response time in milliseconds
- `error_message`: Text (optional) - Error details if request failed
- `request_id`: UUID - Unique request identifier for tracing

**Relationships**: None (metrics are independent records)

**Validation Rules**:
- Status code must be valid HTTP status (100-599)
- Response time must be >= 0
- Error message only present if status code >= 400

**Storage**: Neon Postgres (`api_metrics` table)

**Database Schema**:
```sql
CREATE TABLE api_metrics (
    id BIGSERIAL PRIMARY KEY,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    endpoint VARCHAR(255) NOT NULL,
    method VARCHAR(10) NOT NULL,
    status_code INTEGER NOT NULL,
    response_time_ms INTEGER NOT NULL,
    error_message TEXT,
    request_id UUID NOT NULL,
    CONSTRAINT valid_status_code CHECK (status_code >= 100 AND status_code < 600),
    CONSTRAINT valid_response_time CHECK (response_time_ms >= 0),
    CONSTRAINT error_on_failure CHECK (
        (status_code >= 400 AND error_message IS NOT NULL) OR
        (status_code < 400)
    )
);

CREATE INDEX idx_api_metrics_timestamp ON api_metrics(timestamp DESC);
CREATE INDEX idx_api_metrics_endpoint ON api_metrics(endpoint, timestamp DESC);
CREATE INDEX idx_api_metrics_status ON api_metrics(status_code, timestamp DESC);
```

**Retention Policy**: Keep last 30 days, delete older records

---

## Data Flow Diagrams

### Chat Query Flow

```
User Query (Frontend)
    ↓
POST /api/chat {query, highlighted_context?, session_id?}
    ↓
Backend: Create/Retrieve Chat Session
    ↓
Backend: Create User Message (role='user')
    ↓
Backend: Generate Embedding for Query
    ↓
Qdrant: Vector Similarity Search
    ← Returns top-k chunks with scores
    ↓
Backend: Retrieve Metadata from Postgres (if needed)
    ↓
Backend: Construct Context from Chunks
    ↓
OpenAI Agents SDK: Generate Response
    ← Streams response tokens
    ↓
Backend: Create Assistant Message (role='assistant', citations)
    ↓
Backend: Stream Response to Frontend
    ↓
Frontend: Display Response with Citations
```

### Content Ingestion Flow

```
New/Updated Chapter (Markdown file)
    ↓
Ingestion Script: Read Chapter Content
    ↓
Script: Split into Chunks (500-800 tokens, 100 overlap)
    ↓
Script: Compute Content Hash for Each Chunk
    ↓
Qdrant: Check for Existing Hash
    ↓ (if new)
OpenAI: Generate Embedding (text-embedding-3-large)
    ← Returns 3072-dim vector
    ↓
Qdrant: Store Point (vector + payload)
    ↓
Script: Log Ingestion Stats
```

## Database Migrations

**Tool**: Alembic for Postgres schema migrations

**Migration Strategy**:
1. Create migration for initial schema
2. Apply migrations automatically on deployment
3. Version migrations in Git
4. Test migrations against Neon branch databases

**Initial Migration**:
```bash
# backend/alembic/versions/001_initial_schema.py
def upgrade():
    # Create enums
    op.execute("CREATE TYPE message_role AS ENUM ('user', 'assistant')")

    # Create tables
    op.create_table('chat_sessions', ...)
    op.create_table('chat_messages', ...)
    op.create_table('api_metrics', ...)

    # Create indexes
    ...

def downgrade():
    op.drop_table('api_metrics')
    op.drop_table('chat_messages')
    op.drop_table('chat_sessions')
    op.execute("DROP TYPE message_role")
```

## Indexing Strategy

### Postgres Indexes

1. **chat_sessions**:
   - Primary key (id): B-tree
   - last_activity: B-tree (for cleanup queries)

2. **chat_messages**:
   - Primary key (id): B-tree
   - (session_id, created_at): B-tree composite (for message history)
   - role: B-tree (for analytics)

3. **api_metrics**:
   - Primary key (id): B-tree
   - timestamp DESC: B-tree (for recent metrics)
   - (endpoint, timestamp DESC): B-tree composite (per-endpoint metrics)
   - (status_code, timestamp DESC): B-tree composite (error tracking)

### Qdrant Indexes

- **HNSW Index**: Automatically maintained for vector similarity search
  - M=16: Connections per node (balance between speed and accuracy)
  - ef_construct=100: Index build time parameter
  - ef=64 (search time): Runtime search parameter

## Data Retention Policies

1. **Chat Sessions/Messages**:
   - Retention: Indefinite (small volume)
   - Cleanup: Optional background job to delete sessions inactive > 90 days

2. **API Metrics**:
   - Retention: 30 days rolling window
   - Cleanup: Daily cron job to delete records older than 30 days

3. **Embedding Chunks**:
   - Retention: Indefinite (tied to documentation lifecycle)
   - Update: Re-embed when chapter content changes

## Backup Strategy

1. **Neon Postgres**:
   - Automatic backups by Neon (point-in-time recovery)
   - Export critical data monthly to S3 (optional)

2. **Qdrant Cloud**:
   - Snapshots managed by Qdrant Cloud
   - Can export collection to local file if needed

3. **Documentation Content**:
   - Git repository is source of truth
   - GitHub backups via Git history

## Security Considerations

1. **Data Encryption**:
   - In-transit: TLS for all database connections
   - At-rest: Managed by Neon and Qdrant Cloud

2. **Access Control**:
   - Database credentials in environment variables only
   - Minimum privilege principle for database users
   - No direct database access from frontend

3. **PII Handling**:
   - No personally identifiable information stored
   - Session IDs are UUIDs (not linkable to users)
   - Chat content is ephemeral (can be purged)

## Performance Considerations

1. **Query Optimization**:
   - Use connection pooling (asyncpg for Postgres)
   - Batch operations where possible
   - Limit result sets appropriately

2. **Caching**:
   - Cache frequent queries in-memory (TTL: 5 minutes)
   - Cache embeddings to avoid re-computation
   - Use Qdrant payload caching for repeated searches

3. **Scaling**:
   - Postgres: Neon auto-scales compute
   - Qdrant: Upgrade to paid tier if free tier exhausted
   - Frontend: Static site scales with GitHub Pages CDN

## References

- [Qdrant Collection Configuration](https://qdrant.tech/documentation/concepts/collections/)
- [Neon Postgres Documentation](https://neon.tech/docs/introduction)
- [Alembic Migrations](https://alembic.sqlalchemy.org/en/latest/)
- [Pydantic Models](https://docs.pydantic.dev/)
