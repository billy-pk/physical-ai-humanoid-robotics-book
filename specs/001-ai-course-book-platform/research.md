# Research: AI-Course-Book Platform

**Date**: 2025-11-28
**Feature**: 001-ai-course-book-platform

## Overview

This document consolidates research findings for implementing the AI-Course-Book Platform. All major technology choices (Docusaurus, FastAPI, OpenAI Agents SDK, Qdrant, Neon Postgres) are mandated by the project constitution. Research focuses on best practices, implementation patterns, and integration strategies.

## Technology Stack Decisions

### 1. Frontend Framework: Docusaurus

**Decision**: Use Docusaurus for documentation website

**Rationale**:
- Mandated by project constitution and spec
- Purpose-built for documentation sites with excellent markdown support
- Built-in features: search, versioning, dark mode, mobile responsiveness
- React-based allowing custom component integration (ChatKit widget)
- Static site generation perfect for GitHub Pages deployment
- Large ecosystem and active community

**Implementation Notes**:
- Use Docusaurus v3 (latest stable)
- Custom React component for ChatKit integration
- Configure `docusaurus.config.js` for GitHub Pages base URL
- Use swizzling sparingly to maintain upgradability

### 2. Backend Framework: FastAPI

**Decision**: Use FastAPI with Python 3.12

**Rationale**:
- Mandated by project constitution
- High performance async framework ideal for I/O-bound RAG operations
- Automatic OpenAPI documentation generation
- Built-in validation with Pydantic models
- Native async/await support for concurrent requests
- Excellent type hinting support

**Implementation Notes**:
- Use async route handlers for all endpoints
- Leverage dependency injection for database clients (Qdrant, Neon)
- Implement middleware for CORS, rate limiting, structured logging
- Use Pydantic BaseSettings for environment configuration
- Stream responses using `StreamingResponse` for chatbot

### 3. RAG Implementation: OpenAI Agents SDK

**Decision**: OpenAI Agents SDK for RAG chat functionality

**Rationale**:
- Mandated by spec (RAG-T-004)
- Provides high-level abstractions for RAG patterns
- Built-in support for function calling and tool integration
- Handles conversation history and context management
- Integrates seamlessly with OpenAI embeddings and completions

**Alternative Considered**: OpenAI ChatKit
- ChatKit is primarily a frontend SDK
- Agents SDK is more suitable for backend RAG logic
- Can use ChatKit for frontend widget while Agents SDK handles backend

**Implementation Pattern**:
```python
# Backend uses Agents SDK for RAG
from openai import OpenAI
client = OpenAI()

# Define RAG function for vector search
def search_documentation(query: str) -> list[str]:
    # Query Qdrant for relevant chunks
    # Return context to agent
    pass

# Agent uses function calling to retrieve context
response = client.chat.completions.create(
    model="gpt-4",
    messages=[{"role": "user", "content": query}],
    functions=[{"name": "search_documentation", ...}],
    stream=True
)
```

### 4. Vector Database: Qdrant Cloud

**Decision**: Qdrant Cloud (Free Tier)

**Rationale**:
- Mandated by spec (RAG-T-002)
- Purpose-built for vector similarity search
- Cloud offering eliminates infrastructure management
- Free tier sufficient for documentation-scale content
- Python client with async support
- Supports dense vectors from OpenAI embeddings

**Implementation Notes**:
- Collection configuration:
  - Vector size: 3072 (text-embedding-3-large dimension)
  - Distance metric: Cosine similarity
  - Optimize for search performance vs storage
- Metadata fields: chapter_id, section_title, chunk_index, source_url
- Use payload filtering for highlighted text constraints (RAG-C-002)
- Implement connection pooling and retry logic

### 5. Metadata Database: Neon Serverless Postgres

**Decision**: Neon Serverless Postgres (Free Tier)

**Rationale**:
- Mandated by spec (RAG-T-003)
- Serverless model matches variable traffic patterns
- Generous free tier for metadata/session storage
- Standard Postgres compatibility (use psycopg3 async)
- Auto-scaling and auto-suspend reduce costs
- Branch-based development workflow

**Schema Design**:
```sql
-- Chat sessions table
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP DEFAULT NOW(),
    last_activity TIMESTAMP DEFAULT NOW(),
    metadata JSONB
);

-- Chat messages table
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES chat_sessions(id),
    role VARCHAR(20) NOT NULL, -- 'user' or 'assistant'
    content TEXT NOT NULL,
    citations JSONB, -- Array of source references
    highlighted_context TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Metrics table for observability
CREATE TABLE api_metrics (
    id BIGSERIAL PRIMARY KEY,
    timestamp TIMESTAMP DEFAULT NOW(),
    endpoint VARCHAR(255),
    status_code INT,
    response_time_ms INT,
    error_message TEXT
);
```

### 6. Embeddings: OpenAI text-embedding-3-large

**Decision**: OpenAI text-embedding-3-large model

**Rationale**:
- Mandated by spec (RAG-T-001)
- State-of-the-art embedding quality
- 3072 dimensions provide rich semantic representation
- Cost-effective at $0.13 per 1M tokens
- Native support in OpenAI SDK

**Chunking Strategy**:
- Chunk size: 500-800 tokens (balance context vs granularity)
- Overlap: 100 tokens to preserve context boundaries
- Preserve markdown structure (don't split code blocks, lists)
- Include section headers in chunk metadata
- Implement deduplication (RAG-P-002) using content hashing

### 7. Dependency Management: uv

**Decision**: Use `uv` for Python dependency management

**Rationale**:
- Mandated by spec (DM-001)
- Modern, fast alternative to pip/poetry
- Simplified dependency resolution
- Virtual environment management
- Lock file for reproducible builds

**Setup**:
```bash
# Initialize backend
cd backend
uv init
uv add fastapi uvicorn[standard] openai qdrant-client psycopg[binary] python-dotenv pydantic-settings

# Development dependencies
uv add --dev pytest pytest-asyncio pytest-cov ruff httpx
```

### 8. Code Quality: Ruff + ESLint/Prettier

**Decision**: Ruff for Python, ESLint+Prettier for TypeScript

**Rationale**:
- Mandated by constitution (Code Quality Principles II)
- Ruff: Fast, comprehensive Python linter (PEP8 + flake8 rules)
- ESLint: Industry standard JavaScript/TypeScript linter
- Prettier: Opinionated code formatter for consistency

**Configuration**:
- Ruff: `pyproject.toml` with line length 88, target Python 3.12
- ESLint: Extend recommended rules + TypeScript rules
- Prettier: Integrated with ESLint via `eslint-config-prettier`

## Integration Patterns

### Frontend-Backend Communication

**Pattern**: Direct HTTP calls from ChatKit widget to FastAPI backend

**Flow**:
1. User types question in ChatKit widget
2. Widget sends POST to `/api/chat` with query + optional highlighted text
3. Backend retrieves relevant chunks from Qdrant
4. Backend generates response using OpenAI Agents SDK
5. Backend streams response back to frontend
6. Widget displays response with citations

**CORS Configuration**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://<username>.github.io"],  # GitHub Pages domain
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)
```

### Rate Limiting Strategy

**Decision**: Simple in-memory rate limiting (BE-017)

**Implementation**: Use `slowapi` library
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.post("/api/chat")
@limiter.limit("10/minute")  # 10 requests per minute per IP
async def chat_endpoint(request: Request, query: ChatQuery):
    ...
```

**Rationale**:
- Public API (no auth) requires abuse prevention
- IP-based limiting sufficient for free-tier service
- In-memory state acceptable (Render restarts reset limits)
- Consider Redis if rate limit sharing across instances needed

### Error Handling Pattern

**Decision**: Structured JSON errors with specific messages (RAG-C-004, BE-009)

**Implementation**:
```python
class ErrorResponse(BaseModel):
    error: str
    message: str
    request_id: str
    timestamp: datetime

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    if isinstance(exc, QdrantException):
        return JSONResponse(
            status_code=503,
            content=ErrorResponse(
                error="service_unavailable",
                message="Chat is temporarily unavailable, please try again later",
                request_id=str(request.state.request_id),
                timestamp=datetime.utcnow()
            ).dict()
        )
    # ... handle other exceptions
```

### Observability Implementation

**Decision**: Structured logging + metrics endpoint (OBS-001 to OBS-004)

**Logging**:
```python
import structlog

logger = structlog.get_logger()

@app.middleware("http")
async def log_requests(request: Request, call_next):
    start_time = time.time()
    response = await call_next(request)
    duration_ms = (time.time() - start_time) * 1000

    logger.info(
        "request_completed",
        method=request.method,
        path=request.url.path,
        status_code=response.status_code,
        duration_ms=duration_ms,
        request_id=request.state.request_id
    )
    return response
```

**Metrics Endpoint**:
```python
from collections import Counter, defaultdict

metrics = {
    "request_count": Counter(),
    "error_count": Counter(),
    "status_codes": Counter()
}

@app.get("/metrics")
async def get_metrics():
    return {
        "total_requests": sum(metrics["request_count"].values()),
        "error_rate": sum(metrics["error_count"].values()) / max(sum(metrics["request_count"].values()), 1),
        "status_code_distribution": dict(metrics["status_codes"])
    }
```

## Deployment Strategies

### Frontend Deployment: GitHub Pages

**Process**:
1. GitHub Actions workflow triggers on push to main
2. Build Docusaurus site: `npm run build`
3. Deploy to `gh-pages` branch using `peaceiris/actions-gh-pages@v3`
4. GitHub Pages serves static site

**Configuration**:
```yaml
# .github/workflows/frontend-ci.yml
name: Deploy Frontend
on:
  push:
    branches: [main]
    paths: ['frontend/**']
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      - run: cd frontend && npm ci && npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./frontend/build
```

### Backend Deployment: Render

**Process**:
1. Connect GitHub repository to Render
2. Configure auto-deploy on push to main
3. Render builds using `uv sync` and starts with `uv run uvicorn main:app`
4. Environment variables configured in Render dashboard

**Render Configuration** (`render.yaml`):
```yaml
services:
  - type: web
    name: ai-course-book-backend
    env: python
    region: oregon
    plan: free
    buildCommand: "uv sync"
    startCommand: "uv run uvicorn src.main:app --host 0.0.0.0 --port $PORT"
    envVars:
      - key: PYTHON_VERSION
        value: "3.12"
      - key: OPENAI_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: NEON_DATABASE_URL
        sync: false
```

## Testing Strategies

### Backend Testing

**Unit Tests** (pytest):
- Test RAG service logic in isolation with mocked Qdrant/OpenAI
- Test data models and validation
- Test utility functions (chunking, deduplication)
- Target: 70% coverage minimum

**Integration Tests**:
- Test API endpoints with real database (use test Qdrant collection)
- Test error handling scenarios
- Test rate limiting
- Use `httpx.AsyncClient` for FastAPI testing

```python
# tests/integration/test_chat_api.py
import pytest
from httpx import AsyncClient

@pytest.mark.asyncio
async def test_chat_endpoint_success(client: AsyncClient):
    response = await client.post("/api/chat", json={
        "query": "What is Physical AI?"
    })
    assert response.status_code == 200
    data = response.json()
    assert "response" in data
    assert "citations" in data
```

### Frontend Testing

**Component Tests** (Jest + React Testing Library):
- Test ChatWidget component rendering
- Test user interactions (click to expand, send message)
- Test error states display
- Mock backend API calls

```typescript
// src/components/ChatWidget/ChatWidget.test.tsx
import { render, screen, fireEvent } from '@testing-library/react';
import ChatWidget from './ChatWidget';

test('expands when clicked', () => {
  render(<ChatWidget />);
  const widget = screen.getByRole('button', { name: /chat/i });
  fireEvent.click(widget);
  expect(screen.getByRole('textbox')).toBeInTheDocument();
});
```

## Security Considerations

1. **API Keys**: Never commit to git, use environment variables (BE-011)
2. **CORS**: Restrict to GitHub Pages domain only
3. **Rate Limiting**: Prevent abuse of public API (BE-017)
4. **Input Validation**: Sanitize user queries before processing
5. **Error Messages**: Don't leak internal details in error responses
6. **Dependency Scanning**: Use Dependabot for automated security updates

## Performance Optimizations

1. **Caching Layer** (RAG-T-005):
   - Cache frequent queries with TTL
   - Use Redis or in-memory cache for small scale
   - Cache embeddings to avoid re-computing for same text

2. **Connection Pooling**:
   - Reuse Qdrant and Postgres connections
   - Configure appropriate pool sizes in FastAPI lifespan

3. **Chunk Deduplication** (RAG-P-002):
   - Hash chunk content before embedding
   - Skip embedding if hash exists in database
   - Store hash in metadata for lookup

4. **Async Operations**:
   - Use async/await throughout stack
   - Parallel Qdrant search + metadata lookup
   - Stream OpenAI responses to reduce perceived latency

## Open Questions & Decisions

All technical decisions have been resolved based on constitution and specification requirements. No open questions remain for implementation.

## References

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [OpenAI Agents SDK](https://platform.openai.com/docs/assistants/overview)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Neon Postgres Documentation](https://neon.tech/docs/introduction)
- [uv Documentation](https://github.com/astral-sh/uv)
- [Ruff Documentation](https://docs.astral.sh/ruff/)
