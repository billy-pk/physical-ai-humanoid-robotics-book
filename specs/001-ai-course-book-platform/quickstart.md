# Quickstart Guide: AI-Course-Book Platform

**Date**: 2025-11-28
**Feature**: 001-ai-course-book-platform

## Overview

This quickstart guide walks you through setting up and running the AI-Course-Book Platform locally, deploying to production, and understanding the two-phase implementation approach.

## Prerequisites

**Required**:
- Git
- Node.js 18+ (LTS recommended)
- Python 3.12
- `uv` (Python dependency manager): Install via `curl -LsSf https://astral.sh/uv/install.sh | sh`

**External Services** (Free Tier):
- GitHub account (for GitHub Pages deployment)
- Render account (for backend deployment)
- OpenAI API key
- Qdrant Cloud account (vector database)
- Neon Postgres account (metadata database)

## Phase 1: Documentation Site Setup

### Step 1: Initialize Frontend

```bash
# Clone repository
git clone <repository-url>
cd physical-ai-humanoid-robotics-book

# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Start local development server
npm start
```

The documentation site will open at `http://localhost:3000`.

### Step 2: Create Book Content

Use Spec-Kit Plus to generate chapters:

```bash
# Generate a new chapter
/sp.specify "Chapter on Robot Kinematics covering forward kinematics, inverse kinematics, and workspace analysis with examples and exercises"

# Follow the prompts to generate content
# Content will be created as Markdown in frontend/docs/
```

### Step 3: Configure Docusaurus

Edit `frontend/docusaurus.config.js`:

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to building intelligent robots',
  url: 'https://<username>.github.io',
  baseUrl: '/physical-ai-humanoid-robotics-book/',
  organizationName: '<username>',
  projectName: 'physical-ai-humanoid-robotics-book',

  themeConfig: {
    navbar: {
      title: 'Physical AI Book',
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Learn',
        },
      ],
    },
    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },
  },
};
```

### Step 4: Build and Test Locally

```bash
cd frontend

# Build production site
npm run build

# Serve built site locally
npm run serve
```

Visit `http://localhost:3000` to preview the production build.

### Step 5: Deploy to GitHub Pages

```bash
# First-time setup: configure GitHub Pages in repository settings
# Settings → Pages → Source: Deploy from a branch → Branch: gh-pages

# Deploy using GitHub Actions (automatic on push to main)
git add .
git commit -m "feat: add documentation chapters"
git push origin main

# Or deploy manually
cd frontend
npm run deploy
```

Site will be live at `https://<username>.github.io/physical-ai-humanoid-robotics-book/`

## Phase 2: RAG Chatbot Integration

### Step 1: Backend Setup

```bash
# Navigate to backend directory
cd backend

# Initialize with uv
uv init

# Install dependencies
uv add fastapi uvicorn[standard] openai qdrant-client psycopg[binary] python-dotenv pydantic-settings

# Install development dependencies
uv add --dev pytest pytest-asyncio pytest-cov ruff httpx
```

### Step 2: Configure Environment Variables

Create `backend/.env`:

```bash
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant Cloud
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres
NEON_DATABASE_URL=postgres://user:password@host/database

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
RATE_LIMIT_PER_MINUTE=10

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://<username>.github.io
```

**Never commit `.env` to Git!** Copy `backend/.env.example` as a template.

### Step 3: Set Up Databases

**Qdrant Cloud**:
1. Create account at https://cloud.qdrant.io
2. Create a new cluster (Free Tier)
3. Create collection `book_embeddings`:
   ```python
   from qdrant_client import QdrantClient
   from qdrant_client.models import Distance, VectorParams

   client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

   client.create_collection(
       collection_name="book_embeddings",
       vectors_config=VectorParams(size=3072, distance=Distance.COSINE)
   )
   ```

**Neon Postgres**:
1. Create account at https://neon.tech
2. Create a new project (Free Tier)
3. Run migrations:
   ```bash
   cd backend
   uv run alembic upgrade head
   ```

### Step 4: Ingest Documentation Content

```bash
cd backend

# Run content ingestion script
uv run python scripts/ingest_content.py --docs-path ../frontend/docs

# Monitor progress
# - Chunks created
# - Embeddings generated
# - Vectors stored in Qdrant
```

### Step 5: Run Backend Locally

```bash
cd backend

# Start FastAPI server
uv run uvicorn src.main:app --reload --port 8000

# Server starts at http://localhost:8000
# API docs at http://localhost:8000/docs
```

### Step 6: Integrate ChatKit Widget

Edit `frontend/src/components/ChatWidget/ChatWidget.tsx`:

```typescript
import React, { useState } from 'react';
import { ChatMessage } from '@openai/chatkit-react';

export default function ChatWidget() {
  const [isExpanded, setIsExpanded] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);

  const handleSendMessage = async (query: string) => {
    const response = await fetch('http://localhost:8000/api/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ query }),
    });

    const data = await response.json();

    setMessages([
      ...messages,
      { role: 'user', content: query },
      { role: 'assistant', content: data.response, citations: data.citations }
    ]);
  };

  return (
    <div className="chat-widget" style={{ position: 'fixed', bottom: 20, right: 20 }}>
      {!isExpanded ? (
        <button onClick={() => setIsExpanded(true)}>💬 Chat</button>
      ) : (
        <div className="chat-interface">
          {/* Chat UI implementation */}
        </div>
      )}
    </div>
  );
}
```

Add widget to all pages in `frontend/src/theme/Root.tsx`:

```typescript
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

### Step 7: Test End-to-End

1. Start backend: `cd backend && uv run uvicorn src.main:app --reload`
2. Start frontend: `cd frontend && npm start`
3. Open browser to `http://localhost:3000`
4. Click chat widget (bottom-right)
5. Ask: "What is Physical AI?"
6. Verify response with citations appears

### Step 8: Deploy Backend to Render

**One-time Setup**:
1. Create Render account
2. Create new Web Service
3. Connect GitHub repository
4. Configure:
   - Build Command: `uv sync`
   - Start Command: `uv run uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - Environment: Python 3.12

**Environment Variables** (in Render dashboard):
- `OPENAI_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `NEON_DATABASE_URL`
- `ENVIRONMENT=production`
- `ALLOWED_ORIGINS=https://<username>.github.io`

**Automatic Deployment**:
- Render auto-deploys on push to `main` branch
- Or manually trigger deploy from Render dashboard

### Step 9: Update Frontend for Production

Edit `frontend/src/components/ChatWidget/ChatWidget.tsx`:

```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://ai-course-book-backend.onrender.com'
  : 'http://localhost:8000';

const response = await fetch(`${API_BASE_URL}/api/chat`, {
  // ... rest of the code
});
```

### Step 10: Deploy Frontend to GitHub Pages

```bash
cd frontend

# Build and deploy
npm run deploy

# Or commit and let GitHub Actions handle it
git add .
git commit -m "feat: integrate RAG chatbot with production backend"
git push origin main
```

## CI/CD Automation

### Backend CI/CD (`.github/workflows/backend-ci.yml`)

```yaml
name: Backend CI/CD

on:
  push:
    branches: [main]
    paths: ['backend/**']
  pull_request:
    paths: ['backend/**']

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Install uv
        run: curl -LsSf https://astral.sh/uv/install.sh | sh

      - name: Install dependencies
        run: cd backend && uv sync

      - name: Lint with ruff
        run: cd backend && uv run ruff check .

      - name: Run tests
        run: cd backend && uv run pytest --cov=src --cov-report=term

      - name: Check coverage
        run: cd backend && uv run pytest --cov=src --cov-fail-under=70

  deploy:
    needs: test
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest
    steps:
      - name: Trigger Render Deploy
        run: |
          curl -X POST ${{ secrets.RENDER_DEPLOY_HOOK_URL }}
```

### Frontend CI/CD (`.github/workflows/frontend-ci.yml`)

```yaml
name: Frontend CI/CD

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
          cache: 'npm'
          cache-dependency-path: frontend/package-lock.json

      - name: Install dependencies
        run: cd frontend && npm ci

      - name: Lint
        run: cd frontend && npm run lint

      - name: Build
        run: cd frontend && npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./frontend/build
```

## Verification Checklist

### Phase 1 Complete
- [ ] Documentation site loads at GitHub Pages URL
- [ ] All chapters are accessible and properly formatted
- [ ] Navigation works between pages
- [ ] Site is mobile-responsive
- [ ] Dark mode works

### Phase 2 Complete
- [ ] Backend health check responds: `curl https://ai-course-book-backend.onrender.com/health`
- [ ] Chat widget appears on all documentation pages
- [ ] Widget expands when clicked
- [ ] Questions receive AI-generated answers
- [ ] Responses include citations with links
- [ ] Highlighted text question works
- [ ] Error messages display correctly (try asking off-topic question)
- [ ] Rate limiting works (exceed 10 requests/minute)
- [ ] Metrics endpoint returns data: `curl https://ai-course-book-backend.onrender.com/metrics`

## Troubleshooting

### Frontend Issues

**Build fails**:
- Check Node.js version: `node --version` (should be 18+)
- Clear cache: `rm -rf frontend/node_modules frontend/build && cd frontend && npm install`
- Check for TypeScript errors: `npm run typecheck`

**Pages not loading**:
- Verify `baseUrl` in `docusaurus.config.js` matches repository name
- Check GitHub Pages settings: Settings → Pages → Source should be `gh-pages` branch

### Backend Issues

**FastAPI won't start**:
- Check Python version: `python --version` (should be 3.12)
- Verify `.env` file exists with all required variables
- Check uv installation: `uv --version`
- View logs: `uv run uvicorn src.main:app --log-level debug`

**Qdrant connection fails**:
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Test connection: `python -c "from qdrant_client import QdrantClient; print(QdrantClient(url='...', api_key='...').get_collections())"`

**Chat responses are slow**:
- Check OpenAI API quota/limits
- Verify Qdrant query performance (collection size, index configuration)
- Review backend logs for bottlenecks

### CORS Issues

**Frontend can't reach backend**:
- Verify `ALLOWED_ORIGINS` includes frontend URL
- Check browser console for CORS errors
- Test backend directly: `curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query":"test"}'`

### Deployment Issues

**Render deploy fails**:
- Check build logs in Render dashboard
- Verify environment variables are set
- Test build locally: `cd backend && uv sync && uv run uvicorn src.main:app`

**GitHub Pages deploy fails**:
- Check GitHub Actions logs
- Verify gh-pages branch exists
- Check repository permissions (Actions should have write access)

## Next Steps

1. **Generate More Content**: Use `/sp.specify` to create additional chapters
2. **Customize Theme**: Edit `frontend/src/css/custom.css` for branding
3. **Add Diagrams**: Include Mermaid diagrams in Markdown
4. **Monitor Usage**: Review `/metrics` endpoint for traffic patterns
5. **Optimize Performance**: Implement caching for frequent queries
6. **Add Analytics**: Integrate Google Analytics (optional)

## Support

- **Documentation**: See `README.md` files in `frontend/` and `backend/` directories
- **API Reference**: Visit `http://localhost:8000/docs` when backend is running
- **Issues**: Report bugs via GitHub Issues

## References

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Render Documentation](https://render.com/docs)
- [GitHub Pages Documentation](https://docs.github.com/en/pages)
