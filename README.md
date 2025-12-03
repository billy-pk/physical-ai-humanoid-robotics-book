# Physical AI and Humanoid Robotics Book

A comprehensive interactive learning platform for Physical AI and Humanoid Robotics, featuring AI-generated content and an intelligent RAG-powered chatbot.

## 🚀 Quick Start

### Prerequisites
- **Node.js** 20+ (for frontend and Better Auth service)
- **Python** 3.12+ (for backend)
- **uv** (Python package manager)
- **PostgreSQL** (Neon Serverless Postgres)
- **Qdrant Cloud** account (for vector database)
- **OpenAI API** key

### Running Locally

```bash
# Terminal 1: Better Auth Service
cd backend/auth-service
npm run dev

# Terminal 2: FastAPI Backend  
cd backend
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Terminal 3: Frontend
cd frontend
npm start
```

**Access**:
- Frontend: http://localhost:3000/physical-ai-humanoid-robotics-book/
- Backend API: http://localhost:8000
- Better Auth: http://localhost:3001

## 📚 Project Structure

```
physical-ai-humanoid-robotics-book/
├── frontend/              # Docusaurus documentation site
│   ├── docs/             # Book content (modules 0-4)
│   └── src/              # React components, auth UI
├── backend/              # FastAPI backend
│   ├── src/
│   │   ├── api/          # API routes
│   │   ├── services/     # RAG, embeddings, auth
│   │   └── models/       # Pydantic models
│   ├── auth-service/     # Better Auth standalone service
│   └── ingest_content.py # Vector DB ingestion script
├── specs/                # Feature specifications
└── history/              # Development history
```

## 📖 Documentation

### Getting Started
- **[QUICK-TEST.md](QUICK-TEST.md)** - Fast testing commands
- **[TESTING-GUIDE.md](TESTING-GUIDE.md)** - Comprehensive testing guide
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions

### Specialized Guides
- **[VECTOR-DB-GUIDE.md](VECTOR-DB-GUIDE.md)** - Vector database management
- **[FRONTEND-URLS.md](FRONTEND-URLS.md)** - URL structure and routing

### Component READMEs
- **[backend/README.md](backend/README.md)** - Backend setup and RAG system
- **[frontend/README.md](frontend/README.md)** - Docusaurus configuration
- **[backend/auth-service/README.md](backend/auth-service/README.md)** - Better Auth setup

## 🔧 Key Features

### 1. Interactive Documentation
- 5 comprehensive modules covering Physical AI and Robotics
- Responsive design with dark mode support
- GitHub Pages deployment

### 2. RAG-Powered Chatbot
- OpenAI Agents SDK with GPT-4o-mini
- Qdrant vector database for semantic search
- Context-aware responses with citations
- Highlight-restricted RAG support

### 3. User Authentication & Personalization
- Better Auth integration
- User background questionnaire
- Personalized content recommendations
- Optional authentication (anonymous access supported)
- User preference for content language (e.g., Urdu translation)

### 4. Live Translation
- AI-powered, on-the-fly translation of book content to Urdu
- Preserves code blocks, ensuring technical accuracy
- User-configurable via a toggle in the navigation bar

## 🎯 Common Tasks

### Update Book Content

1. **Edit content**: Modify files in `frontend/docs/module-*/`
2. **Update embeddings**: 
   ```bash
   cd backend
   uv run python ingest_content.py
   ```
3. **Deploy**: 
   ```bash
   cd frontend
   GIT_USER=billy-pk npm run deploy
   ```

### Test Chatbot

```bash
# Test via API
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'

# Or use the browser chat widget
```

### Run Tests

```bash
# Authentication tests
cd backend
./test-complete-auth-flow.sh

# Manual testing checklist
# See QUICK-TEST.md
```

## 🏗️ Technology Stack

### Frontend
- **Framework**: Docusaurus 3.9.2
- **Language**: TypeScript, React 19
- **Auth**: Better Auth client SDK
- **Deployment**: GitHub Pages

### Backend
- **Framework**: FastAPI
- **Language**: Python 3.12
- **RAG**: OpenAI Agents SDK
- **Embeddings**: OpenAI text-embedding-3-large (3072 dims)
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Serverless Postgres
- **Deployment**: Render.com

### Authentication
- **Service**: Better Auth (Node.js/TypeScript)
- **Database**: Shared Neon Postgres
- **Strategy**: Session-based with httpOnly cookies

## 📊 System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    User Browser                         │
│  (http://localhost:3000/physical-ai-humanoid-...)      │
└────────┬────────────────────────────────────────────────┘
         │
         ├──> Frontend (Docusaurus + React)
         │    - Book content rendering
         │    - Chat widget UI
         │    - Auth UI (signup/signin)
         │
         ├──> Better Auth Service (port 3001)
         │    - /api/auth/sign-up/email
         │    - /api/auth/sign-in/email
         │    - Session management
         │
         └──> FastAPI Backend (port 8000)
              - /api/chat (RAG chatbot)
              - /api/auth/profile
              │
              ├──> OpenAI Agents SDK
              │    └──> search_book_content tool
              │         └──> Qdrant Vector DB
              │
              └──> Neon Postgres
                   - User profiles
                   - Session data (shared with Better Auth)
```

## 🔄 Development Workflow

### Spec-Driven Development (SDD)

This project follows Spec-Driven Development:
- **Specifications**: See `specs/001-ai-course-book-platform/`
- **Constitution**: See `CLAUDE.md` / `GEMINI.md`
- **History**: Prompt history in `history/prompts/`

### Making Changes

1. **Understand**: Read relevant spec in `specs/`
2. **Implement**: Make changes following constitution rules
3. **Test**: Use `QUICK-TEST.md` or `TESTING-GUIDE.md`
4. **Document**: Update relevant guides
5. **Deploy**: Follow deployment procedures

## 🚢 Deployment

### Frontend (GitHub Pages)

```bash
cd frontend
GIT_USER=billy-pk npm run deploy
```

Deploys to: https://billy-pk.github.io/physical-ai-humanoid-robotics-book/

### Backend (Render)

Automatic deployment on push to `main` branch (configured in Render dashboard).

### Vector Database

After deploying new content:
```bash
cd backend
uv run python ingest_content.py
```

See [VECTOR-DB-GUIDE.md](VECTOR-DB-GUIDE.md) for details.

## 🐛 Troubleshooting

Common issues and solutions:

1. **"Failed to fetch" errors** → Check backend is running on port 8000
2. **"Not authenticated"** → Verify `BETTER_AUTH_SERVICE_URL` in backend `.env`
3. **CORS errors** → Check CORS configuration allows `http://localhost:3000`
4. **Vector DB payload too large** → Already fixed with batching (see VECTOR-DB-GUIDE.md)
5. **Chat not responding** → Verify Qdrant has embeddings

See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) for comprehensive troubleshooting.

## 📈 Current Status

- ✅ **Modules**: 5 modules (0-4) with 28+ chapters
- ✅ **Vector Database**: 539+ embeddings in Qdrant
- ✅ **Authentication**: Better Auth with user profiles
- ✅ **RAG Chatbot**: Functional with citations
- ✅ **Deployment**: Live on GitHub Pages

## 📝 License

See project license file.

## 🤝 Contributing

This is an educational project following Spec-Driven Development. Changes should:
1. Align with specifications in `specs/`
2. Follow code quality standards in constitution
3. Include appropriate tests
4. Update relevant documentation

## 📞 Support

For help:
1. Check documentation guides (see above)
2. Review troubleshooting guides
3. Check service logs for errors
4. Verify environment variables

---

**Built with** ❤️ **using FastAPI, Docusaurus, OpenAI Agents SDK, and Better Auth**

