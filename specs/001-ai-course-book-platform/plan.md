# Implementation Plan: AI-Course-Book Platform

**Branch**: `001-ai-course-book-platform` | **Date**: 2025-01-27 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-ai-course-book-platform/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a documentation platform hosting AI-generated book content about Physical AI and Humanoid Robotics with an embedded RAG chatbot. The frontend uses Docusaurus deployed to GitHub Pages, and the backend uses FastAPI with OpenAI Agents SDK deployed to Render. The chatbot provides interactive, context-aware assistance using only book content retrieved from a Qdrant vector database. Implementation follows a two-phase approach: Phase 1 establishes the static documentation site, and Phase 2 adds the RAG chatbot functionality with full CI/CD automation.

## Technical Context

**Language/Version**: Python 3.12 (Backend), JavaScript/TypeScript/Node.js LTS (Frontend)

**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant Client, Neon Postgres, Docusaurus, React, TypeScript, uv, ruff, pytest, Jest, React Testing Library

**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (metadata), Static Markdown files (content), OpenAI text-embedding-3-large (embeddings)

**Testing**: pytest (backend, 70% coverage), Jest + React Testing Library (frontend), Precision@k and Context recall (RAG evaluation)

**Target Platform**: GitHub Pages (frontend), Render.com (backend), Modern browsers (Chrome, Firefox, Safari, Edge)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Frontend page load: <1.5 seconds
- Backend API response (excluding LLM): <200ms for 95% of requests
- Chatbot streaming initiation: <1.5 seconds
- Concurrent chat sessions: 100+ without degradation

**Constraints**:
- Backend uptime: 99% SLA (maximum 7.2 hours downtime per month)
- Rate limiting on public API endpoints (no authentication)
- Free tier infrastructure (Qdrant Cloud, Neon Postgres)
- Chat widget must not block page rendering
- Mobile responsive down to 320px width
- Test coverage minimum 70% for backend

**Scale/Scope**:
- Book content: Multiple chapters with learning outcomes, diagrams, examples, exercises
- Expected traffic: 100+ concurrent users
- Vector database: Documentation-scale embeddings (free tier sufficient)
- API endpoints: Chat functionality, health checks, metrics
- CI/CD: Separate pipelines for frontend (GitHub Actions → Pages) and backend (GitHub Actions → Render)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Code Quality**: Plan adheres to Python (Ruff), JS/TS (ESLint + Prettier), TypeScript usage, structured JSON errors, exception wrappers, structured logging, UI source transparency, and secret management (refer to Constitution II). Backend uses ruff for Python, frontend uses ESLint+Prettier for TypeScript. All requirements from spec align with constitution.

- [x] **Testing Standards**: Plan complies with Backend (Pytest, 70% coverage), Frontend (Jest + React Testing Library), and RAG Evaluation (Precision@k, Context recall) requirements (refer to Constitution III). Testing framework selections match constitution requirements exactly.

- [x] **Versioning & Release**: Plan aligns with chapter versioning, Git commits, and backend release tagging/redeployment (refer to Constitution IV). CI/CD includes automatic version tracking and deployment triggers.

- [x] **Deployment**: Plan is consistent with GitHub Pages (frontend) and Render (backend) deployment strategies (refer to Constitution V). Frontend deploys to GitHub Pages via Actions, backend deploys to Render with auto-redeploy on main branch updates.

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-course-book-platform/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── api-spec.yaml    # OpenAPI specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── routes/          # FastAPI route handlers
│   │   ├── middleware/      # CORS, rate limiting, error handling
│   │   └── dependencies.py  # Dependency injection
│   ├── services/
│   │   ├── rag/             # RAG logic with OpenAI Agents SDK
│   │   ├── vectordb/        # Qdrant client and operations
│   │   ├── embeddings/      # Text embedding generation
│   │   └── chat/            # Chat session management
│   ├── models/
│   │   ├── chat.py          # Chat message, session models
│   │   ├── embeddings.py    # Embedding chunk models
│   │   └── errors.py        # Structured error responses
│   ├── core/
│   │   ├── config.py        # Environment variables, settings
│   │   ├── logging.py       # Structured logging setup
│   │   └── monitoring.py   # Metrics tracking (request count, error rate, status codes)
│   └── main.py              # FastAPI app initialization
├── tests/
│   ├── unit/                # Unit tests for services, models
│   ├── integration/         # Integration tests for API routes
│   └── conftest.py          # Pytest fixtures
├── scripts/
│   └── ingest_content.py    # Script to embed documentation into Qdrant
├── alembic/                 # Database migrations
├── pyproject.toml          # uv project configuration
├── .env.example             # Example environment variables
└── README.md                # Backend setup and deployment docs

frontend/
├── docs/                    # Docusaurus documentation content
│   ├── intro.md
│   └── [module-folders]/
├── src/
│   ├── components/
│   │   └── ChatWidget/      # ChatKit integration component
│   ├── css/
│   │   └── custom.css       # Theme and styling
│   └── pages/
│       └── index.tsx        # Homepage
├── static/
│   └── img/                 # Images, diagrams
├── docusaurus.config.js     # Docusaurus configuration
├── sidebars.js              # Documentation sidebar structure
├── package.json             # npm dependencies
├── tsconfig.json            # TypeScript configuration
└── README.md                # Frontend setup and deployment docs

.github/
├── workflows/
│   ├── backend-ci.yml       # Backend: lint, test, deploy to Render
│   └── frontend-ci.yml      # Frontend: build, deploy to GitHub Pages
└── dependabot.yml           # Automated dependency updates

shared/
└── scripts/
    ├── generate_chapters.sh # Helper to generate book chapters using /sp.specify
    └── validate_links.sh    # Validate internal documentation links
```

**Structure Decision**: Web application structure selected. Backend and frontend are completely separated to support independent deployment (Render for backend, GitHub Pages for frontend). Backend uses modular FastAPI structure with clear separation of API routes, business logic (services), data models, and core utilities. Frontend uses standard Docusaurus structure with custom ChatWidget component for chatbot integration. Shared scripts directory contains utilities for content generation and validation.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations. All constitution requirements are met by the plan.
