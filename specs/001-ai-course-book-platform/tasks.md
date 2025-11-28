# Tasks: AI-Course-Book Platform

**Input**: Design documents from `/specs/001-ai-course-book-platform/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/api-spec.yaml, research.md, quickstart.md

**Tests**: Tests are OPTIONAL and NOT included in this task list unless explicitly requested in the specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- All tasks reference absolute paths from repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create repository directory structure with frontend/, backend/, .github/, and shared/ folders
- [ ] T002 Initialize frontend with Docusaurus in frontend/ directory
- [ ] T003 [P] Initialize backend with Python 3.12 and uv in backend/ directory
- [ ] T004 [P] Configure frontend dependencies: Docusaurus, React, TypeScript, ESLint, Prettier in frontend/package.json
- [ ] T005 [P] Configure backend dependencies using uv: FastAPI, OpenAI SDK, Qdrant client, psycopg in backend/pyproject.toml
- [ ] T006 [P] Setup ruff linting configuration in backend/pyproject.toml
- [ ] T007 [P] Setup ESLint and Prettier configuration in frontend/.eslintrc.js and frontend/.prettierrc
- [ ] T008 [P] Create .env.example file in backend/ with all required environment variables
- [ ] T009 [P] Setup .gitignore for Python, Node.js, and environment files
- [ ] T010 Create README.md files in both frontend/ and backend/ directories

**Checkpoint**: Basic project structure in place - ready for foundational setup

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T011 Setup Neon Postgres database and obtain connection credentials
- [ ] T012 Setup Qdrant Cloud collection "book_embeddings" with 3072-dim vectors, cosine distance
- [X] T013 [P] Create Alembic migrations framework in backend/alembic/
- [X] T014 [P] Create initial database migration for chat_sessions, chat_messages, api_metrics tables in backend/alembic/versions/001_initial_schema.py
- [ ] T015 [P] Create backend/src/core/config.py for environment variables using Pydantic BaseSettings
- [ ] T016 [P] Create backend/src/core/logging.py for structured logging with structlog
- [ ] T017 [P] Create backend/src/core/monitoring.py for metrics tracking (request count, error rate, status codes)
- [ ] T018 [P] Create backend/src/models/errors.py with ErrorResponse model and structured error types
- [ ] T019 [P] Create backend/src/models/chat.py with ChatSession and ChatMessage Pydantic models
- [ ] T020 [P] Create backend/src/models/embeddings.py with EmbeddingChunk and VectorSearchResult models
- [ ] T021 Create backend/src/main.py with FastAPI app initialization, CORS middleware, and lifespan handlers
- [ ] T022 [P] Implement CORS middleware in backend/src/api/middleware/cors.py allowing GitHub Pages domain
- [ ] T023 [P] Implement rate limiting middleware in backend/src/api/middleware/rate_limiter.py (10 req/min per IP)
- [ ] T024 [P] Implement error handling middleware in backend/src/api/middleware/error_handler.py with structured JSON responses
- [ ] T025 [P] Create frontend/docusaurus.config.js with site configuration, GitHub Pages baseUrl, and navbar
- [ ] T026 [P] Create frontend/sidebars.js for documentation navigation structure
- [ ] T027 [P] Create frontend/src/css/custom.css for theme styling and dark mode support

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Read Documentation Content (Priority: P1) 🎯 MVP

**Goal**: Readers visit the documentation website to learn about Physical AI through well-structured, AI-generated content

**Independent Test**: Navigate to deployed GitHub Pages site, verify documentation pages load correctly, are readable, contain properly formatted content with learning outcomes, navigate between chapters, and test mobile responsiveness

### Implementation for User Story 1

- [ ] T028 [US1] Create frontend/docs/intro.md with introduction chapter structure (learning outcomes, explanations, examples)
- [ ] T029 [P] [US1] Create sample chapter structure in frontend/docs/ with placeholder content
- [ ] T030 [P] [US1] Configure Docusaurus sidebar in frontend/sidebars.js to organize chapters by modules
- [ ] T031 [US1] Implement mobile-responsive CSS in frontend/src/css/custom.css (down to 320px width)
- [ ] T032 [US1] Add dark mode theme configuration in frontend/docusaurus.config.js
- [ ] T033 [US1] Create homepage in frontend/src/pages/index.tsx with navigation to documentation
- [ ] T034 [US1] Optimize page load performance: configure build settings for <1.5s load time
- [X] T035 [US1] Test internal navigation links between chapters
- [ ] T036 [US1] Verify all required chapter sections: learning outcomes, explanations, diagrams, examples, exercises

**Checkpoint**: Documentation site is fully functional, navigable, mobile-responsive, and can be deployed to GitHub Pages

---

## Phase 4: User Story 2 - Ask Questions via Embedded Chatbot (Priority: P1)

**Goal**: Readers can ask questions about book content and receive accurate, context-aware answers with citations from embedded chatbot

**Independent Test**: Click chatbot widget on any documentation page, ask question about content from that page, verify response includes accurate information with citations to book sections

### Implementation for User Story 2

- [ ] T037 [P] [US2] Create backend/src/services/vectordb/qdrant_client.py with Qdrant connection and search operations
- [ ] T038 [P] [US2] Create backend/src/services/embeddings/openai_embeddings.py for generating query embeddings
- [ ] T039 [P] [US2] Create backend/src/services/chat/session_manager.py for managing chat sessions in Postgres
- [ ] T040 [US2] Create backend/src/services/rag/retrieval.py for vector similarity search and context retrieval
- [ ] T041 [US2] Create backend/src/services/rag/generation.py using OpenAI Agents SDK for response generation
- [ ] T042 [US2] Create backend/src/api/routes/chat.py with POST /api/chat endpoint implementing RAG flow
- [ ] T043 [P] [US2] Create backend/src/api/routes/health.py with GET /health endpoint for dependency health checks
- [ ] T044 [P] [US2] Create backend/src/api/routes/metrics.py with GET /metrics endpoint for observability
- [ ] T045 [US2] Implement highlighted context RAG in backend/src/services/rag/retrieval.py (RAG-C-002)
- [ ] T046 [US2] Implement citation generation in backend/src/services/rag/generation.py with source URLs
- [ ] T047 [US2] Implement out-of-scope error handling returning "I cannot answer questions outside the documentation"
- [ ] T048 [US2] Implement Qdrant unavailability handling returning "Chat is temporarily unavailable, please try again later"
- [ ] T049 [US2] Implement streaming response support in backend/src/api/routes/chat.py using StreamingResponse
- [ ] T050 [P] [US2] Create frontend/src/components/ChatWidget/ChatWidget.tsx with expandable widget UI
- [ ] T051 [P] [US2] Create frontend/src/components/ChatWidget/ChatWidget.module.css for widget styling (bottom-right, clean UI)
- [ ] T052 [US2] Integrate ChatWidget with backend API: POST /api/chat with query and highlighted_context
- [ ] T053 [US2] Implement chat message display with citations in ChatWidget component
- [ ] T054 [US2] Implement highlight-to-ask feature: capture selected text and send as highlighted_context
- [ ] T055 [US2] Create frontend/src/theme/Root.tsx to embed ChatWidget on all documentation pages
- [ ] T056 [US2] Configure API base URL with environment detection (localhost vs production backend)
- [ ] T057 [US2] Ensure chat widget does not block page rendering (async loading)
- [ ] T058 [US2] Create backend/scripts/ingest_content.py to embed documentation chapters into Qdrant
- [ ] T059 [US2] Implement content chunking strategy (500-800 tokens, 100 overlap) in ingestion script
- [ ] T060 [US2] Implement chunk deduplication using SHA-256 content hashing in ingestion script
- [ ] T061 [US2] Run content ingestion for initial documentation chapters
- [ ] T062 [US2] Verify end-to-end flow: ask question → retrieve context → generate response → display citations

**Checkpoint**: Chatbot is fully functional with RAG, citations, highlight-to-ask, graceful error handling, and content ingestion complete

---

## Phase 5: User Story 3 - Automated Content Deployment (Priority: P2)

**Goal**: Content creators commit changes and have them automatically deployed to production without manual intervention

**Independent Test**: Commit documentation change to main branch, wait for CI/CD pipeline to complete, verify change appears on live GitHub Pages site

### Implementation for User Story 3

- [ ] T063 [P] [US3] Create .github/workflows/frontend-ci.yml for frontend CI/CD workflow
- [ ] T064 [P] [US3] Configure frontend workflow: trigger on push to main and paths: ['frontend/**']
- [ ] T065 [US3] Add frontend build step: npm ci && npm run build in workflow
- [ ] T066 [US3] Add ESLint and Prettier checks in frontend workflow
- [ ] T067 [US3] Add GitHub Pages deployment step using peaceiris/actions-gh-pages@v3
- [ ] T068 [US3] Configure GitHub repository settings: enable GitHub Pages from gh-pages branch
- [ ] T069 [P] [US3] Create .github/workflows/backend-ci.yml for backend CI/CD workflow
- [ ] T070 [P] [US3] Configure backend workflow: trigger on push to main and paths: ['backend/**']
- [ ] T071 [US3] Add backend build step: install uv and run uv sync in workflow
- [ ] T072 [US3] Add ruff linting check in backend workflow
- [ ] T073 [US3] Add pytest test step in backend workflow (if tests exist)
- [ ] T074 [US3] Add Render deployment trigger in backend workflow on success
- [ ] T075 [US3] Configure Render service: connect repository, set build command to "uv sync"
- [ ] T076 [US3] Configure Render start command: "uv run uvicorn src.main:app --host 0.0.0.0 --port $PORT"
- [ ] T077 [US3] Set environment variables in Render dashboard: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL
- [ ] T078 [US3] Test frontend deployment: commit change to frontend/ and verify GitHub Actions runs
- [ ] T079 [US3] Test backend deployment: commit change to backend/ and verify Render redeploys
- [ ] T080 [US3] Verify failed linting blocks deployment in both workflows

**Checkpoint**: CI/CD pipelines are fully automated for both frontend and backend with deployment gates for code quality

---

## Phase 6: User Story 4 - Backend RAG Service Availability (Priority: P2)

**Goal**: Chatbot backend is continuously available with low latency and accurate retrieval from vector database

**Independent Test**: Send HTTP requests to FastAPI backend endpoints, verify responses within latency requirements (<200ms excluding LLM, streaming starts <1.5s)

### Implementation for User Story 4

- [ ] T081 [P] [US4] Implement connection pooling for Qdrant client in backend/src/services/vectordb/qdrant_client.py
- [ ] T082 [P] [US4] Implement connection pooling for Neon Postgres using asyncpg in backend/src/core/config.py
- [ ] T083 [P] [US4] Add response time tracking to logging middleware in backend/src/api/middleware/error_handler.py
- [ ] T084 [P] [US4] Implement metrics collection in backend/src/core/monitoring.py (request_count, error_count, status_codes)
- [ ] T085 [US4] Configure Qdrant HNSW parameters for optimal search performance (m=16, ef_construct=100)
- [ ] T086 [US4] Implement caching layer for frequent queries in backend/src/services/rag/retrieval.py (TTL: 5 minutes)
- [ ] T087 [US4] Optimize async operations: parallel Qdrant search and metadata lookup
- [ ] T088 [US4] Configure FastAPI lifespan events for database connection management
- [ ] T089 [US4] Implement health check logic: verify Qdrant, Postgres, and OpenAI connectivity
- [ ] T090 [US4] Add request ID generation and tracing in middleware
- [ ] T091 [US4] Load test backend with 100 concurrent chat requests to verify no degradation
- [ ] T092 [US4] Verify 99% uptime monitoring is configured (RA-001)
- [ ] T093 [US4] Verify API response time <200ms for 95% of requests (excluding LLM processing)
- [ ] T094 [US4] Verify streaming response initiation <1.5 seconds

**Checkpoint**: Backend service meets all performance, availability, and observability requirements

---

## Phase 7: User Story 5 - Code Quality Assurance via CI/CD (Priority: P3)

**Goal**: Developers maintain code quality through automated linting and testing on every pull request and commit

**Independent Test**: Create pull request with intentionally poor code quality, verify CI/CD pipeline fails with appropriate error messages

### Implementation for User Story 5

- [ ] T095 [P] [US5] Add pull request triggers to frontend workflow in .github/workflows/frontend-ci.yml
- [ ] T096 [P] [US5] Add pull request triggers to backend workflow in .github/workflows/backend-ci.yml
- [ ] T097 [P] [US5] Configure ruff with PEP8 rules and line length 88 in backend/pyproject.toml
- [ ] T098 [P] [US5] Add ruff format check to backend CI workflow
- [ ] T099 [P] [US5] Configure ESLint with TypeScript rules in frontend/.eslintrc.js
- [ ] T100 [P] [US5] Add TypeScript type checking to frontend CI workflow
- [ ] T101 [US5] Add pytest coverage report step in backend workflow with 70% minimum threshold
- [ ] T102 [US5] Configure pytest.ini with coverage settings in backend/
- [ ] T103 [US5] Add status check requirements in GitHub repository settings for branch protection
- [ ] T104 [US5] Test backend PR: create PR with linting violations, verify workflow fails
- [ ] T105 [US5] Test frontend PR: create PR with ESLint errors, verify workflow fails
- [ ] T106 [US5] Test coverage check: ensure pytest coverage threshold is enforced
- [ ] T107 [US5] Verify deployment only triggers on main branch after checks pass

**Checkpoint**: Code quality gates are enforced for all code changes with automated checks

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple user stories and ensuring constitution compliance

- [ ] T108 [P] Run ruff check and ruff format on all backend Python code (Constitution II)
- [ ] T109 [P] Run ESLint and Prettier on all frontend TypeScript code (Constitution II)
- [ ] T110 [P] Verify backend test coverage reaches 70% minimum (Constitution III)
- [ ] T111 [P] Create comprehensive README.md in repository root with quickstart guide
- [ ] T112 [P] Update backend/README.md with local development setup using uv (DOC-001, DOC-002)
- [ ] T113 [P] Update frontend/README.md with Docusaurus setup and widget integration (DOC-003)
- [ ] T114 [P] Document deployment procedures for Render and GitHub Pages (DOC-004, DOC-005)
- [ ] T115 [P] Document CI/CD workflows for both frontend and backend (DOC-006)
- [ ] T116 [P] Create troubleshooting guide in docs/ covering environment variables, CORS, widget errors (DOC-007)
- [ ] T117 Create .github/dependabot.yml for automated security dependency updates
- [ ] T118 [P] Verify all success criteria from spec.md are met (SC-001 through SC-015)
- [ ] T119 [P] Verify all constitution requirements are satisfied
- [ ] T120 Performance optimization: review and optimize frontend bundle size
- [ ] T121 [P] Security hardening: review input validation and sanitization
- [ ] T122 [P] Verify mobile responsiveness on screens down to 320px width
- [ ] T123 [P] Verify dark mode works correctly across all pages
- [ ] T124 Run quickstart.md validation: follow all steps and verify they work
- [ ] T125 [P] Add monitoring alerts for 99% uptime SLA tracking
- [ ] T126 Final end-to-end testing: complete user journey through all 5 user stories
- [ ] T127 Create Git tag for v1.0.0 release (Constitution IV)
- [ ] T128 Deploy final version to production (Render + GitHub Pages)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - No dependencies on other stories
- **User Story 2 (Phase 4)**: Depends on Foundational - Integrates with US1 (widget on pages) but independently testable
- **User Story 3 (Phase 5)**: Depends on US1 and US2 being complete (deploys their code)
- **User Story 4 (Phase 6)**: Depends on US2 being complete (optimizes RAG backend)
- **User Story 5 (Phase 7)**: Depends on Setup being complete (adds CI/CD checks)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Frontend documentation site - No dependencies on other stories ✅ Can start after Foundational
- **User Story 2 (P1)**: RAG chatbot functionality - Depends on US1 (widget embedded on pages created in US1)
- **User Story 3 (P2)**: Automated deployment - Depends on US1 and US2 (deploys their artifacts)
- **User Story 4 (P2)**: Backend performance/availability - Depends on US2 (optimizes backend created in US2)
- **User Story 5 (P3)**: Code quality CI/CD - Can start after Setup (independent of other stories) ✅ Can run in parallel with US1

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before API routes
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

**Setup Phase (Phase 1)**:
- T004, T005, T006, T007, T008, T009 can all run in parallel

**Foundational Phase (Phase 2)**:
- T013-T020 (database, config, logging, models) can run in parallel
- T022-T024 (middleware) can run in parallel
- T025-T027 (frontend config) can run in parallel

**User Story 1**:
- T028, T029, T030 (content creation) can run in parallel
- After foundation: can work on US1 and US5 simultaneously (different teams)

**User Story 2**:
- T037, T038, T039 (backend services) can run in parallel
- T043, T044 (health and metrics endpoints) can run in parallel
- T050, T051 (widget UI) can run in parallel

**User Story 3**:
- T063-T067 (frontend workflow) and T069-T074 (backend workflow) can run in parallel

**User Story 4**:
- T081-T084 (connection pooling and monitoring) can run in parallel

**User Story 5**:
- T095-T100 (CI configuration) can run in parallel

**Polish Phase**:
- T108-T116, T120-T123, T125 all marked [P] can run in parallel

---

## Parallel Example: User Story 2

```bash
# Launch backend services together (all [P]):
Task: "Create backend/src/services/vectordb/qdrant_client.py"
Task: "Create backend/src/services/embeddings/openai_embeddings.py"
Task: "Create backend/src/services/chat/session_manager.py"

# Launch health and metrics endpoints together (all [P]):
Task: "Create backend/src/api/routes/health.py"
Task: "Create backend/src/api/routes/metrics.py"

# Launch frontend widget UI together (all [P]):
Task: "Create frontend/src/components/ChatWidget/ChatWidget.tsx"
Task: "Create frontend/src/components/ChatWidget/ChatWidget.module.css"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T027) - **CRITICAL - blocks all stories**
3. Complete Phase 3: User Story 1 (T028-T036) - Documentation site
4. Complete Phase 4: User Story 2 (T037-T062) - RAG chatbot
5. **STOP and VALIDATE**: Test US1 and US2 independently
6. Deploy/demo MVP: Static documentation with working AI chatbot

### Incremental Delivery

1. **Foundation** (Phases 1-2) → Development environment ready
2. **MVP** (US1 + US2) → Core user value: Documentation + AI assistance
3. **Automation** (US3) → CI/CD enables faster iteration
4. **Performance** (US4) → Backend optimized for production scale
5. **Quality** (US5) → Code quality gates enforced
6. **Polish** (Phase 8) → Production-ready release

### Parallel Team Strategy

With 3 developers after Foundational phase completes:

1. **Developer A**: User Story 1 (T028-T036) - Documentation site
2. **Developer B**: User Story 5 (T095-T107) - CI/CD setup (independent)
3. **Developer C**: Prep for User Story 2 (review backend architecture)

After US1 completes:
1. **Developer A**: User Story 2 frontend (T050-T057) - Chat widget
2. **Developer B**: User Story 2 backend (T037-T049) - RAG service
3. **Developer C**: User Story 3 prep (T063-T080) - CI/CD workflows

---

## Task Summary

- **Total Tasks**: 128
- **Setup Phase**: 10 tasks
- **Foundational Phase**: 17 tasks (CRITICAL - blocks all stories)
- **User Story 1 (P1)**: 9 tasks - Documentation site
- **User Story 2 (P1)**: 26 tasks - RAG chatbot (largest story)
- **User Story 3 (P2)**: 18 tasks - Automated deployment
- **User Story 4 (P2)**: 14 tasks - Backend performance/availability
- **User Story 5 (P3)**: 13 tasks - Code quality CI/CD
- **Polish Phase**: 21 tasks - Cross-cutting concerns

**Parallel Opportunities Identified**: 47 tasks marked [P] can run concurrently with other tasks in same phase

**Suggested MVP Scope**:
- Phase 1: Setup (10 tasks)
- Phase 2: Foundational (17 tasks)
- Phase 3: User Story 1 (9 tasks)
- Phase 4: User Story 2 (26 tasks)
- **Total MVP**: 62 tasks → Delivers documentation site with working AI chatbot

---

## Notes

- [P] tasks = different files, no dependencies within same phase
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Follow quickstart.md for setup verification steps
- Refer to Constitution for code quality, testing, versioning, and deployment standards
