# Implementation Plan: User Personalization Module

**Branch**: `002-user-personalization-module` | **Date**: 2025-12-01 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-user-personalization-module/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This feature adds a comprehensive user personalization system that collects user preferences (experience level, learning topics, goals, content mode, language) after authentication and provides two content delivery modes: (1) full content view with original chapter content, and (2) personalized content generated via OpenAI Agents SDK based on user preferences. Additionally, it supports on-demand Urdu translation while preserving code blocks. The implementation extends the existing Better Auth authentication, user profile database schema, and RAG chatbot infrastructure.

## Technical Context

**Language/Version**: Python 3.12+ (backend), TypeScript/React 19 (frontend), Node.js 20+ (Better Auth service)
**Primary Dependencies**: FastAPI 0.122.0, OpenAI Agents SDK 0.6.1, Better Auth 1.3.10, Docusaurus 3.9.2, Pydantic 2.12+, psycopg3 3.2.13+, Alembic 1.17.2+
**Storage**: Neon Serverless Postgres (user profiles, preferences, cached personalized content), Qdrant Cloud (vector embeddings for RAG)
**Testing**: Pytest (backend, 70% coverage target), Jest + React Testing Library (frontend)
**Target Platform**: Linux server (backend on Render), Web browser (frontend via GitHub Pages)
**Project Type**: Web application (FastAPI backend + Docusaurus/React frontend)
**Performance Goals**: Personalized content generation <10s for 5000-word chapters, Urdu translation <15s, preference submission <2 minutes, mode switching <3s
**Constraints**: OpenAI API rate limits (3500 RPM tier-1), OpenAI context window limits (128k tokens for gpt-4o-mini), session-based auth with httpOnly cookies, code block preservation during all transformations
**Scale/Scope**: ~1000 users initially, ~50 book chapters, estimated 100 concurrent content generation requests max

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Code Quality**: Will adhere to Python (Ruff) for backend services, TypeScript with ESLint + Prettier for frontend components, structured JSON error responses for all API endpoints, exception wrappers for personalization/translation services, structured logging via structlog, and OPENAI_API_KEY via environment variables (never in git).
- [x] **Testing Standards**: Will include Pytest unit tests for preference services, content generation agents, and translation services (targeting 70% coverage). Frontend preference form and content mode switching will be tested with Jest + React Testing Library.
- [x] **Versioning & Release**: Feature will be committed on feature branch with descriptive messages, merged to main with version bump if needed.
- [x] **Deployment**: Frontend preference page and mode-switching UI will deploy to GitHub Pages via gh-pages branch using GitHub Actions. Backend deployment to Render deferred for future consideration.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   └── user.py                    # Extend: Add personalization preference models
│   ├── services/
│   │   ├── user_profile.py            # Extend: Add preference CRUD operations
│   │   ├── personalization/           # NEW: Personalization service
│   │   │   ├── content_generator.py   #   - OpenAI agent for personalized content
│   │   │   ├── translator.py          #   - OpenAI agent for Urdu translation
│   │   │   └── cache.py               #   - Caching layer for generated content
│   │   └── rag/
│   │       └── rag_agent.py           # Extend: Add user context to RAG prompts
│   ├── api/
│   │   └── routes/
│   │       ├── auth.py                # Extend: Add preference endpoints
│   │       └── content.py             # NEW: Content generation/translation endpoints
│   └── core/
│       └── config.py                  # Verify: OPENAI_API_KEY present
├── alembic/
│   └── versions/
│       └── [new]_add_personalization_preferences.py  # NEW: Migration for preferences
└── tests/
    ├── unit/
    │   ├── test_content_generator.py  # NEW: Test personalization logic
    │   ├── test_translator.py         # NEW: Test translation logic
    │   └── test_preference_service.py # NEW: Test preference CRUD
    └── integration/
        └── test_personalization_flow.py  # NEW: End-to-end preference → content flow

frontend/
├── src/
│   ├── components/
│   │   ├── Auth/
│   │   │   └── PreferenceForm.tsx     # NEW: Preference collection form at /popup
│   │   ├── Content/                   # NEW: Content display components
│   │   │   ├── ContentModeSwitch.tsx  #   - Toggle between full/personalized
│   │   │   ├── PersonalizedView.tsx   #   - Personalized content renderer
│   │   │   ├── FullView.tsx           #   - Full content renderer
│   │   │   └── TranslationToggle.tsx  #   - Urdu translation toggle
│   │   └── ChatWidget/
│   │       └── index.tsx              # Extend: Pass user preferences to chat
│   ├── pages/
│   │   └── popup.tsx                  # NEW: Preference submission page
│   ├── hooks/
│   │   ├── useAuth.ts                 # Extend: Check preference completion status
│   │   └── usePersonalization.ts      # NEW: Hook for preference & content mode
│   └── contexts/
│       └── PersonalizationContext.tsx # NEW: Global personalization state
└── tests/
    └── components/
        ├── PreferenceForm.test.tsx    # NEW: Test preference form
        └── ContentModeSwitch.test.tsx # NEW: Test mode switching
```

**Structure Decision**: Web application structure. Backend uses FastAPI with service layer pattern (models, services, API routes). Frontend uses Docusaurus with React components, pages, hooks, and contexts. New personalization logic is isolated in dedicated services and components that extend existing architecture without disrupting current functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
