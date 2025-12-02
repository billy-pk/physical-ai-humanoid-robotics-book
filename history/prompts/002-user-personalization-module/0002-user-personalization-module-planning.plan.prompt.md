---
id: 0002
title: User Personalization Module Planning
stage: plan
date: 2025-12-01
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-user-personalization-module
branch: 002-user-personalization-module
user: bilali
command: /sp.plan
labels: ["personalization", "planning", "architecture", "openai-agents", "caching", "translation"]
links:
  spec: specs/002-user-personalization-module/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-user-personalization-module/plan.md
  - specs/002-user-personalization-module/research.md
  - specs/002-user-personalization-module/data-model.md
  - specs/002-user-personalization-module/contracts/personalization-api.openapi.yaml
  - specs/002-user-personalization-module/quickstart.md
  - CLAUDE.md (agent context updated)
tests:
  - Constitution Check: All items passed
  - Technical Context: Complete (no NEEDS CLARIFICATION markers)
---

## Prompt

Add a user personalization module:  PL check for already implemented features
- After login (better-auth), redirect users to /popup to submit preferences (level, topics, goals, full vs personalized content, Urdu translation).
- Store preferences in DB .
- Provide two modes: full content view OR personalized content generated via OpenAI Agents SDK based on preferences.
- Add Urdu translation: on-demand translation of any chapter using OpenAI agent, preserving code blocks.
- Add API routes for saving/loading preferences and generating personalized or translated content.
- Frontend updates dynamically depending on user preferences (Docusaurus/React).

## Response snapshot

Completed comprehensive planning workflow for User Personalization Module:

**Phase 0: Research (research.md)**
- OpenAI Agents SDK integration pattern (extend existing RAG agent pattern)
- Urdu translation strategy with code block preservation (pre/post-processing)
- Database-backed caching with hash-based keys (Postgres)
- Preference storage design (extend existing user_profiles.preferences JSONB)
- Frontend redirect flow (extend QuestionnaireGuard pattern)
- Model selection: gpt-4o-mini for both personalization and translation
- Risk mitigation strategies for API limits, costs, and quality

**Phase 1: Design & Contracts**
1. **data-model.md**: Entity definitions and relationships
   - Extended user_profiles.preferences JSONB structure
   - New personalized_content_cache table with hash-based cache keys
   - Data access patterns and migration strategy
   - Storage estimates: ~315 MB steady-state for 1000 users

2. **contracts/personalization-api.openapi.yaml**: Complete API specification
   - GET/POST /api/auth/profile/preferences (preference management)
   - POST /api/content/personalize (personalized content generation)
   - POST /api/content/translate (Urdu translation)
   - GET /api/content/chapters/{chapter_id} (unified content endpoint)
   - Full request/response schemas with validation rules

3. **quickstart.md**: Developer onboarding guide
   - 4-phase implementation workflow (Database → Backend → Frontend → Testing)
   - Code samples for all services (content_generator, translator, cache)
   - Testing strategies and troubleshooting guide
   - Estimated implementation time: 9-13 hours total

**Technical Context Filled**:
- Language/Version: Python 3.12+, TypeScript/React 19, Node.js 20+
- Primary Dependencies: FastAPI, OpenAI Agents SDK, Better Auth, Docusaurus, Pydantic, psycopg3, Alembic
- Storage: Neon Postgres + Qdrant Cloud
- Performance Goals: <10s personalization, <15s translation, <3s mode switching
- Constraints: OpenAI rate limits (3500 RPM), 128k context window, code preservation required

**Constitution Check**: All items passed (Code Quality, Testing Standards, Versioning, Deployment)

**Project Structure**: Defined file locations for all new/modified components
- Backend: models, services/personalization/, api/routes/content.py, Alembic migration
- Frontend: components/Auth/PreferenceForm.tsx, pages/popup.tsx, contexts/PersonalizationContext.tsx
- Tests: Unit tests for all services, component tests for forms

**Agent Context**: Updated CLAUDE.md with new technologies for feature 002-user-personalization-module

## Outcome

- ✅ Impact: Complete planning phase with research, design, and contracts ready for implementation
- 🧪 Tests: Constitution Check passed, technical context validated, no NEEDS CLARIFICATION markers
- 📁 Files: 5 design documents created (plan.md, research.md, data-model.md, API contract, quickstart.md), agent context updated
- 🔁 Next prompts: Execute /sp.tasks to generate implementation tasks from planning artifacts
- 🧠 Reflection: Leveraged existing architecture investigation (Better Auth, user_profiles, OpenAI Agents SDK, RAG pattern) to create practical, implementable plan without needing clarifications. Design extends existing patterns rather than introducing new architectural components.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow completed successfully end-to-end
- Graders run and results (PASS/FAIL): Constitution Check: PASS, Technical Context: PASS (complete), Spec Quality: PASS (from earlier validation)
- Prompt variant (if applicable): Standard /sp.plan workflow with comprehensive codebase exploration beforehand
- Next experiment (smallest change to try): Proceed to /sp.tasks to validate that planning artifacts are sufficient for generating concrete, actionable implementation tasks
