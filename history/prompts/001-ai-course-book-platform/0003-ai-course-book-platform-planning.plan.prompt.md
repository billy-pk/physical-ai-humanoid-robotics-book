---
id: 0003
title: AI Course Book Platform Planning
stage: plan
date: 2025-11-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-course-book-platform
branch: 001-ai-course-book-platform
user: billy-pk
command: /sp.plan
labels: ["architecture", "planning", "data-model", "api-design", "two-phase-implementation"]
links:
  spec: specs/001-ai-course-book-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-course-book-platform/plan.md
  - specs/001-ai-course-book-platform/research.md
  - specs/001-ai-course-book-platform/data-model.md
  - specs/001-ai-course-book-platform/contracts/api-spec.yaml
  - specs/001-ai-course-book-platform/quickstart.md
  - CLAUDE.md (updated agent context)
tests:
  - Constitution check validation (all requirements met)
---

## Prompt

User invoked `/sp.plan` command with detailed two-phase implementation approach:

**Phase 1: AI-Generated Book Creation and Deployment**
- Initialize project repository with frontend/ and backend/ folders
- Generate book content using Spec-Kit Plus and Claude Code
- Set up Docusaurus frontend with site configuration and sidebar
- Apply styling and branding
- Test rendering, navigation, and responsive design
- Deploy to GitHub Pages

**Phase 2: RAG Chatbot Integration**
- Initialize FastAPI backend with uv dependency management
- Implement RAG logic using OpenAI Agents SDK
- Set up Qdrant vector database and Neon Postgres metadata database
- Embed ChatKit widget on all pages (bottom-right, expandable)
- Connect frontend widget with backend RAG endpoints
- Set up CI/CD workflows for both frontend and backend
- Create comprehensive documentation

## Response snapshot

Generated complete implementation plan with five key deliverables:

**1. plan.md** - Main implementation plan including:
- Summary of platform architecture
- Technical context (languages, dependencies, storage, testing, platforms, performance goals, constraints, scale)
- Constitution check validation (all requirements met)
- Detailed project structure for web application (backend/, frontend/, .github/, shared/)
- Zero complexity violations

**2. research.md** - Technology research covering:
- 8 major technology decisions (Docusaurus, FastAPI, OpenAI Agents SDK, Qdrant, Neon Postgres, uv, Ruff, ESLint/Prettier)
- Implementation patterns for frontend-backend communication, rate limiting, error handling, observability
- Deployment strategies for GitHub Pages and Render
- Testing strategies for backend (pytest) and frontend (Jest + React Testing Library)
- Security considerations and performance optimizations

**3. data-model.md** - Complete data model specification:
- 6 entity definitions with attributes, relationships, validation rules, state transitions
- Database schemas for Postgres (chat_sessions, chat_messages, api_metrics) with indexes and constraints
- Qdrant collection configuration for vector embeddings
- Data flow diagrams for chat queries and content ingestion
- Migration strategy using Alembic
- Backup strategy and retention policies

**4. contracts/api-spec.yaml** - OpenAPI 3.0 specification:
- 3 API endpoints: /api/chat (RAG chatbot), /health (monitoring), /metrics (observability)
- Complete request/response schemas with validation
- Error response patterns including out-of-scope and service unavailable messages
- Rate limiting documentation
- Examples for all scenarios

**5. quickstart.md** - Comprehensive setup guide:
- Phase 1 walkthrough: frontend setup, content generation, Docusaurus configuration, deployment
- Phase 2 walkthrough: backend setup, database configuration, content ingestion, widget integration, production deployment
- CI/CD automation configuration for both GitHub Actions workflows
- Verification checklist for both phases
- Troubleshooting guide for common issues

**Agent Context Update**: Updated CLAUDE.md with new project structure information.

## Outcome

- ✅ Impact: Created production-ready implementation plan with complete architecture, data models, API contracts, and operational procedures. Plan supports two-phase rollout enabling incremental delivery and validation.
- 🧪 Tests: Constitution check passed - all code quality, testing, versioning, and deployment requirements validated
- 📁 Files: Generated 5 planning artifacts totaling ~900 lines of documentation
- 🔁 Next prompts: `/sp.tasks` to generate actionable, dependency-ordered task list
- 🧠 Reflection: Successfully translated user's phased approach into comprehensive technical plan. Research phase identified all implementation patterns needed. Data model provides clear foundation for both Postgres and Qdrant integration. API spec enables parallel frontend/backend development. Quickstart ensures smooth developer onboarding.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow completed successfully
- Graders run and results (PASS/FAIL): Constitution check - PASS (all 4 gates validated)
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided phased approach integrated
- Next experiment (smallest change to try): None needed - proceed to task generation phase
