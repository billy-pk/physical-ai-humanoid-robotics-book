<!-- Sync Impact Report:
Version change: None (initial creation) → 1.0.0
List of modified principles:
  - (New) Book Creation Principles
  - (New) Code Quality Principles
  - (New) Testing Standards
  - (New) Versioning & Release Principles
  - (New) Deployment Principles
Added sections: Project Components & Architecture, Structure, Naming & Non-Functional Requirements
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending (check alignment with new principles)
  - .specify/templates/spec-template.md: ⚠ pending (check alignment with new principles)
  - .specify/templates/tasks-template.md: ⚠ pending (check alignment with new principles)
  - .specify/templates/commands/*.md: ⚠ pending (verify no outdated references)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Book Creation Principles
All book chapters MUST be created via `/sp.specify` templates. Each chapter MUST include learning outcomes, core explanations, illustrations/diagrams, real-world examples, and exercises or assessments. The writing style MUST be beginner-friendly, step-by-step, real-world robotics oriented, and maintain consistent tone, terminology, and formatting. All chapters MUST be internally linked to a glossary, module reference guides, and weekly breakdowns.

### II. Code Quality Principles
Python code MUST adhere to Ruff (PEP8 + flake8 compatibility) for linting. JavaScript/TypeScript code MUST use ESLint + Prettier. TypeScript MUST be used for all frontend code. APIs MUST return structured JSON errors and every API route MUST include exception wrappers. Structured logs MUST be used. The chatbot UI MUST clearly show "sources used". API keys MUST NEVER be stored in git; Render/GitHub secrets MUST be used.

### III. Testing Standards
Backend unit tests MUST use Pytest and include embedding functions, vector search pipeline, Postgres integration, and FastAPI routes, with a minimum coverage of 70%. Frontend tests MUST use Jest + React Testing Library. RAG evaluation MUST benchmark retrieval quality using Precision@k and Context recall, and include unit tests for chunking logic.

### IV. Versioning & Release Principles
Every new chapter release MUST be accompanied by a Git commit and a version bump in the book's frontmatter. Backend changes MUST require a Git tag and a Render redeploy.

### V. Deployment Principles
The frontend MUST be deployed with GitHub Pages via GitHub Actions, with each successful commit triggering an auto-build. The backend MUST be deployed to Render, with automatic redeploy on `main` branch updates.

## Project Components & Architecture

### Project Components
The project consists of a book (frontend) and a RAG chatbot (backend).
- **Book (Frontend):** Authored using Spec-Kit Plus, designed with Docusaurus, and deployed to GitHub Pages. It MUST include diagrams, examples, weekly breakdowns, module summaries, and high-quality human-readable explanations.
- **RAG Chatbot (Backend):** Built with FastAPI, uses OpenAI ChatKit / Agents SDK, Qdrant Cloud for vector database (Free Tier), and Neon Postgres for runtime metadata DB. Features include general book questions, highlighted-text-only answering, streaming responses, and direct embedding into the Docusaurus frontend.

### Architecture Guidelines
- **Frontend:** Located in `frontend/`, fully static, deployed via GitHub Pages. The chat widget MUST be embedded in the sidebar or floating bottom-right. Documentation chapters are auto-generated through Spec-Kit Plus.
- **Backend:** Located in `backend/`, a FastAPI app with routes: `/embed` (embed text), `/search` (vector search pipeline), and `/ask` (RAG completion using OpenAI Agents). Deployment preference is Render.com.
- **Shared:** Includes scripts for ingestion, chunking, and embeddings extraction.

## Structure, Naming & Non-Functional Requirements

### Structure & Naming Rules
- Root folder structure: Not explicitly defined beyond `frontend/` and `backend/`.
- File Naming Rules: `kebab-case` MUST be used for folders and non-code files. `snake_case` MUST be used for Python files. `camelCase` MUST be used for frontend JavaScript.

### RAG Chatbot Requirements
- **Capabilities:** MUST answer only using book content (no hallucinations), support highlight-restricted RAG (user-selected text only), and provide citations to book sections.
- **Technical Requirements:** OpenAI `text-embedding-3-large` (or better) for embeddings model. Qdrant Cloud for vector DB. Neon Serverless Postgres for metadata DB. OpenAI ChatKit or Agents SDK for the Chat Agent. A caching layer is encouraged.
- **Performance Requirements:** Query latency MUST be < 1.5 seconds. Embedding cost MUST be minimized with chunk deduplication. Streaming responses MUST be supported.

### Performance Requirements
Embedding batch size MUST be optimized. FastAPI MUST respond under 200 ms (excluding LLM time). Docusaurus page MUST load in < 1.5 seconds. The chat widget MUST NOT block page render.

### UX Principles
The chatbot MUST be accessible from every page. Users MUST visually see response text, sources, and optionally confidence level. The UI MUST be minimalistic, clean, mobile-responsive, and support dark mode.

## Governance
All future specifications MUST follow this constitution. Changes to the constitution require a new `/sp.constitution` command and explicit confirmation from the user.

**Version**: 1.0.0 | **Ratified**: 2025-11-28 | **Last Amended**: 2025-11-28