---
id: 0006
title: Execute implementation planning workflow
stage: plan
date: 2025-11-30
surface: agent
model: claude-sonnet-4.5
feature: 001-ai-course-book-platform
branch: 001-ai-course-book-platform
user: billy-pk
command: /sp.plan
labels: ["planning", "workflow", "architecture"]
links:
  spec: specs/001-ai-course-book-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-course-book-platform/plan.md
  - .cursor/rules/specify-rules.mdc
tests:
  - null
---

## Prompt

---
description: Execute the implementation planning workflow using the plan template to generate design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/bash/setup-plan.sh --json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/bash/update-agent-context.sh cursor-agent`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

## Response snapshot

Executed the implementation planning workflow for feature `001-ai-course-book-platform`:

1. **Setup**: Ran `.specify/scripts/bash/setup-plan.sh --json` to get paths:
   - FEATURE_SPEC: `specs/001-ai-course-book-platform/spec.md`
   - IMPL_PLAN: `specs/001-ai-course-book-platform/plan.md`
   - BRANCH: `001-ai-course-book-platform`

2. **Loaded context**: Read spec.md and constitution.md to understand requirements and principles.

3. **Filled plan.md template**: Updated plan.md with:
   - Summary: Documentation platform with RAG chatbot
   - Technical Context: Python 3.12 backend, TypeScript frontend, FastAPI, Docusaurus, Qdrant, Neon Postgres
   - Constitution Check: All checks passed ✓
   - Project Structure: Web application structure with backend/ and frontend/ directories

4. **Verified Phase 0 artifacts**: research.md already exists and is complete with all technology decisions documented.

5. **Verified Phase 1 artifacts**: 
   - data-model.md exists with entity definitions
   - contracts/api-spec.yaml exists with API specification
   - quickstart.md exists with setup instructions

6. **Updated agent context**: Ran `.specify/scripts/bash/update-agent-context.sh cursor-agent` to update `.cursor/rules/specify-rules.mdc` with project technology stack.

**Result**: Planning workflow complete. All Phase 0 and Phase 1 artifacts verified. Plan.md updated with current project state. Agent context file updated with technology stack information.

## Outcome

- ✅ Impact: Planning workflow executed successfully. Plan.md now contains complete technical context, constitution compliance check, and project structure. Agent context updated with current technology stack (Python 3.12, FastAPI, Docusaurus, TypeScript, Qdrant, Neon Postgres).
- 🧪 Tests: No tests required for planning phase
- 📁 Files: Updated `specs/001-ai-course-book-platform/plan.md` and `.cursor/rules/specify-rules.mdc`
- 🔁 Next prompts: Continue with `/sp.tasks` to generate implementation tasks, or proceed with development based on existing tasks.md
- 🧠 Reflection: Planning artifacts were already present from previous work. The workflow successfully validated completeness and updated the plan.md template with current project state. Agent context update ensures Cursor IDE has current technology information.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (planning phase)
- Prompt variant (if applicable): Standard planning workflow
- Next experiment (smallest change to try): N/A
