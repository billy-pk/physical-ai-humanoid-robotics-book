---
id: 0001
title: Update Project Constitution
stage: constitution
date: 2025-11-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Unknown
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

# 📘 Project Constitution
### **Physical AI & Humanoid Robotics — Spec-Driven Book + RAG Chatbot**

This constitution defines the rules, structure, scope, expectations, and quality standards for the unified book and RAG chatbot project. It guides all future specifications generated via Spec-Kit Plus.

---

## **1. Project Identity**

- **Project Name:** Physical AI & Humanoid Robotics Book
- **Repository Folder:** `physical-ai-humanoid-robotics-book`
- **Core Deliverables:**
  - A fully authored **Spec-Driven book** using **Docusaurus**, deployed to GitHub Pages.
  - An integrated **RAG Chatbot** capable of answering questions strictly from book content.

---

## **2. Purpose & Vision**

This project teaches *Physical AI*—AI systems operating in the real world—by combining:

- Humanoid robotics (ROS 2, Gazebo, Unity, NVIDIA Isaac)
- Embodied intelligence concepts
- LLM-driven cognitive planning
- Real-world deployment to Jetson hardware
- Conversational RAG augmentation for students and readers

The vision:
**Create the definitive, accessible, AI-augmented textbook for Physical AI and Humanoid Robotics.**

---

## **3. Project Components**

### **3.1. Book (Frontend)**
- Written using **Spec-Kit Plus** → `/sp.specify` command
- Designed & rendered with **Docusaurus**
- Deployed to **GitHub Pages**
- Includes diagrams, examples, weekly breakdowns, module summaries
- Must contain high-quality human-readable explanations

### **3.2. RAG Chatbot (Backend)**
- Built with **FastAPI**
- Uses **OpenAI ChatKit / Agents SDK**
- Vector database: **Qdrant Cloud (Free Tier)**
- Runtime metadata DB: **Neon Postgres**
- Features:
  - General questions about the book
  - Highlighted-text-only answering
  - Supports streaming responses
  - Embeds directly into the Docusaurus frontend

---

## **4. Architecture Guidelines**

### **Frontend**
- `frontend/` → Docusaurus project
- Fully static, deployed via GitHub Pages
- Chat widget embedded in sidebar or floating bottom-right
- Documentation chapters auto-generated through Spec-Kit Plus

### **Backend**
- `backend/` → FastAPI app
- Routes:
  - `/embed` → embed text
  - `/search` → vector search pipeline
  - `/ask` → RAG completion using OpenAI Agents
- Deployed on **Render.com** (preferred)

### **Shared**
- Scripts for ingestion, chunking, embeddings extraction

---

## **5. Structure & Naming Rules**

### **5.1 Root Folder Structure**

### **5.2 File Naming Rules**
- Use `kebab-case` for folders and non-code files
- Use `snake_case` for Python
- Use `camelCase` for frontend JavaScript

---

## **6. Book Creation Principles**

1. All chapters are created via `/sp.specify` templates.
2. Each chapter includes:
   - Learning outcomes
   - Core explanations
   - Illustrations/diagrams
   - Real-world examples
   - Exercises or assessments
3. Writing style:
   - Beginner-friendly
   - Step-by-step
   - Real-world robotics orientation
   - Consistent tone, terminology, formatting
4. All chapters must be internally linked
   - Glossary
   - Module reference guides
   - Weekly breakdown

---

## **7. RAG Chatbot Requirements**

### **Capabilities**
- Answer only using the book content (no hallucinations)
- Highlight-restricted RAG (user-selected text only)
- Provide citations to book sections

### **Technical Requirements**
- Embeddings model: OpenAI text-embedding-3-large (or better)
- Vector DB: Qdrant Cloud
- Metadata DB: Neon Serverless Postgres
- Chat Agent: OpenAI ChatKit or Agents SDK
- Caching layer encouraged to reduce token cost

### **Performance Requirements**
- Query latency < 1.5 seconds
- Embedding cost minimized with chunk dedupe
- Must support streaming responses

---

## **8. Code Quality Principles**

1. **Linting**
   - Python: Ruff (PEP8 + flake8 compatibility)
   - JS/TS: ESLint + Prettier
2. **Type Safety**
   - Use TypeScript for all frontend code
3. **Error Handling**
   - Return structured JSON errors
   - Every API route must include exception wrappers
4. **Logging**
   - Structured logs using `loguru` or standard logging
5. **Transparency**
   - Chatbot must show "sources used" in the UI
6. **Security**
   - Never store API keys in git
   - Use Render / GitHub secrets

---

## **9. Testing Standards**

### **Backend Tests**
- Pytest for unit tests
- Tests include:
  - Embedding functions
  - Vector search pipeline
  - Postgres integration
  - FastAPI routes
- Minimum coverage: **70%**

### **Frontend Tests**
- Jest + React Testing Library

### **RAG Evaluation**
- Benchmark retrieval quality using:
  - Precision@k
  - Context recall
- Unit tests for chunking logic

---

## **10. Performance Requirements**

- Embedding batch size optimized to reduce API calls
- FastAPI must respond under 200 ms (excluding LLM time)
- Docusaurus page must load in < 1.5 seconds
- Chat widget must not block page render

---

## **11. UX Principles**

1. Chatbot must be accessible from every page
2. Users must visually see:
   - Response text
   - Sources
   - Confidence level (optional)
3. UI must be minimalistic, clean, mobile-responsive
4. Dark mode support required

---

## **12. Versioning & Release Rules**

- Every new chapter requires:
  - Git commit
  - Version bump in the book (frontmatter)
- Backend changes require:
  - Git tag + Render redeploy

---

## **13. Deployment Rules**

### **Frontend**
- Deploy with GitHub Pages via GitHub Actions
- Each successful commit → auto-build environment

### **Backend**
- Deploy to Render
- Automatic redeploy on `main` branch updates

---

## **14. Governance**

- All future specifications must follow this constitution
- Changes to the constitution require:
  - A new `/sp.constitution` command
  - Explicit confirmation

## Response snapshot

Updated the project constitution in `.specify/memory/constitution.md` to version 1.0.0, incorporating new principles and sections related to book creation, code quality, testing standards, versioning, and deployment.

## Outcome

- ✅ Impact: Successfully updated the core project governance document.
- 🧪 Tests: N/A
- 📁 Files: Modified 1 file: .specify/memory/constitution.md
- 🔁 Next prompts: Review remaining template alignments, then commit changes.
- 🧠 Reflection: Successfully parsed the user's constitution input, populated a new constitution, and generated a PHR. Need to ensure dependent templates are updated.

## Evaluation notes (flywheel)

- Failure modes observed: None observed.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
