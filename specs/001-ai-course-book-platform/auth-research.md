# Research: Better Auth Integration & User Personalization

**Date**: 2025-11-30  
**Feature**: 001-ai-course-book-platform  
**Phase**: Authentication & User Personalization

## Overview

This document consolidates research findings for implementing user authentication using Better Auth and user background-based content personalization in the AI-Course-Book Platform.

---

## AUTH-001: Better Auth Integration Options for FastAPI + Docusaurus

### Decision: Standalone Node.js Better Auth Service

**Rationale**:
- Better Auth is TypeScript/Node.js native and doesn't have official Python/FastAPI support
- FastAPI backend can remain focused on RAG chatbot and business logic
- Better Auth can run as a separate service on the same infrastructure
- Both services share the same Neon Postgres database
- Clean separation of concerns: auth service vs. application service

**Architecture**:
```
┌─────────────────┐         ┌──────────────────┐         ┌──────────────┐
│   Docusaurus    │────────▶│  Better Auth     │────────▶│   Neon       │
│   Frontend      │         │  Service (Node)  │         │   Postgres   │
│                 │         │  Port: 3001      │         │              │
└─────────────────┘         └──────────────────┘         └──────────────┘
       │                              │                            │
       │                              │                            │
       └──────────────────────────────┼────────────────────────────┘
                                      │
                            ┌─────────▼──────────┐
                            │   FastAPI Backend  │
                            │   Port: 8000       │
                            │   (RAG Chatbot)    │
                            └────────────────────┘
```

**Implementation Approach**:
1. Create `backend/auth-service/` directory with Node.js/TypeScript setup
2. Install Better Auth: `npm install better-auth`
3. Configure Better Auth with Neon Postgres adapter
4. Run Better Auth service on port 3001 (or configurable)
5. Frontend calls Better Auth directly for authentication
6. FastAPI validates sessions by querying Better Auth's database tables
7. FastAPI manages user profiles and questionnaire data

**Alternatives Considered**:
- **Option A**: Python auth library (like FastAPI-Users)
  - Rejected: User specifically requested Better Auth
- **Option B**: Better Auth client-side only
  - Rejected: Need server-side session validation for FastAPI endpoints
- **Option C**: Proxy Better Auth through FastAPI
  - Rejected: Adds complexity, Better Auth handles its own routes better

**Integration Points**:
- Better Auth exposes `/api/auth/*` endpoints
- FastAPI reads `user` and `session` tables from shared database
- Frontend uses Better Auth client SDK for auth operations
- FastAPI uses SQLAlchemy to query user data for personalization

---

## AUTH-002: User Background Questionnaire Schema

### Decision: Structured JSON Schema with Categorical Fields

**Questionnaire Fields**:

```typescript
interface UserBackground {
  // Software Experience (multi-select)
  software_background: string[]; // ["Python", "JavaScript", "C++", "None"]
  
  // Hardware Experience (multi-select)
  hardware_background: string[]; // ["Arduino", "Raspberry Pi", "ROS", "None"]
  
  // Experience Level (single select)
  experience_level: "beginner" | "intermediate" | "advanced";
  
  // Learning Goals (free text)
  learning_goals: string; // Max 500 characters
  
  // Prior Robotics Projects (boolean + description)
  has_robotics_projects: boolean;
  robotics_projects_description?: string; // Required if has_robotics_projects is true
  
  // Programming Experience Years (optional)
  programming_years?: number; // 0-50
  
  // Preferred Learning Style (optional)
  learning_style?: "visual" | "hands-on" | "theoretical" | "mixed";
}
```

**Database Storage**:
- Store in `user_profiles` table as JSONB column for flexibility
- Also create normalized tables for analytics: `user_software_skills`, `user_hardware_skills`

**Validation Rules**:
- `software_background`: At least one selection OR "None"
- `hardware_background`: At least one selection OR "None"
- `experience_level`: Required
- `learning_goals`: Optional but recommended (max 500 chars)
- `has_robotics_projects`: Required boolean
- `robotics_projects_description`: Required if `has_robotics_projects` is true

---

## AUTH-003: Database Schema Design

### Tables Required

**1. `users` table** (managed by Better Auth):
- `id` (UUID, primary key)
- `email` (string, unique, indexed)
- `name` (string)
- `email_verified` (boolean)
- `image` (string, nullable)
- `created_at` (timestamp)
- `updated_at` (timestamp)

**2. `sessions` table** (managed by Better Auth):
- `id` (UUID, primary key)
- `user_id` (UUID, foreign key to users.id)
- `expires_at` (timestamp, indexed)
- `token_hash` (string)
- `created_at` (timestamp)

**3. `user_profiles` table** (managed by FastAPI):
- `user_id` (UUID, primary key, foreign key to users.id)
- `software_background` (JSONB) - Array of software skills
- `hardware_background` (JSONB) - Array of hardware skills
- `experience_level` (enum: beginner/intermediate/advanced)
- `learning_goals` (text, max 500 chars)
- `has_robotics_projects` (boolean)
- `robotics_projects_description` (text, nullable)
- `programming_years` (integer, nullable)
- `learning_style` (enum, nullable)
- `questionnaire_completed` (boolean, default false)
- `questionnaire_completed_at` (timestamp, nullable)
- `preferences` (JSONB) - For future extensibility
- `created_at` (timestamp)
- `updated_at` (timestamp)

**4. `user_background_questionnaire` table** (optional, for detailed tracking):
- `id` (UUID, primary key)
- `user_id` (UUID, foreign key to users.id)
- `question_id` (string) - e.g., "software_python", "hardware_arduino"
- `answer` (text or JSONB)
- `answered_at` (timestamp)

**Indexes**:
- `users.email` (unique index)
- `sessions.user_id` (index)
- `sessions.expires_at` (index for cleanup)
- `user_profiles.user_id` (unique index)
- `user_background_questionnaire.user_id` (index)

**Relationships**:
- `users` 1:1 `user_profiles`
- `users` 1:many `sessions`
- `users` 1:many `user_background_questionnaire`
- `users` 1:many `chat_sessions` (update existing table)

---

## AUTH-004: API Contracts Design

### Better Auth Endpoints (handled by Better Auth service)

**POST /api/auth/sign-up/email**
- Request: `{ email, password, name }`
- Response: `{ user: {...}, session: {...} }`

**POST /api/auth/sign-in/email**
- Request: `{ email, password, rememberMe? }`
- Response: `{ user: {...}, session: {...} }`

**POST /api/auth/sign-out**
- Request: (authenticated)
- Response: `{ success: true }`

**GET /api/auth/session**
- Request: (cookies/headers)
- Response: `{ user: {...}, session: {...} }` or `null`

### FastAPI Endpoints (handled by FastAPI backend)

**POST /api/auth/profile/background**
- Authentication: Required (Bearer token or session cookie)
- Request: `UserBackground` object
- Response: `{ success: true, profile: {...} }`
- Validates questionnaire data and saves to `user_profiles`

**GET /api/auth/profile**
- Authentication: Required
- Response: `{ user: {...}, profile: {...} }`
- Returns user info from Better Auth + profile from FastAPI

**PUT /api/auth/profile**
- Authentication: Required
- Request: Partial `UserBackground` object
- Response: `{ success: true, profile: {...} }`
- Updates user profile

**GET /api/personalization/recommendations**
- Authentication: Optional (enhanced if authenticated)
- Response: `{ recommended_modules: [...], recommended_chapters: [...] }`
- Uses user background to recommend content

**POST /api/chat** (updated)
- Authentication: Optional
- Request: `{ query, session_id?, highlighted_context?, user_id? }`
- Response: Personalized response based on user background (if authenticated)

---

## AUTH-005: Content Personalization Strategy

### Personalization Rules

**1. Module Recommendations**:
- **Beginner + No Software Background**: Start with Module 1 (Foundations)
- **Intermediate + Python Background**: Can skip to Module 2 (Computer Vision)
- **Advanced + Hardware Background**: Highlight advanced modules, suggest skipping basics

**2. Chapter Filtering**:
- Show all chapters but highlight recommended ones
- Add "Beginner-friendly" badges for simple chapters
- Add "Advanced" badges for complex chapters
- Filter out chapters below user's level (optional, configurable)

**3. Chat Personalization**:
- Adjust RAG agent instructions based on `experience_level`
- For beginners: Use simpler language, more explanations
- For advanced: Assume more knowledge, provide deeper technical details
- Include user's software/hardware background in RAG context

**4. Learning Path Suggestions**:
- Generate personalized learning path based on:
  - Experience level
  - Software background (emphasize relevant chapters)
  - Hardware background (emphasize relevant chapters)
  - Learning goals (if provided)

**5. Progress Tracking** (future):
- Track which chapters user has read
- Track which chapters user has completed exercises for
- Show progress dashboard

### Implementation Approach

**Backend** (`backend/src/services/personalization.py`):
```python
def get_recommended_modules(user_profile: UserProfile) -> List[str]:
    # Logic based on experience_level and background
    
def get_personalized_chat_context(user_profile: UserProfile) -> str:
    # Add context about user's background to RAG agent
    
def filter_chapters_by_level(chapters: List[Chapter], level: str) -> List[Chapter]:
    # Filter chapters based on difficulty
```

**Frontend** (`frontend/src/hooks/usePersonalization.ts`):
```typescript
export function usePersonalization() {
  // Fetch recommendations
  // Display personalized content
  // Update UI based on user profile
}
```

**RAG Agent Integration**:
- Modify `rag_agent.py` to accept user context
- Include user background in system prompt
- Adjust response style based on experience level

---

## Technology Stack Decisions

### Better Auth Configuration

**Database Adapter**: Neon Postgres (using `@better-auth/postgres` or custom adapter)
**Authentication Methods**: Email/Password (primary), future: OAuth providers
**Session Management**: Cookie-based sessions (secure, httpOnly)
**Password Policy**: Min 8 chars, max 128 chars (Better Auth default)

### FastAPI Integration

**Session Validation**: Query `sessions` table directly using SQLAlchemy
**User Profile Management**: Custom FastAPI endpoints with SQLAlchemy models
**Authentication Middleware**: Custom middleware to validate Better Auth sessions

### Frontend Integration

**Better Auth Client**: `better-auth` npm package
**React Hooks**: Custom hooks wrapping Better Auth client
**State Management**: React Context for auth state
**Protected Routes**: Custom component wrapping Docusaurus pages

---

## Deployment Considerations

### Development
- Better Auth service: `localhost:3001`
- FastAPI backend: `localhost:8000`
- Frontend: `localhost:3000`

### Production
- Better Auth service: Deploy to Render as separate service (or same service with subdomain)
- FastAPI backend: Existing Render deployment
- Frontend: GitHub Pages (static, calls both services)

### Environment Variables

**Better Auth Service**:
- `DATABASE_URL`: Neon Postgres connection string
- `BETTER_AUTH_SECRET`: Secret key for session encryption
- `BETTER_AUTH_URL`: Public URL of Better Auth service
- `BETTER_AUTH_TRUSTED_ORIGINS`: Frontend domain(s)

**FastAPI Backend**:
- `NEON_DATABASE_URL`: Same Neon Postgres (for user profile queries)
- `BETTER_AUTH_SERVICE_URL`: URL of Better Auth service (for session validation)

**Frontend**:
- `NEXT_PUBLIC_BETTER_AUTH_URL`: Public URL of Better Auth service
- `NEXT_PUBLIC_API_URL`: Public URL of FastAPI backend

---

## Security Considerations

1. **Session Validation**: FastAPI must validate sessions by checking database, not trusting client
2. **CORS**: Configure CORS properly for Better Auth service
3. **Cookies**: Use secure, httpOnly cookies for sessions
4. **Password Hashing**: Better Auth handles this automatically
5. **Rate Limiting**: Apply to auth endpoints (Better Auth may have built-in)
6. **Email Verification**: Consider requiring email verification (Better Auth supports this)

---

## Next Steps

1. ✅ Research complete - Standalone Node.js service approach selected
2. ✅ Schema designed - User background questionnaire structure defined
3. ✅ Database schema designed - Tables and relationships documented
4. ✅ API contracts designed - Endpoints for auth and profiles defined
5. ✅ Personalization strategy planned - Rules and implementation approach documented

**Ready for Implementation**: Phase 2 (Database Setup) can begin.
