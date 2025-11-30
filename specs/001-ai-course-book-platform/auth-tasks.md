# Tasks: Authentication & User Personalization with Better Auth

**Feature**: User Authentication and Personalization  
**Input**: Requirement to implement signup/signin using Better Auth with user background questionnaire for content personalization  
**Prerequisites**: Existing platform (backend FastAPI, frontend Docusaurus)

**Organization**: Tasks are grouped by implementation phase to enable incremental delivery.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

---

## Phase 1: Research & Planning

**Purpose**: Understand Better Auth integration approach and design user background schema

- [X] AUTH-001 Research Better Auth integration options for FastAPI + Docusaurus architecture
- [X] AUTH-002 Design user background questionnaire schema (software/hardware experience fields)
- [X] AUTH-003 Design database schema for users and user_profiles tables
- [X] AUTH-004 Design API contracts for authentication endpoints and user profile management
- [X] AUTH-005 Plan content personalization strategy based on user background data

**Checkpoint**: Architecture decisions documented, schema designed, ready for implementation

**Research Document**: See `auth-research.md` for detailed findings and decisions.

---

## Phase 2: Backend Database Setup

**Purpose**: Create database tables for user profiles (Better Auth creates user/session tables)

**⚠️ IMPORTANT**: Better Auth will create `user` and `session` tables automatically. We only create complementary tables.

- [ ] AUTH-006 Verify Better Auth has created `user` and `session` tables (check after Better Auth setup)
- [ ] AUTH-007 Create Alembic migration to add `user_id` (text, nullable) to `chat_sessions` table with foreign key to `user.id`
- [ ] AUTH-008 [P] Create Alembic migration for `user_profiles` table (user_id TEXT FK to user.id, software_background JSONB, hardware_background JSONB, experience_level, learning_goals, etc.)
- [ ] AUTH-009 [P] Create Alembic migration for `user_background_questionnaire` table (id UUID, user_id TEXT FK, question_id, answer JSONB, answered_at)
- [ ] AUTH-010 Add foreign key constraints (CASCADE delete) and indexes for user-related tables
- [ ] AUTH-011 Run migrations and verify schema creation

**Checkpoint**: Database schema ready for user data storage. Better Auth tables exist, FastAPI profile tables created.

**Note**: See `alembic-setup-review.md` for detailed migration plan and schema definitions.

---

## Phase 3: Better Auth Server Setup

**Purpose**: Set up Better Auth authentication service

- [X] AUTH-012 Create auth service directory structure: `backend/auth-service/` (standalone Node.js service)
- [X] AUTH-013 [P] Initialize Node.js/TypeScript project for Better Auth service
- [X] AUTH-014 [P] Install Better Auth: `npm install better-auth` (v1.3.10 installed)
- [X] AUTH-015 Configure Better Auth with Neon Postgres database connection (using pg Pool)
- [X] AUTH-016 Configure Better Auth email/password authentication (enabled, min 8 chars, auto sign-in)
- [X] AUTH-017 Configure Better Auth session management (7-day expiration, 24h update age)
- [X] AUTH-018 Set up Better Auth environment variables (.env.example created with all required vars)
- [X] AUTH-019 Create Better Auth API route handler (Express server with /api/auth/* routes)
- [X] AUTH-020 Test Better Auth signup/signin endpoints locally ✅ All tests passing

**Checkpoint**: Better Auth service configured and ready to run. Service structure complete, needs .env file to start.

**Files Created**:
- `backend/auth-service/package.json` - Node.js project configuration
- `backend/auth-service/tsconfig.json` - TypeScript configuration
- `backend/auth-service/src/auth.ts` - Better Auth instance configuration
- `backend/auth-service/src/server.ts` - Express server with auth routes
- `backend/auth-service/src/config.ts` - Environment configuration
- `backend/auth-service/.env.example` - Environment variables template
- `backend/auth-service/README.md` - Service documentation
- `backend/auth-service/SETUP.md` - Setup guide
- `backend/auth-service/test-auth.ts` - Test script

---

## Phase 4: User Background Questionnaire Backend

**Purpose**: Create backend endpoints for collecting and managing user background information

- [ ] AUTH-021 Create backend/src/models/user.py with User, UserProfile, UserBackgroundQuestionnaire models
- [ ] AUTH-022 Create backend/src/api/routes/auth.py with user profile endpoints
- [ ] AUTH-023 [P] Implement POST /api/auth/profile/background endpoint to save questionnaire answers
- [ ] AUTH-024 [P] Implement GET /api/auth/profile endpoint to retrieve user profile with background
- [ ] AUTH-025 [P] Implement PUT /api/auth/profile endpoint to update user profile
- [ ] AUTH-026 Create backend/src/services/user_profile.py for user profile business logic
- [ ] AUTH-027 Implement questionnaire answer validation (required fields, data types)
- [ ] AUTH-028 Add middleware to authenticate requests using Better Auth session/token
- [ ] AUTH-029 Create backend/src/core/auth.py for Better Auth integration utilities

**Checkpoint**: Backend endpoints ready for user profile and background data management

---

## Phase 5: Frontend Better Auth Integration

**Purpose**: Integrate Better Auth client into Docusaurus frontend

- [ ] AUTH-030 Install Better Auth client: `npm install better-auth` in frontend/
- [ ] AUTH-031 Create frontend/src/lib/auth.ts with Better Auth client configuration
- [ ] AUTH-032 Create frontend/src/contexts/AuthContext.tsx for React auth context provider
- [ ] AUTH-033 Create frontend/src/hooks/useAuth.ts for authentication hooks
- [ ] AUTH-034 Create frontend/src/components/Auth/SignUpForm.tsx with email/password signup form
- [ ] AUTH-035 Create frontend/src/components/Auth/SignInForm.tsx with email/password signin form
- [X] AUTH-036 Create frontend/src/components/Auth/UserBackgroundQuestionnaire.tsx for collecting background info ✅
- [X] AUTH-037 Integrate questionnaire component into signup flow (show after successful signup) ✅
- [X] AUTH-038 Create frontend/src/components/Auth/ProtectedRoute.tsx for protecting authenticated routes ✅
- [ ] AUTH-039 Update frontend/src/theme/Root.tsx to include AuthContext provider

**Checkpoint**: Frontend authentication UI components ready

---

## Phase 6: User Background Questionnaire Frontend

**Purpose**: Implement questionnaire UI and integration with signup flow

- [X] AUTH-040 Design questionnaire form fields ✅
- [X] AUTH-041 Create frontend/src/components/Auth/UserBackgroundQuestionnaire.tsx with all questionnaire fields ✅
- [X] AUTH-042 Implement form validation for questionnaire (required fields, format validation) ✅
- [X] AUTH-043 Add progress indicator for multi-step signup (signup → questionnaire → complete) ✅
- [X] AUTH-044 Implement questionnaire submission to backend API ✅
- [X] AUTH-045 Add error handling and success messages for questionnaire submission ✅
- [X] AUTH-046 Store questionnaire completion status in user profile (backend) ✅
- [X] AUTH-047 Create QuestionnaireGuard component to redirect users who haven't completed questionnaire ✅

**Checkpoint**: Complete signup flow with background questionnaire functional

---

## Phase 7: Content Personalization Backend

**Purpose**: Implement backend logic to personalize content based on user background

- [ ] AUTH-048 Create backend/src/services/personalization.py for content personalization logic
- [ ] AUTH-049 Implement function to determine user's recommended starting module based on background
- [ ] AUTH-050 Implement function to filter/sort chapters by relevance to user's background
- [ ] AUTH-051 Create backend/src/api/routes/personalization.py with GET /api/personalization/recommendations endpoint
- [ ] AUTH-052 Integrate personalization into chat endpoint (modify RAG agent context based on user background)
- [ ] AUTH-053 Add user_id to chat_sessions table (migration) to link sessions to users
- [ ] AUTH-054 Update chat endpoint to use authenticated user's background for personalized responses
- [ ] AUTH-055 Create backend/src/services/content_filter.py for filtering content by user level

**Checkpoint**: Backend personalization logic ready to serve personalized content

---

## Phase 8: Content Personalization Frontend

**Purpose**: Display personalized content recommendations and adapt UI based on user background

- [ ] AUTH-056 Create frontend/src/components/Personalization/RecommendedContent.tsx component
- [ ] AUTH-057 Create frontend/src/components/Personalization/UserDashboard.tsx showing personalized dashboard
- [ ] AUTH-058 Update frontend homepage to show personalized recommendations for authenticated users
- [ ] AUTH-059 Update sidebar navigation to highlight recommended modules/chapters
- [ ] AUTH-060 Add "Your Progress" section showing completed chapters based on user level
- [ ] AUTH-061 Update ChatWidget to use user background context when making requests
- [ ] AUTH-062 Create frontend/src/hooks/usePersonalization.ts for fetching personalized content
- [ ] AUTH-063 Add visual indicators (badges, highlights) for content matching user's background

**Checkpoint**: Frontend displays personalized content based on user background

---

## Phase 9: Authentication UI Polish

**Purpose**: Improve authentication UX and add missing features

- [ ] AUTH-064 Create frontend/src/components/Auth/ForgotPasswordForm.tsx for password reset
- [ ] AUTH-065 Create frontend/src/components/Auth/ResetPasswordForm.tsx for password reset confirmation
- [ ] AUTH-066 Implement "Remember me" functionality in signin form
- [ ] AUTH-067 Add loading states and error messages to all auth forms
- [ ] AUTH-068 Create frontend/src/components/Auth/UserMenu.tsx dropdown for authenticated users
- [ ] AUTH-069 Add sign out functionality
- [ ] AUTH-070 Add user profile page showing background information and ability to update
- [ ] AUTH-071 Style authentication forms to match Docusaurus theme (dark mode support)
- [ ] AUTH-072 Add form validation feedback (real-time validation, error messages)
- [ ] AUTH-073 Test authentication flow on mobile devices (responsive design)

**Checkpoint**: Complete authentication UI with all features polished

---

## Phase 10: Integration & Testing

**Purpose**: Integrate authentication with existing features and test end-to-end

- [ ] AUTH-074 Update chat endpoint to require authentication (or make optional with enhanced features for authenticated users)
- [ ] AUTH-075 Link chat sessions to authenticated users (update chat_sessions table)
- [ ] AUTH-076 Update rate limiting middleware to consider authenticated users (higher limits)
- [ ] AUTH-077 Test complete signup flow: signup → questionnaire → personalized dashboard
- [ ] AUTH-078 Test signin flow: signin → access personalized content → chat with context
- [ ] AUTH-079 Test password reset flow end-to-end
- [ ] AUTH-080 Test session persistence and expiration
- [ ] AUTH-081 Test personalization: verify content recommendations match user background
- [ ] AUTH-082 Test chat personalization: verify RAG responses adapt to user's experience level
- [ ] AUTH-083 Add integration tests for authentication endpoints
- [ ] AUTH-084 Add integration tests for user profile endpoints
- [ ] AUTH-085 Add E2E tests for signup/signin flows

**Checkpoint**: Authentication fully integrated and tested

---

## Phase 11: Documentation & Deployment

**Purpose**: Document authentication features and prepare for deployment

- [ ] AUTH-086 Update backend/README.md with Better Auth setup instructions
- [ ] AUTH-087 Update frontend/README.md with authentication setup and usage
- [ ] AUTH-088 Document environment variables needed for Better Auth (BETTER_AUTH_SECRET, etc.)
- [ ] AUTH-089 Update quickstart.md with authentication setup steps
- [ ] AUTH-090 Create user guide for signup/signin and profile management
- [ ] AUTH-091 Update API documentation (contracts/api-spec.yaml) with auth endpoints
- [ ] AUTH-092 Configure Better Auth for production (secure cookies, HTTPS, domain settings)
- [ ] AUTH-093 Set up email service for password reset and verification (if required)
- [ ] AUTH-094 Update deployment scripts to include Better Auth service (if separate)
- [ ] AUTH-095 Test deployment on staging environment

**Checkpoint**: Authentication feature documented and ready for production

---

## Task Summary

- **Total Tasks**: 95
- **Phase 1**: 5 tasks - Research & Planning
- **Phase 2**: 6 tasks - Database Setup
- **Phase 3**: 9 tasks - Better Auth Server Setup
- **Phase 4**: 9 tasks - User Background Questionnaire Backend
- **Phase 5**: 10 tasks - Frontend Better Auth Integration
- **Phase 6**: 8 tasks - User Background Questionnaire Frontend
- **Phase 7**: 8 tasks - Content Personalization Backend
- **Phase 8**: 8 tasks - Content Personalization Frontend
- **Phase 9**: 10 tasks - Authentication UI Polish
- **Phase 10**: 12 tasks - Integration & Testing
- **Phase 11**: 10 tasks - Documentation & Deployment

**Parallel Opportunities Identified**: Tasks marked [P] can run concurrently

---

## Dependencies & Execution Order

### Critical Path
1. **Phase 1** (Research) → Must complete before implementation
2. **Phase 2** (Database) → Blocks backend development
3. **Phase 3** (Better Auth Setup) → Blocks authentication features
4. **Phase 4** (Backend API) → Can run parallel with Phase 5
5. **Phase 5** (Frontend Auth) → Can run parallel with Phase 4
6. **Phase 6** (Questionnaire UI) → Depends on Phase 5
7. **Phase 7** (Personalization Backend) → Depends on Phase 4
8. **Phase 8** (Personalization Frontend) → Depends on Phase 6 and Phase 7
9. **Phase 9** (UI Polish) → Can run parallel with Phase 8
10. **Phase 10** (Integration) → Depends on all previous phases
11. **Phase 11** (Documentation) → Can start early, complete after Phase 10

### Key Integration Points
- Better Auth service must be accessible from both frontend and backend
- User profile data must be synchronized between Better Auth and FastAPI backend
- Chat personalization requires authenticated user context
- Questionnaire completion must trigger personalization updates

---

## Notes

- Better Auth is TypeScript-based and may require a Node.js service or integration layer
- Consider using Better Auth's adapter pattern if available for Python/FastAPI
- User background questionnaire should be optional but encouraged (can skip and complete later)
- Personalization should gracefully degrade if user hasn't completed questionnaire
- Maintain backward compatibility: allow anonymous chat but enhance for authenticated users
- Consider privacy: user background data should be stored securely and only used for personalization
