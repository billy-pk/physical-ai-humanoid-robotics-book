# Tasks: User Personalization Module

**Input**: Design documents from `/specs/002-user-personalization-module/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/personalization-api.openapi.yaml

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Implementation Strategy

**MVP**: User Story 1 (P1) - Preference Collection After Login

**Incremental Delivery**:
1. MVP: US1 (preference collection + storage)
2. +US2 (full content mode infrastructure)
3. +US3 (personalized content generation)
4. +US4 (Urdu translation)
5. +US5 (preference management UI)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and verify existing infrastructure

- [x] T001 Verify Python 3.12+ and Node.js 20+ installed
- [x] T002 [P] Verify backend dependencies: FastAPI 0.122.0, OpenAI Agents SDK 0.6.1, Pydantic 2.12+, psycopg3 3.2.13+, Alembic 1.17.2+
- [x] T003 [P] Verify frontend dependencies: Docusaurus 3.9.2, React 19, TypeScript, Better Auth Client
- [x] T004 [P] Verify OPENAI_API_KEY is present in backend/.env
- [x] T005 [P] Verify Neon Postgres connection string in backend/.env
- [x] T006 [P] Verify Better Auth service is running on port 3001

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Schema

- [x] T007 Create Alembic migration for personalized_content_cache table in backend/alembic/versions/[timestamp]_create_personalized_content_cache.py
- [x] T008 Run Alembic migration to create personalized_content_cache table

### Backend Models

- [x] T009 [P] Create PersonalizationPreferences Pydantic model in backend/src/models/user.py
- [x] T010 [P] Create PersonalizedContentCacheEntry Pydantic model in backend/src/models/user.py
- [x] T011 [P] Add validation rules for PersonalizationPreferences (experience_level enum, learning_topics length, learning_goals max 500 chars)

### Personalization Services Infrastructure

- [x] T012 Create backend/src/services/personalization/ directory
- [x] T013 [P] Create PersonalizationCache class in backend/src/services/personalization/cache.py with _compute_preferences_hash and _compute_content_hash methods
- [x] T014 [P] Implement get_cached method in PersonalizationCache with database query and hash matching
- [x] T015 [P] Implement set_cached method in PersonalizationCache with upsert logic and conflict handling

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Preference Collection After Login (Priority: P1) 🎯 MVP

**Goal**: Collect user personalization preferences after login via /popup page and store them in the database

**Independent Test**: Create new account, log in, get redirected to /popup, fill preference form, submit, verify data saved to user_profiles.preferences and redirect to main content

### Backend - Preference API

- [x] T016 [P] [US1] Extend user_profile.py service to add get_personalization_preferences(user_id) function in backend/src/services/user_profile.py
- [x] T017 [P] [US1] Extend user_profile.py service to add update_personalization_preferences(user_id, preferences) function with preferences_version increment logic
- [x] T018 [US1] Add GET /api/auth/profile/preferences endpoint in backend/src/api/routes/auth.py
- [x] T019 [US1] Add POST /api/auth/profile/preferences endpoint in backend/src/api/routes/auth.py with validation and error handling
- [x] T020 [US1] Add authentication middleware check for both preference endpoints using get_current_user dependency

### Frontend - Preference Form UI

- [x] T021 [P] [US1] Create PersonalizationPreferences TypeScript interface in frontend/src/types/personalization.ts
- [x] T022 [P] [US1] Create PreferenceForm component in frontend/src/components/Auth/PreferenceForm.tsx with all form fields (experience_level, learning_topics, learning_goals, content_mode, urdu_translation_enabled)
- [x] T023 [P] [US1] Add form validation to PreferenceForm (required fields, learning_goals max 500 chars, learning_topics max 10)
- [x] T024 [US1] Add form submission handler in PreferenceForm that POSTs to /api/auth/profile/preferences
- [x] T025 [US1] Add loading state and error handling to PreferenceForm
- [x] T026 [US1] Create /popup page in frontend/src/pages/popup.tsx that renders PreferenceForm component
- [x] T027 [US1] Add redirect to homepage after successful preference submission in popup.tsx

### Frontend - Route Guard

- [x] T028 [US1] Create PersonalizationGuard component in frontend/src/components/Auth/PersonalizationGuard.tsx
- [x] T029 [US1] Implement useEffect logic in PersonalizationGuard to check if user has submitted preferences via /api/auth/profile API call
- [x] T030 [US1] Add redirect logic to /popup if preferences not submitted (check for experience_level and content_mode presence)
- [x] T031 [US1] Update frontend/src/theme/Root.tsx to wrap app with PersonalizationGuard component

### Frontend - Personalization Context

- [x] T032 [P] [US1] Create PersonalizationContext in frontend/src/contexts/PersonalizationContext.tsx with preferences state and loading state
- [x] T033 [P] [US1] Implement fetchPreferences function in PersonalizationContext that calls GET /api/auth/profile/preferences
- [x] T034 [P] [US1] Implement updatePreferences function in PersonalizationContext that calls POST /api/auth/profile/preferences
- [x] T035 [US1] Add useEffect to fetch preferences on mount when user is authenticated
- [x] T036 [US1] Create usePersonalization custom hook in frontend/src/hooks/usePersonalization.ts that returns context values

**Checkpoint**: At this point, User Story 1 (preference collection) should be fully functional - users can submit preferences after login and data persists

---

## Phase 4: User Story 2 - Full Content View Mode (Priority: P2)

**Goal**: Display original, unmodified chapter content when user selects "full content" mode

**Independent Test**: Set user preference to "full content" mode, navigate to any chapter, verify original content displays without modifications

### Backend - Content Loading

- [x] T037 [P] [US2] Create get_chapter_content(chapter_id) function in backend/src/services/content_loader.py that reads chapter markdown files from frontend/docs/ directory
- [x] T038 [P] [US2] Add chapter path resolution logic to support module-*/chapter-* naming pattern

### Backend - Content API

- [x] T039 [US2] Create backend/src/api/routes/content.py with FastAPI router
- [x] T040 [US2] Add GET /api/content/chapters/{chapter_id} endpoint that checks user content_mode preference and returns full content if mode is "full"
- [x] T041 [US2] Add authentication middleware to content routes using get_current_user dependency
- [x] T042 [US2] Add structured error handling for chapter not found (404) and auth errors (401)
- [x] T043 [US2] Register content router in backend/src/main.py

### Frontend - Content Display Components

- [x] T044 [P] [US2] Create FullView component in frontend/src/components/Content/FullView.tsx that renders markdown content
- [x] T045 [P] [US2] Add markdown rendering logic to FullView using Docusaurus MDX components
- [x] T046 [P] [US2] Create ContentModeSwitch component in frontend/src/components/Content/ContentModeSwitch.tsx with toggle UI
- [x] T047 [US2] Add onChange handler in ContentModeSwitch that calls updatePreferences from PersonalizationContext
- [x] T048 [US2] Integrate FullView into chapter layout based on user content_mode preference

**Checkpoint**: At this point, User Stories 1 AND 2 work independently - users can select full content mode and see original chapters

---

## Phase 5: User Story 3 - Personalized Content Generation (Priority: P3)

**Goal**: Generate personalized content adapted to user's experience level, topics, and goals using OpenAI Agents SDK

**Independent Test**: Set preferences to "personalized" mode with specific level/topics/goals, view a chapter, verify content is dynamically generated and differs from full version

### Backend - Content Personalization Agent

- [x] T049 [P] [US3] Create @function_tool fetch_chapter_content in backend/src/services/personalization/content_generator.py
- [x] T050 [P] [US3] Create personalization_agent with gpt-4o-mini model and detailed instructions for content adaptation based on user level in content_generator.py
- [x] T051 [US3] Add level-specific instruction guidelines (beginner: simple language, intermediate: application focus, advanced: edge cases)
- [x] T052 [US3] Add explicit code block preservation instruction to agent
- [x] T053 [US3] Implement generate_personalized_content(chapter_id, user_level, user_topics, user_goals) async function in content_generator.py
- [x] T054 [US3] Add Runner.run execution logic with proper prompt formatting
- [x] T055 [US3] Add error handling and fallback to original content on OpenAI API failure

### Backend - Content Generation API with Caching

- [x] T056 [US3] Add POST /api/content/personalize endpoint in backend/src/api/routes/content.py
- [x] T057 [US3] Implement cache check logic in /api/content/personalize using PersonalizationCache.get_cached before generation
- [x] T058 [US3] Add chapter content hash calculation in personalize endpoint
- [x] T059 [US3] Add call to generate_personalized_content if cache miss
- [x] T060 [US3] Add cache storage logic using PersonalizationCache.set_cached after generation
- [x] T061 [US3] Add generation metadata tracking (model, tokens_used, generation_time_ms)
- [x] T062 [US3] Update GET /api/content/chapters/{chapter_id} to check content_mode preference and call personalize endpoint if mode is "personalized"

### Backend - Code Block Validation

- [x] T063 [P] [US3] Add regex validation function to verify code blocks preserved after personalization in content_generator.py
- [x] T064 [P] [US3] Add pre-generation code block count extraction
- [x] T065 [US3] Add post-generation code block count validation and error logging if mismatch

### Frontend - Personalized Content Display

- [x] T066 [P] [US3] Create PersonalizedView component in frontend/src/components/Content/PersonalizedView.tsx
- [x] T067 [P] [US3] Add loading indicator for content generation in PersonalizedView
- [x] T068 [P] [US3] Add error handling with fallback message in PersonalizedView
- [x] T069 [US3] Add API call to /api/content/personalize in PersonalizedView with chapter_id from URL
- [x] T070 [US3] Integrate PersonalizedView into chapter layout based on content_mode preference

### Frontend - Generation Progress

- [x] T071 [P] [US3] Add streaming response support (if OpenAI Agents SDK supports) in PersonalizedView
- [x] T072 [P] [US3] Add progress indicator showing estimated time during generation

**Checkpoint**: User Stories 1, 2, AND 3 all work independently - users can select personalized mode and get adapted content

---

## Phase 6: User Story 4 - Urdu Translation On-Demand (Priority: P4)

**Goal**: Translate chapter content to Urdu while preserving code blocks using OpenAI agent

**Independent Test**: Enable Urdu preference, request translation for a chapter, verify output is in Urdu with code blocks preserved in English

### Backend - Translation Agent

- [x] T073 [P] [US4] Create translation_agent with gpt-4o-mini model in backend/src/services/personalization/translator.py
- [x] T074 [P] [US4] Add Urdu translation instructions to agent (technical accuracy, terminology handling, placeholder preservation)
- [x] T075 [US4] Implement translate_to_urdu(chapter_content) async function with pre-processing logic
- [x] T076 [US4] Add regex code block extraction logic that replaces code with {{CODE_BLOCK_N}} placeholders
- [x] T077 [US4] Add Runner.run call for translation with placeholders
- [x] T078 [US4] Add post-processing logic to restore code blocks from placeholders
- [x] T079 [US4] Add code block preservation validation (count match check)

### Backend - Translation API with Caching

- [x] T080 [US4] Add POST /api/content/translate endpoint in backend/src/api/routes/content.py
- [x] T081 [US4] Add target_language validation (only "ur" supported) in translate endpoint
- [x] T082 [US4] Implement cache check logic using PersonalizationCache.get_cached with target_language parameter
- [x] T083 [US4] Add call to translate_to_urdu if cache miss
- [x] T084 [US4] Add cache storage with content_type="translated" and target_language="ur"
- [x] T085 [US4] Update GET /api/content/chapters/{chapter_id} to check urdu_translation_enabled preference and call translate endpoint if enabled

### Frontend - Translation Toggle

- [x] T086 [P] [US4] Create TranslationToggle component in frontend/src/components/Content/TranslationToggle.tsx
- [x] T087 [P] [US4] Add toggle button UI with language selection (English/Urdu)
- [x] T088 [US4] Add onClick handler that triggers translation API call
- [x] T089 [US4] Add loading indicator during translation
- [x] T090 [US4] Store translated content in component state and toggle between original/translated
- [x] T091 [US4] Integrate TranslationToggle into chapter layout when urdu_translation_enabled is true

**Checkpoint**: User Stories 1-4 all work independently - users can translate chapters to Urdu with code preservation

---

## Phase 7: User Story 5 - Preference Management (Priority: P5)

**Goal**: Allow users to view and update their preferences from a settings/profile page

**Independent Test**: Access preference management interface, update fields, verify changes persist and affect content display

### Frontend - Preference Management UI

- [x] T092 [P] [US5] Create PreferenceManagement component in frontend/src/components/Auth/PreferenceManagement.tsx
- [x] T093 [P] [US5] Add form pre-population logic that loads current preferences from PersonalizationContext
- [x] T094 [US5] Add form submission handler that calls updatePreferences from context
- [x] T095 [US5] Add success/error notifications after preference update
- [x] T096 [US5] Create /settings or /profile page in frontend/src/pages/ that includes PreferenceManagement component

### Frontend - Preference Update Integration

- [x] T097 [US5] Update PersonalizationContext to trigger content reload after preference update
- [x] T098 [US5] Add cache invalidation logic on frontend (force_regenerate flag) when user updates preferences
- [x] T099 [US5] Update ContentModeSwitch to reflect new preference immediately after change

**Checkpoint**: All user stories (1-5) are now independently functional - complete personalization system is operational

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, monitoring, and production readiness

### Error Handling & Fallbacks

- [ ] T100 [P] Add global error boundary in frontend for content generation failures
- [ ] T101 [P] Implement fallback to full content mode on personalization/translation errors
- [ ] T102 [P] Add user-friendly error messages for OpenAI API rate limits (429)
- [ ] T103 [P] Add user-friendly error messages for authentication failures (401)

### Logging & Monitoring

- [ ] T104 [P] Add structured logging for preference submissions using structlog
- [ ] T105 [P] Add structured logging for content generation requests with user_id, chapter_id, cache_hit status
- [ ] T106 [P] Add structured logging for translation requests
- [ ] T107 [P] Add cache hit rate metrics calculation
- [ ] T108 [P] Add OpenAI token usage tracking per request

### Performance Optimization

- [ ] T109 [P] Implement content chunking strategy for chapters exceeding 100k tokens
- [ ] T110 [P] Add in-memory caching for user preferences with 5-minute TTL
- [ ] T111 [P] Add request queuing for OpenAI API to handle rate limits gracefully
- [ ] T112 [P] Optimize database queries with EXPLAIN ANALYZE on cache lookup queries

### Security

- [ ] T113 [P] Verify OPENAI_API_KEY is never exposed in frontend or logs
- [ ] T114 [P] Add rate limiting to preference update endpoint (max 10 updates per hour per user)
- [ ] T115 [P] Add input sanitization for learning_goals text field to prevent XSS
- [ ] T116 [P] Verify session token validation on all authenticated endpoints

### Documentation

- [ ] T117 [P] Add API documentation using FastAPI automatic OpenAPI generation
- [ ] T118 [P] Update README.md with setup instructions for personalization feature
- [ ] T119 [P] Add inline code comments for complex logic in content_generator.py and translator.py
- [ ] T120 [P] Document cache invalidation strategy in code comments

### Deployment Preparation

- [ ] T121 Add GitHub Actions workflow for deploying frontend to gh-pages branch
- [ ] T122 Verify all environment variables are documented in backend/.env.example
- [ ] T123 Run Alembic migration on staging database
- [ ] T124 Test complete user flow on staging environment (signup → preferences → personalized content → translation)

---

## Dependencies & Parallel Execution

### Story Dependencies (Must Complete in Order)

```
Phase 1 (Setup) → Phase 2 (Foundational) → [User Stories can proceed in parallel]

User Story Dependencies:
- US1 (P1): No dependencies - can start after Phase 2
- US2 (P2): Depends on US1 for preference infrastructure
- US3 (P3): Depends on US1 (preferences) and US2 (content infrastructure)
- US4 (P4): Depends on US1 (preferences) and US2 (content infrastructure)
- US5 (P5): Depends on US1 (can start after US1 complete)
```

### Parallel Execution Opportunities

**After Phase 2 Foundation:**
- US1 backend (T016-T020) can run parallel with US1 frontend (T021-T036)
- US2 backend (T037-T043) can run parallel with US2 frontend (T044-T048)
- US3 agent work (T049-T055) can run parallel with caching work (T056-T062)
- US4 translation agent (T073-T079) can run parallel with translation API (T080-T085)

**Within Each Story:**
- Tasks marked [P] can execute in parallel (different files, no dependencies)
- Backend and frontend work for same story can overlap
- Caching, validation, and error handling can be done in parallel

**Phase 8 Polish:**
- All tasks in Phase 8 are parallelizable (T100-T124)

---

## MVP Scope Recommendation

**Minimum Viable Product**: User Story 1 (P1) - Preference Collection

**Why**: This delivers immediate value by:
- Establishing user context for future personalization
- Validating authentication integration
- Testing database preference storage
- Providing foundation for all other stories

**MVP Tasks**: T001-T036 (36 tasks, estimated 12-16 hours)

**Next Increment**: +US2 (Full Content Mode) for baseline content delivery

**Full Feature**: All 5 user stories (124 tasks, estimated 40-50 hours)

---

## Task Summary

| Phase | Task Range | Count | Estimated Hours |
|-------|------------|-------|-----------------|
| Phase 1: Setup | T001-T006 | 6 | 1-2h |
| Phase 2: Foundational | T007-T015 | 9 | 4-6h |
| Phase 3: US1 (MVP) | T016-T036 | 21 | 8-10h |
| Phase 4: US2 | T037-T048 | 12 | 4-6h |
| Phase 5: US3 | T049-T072 | 24 | 10-12h |
| Phase 6: US4 | T073-T091 | 19 | 8-10h |
| Phase 7: US5 | T092-T099 | 8 | 3-4h |
| Phase 8: Polish | T100-T124 | 25 | 6-8h |
| **TOTAL** | T001-T124 | **124** | **44-58h** |

**Parallel Opportunities**: 60+ tasks can run in parallel (48% of total)

**Independent Deliverables**: Each user story is independently testable and deployable

---

## Validation

✅ All tasks follow checklist format: `- [ ] [TaskID] [P?] [Story] Description with file path`
✅ Tasks organized by user story for independent implementation
✅ Each story has clear goal and independent test criteria
✅ Parallelizable tasks marked with [P]
✅ Story labels ([US1], [US2], etc.) included for story phases
✅ File paths specified for all implementation tasks
✅ Dependencies documented
✅ MVP scope clearly identified (US1)
✅ Total task count and estimates provided
