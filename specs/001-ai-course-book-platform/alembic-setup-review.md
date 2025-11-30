# Alembic Setup Review & Authentication Migration Plan

**Date**: 2025-11-30  
**Reviewer**: AI Assistant  
**Status**: Setup Reviewed, Migration Plan Ready

---

## Current Alembic Setup Analysis

### ✅ Existing Configuration

**1. Alembic Configuration (`alembic.ini`)**:
- Script location: `alembic/`
- Database URL: Configured in `env.py` (not in ini file)
- Uses standard Alembic configuration

**2. Alembic Environment (`alembic/env.py`)**:
- ✅ Imports `settings.NEON_DATABASE_URL` from `src.core.config`
- ✅ Sets database URL: `config.set_main_option("sqlalchemy.url", settings.NEON_DATABASE_URL)`
- ⚠️ `target_metadata = None` - **Autogenerate is disabled**
- ✅ Uses manual migrations (op.create_table approach)
- ✅ Properly configured for PostgreSQL

**3. Current Migration Status**:
- **Current Revision**: `e58d1bdc4ddb` (head)
- **Migration**: `initial_schema_chat_tables`
- **Tables Created**:
  - `chat_sessions` (session_id UUID, created_at, updated_at)
  - `chat_messages` (message_id UUID, session_id FK, role, content, citations JSON, etc.)
  - `api_metrics` (metric_id, endpoint, method, status_code, response_time_ms, created_at)

**4. Migration Style**:
- Uses manual `op.create_table()` approach
- No SQLAlchemy models/base for autogenerate
- Proper indexes and foreign keys defined
- Uses UUID for primary keys (chat_sessions, chat_messages)

---

## Better Auth Schema Requirements

### Tables Created by Better Auth

Better Auth will automatically create these tables when configured:

**1. `user` table**:
- `id` (text/varchar, primary key) - **Note: Text ID, not UUID**
- `email` (text, unique)
- `name` (text, nullable)
- `emailVerified` (boolean)
- `image` (text, nullable)
- `createdAt` (timestamp)
- `updatedAt` (timestamp)
- Additional fields based on plugins

**2. `session` table**:
- `id` (text/varchar, primary key)
- `expiresAt` (timestamp)
- `token` (text, unique)
- `createdAt` (timestamp)
- `updatedAt` (timestamp)
- `ipAddress` (text, nullable)
- `userAgent` (text, nullable)
- `userId` (text, foreign key to user.id, cascade delete)
- Index on `userId`

**3. Additional Better Auth Tables** (may be created):
- `account` (for OAuth providers, credentials)
- `verification` (for email verification)
- `oneTimePassword` (for 2FA, if enabled)
- Plugin-specific tables

---

## Migration Strategy

### Important Considerations

1. **ID Type Mismatch**:
   - Better Auth uses **text IDs** (not UUIDs)
   - Our `chat_sessions` uses **UUID**
   - Need to decide: Keep UUID for chat_sessions or migrate to text IDs?

2. **Better Auth Schema Generation**:
   - Better Auth CLI can generate schema migrations
   - OR Better Auth adapter creates tables automatically on first run
   - We need to coordinate: Better Auth creates auth tables, Alembic creates profile tables

3. **Migration Order**:
   - Better Auth tables should be created FIRST (or Better Auth handles it)
   - Then create FastAPI-managed tables that reference Better Auth tables

### Recommended Approach

**Option A: Let Better Auth Create Its Tables First** (Recommended)
1. Set up Better Auth service
2. Run Better Auth initialization (creates user, session tables)
3. Create Alembic migration for FastAPI tables that reference Better Auth tables
4. Use text type for foreign keys to user.id (since Better Auth uses text IDs)

**Option B: Manual Migration for Better Auth Tables**
1. Create Alembic migration for Better Auth tables (user, session)
2. Create Alembic migration for FastAPI tables (user_profiles, etc.)
3. Configure Better Auth to use existing tables

**Decision**: Use **Option A** - Let Better Auth manage its own schema, then create complementary migrations.

---

## Required Migrations

### Migration 1: Update chat_sessions for User Linking

**File**: `alembic/versions/XXXX_add_user_id_to_chat_sessions.py`

**Purpose**: Add `user_id` column to link chat sessions to authenticated users

**Changes**:
- Add `user_id` column (text, nullable, foreign key to user.id)
- Add index on `user_id`
- Make it nullable (anonymous sessions don't have user_id)

**Note**: Better Auth's `user` table must exist first (created by Better Auth)

### Migration 2: Create user_profiles Table

**File**: `alembic/versions/XXXX_create_user_profiles.py`

**Purpose**: Create table for user profile and background questionnaire data

**Schema**:
```python
op.create_table(
    'user_profiles',
    sa.Column('user_id', sa.Text(), nullable=False),  # Text to match Better Auth user.id
    sa.Column('software_background', sa.JSON(), nullable=True),
    sa.Column('hardware_background', sa.JSON(), nullable=True),
    sa.Column('experience_level', sa.String(20), nullable=True),  # beginner/intermediate/advanced
    sa.Column('learning_goals', sa.Text(), nullable=True),
    sa.Column('has_robotics_projects', sa.Boolean(), nullable=True),
    sa.Column('robotics_projects_description', sa.Text(), nullable=True),
    sa.Column('programming_years', sa.Integer(), nullable=True),
    sa.Column('learning_style', sa.String(20), nullable=True),
    sa.Column('questionnaire_completed', sa.Boolean(), server_default='false', nullable=False),
    sa.Column('questionnaire_completed_at', sa.TIMESTAMP(timezone=True), nullable=True),
    sa.Column('preferences', sa.JSON(), nullable=True),
    sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
    sa.Column('updated_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
    sa.PrimaryKeyConstraint('user_id'),
    sa.ForeignKeyConstraint(['user_id'], ['user.id'], ondelete='CASCADE')
)
op.create_index('ix_user_profiles_user_id', 'user_profiles', ['user_id'])
```

### Migration 3: Create user_background_questionnaire Table (Optional)

**File**: `alembic/versions/XXXX_create_user_background_questionnaire.py`

**Purpose**: Track individual questionnaire answers for analytics

**Schema**:
```python
op.create_table(
    'user_background_questionnaire',
    sa.Column('id', sa.UUID(), nullable=False),
    sa.Column('user_id', sa.Text(), nullable=False),
    sa.Column('question_id', sa.String(100), nullable=False),
    sa.Column('answer', sa.JSON(), nullable=True),
    sa.Column('answered_at', sa.TIMESTAMP(timezone=True), server_default=sa.text('now()'), nullable=False),
    sa.PrimaryKeyConstraint('id'),
    sa.ForeignKeyConstraint(['user_id'], ['user.id'], ondelete='CASCADE')
)
op.create_index('ix_user_background_questionnaire_user_id', 'user_background_questionnaire', ['user_id'])
op.create_index('ix_user_background_questionnaire_question_id', 'user_background_questionnaire', ['question_id'])
```

---

## Migration Dependencies

### Dependency Chain

```
Base (empty)
  └─ e58d1bdc4ddb (initial_schema_chat_tables) [EXISTS]
      └─ [Better Auth creates: user, session tables] [EXTERNAL]
          └─ XXXX_add_user_id_to_chat_sessions [NEW - depends on Better Auth user table]
          └─ XXXX_create_user_profiles [NEW - depends on Better Auth user table]
          └─ XXXX_create_user_background_questionnaire [NEW - depends on Better Auth user table]
```

### Important Notes

1. **Better Auth Tables Must Exist First**: Migrations that reference `user.id` require Better Auth tables to exist
2. **ID Type Consistency**: Use `sa.Text()` for foreign keys to Better Auth's `user.id` (not UUID)
3. **Cascade Deletes**: Foreign keys should cascade delete when user is deleted
4. **Nullable user_id**: `chat_sessions.user_id` should be nullable to support anonymous sessions

---

## Updated Task Plan

### Phase 2: Backend Database Setup (Revised)

**Prerequisites**: Better Auth service must be set up and initialized first (creates user/session tables)

- [ ] AUTH-006: Wait for Better Auth to create `user` and `session` tables
- [ ] AUTH-007: Create Alembic migration to add `user_id` to `chat_sessions` table
- [ ] AUTH-008: Create Alembic migration for `user_profiles` table
- [ ] AUTH-009: Create Alembic migration for `user_background_questionnaire` table (optional)
- [ ] AUTH-010: Verify foreign key constraints and indexes
- [ ] AUTH-011: Run migrations and verify schema creation

**Note**: Better Auth will create its own tables. We only create complementary tables.

---

## Action Items

1. ✅ **Review Complete**: Alembic setup is properly configured
2. ⏳ **Next Step**: Set up Better Auth service first (Phase 3)
3. ⏳ **Then**: Create Alembic migrations for FastAPI-managed tables (Phase 2, after Better Auth)

---

## Key Findings

1. ✅ Alembic is properly configured with Neon Postgres connection
2. ✅ Current migration uses manual approach (no autogenerate) - consistent style
3. ⚠️ Need to coordinate Better Auth schema creation with Alembic migrations
4. ⚠️ ID type mismatch: Better Auth uses text IDs, chat_sessions uses UUID
5. ✅ Foreign keys should use text type to reference Better Auth's user.id

---

**Status**: Setup reviewed and understood. Ready to proceed with Better Auth setup, then create complementary migrations.
