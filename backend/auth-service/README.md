# Better Auth Service

Authentication service for AI-Course-Book Platform using Better Auth.

## Setup

1. **Install dependencies**:
   ```bash
   npm install
   ```

2. **Configure environment variables**:
   ```bash
   cp .env.example .env
   # Edit .env with your actual values:
   # - DATABASE_URL: Copy from backend/.env (NEON_DATABASE_URL)
   # - BETTER_AUTH_SECRET: Use the generated secret or create a new one
   ```

3. **Run database migrations** (creates Better Auth tables):
   ```bash
   npm run migrate
   # Or manually: echo "y" | npx @better-auth/cli@latest migrate
   ```

3. **Required environment variables**:
   - `DATABASE_URL`: Neon Postgres connection string
   - `BETTER_AUTH_SECRET`: Secret key for session encryption (min 32 characters)
   - `BETTER_AUTH_URL`: Public URL of this service (e.g., `http://localhost:3001`)
   - `BETTER_AUTH_TRUSTED_ORIGINS`: Comma-separated list of allowed origins
   - `PORT`: Port to run the service on (default: 3001)

## Running

**Development**:
```bash
npm run dev
```

**Production**:
```bash
npm run build
npm start
```

## Endpoints

- `GET /health` - Health check
- `POST /api/auth/sign-up/email` - User signup
- `POST /api/auth/sign-in/email` - User signin
- `POST /api/auth/sign-out` - User signout
- `GET /api/auth/session` - Get current session
- All other Better Auth endpoints under `/api/auth/*`

## Database

Better Auth automatically creates the following tables in the Neon Postgres database:
- `user` - User accounts
- `session` - Active sessions
- `account` - OAuth accounts (if OAuth providers are configured)
- `verification` - Email verification tokens (if email verification is enabled)

## Integration

The frontend should connect to this service using the Better Auth client SDK:
```typescript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.NEXT_PUBLIC_BETTER_AUTH_URL || "http://localhost:3001",
});
```

The FastAPI backend can validate sessions by querying the `session` table directly.
