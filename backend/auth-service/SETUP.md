# Better Auth Service Setup Guide

## Quick Start

1. **Copy environment file**:
   ```bash
   cp .env.example .env
   ```

2. **Update `.env` with your values**:
   - `DATABASE_URL`: Copy from `backend/.env.example` (NEON_DATABASE_URL)
   - `BETTER_AUTH_SECRET`: Use the generated secret from `.env.example` or generate a new one:
     ```bash
     node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
     ```
   - `BETTER_AUTH_URL`: `http://localhost:3001` (development) or your production URL (e.g., `https://your-auth-service.onrender.com`)
   - `BETTER_AUTH_TRUSTED_ORIGINS`: Comma-separated list of frontend domains
     - Development: `http://localhost:3000`
     - Production: `https://billy-pk.github.io` (your GitHub Pages domain)

3. **Install dependencies** (if not already done):
   ```bash
   npm install
   ```

4. **Start the service**:
   ```bash
   npm run dev
   ```

5. **Verify it's running**:
   ```bash
   curl http://localhost:3001/health
   ```

## Testing

Run the test script (requires server to be running):
```bash
npm test
```

Or test manually:
```bash
# Health check
curl http://localhost:3001/health

# Signup
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123","name":"Test User"}'

# Signin
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123"}'
```

## Database Tables

Better Auth will automatically create these tables on first run:
- `user` - User accounts
- `session` - Active sessions  
- `account` - OAuth accounts (if OAuth is configured)
- `verification` - Email verification tokens (if email verification is enabled)

**Important**: These tables are created automatically by Better Auth. Do NOT create them manually via Alembic.

## Production Deployment

1. Set `BETTER_AUTH_URL` to your production domain
2. Set `BETTER_AUTH_TRUSTED_ORIGINS` to your frontend domain(s)
3. Use a strong `BETTER_AUTH_SECRET` (at least 32 characters)
4. Build the service:
   ```bash
   npm run build
   ```
5. Run the production server:
   ```bash
   npm start
   ```

## Integration with FastAPI

The FastAPI backend can validate sessions by:
1. Reading the session cookie/token from requests
2. Querying the `session` table in the shared Neon Postgres database
3. Checking if the session exists and hasn't expired

See `backend/src/core/auth.py` (to be created) for session validation utilities.
