# Sign-In 401 Unauthorized Fix - Version 2

## Issue
After implementing token-based authentication, FastAPI was still returning 401 when validating sessions via Better Auth's API.

## Root Cause
Better Auth's `/api/auth/get-session` endpoint returns `null` when called from server-to-server requests (FastAPI → Better Auth), likely due to origin/header validation. The endpoint works from browser requests but not from backend services.

## Solution
Since both FastAPI and Better Auth share the same PostgreSQL database, we now validate sessions by querying the `session` table directly instead of calling Better Auth's API.

### Changes Made

**`backend/src/core/auth.py`**:
- Updated `get_user_from_session()` to query the session table directly
- Added fallback to Better Auth API if database query fails
- Query checks:
  - Session token matches
  - Session hasn't expired (`expiresAt > now()`)
  - Joins with `user` table to get user information

### How It Works

1. Frontend sends token in `Authorization: Bearer <token>` header
2. FastAPI extracts token from header
3. FastAPI queries `session` table directly:
   ```sql
   SELECT s."userId", u.id, u.email, u.name, ...
   FROM session s
   INNER JOIN "user" u ON u.id = s."userId"
   WHERE s.token = :token AND s."expiresAt" > :now
   ```
4. If session is valid, return user information
5. If database query fails, fallback to Better Auth API

### Benefits

- ✅ More reliable (no dependency on Better Auth API availability)
- ✅ Faster (direct database query vs HTTP request)
- ✅ Works for server-to-server requests
- ✅ Fallback to API if database query fails

## Testing

After this fix, the preferences endpoint should work:
- ✅ Token is extracted from Authorization header
- ✅ Session is validated via database query
- ✅ User information is returned
- ✅ Preferences endpoint returns 200 OK

