# Authentication Debugging Summary

## Problem

When users sign up and submit the questionnaire, they get an "Authentication required" error.

## Root Cause

**Cross-Origin Cookie Issue:**

1. Better Auth (localhost:3001) sets an httpOnly cookie: `better-auth.session_token=TOKEN.SIGNATURE`
2. The browser **does not send** cookies from localhost:3001 to localhost:8000 (FastAPI)
3. The session token in the JSON response is **unsigned** and won't validate with Better Auth
4. The **signed** cookie (with `.SIGNATURE` suffix) is needed for validation, but it's httpOnly and can't be read by JavaScript

## Test Results

```bash
# ✅ Signup works and returns unsigned token
Token from JSON: pBZ4XPpn6bo33BbLNKHZNe15vvEWObzx

# ✅ Better Auth validates with signed cookie
Cookie: pBZ4XPpn6bo33BbLNKHZNe15vvEWObzx.Dv6G+XOfQwqaI9wOjV+2G+UHDvUqFFzLcZJbuaVXzCE=
Result: Session valid with user info

# ❌ FastAPI rejects unsigned token
Authorization: Bearer pBZ4XPpn6bo33BbLNKHZNe15vvEWObzx
Result: 401 Authentication required
```

## Solutions Implemented

### 1. Backend Changes

#### New Session Validation Proxy (`backend/src/api/routes/session_proxy.py`)
- Endpoint: `POST /api/session/validate`
- Accepts a token and validates it with Better Auth
- Returns user information if valid

#### Updated Auth Routes
- Added `X-Session-Token` header support
- Enhanced logging for debugging
- Supports multiple auth methods (Bearer, Cookie, X-Session-Token)

### 2. Frontend Changes

#### Updated Questionnaire Component
- Attempts to get session from Better Auth
- Tries to read cookie from `document.cookie` (won't work if httpOnly)
- Falls back to Bearer token if cookie not available
- Enhanced error logging in console

## Current Status

🔴 **Better Auth service is DOWN** - needs manual restart

## Next Steps

### Immediate Actions (User)

1. **Restart Better Auth service:**
   ```bash
   cd backend/auth-service
   # Press Ctrl+C in the terminal where it's running
   npm run dev
   ```

2. **Verify all services are running:**
   ```bash
   curl http://localhost:3001/health  # Better Auth
   curl http://localhost:8000/health  # FastAPI
   curl http://localhost:3000         # Frontend
   ```

### Solution Options

#### Option A: Temporary Development Fix (Recommended)
Since we can't easily share httpOnly cookies across ports, we'll modify the signup flow to:

1. After successful signup, make a request to Better Auth to get the signed cookie value from response headers
2. Store it in sessionStorage (not localStorage for security)
3. Send it to FastAPI in the `X-Session-Token` header
4. FastAPI validates it with Better Auth

#### Option B: Production-Ready Solution
1. Run all services behind a reverse proxy (Nginx/Caddy) on the same domain
2. Configure proxy to route:
   - `/` → Frontend (Docusaurus)
   - `/api/auth/*` → Better Auth
   - `/api/*` → FastAPI
3. Cookies work seamlessly across all endpoints

#### Option C: Shared Session Store
1. Configure Better Auth to store sessions in PostgreSQL
2. FastAPI reads sessions directly from the database
3. No need for cross-service HTTP calls

## Files Modified

### Backend
- `backend/src/core/auth.py` - Added debug logging
- `backend/src/api/routes/auth.py` - Added X-Session-Token support
- `backend/src/api/routes/session_proxy.py` - NEW: Session validation proxy
- `backend/src/main.py` - Registered session_proxy router
- `backend/auth-service/src/auth.ts` - Attempted cookie config (reverted)

### Frontend
- `frontend/src/components/Auth/UserBackgroundQuestionnaire.tsx` - Enhanced auth flow

### Testing
- `backend/test-questionnaire-auth.sh` - Test script for auth flow
- `backend/test-token-validation.py` - Token validation tests
- `backend/test-fresh-session.sh` - Fresh session creation tests

## Testing Commands

```bash
# Test signup and questionnaire flow
cd backend
./test-questionnaire-auth.sh

# Test session validation
./test-fresh-session.sh

# Manual curl test
EMAIL="test-$(date +%s)@example.com"
curl -c cookies.txt -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d "{\"email\":\"$EMAIL\",\"password\":\"test123\",\"name\":\"Test\"}"

# Get signed cookie
grep better-auth.session_token cookies.txt

# Test with FastAPI
curl -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -H "X-Session-Token: SIGNED_COOKIE_HERE" \
  -d '{"software_background":["Python"],"hardware_background":["Arduino"],"experience_level":"beginner"}'
```

## Related Documentation

- [Better Auth Documentation](https://www.better-auth.com/)
- [FastAPI CORS](https://fastapi.tiangolo.com/tutorial/cors/)
- [MDN - HTTP Cookies](https://developer.mozilla.org/en-US/docs/Web/HTTP/Cookies)
- [Same-Origin Policy](https://developer.mozilla.org/en-US/docs/Web/Security/Same-origin_policy)

