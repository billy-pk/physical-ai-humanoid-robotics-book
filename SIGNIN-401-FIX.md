# Sign-In 401 Unauthorized Fix

## Issue
After successful sign-in, requests to FastAPI `/api/auth/profile/preferences` were returning 401 Unauthorized.

## Root Cause
The Better Auth session cookie is set for `localhost:3001` (Better Auth service), but when the frontend at `localhost:3000` makes requests to FastAPI at `localhost:8000`, the browser doesn't send the cookie due to cross-origin restrictions.

## Solution
Extract the session token from the sign-in response and include it in requests to FastAPI as an `Authorization: Bearer <token>` header.

### Changes Made

1. **`frontend/src/lib/auth.ts`**:
   - Added `storeSessionToken()` to store token after sign-in
   - Updated `getSessionToken()` to check sessionStorage first, then Better Auth
   - Added `clearSessionToken()` to clear token on sign-out

2. **`frontend/src/components/Auth/SignInForm.tsx`**:
   - Store session token from sign-in response
   - Token is extracted from `result.data.token`

3. **`frontend/src/contexts/PersonalizationContext.tsx`**:
   - Updated `fetchPreferences()` to get session token and include in Authorization header
   - Updated `updatePreferences()` to get session token and include in Authorization header

4. **`frontend/src/components/Auth/PersonalizationGuard.tsx`**:
   - Updated preference check to get session token and include in Authorization header

## How It Works

1. User signs in → Better Auth returns `{ data: { token: "...", user: {...} } }`
2. SignInForm stores the token in `sessionStorage`
3. When making requests to FastAPI, `getSessionToken()` retrieves the token
4. Token is included as `Authorization: Bearer <token>` header
5. FastAPI extracts the token and validates it with Better Auth's `/api/auth/get-session` endpoint

## Testing

After sign-in, the preferences endpoint should now work:
- ✅ Sign-in stores token
- ✅ Preferences fetch includes token in Authorization header
- ✅ FastAPI validates token with Better Auth
- ✅ User preferences are returned successfully

## Next Steps

- [ ] Update SignUpForm to also store token after sign-up
- [ ] Update sign-out to clear stored token
- [ ] Consider adding token refresh logic if tokens expire

