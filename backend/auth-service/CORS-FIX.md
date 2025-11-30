# CORS Fix Applied

## Issue
"Failed to fetch" error when submitting signup form from frontend.

## Root Cause
Express server was missing CORS middleware, causing browser to block cross-origin requests.

## Fix Applied
1. ✅ Installed `cors` package
2. ✅ Added CORS middleware to Express server
3. ✅ Configured CORS to allow requests from trusted origins
4. ✅ Enabled credentials (cookies) support

## Changes Made

### `src/server.ts`
- Added `cors` import
- Added CORS middleware before Better Auth handler
- Configured to use `trustedOrigins` from config
- Enabled credentials support

## Next Steps

**Restart the Better Auth server** to apply changes:

```bash
cd backend/auth-service
npm run dev
```

After restart, the signup form should work correctly.

## Verification

After restart, test:
1. Open: http://localhost:3000/physical-ai-humanoid-robotics-book/signup
2. Fill signup form
3. Submit
4. Should work without "failed to fetch" error

## CORS Configuration

The CORS middleware is configured to:
- Allow origins from `BETTER_AUTH_TRUSTED_ORIGINS` env var
- Default: `http://localhost:3000` and `https://billy-pk.github.io`
- Allow methods: GET, POST, PUT, DELETE, OPTIONS
- Enable credentials (cookies)
- Allow headers: Content-Type, Authorization, Cookie
