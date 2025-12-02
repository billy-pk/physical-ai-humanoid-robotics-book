# Better Auth Email Validation Debug

## Issue Summary

Better Auth React client is returning `INVALID_EMAIL` errors for valid email addresses that work fine via curl.

## Key Findings

### Version Information
- **Frontend**: `better-auth@1.4.3`
- **Backend**: `better-auth@1.4.3` (installed, package.json shows 1.3.10)
- **Status**: Versions match (both 1.4.3)

### Test Results

#### ✅ curl Request - WORKS
```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"testuser1764651500@example.com","password":"testpass123","name":"Test User"}'

# Response: 200 OK - User created successfully
```

#### ❌ Browser Request - FAILS
- **Error**: `{"code":"INVALID_EMAIL","message":"Invalid email"}`
- **Status**: 400 Bad Request
- **Same email format** works via curl
- **Same password** works via curl

## Debugging Steps Taken

### 1. Enhanced Error Logging
- Added `JSON.stringify()` for better error visibility in console
- Added detailed logging of request data before sending

### 2. Client Configuration
- Added `fetchOptions: { credentials: 'include' }` to ensure cookies are sent
- Verified `baseURL` is correctly set to `http://localhost:3001`

### 3. Server Configuration
- Verified CORS is configured correctly
- Verified cookie settings (`sameSite: "lax"`)
- Verified email/password authentication is enabled

### 4. Request Comparison
- curl requests work perfectly
- Browser requests fail with INVALID_EMAIL

## Hypothesis

The Better Auth React client may be:
1. Performing client-side email validation that's stricter than server-side
2. Modifying the email format before sending
3. Having a bug in email validation logic

## Next Steps

1. **Capture Exact Request**: Add network request interception to see exact payload
2. **Check Better Auth Source**: Look at client-side validation code
3. **Test with Different Email Formats**: Try various email formats to identify pattern
4. **Check Browser Console**: Monitor exact error response from server

## Files Modified

1. `frontend/src/lib/auth.ts` - Added `fetchOptions` with credentials
2. `frontend/src/components/Auth/SignUpForm.tsx` - Added detailed request logging

## Testing Commands

### Test via curl (works):
```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"testuser@example.com","password":"testpass123","name":"Test User"}'
```

### Check server logs:
```bash
tail -f /tmp/better-auth.log
```

## Configuration Files

### Frontend Auth Client (`frontend/src/lib/auth.ts`):
```typescript
export const authClient = createAuthClient({
  baseURL: BETTER_AUTH_URL,
  fetchOptions: {
    credentials: 'include',
  },
});
```

### Backend Auth Config (`backend/auth-service/src/auth.ts`):
- Email/password enabled
- No email verification required
- Min password length: 8
- Auto sign-in after signup: true

