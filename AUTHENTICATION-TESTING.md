# Authentication Testing Guide

## ✅ **FIXED!** Authentication now works correctly

The "Authentication required" error during signup has been resolved. Here's how to test:

---

## Automated Test (CLI)

Run the automated test script:

```bash
cd backend
./test-questionnaire-auth.sh
```

**Expected Result:**
```
✅ SUCCESS: Questionnaire submitted successfully with cookie-based authentication!
```

---

## Browser Test (Manual)

### 1. Open the Application
Navigate to: **http://localhost:3000**

### 2. Sign Up
1. Click "Sign Up" or "Get Started"
2. Enter your details:
   - Email: `test@example.com` (use any test email)
   - Password: `testpass123` (minimum 8 characters)
   - Name: Your name
3. Click "Sign Up"

### 3. Complete Questionnaire
You'll be redirected to the background questionnaire. Fill in:
- ✅ Software background (select at least one option)
- ✅ Hardware background (select at least one option)
- ✅ Experience level (required)
- Optional: Learning goals, projects, etc.

### 4. Submit
Click **"Complete Setup"**

**Expected Result:**
- ✅ No "Authentication required" error
- ✅ Success screen appears: "🎉 Welcome! Your account has been created successfully"
- ✅ User profile saved successfully
- ✅ "Start Learning →" button displayed

### 5. Navigate to Book
Click **"Start Learning →"**

**Expected Result:**
- ✅ Redirected to the book introduction page
- ✅ User is authenticated and can explore content
- ✅ No "Page Not Found" errors

---

## Debugging in Browser

### Check Browser Console
Open Developer Tools (F12) → Console tab

**You should see:**
```
Submitting questionnaire to FastAPI...
FastAPI response status: 200
Success! Questionnaire saved: {...}
```

### Check Network Tab
Developer Tools (F12) → Network tab

**Look for the POST request to:**
`http://localhost:8000/api/auth/profile/background`

**Check:**
- ✅ Status: 200 OK
- ✅ Request Headers should include cookies
- ✅ Response should contain `user_id`

### Check Cookies
Developer Tools (F12) → Application tab → Cookies → `http://localhost:3000`

**You should see:**
- `better-auth.session_token` (httpOnly, value is signed)
- `better-auth.session_data` (contains cached session info)

---

## What Was Fixed

### Root Cause
Better Auth uses **signed cookies** for authentication. The cookie contains:
```
token.signature
```

The frontend was trying to send only the token, which failed validation.

### Solution
1. **Better Auth Configuration** - Set `sameSite: "lax"` to allow cross-port cookie sharing
2. **Frontend Simplification** - Use `credentials: 'include'` to let browser handle cookies automatically
3. **Backend** - Already correctly forwarding cookies to Better Auth

---

## Services Status

Check all services are running:

```bash
curl http://localhost:3001/health  # Better Auth
curl http://localhost:8000/health  # FastAPI
curl http://localhost:3000         # Frontend
```

---

## Troubleshooting

### Issue: "Authentication required" error

**Check:**
1. All services running (see above)
2. Browser console for error messages
3. Network tab shows cookies being sent
4. Backend logs: `tail -f /tmp/fastapi.log`

### Issue: Cookies not being sent

**Solution:**
- Clear browser cookies for localhost
- Sign up with a new email
- Make sure you're using `http://localhost:3000` (not 127.0.0.1)

### Issue: CORS errors

**Check:**
- FastAPI CORS configuration includes `localhost:3000`
- Better Auth trustedOrigins includes `localhost:3000`

---

## Files Modified

1. `backend/auth-service/src/auth.ts` - Cookie configuration
2. `frontend/src/components/Auth/UserBackgroundQuestionnaire.tsx` - Simplified auth
3. `backend/src/core/auth.py` - Debug logging
4. `backend/test-questionnaire-auth.sh` - Updated test script

---

## Next Steps

After confirming the signup works:

1. ✅ Test other authenticated endpoints (profile updates, etc.)
2. ✅ Test the RAG chatbot (should also work with same cookie auth)
3. ✅ Deploy to production with updated Better Auth config

---

## Documentation

- [AUTH-FIX-SUMMARY.md](AUTH-FIX-SUMMARY.md) - Detailed technical summary
- [AUTH-DEBUGGING.md](AUTH-DEBUGGING.md) - Previous debugging notes
- [TESTING-GUIDE.md](TESTING-GUIDE.md) - Full testing guide

---

**Status:** ✅ RESOLVED  
**Date:** 2025-11-30

