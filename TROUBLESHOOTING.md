# Troubleshooting Guide

## Common Issues and Solutions

### Issue: "Failed to fetch" Error

**Possible Causes:**
1. **Backend not running** - FastAPI backend (port 8000) must be running
2. **CORS error** - Browser blocking cross-origin requests
3. **Network error** - Service unreachable

**Solutions:**

1. **Check if FastAPI is running:**
   ```bash
   curl http://localhost:8000/health
   ```
   If not running, start it:
   ```bash
   cd backend
   uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
   ```

2. **Check browser console** (F12 → Console tab):
   - Look for CORS errors
   - Look for network errors
   - Check if request is being blocked

3. **Verify CORS configuration:**
   - FastAPI must allow `http://localhost:3000` in `allow_origins`
   - `allow_credentials: True` must be set

### Issue: "Not authenticated. Please sign in again"

**Possible Causes:**
1. **BETTER_AUTH_SERVICE_URL not configured** - FastAPI can't validate sessions
2. **Session cookie not being sent** - Browser not sending cookies
3. **Cookie parsing issue** - FastAPI not reading cookies correctly

**Solutions:**

1. **Check backend/.env has BETTER_AUTH_SERVICE_URL:**
   ```bash
   grep BETTER_AUTH_SERVICE_URL backend/.env
   ```
   Should show: `BETTER_AUTH_SERVICE_URL="http://localhost:3001"`

2. **Restart FastAPI** after adding environment variable:
   ```bash
   # Stop FastAPI (Ctrl+C)
   cd backend
   uv run uvicorn src.main:app --reload
   ```

3. **Check browser cookies:**
   - Open DevTools (F12) → Application → Cookies
   - Look for `better-auth.session_token`
   - Should be present after signup

4. **Verify Better Auth is running:**
   ```bash
   curl http://localhost:3001/health
   ```

### Issue: CORS Errors in Browser Console

**Error:** `Access to fetch at 'http://localhost:8000/...' from origin 'http://localhost:3000' has been blocked by CORS policy`

**Solution:**
1. Check FastAPI CORS configuration in `backend/src/main.py`
2. Ensure `allow_origins` includes `"http://localhost:3000"`
3. Ensure `allow_credentials: True`
4. Restart FastAPI

### Issue: 404 Error on /signup

**Solution:**
Use the full URL with baseUrl:
- ✅ Correct: http://localhost:3000/physical-ai-humanoid-robotics-book/signup
- ❌ Wrong: http://localhost:3000/signup

### Issue: Session Token Not Found

**Solution:**
1. Make sure you've completed signup first
2. Check browser cookies (DevTools → Application → Cookies)
3. Verify Better Auth service is running
4. Try signing in again if session expired

## Quick Diagnostic Commands

```bash
# Check all services
curl http://localhost:3001/health  # Better Auth
curl http://localhost:8000/health  # FastAPI
curl http://localhost:3000         # Frontend

# Check environment variables
cd backend
uv run python -c "from src.core.config import settings; print('BETTER_AUTH_SERVICE_URL:', settings.BETTER_AUTH_SERVICE_URL)"

# Check FastAPI logs
tail -f /tmp/fastapi.log
```

## Restart All Services

```bash
# Terminal 1: Better Auth
cd backend/auth-service
npm run dev

# Terminal 2: FastAPI Backend
cd backend
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Terminal 3: Frontend
cd frontend
npm start
```
