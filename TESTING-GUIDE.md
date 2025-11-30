# Authentication System Testing Guide

**Date**: 2025-11-30  
**Status**: Complete Testing Guide

---

## Prerequisites

Before testing, ensure all services are running:

1. **Better Auth Service** (Port 3001)
2. **FastAPI Backend** (Port 8000)
3. **Frontend** (Port 3000)
4. **Database** (Neon Postgres - should be accessible)

---

## Quick Start Testing

### 1. Start All Services

#### Terminal 1: Better Auth Service
```bash
cd backend/auth-service
npm run dev
```

**Expected Output**:
```
✅ Better Auth service running on http://localhost:3001
📡 Auth endpoints available at http://localhost:3001/api/auth/*
💚 Health check: http://localhost:3001/health
```

#### Terminal 2: FastAPI Backend
```bash
cd backend
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete.
```

#### Terminal 3: Frontend
```bash
cd frontend
npm start
```

**Expected Output**:
```
Docusaurus v3.9.2
Local: http://localhost:3000
```

---

## Testing Scenarios

### Test 1: Health Checks

#### 1.1 Better Auth Service Health
```bash
curl http://localhost:3001/health
```

**Expected Response**:
```json
{"status":"ok","service":"better-auth"}
```

#### 1.2 FastAPI Backend Health
```bash
curl http://localhost:8000/health
```

**Expected Response**:
```json
{"status":"ok","message":"Service is healthy"}
```

---

### Test 2: User Signup Flow

#### 2.1 Manual Browser Testing

1. **Open Frontend**: Navigate to `http://localhost:3000/signup`

2. **Fill Signup Form**:
   - Name: `Test User`
   - Email: `test@example.com`
   - Password: `testpass123` (min 8 chars)

3. **Submit Form**: Click "Sign Up"

4. **Expected Behavior**:
   - Form submits successfully
   - Progress indicator moves to Step 2 (Questionnaire)
   - Questionnaire form appears

#### 2.2 API Testing - Signup

```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "testpass123",
    "name": "Test User"
  }'
```

**Expected Response**:
```json
{
  "token": "...",
  "user": {
    "id": "...",
    "email": "test@example.com",
    "name": "Test User",
    "emailVerified": false,
    "createdAt": "...",
    "updatedAt": "..."
  }
}
```

**Save the token** for subsequent requests:
```bash
export AUTH_TOKEN="<token_from_response>"
```

---

### Test 3: Questionnaire Submission

#### 3.1 Manual Browser Testing

1. **After Signup**: You should see the questionnaire form

2. **Fill Questionnaire**:
   - **Software Background**: Select "Python", "JavaScript"
   - **Hardware Background**: Select "Arduino", "Raspberry Pi"
   - **Experience Level**: Select "intermediate"
   - **Learning Goals**: Enter "Build a humanoid robot"
   - **Programming Years**: Enter `5`
   - **Has Robotics Projects**: Check the box
   - **Robotics Projects Description**: Enter "Built a line-following robot"
   - **Learning Style**: Select "hands-on"

3. **Submit**: Click "Save & Continue"

4. **Expected Behavior**:
   - Form validates successfully
   - Progress indicator moves to Step 3 (Complete)
   - Success message appears
   - Redirects to home page after 2 seconds

#### 3.2 API Testing - Questionnaire Submission

**Get Session Cookie First**:
```bash
# Sign in to get session cookie
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -c cookies.txt \
  -d '{
    "email": "test@example.com",
    "password": "testpass123"
  }'
```

**Submit Questionnaire**:
```bash
curl -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -b cookies.txt \
  -d '{
    "software_background": ["Python", "JavaScript"],
    "hardware_background": ["Arduino", "Raspberry Pi"],
    "experience_level": "intermediate",
    "learning_goals": "Build a humanoid robot",
    "has_robotics_projects": true,
    "robotics_projects_description": "Built a line-following robot",
    "programming_years": 5,
    "learning_style": "hands-on"
  }'
```

**Expected Response**:
```json
{
  "user_id": "...",
  "software_background": ["Python", "JavaScript"],
  "hardware_background": ["Arduino", "Raspberry Pi"],
  "experience_level": "intermediate",
  "learning_goals": "Build a humanoid robot",
  "has_robotics_projects": true,
  "robotics_projects_description": "Built a line-following robot",
  "programming_years": 5,
  "learning_style": "hands-on",
  "questionnaire_completed": true,
  "questionnaire_completed_at": "...",
  "created_at": "...",
  "updated_at": "..."
}
```

---

### Test 4: Get User Profile

#### 4.1 API Testing

```bash
curl http://localhost:8000/api/auth/profile \
  -b cookies.txt
```

**Expected Response**: User profile with all questionnaire data

---

### Test 5: Sign In Flow

#### 5.1 Manual Browser Testing

1. **Navigate**: Go to `http://localhost:3000/signin`

2. **Fill Sign In Form**:
   - Email: `test@example.com`
   - Password: `testpass123`

3. **Submit**: Click "Sign In"

4. **Expected Behavior**:
   - Successful sign in
   - Redirects to home page
   - User session established

#### 5.2 API Testing

```bash
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -c cookies.txt \
  -d '{
    "email": "test@example.com",
    "password": "testpass123"
  }'
```

---

### Test 6: Protected Routes

#### 6.1 Test ProtectedRoute Component

Create a test page that uses ProtectedRoute:

```typescript
// frontend/src/pages/test-protected.tsx
import React from "react";
import Layout from "@theme/Layout";
import ProtectedRoute from "../components/Auth/ProtectedRoute";

export default function TestProtectedPage() {
  return (
    <Layout title="Protected Page">
      <ProtectedRoute>
        <div>
          <h1>This is a protected page</h1>
          <p>Only authenticated users can see this.</p>
        </div>
      </ProtectedRoute>
    </Layout>
  );
}
```

**Testing**:
1. Navigate to `/test-protected` while not signed in
2. **Expected**: Sign-in form appears
3. Sign in
4. **Expected**: Protected content appears

---

### Test 7: Questionnaire Validation

#### 7.1 Test Required Fields

1. **Try submitting empty form**
   - **Expected**: Error messages appear
   - Software background error
   - Hardware background error
   - Experience level error

2. **Fill only some fields**
   - Select software but not hardware
   - **Expected**: Hardware background error

3. **Check "Has Robotics Projects" but leave description empty**
   - **Expected**: Description required error

#### 7.2 Test Character Limits

1. **Enter learning goals > 500 characters**
   - **Expected**: Character counter shows limit exceeded
   - Form prevents submission

---

### Test 8: Database Verification

#### 8.1 Check Database Tables

```bash
cd backend
uv run python -c "
import psycopg
from src.core.config import settings

conn = psycopg.connect(settings.NEON_DATABASE_URL)
cur = conn.cursor()

# Check user table
cur.execute('SELECT id, email, name FROM \"user\" ORDER BY \"createdAt\" DESC LIMIT 5')
users = cur.fetchall()
print('Recent users:')
for user in users:
    print(f'  {user[1]} ({user[2]})')

# Check user_profiles table
cur.execute('SELECT user_id, experience_level, questionnaire_completed FROM user_profiles')
profiles = cur.fetchall()
print('\nUser profiles:')
for profile in profiles:
    print(f'  User {profile[0]}: {profile[1]} (completed: {profile[2]})')

conn.close()
"
```

---

## End-to-End Testing Flow

### Complete User Journey

1. **New User Signup**:
   ```
   Visit /signup → Fill form → Submit → See questionnaire
   ```

2. **Complete Questionnaire**:
   ```
   Fill all fields → Submit → See success message → Redirect to home
   ```

3. **Sign Out**:
   ```
   Click sign out → Session cleared → Redirected to home
   ```

4. **Sign In Again**:
   ```
   Visit /signin → Enter credentials → Sign in → Redirected to home
   ```

5. **Access Protected Content**:
   ```
   Visit protected page → See content (already authenticated)
   ```

---

## Automated Testing Scripts

### Test Script 1: Auth Service Health

```bash
#!/bin/bash
# test-auth-health.sh

echo "Testing Better Auth Service Health..."
response=$(curl -s http://localhost:3001/health)
if echo "$response" | grep -q "ok"; then
    echo "✅ Auth service is healthy"
else
    echo "❌ Auth service health check failed"
    exit 1
fi
```

### Test Script 2: Signup and Questionnaire

```bash
#!/bin/bash
# test-signup-flow.sh

echo "Testing Signup Flow..."

# Signup
echo "1. Signing up..."
SIGNUP_RESPONSE=$(curl -s -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -c cookies.txt \
  -d '{
    "email": "test-'$(date +%s)'@example.com",
    "password": "testpass123",
    "name": "Test User"
  }')

if echo "$SIGNUP_RESPONSE" | grep -q "user"; then
    echo "✅ Signup successful"
else
    echo "❌ Signup failed: $SIGNUP_RESPONSE"
    exit 1
fi

# Get session token
SESSION_TOKEN=$(grep "better-auth.session_token" cookies.txt | awk '{print $7}')

# Submit questionnaire
echo "2. Submitting questionnaire..."
QUESTIONNAIRE_RESPONSE=$(curl -s -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -H "Cookie: better-auth.session_token=$SESSION_TOKEN" \
  -d '{
    "software_background": ["Python"],
    "hardware_background": ["Arduino"],
    "experience_level": "beginner",
    "learning_goals": "Learn robotics"
  }')

if echo "$QUESTIONNAIRE_RESPONSE" | grep -q "questionnaire_completed"; then
    echo "✅ Questionnaire submitted successfully"
else
    echo "❌ Questionnaire submission failed: $QUESTIONNAIRE_RESPONSE"
    exit 1
fi

echo "✅ All tests passed!"
```

---

## Troubleshooting

### Issue 1: "relation 'user' does not exist"

**Solution**: Run Better Auth migrations
```bash
cd backend/auth-service
npm run migrate
```

### Issue 2: CORS Errors

**Check**:
- Better Auth `BETTER_AUTH_TRUSTED_ORIGINS` includes `http://localhost:3000`
- FastAPI CORS middleware allows `http://localhost:3000`

### Issue 3: Session Not Persisting

**Check**:
- Cookies are enabled in browser
- `allow_credentials: true` in CORS config
- Session token is being set in cookies

### Issue 4: "Authentication required" Error

**Check**:
- Session token is being sent in request headers/cookies
- Better Auth service is running
- Session hasn't expired

### Issue 5: Database Connection Errors

**Check**:
- `NEON_DATABASE_URL` is correct in `.env` files
- Database is accessible
- Migrations have been run

---

## Browser DevTools Testing

### Check Network Requests

1. **Open Browser DevTools** (F12)
2. **Go to Network tab**
3. **Perform actions**:
   - Sign up
   - Submit questionnaire
   - Sign in

4. **Verify**:
   - Requests to `/api/auth/sign-up/email` (Better Auth)
   - Requests to `/api/auth/profile/background` (FastAPI)
   - Cookies being set (`better-auth.session_token`)

### Check Application State

1. **Go to Application tab**
2. **Check Cookies**:
   - `better-auth.session_token` should be present after signin
   - Domain should match your frontend domain

3. **Check Local Storage** (if used):
   - Any auth-related data stored

---

## Test Checklist

- [ ] Better Auth service starts successfully
- [ ] FastAPI backend starts successfully
- [ ] Frontend starts successfully
- [ ] Health checks return OK
- [ ] User can sign up via browser
- [ ] User can sign up via API
- [ ] Questionnaire form appears after signup
- [ ] Questionnaire validation works
- [ ] Questionnaire submission works via browser
- [ ] Questionnaire submission works via API
- [ ] User profile can be retrieved
- [ ] User can sign in via browser
- [ ] User can sign in via API
- [ ] Protected routes work correctly
- [ ] Session persists across page reloads
- [ ] Sign out works correctly
- [ ] Database tables are created correctly
- [ ] User data is stored in database

---

## Performance Testing

### Load Test Signup Endpoint

```bash
# Install Apache Bench (if not installed)
# Ubuntu/Debian: sudo apt-get install apache2-utils
# macOS: brew install httpd

ab -n 100 -c 10 -p signup.json -T application/json \
  http://localhost:3001/api/auth/sign-up/email
```

**signup.json**:
```json
{
  "email": "test@example.com",
  "password": "testpass123",
  "name": "Test User"
}
```

---

## Security Testing

### Test 1: Password Validation
- Try password < 8 characters → Should fail
- Try password > 128 characters → Should fail

### Test 2: SQL Injection
- Try SQL in email field → Should be sanitized

### Test 3: XSS Protection
- Try script tags in name field → Should be escaped

### Test 4: Session Security
- Try accessing profile without session → Should return 401
- Try using expired session → Should return 401

---

## Next Steps After Testing

1. **Fix any bugs found**
2. **Document test results**
3. **Create automated test suite** (optional)
4. **Set up CI/CD testing** (optional)
5. **Performance optimization** (if needed)

---

## Quick Test Commands Summary

```bash
# Health checks
curl http://localhost:3001/health
curl http://localhost:8000/health

# Signup
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123","name":"Test"}'

# Signin (save cookies)
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -c cookies.txt \
  -d '{"email":"test@example.com","password":"testpass123"}'

# Get profile
curl http://localhost:8000/api/auth/profile -b cookies.txt

# Submit questionnaire
curl -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -b cookies.txt \
  -d '{"software_background":["Python"],"hardware_background":["Arduino"],"experience_level":"beginner"}'
```

---

---

## 🤖 RAG Chatbot / Vector Database Testing

### Update Vector Database

When book content changes, update embeddings:

```bash
cd backend
uv run python ingest_content.py
```

**Expected**: 
- Processes all `frontend/docs/module-*` files
- Creates ~150-250 chunks per module
- Takes 5-10 minutes for full update

### Verify Vector Database

```bash
cd backend
uv run python -c "from src.services.vectordb.qdrant_client import get_qdrant_client; client = get_qdrant_client(); info = client.get_collection('book_embeddings'); print(f'Vectors: {info.points_count}, Status: {info.status}')"
```

**Expected**: Shows total vector count and "green" status

### Test RAG Search

**Via API**:
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

**Expected**: JSON response with answer and citations from book content

**Via Browser**:
1. Open chat widget at http://localhost:3000/physical-ai-humanoid-robotics-book/
2. Ask: "What is ROS 2?"
3. **Expected**: Response with information from Module 1 content

For detailed vector database management, see `VECTOR-DB-GUIDE.md`

---

## Need Help?

If tests fail:
1. Check service logs for errors
2. Verify environment variables are set
3. Check database connectivity
4. Verify CORS configuration
5. Check browser console for frontend errors
6. For RAG issues, see `VECTOR-DB-GUIDE.md`
