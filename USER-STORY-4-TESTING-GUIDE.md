# User Story 4 (Urdu Translation) - Testing Guide

**Goal**: Verify that Urdu translation feature works correctly with code block preservation

**Independent Test**: Enable Urdu preference, request translation for a chapter, verify output is in Urdu with code blocks preserved in English

---

## Prerequisites

### 1. Start All Services

**Terminal 1 - Better Auth Service**:
```bash
cd backend/auth-service
npm run dev
```

**Terminal 2 - FastAPI Backend**:
```bash
cd backend
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Terminal 3 - Frontend**:
```bash
cd frontend
npm start
```

### 2. Ensure You Have:
- A user account (sign up if needed)
- User preferences set (especially `urdu_translation_enabled: true`)
- At least one chapter available (e.g., `intro` or `module-1/chapter-1`)

---

## Testing Methods

### Method 1: Browser Testing (Recommended)

#### Step 1: Sign In and Enable Urdu Translation

1. **Sign In**:
   - Navigate to: `http://localhost:3000/physical-ai-humanoid-robotics-book/signin`
   - Sign in with your credentials

2. **Enable Urdu Translation**:
   - Option A: Via Preferences Form
     - Go to: `http://localhost:3000/physical-ai-humanoid-robotics-book/popup`
     - Check "Urdu Translation Enabled"
     - Submit preferences
   
   - Option B: Via Settings Page
     - Go to: `http://localhost:3000/physical-ai-humanoid-robotics-book/settings`
     - Toggle "Urdu Translation Enabled"
     - Save preferences

#### Step 2: Navigate to a Chapter

1. **Open a Chapter**:
   - Navigate to: `http://localhost:3000/physical-ai-humanoid-robotics-book/chapter?chapter_id=intro`
   - Or any valid chapter ID (e.g., `module-1/chapter-1`)

2. **Verify Translation Toggle Appears**:
   - You should see a checkbox labeled "Translate to Urdu" in the top-right area
   - The toggle should be visible when `urdu_translation_enabled` is `true` in preferences

#### Step 3: Test Translation

1. **Enable Translation**:
   - Check the "Translate to Urdu" checkbox
   - Wait for translation to complete (loading indicator should appear)

2. **Verify Translation**:
   - Content should be displayed in Urdu
   - Code blocks (```...```) should remain in English
   - Markdown formatting should be preserved

3. **Toggle Back to English**:
   - Uncheck "Translate to Urdu"
   - Content should revert to original English

#### Step 4: Verify Code Block Preservation

1. **Find Code Blocks**:
   - Look for code blocks in the original content (marked with ```)
   - Note their content

2. **After Translation**:
   - Verify the same code blocks appear in the translated content
   - Code should be in English (not translated)
   - Code structure should be identical

---

### Method 2: API Testing (Backend Verification)

#### Step 1: Get Authentication Token

```bash
# Sign in and save session token
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -c cookies.txt \
  -d '{"email":"your-email@example.com","password":"your-password"}'

# Extract token from response (look for "token" field)
# Or get session from Better Auth
curl http://localhost:3001/api/auth/get-session \
  -b cookies.txt \
  -c cookies.txt
```

**Save the token** from the response for use in Authorization header.

#### Step 2: Enable Urdu Translation in Preferences

```bash
# Update preferences to enable Urdu translation
curl -X POST http://localhost:8000/api/auth/profile/preferences \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN" \
  -d '{
    "experience_level": "intermediate",
    "learning_topics": ["ROS 2", "Computer Vision"],
    "learning_goals": "Learn robotics",
    "content_mode": "full",
    "urdu_translation_enabled": true
  }'
```

**Expected Response**: `200 OK` with updated preferences JSON

#### Step 3: Test Translation Endpoint

```bash
# Request translation for a chapter
curl -X POST http://localhost:8000/api/content/translate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN" \
  -d '{
    "chapter_id": "intro",
    "target_language": "ur",
    "force_regenerate": false
  }'
```

**Expected Response**: 
- `200 OK`
- Streaming response with translated markdown content
- Code blocks preserved in English

#### Step 4: Verify Caching

```bash
# Request same translation again (should be cached)
curl -X POST http://localhost:8000/api/content/translate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN" \
  -d '{
    "chapter_id": "intro",
    "target_language": "ur",
    "force_regenerate": false
  }'
```

**Expected**: 
- Faster response (cached)
- Check FastAPI logs for "Cache hit" message

#### Step 5: Test Force Regenerate

```bash
# Force regenerate (bypass cache)
curl -X POST http://localhost:8000/api/content/translate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN" \
  -d '{
    "chapter_id": "intro",
    "target_language": "ur",
    "force_regenerate": true
  }'
```

**Expected**: 
- New translation generated
- Check FastAPI logs for "Cache miss" or "force regenerate" message

---

### Method 3: Test via Chapter Endpoint (Automatic Translation)

#### Test Automatic Translation When Preference Enabled

```bash
# Get chapter content (should auto-translate if urdu_translation_enabled is true)
curl -X GET "http://localhost:8000/api/content/chapters/intro" \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN" \
  -H "Accept: text/markdown"
```

**Expected Behavior**:
- If `urdu_translation_enabled: true` in preferences:
  - Content is automatically translated to Urdu
  - Code blocks preserved
- If `urdu_translation_enabled: false`:
  - Original English content returned

---

## Verification Checklist

### ✅ Backend Verification

- [ ] Translation endpoint exists: `POST /api/content/translate`
- [ ] Endpoint requires authentication (401 without token)
- [ ] Endpoint validates `target_language` (only "ur" accepted)
- [ ] Translation returns Urdu text
- [ ] Code blocks are preserved in English
- [ ] Caching works (second request is faster)
- [ ] Cache can be bypassed with `force_regenerate: true`
- [ ] Chapter endpoint auto-translates when preference enabled

### ✅ Frontend Verification

- [ ] TranslationToggle component appears on chapter page
- [ ] Toggle is visible when `urdu_translation_enabled: true`
- [ ] Toggle triggers translation API call
- [ ] Loading indicator shows during translation
- [ ] Translated content displays in Urdu
- [ ] Code blocks remain in English
- [ ] Toggle can switch back to English
- [ ] Error handling works (shows error message on failure)

### ✅ Code Block Preservation

- [ ] Original content has code blocks
- [ ] Translated content has same number of code blocks
- [ ] Code block content is identical (not translated)
- [ ] Code block syntax (```) is preserved
- [ ] Code block language tags (if any) are preserved

### ✅ Caching Verification

- [ ] First translation request takes time (generation)
- [ ] Second request is faster (cached)
- [ ] Cache persists across page refreshes
- [ ] Force regenerate bypasses cache
- [ ] Cache key includes chapter_id, user_id, target_language

---

## Expected Test Results

### Successful Translation Response

**Request**:
```json
{
  "chapter_id": "intro",
  "target_language": "ur",
  "force_regenerate": false
}
```

**Response** (Streaming):
```markdown
# تعارف

یہ باب آپ کو Physical AI اور Humanoid Robotics کے بارے میں تعارف فراہم کرتا ہے۔

```python
# This code block should remain in English
def hello():
    print("Hello, World!")
```
```

**Key Points**:
- ✅ Text is in Urdu
- ✅ Code block is in English
- ✅ Markdown formatting preserved

---

## Debugging

### Check FastAPI Logs

```bash
# Watch FastAPI logs in Terminal 2
# Look for:
# - "Cache hit" or "Cache miss"
# - "Translating..." messages
# - "Code block count mismatch" warnings (if any)
```

### Check Browser Console

1. Open browser DevTools (F12)
2. Go to Console tab
3. Look for:
   - Translation API calls
   - Error messages
   - Loading states

### Common Issues

**Issue**: Translation toggle doesn't appear
- **Check**: User preferences have `urdu_translation_enabled: true`
- **Fix**: Update preferences via `/popup` or `/settings`

**Issue**: Translation fails with 401
- **Check**: Session token is valid and included in Authorization header
- **Fix**: Re-sign in and get new token

**Issue**: Code blocks are translated
- **Check**: Code block extraction regex is working
- **Fix**: Verify `CODE_BLOCK_REGEX` in `translator.py`

**Issue**: Translation is slow
- **Check**: OpenAI API response time
- **Note**: First request is slower (generation), subsequent requests use cache

---

## Database Verification

### Check Cache Table

```sql
-- Connect to your Neon Postgres database
SELECT 
    chapter_id,
    content_type,
    target_language,
    created_at,
    LENGTH(generated_content) as content_length
FROM personalized_content_cache
WHERE content_type = 'translated'
ORDER BY created_at DESC
LIMIT 10;
```

**Expected**: Rows with `content_type='translated'` and `target_language='ur'`

---

## Performance Testing

### Measure Translation Time

```bash
# First request (generation)
time curl -X POST http://localhost:8000/api/content/translate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{"chapter_id":"intro","target_language":"ur","force_regenerate":true}'

# Second request (cached)
time curl -X POST http://localhost:8000/api/content/translate \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{"chapter_id":"intro","target_language":"ur","force_regenerate":false}'
```

**Expected**:
- First request: 5-15 seconds (depends on content length)
- Second request: < 1 second (cached)

---

## Next Steps

After verifying User Story 4 works:

1. ✅ All tests pass
2. ✅ Code blocks preserved
3. ✅ Caching works
4. ✅ Frontend integration works

**Then proceed with**:
- Refactoring to use Agents SDK (if desired)
- Performance optimizations
- Additional language support (if needed)

