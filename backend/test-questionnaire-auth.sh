#!/bin/bash
# Test script to verify questionnaire authentication flow with cookie-based auth

set -e

echo "=== Testing Questionnaire Authentication Flow (Cookie-Based) ==="
echo ""

# Step 1: Create a new user
echo "Step 1: Creating new test user..."
EMAIL="test-$(date +%s)@example.com"
SIGNUP_RESPONSE=$(curl -s -c /tmp/test-cookies.txt -b /tmp/test-cookies.txt -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -H "Origin: http://localhost:3000" \
  -d "{\"email\":\"$EMAIL\",\"password\":\"testpass123\",\"name\":\"Test User\"}")

echo "Signup response:"
echo "$SIGNUP_RESPONSE" | python3 -m json.tool 2>/dev/null || echo "$SIGNUP_RESPONSE"
echo ""

# Extract token from response
TOKEN=$(echo "$SIGNUP_RESPONSE" | python3 -c "import sys, json; print(json.load(sys.stdin).get('token', ''))" 2>/dev/null || echo "")

if [ -z "$TOKEN" ]; then
  echo "ERROR: No token in signup response"
  exit 1
fi

echo "Session token: ${TOKEN:0:20}..."
echo ""

# Step 2: Verify session with Better Auth (using cookies)
echo "Step 2: Verifying session with Better Auth (via cookies)..."
SESSION_RESPONSE=$(curl -s -b /tmp/test-cookies.txt http://localhost:3001/api/auth/get-session \
  -H "Origin: http://localhost:3000")
echo "$SESSION_RESPONSE" | python3 -m json.tool 2>/dev/null || echo "$SESSION_RESPONSE"
echo ""

# Step 3: Test questionnaire submission WITH COOKIES (simulating browser behavior)
echo "Step 3: Submitting questionnaire to FastAPI with cookies..."
echo "(Simulating browser with credentials: 'include')"
QUEST_RESPONSE=$(curl -s -w "\nHTTP_STATUS:%{http_code}" \
  -b /tmp/test-cookies.txt \
  -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -H "Origin: http://localhost:3000" \
  -d '{
    "software_background": ["Python"],
    "hardware_background": ["Arduino"],
    "experience_level": "beginner",
    "learning_goals": "Test goals"
  }')

HTTP_STATUS=$(echo "$QUEST_RESPONSE" | grep "HTTP_STATUS" | cut -d':' -f2)
RESPONSE_BODY=$(echo "$QUEST_RESPONSE" | sed '/HTTP_STATUS/d')

echo "HTTP Status: $HTTP_STATUS"
echo "Response:"
echo "$RESPONSE_BODY" | python3 -m json.tool 2>/dev/null || echo "$RESPONSE_BODY"
echo ""

if [ "$HTTP_STATUS" = "200" ]; then
  echo "✅ SUCCESS: Questionnaire submitted successfully with cookie-based authentication!"
  echo ""
  echo "Summary:"
  echo "- User created with Better Auth"
  echo "- Signed cookie set by Better Auth (sameSite: lax)"
  echo "- Cookie automatically sent to FastAPI (different port)"
  echo "- FastAPI validated session with Better Auth"
  echo "- Profile created successfully"
  exit 0
else
  echo "❌ FAILED: HTTP $HTTP_STATUS"
  echo ""
  echo "Checking if cookies were saved..."
  echo "Cookies in /tmp/test-cookies.txt:"
  cat /tmp/test-cookies.txt | grep -v "^#"
  exit 1
fi
