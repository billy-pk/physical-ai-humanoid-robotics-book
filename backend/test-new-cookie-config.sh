#!/bin/bash
# Test if new cookie configuration works

EMAIL="test-$(date +%s)@example.com"
echo "Testing new cookie configuration with: $EMAIL"
echo ""

# Signup and capture cookie headers
echo "=== Signup Response with Headers ==="
curl -v -c /tmp/new-config-cookies.txt -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d "{\"email\":\"$EMAIL\",\"password\":\"testpass123\",\"name\":\"Test User\"}" \
  2>&1 | grep -E "Set-Cookie|better-auth"

echo ""
echo "=== Cookie File Contents ==="
cat /tmp/new-config-cookies.txt | tail -2
echo ""

# Extract cookie value
COOKIE_VALUE=$(grep "better-auth.session_token" /tmp/new-config-cookies.txt | awk '{print $NF}')
echo "Signed cookie value: ${COOKIE_VALUE:0:40}..."
echo ""

# Test if this cookie works with FastAPI when sent in Cookie header
echo "=== Testing with FastAPI (sending cookie in Cookie header) ==="
curl -s -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -H "Cookie: better-auth.session_token=$COOKIE_VALUE" \
  -d '{"software_background":["Python"],"hardware_background":["Arduino"],"experience_level":"beginner"}' \
  | python3 -m json.tool
