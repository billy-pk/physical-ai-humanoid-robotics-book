#!/bin/bash
# Create fresh session and test immediately

EMAIL="test-$(date +%s)@example.com"
echo "Creating fresh user: $EMAIL"

# Signup and capture both response and cookies
RESPONSE=$(curl -s -c /tmp/fresh-cookies.txt -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d "{\"email\":\"$EMAIL\",\"password\":\"testpass123\",\"name\":\"Test User\"}")

echo "Signup response:"
echo "$RESPONSE" | python3 -m json.tool

# Get the actual cookie value
COOKIE_VALUE=$(grep "better-auth.session_token" /tmp/fresh-cookies.txt | awk '{print $NF}')
echo ""
echo "Cookie value from file: $COOKIE_VALUE"
echo ""

# Test immediately with cookie
echo "Testing with cookie from file..."
curl -s -b /tmp/fresh-cookies.txt http://localhost:3001/api/auth/get-session | python3 -m json.tool
echo ""

# Now test what the backend does - extract token and use it
TOKEN=$(echo "$RESPONSE" | python3 -c "import sys, json; print(json.load(sys.stdin).get('token', ''))")
echo "Token from JSON response: $TOKEN"
echo ""
echo "Testing FastAPI with Bearer token..."
curl -s -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $TOKEN" \
  -d '{"software_background":["Python"],"hardware_background":["Arduino"],"experience_level":"beginner"}' \
  | python3 -m json.tool
