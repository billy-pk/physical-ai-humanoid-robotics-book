#!/bin/bash
# Complete authentication flow test script

set -e

AUTH_URL="http://localhost:3001"
API_URL="http://localhost:8000"
TEST_EMAIL="test-complete-$(date +%s)@example.com"
TEST_PASSWORD="testpass123"
TEST_NAME="Complete Test User"

echo "🧪 Testing Complete Authentication Flow"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "Test Email: $TEST_EMAIL"
echo ""

# Step 1: Signup
echo "Step 1: User Signup..."
SIGNUP_RESPONSE=$(curl -s -X POST $AUTH_URL/api/auth/sign-up/email \
    -H "Content-Type: application/json" \
    -c /tmp/auth_test_cookies.txt \
    -d "{\"email\":\"$TEST_EMAIL\",\"password\":\"$TEST_PASSWORD\",\"name\":\"$TEST_NAME\"}")

if echo "$SIGNUP_RESPONSE" | grep -q "user"; then
    echo -e "${GREEN}✅ Signup successful${NC}"
    USER_ID=$(echo "$SIGNUP_RESPONSE" | python3 -c "import sys, json; print(json.load(sys.stdin)['user']['id'])" 2>/dev/null || echo "")
    echo "   User ID: $USER_ID"
else
    echo -e "${RED}❌ Signup failed${NC}"
    echo "$SIGNUP_RESPONSE"
    exit 1
fi

echo ""

# Step 2: Signin (with same credentials)
echo "Step 2: User Signin..."
SIGNIN_RESPONSE=$(curl -s -X POST $AUTH_URL/api/auth/sign-in/email \
    -H "Content-Type: application/json" \
    -c /tmp/auth_signin_cookies.txt \
    -d "{\"email\":\"$TEST_EMAIL\",\"password\":\"$TEST_PASSWORD\"}")

if echo "$SIGNIN_RESPONSE" | grep -q "user"; then
    echo -e "${GREEN}✅ Signin successful${NC}"
else
    echo -e "${RED}❌ Signin failed${NC}"
    echo "$SIGNIN_RESPONSE"
    exit 1
fi

echo ""

# Step 3: Get Session
echo "Step 3: Get Session..."
SESSION_TOKEN=$(grep "better-auth.session_token" /tmp/auth_test_cookies.txt | awk '{print $7}' | python3 -c "import sys, urllib.parse; print(urllib.parse.unquote(sys.stdin.read().strip()))" 2>/dev/null || grep "better-auth.session_token" /tmp/auth_test_cookies.txt | awk '{print $7}')

if [ -z "$SESSION_TOKEN" ]; then
    echo -e "${YELLOW}⚠️  Warning: Could not extract session token${NC}"
    echo "Cookie file contents:"
    cat /tmp/auth_test_cookies.txt
else
    SESSION_RESPONSE=$(curl -s $AUTH_URL/api/auth/get-session \
        -H "Cookie: better-auth.session_token=$SESSION_TOKEN")
    
    if echo "$SESSION_RESPONSE" | grep -q "user"; then
        echo -e "${GREEN}✅ Session retrieved successfully${NC}"
    else
        echo -e "${RED}❌ Session retrieval failed${NC}"
        echo "$SESSION_RESPONSE"
    fi
fi

echo ""

# Step 4: Submit Questionnaire
echo "Step 4: Submit Questionnaire..."
if [ -n "$SESSION_TOKEN" ]; then
    QUESTIONNAIRE_RESPONSE=$(curl -s -X POST $API_URL/api/auth/profile/background \
        -H "Content-Type: application/json" \
        -H "Cookie: better-auth.session_token=$SESSION_TOKEN" \
        -d '{
            "software_background": ["Python", "JavaScript"],
            "hardware_background": ["Arduino", "Raspberry Pi"],
            "experience_level": "intermediate",
            "learning_goals": "Build a humanoid robot",
            "has_robotics_projects": true,
            "robotics_projects_description": "Built a line-following robot",
            "programming_years": 5,
            "learning_style": "hands-on"
        }')
    
    if echo "$QUESTIONNAIRE_RESPONSE" | grep -q "questionnaire_completed"; then
        echo -e "${GREEN}✅ Questionnaire submitted successfully${NC}"
    else
        echo -e "${YELLOW}⚠️  Questionnaire submission response:${NC}"
        echo "$QUESTIONNAIRE_RESPONSE"
    fi
else
    echo -e "${YELLOW}⚠️  Skipping questionnaire (no session token)${NC}"
fi

echo ""

# Step 5: Get Profile
echo "Step 5: Get User Profile..."
if [ -n "$SESSION_TOKEN" ]; then
    PROFILE_RESPONSE=$(curl -s $API_URL/api/auth/profile \
        -H "Cookie: better-auth.session_token=$SESSION_TOKEN")
    
    if echo "$PROFILE_RESPONSE" | grep -q "user_id"; then
        echo -e "${GREEN}✅ Profile retrieved successfully${NC}"
        echo "$PROFILE_RESPONSE" | python3 -c "import sys, json; p=json.load(sys.stdin); print(f\"   Experience: {p.get('experience_level', 'N/A')}\"); print(f\"   Completed: {p.get('questionnaire_completed', False)}\")" 2>/dev/null || echo "   Profile data retrieved"
    else
        echo -e "${YELLOW}⚠️  Profile response:${NC}"
        echo "$PROFILE_RESPONSE"
    fi
else
    echo -e "${YELLOW}⚠️  Skipping profile retrieval (no session token)${NC}"
fi

echo ""
echo "========================================"
echo -e "${GREEN}✅ Complete authentication flow test finished${NC}"
echo ""
echo "Test email used: $TEST_EMAIL"
echo "You can use this email to test signin in the browser"
