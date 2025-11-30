#!/bin/bash
# Integration test script for authentication system

set -e

AUTH_URL="http://localhost:3001"
API_URL="http://localhost:8000"
TEST_EMAIL="test-$(date +%s)@example.com"
TEST_PASSWORD="testpass123"
TEST_NAME="Test User"

echo "🧪 Starting Authentication Integration Tests"
echo "=============================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
PASSED=0
FAILED=0

test_step() {
    local name=$1
    local command=$2
    
    echo -n "Testing: $name... "
    if eval "$command" > /tmp/test_output.txt 2>&1; then
        echo -e "${GREEN}✅ PASSED${NC}"
        ((PASSED++))
        return 0
    else
        echo -e "${RED}❌ FAILED${NC}"
        cat /tmp/test_output.txt
        ((FAILED++))
        return 1
    fi
}

# Test 1: Better Auth Health
test_step "Better Auth Health Check" \
    "curl -s $AUTH_URL/health | grep -q 'ok'"

# Test 2: FastAPI Health
test_step "FastAPI Health Check" \
    "curl -s $API_URL/health | grep -q 'ok'"

# Test 3: User Signup
echo ""
echo "Testing User Signup..."
SIGNUP_RESPONSE=$(curl -s -X POST $AUTH_URL/api/auth/sign-up/email \
    -H "Content-Type: application/json" \
    -c /tmp/cookies.txt \
    -d "{\"email\":\"$TEST_EMAIL\",\"password\":\"$TEST_PASSWORD\",\"name\":\"$TEST_NAME\"}")

if echo "$SIGNUP_RESPONSE" | grep -q "user"; then
    echo -e "${GREEN}✅ User Signup: PASSED${NC}"
    ((PASSED++))
else
    echo -e "${RED}❌ User Signup: FAILED${NC}"
    echo "$SIGNUP_RESPONSE"
    ((FAILED++))
    exit 1
fi

# Extract session token
SESSION_TOKEN=$(grep "better-auth.session_token" /tmp/cookies.txt | awk '{print $7}' | tr -d '\n')

if [ -z "$SESSION_TOKEN" ]; then
    echo -e "${YELLOW}⚠️  Warning: Session token not found in cookies${NC}"
    echo "Cookies file contents:"
    cat /tmp/cookies.txt
fi

# Test 4: Get Session
echo ""
echo "Testing Session Retrieval..."
SESSION_RESPONSE=$(curl -s $AUTH_URL/api/auth/get-session \
    -H "Cookie: better-auth.session_token=$SESSION_TOKEN")

if echo "$SESSION_RESPONSE" | grep -q "user"; then
    echo -e "${GREEN}✅ Get Session: PASSED${NC}"
    ((PASSED++))
else
    echo -e "${RED}❌ Get Session: FAILED${NC}"
    echo "$SESSION_RESPONSE"
    ((FAILED++))
fi

# Test 5: Submit Questionnaire
echo ""
echo "Testing Questionnaire Submission..."
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
    echo -e "${GREEN}✅ Questionnaire Submission: PASSED${NC}"
    ((PASSED++))
else
    echo -e "${RED}❌ Questionnaire Submission: FAILED${NC}"
    echo "$QUESTIONNAIRE_RESPONSE"
    ((FAILED++))
fi

# Test 6: Get User Profile
echo ""
echo "Testing Get User Profile..."
PROFILE_RESPONSE=$(curl -s $API_URL/api/auth/profile \
    -H "Cookie: better-auth.session_token=$SESSION_TOKEN")

if echo "$PROFILE_RESPONSE" | grep -q "user_id"; then
    echo -e "${GREEN}✅ Get Profile: PASSED${NC}"
    ((PASSED++))
    
    # Check questionnaire completion
    if echo "$PROFILE_RESPONSE" | grep -q '"questionnaire_completed": true'; then
        echo -e "${GREEN}✅ Questionnaire Completion Status: PASSED${NC}"
        ((PASSED++))
    else
        echo -e "${RED}❌ Questionnaire Completion Status: FAILED${NC}"
        ((FAILED++))
    fi
else
    echo -e "${RED}❌ Get Profile: FAILED${NC}"
    echo "$PROFILE_RESPONSE"
    ((FAILED++))
fi

# Test 7: Sign In
echo ""
echo "Testing Sign In..."
SIGNIN_RESPONSE=$(curl -s -X POST $AUTH_URL/api/auth/sign-in/email \
    -H "Content-Type: application/json" \
    -c /tmp/cookies_signin.txt \
    -d "{\"email\":\"$TEST_EMAIL\",\"password\":\"$TEST_PASSWORD\"}")

if echo "$SIGNIN_RESPONSE" | grep -q "user"; then
    echo -e "${GREEN}✅ Sign In: PASSED${NC}"
    ((PASSED++))
else
    echo -e "${RED}❌ Sign In: FAILED${NC}"
    echo "$SIGNIN_RESPONSE"
    ((FAILED++))
fi

# Summary
echo ""
echo "=============================================="
echo "Test Summary"
echo "=============================================="
echo -e "${GREEN}Passed: $PASSED${NC}"
if [ $FAILED -gt 0 ]; then
    echo -e "${RED}Failed: $FAILED${NC}"
    exit 1
else
    echo -e "${GREEN}Failed: $FAILED${NC}"
    echo ""
    echo -e "${GREEN}✅ All tests passed!${NC}"
    exit 0
fi
