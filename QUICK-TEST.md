# Quick Testing Guide

## 🚀 Quick Start

### 1. Start All Services

**Terminal 1 - Better Auth**:
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

### 2. Test in Browser

1. **Open**: http://localhost:3000/signup
2. **Sign Up**: Fill form and submit
3. **Questionnaire**: Fill questionnaire and submit
4. **Verify**: Should redirect to home page

### 3. Test API Endpoints

**Signup**:
```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123","name":"Test"}'
```

**Signin** (save cookies):
```bash
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -c cookies.txt \
  -d '{"email":"test@example.com","password":"testpass123"}'
```

**Get Profile**:
```bash
curl http://localhost:8000/api/auth/profile -b cookies.txt
```

**Submit Questionnaire**:
```bash
curl -X POST http://localhost:8000/api/auth/profile/background \
  -H "Content-Type: application/json" \
  -b cookies.txt \
  -d '{
    "software_background": ["Python"],
    "hardware_background": ["Arduino"],
    "experience_level": "beginner",
    "learning_goals": "Learn robotics"
  }'
```

### 4. Run Automated Tests

```bash
cd backend
./test-auth-integration.sh
```

---

## ✅ Checklist

- [ ] All 3 services running
- [ ] Can sign up via browser
- [ ] Questionnaire appears after signup
- [ ] Can submit questionnaire
- [ ] Can sign in
- [ ] Profile API works
- [ ] Database has user data

---

For detailed testing, see `TESTING-GUIDE.md`
