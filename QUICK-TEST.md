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

## 🔍 Vector Database / RAG

### Update Embeddings (After Adding New Book Content)

```bash
cd backend
uv run python ingest_content.py
```

**What it does**:
- Scans `frontend/docs/module-*` for markdown files
- Generates embeddings using OpenAI
- Uploads to Qdrant vector database
- Takes ~5-10 minutes for all modules

**Verify embeddings uploaded**:
```bash
cd backend
uv run python -c "from src.services.vectordb.qdrant_client import get_qdrant_client; client = get_qdrant_client(); info = client.get_collection('book_embeddings'); print(f'Vectors: {info.points_count}')"
```

### Test RAG Chatbot

**Via API**:
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

**Via Browser**:
1. Open: http://localhost:3000/physical-ai-humanoid-robotics-book/
2. Click chat widget (bottom-right)
3. Ask: "What is ROS 2?"

---

## ✅ Checklist

- [ ] All 3 services running
- [ ] Can sign up via browser
- [ ] Questionnaire appears after signup
- [ ] Can submit questionnaire
- [ ] Can sign in
- [ ] Profile API works
- [ ] Database has user data
- [ ] Vector database has embeddings
- [ ] RAG chatbot responds correctly

---

For detailed testing, see `TESTING-GUIDE.md`
