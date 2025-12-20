# 🚀 Quick Start - RAG Chatbot

## ⚡ 3-Minute Test

```bash
# 1. Test all components (30 seconds)
cd backend
python test_rag.py

# 2. Start backend (30 seconds to start, 10 min for first ingestion)
python main_rag.py

# 3. In new terminal: Test API (10 seconds)
curl http://localhost:8000/health

# 4. In new terminal: Start frontend (2 minutes)
cd ..
npm start

# 5. Visit http://localhost:3000 and click 💬 button!
```

## ✅ Expected Results

### After `test_rag.py`:
```
✅ ALL TESTS PASSED!
```

### After `main_rag.py` starts:
```
✅ Database initialized successfully
✅ Qdrant collection exists: physical_ai_book
📚 Found 18 documents
✅ Ingestion complete! 450 chunks uploaded
```

### After `/health` check:
```json
{
  "status": "healthy",
  "qdrant_points": 450,
  "database_connected": true
}
```

### In browser (localhost:3000):
- See chatbot button (💬) bottom-right
- Click to open
- Ask: "What is Physical AI?"
- Get answer with sources!

## 🌐 Deploy to Vercel

```bash
# 1. Add environment variables in Vercel dashboard
# (See RAG_IMPLEMENTATION_SUMMARY.md for full list)

# 2. Deploy
git add .
git commit -m "Add RAG chatbot"
git push origin main

# 3. After deploy succeeds, ingest documents
curl -X POST https://physical-ai-book.vercel.app/api/ingest

# 4. Test
curl https://physical-ai-book.vercel.app/api/health
```

## 🧪 Quick Tests

```bash
# Test query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'

# Check stats
curl http://localhost:8000/stats

# Check Qdrant points
curl http://localhost:8000/health
```

## 📚 Full Documentation

- **Setup Guide**: `SETUP_RAG_CHATBOT.md`
- **Implementation Summary**: `RAG_IMPLEMENTATION_SUMMARY.md`
- **Project Memory**: `PROJECT_MEMORY.md`

## 🆘 Troubleshooting

**Issue**: "Collection is empty"
```bash
curl -X POST http://localhost:8000/ingest
```

**Issue**: "Database connection error"
```bash
# Check .env file has DATABASE_URL
cat backend/.env | grep DATABASE_URL
```

**Issue**: "Module not found"
```bash
cd backend
pip install -r requirements_rag.txt
```

## 🎯 Success Checklist

- [ ] `test_rag.py` passes all tests
- [ ] Backend starts without errors
- [ ] `/health` returns "healthy"
- [ ] Frontend shows chatbot button
- [ ] Can ask questions and get answers
- [ ] Sources are displayed
- [ ] Text selection works

## 🎉 You're Done!

Chatbot is ready to use! 🚀
