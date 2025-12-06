# 🚀 Quick Start Guide

Get your RAG chatbot running in 5 minutes!

## Step 1: Get API Keys (2 minutes)

### OpenAI API Key
1. Go to https://platform.openai.com/api-keys
2. Click "Create new secret key"
3. Copy the key (starts with `sk-`)

### Qdrant Cloud (Free)
1. Go to https://cloud.qdrant.io/
2. Sign up with GitHub/Google
3. Click "Create Cluster" → Free tier
4. Copy the cluster URL and API key

## Step 2: Setup Backend (2 minutes)

```bash
# Navigate to backend
cd backend

# Copy environment file
copy .env.example .env    # Windows
# OR
cp .env.example .env      # Mac/Linux

# Edit .env and paste your keys
notepad .env              # Windows
# OR
nano .env                 # Mac/Linux
```

Your `.env` should look like:
```env
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxx
QDRANT_URL=https://xxxxx.qdrant.io
QDRANT_API_KEY=xxxxxxxxxxxxx
```

## Step 3: Start Backend (1 minute)

### Windows:
```bash
start.bat
```

### Mac/Linux:
```bash
chmod +x start.sh
./start.sh
```

Wait for: `Application startup complete.`

## Step 4: Ingest Book Content (30 seconds)

Open a new terminal:

```bash
# Windows PowerShell
Invoke-WebRequest -Uri http://localhost:8000/ingest -Method POST

# Mac/Linux/Git Bash
curl -X POST http://localhost:8000/ingest
```

You should see: `"Ingested X chunks from Y documents"`

## Step 5: Test It! (30 seconds)

### Test API:
```bash
curl -X POST http://localhost:8000/query ^
  -H "Content-Type: application/json" ^
  -d "{\"question\": \"What is Physical AI?\"}"
```

### Test Frontend:
```bash
# In project root
npm install
npm start
```

Visit http://localhost:3000 and click the 💬 button!

## ✅ Verification Checklist

- [ ] Backend running on http://localhost:8000
- [ ] `/health` endpoint returns `{"status": "ok"}`
- [ ] `/ingest` completed successfully
- [ ] Test query returns an answer
- [ ] Frontend loads at http://localhost:3000
- [ ] Chatbot widget appears (💬 button)
- [ ] Chatbot responds to questions

## 🐛 Troubleshooting

### Backend won't start
```bash
# Check Python version (need 3.11+)
python --version

# Install dependencies manually
pip install fastapi uvicorn qdrant-client openai python-dotenv
```

### "Collection not found" error
```bash
# Run ingest again
curl -X POST http://localhost:8000/ingest
```

### Chatbot not responding
1. Check browser console (F12)
2. Verify backend is running
3. Check `API_URL` in `static/chatbot-widget.js`

### OpenAI API errors
- Verify API key is correct
- Check you have credits: https://platform.openai.com/usage
- Try a different model in `main.py`

## 🎯 Next Steps

1. **Customize the chatbot**:
   - Edit `static/chatbot-widget.js` for styling
   - Modify prompts in `backend/main.py`

2. **Deploy**:
   - Follow `DEPLOYMENT.md` for production setup
   - Deploy backend to Render.com
   - Deploy frontend to GitHub Pages

3. **Enhance**:
   - Add chat history
   - Implement feedback buttons
   - Add more book content

## 📚 Documentation

- Full setup: `README.md`
- Deployment: `DEPLOYMENT.md`
- Backend API: `backend/README.md`
- API docs: http://localhost:8000/docs (when running)

## 💡 Tips

- Keep backend running while developing
- Use `/docs` endpoint to test API interactively
- Check logs for debugging
- Test with different questions
- Try the text selection feature!

## 🆘 Need Help?

1. Check the error message carefully
2. Review the relevant README section
3. Verify all API keys are correct
4. Test each component separately
5. Check GitHub issues for similar problems

---

**Ready to deploy?** See `DEPLOYMENT.md` for production setup!
