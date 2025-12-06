# 🚀 START HERE - Quick Setup

Follow these steps in order to get everything working.

## Step 1: Setup Qdrant (5 minutes)

1. Go to https://cloud.qdrant.io/
2. Sign up (free, no credit card)
3. Create a cluster (free tier)
4. Copy your cluster URL and API key
5. Update `backend/.env`:
   ```env
   QDRANT_URL=https://your-cluster-url.qdrant.io:6333
   QDRANT_API_KEY=your-api-key-here
   ```

**Detailed guide**: See `SETUP_QDRANT.md`

## Step 2: Start Backend (2 minutes)

Open Terminal 1:

```bash
cd backend

# Windows
start.bat

# Mac/Linux
chmod +x start.sh
./start.sh
```

Wait for: `Application startup complete.`

## Step 3: Ingest Book Content (1 minute)

Open Terminal 2:

```bash
# Windows PowerShell
Invoke-WebRequest -Uri http://localhost:8000/ingest -Method POST

# Mac/Linux/Git Bash
curl -X POST http://localhost:8000/ingest
```

You should see: `"Ingested X chunks from Y documents"`

## Step 4: Start Frontend (1 minute)

Open Terminal 3:

```bash
# In project root
npm install
npm start
```

Wait for browser to open automatically.

## Step 5: Test Everything

### Test 1: Check Main Page
- URL: http://localhost:3000/physical-ai-book/
- Should show: Book introduction page
- Should see: 💬 button in bottom-right corner

### Test 2: Check Chatbot Widget
- Click the 💬 button
- Chat window should open
- Type: "What is Physical AI?"
- Press Send
- Should get an answer with sources

### Test 3: Test Text Selection
- Select any text on the page
- Notice it appears in chatbot
- Ask: "Explain this"
- Should get contextual answer

## ✅ Success Checklist

- [ ] Backend running on http://localhost:8000
- [ ] `/health` returns `{"status":"ok"}`
- [ ] `/ingest` completed successfully
- [ ] Frontend loads at http://localhost:3000/physical-ai-book/
- [ ] Chatbot 💬 button visible
- [ ] Chatbot responds to questions
- [ ] Text selection works

## 🐛 Troubleshooting

### Chatbot not appearing?

1. **Check browser console** (F12):
   - Look for errors
   - Should see: "Chatbot widget loaded successfully"

2. **Try test page**:
   - http://localhost:3000/physical-ai-book/test-chatbot.html
   - If it works here, restart dev server

3. **Clear cache and restart**:
   ```bash
   npm run clear
   npm start
   ```

### Backend errors?

1. **Check Qdrant URL**:
   - Must include `:6333` port
   - Must be real cluster URL (not placeholder)

2. **Check OpenAI key**:
   - Should start with `sk-proj-`
   - Test at: https://platform.openai.com/api-keys

3. **Restart backend**:
   ```bash
   cd backend
   # Stop with Ctrl+C
   uvicorn main:app --reload --port 8000
   ```

### No answers from chatbot?

1. **Run ingest again**:
   ```bash
   curl -X POST http://localhost:8000/ingest
   ```

2. **Check backend logs**:
   - Look in Terminal 1 for errors

3. **Test backend directly**:
   ```bash
   curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d "{\"question\":\"What is Physical AI?\"}"
   ```

## 📚 More Help

- **Full debug guide**: `DEBUG_CHECKLIST.md`
- **Qdrant setup**: `SETUP_QDRANT.md`
- **Deployment**: `DEPLOYMENT.md`
- **Quick start**: `QUICKSTART.md`

## 🎯 Next Steps

Once everything works locally:

1. **Test thoroughly**:
   - Try different questions
   - Test text selection
   - Check on mobile view

2. **Deploy**:
   - Follow `DEPLOYMENT.md`
   - Deploy backend to Render
   - Deploy frontend to GitHub Pages

3. **Record demo video**:
   - Show book navigation
   - Demonstrate chatbot
   - Show text selection feature

4. **Submit**:
   - Fill form: https://forms.gle/rw7Lepqwbob1y6hf6
   - Join Zoom on Monday 6 PM

Good luck! 🚀
