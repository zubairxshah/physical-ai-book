# Deployment Guide - Physical AI Book with RAG Chatbot

Complete guide to deploy your Docusaurus book with integrated RAG chatbot to GitHub Pages.

## Prerequisites

- GitHub account
- OpenAI API key
- Qdrant Cloud account (free tier)
- Render.com account (free tier) or similar hosting

## Step 1: Setup Backend

### 1.1 Get API Keys

**OpenAI:**
1. Go to https://platform.openai.com/api-keys
2. Create new API key
3. Copy and save securely

**Qdrant Cloud:**
1. Go to https://cloud.qdrant.io/
2. Sign up for free
3. Create a new cluster
4. Copy cluster URL and API key

### 1.2 Configure Backend

```bash
cd backend
cp .env.example .env
```

Edit `.env`:
```env
OPENAI_API_KEY=sk-proj-xxxxx
QDRANT_URL=https://xxxxx.qdrant.io
QDRANT_API_KEY=xxxxx
```

### 1.3 Test Locally

```bash
# Install dependencies
pip install -r requirements.txt

# Run server
uvicorn main:app --reload --port 8000

# In another terminal, ingest content
curl -X POST http://localhost:8000/ingest

# Test query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

## Step 2: Deploy Backend to Render

### 2.1 Prepare for Deployment

Create `backend/render.yaml`:
```yaml
services:
  - type: web
    name: physical-ai-chatbot
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: OPENAI_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
```

### 2.2 Deploy to Render

1. Go to https://render.com
2. Click "New +" → "Web Service"
3. Connect your GitHub repository
4. Configure:
   - **Name**: physical-ai-chatbot
   - **Root Directory**: backend
   - **Environment**: Python 3
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables:
   - OPENAI_API_KEY
   - QDRANT_URL
   - QDRANT_API_KEY
6. Click "Create Web Service"

### 2.3 Ingest Content

After deployment:
```bash
curl -X POST https://your-app.onrender.com/ingest
```

## Step 3: Update Frontend

Edit `static/chatbot-widget.js`:

```javascript
// Change this line:
const API_URL = 'http://localhost:8000';

// To your deployed URL:
const API_URL = 'https://your-app.onrender.com';
```

## Step 4: Deploy to GitHub Pages

### 4.1 Update docusaurus.config.ts

Ensure these settings are correct:
```typescript
url: 'https://YOUR_USERNAME.github.io',
baseUrl: '/physical-ai-book/',
organizationName: 'YOUR_USERNAME',
projectName: 'physical-ai-book',
```

### 4.2 Build and Deploy

```bash
# Install dependencies
npm install

# Build
npm run build

# Deploy to GitHub Pages
npm run deploy
```

Or use GitHub Actions (create `.github/workflows/deploy.yml`):

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### 4.3 Configure GitHub Pages

1. Go to repository Settings → Pages
2. Source: Deploy from branch
3. Branch: gh-pages / root
4. Save

Your site will be live at: `https://YOUR_USERNAME.github.io/physical-ai-book/`

## Step 5: Test Everything

1. Visit your deployed site
2. Click the chatbot widget (💬 button)
3. Try these tests:
   - Ask: "What is Physical AI?"
   - Select some text on the page
   - Ask: "Explain this in simple terms"
   - Ask: "What are the main challenges?"

## Troubleshooting

### Chatbot not responding

**Check browser console:**
```javascript
// Open DevTools (F12) → Console
// Look for errors
```

**Common issues:**
- CORS error: Backend needs to allow your GitHub Pages domain
- 404 error: Check API_URL in chatbot-widget.js
- 500 error: Check backend logs on Render

### Backend issues

**View logs on Render:**
1. Go to your service dashboard
2. Click "Logs" tab
3. Look for errors

**Common issues:**
- Missing environment variables
- Qdrant connection failed
- OpenAI API key invalid

### Ingest failed

```bash
# Check if docs folder is accessible
curl https://your-app.onrender.com/health

# Try ingesting again
curl -X POST https://your-app.onrender.com/ingest
```

## Alternative Deployment Options

### Backend Alternatives

**Railway.app:**
- Similar to Render
- Free tier available
- Easy deployment

**Vercel (Serverless):**
- Need to adapt to serverless functions
- Free tier generous
- Global CDN

**AWS Lambda:**
- Serverless option
- More complex setup
- Pay per use

### Frontend Alternatives

**Vercel:**
```bash
npm install -g vercel
vercel --prod
```

**Netlify:**
```bash
npm install -g netlify-cli
netlify deploy --prod
```

## Cost Breakdown

**Free Tier:**
- GitHub Pages: Free
- Render: 750 hours/month free
- Qdrant Cloud: 1GB free
- OpenAI: Pay per use (~$1-5/month for moderate use)

**Total: ~$1-5/month** (mostly OpenAI usage)

## Monitoring

**Backend Health:**
```bash
curl https://your-app.onrender.com/health
```

**Check Qdrant:**
- Login to Qdrant Cloud dashboard
- View cluster metrics
- Check collection size

**OpenAI Usage:**
- Visit https://platform.openai.com/usage
- Monitor API calls and costs

## Security Best Practices

1. **Never commit API keys** to GitHub
2. Use environment variables
3. Rotate keys periodically
4. Monitor usage for anomalies
5. Set spending limits on OpenAI

## Next Steps

1. **Customize chatbot appearance** in `chatbot-widget.js`
2. **Improve prompts** in `backend/main.py`
3. **Add more features**:
   - Chat history
   - Feedback buttons
   - Export conversations
4. **Optimize performance**:
   - Cache frequent queries
   - Adjust chunk sizes
   - Fine-tune retrieval

## Support

If you encounter issues:
1. Check logs (browser console + Render logs)
2. Verify all API keys are correct
3. Test backend endpoints directly
4. Review CORS configuration

Good luck with your hackathon! 🚀
