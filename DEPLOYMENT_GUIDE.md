# GitHub Pages Deployment Guide

## 🚀 Quick Deployment Steps

### 1. Push to GitHub
```bash
git add .
git commit -m "Ready for deployment"
git push origin main
```

### 2. Enable GitHub Pages
1. Go to your repository: https://github.com/zubairxshah/physical-ai-book
2. Click **Settings** → **Pages**
3. Under **Source**, select **GitHub Actions**
4. The workflow will automatically deploy on every push to main

### 3. Access Your Site
After deployment completes (2-3 minutes):
- **Live URL**: https://zubairxshah.github.io/physical-ai-book/

---

## ⚠️ Important: Backend Limitations

### What Works on GitHub Pages:
✅ Full Docusaurus book with all 12 chapters
✅ Beautiful homepage with navigation
✅ Responsive design and styling
✅ All chapter links and navigation

### What DOESN'T Work on GitHub Pages:
❌ **Chatbot widget** - GitHub Pages only hosts static files, cannot run Python backend
❌ **RAG functionality** - Requires FastAPI server running

---

## 🔧 Backend Deployment Options

### Option 1: Deploy Backend Separately (Recommended)

#### A. Deploy to Render.com (Free Tier)
1. Create account at https://render.com
2. Click **New** → **Web Service**
3. Connect your GitHub repo
4. Configure:
   - **Root Directory**: `backend`
   - **Build Command**: `pip install -r requirements_simple.txt`
   - **Start Command**: `uvicorn main_simple:app --host 0.0.0.0 --port $PORT`
5. Add environment variables:
   - `OPENAI_API_KEY`: (your key)
6. Deploy and get URL (e.g., `https://your-app.onrender.com`)

#### B. Deploy to Railway.app (Free Tier)
1. Create account at https://railway.app
2. Click **New Project** → **Deploy from GitHub**
3. Select your repo and `backend` folder
4. Add environment variables
5. Deploy and get URL

#### C. Deploy to Vercel (Serverless)
1. Create account at https://vercel.com
2. Install Vercel CLI: `npm i -g vercel`
3. In backend folder: `vercel`
4. Follow prompts

### Option 2: Update Widget to Use Deployed Backend

After deploying backend, update the API URL:

**File**: `static/chatbot-widget-v2.js`
```javascript
// Change this line:
const API_URL = 'http://localhost:8000';

// To your deployed URL:
const API_URL = 'https://your-backend-url.onrender.com';
```

Then rebuild and redeploy:
```bash
npm run build
git add .
git commit -m "Update backend URL"
git push
```

---

## 📝 Manual Deployment (Alternative)

If you prefer manual deployment:

```bash
# Build the site
npm run build

# Deploy using Docusaurus CLI
GIT_USER=zubairxshah npm run deploy
```

This pushes directly to `gh-pages` branch.

---

## 🧪 Test Locally Before Deploying

```bash
# Build production version
npm run build

# Serve locally to test
npm run serve
```

Visit http://localhost:3000/physical-ai-book/ to verify everything works.

---

## 🔍 Troubleshooting

### Deployment fails?
- Check GitHub Actions tab for error logs
- Ensure all dependencies are in package.json
- Verify baseUrl matches repo name

### 404 errors?
- Ensure `trailingSlash: false` in docusaurus.config.ts
- Check all links use `/chapterX` format (not `./chapterX`)

### Chatbot not working?
- Expected on GitHub Pages without backend
- Deploy backend separately (see options above)
- Update API_URL in chatbot-widget-v2.js

---

## 📊 What You'll Have After Deployment

### ✅ Working Features:
- Complete interactive book with 12 chapters
- Beautiful gradient homepage
- Chapter navigation (prev/next)
- Responsive mobile design
- Search functionality
- Dark mode toggle

### 🔄 Requires Backend Setup:
- AI chatbot responses
- RAG-based question answering
- Document ingestion

---

## 🎯 Recommended Setup for Full Functionality

1. **Frontend (GitHub Pages)**: Free, automatic
2. **Backend (Render.com)**: Free tier, 750 hours/month
3. **Total Cost**: $0

This gives you a fully functional deployed application!

---

## 📞 Support

If deployment fails:
1. Check GitHub Actions logs
2. Verify repository settings
3. Ensure all files are committed
4. Check docusaurus.config.ts settings match your repo

**Your deployment is configured and ready to go!** Just push to GitHub and it will automatically deploy. 🚀
