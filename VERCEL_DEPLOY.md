# Deploy to Vercel - Quick Guide

## 🚀 Deploy Frontend (2 Minutes)

### Step 1: Sign Up & Import
1. Go to https://vercel.com/signup
2. Sign up with GitHub
3. Click **Add New** → **Project**
4. Import `physical-ai-book` repository
5. Click **Deploy**

That's it! Your site will be live at: `https://physical-ai-book.vercel.app`

### Step 2: Custom Domain (Optional)
1. Go to Project Settings → Domains
2. Add your custom domain
3. Follow DNS instructions

---

## 🤖 Deploy Backend (5 Minutes)

### Option A: Vercel Serverless (Recommended)

1. Create `backend/api/index.py`:
```python
from main_simple import app
```

2. Create `backend/vercel.json`:
```json
{
  "builds": [
    {
      "src": "api/index.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "api/index.py"
    }
  ]
}
```

3. Deploy backend separately:
```bash
cd backend
vercel
```

4. Add environment variables in Vercel dashboard:
   - `OPENAI_API_KEY`

5. Get backend URL (e.g., `https://backend-xyz.vercel.app`)

6. Update `static/chatbot-widget-v2.js`:
```javascript
const API_URL = 'https://backend-xyz.vercel.app';
```

### Option B: Render.com (Easier for FastAPI)

1. Go to https://render.com
2. New → Web Service
3. Connect GitHub repo
4. Root Directory: `backend`
5. Build: `pip install -r requirements_simple.txt`
6. Start: `uvicorn main_simple:app --host 0.0.0.0 --port $PORT`
7. Add env var: `OPENAI_API_KEY`
8. Deploy and copy URL

---

## ✅ Advantages of Vercel

✅ **Instant deployment** - Push to GitHub, auto-deploys
✅ **Free SSL** - Automatic HTTPS
✅ **Global CDN** - Fast worldwide
✅ **Preview deployments** - Every PR gets a preview URL
✅ **Zero config** - Works out of the box
✅ **Custom domains** - Free
✅ **Analytics** - Built-in

---

## 🔄 Update Workflow

After initial deployment, every push to GitHub automatically deploys:

```bash
git add .
git commit -m "Update content"
git push
```

Vercel automatically:
1. Detects the push
2. Builds your site
3. Deploys to production
4. Updates your live URL

---

## 📊 What You Get

### Frontend (Vercel):
- Live URL: `https://physical-ai-book.vercel.app`
- Auto SSL/HTTPS
- Global CDN
- Instant updates on push

### Backend (Render/Vercel):
- API endpoint for chatbot
- Environment variables secured
- Auto-scaling
- Free tier available

---

## 🎯 Quick Commands

```bash
# Install Vercel CLI (optional)
npm i -g vercel

# Deploy manually
vercel

# Deploy to production
vercel --prod
```

---

## 🔍 Troubleshooting

### Build fails?
- Check build logs in Vercel dashboard
- Ensure all dependencies in package.json
- Verify Node version (18+)

### 404 on routes?
- Check baseUrl is '/' in docusaurus.config.ts
- Verify vercel.json is present

### Chatbot not working?
- Deploy backend separately
- Update API_URL in chatbot-widget-v2.js
- Add OPENAI_API_KEY to backend env vars

---

## 🎉 You're Done!

Your site is now live with:
- ✅ Automatic deployments
- ✅ HTTPS/SSL
- ✅ Global CDN
- ✅ Preview URLs for PRs
- ✅ Zero maintenance

Just push to GitHub and Vercel handles the rest! 🚀
