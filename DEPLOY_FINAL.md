# Final Deployment Guide

## Current Status
- ✅ GitHub repo ready
- ✅ Local build works
- ✅ Backend configured for Vercel

## Deploy Frontend on Vercel

### Step 1: Clear Old Deployment
1. Go to https://vercel.com/dashboard
2. Delete any existing `physical-ai-book` project
3. Start fresh

### Step 2: New Deployment
1. Click **Add New** → **Project**
2. Import `physical-ai-book` from GitHub
3. **IMPORTANT Settings**:
   - **Framework Preset**: Docusaurus
   - **Root Directory**: `./` (leave as root)
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
   - **Install Command**: `npm install --force`
4. Click **Deploy**
5. Wait 2-3 minutes

### Step 3: Verify
Visit your Vercel URL (e.g., `https://physical-ai-book.vercel.app`)
- Should show homepage with gradient design
- All chapters should work
- Images and styling should load

## Deploy Backend on Vercel

### Step 1: New Project for Backend
1. Go to https://vercel.com/new
2. Import `physical-ai-book` AGAIN (separate project)
3. **IMPORTANT Settings**:
   - **Project Name**: `physical-ai-book-backend`
   - **Root Directory**: `backend`
   - **Framework Preset**: Other
4. Add Environment Variable:
   - Key: `OPENAI_API_KEY`
   - Value: (your OpenAI API key)
5. Click **Deploy**

### Step 2: Get Backend URL
After deployment, copy the URL (e.g., `https://physical-ai-book-backend.vercel.app`)

### Step 3: Enable Chatbot
1. Uncomment chatbot script in `docusaurus.config.ts`:
```typescript
scripts: [
  {
    src: '/chatbot-widget-v2.js',
    async: true,
  },
],
```

2. Update `static/chatbot-widget-v2.js` line 6:
```javascript
const API_URL = 'https://physical-ai-book-backend.vercel.app';
```

3. Commit and push:
```bash
git add .
git commit -m "Enable chatbot with backend URL"
git push origin main
```

Vercel will auto-redeploy frontend with working chatbot.

## Troubleshooting

### Page looks broken on Vercel?
- Check Vercel build logs for errors
- Verify `baseUrl: '/'` in docusaurus.config.ts
- Clear browser cache (Ctrl+Shift+R)
- Redeploy from Vercel dashboard

### Chatbot not working?
- Check backend is deployed and running
- Verify OPENAI_API_KEY is set in backend env vars
- Check browser console for errors
- Test backend: `https://your-backend.vercel.app/health`

### Build fails?
- Ensure `npm install --force` in install command
- Check Node version is 18+ in Vercel settings
- Review build logs for specific errors

## Final URLs
- **Frontend**: https://physical-ai-book.vercel.app
- **Backend**: https://physical-ai-book-backend.vercel.app
- **GitHub**: https://github.com/zubairxshah/physical-ai-book

## Alternative: Use GitHub Pages Only
If Vercel doesn't work, stick with GitHub Pages:

1. Revert docusaurus.config.ts:
```typescript
url: 'https://zubairxshah.github.io',
baseUrl: '/physical-ai-book/',
```

2. Push to GitHub
3. Visit: https://zubairxshah.github.io/physical-ai-book/

GitHub Pages is more reliable but slower to update.
