# Debug Checklist - Chatbot Not Showing

## Issue: Chatbot widget not appearing on http://localhost:3000/physical-ai-book/

### Quick Fixes to Try:

## Fix 1: Check Browser Console

1. Open the page: http://localhost:3000/physical-ai-book/
2. Press F12 to open Developer Tools
3. Go to "Console" tab
4. Look for errors related to `chatbot-widget.js`

**Common errors:**
- `404 Not Found: chatbot-widget.js` → Script path is wrong
- `Uncaught SyntaxError` → JavaScript error in widget
- `CORS error` → Backend not running or wrong URL

## Fix 2: Test Chatbot Widget Directly

1. Open: http://localhost:3000/physical-ai-book/test-chatbot.html
2. You should see the 💬 button
3. If it works here but not on main page, it's a Docusaurus config issue

## Fix 3: Verify File Locations

Check these files exist:
```
physical-ai-book/
├── static/
│   ├── chatbot-widget.js  ← Must be here
│   └── test-chatbot.html  ← Test file
└── docusaurus.config.ts   ← Check scripts section
```

## Fix 4: Restart Development Server

```bash
# Stop the server (Ctrl+C)
# Clear cache
npm run clear

# Start again
npm start
```

## Fix 5: Check Script Loading

In `docusaurus.config.ts`, verify:

```typescript
scripts: [
  {
    src: '/physical-ai-book/chatbot-widget.js',
    async: true,
  },
],
```

## Fix 6: Alternative - Add Script to HTML

If above doesn't work, manually add to `src/theme/Root.tsx`:

Create file: `src/theme/Root.tsx`

```tsx
import React, { useEffect } from 'react';

export default function Root({children}) {
  useEffect(() => {
    const script = document.createElement('script');
    script.src = '/physical-ai-book/chatbot-widget.js';
    script.async = true;
    document.body.appendChild(script);
    
    return () => {
      document.body.removeChild(script);
    };
  }, []);

  return <>{children}</>;
}
```

## Fix 7: Check Main Page Issue

The main page might be empty because:

1. **Docs as homepage**: With `routeBasePath: '/'`, the intro.md should show
2. **Check intro.md exists**: `docs/intro.md`
3. **Try direct URL**: http://localhost:3000/physical-ai-book/intro

## Fix 8: Backend Not Running

If chatbot appears but doesn't respond:

```bash
# Terminal 1: Start backend
cd backend
uvicorn main:app --reload --port 8000

# Terminal 2: Test backend
curl http://localhost:8000/health
```

Should return: `{"status":"ok"}`

## Fix 9: Ingest Content

If backend works but no answers:

```bash
curl -X POST http://localhost:8000/ingest
```

Should return: `{"message":"Ingested X chunks from Y documents"}`

## Fix 10: Update Qdrant URL

Your current .env has placeholder URL. Follow `SETUP_QDRANT.md` to:
1. Create Qdrant Cloud account
2. Get real cluster URL
3. Update `backend/.env`

## Complete Test Sequence

Run these in order:

```bash
# 1. Check Qdrant setup
cd backend
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print('OpenAI:', os.getenv('OPENAI_API_KEY')[:20]); print('Qdrant URL:', os.getenv('QDRANT_URL'))"

# 2. Start backend
uvicorn main:app --reload --port 8000

# 3. In new terminal, test health
curl http://localhost:8000/health

# 4. Ingest content
curl -X POST http://localhost:8000/ingest

# 5. Test query
curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d "{\"question\":\"What is Physical AI?\"}"

# 6. In new terminal, start frontend
cd ..
npm start

# 7. Open browser
# http://localhost:3000/physical-ai-book/
# http://localhost:3000/physical-ai-book/test-chatbot.html
```

## Expected Results

✅ Backend health check returns `{"status":"ok"}`  
✅ Ingest returns success message  
✅ Query returns an answer  
✅ Frontend loads at http://localhost:3000/physical-ai-book/  
✅ Chatbot 💬 button appears in bottom-right  
✅ Clicking button opens chat window  
✅ Typing question and pressing Send gets response  

## Still Not Working?

1. **Check browser console** (F12) for JavaScript errors
2. **Check backend terminal** for Python errors
3. **Verify all files** are in correct locations
4. **Try test page**: http://localhost:3000/physical-ai-book/test-chatbot.html
5. **Clear browser cache**: Ctrl+Shift+Delete
6. **Try different browser**: Chrome, Firefox, Edge

## Get Help

If still stuck, provide:
1. Browser console errors (screenshot)
2. Backend terminal output
3. Output of: `npm run clear && npm start`
4. Which step fails in the test sequence above
