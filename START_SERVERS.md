# 🚀 How to Start the Project

**IMPORTANT:** This project requires MULTIPLE servers running simultaneously!

---

## ⚠️ Critical: Run ALL Servers Together

This project has **4 separate servers** that must ALL be running:

1. **Frontend** (Docusaurus) - Port 3000
2. **Auth Server** (Node.js) - Port 8000
3. **Personalization API** (Python) - Port 8001
4. **Translation API** (Python) - Port 8002

**If you only run `npm start`, auth/personalization/translation won't work!**

---

## 🔧 Prerequisites

### 1. Install Dependencies

```bash
# Node.js dependencies
npm install

# Python dependencies
pip install -r backend/requirements_rag.txt
```

### 2. Configure Environment Variables

```bash
# Copy example file
cp .env.example .env.local

# Edit .env.local and fill in:
# - DATABASE_URL (PostgreSQL from Neon)
# - OPENAI_API_KEY
# - QDRANT_URL and QDRANT_API_KEY (if using RAG)
```

### 3. Setup Database

```bash
# Connect to your PostgreSQL database and run:
psql $DATABASE_URL -f scripts/setup_auth_db.sql
```

---

## 🚀 Starting All Servers

You need **4 separate terminal windows/tabs**.

### Terminal 1: Frontend (Docusaurus)
```bash
npm start
```
**Runs on:** http://localhost:3000
**Status:** ✅ Should start successfully

---

### Terminal 2: Auth Server (Node.js)
```bash
node auth-server.js
```
**Runs on:** http://localhost:8000
**Provides:**
- `/api/auth/sign-up` - User registration
- `/api/auth/sign-in` - User login
- `/api/auth/session` - Session check
- `/api/auth/sign-out` - Logout

**Status:** ⚠️ Requires working PostgreSQL database

**Common Errors:**
- `Connection timeout` → Database not accessible
- `EADDRINUSE` → Port 8000 already in use

---

### Terminal 3: Personalization API (Python)
```bash
python backend/personalization_api.py
```
**Runs on:** http://localhost:8001
**Provides:**
- `/tooltips` - Get personalized tooltip definitions
- `/terms` - List all available terms
- `/health` - Health check

**Status:** ⚠️ Requires working PostgreSQL database

---

### Terminal 4: Translation API (Python)
```bash
python backend/translation_api.py
```
**Runs on:** http://localhost:8002
**Provides:**
- `/translate` - Translate content to Urdu
- `/stats` - Cache statistics
- `/health` - Health check

**Status:** ⚠️ Requires working PostgreSQL database

---

## ✅ Verify All Servers Are Running

Open these URLs in your browser:

1. ✅ **Frontend:** http://localhost:3000
2. ⚠️ **Auth:** http://localhost:8000/api/auth/health
3. ⚠️ **Personalization:** http://localhost:8001/health
4. ⚠️ **Translation:** http://localhost:8002/health

All should return `200 OK` status.

---

## 🧪 Testing the Complete System

### 1. Test Authentication
```bash
# Visit signup page
http://localhost:3000/signup

# Create account with:
# - Name: Test User
# - Email: test@example.com
# - Password: password123
# - Fill background questionnaire

# Should auto-login and redirect to chapter4
```

### 2. Test Personalization
```bash
# After login, visit chapter4
http://localhost:3000/docs/chapter4

# Click "Personalize" button
# Hover over "AI" or "Foundation models" text
# Should see tooltip with definition based on your experience level
```

### 3. Test Translation
```bash
# After login, visit chapter4
http://localhost:3000/docs/chapter4

# Click "اردو (Urdu)" button
# Wait 3-8 seconds for first translation
# Content should change to Urdu (right-to-left)
# Second time should be instant (cached)
```

---

## 🐛 Troubleshooting

### Error: "Failed to load resource: 404" on `/api/auth/*`

**Problem:** Auth server (port 8000) is not running

**Solution:**
```bash
# In separate terminal:
node auth-server.js

# Check it's running:
curl http://localhost:8000/api/auth/health
```

---

### Error: "Connection timeout" when starting servers

**Problem:** PostgreSQL database not accessible

**Solutions:**
1. Check database is active in Neon dashboard
2. Verify `DATABASE_URL` in `.env.local`
3. Test connection:
   ```bash
   psql $DATABASE_URL -c "SELECT 1"
   ```
4. Check firewall/network settings

---

### Error: "Port already in use"

**Problem:** Port is occupied by another process

**Solution:**
```bash
# Windows - Find and kill process on port 8000:
netstat -ano | findstr :8000
taskkill /PID <PID_NUMBER> /F

# Or use different port:
PORT=8001 node auth-server.js
```

---

### ProtectedContent not blocking chapters

**Problem:** Can access all chapters without login

**Current Status:** ⚠️ Known issue - ProtectedContent component exists but doesn't enforce blocking

**Workaround:** Requires fixing `src/components/ProtectedContent.tsx` logic

---

## 📋 Quick Start Checklist

Before starting development:

- [ ] PostgreSQL database is accessible
- [ ] `.env.local` file configured with all keys
- [ ] Node.js dependencies installed (`npm install`)
- [ ] Python dependencies installed (`pip install -r backend/requirements_rag.txt`)
- [ ] Database schema applied (`psql $DATABASE_URL -f scripts/setup_auth_db.sql`)

To start working:

- [ ] Terminal 1: `npm start` (Frontend)
- [ ] Terminal 2: `node auth-server.js` (Auth)
- [ ] Terminal 3: `python backend/personalization_api.py` (Personalization)
- [ ] Terminal 4: `python backend/translation_api.py` (Translation)
- [ ] Verify all health checks return 200 OK

---

## 🎯 Common Workflows

### Just Testing Frontend (No Auth)
```bash
# Only need this:
npm start

# Visit: http://localhost:3000
# Note: Auth/personalization/translation won't work
```

### Testing Auth Features
```bash
# Need both:
npm start              # Terminal 1
node auth-server.js    # Terminal 2

# Visit: http://localhost:3000/signup
```

### Testing Full System (All Features)
```bash
# Need all four:
npm start                                      # Terminal 1
node auth-server.js                            # Terminal 2
python backend/personalization_api.py          # Terminal 3
python backend/translation_api.py              # Terminal 4
```

---

## 🔐 Security Notes

- **Never commit `.env.local`** - Contains secrets
- Database credentials are in `.env.local`
- Sessions expire after 7 days
- Passwords are hashed with bcrypt (10 rounds)
- All API endpoints use CORS with credentials

---

## 📊 Server Architecture

```
┌─────────────────────────────────────────────┐
│  Browser (localhost:3000)                   │
│  - Docusaurus frontend                      │
│  - React components                         │
└─────────────────────────────────────────────┘
                    │
                    │ Webpack proxy: /api/* → localhost:8000
                    ▼
┌─────────────────────────────────────────────┐
│  Auth Server (localhost:8000)               │
│  - Express.js                               │
│  - Better Auth                              │
│  - Session management                       │
└─────────────────────────────────────────────┘
                    │
                    ├─────────────────┬───────────────────┐
                    ▼                 ▼                   ▼
┌────────────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ PostgreSQL (Neon)      │  │ Personalization  │  │ Translation API  │
│ - User accounts        │  │ API (port 8001)  │  │ (port 8002)      │
│ - Sessions             │  │ - FastAPI        │  │ - FastAPI        │
│ - Tooltip definitions  │  │ - Tooltips       │  │ - GPT-4 Turbo    │
│ - Translation cache    │  │                  │  │ - Caching        │
└────────────────────────┘  └──────────────────┘  └──────────────────┘
```

---

## 📁 Key Files

### Server Files
- `auth-server.js` - Main authentication server (Express)
- `backend/personalization_api.py` - Personalization service
- `backend/translation_api.py` - Translation service
- `backend/main_rag.py` - RAG chatbot backend (separate)

### Configuration
- `.env.local` - Environment variables (not in git)
- `.env.example` - Template for environment variables
- `docusaurus.config.ts` - Frontend config with proxy settings
- `vercel.json` - Deployment configuration

### Frontend Components
- `src/components/Auth/` - Signup/signin forms
- `src/components/ChapterControls.tsx` - Personalization/translation toggles
- `src/components/EnhancedChapter.tsx` - Chapter wrapper with features
- `src/components/ProtectedContent.tsx` - Access control wrapper
- `src/hooks/useAuth.ts` - Global auth state

---

## 💡 Pro Tips

1. **Use tmux/screen** for managing multiple terminals
2. **Create a startup script** to launch all servers:
   ```bash
   #!/bin/bash
   npm start &
   node auth-server.js &
   python backend/personalization_api.py &
   python backend/translation_api.py &
   wait
   ```
3. **Check logs** if something fails - each server prints to its terminal
4. **Test health endpoints** before testing features
5. **Keep database connection string handy** for quick debugging

---

## 🆘 Still Having Issues?

Check these files for detailed information:
- `PROJECT_STATUS_2025-12-20.md` - Current implementation status
- `AUTH_IMPLEMENTATION_COMPLETE.md` - Auth system documentation
- `READY_FOR_TESTING.md` - Testing procedures
- `SETUP_RAG_CHATBOT.md` - RAG setup instructions

---

**Last Updated:** 2025-12-20
**Status:** ⚠️ Database connection blocking full functionality
**Working Features:** Frontend, Skills & Subagents
**Blocked Features:** Auth, Personalization, Translation (need database)
