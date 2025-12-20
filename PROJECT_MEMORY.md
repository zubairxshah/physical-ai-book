# Project Memory - Physical AI & Humanoid Robotics Book

**Last Updated**: 2025-12-21 (Session 3 - Theme, Feedback & Deployment)
**Project Status**: Fully Deployed & Functional
**Current Phase**: Production Ready

---

## Project Overview

**Project Name**: Physical AI and Humanoid Robotics - Interactive Book with RAG Chatbot
**Purpose**: GIAIC AI/Spec-Driven Online Hackathon Submission
**Repository**: https://github.com/zubairxshah/physical-ai-book
**Live URL**: https://physical-ai-book-xi.vercel.app
**Backend URL**: https://engisoft-physical-ai-backend.hf.space

---

## Test Users

| Email | Password | Experience Level |
|-------|----------|------------------|
| `beginner@test.com` | `test123` | Beginner (no ROS, no AI) |
| `intermediate@test.com` | `test123` | Intermediate (basic ROS, basic AI) |
| `expert@test.com` | `test123` | Expert (expert ROS, expert AI) |

---

## Technology Stack

### Frontend
- **Framework**: Docusaurus 3.6.3
- **Language**: TypeScript, React 18
- **Styling**: Custom CSS with hero gradient theme (#667eea → #764ba2 → #f093fb)
- **Deployment**: Vercel (auto-deploy from GitHub)

### Backend (Hugging Face Spaces)
- **Framework**: FastAPI (Python)
- **Chat Model**: OpenRouter API with GPT-3.5-turbo
- **Database**: Neon Serverless PostgreSQL
- **Auth**: Custom session-based with bcrypt
- **Deployment**: Docker on HF Spaces

### Features
- AI Chatbot (OpenRouter)
- Personalized Tooltips (experience-based)
- Urdu Translation (with caching)
- User Feedback System
- Authentication (sign-in/sign-up)

---

## Project Structure

```
physical-ai-book/
├── docs/                    # Book chapters (MDX with tooltips)
│   ├── intro.md
│   ├── chapter1.mdx - chapter12.mdx
│   ├── module1-ros2.md
│   ├── module2-digital-twin.md
│   ├── module3-nvidia-isaac.md
│   └── module4-vla.md
├── src/
│   ├── components/
│   │   ├── Auth/           # SignIn/SignUp forms
│   │   ├── FloatingToolbar.tsx
│   │   ├── PersonalizedTooltip.tsx
│   │   └── UserDropdown.tsx
│   ├── context/
│   │   └── PersonalizationContext.tsx
│   ├── pages/
│   │   ├── index.tsx       # Hero page
│   │   ├── signin.tsx
│   │   ├── signup.tsx
│   │   └── feedback.tsx    # Feedback form
│   ├── css/custom.css      # Hero theme styling
│   └── config/api.ts       # API URLs
├── backend/                 # Local development servers
│   ├── personalization_api.py
│   ├── translation_api.py
│   └── chatbot_simple.py
├── huggingface-space/      # HF Space deployment
│   ├── app.py              # Combined API (all endpoints)
│   ├── Dockerfile
│   └── requirements.txt
├── static/
│   └── chatbot-widget-v3.js
├── docusaurus.config.ts
├── vercel.json
└── README.md
```

---

## API Endpoints (HF Space)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Service info |
| `/health` | GET | Health check |
| `/query` | POST | Chat with AI |
| `/tooltips` | POST | Get personalized tooltip |
| `/translate` | POST | Translate to Urdu |
| `/sign-up` | POST | Register user |
| `/sign-in` | POST | Login user |
| `/sign-out` | POST | Logout user |
| `/session` | GET | Get current session |
| `/feedback` | POST | Submit feedback |
| `/feedback` | GET | List all feedback |

---

## Database Tables

### users
```sql
CREATE TABLE users (
  id VARCHAR(64) PRIMARY KEY,
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  name VARCHAR(255),
  programming_experience VARCHAR(50),
  ros_familiarity VARCHAR(50),
  ai_ml_background VARCHAR(50),
  created_at TIMESTAMP DEFAULT NOW()
);
```

### feedback
```sql
CREATE TABLE feedback (
  id VARCHAR(64) PRIMARY KEY,
  user_id VARCHAR(64) NOT NULL,
  user_email VARCHAR(255) NOT NULL,
  feedback_type VARCHAR(50),
  message TEXT NOT NULL,
  rating INTEGER DEFAULT 5,
  created_at TIMESTAMP DEFAULT NOW()
);
```

### personalization_tooltips
```sql
CREATE TABLE personalization_tooltips (
  term VARCHAR(255),
  definition TEXT,
  experience_level VARCHAR(50)
);
```

### urdu_translations
```sql
CREATE TABLE urdu_translations (
  content_hash VARCHAR(64) PRIMARY KEY,
  chapter_id VARCHAR(50),
  original_text TEXT,
  translated_text TEXT,
  tokens_used INTEGER,
  model_used VARCHAR(50),
  access_count INTEGER DEFAULT 1,
  last_accessed TIMESTAMP,
  created_at TIMESTAMP DEFAULT NOW()
);
```

---

## Recent Changes (Session 3 - 2025-12-21)

### Theme Updates
- Applied hero gradient colors to entire site
- Purple/pink gradient navbar (#667eea → #764ba2)
- Styled sidebar, buttons, scrollbars
- Dark mode support
- Gradient chapter headings

### Navigation Changes
- Removed "Start Reading" from navbar
- Added "Feedback" link to navbar

### Feedback Feature
- Created /feedback page
- Only authenticated users can submit
- Feedback types: suggestion, bug, content, feature, other
- Star rating (1-5)
- Stored in PostgreSQL

### Deployment Fixes
- Fixed Vercel 404 errors for signin/signup pages
- Updated HF Space URL to Engisoft
- Removed Python serverless functions (using HF Space instead)
- Simplified vercel.json

### Authentication
- Added auth endpoints to HF Space backend
- Session-based authentication with cookies
- Password hashing with bcrypt

---

## Environment Variables

### Vercel
- `NODE_ENV=production`

### HF Space Secrets
- `OPENROUTER_API_KEY`
- `DATABASE_URL`
- `BETTER_AUTH_SECRET`
- `BETTER_AUTH_URL`

---

## Local Development

```bash
# Start all 4 servers
# Terminal 1 - Frontend (port 3000)
npm start

# Terminal 2 - Auth Server (port 8000)
node auth-server.js

# Terminal 3 - Personalization API (port 8001)
python backend/personalization_api.py

# Terminal 4 - Translation/Chatbot API (port 8002/8003)
python backend/chatbot_simple.py
```

---

## Deployment

### Frontend (Vercel)
- Auto-deploys from GitHub main branch
- URL: https://physical-ai-book-xi.vercel.app

### Backend (Hugging Face Spaces)
- Docker-based deployment
- Upload files from `huggingface-space/` folder
- URL: https://engisoft-physical-ai-backend.hf.space

---

## Contact

**Developer**: Zubair Shah
**Email**: engisoft@yahoo.com
**GitHub**: @zubairxshah
**Course**: GIAIC Thursday Evening AI
**Hackathon**: AI/Spec-Driven Online Hackathon 1
**Date**: December 2024

---

**End of Memory File**
*Last updated: 2025-12-21*
