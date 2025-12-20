# Physical AI and Humanoid Robotics
### Interactive AI-Powered Book with RAG Chatbot

🤖 **Live Book**: [https://physical-ai-book-xi.vercel.app](https://physical-ai-book-xi.vercel.app)
🔗 **API Backend**: [https://huggingface.co/spaces/Engisoft/physical-ai-backend](https://huggingface.co/spaces/Engisoft/physical-ai-backend)
📹 **Demo Video**: [Your video link]

---

## 🔐 Test Users

Use these accounts to test the application:

| Email | Password | Experience Level |
|-------|----------|------------------|
| `beginner@test.com` | `test123` | Beginner (no ROS, no AI) |
| `intermediate@test.com` | `test123` | Intermediate (basic ROS, basic AI) |
| `expert@test.com` | `test123` | Expert (expert ROS, expert AI) |

**Features to test:**
- Sign in at `/signin`
- Personalized tooltips (hover over technical terms in chapters)
- Urdu translation (toggle in Reading Tools)
- Submit feedback at `/feedback`
- AI Chatbot (bottom-right corner)

---

## 🎯 Project Overview

This project was built for the GIAIC AI/Spec-Driven Online Hackathon. It demonstrates the future of interactive documentation by combining:

- **AI-Generated Content**: Complete book written using Claude Code and Spec-Kit Plus
- **Modern Documentation**: Built with Docusaurus 3.6
- **Intelligent RAG Chatbot**: Answers questions about book content using OpenRouter
- **Text Selection Queries**: Users can highlight text and ask specific questions
- **Personalization**: Experience-based tooltip definitions
- **Translation**: Urdu translation with caching

---

## 🚀 Features

### Core Features
- **Complete Book** - 12 chapters on Physical AI and Humanoid Robotics
- **RAG Chatbot** - AI-powered Q&A using OpenRouter
- **Authentication System** - User registration with experience profiles
- **Content Personalization** - Tooltips adapt to user's experience level
- **Urdu Translation** - GPT-powered translation with intelligent caching
- **Feedback System** - Registered users can submit feedback
- **Hero Theme** - Beautiful purple/pink gradient design
- **Responsive Design** - Works on desktop and mobile

### Technology Stack
- **Frontend**: Docusaurus 3.6, React, TypeScript
- **Backend**: FastAPI (Python) on Hugging Face Spaces
- **Auth**: Custom auth with PostgreSQL (Neon), Bcrypt
- **AI/ML**: OpenRouter API (GPT-3.5/4)
- **Database**: Neon Serverless PostgreSQL
- **Deployment**: Vercel (frontend), Hugging Face Spaces (backend)

---

## 🛠️ Quick Start

### Prerequisites
- Node.js 18+
- Python 3.11+
- OpenRouter API key

### Local Development

```bash
# Clone repository
git clone https://github.com/zubairxshah/physical-ai-book.git
cd physical-ai-book

# Install dependencies
npm install

# Create .env.local with your keys (see .env.example)

# Start all servers (4 terminals needed)
# Terminal 1 - Frontend
npm start

# Terminal 2 - Auth Server
node auth-server.js

# Terminal 3 - Personalization API
python backend/personalization_api.py

# Terminal 4 - Translation API
python backend/translation_api.py
```

Or use the batch script:
```bash
# Windows
start-all-servers.bat

# Linux/Mac
./start-all-servers.sh
```

---

## 📚 Book Contents

| Part | Chapters |
|------|----------|
| **Foundations** | 1. Introduction to Physical AI<br>2. Core Technologies<br>3. AI Models for Physical Systems |
| **Humanoid Robotics** | 4. Rise of Humanoid Robots<br>5. Mechanical Design<br>6. Control Systems |
| **Intelligence** | 7. Perception Systems<br>8. Learning and Adaptation<br>9. Natural Language AI |
| **Applications** | 10. Real-World Applications<br>11. Challenges<br>12. Future of Physical AI |

### Technical Modules
- ROS 2 Deep Dive
- Digital Twins (Gazebo & Isaac Sim)
- Vision-Language-Action Models

---

## 🤖 How the Chatbot Works

1. **User Query**: Type a question or select text and ask
2. **Context Building**: System adds Physical AI domain knowledge
3. **AI Response**: OpenRouter generates contextual answers
4. **Session Memory**: Chat history maintained per session

### Example Queries
- "What is Physical AI?"
- "How do humanoid robots maintain balance?"
- *[Select text]* "Explain this in simpler terms"
- "What are VLA models?"

---

## 📁 Project Structure

```
physical-ai-book/
├── docs/                    # Book chapters (MDX with tooltips)
├── src/
│   ├── components/          # React components
│   │   ├── Auth/           # SignIn/SignUp forms
│   │   ├── FloatingToolbar.tsx
│   │   └── PersonalizedTooltip.tsx
│   ├── context/            # React context (Personalization)
│   ├── pages/              # Custom pages (feedback, signin, signup)
│   └── css/custom.css      # Hero theme styling
├── backend/                 # Local development servers
├── huggingface-space/      # HF Space deployment files
│   ├── app.py              # Combined API
│   ├── Dockerfile
│   └── requirements.txt
├── static/
│   └── chatbot-widget-v3.js
└── docusaurus.config.ts
```

---

## 🚀 Deployment

### Frontend (Vercel)
- Auto-deploys from GitHub `main` branch
- URL: https://physical-ai-book-xi.vercel.app

### Backend (Hugging Face Spaces)
- Docker-based deployment
- URL: https://engisoft-physical-ai-backend.hf.space

### Environment Variables

**Vercel:**
- `NODE_ENV=production`

**HF Space Secrets:**
- `OPENROUTER_API_KEY`
- `DATABASE_URL`
- `BETTER_AUTH_SECRET`

---

## 🏆 Features Implemented

### Authentication
- User registration with experience profiles
- Session-based authentication
- Protected feedback submission

### Personalization
- Tooltips adapt to user's:
  - Programming experience
  - ROS familiarity
  - AI/ML background

### Translation
- English to Urdu translation
- Intelligent caching (saves API costs)
- Preserves technical terms

### Feedback System
- Only registered users can submit
- Multiple feedback types (bug, suggestion, feature, etc.)
- Star rating system
- Stored in PostgreSQL

---

## 👥 Credits

**Developer**: Zubair Shah
**Course**: GIAIC Thursday Evening AI
**Hackathon**: AI/Spec-Driven Online Hackathon 1
**Date**: December 2024

**Technologies Used**:
- Claude Code (AI pair programmer)
- Docusaurus (documentation framework)
- OpenRouter (LLM API gateway)
- Hugging Face Spaces (backend hosting)
- Neon (PostgreSQL database)
- Vercel (frontend hosting)

---

## 📝 License

MIT License - feel free to use this code for learning and projects!

---

## 📧 Contact

For questions or feedback:
- GitHub: [@zubairxshah](https://github.com/zubairxshah)
- Email: engisoft@yahoo.com

---

**Built with ❤️ for GIAIC Hackathon**
