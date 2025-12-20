---
title: Physical AI Book Backend
emoji: 🤖
colorFrom: purple
colorTo: blue
sdk: docker
pinned: false
license: mit
---

# Physical AI Book - Backend API

Combined backend API for the Physical AI and Humanoid Robotics book.

## Features

- **Chatbot** (`/query`) - AI-powered Q&A about Physical AI concepts
- **Personalization** (`/tooltips`) - Context-aware tooltips based on user experience
- **Translation** (`/translate`) - English to Urdu translation with caching

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Service info |
| `/health` | GET | Health check |
| `/query` | POST | Chat with AI about Physical AI |
| `/clear` | POST | Clear chat session |
| `/tooltips` | POST | Get personalized tooltip |
| `/terms` | GET | List available terms |
| `/translate` | POST | Translate to Urdu |
| `/translation/stats` | GET | Translation cache stats |

## Environment Variables

Set these in Space Settings > Secrets:

| Variable | Required | Description |
|----------|----------|-------------|
| `OPENROUTER_API_KEY` | Yes | OpenRouter API key for LLM |
| `DATABASE_URL` | No | PostgreSQL URL for caching |
| `OPENAI_API_KEY` | No | Fallback API key |

## Usage

### Chat Query
```bash
curl -X POST https://your-space.hf.space/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

### Get Tooltip
```bash
curl -X POST https://your-space.hf.space/tooltips \
  -H "Content-Type: application/json" \
  -d '{
    "term": "ROS 2",
    "chapter_id": "chapter4",
    "user_profile": {
      "programming_experience": "beginner",
      "ros_familiarity": "none",
      "ai_ml_background": "none"
    }
  }'
```

### Translate Content
```bash
curl -X POST https://your-space.hf.space/translate \
  -H "Content-Type: application/json" \
  -d '{
    "chapter_id": "chapter1",
    "content": "Physical AI combines artificial intelligence with robotics."
  }'
```

## Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Set environment variables
export OPENROUTER_API_KEY=your_key_here

# Run the server
python app.py
```

## License

MIT License - Part of the Physical AI Book project for GIAIC Hackathon.
