"""
Simple Chatbot API using OpenRouter
No RAG, no Qdrant - just direct LLM chat about Physical AI
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import sys
from dotenv import load_dotenv
import httpx

# Fix Windows console encoding
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv(dotenv_path='.env.local')

app = FastAPI(title="Physical AI Chatbot - Simple")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
# Use OpenAI GPT-3.5 Turbo via OpenRouter (widely available)
OPENROUTER_MODEL = "openai/gpt-3.5-turbo"  # Reliable and cheap

# Store chat history per session (in-memory)
chat_sessions = {}

class QueryRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[str] = []
    session_id: str

@app.get("/")
async def root():
    return {
        "service": "Physical AI Chatbot",
        "status": "running",
        "model": OPENROUTER_MODEL,
        "features": ["chat", "context-aware"]
    }

@app.get("/health")
async def health():
    return {
        "status": "healthy",
        "service": "chatbot",
        "model": OPENROUTER_MODEL,
        "openrouter": "configured" if OPENROUTER_API_KEY else "missing"
    }

@app.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    """
    Answer questions about Physical AI using OpenRouter
    """

    if not OPENROUTER_API_KEY:
        raise HTTPException(status_code=500, detail="OpenRouter API key not configured")

    # Get or create session
    session_id = request.session_id or f"session_{os.urandom(8).hex()}"
    if session_id not in chat_sessions:
        chat_sessions[session_id] = []

    # Build context
    context = """You are an expert AI assistant for the "Physical AI and Humanoid Robotics" book.
You help readers understand concepts about:
- Physical AI and embodied intelligence
- Humanoid robotics
- ROS 2 (Robot Operating System)
- Computer vision and perception
- Control systems and locomotion
- AI/ML for robotics
- Sensor fusion
- Digital twins
- Vision-Language-Action (VLA) models

Provide clear, accurate, and helpful answers. If you don't know something, say so."""

    # Prepare messages
    messages = [
        {"role": "system", "content": context}
    ]

    # Add chat history (last 5 exchanges)
    for msg in chat_sessions[session_id][-10:]:
        messages.append(msg)

    # Add selected text if provided
    user_message = request.question
    if request.selected_text:
        user_message = f"Selected text: \"{request.selected_text}\"\n\nQuestion: {request.question}"

    messages.append({"role": "user", "content": user_message})

    try:
        # Call OpenRouter API
        async with httpx.AsyncClient(timeout=60.0) as client:
            response = await client.post(
                "https://openrouter.ai/api/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {OPENROUTER_API_KEY}",
                    "Content-Type": "application/json",
                    "HTTP-Referer": "http://localhost:3000",
                    "X-Title": "Physical AI Book Chatbot"
                },
                json={
                    "model": OPENROUTER_MODEL,
                    "messages": messages,
                    "temperature": 0.7,
                    "max_tokens": 500
                }
            )

            if response.status_code != 200:
                raise HTTPException(
                    status_code=response.status_code,
                    detail=f"OpenRouter API error: {response.text}"
                )

            data = response.json()
            answer = data['choices'][0]['message']['content']

            # Save to history
            chat_sessions[session_id].append({"role": "user", "content": user_message})
            chat_sessions[session_id].append({"role": "assistant", "content": answer})

            # Limit history size
            if len(chat_sessions[session_id]) > 20:
                chat_sessions[session_id] = chat_sessions[session_id][-20:]

            return QueryResponse(
                answer=answer,
                sources=[],  # No RAG sources in simple mode
                session_id=session_id
            )

    except httpx.HTTPError as e:
        raise HTTPException(status_code=500, detail=f"HTTP error: {str(e)}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error: {str(e)}")

@app.post("/clear")
async def clear_session(session_id: str):
    """Clear chat history for a session"""
    if session_id in chat_sessions:
        del chat_sessions[session_id]
    return {"success": True}

if __name__ == "__main__":
    import uvicorn
    print("=" * 70)
    print("Starting Physical AI Chatbot (Simple Mode - OpenRouter)")
    print("=" * 70)
    print(f"Server URL: http://localhost:8003")
    print(f"Model: {OPENROUTER_MODEL}")
    print(f"OpenRouter API: {'Configured' if OPENROUTER_API_KEY else 'Missing'}")
    print("=" * 70)
    uvicorn.run(app, host="0.0.0.0", port=8003)
