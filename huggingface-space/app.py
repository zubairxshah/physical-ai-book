"""
Physical AI Book - Combined Backend API for Hugging Face Spaces
Combines: Chatbot, Personalization, and Translation APIs
Runs on port 7860 (HF Space default)
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import sys
import hashlib
import logging
import httpx
from dotenv import load_dotenv

# Fix Windows console encoding
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ============================================================================
# FastAPI App Setup
# ============================================================================

app = FastAPI(
    title="Physical AI Book - Backend API",
    description="Combined API for Chatbot, Personalization, and Translation",
    version="1.0.0"
)

# CORS - Allow all origins for HF Space
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# Environment Variables
# ============================================================================

OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
DATABASE_URL = os.getenv("DATABASE_URL")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# ============================================================================
# Database Setup (Optional - for caching)
# ============================================================================

db_engine = None
SessionLocal = None

if DATABASE_URL:
    try:
        from sqlalchemy import create_engine, text
        from sqlalchemy.orm import sessionmaker
        db_engine = create_engine(DATABASE_URL)
        SessionLocal = sessionmaker(bind=db_engine)
        logger.info("Database connected successfully")
    except Exception as e:
        logger.warning(f"Database connection failed: {e}. Running without caching.")

# ============================================================================
# Request/Response Models
# ============================================================================

# Chatbot Models
class ChatRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[str] = []
    session_id: str

# Personalization Models
class UserProfile(BaseModel):
    programming_experience: str = "beginner"
    ros_familiarity: str = "none"
    ai_ml_background: str = "none"

class TooltipRequest(BaseModel):
    term: str
    chapter_id: str = "unknown"
    user_profile: UserProfile = UserProfile()

class TooltipResponse(BaseModel):
    term: str
    definition: str
    experience_level: str
    source: str

# Translation Models
class TranslationRequest(BaseModel):
    chapter_id: str
    content: str
    max_length: int = 50000

class TranslationResponse(BaseModel):
    chapter_id: str
    translated_content: str
    cached: bool
    tokens_used: int
    model_used: str

# ============================================================================
# Chat Sessions Storage
# ============================================================================

chat_sessions = {}

# ============================================================================
# Helper Functions
# ============================================================================

def get_experience_level(profile: UserProfile, term: str) -> str:
    """Determine experience level based on term and user profile."""
    term_lower = term.lower()

    ros_keywords = ['ros', 'node', 'topic', 'service', 'action', 'launch', 'rqt', 'rviz',
                    'gazebo', 'navigation', 'tf', 'urdf', 'xacro', 'colcon', 'ament']
    if any(keyword in term_lower for keyword in ros_keywords):
        level = profile.ros_familiarity
        if level in ['none', 'basic']:
            return 'beginner'
        elif level == 'intermediate':
            return 'intermediate'
        return 'advanced'

    ai_keywords = ['neural', 'model', 'training', 'inference', 'tensor', 'transformer',
                   'cnn', 'rnn', 'lstm', 'gpt', 'bert', 'vla', 'reinforcement']
    if any(keyword in term_lower for keyword in ai_keywords):
        level = profile.ai_ml_background
        if level in ['none', 'basic']:
            return 'beginner'
        elif level == 'intermediate':
            return 'intermediate'
        return 'advanced'

    return 'beginner'


async def generate_ai_tooltip(term: str, experience_level: str) -> Optional[dict]:
    """Generate tooltip using OpenRouter."""
    api_key = OPENROUTER_API_KEY or OPENAI_API_KEY
    if not api_key:
        return None

    level_descriptions = {
        "beginner": "a complete beginner using very simple words",
        "intermediate": "someone with basic technical knowledge",
        "advanced": "a technical expert"
    }
    level_desc = level_descriptions.get(experience_level, level_descriptions["beginner"])

    prompt = f"""Define "{term}" for {level_desc} in robotics context.
Rules:
- Maximum 1-2 SHORT sentences (under 100 characters total)
- No jargon for beginners
- Do not repeat the term
- Be direct and concise"""

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(
                "https://openrouter.ai/api/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": "openai/gpt-3.5-turbo",
                    "messages": [
                        {"role": "system", "content": "Give ultra-brief definitions. Max 15 words."},
                        {"role": "user", "content": prompt}
                    ],
                    "max_tokens": 50,
                    "temperature": 0.2
                }
            )

            if response.status_code == 200:
                data = response.json()
                definition = data['choices'][0]['message']['content'].strip()
                if len(definition) > 150:
                    definition = definition[:147] + "..."
                return {
                    "definition": definition,
                    "experience_level": experience_level,
                    "source": "ai_generated"
                }
    except Exception as e:
        logger.error(f"AI tooltip error: {e}")

    return None


async def translate_text(text: str) -> tuple[str, int]:
    """Translate text to Urdu using OpenRouter."""
    api_key = OPENROUTER_API_KEY or OPENAI_API_KEY
    if not api_key:
        raise HTTPException(status_code=500, detail="No API key configured")

    try:
        async with httpx.AsyncClient(timeout=60.0) as client:
            response = await client.post(
                "https://openrouter.ai/api/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": "openai/gpt-3.5-turbo",
                    "messages": [
                        {
                            "role": "system",
                            "content": """Translate English to Urdu for technical robotics content.
Keep technical terms in English (ROS 2, API, Python, etc).
Keep code blocks unchanged. Use formal Urdu."""
                        },
                        {"role": "user", "content": text}
                    ],
                    "max_tokens": 4000,
                    "temperature": 0.3
                }
            )

            if response.status_code == 200:
                data = response.json()
                translated = data['choices'][0]['message']['content']
                tokens = data.get('usage', {}).get('total_tokens', 0)
                return translated, tokens
            else:
                raise HTTPException(status_code=500, detail="Translation API error")
    except httpx.HTTPError as e:
        raise HTTPException(status_code=500, detail=f"HTTP error: {e}")


# ============================================================================
# API Endpoints - Root
# ============================================================================

@app.get("/")
async def root():
    return {
        "service": "Physical AI Book Backend",
        "status": "running",
        "endpoints": {
            "chatbot": "/query",
            "personalization": "/tooltips",
            "translation": "/translate",
            "health": "/health"
        }
    }


@app.get("/health")
async def health():
    return {
        "status": "healthy",
        "services": {
            "chatbot": "active",
            "personalization": "active",
            "translation": "active"
        },
        "database": "connected" if db_engine else "not_configured",
        "openrouter": "configured" if OPENROUTER_API_KEY else "not_configured"
    }


# ============================================================================
# API Endpoints - Chatbot
# ============================================================================

@app.post("/query", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """Chat endpoint for Physical AI questions."""
    api_key = OPENROUTER_API_KEY or OPENAI_API_KEY
    if not api_key:
        raise HTTPException(status_code=500, detail="API key not configured")

    session_id = request.session_id or f"session_{os.urandom(8).hex()}"
    if session_id not in chat_sessions:
        chat_sessions[session_id] = []

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

Provide clear, accurate, and helpful answers."""

    messages = [{"role": "system", "content": context}]
    for msg in chat_sessions[session_id][-10:]:
        messages.append(msg)

    user_message = request.question
    if request.selected_text:
        user_message = f"Selected text: \"{request.selected_text}\"\n\nQuestion: {request.question}"

    messages.append({"role": "user", "content": user_message})

    try:
        async with httpx.AsyncClient(timeout=60.0) as client:
            response = await client.post(
                "https://openrouter.ai/api/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {api_key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": "openai/gpt-3.5-turbo",
                    "messages": messages,
                    "temperature": 0.7,
                    "max_tokens": 500
                }
            )

            if response.status_code != 200:
                raise HTTPException(status_code=500, detail="Chat API error")

            data = response.json()
            answer = data['choices'][0]['message']['content']

            chat_sessions[session_id].append({"role": "user", "content": user_message})
            chat_sessions[session_id].append({"role": "assistant", "content": answer})

            if len(chat_sessions[session_id]) > 20:
                chat_sessions[session_id] = chat_sessions[session_id][-20:]

            return ChatResponse(answer=answer, sources=[], session_id=session_id)

    except httpx.HTTPError as e:
        raise HTTPException(status_code=500, detail=f"HTTP error: {e}")


@app.post("/clear")
async def clear_chat(session_id: str):
    """Clear chat session."""
    if session_id in chat_sessions:
        del chat_sessions[session_id]
    return {"success": True}


# ============================================================================
# API Endpoints - Personalization
# ============================================================================

@app.post("/tooltips", response_model=TooltipResponse)
async def get_tooltip(request: TooltipRequest):
    """Get personalized tooltip for a term."""
    experience_level = get_experience_level(request.user_profile, request.term)

    # Try database first if available
    if SessionLocal:
        try:
            session = SessionLocal()
            from sqlalchemy import text
            query = text("""
                SELECT definition, experience_level
                FROM personalization_tooltips
                WHERE LOWER(term) = LOWER(:term)
                AND experience_level = :level
                LIMIT 1
            """)
            result = session.execute(query, {"term": request.term, "level": experience_level}).fetchone()
            session.close()

            if result:
                return TooltipResponse(
                    term=request.term,
                    definition=result[0],
                    experience_level=result[1],
                    source="database"
                )
        except Exception as e:
            logger.warning(f"Database query failed: {e}")

    # Fall back to AI generation
    tooltip = await generate_ai_tooltip(request.term, experience_level)
    if tooltip:
        return TooltipResponse(
            term=request.term,
            definition=tooltip["definition"],
            experience_level=tooltip["experience_level"],
            source=tooltip["source"]
        )

    raise HTTPException(status_code=404, detail=f"Tooltip not found for: {request.term}")


@app.get("/terms")
async def list_terms():
    """List available tooltip terms."""
    if not SessionLocal:
        return {"total_terms": 0, "terms": [], "note": "Database not configured"}

    try:
        session = SessionLocal()
        from sqlalchemy import text
        query = text("SELECT DISTINCT term FROM personalization_tooltips ORDER BY term")
        results = session.execute(query).fetchall()
        session.close()

        terms = [row[0] for row in results]
        return {"total_terms": len(terms), "terms": terms}
    except Exception as e:
        logger.error(f"Error listing terms: {e}")
        return {"total_terms": 0, "terms": [], "error": str(e)}


# ============================================================================
# API Endpoints - Translation
# ============================================================================

@app.post("/translate", response_model=TranslationResponse)
async def translate_content(request: TranslationRequest):
    """Translate content to Urdu."""
    if len(request.content) > request.max_length:
        raise HTTPException(status_code=400, detail="Content too long")

    content_hash = hashlib.sha256(request.content.encode()).hexdigest()

    # Check cache if database available
    if SessionLocal:
        try:
            session = SessionLocal()
            from sqlalchemy import text
            query = text("""
                SELECT translated_text, tokens_used, model_used
                FROM urdu_translations
                WHERE content_hash = :hash
            """)
            result = session.execute(query, {"hash": content_hash}).fetchone()

            if result:
                # Update access count
                update_query = text("""
                    UPDATE urdu_translations
                    SET last_accessed = NOW(), access_count = access_count + 1
                    WHERE content_hash = :hash
                """)
                session.execute(update_query, {"hash": content_hash})
                session.commit()
                session.close()

                return TranslationResponse(
                    chapter_id=request.chapter_id,
                    translated_content=result[0],
                    cached=True,
                    tokens_used=result[1],
                    model_used=result[2]
                )
            session.close()
        except Exception as e:
            logger.warning(f"Cache lookup failed: {e}")

    # Perform translation
    translated_text, tokens_used = await translate_text(request.content)

    # Cache result if database available
    if SessionLocal:
        try:
            session = SessionLocal()
            from sqlalchemy import text
            query = text("""
                INSERT INTO urdu_translations
                (content_hash, chapter_id, original_text, translated_text, tokens_used, model_used)
                VALUES (:hash, :chapter_id, :original, :translated, :tokens, :model)
                ON CONFLICT (content_hash) DO UPDATE
                SET last_accessed = NOW(), access_count = urdu_translations.access_count + 1
            """)
            session.execute(query, {
                "hash": content_hash,
                "chapter_id": request.chapter_id,
                "original": request.content[:5000],
                "translated": translated_text,
                "tokens": tokens_used,
                "model": "openai/gpt-3.5-turbo"
            })
            session.commit()
            session.close()
        except Exception as e:
            logger.warning(f"Cache store failed: {e}")

    return TranslationResponse(
        chapter_id=request.chapter_id,
        translated_content=translated_text,
        cached=False,
        tokens_used=tokens_used,
        model_used="openai/gpt-3.5-turbo"
    )


@app.get("/translation/stats")
async def translation_stats():
    """Get translation cache statistics."""
    if not SessionLocal:
        return {"note": "Database not configured", "stats": None}

    try:
        session = SessionLocal()
        from sqlalchemy import text
        query = text("""
            SELECT COUNT(*), SUM(tokens_used), SUM(access_count)
            FROM urdu_translations
        """)
        result = session.execute(query).fetchone()
        session.close()

        return {
            "total_translations": result[0] or 0,
            "total_tokens": result[1] or 0,
            "total_accesses": result[2] or 0
        }
    except Exception as e:
        return {"error": str(e)}


# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 7860))
    print("=" * 70)
    print("Physical AI Book - Combined Backend API")
    print("=" * 70)
    print(f"Server URL: http://0.0.0.0:{port}")
    print(f"OpenRouter: {'Configured' if OPENROUTER_API_KEY else 'Not configured'}")
    print(f"Database: {'Connected' if db_engine else 'Not configured'}")
    print("=" * 70)
    uvicorn.run(app, host="0.0.0.0", port=port)
