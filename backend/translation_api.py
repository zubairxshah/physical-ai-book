"""
Translation API for Physical AI Book
Translates content to Urdu with intelligent caching
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
import hashlib
from dotenv import load_dotenv
from pathlib import Path
from openai import OpenAI
from sqlalchemy import create_engine, text
from sqlalchemy.orm import sessionmaker
import logging
from datetime import datetime

# Load environment variables from .env.local in project root
env_path = Path(__file__).parent.parent / '.env.local'
load_dotenv(dotenv_path=env_path)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Physical AI Book - Translation API",
    description="Cached Urdu translation service",
    version="1.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://physical-ai-book.vercel.app"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL")
if not DATABASE_URL:
    logger.error("DATABASE_URL environment variable not set")
    raise ValueError("DATABASE_URL must be set")

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(bind=engine)

# OpenRouter setup (use instead of OpenAI for better availability)
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
if not OPENROUTER_API_KEY:
    logger.error("OPENROUTER_API_KEY environment variable not set")
    raise ValueError("OPENROUTER_API_KEY must be set")

client = OpenAI(
    api_key=OPENROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1"
)


# ============================================================================
# Request/Response Models
# ============================================================================

class TranslationRequest(BaseModel):
    chapter_id: str
    content: str
    max_length: int = 50000  # Increased limit for full chapters

class TranslationResponse(BaseModel):
    chapter_id: str
    translated_content: str
    cached: bool
    tokens_used: int
    model_used: str


# ============================================================================
# Helper Functions
# ============================================================================

def compute_content_hash(content: str) -> str:
    """
    Generate SHA256 hash for content caching.

    Why hashing?
    - Fast lookup in database
    - Automatic deduplication (same content = same hash)
    - Protects against cache poisoning
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def get_cached_translation(session, content_hash: str) -> dict | None:
    """
    Check if translation exists in cache.

    Returns dict with translation data or None if not found.
    """
    try:
        query = text("""
            SELECT translated_text, tokens_used, model_used
            FROM urdu_translations
            WHERE content_hash = :hash
        """)

        result = session.execute(query, {"hash": content_hash}).fetchone()

        if result:
            logger.info(f"Cache HIT for hash {content_hash[:16]}...")
            return {
                "translated_text": result[0],
                "tokens_used": result[1],
                "model_used": result[2]
            }

        logger.info(f"Cache MISS for hash {content_hash[:16]}...")
        return None

    except Exception as e:
        logger.error(f"Cache lookup error: {str(e)}")
        return None


def update_cache_access(session, content_hash: str):
    """
    Update access statistics for cached translation.
    """
    try:
        query = text("""
            UPDATE urdu_translations
            SET last_accessed = NOW(),
                access_count = access_count + 1
            WHERE content_hash = :hash
        """)
        session.execute(query, {"hash": content_hash})
        session.commit()

    except Exception as e:
        logger.error(f"Failed to update cache access: {str(e)}")
        session.rollback()


def translate_with_openai(text: str) -> tuple[str, int]:
    """
    Translate English text to Urdu using OpenAI GPT-4 Turbo.

    Returns:
        (translated_text, tokens_used)

    Translation Guidelines:
    - Preserve technical terms in English (ROS 2, API, Python)
    - Maintain markdown formatting
    - Keep code blocks untranslated
    - Use formal Urdu for technical content
    """
    try:
        response = client.chat.completions.create(
            model="openai/gpt-3.5-turbo",  # Use GPT-3.5 via OpenRouter
            messages=[
                {
                    "role": "system",
                    "content": """You are a professional translator specializing in technical documentation translation from English to Urdu.

Translation Guidelines:
1. Preserve technical terms in English when appropriate:
   - Keep: ROS 2, API, Python, Node, Topic, Service, Action, Docker, Kubernetes, GPU, CPU
   - Translate: General concepts, explanations, instructions

2. Maintain markdown formatting exactly:
   - Keep headers (# ## ###)
   - Keep code blocks (```)
   - Keep links [text](url)
   - Keep lists and tables

3. Code blocks:
   - DO NOT translate code
   - DO NOT translate code comments (keep them in English)
   - Preserve all code syntax exactly

4. Style:
   - Use formal Urdu suitable for technical education
   - Maintain clarity and readability
   - Keep sentence structure when possible

5. Numbers and symbols:
   - Keep Arabic numerals (1, 2, 3)
   - Keep mathematical symbols (+, -, ×, ÷)
   - Keep punctuation

Translate the following English text about robotics and AI into fluent, accurate Urdu:"""
                },
                {
                    "role": "user",
                    "content": text
                }
            ],
            temperature=0.3,  # Lower temperature for more consistent translations
            max_tokens=4000,
            top_p=0.9
        )

        translated_text = response.choices[0].message.content
        tokens_used = response.usage.total_tokens

        logger.info(f"Translation completed: {tokens_used} tokens used")

        return translated_text, tokens_used

    except Exception as e:
        logger.error(f"OpenAI translation error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {str(e)}"
        )


def store_translation(session, content_hash: str, chapter_id: str,
                      original_text: str, translated_text: str,
                      tokens_used: int, model_used: str):
    """
    Store translation in cache database.
    """
    try:
        # Store only first 5000 chars of original for reference
        original_preview = original_text[:5000] if len(original_text) > 5000 else original_text

        query = text("""
            INSERT INTO urdu_translations
            (content_hash, chapter_id, original_text, translated_text, tokens_used, model_used)
            VALUES (:hash, :chapter_id, :original, :translated, :tokens, :model)
            ON CONFLICT (content_hash) DO UPDATE
            SET last_accessed = NOW(),
                access_count = urdu_translations.access_count + 1
        """)

        session.execute(query, {
            "hash": content_hash,
            "chapter_id": chapter_id,
            "original": original_preview,
            "translated": translated_text,
            "tokens": tokens_used,
            "model": model_used
        })
        session.commit()

        logger.info(f"Translation cached for chapter {chapter_id}")

    except Exception as e:
        logger.error(f"Failed to cache translation: {str(e)}")
        session.rollback()
        # Don't raise - cache failure shouldn't block translation


# ============================================================================
# API Endpoints
# ============================================================================

@app.post("/translate", response_model=TranslationResponse)
async def translate_content(request: TranslationRequest):
    """
    Translates chapter content to Urdu with intelligent caching.

    **Process:**
    1. Compute content hash
    2. Check cache for existing translation
    3. If cache hit: Return cached translation (instant)
    4. If cache miss: Call OpenAI GPT-4 Turbo
    5. Store new translation in cache
    6. Return translated content

    **Example Request:**
    ```json
    {
        "chapter_id": "chapter4",
        "content": "# Chapter 4: Humanoid Robots\\n\\nHumanoid robots..."
    }
    ```

    **Example Response (Cache Hit):**
    ```json
    {
        "chapter_id": "chapter4",
        "translated_content": "# باب 4: انسانی روبوٹس...",
        "cached": true,
        "tokens_used": 2847,
        "model_used": "openai/gpt-3.5-turbo"
    }
    ```

    **Cost Savings:**
    - First request: ~$0.03 (3000 tokens)
    - Cached requests: $0.00 (instant)
    - 95% cache hit rate = 95% cost reduction
    """
    # Validate content length
    if len(request.content) > request.max_length:
        raise HTTPException(
            status_code=400,
            detail=f"Content too long. Maximum {request.max_length} characters allowed."
        )

    session = SessionLocal()

    try:
        content_hash = compute_content_hash(request.content)

        # Check cache first
        cached_translation = get_cached_translation(session, content_hash)

        if cached_translation:
            # Update access statistics
            update_cache_access(session, content_hash)

            return TranslationResponse(
                chapter_id=request.chapter_id,
                translated_content=cached_translation["translated_text"],
                cached=True,
                tokens_used=cached_translation["tokens_used"],
                model_used=cached_translation["model_used"]
            )

        # Cache miss - perform translation
        logger.info(f"Translating chapter {request.chapter_id} ({len(request.content)} chars)")

        translated_text, tokens_used = translate_with_openai(request.content)

        # Store in cache for future requests
        store_translation(
            session=session,
            content_hash=content_hash,
            chapter_id=request.chapter_id,
            original_text=request.content,
            translated_text=translated_text,
            tokens_used=tokens_used,
            model_used="gpt-4-turbo"
        )

        return TranslationResponse(
            chapter_id=request.chapter_id,
            translated_content=translated_text,
            cached=False,
            tokens_used=tokens_used,
            model_used="gpt-4-turbo"
        )

    except HTTPException:
        raise

    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Internal server error during translation"
        )

    finally:
        session.close()


@app.get("/stats")
async def get_translation_stats():
    """
    Returns cache statistics and cost savings.

    **Metrics:**
    - Total cached translations
    - Total tokens saved
    - Total cache accesses
    - Average cache hits per content
    - Estimated cost savings (USD)
    """
    session = SessionLocal()

    try:
        query = text("""
            SELECT
                COUNT(*) as total_translations,
                SUM(tokens_used) as total_tokens,
                SUM(access_count) as total_accesses,
                AVG(access_count) as avg_cache_hits,
                MIN(created_at) as first_translation,
                MAX(last_accessed) as last_access
            FROM urdu_translations
        """)

        result = session.execute(query).fetchone()

        if result and result[0] > 0:
            total_translations = result[0]
            total_tokens = result[1] or 0
            total_accesses = result[2] or 0
            avg_cache_hits = float(result[3] or 0)
            first_translation = result[4]
            last_access = result[5]

            # Cost calculation (GPT-4 Turbo pricing)
            # Input: $0.01 per 1K tokens, Output: $0.03 per 1K tokens
            # Assume 60% input, 40% output (typical for translation)
            cost_per_token = (0.01 * 0.6 + 0.03 * 0.4) / 1000  # ~$0.018 per 1K tokens

            tokens_without_cache = total_tokens * total_accesses
            tokens_with_cache = total_tokens  # Only paid once per unique content
            tokens_saved = tokens_without_cache - tokens_with_cache

            cost_without_cache = tokens_without_cache * cost_per_token
            cost_with_cache = tokens_with_cache * cost_per_token
            cost_savings = cost_without_cache - cost_with_cache

            cache_hit_rate = ((total_accesses - total_translations) / total_accesses * 100) if total_accesses > 0 else 0

            return {
                "cache_stats": {
                    "total_cached_translations": total_translations,
                    "total_cache_accesses": total_accesses,
                    "average_cache_hits_per_content": round(avg_cache_hits, 2),
                    "cache_hit_rate_percent": round(cache_hit_rate, 2)
                },
                "token_stats": {
                    "total_tokens_used_for_translations": total_tokens,
                    "total_tokens_served": tokens_without_cache,
                    "tokens_saved_by_caching": tokens_saved
                },
                "cost_savings": {
                    "cost_without_cache_usd": round(cost_without_cache, 2),
                    "cost_with_cache_usd": round(cost_with_cache, 2),
                    "total_savings_usd": round(cost_savings, 2),
                    "savings_percentage": round((cost_savings / cost_without_cache * 100) if cost_without_cache > 0 else 0, 2)
                },
                "timeline": {
                    "first_translation": str(first_translation) if first_translation else None,
                    "last_access": str(last_access) if last_access else None
                }
            }

        else:
            return {
                "cache_stats": {
                    "total_cached_translations": 0,
                    "total_cache_accesses": 0,
                    "average_cache_hits_per_content": 0,
                    "cache_hit_rate_percent": 0
                },
                "token_stats": {
                    "total_tokens_used_for_translations": 0,
                    "total_tokens_served": 0,
                    "tokens_saved_by_caching": 0
                },
                "cost_savings": {
                    "cost_without_cache_usd": 0,
                    "cost_with_cache_usd": 0,
                    "total_savings_usd": 0,
                    "savings_percentage": 0
                },
                "timeline": {
                    "first_translation": None,
                    "last_access": None
                }
            }

    except Exception as e:
        logger.error(f"Error getting stats: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to get stats")

    finally:
        session.close()


@app.get("/chapters")
async def list_translated_chapters():
    """
    List all chapters with cached translations.
    """
    session = SessionLocal()

    try:
        query = text("""
            SELECT
                chapter_id,
                COUNT(*) as translation_count,
                SUM(access_count) as total_accesses,
                MAX(last_accessed) as last_accessed
            FROM urdu_translations
            GROUP BY chapter_id
            ORDER BY chapter_id
        """)

        results = session.execute(query).fetchall()

        chapters = [
            {
                "chapter_id": row[0],
                "cached_translations": row[1],
                "total_accesses": row[2],
                "last_accessed": str(row[3]) if row[3] else None
            }
            for row in results
        ]

        return {
            "total_chapters": len(chapters),
            "chapters": chapters
        }

    except Exception as e:
        logger.error(f"Error listing chapters: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to list chapters")

    finally:
        session.close()


@app.get("/health")
async def health_check():
    """
    Health check endpoint for monitoring.
    """
    try:
        # Test database connection
        session = SessionLocal()
        session.execute(text("SELECT 1"))
        session.close()

        # Test OpenAI API key is set
        openai_configured = bool(OPENAI_API_KEY)

        return {
            "status": "healthy",
            "service": "translation",
            "database": "connected",
            "openai": "configured" if openai_configured else "not_configured"
        }

    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return {
            "status": "unhealthy",
            "service": "translation",
            "database": "disconnected",
            "error": str(e)
        }


# ============================================================================
# Entry Point for Vercel Serverless
# ============================================================================

# For local development
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8002)
