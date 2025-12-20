"""
Personalization API for Physical AI Book
Serves contextual tooltips based on user's experience level
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv
from pathlib import Path
from sqlalchemy import create_engine, text
from sqlalchemy.orm import sessionmaker
import logging
import openai

# Load environment variables from .env.local in project root
env_path = Path(__file__).parent.parent / '.env.local'
load_dotenv(dotenv_path=env_path)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Physical AI Book - Personalization API",
    description="Context-aware tooltips for personalized learning",
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

# OpenAI setup for fallback tooltip generation
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if OPENAI_API_KEY:
    openai.api_key = OPENAI_API_KEY
    logger.info("OpenAI API configured for fallback tooltips")


# ============================================================================
# Request/Response Models
# ============================================================================

class UserProfile(BaseModel):
    programming_experience: str
    ros_familiarity: str
    ai_ml_background: str

class TooltipRequest(BaseModel):
    term: str
    chapter_id: str
    user_profile: UserProfile

class TooltipResponse(BaseModel):
    term: str
    definition: str
    experience_level: str
    source: str  # 'database' or 'fallback'


# ============================================================================
# Helper Functions
# ============================================================================

def get_experience_level(profile: UserProfile, term: str) -> str:
    """
    Determine appropriate experience level for tooltip based on term category.

    Logic:
    - ROS-specific terms → Use ros_familiarity
    - AI/ML terms → Use ai_ml_background
    - Programming terms → Use programming_experience
    - Default → beginner
    """
    term_lower = term.lower()

    # ROS-specific terms
    ros_keywords = ['ros', 'node', 'topic', 'service', 'action', 'launch', 'rqt', 'rviz',
                    'gazebo', 'navigation', 'tf', 'urdf', 'xacro', 'colcon', 'ament']
    if any(keyword in term_lower for keyword in ros_keywords):
        level = profile.ros_familiarity
        # Map ROS familiarity to experience level
        if level == 'none':
            return 'beginner'
        elif level == 'basic':
            return 'beginner'
        elif level == 'intermediate':
            return 'intermediate'
        elif level == 'expert':
            return 'advanced'
        return 'beginner'

    # AI/ML terms
    ai_keywords = ['neural', 'model', 'training', 'inference', 'tensor', 'transformer',
                   'cnn', 'rnn', 'lstm', 'gpt', 'bert', 'vla', 'reinforcement', 'supervised',
                   'unsupervised', 'deep learning', 'machine learning', 'backpropagation',
                   'gradient', 'loss function', 'optimizer', 'embedding', 'attention']
    if any(keyword in term_lower for keyword in ai_keywords):
        level = profile.ai_ml_background
        # Map AI/ML background to experience level
        if level in ['none', 'basic']:
            return 'beginner'
        elif level == 'intermediate':
            return 'intermediate'
        elif level == 'expert':
            return 'advanced'
        return 'beginner'

    # Programming terms
    prog_keywords = ['class', 'function', 'api', 'callback', 'async', 'await', 'promise',
                     'interface', 'struct', 'pointer', 'reference', 'thread', 'mutex',
                     'decorator', 'generator', 'lambda', 'closure', 'scope', 'namespace']
    if any(keyword in term_lower for keyword in prog_keywords):
        return profile.programming_experience

    # Default to beginner
    return 'beginner'


def get_tooltip_from_db(session, term: str, experience_level: str) -> Optional[dict]:
    """
    Query database for exact match tooltip.
    """
    try:
        query = text("""
            SELECT definition, experience_level
            FROM personalization_tooltips
            WHERE LOWER(term) = LOWER(:term)
            AND experience_level = :level
            LIMIT 1
        """)

        result = session.execute(
            query,
            {"term": term, "level": experience_level}
        ).fetchone()

        if result:
            return {
                "definition": result[0],
                "experience_level": result[1],
                "source": "database"
            }

        return None

    except Exception as e:
        logger.error(f"Database query error: {str(e)}")
        return None


def get_fallback_tooltip(session, term: str) -> Optional[dict]:
    """
    Get fallback tooltip (lowest experience level available).
    """
    try:
        query = text("""
            SELECT definition, experience_level
            FROM personalization_tooltips
            WHERE LOWER(term) = LOWER(:term)
            ORDER BY
                CASE experience_level
                    WHEN 'beginner' THEN 1
                    WHEN 'intermediate' THEN 2
                    WHEN 'advanced' THEN 3
                END
            LIMIT 1
        """)

        result = session.execute(query, {"term": term}).fetchone()

        if result:
            return {
                "definition": result[0],
                "experience_level": result[1],
                "source": "fallback"
            }

        return None

    except Exception as e:
        logger.error(f"Fallback query error: {str(e)}")
        return None


async def generate_ai_tooltip(term: str, experience_level: str) -> Optional[dict]:
    """
    Generate a tooltip using OpenAI when not found in database.
    """
    if not OPENAI_API_KEY:
        logger.warning("OpenAI API key not configured, cannot generate AI tooltip")
        return None

    try:
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

        client = openai.OpenAI(api_key=OPENAI_API_KEY)
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "Give ultra-brief definitions. Max 15 words."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=50,
            temperature=0.2
        )

        definition = response.choices[0].message.content.strip()

        # Ensure it's not too long
        if len(definition) > 150:
            definition = definition[:147] + "..."

        logger.info(f"Generated AI tooltip for '{term}' at level '{experience_level}'")

        return {
            "definition": definition,
            "experience_level": experience_level,
            "source": "ai_generated"
        }

    except Exception as e:
        logger.error(f"AI tooltip generation error: {str(e)}")
        return None


# ============================================================================
# API Endpoints
# ============================================================================

@app.post("/tooltips", response_model=TooltipResponse)
async def get_tooltip(request: TooltipRequest):
    """
    Returns personalized tooltip based on user's experience level.

    **Process:**
    1. Determine appropriate experience level based on term category
    2. Query database for matching tooltip
    3. Fallback to beginner level if exact match not found
    4. Return 404 if term not in database

    **Example Request:**
    ```json
    {
        "term": "ROS 2",
        "chapter_id": "chapter4",
        "user_profile": {
            "programming_experience": "intermediate",
            "ros_familiarity": "basic",
            "ai_ml_background": "none"
        }
    }
    ```

    **Example Response:**
    ```json
    {
        "term": "ROS 2",
        "definition": "Robot Operating System 2 - A beginner-friendly framework...",
        "experience_level": "beginner",
        "source": "database"
    }
    ```
    """
    session = SessionLocal()

    try:
        # Determine appropriate experience level
        experience_level = get_experience_level(request.user_profile, request.term)

        logger.info(f"Tooltip request: term='{request.term}', level='{experience_level}'")

        # Try exact match first
        tooltip = get_tooltip_from_db(session, request.term, experience_level)

        if tooltip:
            return TooltipResponse(
                term=request.term,
                definition=tooltip["definition"],
                experience_level=tooltip["experience_level"],
                source=tooltip["source"]
            )

        # Try fallback to any level
        tooltip = get_fallback_tooltip(session, request.term)

        if tooltip:
            logger.info(f"Using fallback tooltip for '{request.term}'")
            return TooltipResponse(
                term=request.term,
                definition=tooltip["definition"],
                experience_level=tooltip["experience_level"],
                source=tooltip["source"]
            )

        # Try AI generation as last resort
        tooltip = await generate_ai_tooltip(request.term, experience_level)

        if tooltip:
            return TooltipResponse(
                term=request.term,
                definition=tooltip["definition"],
                experience_level=tooltip["experience_level"],
                source=tooltip["source"]
            )

        # Term not found and AI generation failed
        logger.warning(f"Tooltip not found for term: '{request.term}'")
        raise HTTPException(
            status_code=404,
            detail=f"Tooltip not found for term: {request.term}"
        )

    except HTTPException:
        raise

    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Internal server error while fetching tooltip"
        )

    finally:
        session.close()


@app.get("/terms")
async def list_available_terms():
    """
    List all available terms in the tooltip database.

    Useful for frontend to know which terms have tooltips available.
    """
    session = SessionLocal()

    try:
        query = text("""
            SELECT DISTINCT term, COUNT(*) as level_count
            FROM personalization_tooltips
            GROUP BY term
            ORDER BY term
        """)

        results = session.execute(query).fetchall()

        terms = [
            {
                "term": row[0],
                "available_levels": row[1]
            }
            for row in results
        ]

        return {
            "total_terms": len(terms),
            "terms": terms
        }

    except Exception as e:
        logger.error(f"Error listing terms: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to list terms")

    finally:
        session.close()


@app.get("/stats")
async def get_stats():
    """
    Get statistics about tooltip database.
    """
    session = SessionLocal()

    try:
        query = text("""
            SELECT
                COUNT(DISTINCT term) as unique_terms,
                COUNT(*) as total_definitions,
                COUNT(CASE WHEN experience_level = 'beginner' THEN 1 END) as beginner_count,
                COUNT(CASE WHEN experience_level = 'intermediate' THEN 1 END) as intermediate_count,
                COUNT(CASE WHEN experience_level = 'advanced' THEN 1 END) as advanced_count
            FROM personalization_tooltips
        """)

        result = session.execute(query).fetchone()

        return {
            "unique_terms": result[0],
            "total_definitions": result[1],
            "by_level": {
                "beginner": result[2],
                "intermediate": result[3],
                "advanced": result[4]
            }
        }

    except Exception as e:
        logger.error(f"Error getting stats: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to get stats")

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

        return {
            "status": "healthy",
            "service": "personalization",
            "database": "connected"
        }

    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return {
            "status": "unhealthy",
            "service": "personalization",
            "database": "disconnected",
            "error": str(e)
        }


# ============================================================================
# Entry Point for Vercel Serverless
# ============================================================================

# For local development
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
