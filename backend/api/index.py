# Vercel Serverless Function Handler for RAG Chatbot
import sys
from pathlib import Path

# Add parent directory to path to import main_rag
sys.path.insert(0, str(Path(__file__).parent.parent))

from main_rag import app
from mangum import Mangum

# Create handler for Vercel (disable lifespan for serverless)
handler = Mangum(app, lifespan="off")
