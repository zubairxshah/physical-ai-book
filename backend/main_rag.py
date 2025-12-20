from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import sys
from dotenv import load_dotenv
from pathlib import Path
import httpx
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import psycopg2
from datetime import datetime
import uuid
import hashlib

# Fix Windows console encoding issues
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv()

app = FastAPI(title="Physical AI Book RAG Chatbot")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
OPENROUTER_MODEL = os.getenv("OPENROUTER_MODEL", "qwen/qwen-2.5-vl-7b-instruct:free")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
DATABASE_URL = os.getenv("DATABASE_URL")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")
EMBEDDING_DIMENSION = int(os.getenv("EMBEDDING_DIMENSION", 1536))
CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", 1000))
CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", 200))
TOP_K_RESULTS = int(os.getenv("TOP_K_RESULTS", 5))

# Initialize clients
openai_client = OpenAI(api_key=OPENAI_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Request/Response Models
class QueryRequest(BaseModel):
    question: str
    selected_text: Optional[str] = ""
    session_id: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[str] = []
    session_id: str

class Document(BaseModel):
    text: str
    source: str
    chunk_id: int

# Database functions
def get_db_connection():
    """Get PostgreSQL connection"""
    try:
        return psycopg2.connect(DATABASE_URL)
    except Exception as e:
        print(f"Database connection error: {e}")
        return None

def init_database():
    """Initialize Neon Postgres database schema"""
    conn = get_db_connection()
    if not conn:
        return False

    try:
        cursor = conn.cursor()

        # Create chat_history table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_history (
                id SERIAL PRIMARY KEY,
                session_id VARCHAR(255) NOT NULL,
                question TEXT NOT NULL,
                answer TEXT NOT NULL,
                selected_text TEXT,
                sources TEXT[],
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """)

        # Create index on session_id
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_session_id
            ON chat_history(session_id);
        """)

        # Create documents metadata table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS documents (
                id SERIAL PRIMARY KEY,
                filename VARCHAR(255) NOT NULL,
                chunk_count INTEGER,
                last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                content_hash VARCHAR(64) UNIQUE
            );
        """)

        conn.commit()
        cursor.close()
        conn.close()
        print("✅ Database initialized successfully")
        return True

    except Exception as e:
        print(f"❌ Database initialization error: {e}")
        if conn:
            conn.close()
        return False

def save_chat_history(session_id: str, question: str, answer: str,
                     selected_text: str, sources: List[str]):
    """Save chat interaction to Neon Postgres"""
    conn = get_db_connection()
    if not conn:
        return

    try:
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO chat_history
            (session_id, question, answer, selected_text, sources)
            VALUES (%s, %s, %s, %s, %s)
        """, (session_id, question, answer, selected_text or "", sources))

        conn.commit()
        cursor.close()
        conn.close()

    except Exception as e:
        print(f"Error saving chat history: {e}")
        if conn:
            conn.close()

# Qdrant functions
def init_qdrant_collection():
    """Initialize Qdrant collection"""
    try:
        collections = qdrant_client.get_collections().collections
        collection_names = [col.name for col in collections]

        if QDRANT_COLLECTION not in collection_names:
            qdrant_client.create_collection(
                collection_name=QDRANT_COLLECTION,
                vectors_config=VectorParams(
                    size=EMBEDDING_DIMENSION,
                    distance=Distance.COSINE
                )
            )
            print(f"✅ Created Qdrant collection: {QDRANT_COLLECTION}")
        else:
            print(f"✅ Qdrant collection exists: {QDRANT_COLLECTION}")

        return True

    except Exception as e:
        print(f"❌ Qdrant initialization error: {e}")
        return False

def chunk_text(text: str) -> List[str]:
    """Split text into overlapping chunks"""
    chunks = []
    start = 0
    text_length = len(text)

    while start < text_length:
        end = start + CHUNK_SIZE
        chunk = text[start:end]

        if chunk.strip():
            chunks.append(chunk.strip())

        start += CHUNK_SIZE - CHUNK_OVERLAP

    return chunks

def get_embedding(text: str) -> List[float]:
    """Get OpenAI embedding for text"""
    try:
        response = openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding
    except Exception as e:
        print(f"Embedding error: {e}")
        return None

def ingest_documents_to_qdrant():
    """Ingest all book chapters into Qdrant"""
    docs_path = Path(__file__).parent.parent / "docs"

    if not docs_path.exists():
        print(f"❌ Docs folder not found: {docs_path}")
        return False

    points = []
    point_id = 0

    # Get all markdown files
    md_files = list(docs_path.glob("*.md"))
    print(f"📚 Found {len(md_files)} documents")

    for md_file in md_files:
        if md_file.name.startswith("_"):
            continue

        print(f"Processing: {md_file.name}")

        try:
            content = md_file.read_text(encoding="utf-8")

            # Remove frontmatter
            if content.startswith("---"):
                parts = content.split("---", 2)
                if len(parts) >= 3:
                    content = parts[2].strip()

            # Create chunks
            chunks = chunk_text(content)
            print(f"  → Created {len(chunks)} chunks")

            # Create embeddings and points
            for i, chunk in enumerate(chunks):
                embedding = get_embedding(chunk)

                if embedding:
                    point = PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            "text": chunk,
                            "source": md_file.name,
                            "chunk_id": i,
                            "total_chunks": len(chunks)
                        }
                    )
                    points.append(point)
                    point_id += 1

        except Exception as e:
            print(f"  ❌ Error processing {md_file.name}: {e}")
            continue

    # Upload to Qdrant in batches
    print(f"📤 Uploading {len(points)} points to Qdrant...")

    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        try:
            qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION,
                points=batch
            )
            print(f"  Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")
        except Exception as e:
            print(f"  ❌ Upload error: {e}")

    print(f"✅ Ingestion complete! {len(points)} chunks uploaded")
    return True

def search_similar_chunks(query: str, top_k: int = TOP_K_RESULTS) -> List[dict]:
    """Search for similar chunks in Qdrant"""
    try:
        # Get query embedding
        query_embedding = get_embedding(query)

        if not query_embedding:
            return []

        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION,
            query_vector=query_embedding,
            limit=top_k
        )

        # Extract results
        chunks = []
        for result in search_results:
            chunks.append({
                "text": result.payload["text"],
                "source": result.payload["source"],
                "chunk_id": result.payload["chunk_id"],
                "score": result.score
            })

        return chunks

    except Exception as e:
        print(f"Search error: {e}")
        return []

async def query_openrouter(messages: List[dict]) -> str:
    """Query OpenRouter API with Qwen model"""
    url = "https://openrouter.ai/api/v1/chat/completions"

    headers = {
        "Authorization": f"Bearer {OPENROUTER_API_KEY}",
        "Content-Type": "application/json",
        "HTTP-Referer": "https://physical-ai-book.vercel.app",
        "X-Title": "Physical AI Book Chatbot"
    }

    data = {
        "model": OPENROUTER_MODEL,
        "messages": messages,
        "temperature": 0.7,
        "max_tokens": 1000
    }

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(url, json=data, headers=headers)
            response.raise_for_status()
            result = response.json()
            return result["choices"][0]["message"]["content"]

    except Exception as e:
        print(f"OpenRouter error: {e}")
        raise HTTPException(status_code=500, detail=f"OpenRouter API error: {str(e)}")

# API Endpoints
@app.on_event("startup")
async def startup_event():
    """Initialize on startup"""
    print("🚀 Starting Physical AI Book RAG Chatbot...")

    # Initialize databases
    db_ok = init_database()
    qdrant_ok = init_qdrant_collection()

    if not db_ok:
        print("⚠️ Warning: Neon Postgres not initialized")

    if not qdrant_ok:
        print("⚠️ Warning: Qdrant not initialized")

    # Check if collection is empty
    try:
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION)
        if collection_info.points_count == 0:
            print("📚 Collection is empty, starting ingestion...")
            ingest_documents_to_qdrant()
        else:
            print(f"✅ Collection has {collection_info.points_count} points")
    except Exception as e:
        print(f"Could not check collection: {e}")

@app.get("/")
async def root():
    return {
        "message": "Physical AI Book RAG Chatbot",
        "status": "running",
        "model": OPENROUTER_MODEL
    }

@app.get("/health")
async def health():
    """Health check endpoint"""
    try:
        # Check Qdrant
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION)
        points_count = collection_info.points_count

        # Check Postgres
        conn = get_db_connection()
        db_ok = conn is not None
        if conn:
            conn.close()

        return {
            "status": "healthy",
            "qdrant_points": points_count,
            "database_connected": db_ok,
            "model": OPENROUTER_MODEL
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }

@app.post("/ingest")
async def trigger_ingestion(background_tasks: BackgroundTasks):
    """Manually trigger document ingestion"""
    background_tasks.add_task(ingest_documents_to_qdrant)
    return {"message": "Ingestion started in background"}

@app.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    """
    Main RAG query endpoint
    Supports both regular questions and text selection queries
    """
    try:
        # Generate or use session ID
        session_id = request.session_id or str(uuid.uuid4())

        # Build search query
        if request.selected_text:
            # If text is selected, combine with question
            search_query = f"{request.selected_text}\n\n{request.question}"
        else:
            search_query = request.question

        # Search for relevant chunks
        relevant_chunks = search_similar_chunks(search_query, top_k=TOP_K_RESULTS)

        if not relevant_chunks:
            return QueryResponse(
                answer="I couldn't find relevant information in the book to answer your question.",
                sources=[],
                session_id=session_id
            )

        # Build context from chunks
        context_parts = []
        sources = []

        for i, chunk in enumerate(relevant_chunks, 1):
            context_parts.append(f"[Source {i}: {chunk['source']}]\n{chunk['text']}")
            if chunk['source'] not in sources:
                sources.append(chunk['source'])

        context = "\n\n---\n\n".join(context_parts)

        # Build prompt
        system_prompt = """You are an AI assistant for the "Physical AI and Humanoid Robotics" book.
Your role is to answer questions accurately based on the provided context from the book.

Instructions:
1. Answer based ONLY on the provided context
2. Be concise and clear
3. If the context doesn't contain the answer, say so
4. Reference sources when appropriate
5. If text was selected by the user, focus on explaining that specific text"""

        if request.selected_text:
            user_prompt = f"""The user selected this text from the book:
"{request.selected_text}"

User's question about the selected text: {request.question}

Relevant context from the book:
{context}

Please answer the user's question about the selected text, using the context provided."""
        else:
            user_prompt = f"""User question: {request.question}

Relevant context from the book:
{context}

Please answer based on the context above."""

        # Query OpenRouter
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        answer = await query_openrouter(messages)

        # Save to chat history
        save_chat_history(
            session_id=session_id,
            question=request.question,
            answer=answer,
            selected_text=request.selected_text or "",
            sources=sources
        )

        return QueryResponse(
            answer=answer,
            sources=sources,
            session_id=session_id
        )

    except Exception as e:
        print(f"Query error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/stats")
async def get_stats():
    """Get chatbot statistics"""
    try:
        # Qdrant stats
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION)

        # Postgres stats
        conn = get_db_connection()
        total_chats = 0

        if conn:
            cursor = conn.cursor()
            cursor.execute("SELECT COUNT(*) FROM chat_history")
            total_chats = cursor.fetchone()[0]
            cursor.close()
            conn.close()

        return {
            "total_documents": collection_info.points_count,
            "total_chats": total_chats,
            "model": OPENROUTER_MODEL,
            "collection": QDRANT_COLLECTION
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    # Use port 8003 to avoid conflict with auth server (8000)
    uvicorn.run(app, host="0.0.0.0", port=8003)
