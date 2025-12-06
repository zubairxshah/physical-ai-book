from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import OpenAI
import os
from pathlib import Path
import hashlib

app = FastAPI()

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = "physical_ai_book"

class QueryRequest(BaseModel):
    question: str
    selected_text: str = ""

class QueryResponse(BaseModel):
    answer: str
    sources: list[str] = []

def get_embedding(text: str) -> list[float]:
    """Get embedding from OpenAI"""
    response = openai_client.embeddings.create(
        model="text-embedding-3-small",
        input=text
    )
    return response.data[0].embedding

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> list[str]:
    """Split text into overlapping chunks"""
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        if chunk.strip():
            chunks.append(chunk.strip())
        start += chunk_size - overlap
    return chunks

@app.on_event("startup")
async def startup_event():
    """Initialize Qdrant collection"""
    try:
        collections = qdrant_client.get_collections().collections
        if not any(c.name == COLLECTION_NAME for c in collections):
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
            )
            print(f"Created collection: {COLLECTION_NAME}")
    except Exception as e:
        print(f"Error initializing Qdrant: {e}")

@app.post("/ingest")
async def ingest_documents():
    """Ingest book chapters into Qdrant"""
    docs_path = Path(__file__).parent.parent / "docs"
    
    if not docs_path.exists():
        raise HTTPException(status_code=404, detail="Docs folder not found")
    
    points = []
    point_id = 0
    
    for md_file in docs_path.glob("*.md"):
        if md_file.name.startswith("_"):
            continue
            
        content = md_file.read_text(encoding="utf-8")
        
        # Remove frontmatter
        if content.startswith("---"):
            parts = content.split("---", 2)
            if len(parts) >= 3:
                content = parts[2].strip()
        
        chunks = chunk_text(content)
        
        for chunk in chunks:
            embedding = get_embedding(chunk)
            points.append(
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "text": chunk,
                        "source": md_file.name
                    }
                )
            )
            point_id += 1
    
    if points:
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
    
    return {"message": f"Ingested {len(points)} chunks from {point_id} documents"}

@app.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    """Query the RAG system"""
    try:
        # Build query context
        query_text = request.question
        if request.selected_text:
            query_text = f"Based on this text: '{request.selected_text}'\n\nQuestion: {request.question}"
        
        # Get embedding for query
        query_embedding = get_embedding(query_text)
        
        # Search Qdrant
        search_results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=5
        )
        
        # Build context from results
        context_parts = []
        sources = []
        for result in search_results:
            context_parts.append(result.payload["text"])
            source = result.payload["source"]
            if source not in sources:
                sources.append(source)
        
        context = "\n\n".join(context_parts)
        
        # Build prompt
        system_prompt = """You are an AI assistant for the Physical AI and Humanoid Robotics book. 
Answer questions based on the provided context. Be concise and accurate.
If the context doesn't contain the answer, say so."""
        
        user_prompt = f"""Context from the book:
{context}

Question: {request.question}"""
        
        if request.selected_text:
            user_prompt = f"""Selected text: {request.selected_text}

Context from the book:
{context}

Question: {request.question}"""
        
        # Get answer from OpenAI
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=500
        )
        
        answer = response.choices[0].message.content
        
        return QueryResponse(answer=answer, sources=sources)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health():
    return {"status": "ok"}
