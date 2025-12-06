from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from openai import OpenAI
import os
from pathlib import Path
import json

app = FastAPI()

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize OpenAI client
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# In-memory storage (simple alternative to Qdrant)
documents_store = []

class QueryRequest(BaseModel):
    question: str
    selected_text: str = ""

class QueryResponse(BaseModel):
    answer: str
    sources: list[str] = []

def chunk_text(text: str, chunk_size: int = 1000) -> list[str]:
    """Split text into chunks"""
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        if chunk.strip():
            chunks.append(chunk.strip())
        start += chunk_size - 200  # 200 char overlap
    return chunks

@app.on_event("startup")
async def startup_event():
    """Load documents on startup"""
    print("Starting up...")

@app.post("/ingest")
async def ingest_documents():
    """Ingest book chapters"""
    global documents_store
    docs_path = Path(__file__).parent.parent / "docs"
    
    if not docs_path.exists():
        raise HTTPException(status_code=404, detail="Docs folder not found")
    
    documents_store = []
    
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
            documents_store.append({
                "text": chunk,
                "source": md_file.name
            })
    
    return {"message": f"Ingested {len(documents_store)} chunks from documents"}

@app.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    """Query using simple keyword matching + GPT"""
    try:
        if not documents_store:
            # Try to ingest if empty
            await ingest_documents()
        
        # Simple keyword-based retrieval
        query_words = set(request.question.lower().split())
        
        # Score each document by keyword overlap
        scored_docs = []
        for doc in documents_store:
            doc_words = set(doc["text"].lower().split())
            score = len(query_words & doc_words)
            if score > 0:
                scored_docs.append((score, doc))
        
        # Get top 5 documents
        scored_docs.sort(reverse=True, key=lambda x: x[0])
        top_docs = [doc for _, doc in scored_docs[:5]]
        
        # Build context
        context = "\n\n".join([doc["text"] for doc in top_docs])
        sources = list(set([doc["source"] for doc in top_docs]))
        
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
    return {"status": "ok", "documents": len(documents_store)}
