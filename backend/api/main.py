from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from openai import OpenAI
import os

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

class QueryRequest(BaseModel):
    question: str
    selected_text: str = ""

class QueryResponse(BaseModel):
    answer: str
    sources: list[str] = []

@app.get("/")
async def root():
    return {"message": "Physical AI Book Backend API", "status": "running"}

@app.get("/health")
async def health():
    return {"status": "ok"}

@app.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    try:
        system_prompt = """You are an AI assistant for the Physical AI and Humanoid Robotics book. 
Answer questions about physical AI, robotics, and related topics. Be concise and helpful."""
        
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.question}
            ],
            temperature=0.7,
            max_tokens=500
        )
        
        answer = response.choices[0].message.content
        
        return QueryResponse(answer=answer, sources=["General Knowledge"])
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
