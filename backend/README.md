# Physical AI Book - RAG Chatbot Backend

This is the FastAPI backend for the RAG (Retrieval-Augmented Generation) chatbot integrated into the Physical AI book.

## Features

- **RAG System**: Uses Qdrant vector database for semantic search
- **OpenAI Integration**: GPT-4o-mini for intelligent responses
- **Text Selection Support**: Answer questions about selected text
- **CORS Enabled**: Works with GitHub Pages deployment

## Setup Instructions

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create a `.env` file:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key-here
```

**Get Free Qdrant Cloud:**
1. Go to https://cloud.qdrant.io/
2. Sign up for free tier
3. Create a cluster
4. Copy URL and API key

### 3. Run the Server

```bash
uvicorn main:app --reload --port 8000
```

### 4. Ingest Book Content

```bash
curl -X POST http://localhost:8000/ingest
```

This will:
- Read all markdown files from `docs/` folder
- Split them into chunks
- Generate embeddings using OpenAI
- Store in Qdrant vector database

### 5. Test the API

```bash
# Health check
curl http://localhost:8000/health

# Query example
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'

# Query with selected text
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"question": "Explain this", "selected_text": "Physical AI combines perception, cognition, and action"}'
```

## Deployment

### Deploy to Render.com (Free)

1. Create account at https://render.com
2. Create new Web Service
3. Connect your GitHub repo
4. Configure:
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables in Render dashboard
6. Deploy!

### Update Frontend

After deployment, update `static/chatbot-widget.js`:

```javascript
const API_URL = 'https://your-app.onrender.com';
```

## API Endpoints

### POST /ingest
Ingest book content into vector database.

**Response:**
```json
{
  "message": "Ingested 150 chunks from 13 documents"
}
```

### POST /query
Query the RAG system.

**Request:**
```json
{
  "question": "What is Physical AI?",
  "selected_text": "optional selected text"
}
```

**Response:**
```json
{
  "answer": "Physical AI represents...",
  "sources": ["chapter1.md", "intro.md"]
}
```

### GET /health
Health check endpoint.

## Architecture

```
User Question
    ↓
OpenAI Embeddings (text-embedding-3-small)
    ↓
Qdrant Vector Search (top 5 results)
    ↓
Context + Question → GPT-4o-mini
    ↓
Answer + Sources
```

## Troubleshooting

**Error: Collection not found**
- Run `/ingest` endpoint first

**Error: OpenAI API key invalid**
- Check your `.env` file
- Verify key at https://platform.openai.com/api-keys

**Error: Qdrant connection failed**
- Verify Qdrant URL and API key
- Check if cluster is running

**CORS errors**
- Backend should allow all origins for GitHub Pages
- Check CORS middleware configuration

## Cost Estimation

**Free Tier Usage:**
- Qdrant Cloud: 1GB free (sufficient for book)
- OpenAI: Pay per use
  - Embeddings: ~$0.02 per 1M tokens
  - GPT-4o-mini: ~$0.15 per 1M input tokens
  - Estimated: <$1 for full book ingestion + 1000 queries

## Development

Run with auto-reload:
```bash
uvicorn main:app --reload --port 8000
```

View API docs:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc
