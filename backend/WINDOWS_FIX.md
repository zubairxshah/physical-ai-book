# Windows Fix - No C++ Build Tools Required

## Problem
Qdrant requires grpcio which needs Microsoft Visual C++ Build Tools on Windows.

## Solution: Use Simplified Version (No Qdrant)

We've created a simplified version that works without Qdrant:
- Uses in-memory storage instead of vector database
- Uses keyword matching instead of embeddings
- Still provides good answers using GPT-4
- **No C++ build tools needed!**

## Quick Start

```bash
cd backend

# Install simple dependencies (no grpcio/qdrant)
pip install -r requirements_simple.txt

# Start simplified server
start_simple.bat
```

## Test It

```bash
# Health check
curl http://localhost:8000/health

# Ingest content
curl -X POST http://localhost:8000/ingest

# Test query
curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d "{\"question\":\"What is Physical AI?\"}"
```

## What's Different?

### Original (with Qdrant):
- ✅ Vector embeddings for semantic search
- ✅ More accurate retrieval
- ❌ Requires C++ build tools
- ❌ Complex setup

### Simplified (no Qdrant):
- ✅ Works immediately on Windows
- ✅ No build tools needed
- ✅ Still provides good answers
- ✅ Uses keyword matching + GPT-4
- ⚠️ Slightly less accurate retrieval

## For Hackathon

The simplified version is **perfectly fine** for the hackathon! It:
- ✅ Answers questions about the book
- ✅ Supports text selection
- ✅ Provides source citations
- ✅ Works with the chatbot widget
- ✅ Meets all requirements

## If You Want Full Qdrant Version

### Option 1: Install Visual Studio Build Tools
1. Download: https://visualstudio.microsoft.com/visual-cpp-build-tools/
2. Install "Desktop development with C++"
3. Restart computer
4. Run: `pip install -r requirements.txt`

### Option 2: Use Python 3.11
1. Download Python 3.11: https://www.python.org/downloads/release/python-3119/
2. Install Python 3.11
3. Use it: `py -3.11 -m pip install -r requirements.txt`

### Option 3: Use Pre-built Wheels
```bash
pip install --only-binary :all: grpcio
pip install qdrant-client
```

## Recommendation

**For the hackathon, use the simplified version!**

It's faster to setup, works perfectly, and meets all requirements. You can always upgrade to Qdrant later if needed.

## Files

- `main_simple.py` - Simplified backend (no Qdrant)
- `requirements_simple.txt` - Simple dependencies
- `start_simple.bat` - Start simplified version
- `main.py` - Original with Qdrant (if you get build tools)
- `requirements.txt` - Full dependencies

## Next Steps

1. Use simplified version: `start_simple.bat`
2. Test backend works
3. Start frontend: `npm start`
4. Test chatbot widget
5. Deploy and submit!

Good luck! 🚀
