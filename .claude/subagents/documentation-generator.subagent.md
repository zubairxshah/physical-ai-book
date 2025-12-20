# Documentation Generator Subagent

**Subagent Type**: Documentation / Content Creation
**Domain**: API Documentation, Setup Guides, Tutorials
**Version**: 1.0.0
**Created**: 2025-12-19

## Purpose

This subagent automatically generates comprehensive, accurate, and user-friendly documentation for the Physical AI Book project, including API references, setup guides, troubleshooting documents, and tutorial materials.

## Activation Triggers

Automatically activate this subagent when:
- New backend API endpoints are added
- New features or modules are implemented
- Installation procedures change
- Configuration options are modified
- User feedback indicates documentation gaps

## Documentation Types

### 1. API Reference Documentation

**Generates**: Complete API endpoint documentation with examples

```markdown
# API Reference

## POST /query

Ask a question to the RAG chatbot.

### Request

**Endpoint**: `/query`
**Method**: `POST`
**Content-Type**: `application/json`

**Body Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| question | string | Yes | The user's question |
| session_id | string | No | Session identifier for conversation history |
| selected_text | string | No | Text selected by user for context |

**Example Request**:
```json
{
  "question": "What is Physical AI?",
  "session_id": "user-123-session",
  "selected_text": null
}
```

### Response

**Status Code**: 200 OK

**Body**:
```json
{
  "answer": "Physical AI refers to artificial intelligence systems...",
  "sources": [
    {
      "chapter": "Chapter 1: Introduction to Physical AI",
      "file": "chapter1.md",
      "relevance_score": 0.92
    }
  ],
  "session_id": "user-123-session"
}
```

**Error Responses**:

| Status Code | Description | Response |
|-------------|-------------|----------|
| 400 | Bad Request | `{"error": "question field is required"}` |
| 500 | Server Error | `{"error": "Internal server error"}` |

### Example Usage

**cURL**:
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "session_id": "demo-session"
  }'
```

**Python**:
```python
import requests

response = requests.post(
    "http://localhost:8000/query",
    json={
        "question": "What is ROS 2?",
        "session_id": "demo-session"
    }
)

print(response.json()["answer"])
```

**JavaScript**:
```javascript
fetch('http://localhost:8000/query', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    question: 'What is ROS 2?',
    session_id: 'demo-session'
  })
})
.then(res => res.json())
.then(data => console.log(data.answer));
```

---
```

### 2. Setup and Installation Guides

**Generates**: Step-by-step setup instructions with troubleshooting

```markdown
# Setup Guide: RAG Chatbot Backend

## Prerequisites

Before starting, ensure you have:
- Python 3.11 or higher
- pip package manager
- Git
- OpenAI API key
- Qdrant Cloud account (free tier)

### Check Prerequisites

```bash
# Check Python version
python --version  # Should show 3.11+

# Check pip
pip --version

# Check Git
git --version
```

## Installation Steps

### Step 1: Clone Repository

```bash
git clone https://github.com/zubairxshah/physical-ai-book.git
cd physical-ai-book/backend
```

### Step 2: Create Virtual Environment

**Linux/Mac**:
```bash
python -m venv venv
source venv/bin/activate
```

**Windows**:
```bash
python -m venv venv
venv\Scripts\activate
```

You should see `(venv)` in your terminal prompt.

### Step 3: Install Dependencies

```bash
pip install -r requirements_rag.txt
```

**Expected output**:
```
Successfully installed fastapi-0.104.0 uvicorn-0.24.0 ...
```

### Step 4: Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
# Copy template
cp .env.example .env

# Edit with your values
nano .env  # or use any text editor
```

**Required variables**:
```env
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...
```

### Step 5: Verify Configuration

```bash
python -c "from dotenv import load_dotenv; load_dotenv(); import os; print('✓ Config loaded')"
```

### Step 6: Start Backend Server

```bash
python main_rag.py
```

**Expected output**:
```
INFO:     Started server process
INFO:     Waiting for application startup.
✅ Database initialized successfully
✅ Qdrant collection exists: physical_ai_book
📚 Found 18 documents
⚙️  Starting document ingestion...
✅ Ingestion complete! 450 chunks uploaded
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Step 7: Verify Installation

In a new terminal:

```bash
# Check health endpoint
curl http://localhost:8000/health

# Expected response:
# {"status":"healthy","qdrant_points":450,"database_connected":true}

# Test query endpoint
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"question":"What is Physical AI?"}'
```

## Troubleshooting

### Issue: "Module not found" errors

**Symptoms**:
```
ModuleNotFoundError: No module named 'fastapi'
```

**Solution**:
```bash
# Verify virtual environment is activated
which python  # Should point to venv/bin/python

# Reinstall dependencies
pip install -r requirements_rag.txt
```

### Issue: "Connection refused" to Qdrant

**Symptoms**:
```
ConnectionError: Failed to connect to Qdrant
```

**Solution**:
```bash
# Check QDRANT_URL in .env
cat .env | grep QDRANT_URL

# Verify URL is accessible
curl $QDRANT_URL/healthz

# If fails, check Qdrant Cloud dashboard
```

### Issue: "Database connection error"

**Symptoms**:
```
sqlalchemy.exc.OperationalError: connection to server failed
```

**Solution**:
```bash
# Verify DATABASE_URL format
# Correct: postgresql://user:password@host/database
# Incorrect: postgres:// (missing 'ql')

# Test connection
python -c "from sqlalchemy import create_engine; engine = create_engine('$DATABASE_URL'); print('✓ Connected')"
```

### Issue: Server starts but queries fail

**Symptoms**:
- Health check returns "healthy"
- Query returns 500 error

**Solution**:
```bash
# Check if documents ingested
curl http://localhost:8000/stats

# If 0 chunks, manually trigger ingestion
curl -X POST http://localhost:8000/ingest

# Check logs for errors
tail -f logs/app.log
```

## Next Steps

After successful setup:
1. [Test the chatbot widget](./TESTING.md)
2. [Deploy to production](./DEPLOYMENT.md)
3. [Configure advanced options](./CONFIGURATION.md)

---
```

### 3. Configuration Reference

**Generates**: Complete configuration option documentation

```markdown
# Configuration Reference

## Environment Variables

### Required Variables

#### OPENAI_API_KEY
- **Type**: String
- **Required**: Yes
- **Description**: OpenAI API key for embeddings and chat completion
- **Example**: `sk-proj-abc123...`
- **How to get**: https://platform.openai.com/api-keys

#### QDRANT_URL
- **Type**: String (URL)
- **Required**: Yes
- **Description**: Qdrant Cloud cluster URL
- **Example**: `https://xyz.us-east.aws.cloud.qdrant.io:6333`
- **How to get**: Qdrant Cloud dashboard → Cluster → Connection

#### QDRANT_API_KEY
- **Type**: String
- **Required**: Yes
- **Description**: Qdrant API key for authentication
- **Example**: `qk_abc123...`
- **How to get**: Qdrant Cloud dashboard → API Keys

### Optional Variables

#### LOG_LEVEL
- **Type**: Enum
- **Required**: No
- **Default**: `INFO`
- **Options**: `DEBUG`, `INFO`, `WARNING`, `ERROR`
- **Description**: Logging verbosity
- **Example**: `LOG_LEVEL=DEBUG`

#### MAX_CHUNKS
- **Type**: Integer
- **Required**: No
- **Default**: `5`
- **Range**: 1-10
- **Description**: Number of document chunks to retrieve for RAG
- **Example**: `MAX_CHUNKS=3`

#### SIMILARITY_THRESHOLD
- **Type**: Float
- **Required**: No
- **Default**: `0.7`
- **Range**: 0.0-1.0
- **Description**: Minimum similarity score for chunk retrieval
- **Example**: `SIMILARITY_THRESHOLD=0.75`

## Configuration File (config.yaml)

Alternative to environment variables:

```yaml
# config.yaml
api:
  openai_key: ${OPENAI_API_KEY}
  host: 0.0.0.0
  port: 8000

qdrant:
  url: ${QDRANT_URL}
  api_key: ${QDRANT_API_KEY}
  collection_name: physical_ai_book

database:
  url: ${DATABASE_URL}
  pool_size: 10

rag:
  max_chunks: 5
  similarity_threshold: 0.7
  embedding_model: text-embedding-3-small
  chat_model: gpt-4

logging:
  level: INFO
  format: json
  output: logs/app.log
```

**Usage**:
```python
from config import load_config

config = load_config('config.yaml')
print(config.rag.max_chunks)  # 5
```

---
```

### 4. Troubleshooting Guides

**Generates**: Comprehensive troubleshooting documentation

```markdown
# Troubleshooting Guide

## Common Issues

### Backend Issues

#### Server won't start

**Symptoms**:
- `python main_rag.py` exits immediately
- Error message about missing modules or configuration

**Diagnostic Steps**:
1. Check Python version: `python --version` (need 3.11+)
2. Verify virtual environment: `which python`
3. Check dependencies: `pip list | grep fastapi`
4. Validate .env file: `cat .env`

**Solutions**:
- Reinstall dependencies: `pip install -r requirements_rag.txt`
- Recreate virtual environment
- Check .env file has all required variables

#### Slow response times

**Symptoms**:
- Queries take >5 seconds
- Timeout errors

**Diagnostic Steps**:
1. Check network latency: `ping api.openai.com`
2. Monitor Qdrant query time: Check logs
3. Profile API endpoint: Use `time curl ...`

**Solutions**:
- Reduce MAX_CHUNKS from 5 to 3
- Use faster LLM model (gpt-3.5-turbo)
- Enable response caching
- Upgrade Qdrant plan for better performance

### Frontend Issues

#### Chatbot doesn't appear

**Symptoms**:
- No chat button on website
- Console errors

**Diagnostic Steps**:
1. Open browser DevTools (F12)
2. Check Console tab for errors
3. Verify chatbot script loaded: Network tab
4. Check if API_URL is correct

**Solutions**:
- Verify `docusaurus.config.ts` includes chatbot script
- Check API_URL in `chatbot-widget-v3.js`
- Clear browser cache
- Test in incognito mode

#### Queries return errors

**Symptoms**:
- "Error processing query" message
- Network errors in console

**Diagnostic Steps**:
1. Check backend is running: `curl http://localhost:8000/health`
2. Verify CORS configuration
3. Check browser console for specific error

**Solutions**:
- Enable CORS in backend
- Check firewall/antivirus not blocking
- Verify API endpoint URL is correct

### Database Issues

#### Connection failures

**Symptoms**:
```
ConnectionError: could not connect to server
```

**Solutions**:
- Verify DATABASE_URL format
- Check database is running
- Verify network connectivity
- Check credentials are correct

#### Slow queries

**Symptoms**:
- Database operations take >1 second

**Solutions**:
- Add indexes to frequently queried columns
- Optimize query patterns
- Consider read replicas for heavy load

---
```

## Auto-Generation Process

### Step 1: Code Analysis

```python
def analyze_codebase() -> Dict:
    """Analyze codebase to extract documentation info"""

    # Extract API endpoints
    endpoints = extract_fastapi_routes("backend/")

    # Extract configuration options
    config_vars = extract_env_variables(".env.example")

    # Extract functions and classes
    code_elements = extract_python_docstrings("backend/")

    return {
        "endpoints": endpoints,
        "config": config_vars,
        "code": code_elements
    }
```

### Step 2: Template Application

```python
def generate_api_docs(endpoints: List[Endpoint]) -> str:
    """Generate API documentation from endpoints"""

    docs = "# API Reference\n\n"

    for endpoint in endpoints:
        docs += f"## {endpoint.method} {endpoint.path}\n\n"
        docs += f"{endpoint.description}\n\n"
        docs += generate_request_docs(endpoint)
        docs += generate_response_docs(endpoint)
        docs += generate_examples(endpoint)
        docs += "\n---\n\n"

    return docs
```

### Step 3: Validation

```python
def validate_documentation(docs: str) -> List[str]:
    """Validate generated documentation"""
    issues = []

    # Check for broken links
    links = extract_links(docs)
    for link in links:
        if not validate_link(link):
            issues.append(f"Broken link: {link}")

    # Check code examples compile
    code_blocks = extract_code_blocks(docs)
    for block in code_blocks:
        if not validate_syntax(block):
            issues.append(f"Invalid code: {block.location}")

    # Check completeness
    if "Prerequisites" not in docs:
        issues.append("Missing Prerequisites section")

    return issues
```

## Documentation Standards

### Writing Style
- **Clarity**: Use simple, direct language
- **Completeness**: Include all necessary information
- **Examples**: Provide concrete, runnable examples
- **Structure**: Use consistent heading hierarchy
- **Audience**: Write for GIAIC students and developers

### Code Examples
- **Complete**: Show full, runnable code
- **Commented**: Explain key steps
- **Tested**: All examples must work
- **Multiple languages**: Python, JavaScript, cURL where applicable

### Formatting
- Use tables for structured data
- Use code blocks with language syntax highlighting
- Use callouts for important notes
- Include emojis sparingly (✅ ❌ ⚠️)

## Integration Points

### With Content Generation Skill
- Content skill creates educational content
- Documentation Generator creates reference docs
- Both follow consistent style guidelines

### With Technical Reviewer Subagent
- Technical Reviewer validates generated docs
- Checks for accuracy, completeness, clarity
- Flags issues for regeneration

### With RAG System
- Documentation is indexed for RAG retrieval
- Users can ask "How do I set up the backend?"
- Chatbot retrieves relevant setup guide sections

## Success Metrics

Documentation is successful when:
- [ ] 90%+ of user questions answered without human help
- [ ] Setup success rate >95% (users can install without issues)
- [ ] API examples work correctly 100% of the time
- [ ] Documentation is up-to-date with code changes
- [ ] Search/RAG can retrieve relevant sections accurately

## Changelog

### v1.0.0 (2025-12-19)
- Initial subagent creation
- Defined documentation types and templates
- Created auto-generation process
- Established validation criteria

## Related Artifacts

- **Content Generation Skill**: Generates book content (educational)
- **Technical Reviewer Subagent**: Reviews generated documentation
- **RAG Query Testing Skill**: Tests documentation retrieval

## Invocation Example

```bash
# Generate API documentation
claude-code invoke subagent documentation-generator \
  --type api-reference \
  --input backend/ \
  --output docs/API.md

# Generate setup guide
claude-code invoke subagent documentation-generator \
  --type setup-guide \
  --input backend/ \
  --output SETUP.md

# Generate all documentation
claude-code invoke subagent documentation-generator \
  --type all \
  --output docs/
```
