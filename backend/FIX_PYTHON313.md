# Fix for Python 3.13 Compatibility

## Problem
Python 3.13 is too new for some packages. Qdrant-client has compatibility issues.

## Solution - Install Dependencies Manually

Run these commands in the `backend` folder:

```bash
# 1. Upgrade pip
python -m pip install --upgrade pip

# 2. Install packages one by one
pip install fastapi==0.115.5
pip install uvicorn[standard]==0.32.1
pip install openai==1.55.3
pip install python-dotenv==1.0.1
pip install pydantic==2.10.3
pip install grpcio==1.62.0
pip install qdrant-client==1.11.3
```

## Or Use the Install Script

```bash
cd backend
install.bat
```

## Verify Installation

```bash
python -c "from qdrant_client import QdrantClient; print('Success!')"
```

Should print: `Success!`

## Start the Server

```bash
start.bat
```

## Alternative: Use Python 3.11

If issues persist, install Python 3.11:

1. Download from: https://www.python.org/downloads/release/python-3119/
2. Install Python 3.11
3. Use it for this project:
   ```bash
   py -3.11 -m pip install -r requirements.txt
   py -3.11 -m uvicorn main:app --reload --port 8000
   ```

## Test Backend

Once running, test:

```bash
# Health check
curl http://localhost:8000/health

# Should return: {"status":"ok"}
```
