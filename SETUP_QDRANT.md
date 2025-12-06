# Qdrant Cloud Setup Guide

## Step 1: Create Qdrant Cloud Account

1. Go to https://cloud.qdrant.io/
2. Click "Sign Up" or "Get Started"
3. Sign up with GitHub or Google (recommended)

## Step 2: Create a Cluster

1. After login, click "Create Cluster"
2. Choose **Free Tier**:
   - 1GB storage (enough for your book)
   - No credit card required
3. Select region (choose closest to you):
   - US East (recommended for US)
   - EU West (for Europe)
   - Asia Pacific (for Asia)
4. Give it a name: `physical-ai-book`
5. Click "Create"

## Step 3: Get Your Credentials

After cluster is created (takes 1-2 minutes):

1. Click on your cluster name
2. You'll see:
   - **Cluster URL**: Something like `https://xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx.us-east4-0.gcp.cloud.qdrant.io:6333`
   - **API Key**: Click "Show" to reveal

## Step 4: Update Your .env File

Open `backend/.env` and update:

https://i.ibb.co/1Y3VmFzY/image.png

## Step 5: Test Connection

```bash
cd backend
python -c "from qdrant_client import QdrantClient; import os; from dotenv import load_dotenv; load_dotenv(); client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); print('Connected!', client.get_collections())"
```

If successful, you'll see: `Connected! CollectionsResponse(collections=[])`

## Troubleshooting

### "Connection refused"
- Check if URL includes `:6333` port
- Verify cluster is running in Qdrant dashboard

### "Authentication failed"
- Double-check API key
- Make sure no extra spaces in .env file

### "Cluster not found"
- Wait 2-3 minutes for cluster to fully start
- Refresh Qdrant dashboard

## Next Steps

Once Qdrant is configured:

1. Start backend: `cd backend && uvicorn main:app --reload`
2. Ingest content: `curl -X POST http://localhost:8000/ingest`
3. Test query: `curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d "{\"question\":\"What is Physical AI?\"}"`

You're all set! 🚀
