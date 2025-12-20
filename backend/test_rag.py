#!/usr/bin/env python3
"""
Quick test script for RAG chatbot
Tests Qdrant, Neon Postgres, and OpenRouter connections
"""

import os
from dotenv import load_dotenv
import sys

# Load environment variables
load_dotenv()

print("🧪 Testing RAG Chatbot Components...")
print("=" * 60)

# Test 1: Environment Variables
print("\n1️⃣ Checking Environment Variables...")
required_vars = [
    "OPENROUTER_API_KEY",
    "OPENAI_API_KEY",
    "DATABASE_URL",
    "QDRANT_URL",
    "QDRANT_API_KEY"
]

missing_vars = []
for var in required_vars:
    value = os.getenv(var)
    if value:
        masked = value[:10] + "..." if len(value) > 10 else value
        print(f"   ✅ {var}: {masked}")
    else:
        print(f"   ❌ {var}: NOT SET")
        missing_vars.append(var)

if missing_vars:
    print(f"\n❌ Missing variables: {', '.join(missing_vars)}")
    sys.exit(1)

# Test 2: Qdrant Connection
print("\n2️⃣ Testing Qdrant Connection...")
try:
    from qdrant_client import QdrantClient

    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    collections = qdrant_client.get_collections()
    print(f"   ✅ Connected to Qdrant")
    print(f"   📊 Collections: {[c.name for c in collections.collections]}")

    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        print(f"   ✅ Collection '{collection_name}' exists")
        print(f"   📈 Points: {collection_info.points_count}")
    except Exception:
        print(f"   ⚠️  Collection '{collection_name}' not found (will be created)")

except Exception as e:
    print(f"   ❌ Qdrant connection failed: {e}")
    sys.exit(1)

# Test 3: Neon Postgres Connection
print("\n3️⃣ Testing Neon Postgres Connection...")
try:
    import psycopg2

    conn = psycopg2.connect(os.getenv("DATABASE_URL"))
    cursor = conn.cursor()

    # Test query
    cursor.execute("SELECT version();")
    version = cursor.fetchone()[0]
    print(f"   ✅ Connected to Postgres")
    print(f"   📊 Version: {version[:50]}...")

    # Check tables
    cursor.execute("""
        SELECT table_name FROM information_schema.tables
        WHERE table_schema = 'public'
    """)
    tables = cursor.fetchall()
    table_names = [t[0] for t in tables]

    if 'chat_history' in table_names:
        print(f"   ✅ Table 'chat_history' exists")

        cursor.execute("SELECT COUNT(*) FROM chat_history")
        count = cursor.fetchone()[0]
        print(f"   📈 Chat messages: {count}")
    else:
        print(f"   ⚠️  Table 'chat_history' not found (will be created)")

    cursor.close()
    conn.close()

except Exception as e:
    print(f"   ❌ Postgres connection failed: {e}")
    sys.exit(1)

# Test 4: OpenAI Embeddings
print("\n4️⃣ Testing OpenAI Embeddings...")
try:
    from openai import OpenAI

    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    response = openai_client.embeddings.create(
        model="text-embedding-3-small",
        input="Test embedding"
    )

    embedding = response.data[0].embedding
    print(f"   ✅ OpenAI embeddings working")
    print(f"   📊 Embedding dimension: {len(embedding)}")

except Exception as e:
    print(f"   ❌ OpenAI embeddings failed: {e}")
    sys.exit(1)

# Test 5: OpenRouter API
print("\n5️⃣ Testing OpenRouter API...")
try:
    import httpx

    url = "https://openrouter.ai/api/v1/chat/completions"
    headers = {
        "Authorization": f"Bearer {os.getenv('OPENROUTER_API_KEY')}",
        "Content-Type": "application/json"
    }
    data = {
        "model": os.getenv("OPENROUTER_MODEL", "qwen/qwen-2.5-vl-7b-instruct:free"),
        "messages": [{"role": "user", "content": "Say hello"}],
        "max_tokens": 10
    }

    response = httpx.post(url, json=data, headers=headers, timeout=10.0)
    response.raise_for_status()

    result = response.json()
    answer = result["choices"][0]["message"]["content"]

    print(f"   ✅ OpenRouter API working")
    print(f"   💬 Response: {answer}")

except Exception as e:
    print(f"   ❌ OpenRouter API failed: {e}")
    sys.exit(1)

# All tests passed
print("\n" + "=" * 60)
print("✅ ALL TESTS PASSED!")
print("\nYou're ready to run the RAG chatbot:")
print("   python main_rag.py")
print("\nOr test the API:")
print("   curl http://localhost:8000/health")
