"""
Test script for RAG chatbot API
"""
import requests
import json

BASE_URL = "http://localhost:8000"

def test_health():
    """Test health endpoint"""
    print("Testing /health endpoint...")
    response = requests.get(f"{BASE_URL}/health")
    print(f"Status: {response.status_code}")
    print(f"Response: {response.json()}\n")

def test_ingest():
    """Test document ingestion"""
    print("Testing /ingest endpoint...")
    response = requests.post(f"{BASE_URL}/ingest")
    print(f"Status: {response.status_code}")
    print(f"Response: {response.json()}\n")

def test_query(question, selected_text=""):
    """Test query endpoint"""
    print(f"Testing query: {question}")
    if selected_text:
        print(f"With selected text: {selected_text[:50]}...")
    
    payload = {
        "question": question,
        "selected_text": selected_text
    }
    
    response = requests.post(
        f"{BASE_URL}/query",
        json=payload,
        headers={"Content-Type": "application/json"}
    )
    
    print(f"Status: {response.status_code}")
    if response.status_code == 200:
        data = response.json()
        print(f"Answer: {data['answer']}")
        print(f"Sources: {data['sources']}\n")
    else:
        print(f"Error: {response.text}\n")

if __name__ == "__main__":
    print("=" * 60)
    print("RAG Chatbot API Test Suite")
    print("=" * 60 + "\n")
    
    # Test 1: Health check
    test_health()
    
    # Test 2: Ingest documents
    test_ingest()
    
    # Test 3: Simple queries
    test_query("What is Physical AI?")
    test_query("What are the main challenges in humanoid robotics?")
    test_query("Explain sensor fusion")
    
    # Test 4: Query with selected text
    selected = "Physical AI combines perception, cognition, and action in real-world environments."
    test_query("Explain this concept in simple terms", selected)
    
    print("=" * 60)
    print("Tests completed!")
    print("=" * 60)
