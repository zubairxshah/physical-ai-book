# RAG Query Testing Skill

**Skill Type**: Quality Assurance / Testing
**Domain**: Retrieval-Augmented Generation (RAG)
**Version**: 1.0.0
**Created**: 2025-12-19

## Purpose

This skill validates the RAG chatbot's accuracy, relevance, and reliability by systematically testing queries, analyzing responses, and ensuring proper source attribution for the Physical AI Book project.

## When to Use This Skill

- After adding new content to the book
- After regenerating vector embeddings
- Before deploying to production
- When investigating user-reported accuracy issues
- During routine quality assurance checks

## Input Requirements

```yaml
test_type: enum            # "unit" | "integration" | "regression" | "exploratory"
queries: array             # List of test queries to execute
expected_sources: array    # Expected chapter/section citations (optional)
accuracy_threshold: number # Minimum acceptable accuracy (0-1, default: 0.85)
response_time_limit: number # Max acceptable response time in seconds (default: 3)
```

## Output Format

```yaml
test_results:
  timestamp: string        # ISO 8601 format
  total_queries: number
  passed: number
  failed: number
  accuracy_rate: number    # 0-1

detailed_results:
  - query: string
    response: string
    sources_cited: array
    expected_sources: array
    relevance_score: number  # 0-1
    response_time: number    # seconds
    status: enum            # "pass" | "fail" | "warning"
    issues: array           # List of identified problems

recommendations: array     # Suggested improvements
```

## Test Categories

### 1. Factual Accuracy Tests
Verify the chatbot provides correct technical information.

```yaml
- query: "What is Physical AI?"
  expected_content: ["embodied intelligence", "physical systems", "real-world interaction"]
  expected_sources: ["chapter1.md", "intro.md"]

- query: "What is ROS 2?"
  expected_content: ["Robot Operating System", "middleware", "nodes", "topics"]
  expected_sources: ["module1-ros2.md"]

- query: "Name three humanoid robots mentioned in the book"
  expected_content: ["Unitree G1", "Tesla Optimus", "Boston Dynamics Atlas"]
  expected_sources: ["chapter4.md", "chapter5.md"]
```

### 2. Source Attribution Tests
Ensure proper citations are included in responses.

```yaml
- query: "How does bipedal locomotion work?"
  must_include_sources: true
  expected_source_pattern: "chapter6"

- query: "Explain NVIDIA Isaac Sim"
  must_include_sources: true
  expected_source_pattern: "module3"
```

### 3. Context Retrieval Tests
Validate the RAG system retrieves relevant context.

```yaml
- query: "What are Vision-Language-Action models?"
  expected_chunks: 3-5
  expected_sources: ["module4-vla.md", "chapter9.md"]

- query: "How do I set up a ROS 2 node?"
  expected_chunks: 2-4
  expected_sources: ["module1-ros2.md"]
```

### 4. Edge Case Tests
Test boundary conditions and challenging queries.

```yaml
- query: "asdfghjkl"  # Gibberish
  expected_behavior: "graceful_failure"

- query: "What is quantum computing?"  # Out of scope
  expected_behavior: "scope_limitation_message"

- query: ""  # Empty query
  expected_behavior: "validation_error"
```

### 5. Text Selection Tests
Verify text selection feature works correctly.

```yaml
- query: "Explain this in simpler terms"
  selected_text: "Reinforcement learning enables robots to learn optimal policies through trial-and-error interaction with environments."
  expected_behavior: "simplified_explanation"
  expected_sources: ["chapter3.md"]
```

## Test Execution Process

### Step 1: Environment Setup
```python
import requests
import json
from typing import List, Dict

API_URL = "http://localhost:8000"  # or production URL
TEST_SESSION_ID = "test-session-" + timestamp()

def setup_test_environment():
    """Verify API is accessible and healthy"""
    response = requests.get(f"{API_URL}/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"
```

### Step 2: Execute Test Queries
```python
def execute_query_test(query: str, selected_text: str = None) -> Dict:
    """Execute a single test query and capture results"""
    payload = {
        "question": query,
        "session_id": TEST_SESSION_ID,
        "selected_text": selected_text
    }

    start_time = time.time()
    response = requests.post(f"{API_URL}/query", json=payload)
    response_time = time.time() - start_time

    return {
        "query": query,
        "response": response.json(),
        "response_time": response_time,
        "status_code": response.status_code
    }
```

### Step 3: Validate Results
```python
def validate_response(result: Dict, expected: Dict) -> Dict:
    """Validate response against expected criteria"""
    issues = []

    # Check response time
    if result["response_time"] > expected.get("max_response_time", 3.0):
        issues.append(f"Slow response: {result['response_time']:.2f}s")

    # Check source attribution
    if expected.get("must_include_sources"):
        sources = result["response"].get("sources", [])
        if not sources:
            issues.append("Missing source citations")

    # Check content relevance
    response_text = result["response"].get("answer", "").lower()
    for expected_term in expected.get("expected_content", []):
        if expected_term.lower() not in response_text:
            issues.append(f"Missing expected content: {expected_term}")

    return {
        "status": "pass" if not issues else "fail",
        "issues": issues
    }
```

### Step 4: Generate Report
```python
def generate_test_report(results: List[Dict]) -> str:
    """Generate markdown test report"""
    passed = sum(1 for r in results if r["status"] == "pass")
    total = len(results)

    report = f"""# RAG Query Testing Report

**Date**: {datetime.now().isoformat()}
**Total Tests**: {total}
**Passed**: {passed}
**Failed**: {total - passed}
**Success Rate**: {(passed/total)*100:.1f}%

## Detailed Results

"""

    for result in results:
        report += f"""### Query: {result['query']}
- **Status**: {result['status']}
- **Response Time**: {result['response_time']:.2f}s
- **Sources**: {', '.join(result.get('sources', []))}
- **Issues**: {', '.join(result['issues']) if result['issues'] else 'None'}

"""

    return report
```

## Quality Metrics

### Accuracy Metrics
- **Factual Correctness**: 95%+ (manual verification required)
- **Source Attribution**: 100% (automated check)
- **Relevance Score**: 85%+ (based on keyword matching)
- **Response Completeness**: 90%+ (answers address full query)

### Performance Metrics
- **Response Time**: <3 seconds for 95th percentile
- **API Availability**: 99.9% uptime
- **Error Rate**: <1% of queries
- **Token Efficiency**: <1000 tokens per response on average

### User Experience Metrics
- **Answer Helpfulness**: Subjective, gather user feedback
- **Citation Usefulness**: Users can navigate to source
- **Text Selection Accuracy**: Feature works on all pages

## Common Issues and Solutions

### Issue 1: Missing Sources
**Symptom**: Response doesn't include chapter citations
**Causes**:
- Vector database not properly populated
- Similarity threshold too high
- Embedding generation failed

**Solution**:
```bash
# Reingest documents
curl -X POST http://localhost:8000/ingest

# Verify Qdrant points
curl http://localhost:8000/health
```

### Issue 2: Irrelevant Responses
**Symptom**: Answer doesn't address the query
**Causes**:
- Poor query embedding
- Insufficient context retrieved
- LLM hallucination

**Solution**:
- Adjust similarity threshold in RAG config
- Increase number of retrieved chunks (k parameter)
- Add more explicit prompting in system message

### Issue 3: Slow Response Times
**Symptom**: Queries take >3 seconds consistently
**Causes**:
- Database query slow
- LLM API latency
- Network issues

**Solution**:
- Optimize Qdrant query (add indexing)
- Use faster LLM model (e.g., gpt-3.5-turbo vs gpt-4)
- Implement caching for common queries

### Issue 4: Text Selection Not Working
**Symptom**: Selected text not included in context
**Causes**:
- Frontend not capturing selection correctly
- Backend not receiving selected_text parameter
- Selection too long (>500 words)

**Solution**:
- Verify frontend selection capture logic
- Check API payload in browser dev tools
- Implement chunking for long selections

## Integration with CI/CD

### Pre-Deployment Testing
```yaml
# .github/workflows/rag-test.yml
name: RAG Quality Tests

on:
  push:
    branches: [main]
    paths:
      - 'docs/**'
      - 'backend/**'

jobs:
  test-rag:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Start backend
        run: |
          cd backend
          pip install -r requirements_rag.txt
          python main_rag.py &
          sleep 30

      - name: Run RAG tests
        run: |
          python tests/test_rag_queries.py

      - name: Upload report
        uses: actions/upload-artifact@v3
        with:
          name: rag-test-report
          path: test-reports/rag-*.md
```

## Test Suite Examples

### Comprehensive Test Suite
```python
COMPREHENSIVE_TESTS = [
    # Factual accuracy
    {
        "query": "What is Physical AI?",
        "expected_content": ["embodied", "physical systems"],
        "expected_sources": ["intro.md", "chapter1.md"]
    },
    {
        "query": "How does ROS 2 differ from ROS 1?",
        "expected_content": ["DDS", "real-time", "security"],
        "expected_sources": ["module1-ros2.md"]
    },

    # Code-related queries
    {
        "query": "Show me a ROS 2 publisher example",
        "expected_content": ["create_publisher", "rclpy", "timer"],
        "expected_sources": ["module1-ros2.md"]
    },

    # Comparative queries
    {
        "query": "Compare Gazebo and Isaac Sim",
        "expected_content": ["simulation", "physics", "photorealistic"],
        "expected_sources": ["module2-digital-twin.md", "module3-nvidia-isaac.md"]
    },

    # Application queries
    {
        "query": "What are real-world applications of humanoid robots?",
        "expected_content": ["manufacturing", "healthcare", "domestic"],
        "expected_sources": ["chapter10.md"]
    },

    # Edge cases
    {
        "query": "",
        "expected_behavior": "error"
    },
    {
        "query": "What is the meaning of life?",
        "expected_behavior": "out_of_scope"
    }
]
```

## Success Criteria

A RAG system passes testing when:

- [ ] 95%+ of factual queries answered correctly
- [ ] 100% of responses include source citations
- [ ] 90%+ response time under 3 seconds
- [ ] 0% queries return errors (except invalid inputs)
- [ ] Text selection feature works for all test cases
- [ ] Out-of-scope queries handled gracefully
- [ ] No hallucinated information in responses
- [ ] Sources are accurate and navigable links

## Changelog

### v1.0.0 (2025-12-19)
- Initial skill creation
- Defined test categories and metrics
- Created test execution process
- Established quality criteria

## Related Skills

- **Content Generation Skill**: Ensures content is RAG-friendly
- **Code Example Generation Skill**: Validates code examples are retrievable
- **Technical Reviewer Subagent**: Validates factual accuracy of responses

## References

- OpenAI Embeddings API documentation
- Qdrant vector database best practices
- RAG system evaluation frameworks (RAGAS, TruLens)
