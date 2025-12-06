@echo off
echo Starting Physical AI Book RAG Chatbot Backend...
echo.

REM Check if .env exists
if not exist .env (
    echo ERROR: .env file not found!
    echo Please copy .env.example to .env and add your API keys
    pause
    exit /b 1
)

echo.
echo Starting server on http://localhost:8000
echo API docs available at http://localhost:8000/docs
echo.
echo Press Ctrl+C to stop the server
echo.

REM Start the server
uvicorn main:app --reload --port 8000
