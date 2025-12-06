@echo off
echo Starting Physical AI Book Chatbot (Simple Version - No Qdrant)...
echo.

REM Check if .env exists
if not exist .env (
    echo ERROR: .env file not found!
    echo Please copy .env.example to .env and add your OpenAI API key
    pause
    exit /b 1
)

echo Installing dependencies...
pip install -r requirements_simple.txt

echo.
echo Starting server on http://localhost:8000
echo API docs available at http://localhost:8000/docs
echo.
echo Press Ctrl+C to stop the server
echo.

REM Start the server with simple version
uvicorn main_simple:app --reload --port 8000
