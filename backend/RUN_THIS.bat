@echo off
echo ========================================
echo Physical AI Book Chatbot - Quick Start
echo ========================================
echo.

REM Check if .env exists
if not exist .env (
    echo ERROR: .env file not found!
    pause
    exit /b 1
)

echo Starting server...
echo.
echo Ignore any langchain warnings - they don't affect our chatbot!
echo.
echo Server will start on: http://localhost:8000
echo API docs: http://localhost:8000/docs
echo.
echo After server starts, open a NEW terminal and run:
echo   curl -X POST http://localhost:8000/ingest
echo.
echo Press Ctrl+C to stop the server
echo.
echo ========================================
echo.

uvicorn main_simple:app --reload --port 8000
