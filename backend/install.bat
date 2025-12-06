@echo off
echo Installing dependencies for Python 3.13...
echo.

REM Upgrade pip first
python -m pip install --upgrade pip

REM Install packages one by one to avoid conflicts
echo Installing FastAPI...
pip install fastapi==0.115.5

echo Installing Uvicorn...
pip install uvicorn[standard]==0.32.1

echo Installing OpenAI...
pip install openai==1.55.3

echo Installing python-dotenv...
pip install python-dotenv==1.0.1

echo Installing Pydantic...
pip install pydantic==2.10.3

echo Installing grpcio (required for Qdrant)...
pip install grpcio==1.62.0

echo Installing Qdrant client...
pip install qdrant-client==1.11.3

echo.
echo Installation complete!
echo.
pause
