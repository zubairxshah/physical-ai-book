@echo off
echo ========================================
echo Physical AI Book - Starting All Servers
echo ========================================
echo.
echo IMPORTANT: You need 4 terminal windows!
echo.
echo This will open 4 separate windows:
echo   1. Frontend (Docusaurus) - Port 3000
echo   2. Auth Server - Port 8000
echo   3. Personalization API - Port 8001
echo   4. Translation API - Port 8002
echo.
echo Press Ctrl+C in any window to stop that server
echo ========================================
echo.
pause

echo Starting servers...

REM Start Frontend
start "Frontend (Port 3000)" cmd /k "npm start"
timeout /t 2 >nul

REM Start Auth Server
start "Auth Server (Port 8000)" cmd /k "node auth-server.js"
timeout /t 2 >nul

REM Start Personalization API
start "Personalization API (Port 8001)" cmd /k "python backend/personalization_api.py"
timeout /t 2 >nul

REM Start Translation API
start "Translation API (Port 8002)" cmd /k "python backend/translation_api.py"

echo.
echo ========================================
echo All servers starting!
echo ========================================
echo.
echo Check each window for status:
echo   - Frontend: http://localhost:3000
echo   - Auth: http://localhost:8000/api/auth/health
echo   - Personalization: http://localhost:8001/health
echo   - Translation: http://localhost:8002/health
echo.
echo If any server fails, check:
echo   1. Database connection in .env.local
echo   2. All dependencies installed (npm install, pip install -r backend/requirements_rag.txt)
echo   3. No port conflicts (kill processes on ports 3000, 8000, 8001, 8002)
echo.
echo See START_SERVERS.md for troubleshooting
echo ========================================
