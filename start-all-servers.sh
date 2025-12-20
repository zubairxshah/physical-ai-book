#!/bin/bash

echo "========================================"
echo "Physical AI Book - Starting All Servers"
echo "========================================"
echo ""
echo "IMPORTANT: Starting 4 servers in background"
echo ""
echo "This will start:"
echo "  1. Frontend (Docusaurus) - Port 3000"
echo "  2. Auth Server - Port 8000"
echo "  3. Personalization API - Port 8001"
echo "  4. Translation API - Port 8002"
echo ""
echo "Logs will be written to ./logs/"
echo "Use 'pkill -f auth-server' to stop servers"
echo "========================================"
echo ""

# Create logs directory
mkdir -p logs

# Start Frontend
echo "Starting Frontend (Port 3000)..."
npm start > logs/frontend.log 2>&1 &
FRONTEND_PID=$!
echo "Frontend PID: $FRONTEND_PID"
sleep 2

# Start Auth Server
echo "Starting Auth Server (Port 8000)..."
node auth-server.js > logs/auth.log 2>&1 &
AUTH_PID=$!
echo "Auth Server PID: $AUTH_PID"
sleep 2

# Start Personalization API
echo "Starting Personalization API (Port 8001)..."
python backend/personalization_api.py > logs/personalization.log 2>&1 &
PERSON_PID=$!
echo "Personalization API PID: $PERSON_PID"
sleep 2

# Start Translation API
echo "Starting Translation API (Port 8002)..."
python backend/translation_api.py > logs/translation.log 2>&1 &
TRANS_PID=$!
echo "Translation API PID: $TRANS_PID"

echo ""
echo "========================================"
echo "All servers starting!"
echo "========================================"
echo ""
echo "Process IDs:"
echo "  Frontend: $FRONTEND_PID"
echo "  Auth: $AUTH_PID"
echo "  Personalization: $PERSON_PID"
echo "  Translation: $TRANS_PID"
echo ""
echo "Check logs:"
echo "  tail -f logs/frontend.log"
echo "  tail -f logs/auth.log"
echo "  tail -f logs/personalization.log"
echo "  tail -f logs/translation.log"
echo ""
echo "Health checks:"
echo "  Frontend: http://localhost:3000"
echo "  Auth: http://localhost:8000/api/auth/health"
echo "  Personalization: http://localhost:8001/health"
echo "  Translation: http://localhost:8002/health"
echo ""
echo "To stop all servers:"
echo "  kill $FRONTEND_PID $AUTH_PID $PERSON_PID $TRANS_PID"
echo ""
echo "See START_SERVERS.md for troubleshooting"
echo "========================================"

# Save PIDs to file for easy cleanup
echo "$FRONTEND_PID $AUTH_PID $PERSON_PID $TRANS_PID" > .server_pids

# Wait for all processes
wait
