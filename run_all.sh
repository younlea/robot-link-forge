#!/bin/bash

# Robot Link Forge - One-Click Run Script
# This script installs dependencies for both backend (Python) and frontend (Node),
# and then runs them concurrently.

echo "=========================================="
echo "    Starting Robot Link Forge...          "
echo "=========================================="

# 1. Backend Setup
echo ">> [Backend] Checking Python environment..."
if [ ! -d "src/backend/venv" ]; then
    echo ">> [Backend] Creating virtual environment..."
    python3 -m venv src/backend/venv
fi

# Ensure local traffic bypasses any system proxies
export NO_PROXY=localhost,127.0.0.1,0.0.0.0,::1,.local

source src/backend/venv/bin/activate
echo ">> [Backend] Installing dependencies..."
pip install -r src/backend/requirements.txt

# 2. Frontend Setup
echo ">> [Frontend] Installing dependencies..."
cd src/frontend
if [ ! -d "node_modules" ]; then
    npm install
else
    # Check if packages installed match package.json (simple check)
    # Just run install to be safe/fast (caches usually work)
    echo ">> [Frontend] Checking for new dependencies..."
    npm install
fi
cd ../../

# 3. Run Both
echo "=========================================="
echo "    Launching Services                    "
echo "=========================================="

cleanup() {
    echo ""
    echo ">> Stopping servers..."
    kill $BACKEND_PID 2>/dev/null
    kill $FRONTEND_PID 2>/dev/null
    exit
}

trap cleanup SIGINT

# Start Backend
echo ">> Starting Backend (Port 8000)..."
cd src/backend
uvicorn main:app --host 0.0.0.0 --port 8000 --reload &
BACKEND_PID=$!
cd ../../

# Wait a second for backend
sleep 2

# Start Frontend
echo ">> Starting Frontend (Port 5173)..."
cd src/frontend
npm run dev -- --host &
FRONTEND_PID=$!
cd ../../

echo ">> App running at: http://localhost:5173"
echo ">> Press Ctrl+C to stop."

wait
