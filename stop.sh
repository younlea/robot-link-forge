#!/bin/bash

echo "=========================================="
echo "    Stopping Robot Link Forge...          "
echo "=========================================="

# Find PIDs using lsof (more precise than pkill)
BACKEND_PID=$(lsof -t -i:8000)
FRONTEND_PID=$(lsof -t -i:5173)

if [ -n "$BACKEND_PID" ]; then
    echo ">> Killing Backend (PID: $BACKEND_PID)..."
    kill -9 $BACKEND_PID
else
    echo ">> No Backend found on port 8000."
fi

if [ -n "$FRONTEND_PID" ]; then
    echo ">> Killing Frontend (PID: $FRONTEND_PID)..."
    kill -9 $FRONTEND_PID
else
    echo ">> No Frontend found on port 5173."
fi

echo ">> Done."
