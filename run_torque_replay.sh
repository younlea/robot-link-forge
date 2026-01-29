#!/bin/bash
# Replay with Real-time Torque Visualization (Default #0)
echo "Select Visualization Mode:"
echo "1. Joint Torques (3x5 Grid)"
echo "2. Fingertip Sensors (1x5 Grid)"
echo "3. Exit"
read -p "Enter choice [1]: " choice

MODE="joints"
if [ "$choice" = "2" ]; then
    MODE="sensors"
elif [ "$choice" = "3" ]; then
    echo "Exiting..."
    exit 0
fi

# Start Flask server and open web viewer
python3 replay_with_torque.py 0 --mode $MODE