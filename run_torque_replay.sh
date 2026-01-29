# Replay with Real-time Torque Visualization (Default #0)
echo "Select Visualization Mode:"
echo "1. Joint Torques (3x5 Grid)"
echo "2. Fingertip Sensors (1x5 Grid)"
read -p "Enter choice [1]: " choice

MODE="joints"
if [ "$choice" = "2" ]; then
    MODE="sensors"
fi

python3 replay_with_torque.py 0 --mode $MODE