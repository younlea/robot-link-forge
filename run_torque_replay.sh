#!/bin/bash
# Replay with Real-time Torque Visualization (Default #0)
echo "Select Visualization Mode:"
echo "1. Joint Torques (Motion with Torque Values)"
echo "2. Motion with Motor Parameter Adjustment"
echo "3. Fingertip Sensors (Motion with Sensor Values)"
echo "4. Exit"
read -p "Enter choice [1]: " choice

# Define script paths
PYTHON_SCRIPT_DIR="$(dirname "$0")"
TORQUE_SCRIPT="$PYTHON_SCRIPT_DIR/replay_with_torque.py"
MOTOR_PARAM_SCRIPT="$PYTHON_SCRIPT_DIR/motor_parameter_adjustment.py"

if [ "$choice" = "1" ]; then
    # Joint Torques Visualization
    python3 "$TORQUE_SCRIPT" 0 --mode joints
elif [ "$choice" = "2" ]; then
    # Motion with Motor Parameter Adjustment
    python3 "$MOTOR_PARAM_SCRIPT"
elif [ "$choice" = "3" ]; then
    # Fingertip Sensors Visualization
    python3 "$TORQUE_SCRIPT" 0 --mode sensors
elif [ "$choice" = "4" ]; then
    echo "Exiting..."
    exit 0
else
    echo "Invalid choice. Exiting..."
    exit 1
fi