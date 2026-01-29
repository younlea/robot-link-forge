#!/bin/bash
# Replay with Real-time Torque Visualization (Default #0)
echo "Select Visualization Mode:"
echo "1. Joint Torques (Motion with Torque Values)"
echo "2. Motion with Motor Parameter Adjustment"
echo "3. Fingertip Sensors (Motion with Sensor Values)"
echo "4. Exit"
read -p "Enter choice [1]: " choice

if [ "$choice" = "1" ]; then
    # Joint Torques Visualization
    python3 replay_with_torque.py 0 --mode joints
elif [ "$choice" = "2" ]; then
    # Motion with Motor Parameter Adjustment
    python3 motor_parameter_adjustment.py
elif [ "$choice" = "3" ]; then
    # Fingertip Sensors Visualization
    python3 replay_with_torque.py 0 --mode sensors
elif [ "$choice" = "4" ]; then
    echo "Exiting..."
    exit 0
else
    echo "Invalid choice. Exiting..."
    exit 1
fi