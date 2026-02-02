~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh
Installing dependencies (mujoco, matplotlib, numpy)...
========================================
  MuJoCo Motion Analysis Tool
========================================

Select Analysis Mode:

1. Joint Torque Visualization
   - Theoretical torque (inverse dynamics)

2. Motor Sizing Validation
   - Set motor parameters and validate

3. Fingertip Sensor Forces
   - Contact force visualization

========================================
Enter choice [1]: 2
Starting Motor Validation...
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 280
    last_print = now
                    ^
IndentationError: unindent does not match any outer indentation level
younleakim@younleakim-400TEA-400SEA:~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh
Installing dependencies (mujoco, matplotlib, numpy)...
========================================
  MuJoCo Motion Analysis Tool
========================================

Select Analysis Mode:

1. Joint Torque Visualization
   - Theoretical torque (inverse dynamics)

2. Motor Sizing Validation
   - Set motor parameters and validate

3. Fingertip Sensor Forces
   - Contact force visualization

========================================
Enter choice [1]: 3
Starting Sensor Analysis in mode: sensors...
Loading model: direct_hand_parm.xml
Loaded Recording 1768623534448. Mode: sensors. Logging to sensor_log.csv
ERROR: could not create window

run_torque_replqy_0...sh
-------------------------------------------
#!/bin/bash
set -e

# Define venv directory
VENV_DIR="venv"

# 1. Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# 2. Activate venv
source "$VENV_DIR/bin/activate"

# 3. Install dependencies
# Upgrade pip to avoid warnings
pip install --upgrade pip > /dev/null 2>&1

# Install required packages
# Pin matplotlib to <=3.7.3
echo "Installing dependencies (mujoco, matplotlib, numpy)..."
pip install mujoco "matplotlib<=3.7.3" "numpy<2" > /dev/null 2>&1

# 4. Interactive Mode Selection
echo "========================================"
echo "  MuJoCo Motion Analysis Tool"
echo "========================================"
echo ""
echo "Select Analysis Mode:"
echo ""
echo "1. Joint Torque Visualization"
echo "   - Theoretical torque (inverse dynamics)"
echo ""
echo "2. Motor Sizing Validation"  
echo "   - Set motor parameters and validate"
echo ""
echo "3. Fingertip Sensor Forces"
echo "   - Contact force visualization"
echo ""
echo "========================================"
read -p "Enter choice [1]: " choice

MODE="inverse"
SCRIPT="replay_with_torque.py"

if [ "$choice" = "2" ]; then
    # Mode 2 uses separate script
    SCRIPT="replay_motor_validation.py"
    echo "Starting Motor Validation..."
    python3 $SCRIPT 0
elif [ "$choice" = "3" ]; then
    # Mode 3 uses original script with sensors mode
    MODE="sensors"
    echo "Starting Sensor Analysis in mode: $MODE..."
    python3 $SCRIPT 0 --mode $MODE
else
    # Mode 1 (default) uses inverse dynamics
    echo "Starting Analysis in mode: $MODE..."
    python3 $SCRIPT 0 --mode $MODE
fi
