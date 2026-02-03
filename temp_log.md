~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh 
Creating virtual environment...
Installing dependencies (mujoco, matplotlib, numpy)...
========================================
  MuJoCo Motion Analysis Tool
========================================

Select Analysis Mode:

0. Auto Parameter Optimization (NEW)
   - Finds optimal motor parameters automatically
   - Or diagnoses trajectory issues

1. Joint Torque Visualization
   - Theoretical torque (inverse dynamics)

2. Motor Sizing Validation
   - Set motor parameters and validate

3. Fingertip Sensor Forces
   - Contact force visualization

========================================
Enter choice [0 to auto-optimize, 2 for manual tuning]: 2
Starting Motor Validation...
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 248
    gs = fig.add_gridspec(3, 2, height_ratios=[0.5, 2, 1.5], width_ratios=[1, 1.5], 
    ^^
SyntaxError: expected 'except' or 'finally' block
