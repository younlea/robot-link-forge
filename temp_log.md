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
Enter choice [0 to auto-optimize, 2 for manual tuning]: 0
Running automatic parameter optimization...
======================================================================
[MODE 0] AUTOMATIC MOTOR PARAMETER OPTIMIZATION
======================================================================
Model: direct_hand_parm.xml

Step 1: Testing default parameters (kp=1500, kv=150, forcelim=200)...
----------------------------------------------------------------------
  Tracking Error: 1.83deg (max: 3.03deg)
  Saturation: 14.6%
  Stability: 0.88

======================================================================
[SUCCESS] DEFAULT PARAMETERS WORK PERFECTLY!
======================================================================
No optimization needed.


>>  Would you like to open Mode 2 (Interactive Motor Tuning)? [Y/n]: y

======================================================================
[MODE 2] LAUNCHING INTERACTIVE MOTOR TUNING
======================================================================
You can now fine-tune parameters per-joint...

younleakim@younleakim-400TEA-400SEA:~/Downloads/direct_hand_parm$ 
