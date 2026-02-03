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
Loading model: direct_hand_parm.xml
Loaded Recording 1768623534448. Mode: Motor Validation

=== Configuration ===
Control Params (global): kp=800.0, kv=80.0
Motor Specs (default): gear=200.0, forcelim=1000.0
Total joints: 20, Total actuators: 20

First 3 joints:
  LittleFinger-1st_roll: gear=200.0, forcelim=1000.0
  LittleFinger-1st-pitch: gear=200.0, forcelim=1000.0
  LittleFinger-2nd-pitch: gear=200.0, forcelim=1000.0
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 409, in <module>
    btn_reset_joint.on_clicked(on_reset_joint)
NameError: name 'on_reset_joint' is not defined. Did you mean: 'btn_reset_joint'?
