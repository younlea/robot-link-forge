./run_torque_replay_0_recording_1768623534448.sh 
Creating virtual environment...
Installing dependencies (mujoco, matplotlib, numpy, scipy)...
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

4. Inverse-to-Forward Validation (NEW)
   - Use Mode 1 torques as motor limits
   - Test if physics can actually track trajectory

========================================
Enter choice [0/1/2/3/4]: 4
Starting Inverse-to-Forward Validation...
======================================================================
MODE 4 SCRIPT VERSION: 20260205_190537
Generated: 2026-02-05 19:05:37
======================================================================


======================================================================
PREPARING MODEL FOR PURE TORQUE CONTROL
======================================================================
Removing position actuators from MJCF...
  This allows pure qfrc_applied control without actuator interference
  Created temporary MJCF without actuators: tmpgj95naam.xml
  Model loaded: 0 actuators (should be 0), 20 DOFs
  ✅ SUCCESS: Pure torque control model (no actuators)
======================================================================
✅ No actuators in model (pure torque control mode)
Model has 20 joints, 0 actuators
Recording duration: 6.83s (6835ms)
Found 20 joints in recording

🔍 KEYFRAME TIMING:
  Number of keyframes: 5
  First keyframe time: 0.580s
  Last keyframe time: 6.835s
  Recording duration: 6.835s
  Keyframe times: 0.58s 2.09s 3.61s 5.05s 6.83s 

🔍 TRAJECTORY DIAGNOSTIC:
Checking if trajectory actually changes over time...
  IndexFinger-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0247 rad
    Step 2500: -1.3854 rad
    Step 5000: 0.0128 rad
    Range: 1.7792 rad
    Vel range: -1.43 to 1.42 rad/s
    Acc range: -2.87 to 3.27 rad/s²
  MiddleFinger-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0019 rad
    Step 2500: 0.1196 rad
    Step 5000: -1.5628 rad
    Range: 1.7977 rad
    Vel range: -1.44 to 1.34 rad/s
    Acc range: -3.05 to 3.13 rad/s²
  Thumb-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0157 rad
    Step 2500: -0.7703 rad
    Step 5000: -0.7692 rad
    Range: 0.9640 rad
    Vel range: -0.89 to 0.90 rad/s
    Acc range: -2.21 to 2.23 rad/s²

======================================================================
PHASE 1: INVERSE DYNAMICS ANALYSIS
======================================================================
Calculating required torques for trajectory...

🔍 DATA STRUCTURE DIAGNOSTIC:
  model.nu (actuators): 0
  model.nv (DOFs): 20
  model.nq (positions): 20

  Joint → DOF mapping:
    LittleFinger-1st_roll          → DOF 0
    LittleFinger-1st-pitch         → DOF 1
    LittleFinger-2nd-pitch         → DOF 2
    LittleFinger-3rd-pitch         → DOF 3
    RingFinger-1st_roll            → DOF 4
    RingFinger-1st-pitch           → DOF 5
    RingFinger-2nd-pitch           → DOF 6
    RingFinger-3rd-pitch           → DOF 7
    MiddleFinger_1st_roll          → DOF 8
    MiddleFinger-1st-pitch         → DOF 9
    MiddleFinger-2nd-pitch         → DOF 10
    MiddleFinger-3rd-pitch         → DOF 11
    IndexFinger_1st_roll           → DOF 12
    IndexFinger-1st-pitch          → DOF 13
    IndexFinger-2nd-pitch          → DOF 14
    IndexFinger-3rd-pitch          → DOF 15
    thumb_1st_yaw                  → DOF 16
    Thumb-1st-pitch                → DOF 17
    Thumb-2nd-pitch                → DOF 18
    Thumb-3rd-pitch                → DOF 19

  Actuator → Joint mapping:

======================================================================
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/inverse_to_forward_validation.py", line 402, in <module>
    print(f"  Step {step}/{n_steps}: Max torque so far = {np.max(max_torques):.2f} Nm")
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/numpy/core/fromnumeric.py", line 2810, in max
    return _wrapreduction(a, np.maximum, 'max', axis, None, out,
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/numpy/core/fromnumeric.py", line 88, in _wrapreduction
    return ufunc.reduce(obj, axis, dtype, out, **passkwargs)
ValueError: zero-size array to reduction operation maximum which has no identity
younleakim@younleakim-400TEA-400SEA:~/Downloads/direct_hand_parm$ 
