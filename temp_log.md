~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh 
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
MODE 4 SCRIPT VERSION: 20260205_195345
Generated: 2026-02-05 19:53:45
======================================================================


======================================================================
PREPARING MODEL FOR PURE TORQUE CONTROL
======================================================================
Removing position actuators from MJCF...
  This allows pure qfrc_applied control without actuator interference
  Created temporary MJCF without actuators: tmpwu_c7o5o.xml
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
    (No actuators - using pure torque control)

======================================================================
  Step 0/6836: Max torque so far = 2.91 Nm
  Step 500/6836: Max torque so far = 5.52 Nm
  Step 1000/6836: Max torque so far = 5.52 Nm
  Step 1500/6836: Max torque so far = 5.52 Nm
  Step 2000/6836: Max torque so far = 25.84 Nm

🔍 DEBUG Step 2500 - qfrc_inverse values:
  qfrc_inverse shape: (20,)
  qfrc_inverse max: 22.96 Nm
    MiddleFinger-1st-pitch         DOF[9]:   +22.96 Nm
    MiddleFinger-2nd-pitch         DOF[10]:    +1.08 Nm
    MiddleFinger-3rd-pitch         DOF[11]:    +1.75 Nm
    IndexFinger-1st-pitch          DOF[13]:    +0.51 Nm
    IndexFinger-2nd-pitch          DOF[14]:    +2.92 Nm
    Thumb-1st-pitch                DOF[17]:    +0.33 Nm
  Step 2500/6836: Max torque so far = 27.85 Nm
  Step 3000/6836: Max torque so far = 39.10 Nm
  Step 3500/6836: Max torque so far = 39.10 Nm
  Step 4000/6836: Max torque so far = 39.10 Nm
  Step 4500/6836: Max torque so far = 39.10 Nm
  Step 5000/6836: Max torque so far = 39.10 Nm
  Step 5500/6836: Max torque so far = 39.10 Nm
  Step 6000/6836: Max torque so far = 39.10 Nm
  Step 6500/6836: Max torque so far = 39.10 Nm

💾 Saving torque history to CSV...
  Saved 6836 steps × 20 joints to phase1_torque_history.csv
  File size: ~1068.1 KB

Inverse Dynamics Results:
  Joint Name                    | Max Torque (Nm)
  ------------------------------------------------------------
  LittleFinger-1st_roll          |     0.01
  LittleFinger-1st-pitch         |     0.01
  LittleFinger-2nd-pitch         |     0.00
  LittleFinger-3rd-pitch         |     0.00
  RingFinger-1st_roll            |     0.00
  RingFinger-1st-pitch           |     0.01
  RingFinger-2nd-pitch           |     0.00
  RingFinger-3rd-pitch           |     0.00
  MiddleFinger_1st_roll          |     0.42
  MiddleFinger-1st-pitch         |    39.10
  MiddleFinger-2nd-pitch         |    10.07
  MiddleFinger-3rd-pitch         |     1.95
  IndexFinger_1st_roll           |     0.22
  IndexFinger-1st-pitch          |    31.38
  IndexFinger-2nd-pitch          |    10.66
  IndexFinger-3rd-pitch          |     5.34
  thumb_1st_yaw                  |     1.86
  Thumb-1st-pitch                |    24.41
  Thumb-2nd-pitch                |    12.33
  Thumb-3rd-pitch                |     6.02

  Overall Peak Torque: 39.10 Nm
  Average Peak Torque: 7.19 Nm

Applying 2.0x safety margin...
  → Inverse dynamics only accounts for ideal motion
  → Forward simulation needs extra for friction, damping, numerical errors
  Adjusted force limits: 14.38 Nm (avg), 78.21 Nm (max)

======================================================================
PHASE 2: PHYSICS SIMULATION WITH TORQUE CONTROL
======================================================================

Using REAL PHYSICS SIMULATION with torque control:
  Apply torques from Phase 1 → mj_step() → simulate dynamics
  This tests: Can torque control track the trajectory?
  Goal: Prepare for motor parameter tuning (Mode 2 development)

Note: Using mj_step() for REAL dynamics simulation
  NOT kinematic playback - we want to see physics behavior!

📍 Initialization for physics simulation:
  Setting initial pose to TRAJECTORY FIRST FRAME (not qpos=0)
  This avoids geometric constraint violations
  Disabling position actuators (using ONLY qfrc_applied torque control)
  Initial pose from trajectory: [0. 0. 0. 0. 0.]
  Actuators disabled, using pure torque control
  This should be stable (no constraint violations)

=== Initial Position Check ===

Initial mismatch RMS: 0.0000 rad (0.00°)
Note: With torque control, initial mismatch is okay
==================================================

Starting forward simulation with TORQUE CONTROL...

🔍 TORQUE HISTORY DIAGNOSTIC:
  torque_history shape: (6836, 20)
  Expected: (6836, 20)

  Sample torques at key steps:
    Step 0: max=2.91 Nm, nonzero=3/20
      MiddleFinger-3rd-pitch        :    +1.75 Nm
      IndexFinger-2nd-pitch         :    +2.91 Nm
    Step 500: max=2.91 Nm, nonzero=5/20
      MiddleFinger-1st-pitch        :    +0.19 Nm
      MiddleFinger-3rd-pitch        :    +1.75 Nm
      IndexFinger-1st-pitch         :    +2.22 Nm
      IndexFinger-2nd-pitch         :    +2.91 Nm
      Thumb-1st-pitch               :    +1.48 Nm
    Step 2500: max=22.96 Nm, nonzero=7/20
      MiddleFinger-1st-pitch        :   +22.96 Nm
      MiddleFinger-2nd-pitch        :    +1.08 Nm
      MiddleFinger-3rd-pitch        :    +1.75 Nm
      IndexFinger-1st-pitch         :    +0.51 Nm
      IndexFinger-2nd-pitch         :    +2.92 Nm
      Thumb-1st-pitch               :    +0.33 Nm
    Step 5000: max=19.27 Nm, nonzero=10/20
      MiddleFinger_1st_roll         :    -0.33 Nm
      MiddleFinger-1st-pitch        :   -19.27 Nm
      MiddleFinger-2nd-pitch        :    -9.41 Nm
      MiddleFinger-3rd-pitch        :    -1.79 Nm
      IndexFinger-1st-pitch         :    +0.27 Nm
      IndexFinger-2nd-pitch         :    +2.91 Nm
      thumb_1st_yaw                 :    +1.13 Nm
      Thumb-1st-pitch               :   -16.37 Nm
      Thumb-2nd-pitch               :    -8.23 Nm
      Thumb-3rd-pitch               :    -4.02 Nm

Initial position set. RMS: 0.0000 rad (should be ~0)
==================================================

Starting forward simulation...

🔍 PHYSICS DEBUG at step 0:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 3/20, Max: 2.91 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
ERROR: zero-size array to reduction operation maximum which has no identity
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/inverse_to_forward_validation.py", line 741, in <module>
    max_ctrl = np.max(np.abs(data.ctrl[:model.nu]))
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/numpy/core/fromnumeric.py", line 2810, in max
    return _wrapreduction(a, np.maximum, 'max', axis, None, out,
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/numpy/core/fromnumeric.py", line 88, in _wrapreduction
    return ufunc.reduce(obj, axis, dtype, out, **passkwargs)
ValueError: zero-size array to reduction operation maximum which has no identity

Cleaned up temporary file: tmpwu_c7o5o.xml

Mode 4 validation complete.
