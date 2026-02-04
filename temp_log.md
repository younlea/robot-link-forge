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

4. Inverse-to-Forward Validation (NEW)
   - Use Mode 1 torques as motor limits
   - Test if physics can actually track trajectory

========================================
Enter choice [0/1/2/3/4]: 4
Starting Inverse-to-Forward Validation...
Model has 20 joints, 20 actuators
Recording duration: 6.83s (6835ms)
Found 20 joints in recording

======================================================================
PHASE 1: INVERSE DYNAMICS ANALYSIS
======================================================================
Calculating required torques for trajectory...
  Step 0/6836: Max torque so far = 2.91 Nm
  Step 500/6836: Max torque so far = 2.91 Nm
  Step 1000/6836: Max torque so far = 2.92 Nm
  Step 1500/6836: Max torque so far = 2.92 Nm
  Step 2000/6836: Max torque so far = 12.76 Nm
  Step 2500/6836: Max torque so far = 38.40 Nm
  Step 3000/6836: Max torque so far = 38.40 Nm
  Step 3500/6836: Max torque so far = 38.40 Nm
  Step 4000/6836: Max torque so far = 38.40 Nm
  Step 4500/6836: Max torque so far = 38.40 Nm
  Step 5000/6836: Max torque so far = 38.40 Nm
  Step 5500/6836: Max torque so far = 38.40 Nm
  Step 6000/6836: Max torque so far = 38.40 Nm
  Step 6500/6836: Max torque so far = 38.40 Nm

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
  MiddleFinger_1st_roll          |     0.57
  MiddleFinger-1st-pitch         |    31.93
  MiddleFinger-2nd-pitch         |    15.32
  MiddleFinger-3rd-pitch         |     3.84
  IndexFinger_1st_roll           |     0.28
  IndexFinger-1st-pitch          |    38.40
  IndexFinger-2nd-pitch          |    15.61
  IndexFinger-3rd-pitch          |     7.31
  thumb_1st_yaw                  |     2.26
  Thumb-1st-pitch                |    34.48
  Thumb-2nd-pitch                |    17.28
  Thumb-3rd-pitch                |     8.42

  Overall Peak Torque: 38.40 Nm
  Average Peak Torque: 8.79 Nm

Applying 1.2x safety margin...
  Adjusted force limits: 10.54 Nm (avg), 46.08 Nm (max)

======================================================================
PHASE 2: FORWARD SIMULATION WITH COMPUTED LIMITS
======================================================================
  Actuator 0: forcelimit = ±0.01 Nm
  Actuator 1: forcelimit = ±0.01 Nm
  Actuator 2: forcelimit = ±0.00 Nm
  Actuator 3: forcelimit = ±0.00 Nm
  Actuator 4: forcelimit = ±0.00 Nm
  Actuator 5: forcelimit = ±0.01 Nm
  Actuator 6: forcelimit = ±0.00 Nm
  Actuator 7: forcelimit = ±0.00 Nm
  Actuator 8: forcelimit = ±0.69 Nm
  Actuator 9: forcelimit = ±38.31 Nm
  Actuator 10: forcelimit = ±18.38 Nm
  Actuator 11: forcelimit = ±4.61 Nm
  Actuator 12: forcelimit = ±0.34 Nm
  Actuator 13: forcelimit = ±46.08 Nm
  Actuator 14: forcelimit = ±18.73 Nm
  Actuator 15: forcelimit = ±8.78 Nm
  Actuator 16: forcelimit = ±2.71 Nm
  Actuator 17: forcelimit = ±41.38 Nm
  Actuator 18: forcelimit = ±20.74 Nm
  Actuator 19: forcelimit = ±10.10 Nm

Control gains: kp=50.0, kv=10.0
Note: Gains reduced to work within computed torque limits

Starting forward simulation...
  T=0.00s: RMS error=0.0805 rad, Max torque=0.00 Nm
  T=1.00s: RMS error=0.4839 rad, Max torque=46.08 Nm
  T=1.50s: RMS error=0.5857 rad, Max torque=46.08 Nm
  T=4.00s: RMS error=0.5448 rad, Max torque=46.08 Nm

Simulation complete!

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 0.552695 rad (31.667 deg)
  Maximum RMS Error: 0.700390 rad (40.129 deg)

Torque Usage:
  Average Torque: 46.04 Nm
  Peak Torque: 46.08 Nm
  Computed Limit: 46.08 Nm
  Usage: 100.0%

======================================================================
✗ FAILED: Cannot track trajectory even with computed torques
  → Problem is in physics model or timestep
  → Check: mass, inertia, timestep, solver settings
======================================================================
