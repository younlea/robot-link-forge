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

Applying 2.0x safety margin...
  → Inverse dynamics only accounts for ideal motion
  → Forward simulation needs extra for friction, damping, numerical errors
  Adjusted force limits: 17.57 Nm (avg), 76.79 Nm (max)

======================================================================
PHASE 2: FORWARD SIMULATION WITH COMPUTED LIMITS
======================================================================
  Actuator 0: forcelimit = ±0.01 Nm
  Actuator 1: forcelimit = ±0.01 Nm
  Actuator 2: forcelimit = ±0.00 Nm
  Actuator 3: forcelimit = ±0.00 Nm
  Actuator 4: forcelimit = ±0.01 Nm
  Actuator 5: forcelimit = ±0.01 Nm
  Actuator 6: forcelimit = ±0.00 Nm
  Actuator 7: forcelimit = ±0.00 Nm
  Actuator 8: forcelimit = ±1.15 Nm
  Actuator 9: forcelimit = ±63.85 Nm
  Actuator 10: forcelimit = ±30.63 Nm
  Actuator 11: forcelimit = ±7.69 Nm
  Actuator 12: forcelimit = ±0.57 Nm
  Actuator 13: forcelimit = ±76.79 Nm
  Actuator 14: forcelimit = ±31.21 Nm
  Actuator 15: forcelimit = ±14.63 Nm
  Actuator 16: forcelimit = ±4.52 Nm
  Actuator 17: forcelimit = ±68.96 Nm
  Actuator 18: forcelimit = ±34.56 Nm
  Actuator 19: forcelimit = ±16.84 Nm

Control gains: kp=50.0, kv=10.0
Note: Gains reduced to work within computed torque limits

Starting forward simulation...
  T=0.00s: RMS error=0.0895 rad, Max torque=0.00 Nm
    Worst errors: IndexFinger-2nd-pitch=18.4° (0.0Nm)  MiddleFinger-3rd-pitch=13.3° (0.0Nm)  IndexFinger-1st-pitch=2.8° (0.0Nm)  
  T=2.00s: RMS error=0.6375 rad, Max torque=76.79 Nm
    Worst errors: IndexFinger-3rd-pitch=95.1° (14.6Nm)  MiddleFinger-3rd-pitch=81.2° (7.7Nm)  MiddleFinger-1st-pitch=72.8° (63.9Nm)  
    Saturated: LittleFinger-1st_roll, LittleFinger-1st-pitch, LittleFinger-2nd-pitch, LittleFinger-3rd-pitch, RingFinger-1st_roll

Simulation complete!

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 0.581137 rad (33.297 deg)
  Maximum RMS Error: 0.891209 rad (51.063 deg)

Torque Usage:
  Average Torque: 76.74 Nm
  Peak Torque: 76.79 Nm
  Computed Limit: 76.79 Nm
  Usage: 100.0%

======================================================================
✗ FAILED: Cannot track trajectory even with computed torques
  → Problem is in physics model or timestep
  → Check: mass, inertia, timestep, solver settings
======================================================================
