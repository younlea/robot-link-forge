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
PHASE 2: FORWARD SIMULATION WITH COMPUTED TORQUES
======================================================================

Using pure feedforward control:
  data.ctrl = torque_history[step]
  (Direct application of inverse dynamics forces)

This tests: Can the actuators track trajectory with computed forces?

Letting physics settle at initial pose...
Initial RMS error after stabilization: 0.0000 rad (0.00°)

Starting forward simulation...
  T=0.00s: RMS error=0.0916 rad, Max torque=2.91 Nm
    Worst errors: IndexFinger-2nd-pitch=18.1° (2.9Nm)  MiddleFinger-3rd-pitch=13.3° (1.7Nm)  Thumb-1st-pitch=6.0° (0.0Nm)  
WARNING: Nan, Inf or huge value in QACC at DOF 13. The simulation is unstable. Time = 0.0070.

  T=1.00s: RMS error=7.0789 rad, Max torque=2.92 Nm
    Worst errors: MiddleFinger-1st-pitch=891.5° (0.0Nm)  thumb_1st_yaw=658.0° (0.0Nm)  MiddleFinger-2nd-pitch=618.9° (0.0Nm)  
  T=2.50s: RMS error=79.9848 rad, Max torque=2.92 Nm
    Worst errors: IndexFinger-1st-pitch=16260.0° (0.6Nm)  MiddleFinger-2nd-pitch=10792.0° (0.0Nm)  RingFinger-1st_roll=4629.7° (0.0Nm)  

Simulation complete!

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 83.709375 rad (4796.194 deg)
  Maximum RMS Error: 2367.392745 rad (135641.613 deg)

Torque Usage:
  Average Torque: 2.99 Nm
  Peak Torque: 37.88 Nm
  Computed Limit: 76.79 Nm
  Usage: 49.3%

======================================================================
✗ FAILED: Cannot track trajectory even with computed torques
  → Problem is in physics model or timestep
  → Check: mass, inertia, timestep, solver settings
======================================================================
