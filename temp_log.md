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

  T=1.00s: RMS error=6.4881 rad, Max torque=2.91 Nm
    Worst errors: IndexFinger-2nd-pitch=745.5° (2.9Nm)  RingFinger-3rd-pitch=570.4° (0.0Nm)  LittleFinger-3rd-pitch=570.3° (0.0Nm)  
  T=1.50s: RMS error=6.9027 rad, Max torque=2.91 Nm
    Worst errors: MiddleFinger-1st-pitch=922.7° (0.0Nm)  MiddleFinger-2nd-pitch=618.8° (0.0Nm)  IndexFinger_1st_roll=552.8° (0.0Nm)  
  T=2.00s: RMS error=12.3248 rad, Max torque=2.92 Nm
    Worst errors: MiddleFinger_1st_roll=2606.0° (0.0Nm)  MiddleFinger-2nd-pitch=1291.5° (0.0Nm)  thumb_1st_yaw=560.2° (0.0Nm)  
  T=2.50s: RMS error=15.0454 rad, Max torque=2.92 Nm
    Worst errors: MiddleFinger-2nd-pitch=2005.1° (0.0Nm)  RingFinger-1st_roll=1850.4° (0.0Nm)  MiddleFinger-1st-pitch=1815.4° (0.0Nm)  
  T=4.50s: RMS error=6.5116 rad, Max torque=2.92 Nm
    Worst errors: MiddleFinger-2nd-pitch=1294.3° (0.0Nm)  IndexFinger-2nd-pitch=509.3° (2.9Nm)  RingFinger-3rd-pitch=321.2° (0.0Nm)  

Simulation complete!

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 73.316517 rad (4200.727 deg)
  Maximum RMS Error: 2447.263134 rad (140217.849 deg)

Torque Usage:
  Average Torque: 3.19 Nm
  Peak Torque: 38.40 Nm
  Computed Limit: 76.79 Nm
  Usage: 50.0%

======================================================================
✗ FAILED: Cannot track trajectory even with computed torques
  → Problem is in physics model or timestep
  → Check: mass, inertia, timestep, solver settings
======================================================================

Plot saved to: mode4_validation.png
