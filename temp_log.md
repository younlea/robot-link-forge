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
Using feedforward control: data.ctrl = inverse_dynamics_torque
If this works perfectly, it validates the physics model.
If it fails, there's a mismatch between inverse and forward dynamics.

Stabilizing initial pose with inverse dynamics torques...
WARNING: Nan, Inf or huge value in QACC at DOF 13. The simulation is unstable. Time = 0.0070.

  Warning: LittleFinger-1st_roll has 186.0° initial error
  Warning: LittleFinger-1st-pitch has 84.1° initial error
  Warning: LittleFinger-2nd-pitch has 247.1° initial error
  Warning: LittleFinger-3rd-pitch has 299.8° initial error
  Warning: RingFinger-1st_roll has 119.1° initial error
  Warning: RingFinger-1st-pitch has 256.0° initial error
  Warning: RingFinger-2nd-pitch has 53.5° initial error
  Warning: RingFinger-3rd-pitch has 321.2° initial error
  Warning: MiddleFinger_1st_roll has 183.8° initial error
  Warning: MiddleFinger-1st-pitch has 73.7° initial error
  Warning: MiddleFinger-2nd-pitch has 1286.5° initial error
  Warning: MiddleFinger-3rd-pitch has 113.5° initial error
  Warning: IndexFinger_1st_roll has 199.5° initial error
  Warning: IndexFinger-1st-pitch has 302.9° initial error
  Warning: IndexFinger-2nd-pitch has 514.8° initial error
  Warning: IndexFinger-3rd-pitch has 173.5° initial error
  Warning: thumb_1st_yaw has 303.9° initial error
  Warning: Thumb-1st-pitch has 113.1° initial error
  Warning: Thumb-2nd-pitch has 185.4° initial error
  Warning: Thumb-3rd-pitch has 118.9° initial error
Initial RMS error after stabilization: 6.3721 rad (365.10°)
  ⚠️  Large initial error! Physics may be unstable or gains too weak

Starting forward simulation...
  T=0.00s: RMS error=14.7882 rad, Max torque=2.91 Nm
    Worst errors: MiddleFinger-2nd-pitch=2005.1° (0.0Nm)  RingFinger-1st_roll=1850.4° (0.0Nm)  MiddleFinger-1st-pitch=1815.4° (0.0Nm)  
  T=3.00s: RMS error=6.8917 rad, Max torque=2.92 Nm
    Worst errors: MiddleFinger-1st-pitch=922.7° (0.0Nm)  MiddleFinger-2nd-pitch=618.8° (0.0Nm)  IndexFinger_1st_roll=553.8° (0.0Nm)  
  T=4.00s: RMS error=14.6863 rad, Max torque=12.76 Nm
    Worst errors: MiddleFinger-2nd-pitch=2420.0° (0.0Nm)  RingFinger-1st-pitch=1729.8° (0.0Nm)  MiddleFinger_1st_roll=1594.9° (0.0Nm)  
  T=5.50s: RMS error=80.8857 rad, Max torque=2.92 Nm
    Worst errors: IndexFinger-1st-pitch=14188.2° (0.5Nm)  MiddleFinger-2nd-pitch=10782.5° (0.0Nm)  IndexFinger-2nd-pitch=7568.6° (2.9Nm)  
  T=6.50s: RMS error=6.8256 rad, Max torque=2.92 Nm
    Worst errors: MiddleFinger-1st-pitch=874.5° (0.0Nm)  MiddleFinger-2nd-pitch=621.3° (0.0Nm)  IndexFinger_1st_roll=559.7° (0.0Nm)  

Simulation complete!

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 74.724954 rad (4281.425 deg)
  Maximum RMS Error: 2201.461261 rad (126134.439 deg)

Torque Usage:
  Average Torque: 3.17 Nm
  Peak Torque: 37.88 Nm
  Computed Limit: 76.79 Nm
  Usage: 49.3%

======================================================================
✗ FAILED: Cannot track trajectory even with computed torques
  → Problem is in physics model or timestep
  → Check: mass, inertia, timestep, solver settings
======================================================================

Plot saved to: mode4_validation.png

