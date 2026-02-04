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
⚠️  WARNING: scipy not installed!
   For smooth trajectories, install scipy:
   pip install scipy
   Using fallback linear interpolation (may cause torque spikes)

Model has 20 joints, 20 actuators
Recording duration: 6.83s (6835ms)
Found 20 joints in recording

🔍 KEYFRAME TIMING:
  Number of keyframes: 5
  First keyframe time: 0.580s
  Last keyframe time: 6.835s
  Recording duration: 6.835s
  Keyframe times: 0.58s 2.09s 3.61s 5.05s 6.83s 
  Warning: Thumb-1st-pitch using linear interpolation (install scipy for better results)
  Warning: RingFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: RingFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: IndexFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: MiddleFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: IndexFinger-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: LittleFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: MiddleFinger-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: LittleFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: MiddleFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: Thumb-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: MiddleFinger_1st_roll using linear interpolation (install scipy for better results)
  Warning: IndexFinger_1st_roll using linear interpolation (install scipy for better results)
  Warning: Thumb-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: LittleFinger-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: IndexFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: thumb_1st_yaw using linear interpolation (install scipy for better results)
  Warning: LittleFinger-1st_roll using linear interpolation (install scipy for better results)
  Warning: RingFinger-1st_roll using linear interpolation (install scipy for better results)
  Warning: RingFinger-2nd-pitch using linear interpolation (install scipy for better results)

🔍 TRAJECTORY DIAGNOSTIC:
Checking if trajectory actually changes over time...
  IndexFinger-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0000 rad
    Step 2500: -1.1505 rad
    Step 5000: 0.0000 rad
    Range: 1.5708 rad
    Vel range: -1.04 to 1.03 rad/s
    Acc range: -49.44 to 98.62 rad/s²
  MiddleFinger-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0000 rad
    Step 2500: 0.0000 rad
    Step 5000: -1.5140 rad
    Range: 1.5708 rad
    Vel range: -1.09 to 0.88 rad/s
    Acc range: -52.02 to 93.97 rad/s²
  Thumb-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0000 rad
    Step 2500: -0.6891 rad
    Step 5000: -0.7526 rad
    Range: 0.9408 rad
    Vel range: -0.62 to 0.62 rad/s
    Acc range: -55.31 to 59.06 rad/s²

======================================================================
PHASE 1: INVERSE DYNAMICS ANALYSIS
======================================================================
Calculating required torques for trajectory...

🔍 DATA STRUCTURE DIAGNOSTIC:
  model.nu (actuators): 20
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
    Actuator[0] LittleFinger-1st_roll_act → Joint LittleFinger-1st_roll
    Actuator[1] LittleFinger-1st-pitch_act → Joint LittleFinger-1st-pitch
    Actuator[2] LittleFinger-2nd-pitch_act → Joint LittleFinger-2nd-pitch
    Actuator[3] LittleFinger-3rd-pitch_act → Joint LittleFinger-3rd-pitch
    Actuator[4] RingFinger-1st_roll_act   → Joint RingFinger-1st_roll
    Actuator[5] RingFinger-1st-pitch_act  → Joint RingFinger-1st-pitch
    Actuator[6] RingFinger-2nd-pitch_act  → Joint RingFinger-2nd-pitch
    Actuator[7] RingFinger-3rd-pitch_act  → Joint RingFinger-3rd-pitch
    Actuator[8] MiddleFinger_1st_roll_act → Joint MiddleFinger_1st_roll
    Actuator[9] MiddleFinger-1st-pitch_act → Joint MiddleFinger-1st-pitch
    Actuator[10] MiddleFinger-2nd-pitch_act → Joint MiddleFinger-2nd-pitch
    Actuator[11] MiddleFinger-3rd-pitch_act → Joint MiddleFinger-3rd-pitch
    Actuator[12] IndexFinger_1st_roll_act  → Joint IndexFinger_1st_roll
    Actuator[13] IndexFinger-1st-pitch_act → Joint IndexFinger-1st-pitch
    Actuator[14] IndexFinger-2nd-pitch_act → Joint IndexFinger-2nd-pitch
    Actuator[15] IndexFinger-3rd-pitch_act → Joint IndexFinger-3rd-pitch
    Actuator[16] thumb_1st_yaw_act         → Joint thumb_1st_yaw
    Actuator[17] Thumb-1st-pitch_act       → Joint Thumb-1st-pitch
    Actuator[18] Thumb-2nd-pitch_act       → Joint Thumb-2nd-pitch
    Actuator[19] Thumb-3rd-pitch_act       → Joint Thumb-3rd-pitch

======================================================================
  Step 0/6836: Max torque so far = 2.91 Nm
  Step 500/6836: Max torque so far = 2.91 Nm
  Step 1000/6836: Max torque so far = 2.92 Nm
  Step 1500/6836: Max torque so far = 2.92 Nm
  Step 2000/6836: Max torque so far = 12.76 Nm

🔍 DEBUG Step 2500 - qfrc_inverse values:
  qfrc_inverse shape: (20,)
  qfrc_inverse max: 2.92 Nm
    MiddleFinger-3rd-pitch         DOF[11]:    +1.75 Nm
    IndexFinger-1st-pitch          DOF[13]:    +0.55 Nm
    IndexFinger-2nd-pitch          DOF[14]:    +2.92 Nm
    Thumb-1st-pitch                DOF[17]:    +0.27 Nm
  Step 2500/6836: Max torque so far = 35.76 Nm
  Step 3000/6836: Max torque so far = 35.76 Nm
  Step 3500/6836: Max torque so far = 35.76 Nm
  Step 4000/6836: Max torque so far = 35.76 Nm
  Step 4500/6836: Max torque so far = 35.76 Nm
  Step 5000/6836: Max torque so far = 35.76 Nm
  Step 5500/6836: Max torque so far = 35.76 Nm
  Step 6000/6836: Max torque so far = 35.76 Nm
  Step 6500/6836: Max torque so far = 35.76 Nm

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
  MiddleFinger_1st_roll          |     0.57
  MiddleFinger-1st-pitch         |    28.99
  MiddleFinger-2nd-pitch         |    13.94
  MiddleFinger-3rd-pitch         |     3.40
  IndexFinger_1st_roll           |     0.28
  IndexFinger-1st-pitch          |    35.76
  IndexFinger-2nd-pitch          |    14.39
  IndexFinger-3rd-pitch          |     6.93
  thumb_1st_yaw                  |     2.10
  Thumb-1st-pitch                |    32.84
  Thumb-2nd-pitch                |    16.51
  Thumb-3rd-pitch                |     8.08

  Overall Peak Torque: 35.76 Nm
  Average Peak Torque: 8.19 Nm

Applying 2.0x safety margin...
  → Inverse dynamics only accounts for ideal motion
  → Forward simulation needs extra for friction, damping, numerical errors
  Adjusted force limits: 16.38 Nm (avg), 71.53 Nm (max)

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

🔍 TORQUE HISTORY DIAGNOSTIC:
  torque_history shape: (6836, 20)
  Expected: (6836, 20)

  Sample torques at key steps:
    Step 0: max=2.91 Nm, nonzero=3/20
      MiddleFinger-3rd-pitch        :    +1.75 Nm
      IndexFinger-2nd-pitch         :    +2.91 Nm
    Step 500: max=2.91 Nm, nonzero=3/20
      MiddleFinger-3rd-pitch        :    +1.75 Nm
      IndexFinger-2nd-pitch         :    +2.91 Nm
    Step 2500: max=2.92 Nm, nonzero=4/20
      MiddleFinger-3rd-pitch        :    +1.75 Nm
      IndexFinger-1st-pitch         :    +0.55 Nm
      IndexFinger-2nd-pitch         :    +2.92 Nm
      Thumb-1st-pitch               :    +0.27 Nm
    Step 5000: max=16.21 Nm, nonzero=9/20
      MiddleFinger_1st_roll         :    -0.32 Nm
      MiddleFinger-1st-pitch        :   -16.21 Nm
      MiddleFinger-2nd-pitch        :    -7.73 Nm
      MiddleFinger-3rd-pitch        :    -1.14 Nm
      IndexFinger-2nd-pitch         :    +2.91 Nm
      thumb_1st_yaw                 :    -0.61 Nm
      Thumb-1st-pitch               :   -14.23 Nm
      Thumb-2nd-pitch               :    -7.09 Nm
      Thumb-3rd-pitch               :    -3.47 Nm
  T=0.00s: RMS error=0.0890 rad, Max force=2.91 Nm
    Worst errors: IndexFinger-2nd-pitch=18.3° (2.9Nm)  MiddleFinger-3rd-pitch=13.3° (1.7Nm)  IndexFinger-1st-pitch=2.8° (0.0Nm)  
WARNING: Nan, Inf or huge value in QACC at DOF 9. The simulation is unstable. Time = 0.0070.

  T=0.50s: RMS error=6.8910 rad, Max force=2.91 Nm
    Worst errors: MiddleFinger-1st-pitch=922.6° (0.0Nm)  MiddleFinger-2nd-pitch=618.8° (0.0Nm)  IndexFinger_1st_roll=560.9° (0.0Nm)  
  T=1.00s: RMS error=123.5766 rad, Max force=2.92 Nm
    Worst errors: RingFinger-1st_roll=31080.2° (0.0Nm)  MiddleFinger-2nd-pitch=4049.2° (0.0Nm)  RingFinger-1st-pitch=3129.3° (0.0Nm)  
  T=1.50s: RMS error=6.4167 rad, Max force=2.92 Nm
    Worst errors: IndexFinger-2nd-pitch=746.1° (2.9Nm)  RingFinger-3rd-pitch=570.7° (0.0Nm)  LittleFinger-3rd-pitch=570.7° (0.0Nm)  
  T=2.00s: RMS error=15.6103 rad, Max force=12.76 Nm
    Worst errors: MiddleFinger-2nd-pitch=2423.6° (0.0Nm)  RingFinger-1st_roll=2002.2° (0.0Nm)  RingFinger-1st-pitch=1358.4° (0.0Nm)  
  T=2.50s: RMS error=15.5534 rad, Max force=2.92 Nm
    Worst errors: MiddleFinger-2nd-pitch=2423.6° (0.0Nm)  RingFinger-1st_roll=2002.2° (0.0Nm)  RingFinger-1st-pitch=1358.4° (0.0Nm)  
  T=3.00s: RMS error=3.6582 rad, Max force=2.92 Nm
    Worst errors: IndexFinger-2nd-pitch=576.2° (2.9Nm)  MiddleFinger-3rd-pitch=551.5° (1.7Nm)  IndexFinger-3rd-pitch=364.8° (0.0Nm)  
  T=3.50s: RMS error=12.0977 rad, Max force=2.91 Nm
    Worst errors: MiddleFinger_1st_roll=2591.2° (0.0Nm)  MiddleFinger-2nd-pitch=1291.5° (0.0Nm)  IndexFinger-2nd-pitch=523.3° (2.9Nm)  
  T=4.00s: RMS error=3.6939 rad, Max force=2.91 Nm
    Worst errors: IndexFinger-2nd-pitch=576.2° (2.9Nm)  MiddleFinger-3rd-pitch=551.5° (1.7Nm)  IndexFinger-3rd-pitch=364.8° (0.0Nm)  
  T=4.50s: RMS error=6.8209 rad, Max force=2.91 Nm
    Worst errors: MiddleFinger-1st-pitch=866.4° (0.5Nm)  MiddleFinger-2nd-pitch=626.6° (0.1Nm)  IndexFinger_1st_roll=560.9° (0.0Nm)  
  T=5.00s: RMS error=6.8887 rad, Max force=16.21 Nm
    Worst errors: IndexFinger-2nd-pitch=746.1° (2.9Nm)  Thumb-3rd-pitch=573.0° (3.5Nm)  RingFinger-3rd-pitch=570.7° (0.0Nm)  
  T=5.50s: RMS error=19.0201 rad, Max force=2.91 Nm
    Worst errors: MiddleFinger_1st_roll=3129.6° (0.0Nm)  MiddleFinger-2nd-pitch=2411.6° (0.1Nm)  RingFinger-1st_roll=2008.6° (0.0Nm)  
  T=6.00s: RMS error=12.2857 rad, Max force=2.91 Nm
    Worst errors: MiddleFinger_1st_roll=2643.7° (0.0Nm)  MiddleFinger-2nd-pitch=1297.5° (0.1Nm)  IndexFinger-2nd-pitch=523.8° (2.9Nm)  
  T=6.50s: RMS error=12.2571 rad, Max force=2.91 Nm
    Worst errors: MiddleFinger_1st_roll=2642.4° (0.0Nm)  MiddleFinger-2nd-pitch=1294.0° (0.1Nm)  IndexFinger-2nd-pitch=523.8° (2.9Nm)  

Simulation complete!

💾 Saving Phase 2 control history to CSV...
  Saved 684 samples (every 10 steps) to phase2_control_applied.csv

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 57.331805 rad (3284.870 deg)
  Maximum RMS Error: 2783.600743 rad (159488.574 deg)

Torque Usage:
  Average Torque: 3.39 Nm
  Peak Torque: 35.76 Nm
  Computed Limit: 71.53 Nm
  Usage: 50.0%

======================================================================
✗ FAILED: Cannot track trajectory even with computed torques
  → Problem is in physics model or timestep
  → Check: mass, inertia, timestep, solver settings
======================================================================

Plot saved to: mode4_validation.png
