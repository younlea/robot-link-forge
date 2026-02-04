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

🔍 TRAJECTORY DIAGNOSTIC:
Checking if trajectory actually changes over time...
  IndexFinger-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0000 rad
    Step 2500: -1.1505 rad
    Step 5000: 0.0000 rad
    Range: 1.5708 rad
  MiddleFinger-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0000 rad
    Step 2500: 0.0000 rad
    Step 5000: -1.5140 rad
    Range: 1.5708 rad
  Thumb-1st-pitch:
    Step 0:    0.0000 rad
    Step 500:  0.0000 rad
    Step 2500: -0.6891 rad
    Step 5000: -0.7526 rad
    Range: 0.9408 rad

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
  Step 2500/6836: Max torque so far = 38.40 Nm
  Step 3000/6836: Max torque so far = 38.40 Nm
  Step 3500/6836: Max torque so far = 38.40 Nm
  Step 4000/6836: Max torque so far = 38.40 Nm
  Step 4500/6836: Max torque so far = 38.40 Nm
  Step 5000/6836: Max torque so far = 38.40 Nm
  Step 5500/6836: Max torque so far = 38.40 Nm
  Step 6000/6836: Max torque so far = 38.40 Nm
  Step 6500/6836: Max torque so far = 38.40 Nm

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
  T=0.00s: RMS error=0.0890 rad, Max torque=0.00 Nm
    Worst errors: IndexFinger-2nd-pitch=18.3° (0.0Nm)  MiddleFinger-3rd-pitch=13.3° (0.0Nm)  IndexFinger-1st-pitch=2.8° (0.0Nm)  
WARNING: Nan, Inf or huge value in QACC at DOF 9. The simulation is unstable. Time = 0.0070.

  T=2.00s: RMS error=0.4036 rad, Max torque=0.00 Nm
    Worst errors: IndexFinger-1st-pitch=87.3° (0.0Nm)  Thumb-1st-pitch=50.6° (0.0Nm)  IndexFinger-2nd-pitch=18.4° (0.0Nm)  
  T=3.50s: RMS error=6.8945 rad, Max torque=0.00 Nm
    Worst errors: MiddleFinger-1st-pitch=922.6° (0.0Nm)  MiddleFinger-2nd-pitch=618.8° (0.0Nm)  IndexFinger_1st_roll=560.9° (0.0Nm)  
  T=6.00s: RMS error=79.9531 rad, Max torque=0.00 Nm
    Worst errors: RingFinger-2nd-pitch=14723.3° (0.0Nm)  MiddleFinger_1st_roll=11354.0° (0.0Nm)  MiddleFinger-1st-pitch=5179.8° (0.0Nm)  

Simulation complete!

💾 Saving Phase 2 control history to CSV...
  Saved 152 samples (every 10 steps) to phase2_control_applied.csv

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 46.491740 rad (2663.780 deg)
  Maximum RMS Error: 1504.295978 rad (86189.811 deg)

Torque Usage:
  Average Torque: 0.00 Nm
  Peak Torque: 0.00 Nm
  Computed Limit: 76.79 Nm
  Usage: 0.0%

======================================================================
✗ FAILED: Cannot track trajectory even with computed torques
  → Problem is in physics model or timestep
  → Check: mass, inertia, timestep, solver settings
======================================================================
