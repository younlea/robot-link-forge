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
  Warning: Thumb-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: IndexFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: RingFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: thumb_1st_yaw using linear interpolation (install scipy for better results)
  Warning: LittleFinger-1st_roll using linear interpolation (install scipy for better results)
  Warning: LittleFinger-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: RingFinger-1st_roll using linear interpolation (install scipy for better results)
  Warning: Thumb-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: RingFinger-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: Thumb-1st-pitch using linear interpolation (install scipy for better results)
  Warning: MiddleFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: IndexFinger_1st_roll using linear interpolation (install scipy for better results)
  Warning: MiddleFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: MiddleFinger-2nd-pitch using linear interpolation (install scipy for better results)
  Warning: MiddleFinger_1st_roll using linear interpolation (install scipy for better results)
  Warning: LittleFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: RingFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: IndexFinger-1st-pitch using linear interpolation (install scipy for better results)
  Warning: LittleFinger-3rd-pitch using linear interpolation (install scipy for better results)
  Warning: IndexFinger-2nd-pitch using linear interpolation (install scipy for better results)

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
PHASE 2: KINEMATIC REPLAY WITH PD CONTROL
======================================================================

Using position control (not force control):
  Set data.ctrl to target positions from trajectory
  Let position actuators (kp=200, kd=20) drive joints
  This tests: Can the robot track with strong control?

Note: This is NOT pure force feedforward
  Using PD position control for stability
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

🔍 DEBUG at step 0:
  Using position control (not force)
  Nonzero position commands: 0/20
    LittleFinger-1st_roll         : target= +0.0000, current= +0.0000, ctrl= +0.0000 rad
    LittleFinger-1st-pitch        : target= +0.0000, current= +0.0000, ctrl= +0.0000 rad
    LittleFinger-2nd-pitch        : target= +0.0000, current= +0.0000, ctrl= +0.0000 rad
    LittleFinger-3rd-pitch        : target= +0.0000, current= +0.0000, ctrl= +0.0000 rad
    RingFinger-1st_roll           : target= +0.0000, current= +0.0000, ctrl= +0.0000 rad
  T=0.00s: RMS error=0.0895 rad (5.1°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=18.4° (cmd=0.00rad)  MiddleFinger-3rd-pitch=13.3° (cmd=0.00rad)  IndexFinger-1st-pitch=2.8° (cmd=0.00rad)  
WARNING: Nan, Inf or huge value in QACC at DOF 4. The simulation is unstable. Time = 0.0080.


🔍 DEBUG at step 500:
  Using position control (not force)
  Nonzero position commands: 0/20
    LittleFinger-1st_roll         : target= +0.0000, current= -7.7006, ctrl= +0.0000 rad
    LittleFinger-1st-pitch        : target= +0.0000, current= +5.6847, ctrl= +0.0000 rad
    LittleFinger-2nd-pitch        : target= +0.0000, current= -1.5617, ctrl= +0.0000 rad
    LittleFinger-3rd-pitch        : target= +0.0000, current= +7.2628, ctrl= +0.0000 rad
    RingFinger-1st_roll           : target= +0.0000, current= -7.4050, ctrl= +0.0000 rad
  T=0.50s: RMS error=12.1461 rad (695.9°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=2609.1° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1291.6° (cmd=0.00rad)  IndexFinger-2nd-pitch=507.8° (cmd=0.00rad)  
  T=1.00s: RMS error=30.5322 rad (1749.4°), Max cmd=0.44 rad
    Worst errors: MiddleFinger-1st-pitch=4289.3° (cmd=0.00rad)  RingFinger-1st-pitch=3559.4° (cmd=0.00rad)  MiddleFinger-2nd-pitch=3453.8° (cmd=0.00rad)  
  T=1.50s: RMS error=23.2948 rad (1334.7°), Max cmd=0.96 rad
    Worst errors: Thumb-2nd-pitch=3901.1° (cmd=0.00rad)  MiddleFinger-2nd-pitch=2396.3° (cmd=0.00rad)  thumb_1st_yaw=2249.7° (cmd=0.00rad)  
  T=2.00s: RMS error=15.0564 rad (862.7°), Max cmd=1.47 rad
    Worst errors: MiddleFinger-2nd-pitch=2396.3° (cmd=0.00rad)  MiddleFinger_1st_roll=1817.5° (cmd=0.00rad)  RingFinger-1st-pitch=1769.9° (cmd=0.00rad)  
  T=2.50s: RMS error=7.5365 rad (431.8°), Max cmd=1.15 rad
    Worst errors: MiddleFinger-1st-pitch=922.8° (cmd=0.00rad)  MiddleFinger-2nd-pitch=618.8° (cmd=0.00rad)  Thumb-1st-pitch=591.5° (cmd=0.69rad)  
  T=3.00s: RMS error=3.6606 rad (209.7°), Max cmd=0.63 rad
    Worst errors: IndexFinger-2nd-pitch=576.1° (cmd=0.00rad)  MiddleFinger-3rd-pitch=551.4° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=3.50s: RMS error=3.6935 rad (211.6°), Max cmd=0.12 rad
    Worst errors: IndexFinger-2nd-pitch=576.1° (cmd=0.00rad)  MiddleFinger-3rd-pitch=551.4° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=4.00s: RMS error=21.2833 rad (1219.4°), Max cmd=0.42 rad
    Worst errors: Thumb-2nd-pitch=2995.6° (cmd=0.00rad)  MiddleFinger-1st-pitch=2345.4° (cmd=0.42rad)  thumb_1st_yaw=2186.7° (cmd=0.11rad)  
  T=4.50s: RMS error=6.7174 rad (384.9°), Max cmd=0.97 rad
    Worst errors: IndexFinger-2nd-pitch=745.4° (cmd=0.00rad)  Thumb-3rd-pitch=573.2° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=5.00s: RMS error=6.8186 rad (390.7°), Max cmd=1.51 rad
    Worst errors: IndexFinger-2nd-pitch=745.4° (cmd=0.00rad)  Thumb-3rd-pitch=573.1° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=5.50s: RMS error=14.6723 rad (840.7°), Max cmd=1.18 rad
    Worst errors: MiddleFinger-2nd-pitch=2070.2° (cmd=0.17rad)  Thumb-2nd-pitch=1854.6° (cmd=0.00rad)  MiddleFinger-1st-pitch=1574.5° (cmd=1.18rad)  
  T=6.00s: RMS error=6.6654 rad (381.9°), Max cmd=0.74 rad
    Worst errors: IndexFinger-2nd-pitch=745.4° (cmd=0.00rad)  Thumb-3rd-pitch=573.2° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=6.50s: RMS error=1099.6221 rad (63003.7°), Max cmd=0.30 rad
    Worst errors: RingFinger-1st-pitch=272514.9° (cmd=0.00rad)  Thumb-1st-pitch=53262.8° (cmd=0.15rad)  MiddleFinger-2nd-pitch=36659.8° (cmd=0.04rad)  

Simulation complete!

💾 Saving Phase 2 control history to CSV...
  Saved 684 samples (every 10 steps) to phase2_control_applied.csv

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 70.232855 rad (4024.046 deg)
  Maximum RMS Error: 2206.704256 rad (126434.841 deg)

Position Control (PD kp=200, kd=20):
  Average Command: 0.62 rad
  Peak Command: 1.57 rad

======================================================================
✗ FAILED: Large tracking errors even with position control
  → Trajectory may be too fast for physics timestep
  → Or robot model has issues (mass/inertia/constraints)
======================================================================
