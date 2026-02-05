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
Model has 20 joints, 20 actuators
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
WARNING: Nan, Inf or huge value in QACC at DOF 4. The simulation is unstable. Time = 0.0080.

  Warning: LittleFinger-1st_roll has 441.2° initial error
  Warning: LittleFinger-1st-pitch has 325.7° initial error
  Warning: LittleFinger-2nd-pitch has 89.5° initial error
  Warning: LittleFinger-3rd-pitch has 416.1° initial error
  Warning: RingFinger-1st_roll has 424.3° initial error
  Warning: RingFinger-1st-pitch has 281.9° initial error
  Warning: RingFinger-2nd-pitch has 129.8° initial error
  Warning: RingFinger-3rd-pitch has 427.3° initial error
  Warning: MiddleFinger_1st_roll has 155.8° initial error
  Warning: MiddleFinger-1st-pitch has 922.8° initial error
  Warning: MiddleFinger-2nd-pitch has 618.8° initial error
  Warning: MiddleFinger-3rd-pitch has 214.2° initial error
  Warning: IndexFinger_1st_roll has 555.8° initial error
  Warning: IndexFinger-1st-pitch has 92.1° initial error
  Warning: IndexFinger-2nd-pitch has 85.7° initial error
  Warning: IndexFinger-3rd-pitch has 445.3° initial error
  Warning: thumb_1st_yaw has 134.0° initial error
  Warning: Thumb-1st-pitch has 261.0° initial error
  Warning: Thumb-2nd-pitch has 79.7° initial error
  Warning: Thumb-3rd-pitch has 502.9° initial error
Initial RMS error after stabilization: 6.8865 rad (394.57°)
  ⚠️  Large initial error! Physics may be unstable or gains too weak

Starting forward simulation...

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

🔍 DEBUG at step 0:
  Using position control (not force)
  Nonzero position commands: 0/20
    LittleFinger-1st_roll         : target= +0.0000, current= -7.7006, ctrl= +0.0000 rad
    LittleFinger-1st-pitch        : target= +0.0000, current= +5.6847, ctrl= +0.0000 rad
    LittleFinger-2nd-pitch        : target= +0.0000, current= -1.5617, ctrl= +0.0000 rad
    LittleFinger-3rd-pitch        : target= +0.0000, current= +7.2628, ctrl= +0.0000 rad
    RingFinger-1st_roll           : target= +0.0000, current= -7.4050, ctrl= +0.0000 rad
  T=0.00s: RMS error=12.1461 rad (695.9°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=2609.1° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1291.6° (cmd=0.00rad)  IndexFinger-2nd-pitch=507.8° (cmd=0.00rad)  

🔍 DEBUG at step 500:
  Using position control (not force)
  Nonzero position commands: 2/20
    LittleFinger-1st_roll         : target= +0.0000, current= -0.8166, ctrl= +0.0000 rad
    LittleFinger-1st-pitch        : target= +0.0000, current= -1.9201, ctrl= +0.0000 rad
    LittleFinger-2nd-pitch        : target= +0.0000, current= +3.2085, ctrl= +0.0000 rad
    LittleFinger-3rd-pitch        : target= +0.0000, current= +3.8414, ctrl= +0.0000 rad
    RingFinger-1st_roll           : target= +0.0000, current= +3.0935, ctrl= +0.0000 rad
  T=0.50s: RMS error=24.6387 rad (1411.7°), Max cmd=0.02 rad
    Worst errors: MiddleFinger-2nd-pitch=3442.6° (cmd=0.00rad)  MiddleFinger-1st-pitch=3395.0° (cmd=0.00rad)  RingFinger-1st_roll=2388.1° (cmd=0.00rad)  
  T=1.00s: RMS error=0.1410 rad (8.1°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=24.5° (cmd=0.00rad)  IndexFinger-2nd-pitch=18.4° (cmd=0.00rad)  Thumb-1st-pitch=13.7° (cmd=0.00rad)  
  T=1.50s: RMS error=14.6604 rad (840.0°), Max cmd=1.07 rad
    Worst errors: IndexFinger-2nd-pitch=2467.3° (cmd=0.00rad)  IndexFinger-1st-pitch=1619.8° (cmd=1.07rad)  MiddleFinger-1st-pitch=1215.2° (cmd=0.05rad)  
  T=2.00s: RMS error=6.8117 rad (390.3°), Max cmd=1.55 rad
    Worst errors: IndexFinger-2nd-pitch=745.4° (cmd=0.00rad)  Thumb-3rd-pitch=573.0° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=2.50s: RMS error=3.6252 rad (207.7°), Max cmd=1.39 rad
    Worst errors: IndexFinger-2nd-pitch=576.1° (cmd=0.00rad)  MiddleFinger-3rd-pitch=551.4° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=3.00s: RMS error=0.2223 rad (12.7°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=46.5° (cmd=0.00rad)  Thumb-1st-pitch=19.7° (cmd=0.00rad)  IndexFinger-2nd-pitch=18.4° (cmd=0.00rad)  
  T=3.50s: RMS error=3.6961 rad (211.8°), Max cmd=0.11 rad
    Worst errors: IndexFinger-2nd-pitch=576.1° (cmd=0.00rad)  MiddleFinger-3rd-pitch=551.4° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=4.00s: RMS error=0.1439 rad (8.2°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-1st-pitch=26.1° (cmd=0.00rad)  IndexFinger-2nd-pitch=18.4° (cmd=0.00rad)  MiddleFinger-3rd-pitch=13.3° (cmd=0.00rad)  
  T=4.50s: RMS error=6.7073 rad (384.3°), Max cmd=1.15 rad
    Worst errors: IndexFinger-2nd-pitch=745.4° (cmd=0.00rad)  Thumb-3rd-pitch=573.2° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=5.00s: RMS error=61.2908 rad (3511.7°), Max cmd=1.56 rad
    Worst errors: RingFinger-2nd-pitch=14731.3° (cmd=0.00rad)  RingFinger-1st_roll=3948.0° (cmd=0.00rad)  RingFinger-1st-pitch=2385.7° (cmd=0.00rad)  
  T=5.50s: RMS error=0.3614 rad (20.7°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-1st-pitch=76.9° (cmd=0.00rad)  Thumb-1st-pitch=40.2° (cmd=0.00rad)  thumb_1st_yaw=19.3° (cmd=0.00rad)  
  T=6.00s: RMS error=6.6774 rad (382.6°), Max cmd=0.73 rad
    Worst errors: IndexFinger-2nd-pitch=745.4° (cmd=0.00rad)  Thumb-3rd-pitch=573.2° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=6.50s: RMS error=0.0980 rad (5.6°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=18.4° (cmd=0.00rad)  MiddleFinger-3rd-pitch=13.3° (cmd=0.00rad)  MiddleFinger-1st-pitch=8.6° (cmd=0.00rad)  

Simulation complete!

💾 Saving Phase 2 control history to CSV...
  Saved 684 samples (every 10 steps) to phase2_control_applied.csv

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 85.956991 rad (4924.973 deg)
  Maximum RMS Error: 2366.778219 rad (135606.403 deg)

Position Control (PD kp=200, kd=20):
  Average Command: 0.67 rad
  Peak Command: 1.57 rad

======================================================================
✗ FAILED: Large tracking errors even with position control
  → Trajectory may be too fast for physics timestep
  → Or robot model has issues (mass/inertia/constraints)
======================================================================

