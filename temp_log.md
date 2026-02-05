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
MODE 4 SCRIPT VERSION: 20260205_161239
Generated: 2026-02-05 16:12:39
======================================================================

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

======================================================================
DEBUG MODE: Running first 500 steps only (no trajectory motion)
Checking physics stability step-by-step...
======================================================================

📍 STEP 0: Initial state after mj_resetData
  model.qpos0 (first 5): [0. 0. 0. 0. 0.]
  data.qpos (first 5): [0. 0. 0. 0. 0.]
  data.ctrl (first 5): [0. 0. 0. 0. 0.]
  data.qvel (first 5): [0. 0. 0. 0. 0.]

📍 After setting ctrl = model.qpos0:
  data.ctrl (first 5): [0. 0. 0. 0. 0.]

📍 Running 500 steps with detailed logging...
  Step   0: max|qpos|=  0.322, max|qvel|=321.913, max|qacc|=470418.525
  Step   1: max|qpos|= 10.055, max|qvel|=10376.468, max|qacc|=15923521.240
  Step   2: max|qpos|= 13.010, max|qvel|=9971.733, max|qacc|=19712952.373
  Step   3: max|qpos|= 16.105, max|qvel|=19511.436, max|qacc|=29130622.796
  Step   4: max|qpos|= 45.537, max|qvel|=48256.430, max|qacc|=51521713.976
WARNING: Nan, Inf or huge value in QACC at DOF 4. The simulation is unstable. Time = 0.0080.

  Step  50: max|qpos|= 13.010, max|qvel|=9971.733, max|qacc|=19712952.373
  Step 100: max|qpos|= 45.537, max|qvel|=48256.430, max|qacc|=51521713.976
  Step 150: max|qpos|= 74.862, max|qvel|=92147.013, max|qacc|=121969513.223
  Step 200: max|qpos|=  0.322, max|qvel|=321.913, max|qacc|=470418.525
  Step 250: max|qpos|= 13.010, max|qvel|=9971.733, max|qacc|=19712952.373
  Step 300: max|qpos|= 45.537, max|qvel|=48256.430, max|qacc|=51521713.976
  Step 350: max|qpos|= 74.862, max|qvel|=92147.013, max|qacc|=121969513.223
  Step 400: max|qpos|=  0.322, max|qvel|=321.913, max|qacc|=470418.525
  Step 450: max|qpos|= 13.010, max|qvel|=9971.733, max|qacc|=19712952.373

✅ 500 steps completed without explosion
  Final max|qpos|=16.105

  Natural equilibrium found (all 20 joints):
    LittleFinger-1st_roll         : -7.7006 rad (-441.21°)
    LittleFinger-1st-pitch        : +5.6847 rad (+325.71°)
    LittleFinger-2nd-pitch        : -1.5617 rad ( -89.48°)
    LittleFinger-3rd-pitch        : +7.2628 rad (+416.13°)
    RingFinger-1st_roll           : -7.4050 rad (-424.27°)
    RingFinger-1st-pitch          : +4.9199 rad (+281.89°)
    RingFinger-2nd-pitch          : -2.2647 rad (-129.76°)
    RingFinger-3rd-pitch          : +7.4581 rad (+427.32°)
    MiddleFinger_1st_roll         : -2.7191 rad (-155.80°)
    MiddleFinger-1st-pitch        : -16.1055 rad (-922.77°)
    MiddleFinger-2nd-pitch        : +10.8001 rad (+618.80°)
    MiddleFinger-3rd-pitch        : +3.7384 rad (+214.19°)
    IndexFinger_1st_roll          : +9.7001 rad (+555.77°)
    IndexFinger-1st-pitch         : +1.6082 rad ( +92.14°)
    IndexFinger-2nd-pitch         : +1.4952 rad ( +85.67°)
    IndexFinger-3rd-pitch         : -7.7715 rad (-445.27°)
    thumb_1st_yaw                 : +2.3389 rad (+134.01°)
    Thumb-1st-pitch               : +4.5560 rad (+261.04°)
    Thumb-2nd-pitch               : +1.3905 rad ( +79.67°)
    Thumb-3rd-pitch               : +8.7764 rad (+502.85°)

======================================================================
DEBUG MODE COMPLETE - Exiting early for analysis
======================================================================
