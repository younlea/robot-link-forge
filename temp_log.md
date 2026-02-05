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
MODE 4 SCRIPT VERSION: 20260205_185755
Generated: 2026-02-05 18:57:55
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
PHASE 2: PHYSICS SIMULATION WITH TORQUE CONTROL
======================================================================

Using REAL PHYSICS SIMULATION with torque control:
  Apply torques from Phase 1 → mj_step() → simulate dynamics
  This tests: Can torque control track the trajectory?
  Goal: Prepare for motor parameter tuning (Mode 2 development)

Note: Using mj_step() for REAL dynamics simulation
  NOT kinematic playback - we want to see physics behavior!

📍 Initialization for physics simulation:
  Setting initial pose to TRAJECTORY FIRST FRAME (not qpos=0)
  This avoids geometric constraint violations
  Disabling position actuators (using ONLY qfrc_applied torque control)
  Initial pose from trajectory: [0. 0. 0. 0. 0.]
  Actuators disabled, using pure torque control
  This should be stable (no constraint violations)

=== Initial Position Check ===

Initial mismatch RMS: 0.0000 rad (0.00°)
Note: With torque control, initial mismatch is okay
==================================================

Starting forward simulation with TORQUE CONTROL...

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

Initial position set. RMS: 0.0000 rad (should be ~0)
==================================================

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

🔍 PHYSICS DEBUG at step 0:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 3/20, Max: 2.91 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos= +0.000 (target= +0.000, err= +0.000), vel= +0.000, torque=  +0.01 Nm
    LittleFinger-1st-pitch        : pos= +0.000 (target= +0.000, err= +0.000), vel= +0.000, torque=  -0.01 Nm
    LittleFinger-2nd-pitch        : pos= +0.000 (target= +0.000, err= +0.000), vel= +0.000, torque=  -0.00 Nm
  T=0.00s: RMS error=0.0890 rad (5.1°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=18.3° (cmd=0.00rad)  MiddleFinger-3rd-pitch=13.3° (cmd=0.00rad)  IndexFinger-1st-pitch=2.8° (cmd=0.00rad)  
WARNING: Nan, Inf or huge value in QACC at DOF 16. The simulation is unstable. Time = 0.0070.


🔍 PHYSICS DEBUG at step 500:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 20/20, Max: 7317.60 Nm
  ⚠️ WARNING: High velocity detected: 71727.01

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos=+12.996 (target= +0.000, err=-12.996), vel=+1515.849, torque=-164.57 Nm
    LittleFinger-1st-pitch        : pos= -3.457 (target= +0.000, err= +3.457), vel=+9294.616, torque=-926.01 Nm
    LittleFinger-2nd-pitch        : pos=-13.779 (target= +0.000, err=+13.779), vel=-16368.512, torque=+1650.63 Nm
  T=0.50s: RMS error=25.3084 rad (1450.1°), Max cmd=0.00 rad
    Worst errors: Thumb-2nd-pitch=4499.3° (cmd=0.00rad)  Thumb-1st-pitch=3643.2° (cmd=0.00rad)  thumb_1st_yaw=2339.0° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 600:
   1 active contacts detected
      middle_finger-3rd-end <-> thumb-3rd-end, penetration: 89.51mm

🔍 PHYSICS DEBUG at step 1000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 20/20, Max: 1319.79 Nm
  ⚠️ WARNING: High velocity detected: 12956.54

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos= -2.261 (target= +0.000, err= +2.261), vel=+5436.065, torque=-541.34 Nm
    LittleFinger-1st-pitch        : pos= +0.032 (target= +0.000, err= -0.032), vel=-5641.289, torque=+564.09 Nm
    LittleFinger-2nd-pitch        : pos= +3.207 (target= +0.000, err= -3.207), vel=+5254.859, torque=-528.69 Nm
  T=1.00s: RMS error=8.6074 rad (493.2°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-2nd-pitch=1651.1° (cmd=0.00rad)  MiddleFinger-1st-pitch=853.8° (cmd=0.00rad)  RingFinger-1st-pitch=600.2° (cmd=0.00rad)  
  T=1.50s: RMS error=22.4015 rad (1283.5°), Max cmd=0.00 rad
    Worst errors: Thumb-2nd-pitch=4203.6° (cmd=0.00rad)  thumb_1st_yaw=2317.1° (cmd=0.00rad)  Thumb-1st-pitch=1719.1° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 1900:
   3 active contacts detected
      world <-> thumb-3rd-end, penetration: 156.22mm
      world <-> thumb-3rd-end, penetration: 156.21mm
      world <-> thumb-3rd-end, penetration: 155.45mm

🔍 PHYSICS DEBUG at step 2000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 20/20, Max: 1953.49 Nm
  ⚠️ WARNING: High velocity detected: 19470.82

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos= +2.418 (target= +0.000, err= -2.418), vel=+4678.800, torque=-470.29 Nm
    LittleFinger-1st-pitch        : pos= -4.690 (target= +0.000, err= +4.690), vel=-4722.191, torque=+476.90 Nm
    LittleFinger-2nd-pitch        : pos= -1.379 (target= +0.000, err= +1.379), vel=-4585.626, torque=+459.94 Nm
  T=2.00s: RMS error=18.7069 rad (1071.8°), Max cmd=0.00 rad
    Worst errors: thumb_1st_yaw=3170.3° (cmd=0.00rad)  Thumb-2nd-pitch=2348.9° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1891.9° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 2400:
   1 active contacts detected
      little_finger-3rd-end <-> middle_finger-3rd-end, penetration: 33.98mm
  T=2.50s: RMS error=6.3249 rad (362.4°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=718.4° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  LittleFinger-3rd-pitch=570.7° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 3000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 20/20, Max: 2346.10 Nm
  ⚠️ WARNING: High velocity detected: 22874.06

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos= -7.697 (target= +0.000, err= +7.697), vel=-3745.629, torque=+382.27 Nm
    LittleFinger-1st-pitch        : pos= +5.673 (target= +0.000, err= -5.673), vel=+2708.776, torque=-276.56 Nm
    LittleFinger-2nd-pitch        : pos= -2.048 (target= +0.000, err= +2.048), vel=-10163.737, torque=+1018.42 Nm
  T=3.00s: RMS error=19.3785 rad (1110.3°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=4689.4° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1438.8° (cmd=0.00rad)  MiddleFinger-1st-pitch=413.2° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 3300:
   1 active contacts detected
      little_finger-3rd-end <-> middle_finger-3rd-end, penetration: 3.68mm
  T=3.50s: RMS error=6.3481 rad (363.7°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=718.3° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  LittleFinger-3rd-pitch=570.7° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 4000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 20/20, Max: 7041.26 Nm
  ⚠️ WARNING: High velocity detected: 70135.88

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos= +2.339 (target= +0.000, err= -2.339), vel=-2737.837, torque=+271.45 Nm
    LittleFinger-1st-pitch        : pos=+13.146 (target= +0.000, err=-13.146), vel=-1000.357, torque= +86.88 Nm
    LittleFinger-2nd-pitch        : pos=-10.082 (target= +0.000, err=+10.082), vel=-4957.951, torque=+505.88 Nm
  T=4.00s: RMS error=62.5971 rad (3586.5°), Max cmd=0.00 rad
    Worst errors: RingFinger-1st_roll=11147.3° (cmd=0.00rad)  IndexFinger-2nd-pitch=6050.1° (cmd=0.00rad)  IndexFinger-1st-pitch=5851.8° (cmd=0.00rad)  
  T=4.50s: RMS error=20.1725 rad (1155.8°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-2nd-pitch=4266.3° (cmd=0.00rad)  MiddleFinger_1st_roll=1959.3° (cmd=0.00rad)  MiddleFinger-1st-pitch=1325.8° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 5000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 11/20, Max: 35.43 Nm
  ⚠️ WARNING: High velocity detected: 321.91

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos= -0.000 (target= +0.000, err= +0.000), vel= -0.002, torque=  +0.01 Nm
    LittleFinger-1st-pitch        : pos= +0.000 (target= +0.000, err= -0.000), vel= +0.001, torque=  -0.01 Nm
    LittleFinger-2nd-pitch        : pos= +0.000 (target= +0.000, err= -0.000), vel= +0.000, torque=  -0.00 Nm
  T=5.00s: RMS error=3.6930 rad (211.6°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=577.4° (cmd=0.00rad)  MiddleFinger-3rd-pitch=552.3° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=5.50s: RMS error=3.6878 rad (211.3°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=577.4° (cmd=0.00rad)  MiddleFinger-3rd-pitch=552.4° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 6000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 19/20, Max: 1047.03 Nm
  ⚠️ WARNING: High velocity detected: 10398.67

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
    LittleFinger-1st_roll         : pos= +0.026 (target= +0.000, err= -0.026), vel=+25.727, torque=  -2.59 Nm
    LittleFinger-1st-pitch        : pos= -0.020 (target= +0.000, err= +0.020), vel=-19.844, torque=  +2.00 Nm
    LittleFinger-2nd-pitch        : pos= -0.005 (target= +0.000, err= +0.005), vel= -5.284, torque=  +0.53 Nm
  T=6.00s: RMS error=6.3838 rad (365.8°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=718.3° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  LittleFinger-3rd-pitch=570.7° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 6100:
   1 active contacts detected
      middle_finger-3rd-end <-> index_finger-3rd-end, penetration: 20.51mm
  T=6.50s: RMS error=7.1370 rad (408.9°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-1st-pitch=1031.7° (cmd=0.00rad)  MiddleFinger-2nd-pitch=641.0° (cmd=0.00rad)  IndexFinger_1st_roll=508.4° (cmd=0.00rad)  

Simulation complete!

💾 Saving Phase 2 control history to CSV...
  Saved 684 samples (every 10 steps) to phase2_control_applied.csv

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 58.114143 rad (3329.695 deg)
  Maximum RMS Error: 2196.915509 rad (125873.987 deg)

Torque Control (Direct Force Application):
  Average Torque: 0.00 Nm
  Peak Torque: 0.00 Nm

======================================================================
✗ FAILED: Large tracking errors even with position control
  → Trajectory may be too fast for physics timestep
  → Or robot model has issues (mass/inertia/constraints)
======================================================================
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/inverse_to_forward_validation.py", line 857, in <module>
    forces = [abs(step_data[i]) for i in range(4, len(step_data), 3)]  # Every 3rd element starting from index 4
  File "/home/younleakim/Downloads/direct_hand_parm/inverse_to_forward_validation.py", line 857, in <listcomp>
    forces = [abs(step_data[i]) for i in range(4, len(step_data), 3)]  # Every 3rd element starting from index 4
KeyError: 4
younleakim@younleakim-400TEA-400SEA:~/Downloads/direct_hand_parm$ 
