~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh 
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
MODE 4 SCRIPT VERSION: 20260205_195601
Generated: 2026-02-05 19:56:01
======================================================================


======================================================================
PREPARING MODEL FOR PURE TORQUE CONTROL
======================================================================
Removing position actuators from MJCF...
  This allows pure qfrc_applied control without actuator interference
  Created temporary MJCF without actuators: tmp3k6_wn3k.xml
  Model loaded: 0 actuators (should be 0), 20 DOFs
  ✅ SUCCESS: Pure torque control model (no actuators)
======================================================================
✅ No actuators in model (pure torque control mode)
Model has 20 joints, 0 actuators
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
  model.nu (actuators): 0
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
    (No actuators - using pure torque control)

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

🔍 PHYSICS DEBUG at step 0:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 3/20, Max: 2.91 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
  T=0.00s: RMS error=0.0000 rad (0.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=0.0° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  MiddleFinger-1st-pitch=0.0° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 500:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 5/20, Max: 2.91 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
  T=0.50s: RMS error=0.0000 rad (0.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=0.0° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  IndexFinger-2nd-pitch=0.0° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 1000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 5/20, Max: 2.92 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
  T=1.00s: RMS error=0.0003 rad (0.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=0.1° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  MiddleFinger-1st-pitch=0.0° (cmd=0.00rad)  
  T=1.50s: RMS error=0.0003 rad (0.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=0.1° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  MiddleFinger-1st-pitch=0.0° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 1900:
   1 active contacts detected
      index_finger-3rd-end <-> thumb-3rd-end, penetration: 36.31mm

🔍 PHYSICS DEBUG at step 2000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 11/20, Max: 25.84 Nm

  💥 COLLISION DEBUG:
     Active contacts: 1
     ⚠️ COLLISIONS DETECTED! Analyzing contact pairs...
       Contact 1: index_finger-3rd-end      <-> thumb-3rd-end            
                 Penetration: 97.06 mm 🔴 DEEP!

  Joint States:

⚠️ COLLISION WARNING at step 2000:
   1 active contacts detected
      index_finger-3rd-end <-> thumb-3rd-end, penetration: 97.50mm
  T=2.00s: RMS error=0.0001 rad (0.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=0.0° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  MiddleFinger-1st-pitch=0.0° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 2100:
   1 active contacts detected
      index_finger-3rd-end <-> thumb-3rd-end, penetration: 121.09mm

⚠️ COLLISION WARNING at step 2200:
   1 active contacts detected
      index_finger-3rd-end <-> thumb-3rd-end, penetration: 104.25mm

⚠️ COLLISION WARNING at step 2300:
   1 active contacts detected
      index_finger-3rd-end <-> thumb-3rd-end, penetration: 49.48mm
  T=2.50s: RMS error=0.0007 rad (0.0°), Max cmd=0.00 rad
    Worst errors: thumb_1st_yaw=0.2° (cmd=0.00rad)  IndexFinger-1st-pitch=0.1° (cmd=0.00rad)  IndexFinger_1st_roll=0.1° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 3000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 6/20, Max: 39.10 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
  T=3.00s: RMS error=0.0005 rad (0.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=0.1° (cmd=0.00rad)  thumb_1st_yaw=0.1° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  
  T=3.50s: RMS error=0.0003 rad (0.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=0.1° (cmd=0.00rad)  thumb_1st_yaw=0.0° (cmd=0.00rad)  MiddleFinger-1st-pitch=0.0° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 4000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 8/20, Max: 30.18 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
  T=4.00s: RMS error=0.0003 rad (0.0°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-1st-pitch=0.1° (cmd=0.00rad)  thumb_1st_yaw=0.0° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  
  T=4.50s: RMS error=0.0003 rad (0.0°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-1st-pitch=0.1° (cmd=0.00rad)  Thumb-1st-pitch=0.0° (cmd=0.00rad)  thumb_1st_yaw=0.0° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 4900:
   1 active contacts detected
      middle_finger-3rd-end <-> thumb-3rd-end, penetration: 4.86mm

🔍 PHYSICS DEBUG at step 5000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 10/20, Max: 19.27 Nm

  💥 COLLISION DEBUG:
     Active contacts: 1
     ⚠️ COLLISIONS DETECTED! Analyzing contact pairs...
       Contact 1: middle_finger-3rd-end     <-> thumb-3rd-end            
                 Penetration: 64.53 mm 🔴 DEEP!

  Joint States:

⚠️ COLLISION WARNING at step 5000:
   1 active contacts detected
      middle_finger-3rd-end <-> thumb-3rd-end, penetration: 64.93mm
  T=5.00s: RMS error=0.0001 rad (0.0°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-1st-pitch=0.0° (cmd=0.00rad)  IndexFinger-1st-pitch=0.0° (cmd=0.00rad)  Thumb-3rd-pitch=0.0° (cmd=0.00rad)  

⚠️ COLLISION WARNING at step 5100:
   1 active contacts detected
      middle_finger-3rd-end <-> thumb-3rd-end, penetration: 83.95mm

⚠️ COLLISION WARNING at step 5200:
   1 active contacts detected
      middle_finger-3rd-end <-> thumb-3rd-end, penetration: 54.15mm

⚠️ COLLISION WARNING at step 5300:
   1 active contacts detected
      middle_finger-3rd-end <-> thumb-3rd-end, penetration: 21.06mm
  T=5.50s: RMS error=0.0589 rad (3.4°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=8.1° (cmd=0.00rad)  MiddleFinger-2nd-pitch=8.1° (cmd=0.00rad)  thumb_1st_yaw=7.1° (cmd=0.00rad)  

🔍 PHYSICS DEBUG at step 6000:
  Using REAL PHYSICS (mj_step) with torque control
  Applied torques: 9/20, Max: 2.91 Nm

  💥 COLLISION DEBUG:
     Active contacts: 0
     ✅ No collisions (collision exclusions working)

  Joint States:
  T=6.00s: RMS error=0.0257 rad (1.5°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=3.6° (cmd=0.00rad)  MiddleFinger-2nd-pitch=3.5° (cmd=0.00rad)  thumb_1st_yaw=3.1° (cmd=0.00rad)  
  T=6.50s: RMS error=0.0113 rad (0.6°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=1.6° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1.5° (cmd=0.00rad)  thumb_1st_yaw=1.3° (cmd=0.00rad)  

Simulation complete!

💾 Saving Phase 2 control history to CSV...
  Saved 684 samples (every 10 steps) to phase2_control_applied.csv

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 0.008253 rad (0.473 deg)
  Maximum RMS Error: 0.078200 rad (4.481 deg)

Torque Control (Direct Force Application):
  Average Torque: 0.00 Nm
  Peak Torque: 0.00 Nm

======================================================================
⚠ PARTIAL: Some tracking error but acceptable
  → Try increasing kp (stiffness)
  → Or check for collisions/joint limits
======================================================================

