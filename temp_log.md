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
MODE 4 SCRIPT VERSION: 20260205_163211
Generated: 2026-02-05 16:32:11
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
PHASE 2: FORWARD SIMULATION WITH TORQUE CONTROL
======================================================================

Using DIRECT TORQUE control (not position control):
  Apply torques from Phase 1 inverse dynamics
  Use data.qfrc_applied (generalized forces)
  This is pure feedforward torque control

Note: Position actuators in MJCF are IGNORED
  We bypass actuators and apply forces directly

📍 Initialization for torque control:
  Starting from model's default pose (no forced positions)
  Letting model settle to equilibrium (100 steps, no forces)...
WARNING: Nan, Inf or huge value in QACC at DOF 4. The simulation is unstable. Time = 0.0080.

  Equilibrium reached. Sample positions: [-7.70061794  5.68466627 -1.56167053  7.26282187 -7.40498405]

=== Initial Position vs Trajectory Start ===
  LittleFinger-1st_roll: actual=-7.7006, traj_start=+0.0000, diff=+7.7006 (+441.21°)
  LittleFinger-1st-pitch: actual=+5.6847, traj_start=+0.0000, diff=-5.6847 (-325.71°)
  LittleFinger-2nd-pitch: actual=-1.5617, traj_start=+0.0000, diff=+1.5617 (+89.48°)
  LittleFinger-3rd-pitch: actual=+7.2628, traj_start=+0.0000, diff=-7.2628 (-416.13°)
  RingFinger-1st_roll : actual=-7.4050, traj_start=+0.0000, diff=+7.4050 (+424.27°)
  RingFinger-1st-pitch: actual=+4.9199, traj_start=+0.0000, diff=-4.9199 (-281.89°)
  RingFinger-2nd-pitch: actual=-2.2647, traj_start=+0.0000, diff=+2.2647 (+129.76°)
  RingFinger-3rd-pitch: actual=+7.4581, traj_start=+0.0000, diff=-7.4581 (-427.32°)
  MiddleFinger_1st_roll: actual=-2.7191, traj_start=+0.0000, diff=+2.7191 (+155.80°)
  MiddleFinger-1st-pitch: actual=-16.1055, traj_start=+0.0000, diff=+16.1055 (+922.77°)
  MiddleFinger-2nd-pitch: actual=+10.8001, traj_start=+0.0000, diff=-10.8001 (-618.80°)
  MiddleFinger-3rd-pitch: actual=+3.7384, traj_start=+0.0000, diff=-3.7384 (-214.19°)
  IndexFinger_1st_roll: actual=+9.7001, traj_start=+0.0000, diff=-9.7001 (-555.77°)
  IndexFinger-1st-pitch: actual=+1.6082, traj_start=+0.0000, diff=-1.6082 (-92.14°)
  IndexFinger-2nd-pitch: actual=+1.4952, traj_start=+0.0000, diff=-1.4952 (-85.67°)
  IndexFinger-3rd-pitch: actual=-7.7715, traj_start=+0.0000, diff=+7.7715 (+445.27°)
  thumb_1st_yaw       : actual=+2.3389, traj_start=+0.0000, diff=-2.3389 (-134.01°)
  Thumb-1st-pitch     : actual=+4.5560, traj_start=+0.0000, diff=-4.5560 (-261.04°)
  Thumb-2nd-pitch     : actual=+1.3905, traj_start=+0.0000, diff=-1.3905 (-79.67°)
  Thumb-3rd-pitch     : actual=+8.7764, traj_start=+0.0000, diff=-8.7764 (-502.85°)

Initial mismatch RMS: 6.8865 rad (394.57°)
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

Initial RMS error after stabilization: 6.8865 rad (394.57°)
==================================================
  ⚠️  Large initial error! Physics may be unstable or gains too weak

=== Initializing ctrl to current actual positions ===
  LittleFinger-1st_roll: ctrl initialized to -7.7006 rad
  LittleFinger-1st-pitch: ctrl initialized to 5.6847 rad
  LittleFinger-2nd-pitch: ctrl initialized to -1.5617 rad
  LittleFinger-3rd-pitch: ctrl initialized to 7.2628 rad
  RingFinger-1st_roll : ctrl initialized to -7.4050 rad
  RingFinger-1st-pitch: ctrl initialized to 4.9199 rad
  RingFinger-2nd-pitch: ctrl initialized to -2.2647 rad
  RingFinger-3rd-pitch: ctrl initialized to 7.4581 rad
  MiddleFinger_1st_roll: ctrl initialized to -2.7191 rad
  MiddleFinger-1st-pitch: ctrl initialized to -16.1055 rad
  MiddleFinger-2nd-pitch: ctrl initialized to 10.8001 rad
  MiddleFinger-3rd-pitch: ctrl initialized to 3.7384 rad
  IndexFinger_1st_roll: ctrl initialized to 9.7001 rad
  IndexFinger-1st-pitch: ctrl initialized to 1.6082 rad
  IndexFinger-2nd-pitch: ctrl initialized to 1.4952 rad
  IndexFinger-3rd-pitch: ctrl initialized to -7.7715 rad
  thumb_1st_yaw       : ctrl initialized to 2.3389 rad
  Thumb-1st-pitch     : ctrl initialized to 4.5560 rad
  Thumb-2nd-pitch     : ctrl initialized to 1.3905 rad
  Thumb-3rd-pitch     : ctrl initialized to 8.7764 rad
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

🔍 DEBUG at step 0:
  Using TORQUE control (not position)
  Nonzero torques: 3/20
  Max torque: 2.91 Nm
    LittleFinger-1st_roll         : target_pos= +0.0000, current= -7.7006, torque= +0.0071 Nm
    LittleFinger-1st-pitch        : target_pos= +0.0000, current= +5.6847, torque= -0.0057 Nm
    LittleFinger-2nd-pitch        : target_pos= +0.0000, current= -1.5617, torque= -0.0013 Nm
    LittleFinger-3rd-pitch        : target_pos= +0.0000, current= +7.2628, torque= -0.0005 Nm
    RingFinger-1st_roll           : target_pos= +0.0000, current= -7.4050, torque= +0.0038 Nm
  T=0.00s: RMS error=12.1461 rad (695.9°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=2609.1° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1291.6° (cmd=0.00rad)  IndexFinger-2nd-pitch=507.8° (cmd=0.00rad)  

🔍 DEBUG at step 500:
  Using TORQUE control (not position)
  Nonzero torques: 5/20
  Max torque: 2.91 Nm
    LittleFinger-1st_roll         : target_pos= +0.0000, current= +0.0257, torque= +0.0071 Nm
    LittleFinger-1st-pitch        : target_pos= +0.0000, current= -0.0198, torque= -0.0057 Nm
    LittleFinger-2nd-pitch        : target_pos= +0.0000, current= -0.0053, torque= -0.0013 Nm
    LittleFinger-3rd-pitch        : target_pos= +0.0000, current= -0.0110, torque= -0.0005 Nm
    RingFinger-1st_roll           : target_pos= +0.0000, current= +0.0127, torque= +0.0038 Nm
  T=0.50s: RMS error=6.4277 rad (368.3°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=746.0° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  LittleFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=1.00s: RMS error=12.0981 rad (693.2°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=2586.8° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1291.7° (cmd=0.00rad)  IndexFinger-2nd-pitch=524.4° (cmd=0.00rad)  
  T=1.50s: RMS error=1707.7619 rad (97847.5°), Max cmd=0.00 rad
    Worst errors: RingFinger-1st-pitch=432983.9° (cmd=0.00rad)  RingFinger-1st_roll=61331.4° (cmd=0.00rad)  RingFinger-2nd-pitch=12115.8° (cmd=0.00rad)  
  T=2.00s: RMS error=6.8230 rad (390.9°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-1st-pitch=921.6° (cmd=0.00rad)  MiddleFinger-2nd-pitch=618.9° (cmd=0.00rad)  IndexFinger_1st_roll=537.9° (cmd=0.00rad)  
  T=2.50s: RMS error=0.3758 rad (21.5°), Max cmd=0.00 rad
    Worst errors: IndexFinger-1st-pitch=82.2° (cmd=0.00rad)  Thumb-1st-pitch=44.1° (cmd=0.00rad)  IndexFinger-2nd-pitch=18.4° (cmd=0.00rad)  
  T=3.00s: RMS error=13.3596 rad (765.4°), Max cmd=0.00 rad
    Worst errors: MiddleFinger-2nd-pitch=2066.1° (cmd=0.00rad)  RingFinger-1st_roll=2008.6° (cmd=0.00rad)  RingFinger-2nd-pitch=1123.5° (cmd=0.00rad)  
  T=3.50s: RMS error=6.4279 rad (368.3°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=746.1° (cmd=0.00rad)  RingFinger-3rd-pitch=570.7° (cmd=0.00rad)  LittleFinger-3rd-pitch=570.7° (cmd=0.00rad)  
  T=4.00s: RMS error=11.8213 rad (677.3°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=2511.5° (cmd=0.00rad)  MiddleFinger-2nd-pitch=1295.1° (cmd=0.00rad)  IndexFinger-2nd-pitch=492.9° (cmd=0.00rad)  
  T=4.50s: RMS error=839.7422 rad (48113.7°), Max cmd=0.00 rad
    Worst errors: MiddleFinger_1st_roll=191614.0° (cmd=0.00rad)  MiddleFinger-2nd-pitch=93870.6° (cmd=0.00rad)  MiddleFinger-1st-pitch=19289.5° (cmd=0.00rad)  
  T=5.00s: RMS error=3.6878 rad (211.3°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=576.2° (cmd=0.00rad)  MiddleFinger-3rd-pitch=551.4° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=5.50s: RMS error=3.6826 rad (211.0°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=576.2° (cmd=0.00rad)  MiddleFinger-3rd-pitch=551.5° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=6.00s: RMS error=3.6863 rad (211.2°), Max cmd=0.00 rad
    Worst errors: IndexFinger-2nd-pitch=576.2° (cmd=0.00rad)  MiddleFinger-3rd-pitch=551.5° (cmd=0.00rad)  IndexFinger-3rd-pitch=364.8° (cmd=0.00rad)  
  T=6.50s: RMS error=1665.5394 rad (95428.4°), Max cmd=0.00 rad
    Worst errors: RingFinger-1st-pitch=345168.9° (cmd=0.00rad)  MiddleFinger_1st_roll=204330.4° (cmd=0.00rad)  MiddleFinger-2nd-pitch=114312.1° (cmd=0.00rad)  

Simulation complete!

💾 Saving Phase 2 control history to CSV...
  Saved 684 samples (every 10 steps) to phase2_control_applied.csv

======================================================================
VALIDATION RESULTS
======================================================================

Tracking Performance:
  Average RMS Error: 92.380891 rad (5293.035 deg)
  Maximum RMS Error: 2364.727128 rad (135488.884 deg)

Torque Control (Direct Force Application):
  Average Torque: 0.00 Nm
  Peak Torque: 0.00 Nm

======================================================================
✗ FAILED: Large tracking errors even with position control
  → Trajectory may be too fast for physics timestep
  → Or robot model has issues (mass/inertia/constraints)
======================================================================

