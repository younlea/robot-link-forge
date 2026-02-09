:~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh 
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

2. Inverse-to-Forward Validation
   - Use Mode 1 torques as motor limits
   - Test if physics can actually track trajectory

3. Motor Sizing Validation
   - Set motor parameters and validate

4. Fingertip Sensor Forces
   - Contact force visualization

========================================
Enter choice [0/1/2/3/4]: 3
Starting Motor Validation...
======================================================================
  PHASE 1: Inverse Dynamics — Detecting Required Torques
======================================================================
Recording: Recording 1768623534448 (index 0)

======================================================================
  Phase 1 Results: Required Motor Specifications
======================================================================
  Joint                          |   Peak(Nm) |    RMS(Nm) |  MaxVel(r/s)
  ----------------------------------------------------------------------
  IndexFinger-1st-pitch          |     32.474 |     11.875 |        3.032  (gear=30:1, out=48.7Nm, 20.0r/s, speed_margin=85%)
  IndexFinger-2nd-pitch          |     11.132 |      3.500 |        0.000  (gear=30:1, out=16.7Nm, 20.0r/s, speed_margin=100%)
  IndexFinger-3rd-pitch          |      5.541 |      1.106 |        0.000  (gear=30:1, out=8.3Nm, 20.0r/s, speed_margin=100%)
  IndexFinger_1st_roll           |      0.260 |      0.043 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  LittleFinger-1st-pitch         |      0.006 |      0.006 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  LittleFinger-1st_roll          |      0.007 |      0.007 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  LittleFinger-2nd-pitch         |      0.001 |      0.001 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  LittleFinger-3rd-pitch         |      0.000 |      0.000 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  MiddleFinger-1st-pitch         |     38.822 |     13.270 |        1.444  (gear=30:1, out=58.2Nm, 20.0r/s, speed_margin=93%)
  MiddleFinger-2nd-pitch         |     10.092 |      1.929 |        0.203  (gear=30:1, out=15.1Nm, 20.0r/s, speed_margin=99%)
  MiddleFinger-3rd-pitch         |      1.953 |      1.726 |        0.000  (gear=20:1, out=2.9Nm, 20.0r/s, speed_margin=100%)
  MiddleFinger_1st_roll          |      0.425 |      0.071 |        0.023  (gear=15:1, out=0.6Nm, 20.9r/s, speed_margin=100%)
  RingFinger-1st-pitch           |      0.006 |      0.006 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  RingFinger-1st_roll            |      0.004 |      0.004 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  RingFinger-2nd-pitch           |      0.001 |      0.001 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  RingFinger-3rd-pitch           |      0.001 |      0.001 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  Thumb-1st-pitch                |     25.138 |      5.914 |        1.947  (gear=30:1, out=37.7Nm, 20.0r/s, speed_margin=90%)
  Thumb-2nd-pitch                |     12.692 |      2.977 |        0.000  (gear=30:1, out=19.0Nm, 20.0r/s, speed_margin=100%)
  Thumb-3rd-pitch                |      6.188 |      1.448 |        0.000  (gear=30:1, out=9.3Nm, 20.0r/s, speed_margin=100%)
  thumb_1st_yaw                  |      1.936 |      0.438 |        0.362  (gear=20:1, out=2.9Nm, 20.0r/s, speed_margin=98%)

  Global defaults: stall=2.1568Nm, rated=1.2941Nm, speed=5730RPM, gear=30:1

======================================================================
  PHASE 2: Forward Simulation — Motor Physics Pipeline
  FF(100%) + PID(correction) → T-N Curve → Efficiency → Friction → MuJoCo
======================================================================
Created 20 motor physics engines

  Per-joint motor specs:
    IndexFinger-1st-pitch         : stall=1.8041Nm × gear=30 → out=48.7Nm, speed=20.0r/s, PID(292.3/9.7/19.5), traj_maxV=3.03r/s
    IndexFinger-2nd-pitch         : stall=0.6184Nm × gear=30 → out=16.7Nm, speed=20.0r/s, PID(100.2/3.3/6.7), traj_maxV=0.00r/s
    IndexFinger-3rd-pitch         : stall=0.3078Nm × gear=30 → out=8.3Nm, speed=20.0r/s, PID(49.9/1.7/3.3), traj_maxV=0.00r/s
    IndexFinger_1st_roll          : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    LittleFinger-1st-pitch        : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    LittleFinger-1st_roll         : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    LittleFinger-2nd-pitch        : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    LittleFinger-3rd-pitch        : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    MiddleFinger-1st-pitch        : stall=2.1568Nm × gear=30 → out=58.2Nm, speed=20.0r/s, PID(349.4/11.6/23.3), traj_maxV=1.44r/s
    MiddleFinger-2nd-pitch        : stall=0.5607Nm × gear=30 → out=15.1Nm, speed=20.0r/s, PID(90.8/3.0/6.1), traj_maxV=0.20r/s
    MiddleFinger-3rd-pitch        : stall=0.1627Nm × gear=20 → out=2.9Nm, speed=20.0r/s, PID(17.6/0.6/1.2), traj_maxV=0.00r/s
    MiddleFinger_1st_roll         : stall=0.0472Nm × gear=15 → out=0.6Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.02r/s
    RingFinger-1st-pitch          : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    RingFinger-1st_roll           : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    RingFinger-2nd-pitch          : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    RingFinger-3rd-pitch          : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    Thumb-1st-pitch               : stall=1.3966Nm × gear=30 → out=37.7Nm, speed=20.0r/s, PID(226.2/7.5/15.1), traj_maxV=1.95r/s
    Thumb-2nd-pitch               : stall=0.7051Nm × gear=30 → out=19.0Nm, speed=20.0r/s, PID(114.2/3.8/7.6), traj_maxV=0.00r/s
    Thumb-3rd-pitch               : stall=0.3438Nm × gear=30 → out=9.3Nm, speed=20.0r/s, PID(55.7/1.9/3.7), traj_maxV=0.00r/s
    thumb_1st_yaw                 : stall=0.1613Nm × gear=20 → out=2.9Nm, speed=20.0r/s, PID(17.4/0.6/1.2), traj_maxV=0.36r/s
UI created — ▶Play / ⏸Pause / Timeline slider / Hover for joint name

  SIMULATION STARTED — ⏸Pause to inspect, hover graph for joint names
  Close MuJoCo viewer to stop & see final report.
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#0 | RT=1.00x
▶ [T=0.31s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.62x
▶ [T=0.64s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.64x
▶ [T=0.98s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.65x
▶ [T=1.30s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.65x
▶ [T=1.64s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.65x
▶ [T=1.96s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.65x
▶ [T=2.27s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=2.59s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=2.93s] Worst: MiddleFinger-1st-pitch margin=26% OK | Loop#0 | RT=0.65x
▶ [T=3.23s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=3.55s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=3.88s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=4.19s] Worst: IndexFinger-1st-pitch margin=28% OK | Loop#0 | RT=0.64x
▶ [T=4.52s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=4.86s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=5.16s] Worst: MiddleFinger-2nd-pitch margin=44% OK | Loop#0 | RT=0.64x
▶ [T=5.50s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.64x
▶ [T=5.83s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=6.12s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=6.44s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.64x
▶ [T=6.67s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.63x
▶ [T=0.04s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#1 | RT=0.62x
▶ [T=0.26s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.61x
▶ [T=0.59s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.61x
▶ [T=0.88s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.61x
▶ [T=1.21s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.61x
▶ [T=1.54s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.62x
▶ [T=1.84s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.62x
▶ [T=2.16s] Worst: thumb_1st_yaw margin=25% OK | Loop#1 | RT=0.62x
▶ [T=2.49s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.62x
▶ [T=2.79s] Worst: MiddleFinger-1st-pitch margin=32% OK | Loop#1 | RT=0.62x
▶ [T=3.11s] Worst: MiddleFinger-1st-pitch margin=32% OK | Loop#1 | RT=0.62x
▶ [T=3.16s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.60x
▶ [T=3.35s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.59x
▶ [T=3.58s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.59x
▶ [T=3.63s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.57x
▶ [T=3.93s] Worst: IndexFinger-1st-pitch margin=32% OK | Loop#1 | RT=0.58x
▶ [T=4.12s] Worst: IndexFinger-1st-pitch margin=26% OK | Loop#1 | RT=0.57x
▶ [T=4.45s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.57x
▶ [T=4.78s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.57x
▶ [T=5.07s] Worst: MiddleFinger-2nd-pitch margin=27% OK | Loop#1 | RT=0.57x
▶ [T=5.41s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.58x
▶ [T=5.73s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.58x
./run_torque_replay_0_recording_1768623534448.sh: line 98: 25358 Segmentation fault      (core dumped) python3 $SCRIPT 0
