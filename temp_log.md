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
Enabled collision on 16 geoms for finger contact detection
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
▶ [simT=0.00s wallT=0.0s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#0 | RT=1.00x | contacts=0
▶ [simT=0.32s wallT=0.5s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.63x | contacts=0
▶ [simT=0.52s wallT=1.0s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.52x | contacts=0
▶ [simT=0.74s wallT=1.5s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.49x | contacts=0
▶ [simT=1.04s wallT=2.0s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.52x | contacts=0
▶ [simT=1.09s wallT=3.3s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#0 | RT=0.33x | contacts=0
▶ [simT=1.41s wallT=3.8s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.37x | contacts=0
▶ [simT=1.72s wallT=4.3s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.40x | contacts=0
▶ [simT=2.06s wallT=4.8s] Worst: thumb_1st_yaw margin=29% OK | Loop#0 | RT=0.43x | contacts=1
▶ [simT=2.38s wallT=5.3s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.45x | contacts=1
▶ [simT=2.69s wallT=5.8s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.46x | contacts=0
▶ [simT=3.02s wallT=6.3s] Worst: MiddleFinger-1st-pitch margin=26% OK | Loop#0 | RT=0.48x | contacts=0
▶ [simT=3.34s wallT=6.8s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.49x | contacts=0
▶ [simT=3.65s wallT=7.3s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.50x | contacts=0
▶ [simT=3.96s wallT=7.8s] Worst: IndexFinger-1st-pitch margin=31% OK | Loop#0 | RT=0.51x | contacts=0
▶ [simT=4.29s wallT=8.3s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.52x | contacts=0
▶ [simT=4.60s wallT=8.8s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.52x | contacts=0
▶ [simT=4.94s wallT=9.3s] Worst: MiddleFinger-2nd-pitch margin=51% OK | Loop#0 | RT=0.53x | contacts=1
▶ [simT=5.27s wallT=9.8s] Worst: MiddleFinger-3rd-pitch margin=49% OK | Loop#0 | RT=0.54x | contacts=1
▶ [simT=5.56s wallT=10.3s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.54x | contacts=0
▶ [simT=5.89s wallT=10.8s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.54x | contacts=0
▶ [simT=6.22s wallT=11.3s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.55x | contacts=0
▶ [simT=6.55s wallT=11.8s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#0 | RT=0.55x | contacts=0
▶ [simT=0.05s wallT=12.3s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#1 | RT=0.56x | contacts=0
▶ [simT=0.38s wallT=12.8s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.56x | contacts=0
▶ [simT=0.72s wallT=13.3s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.57x | contacts=0
▶ [simT=1.03s wallT=13.8s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#1 | RT=0.57x | contacts=0
▶ [simT=1.37s wallT=14.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.57x | contacts=0
▶ [simT=1.71s wallT=14.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.57x | contacts=0
▶ [simT=2.02s wallT=15.4s] Worst: Thumb-1st-pitch margin=32% OK | Loop#1 | RT=0.58x | contacts=1
▶ [simT=2.36s wallT=15.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.58x | contacts=1
▶ [simT=2.69s wallT=16.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.58x | contacts=0
▶ [simT=2.99s wallT=16.9s] Worst: MiddleFinger-1st-pitch margin=26% OK | Loop#1 | RT=0.58x | contacts=0
▶ [simT=3.33s wallT=17.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.58x | contacts=0
▶ [simT=3.67s wallT=17.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.59x | contacts=0
▶ [simT=3.98s wallT=18.4s] Worst: IndexFinger-1st-pitch margin=29% OK | Loop#1 | RT=0.59x | contacts=0
▶ [simT=4.31s wallT=18.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.59x | contacts=0
▶ [simT=4.64s wallT=19.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.59x | contacts=0
▶ [simT=4.94s wallT=19.9s] Worst: MiddleFinger-2nd-pitch margin=47% OK | Loop#1 | RT=0.59x | contacts=1
▶ [simT=5.28s wallT=20.4s] Worst: MiddleFinger-3rd-pitch margin=43% OK | Loop#1 | RT=0.59x | contacts=1
▶ [simT=5.61s wallT=20.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.60x | contacts=0
▶ [simT=5.92s wallT=21.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.60x | contacts=0
▶ [simT=6.27s wallT=21.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.60x | contacts=0
▶ [simT=6.60s wallT=22.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#1 | RT=0.60x | contacts=0
▶ [simT=0.07s wallT=22.9s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#2 | RT=0.60x | contacts=0
▶ [simT=0.41s wallT=23.4s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#2 | RT=0.60x | contacts=0
▶ [simT=0.75s wallT=23.9s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#2 | RT=0.60x | contacts=0
▶ [simT=1.08s wallT=24.4s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#2 | RT=0.60x | contacts=0
▶ [simT=1.43s wallT=24.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=1.77s wallT=25.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=2.09s wallT=25.9s] Worst: thumb_1st_yaw margin=25% OK | Loop#2 | RT=0.61x | contacts=1
▶ [simT=2.38s wallT=26.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=1
▶ [simT=2.62s wallT=26.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.60x | contacts=0
▶ [simT=2.95s wallT=27.4s] Worst: MiddleFinger-1st-pitch margin=26% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=3.26s wallT=27.9s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=3.59s wallT=28.4s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=3.91s wallT=28.9s] Worst: IndexFinger-1st-pitch margin=34% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=4.22s wallT=29.5s] Worst: IndexFinger-1st-pitch margin=31% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=4.54s wallT=30.0s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=4.87s wallT=30.5s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=5.17s wallT=31.0s] Worst: MiddleFinger-2nd-pitch margin=48% OK | Loop#2 | RT=0.61x | contacts=1
▶ [simT=5.51s wallT=31.5s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=5.84s wallT=32.0s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=6.15s wallT=32.5s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=6.49s wallT=33.0s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=6.83s wallT=33.5s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#2 | RT=0.61x | contacts=0
▶ [simT=0.34s wallT=34.0s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#3 | RT=0.61x | contacts=0
▶ [simT=0.67s wallT=34.5s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#3 | RT=0.61x | contacts=0
▶ [simT=1.02s wallT=35.0s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#3 | RT=0.61x | contacts=0
▶ [simT=1.39s wallT=35.5s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=1.72s wallT=36.0s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=2.05s wallT=36.5s] Worst: thumb_1st_yaw margin=29% OK | Loop#3 | RT=0.62x | contacts=1
▶ [simT=2.39s wallT=37.0s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=1
▶ [simT=2.69s wallT=37.5s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=3.03s wallT=38.0s] Worst: MiddleFinger-1st-pitch margin=26% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=3.37s wallT=38.5s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=3.66s wallT=39.1s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=3.90s wallT=39.6s] Worst: IndexFinger-1st-pitch margin=35% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=4.23s wallT=40.1s] Worst: IndexFinger-1st-pitch margin=33% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=4.53s wallT=40.6s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=0
▶ [simT=4.76s wallT=41.1s] Worst: MiddleFinger-3rd-pitch margin=36% OK | Loop#3 | RT=0.62x | contacts=0

[QUIT] Stopping simulation — generating report...

Simulation ended
CSV log: motor_validation_log.csv

  💥 Total contact events: 354
  Contact pairs:
    index_finger-3rd-end <-> thumb-3rd-end: 227 events, max_force=39.364N
    middle_finger-3rd-end <-> thumb-3rd-end: 127 events, max_force=27.336N


██████████████████████████████████████████████████████████████████████
█  DIAGNOSTIC SUMMARY — Actual vs Spec
██████████████████████████████████████████████████████████████████████
  Joint                          | TrajMaxV |  ActMaxV | RatedSpd |   NoLoad | SatPct | TNlimit
  ──────────────────────────────────────────────────────────────────────────────────────────
  IndexFinger-1st-pitch          |     3.0 |     4.8 |    20.0 |    24.0 |   0.0% |   0.0%
  IndexFinger-2nd-pitch          |     0.0 |     0.6 |    20.0 |    24.0 |   0.0% |   0.0%
  IndexFinger-3rd-pitch          |     0.0 |     1.0 |    20.0 |    24.0 |   0.0% |   0.0%
  IndexFinger_1st_roll           |     0.0 |     0.1 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-1st-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-1st_roll          |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-2nd-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-3rd-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  MiddleFinger-1st-pitch         |     1.4 |     1.4 |    20.0 |    24.0 |   0.0% |   0.0%
  MiddleFinger-2nd-pitch         |     0.2 |     1.5 |    20.0 |    24.0 |   0.0% |   0.0%
  MiddleFinger-3rd-pitch         |     0.0 |     0.1 |    20.0 |    24.0 |   0.0% |   0.0%
  MiddleFinger_1st_roll          |     0.0 |     0.2 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-1st-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-1st_roll            |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-2nd-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-3rd-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  Thumb-1st-pitch                |     1.9 |     3.0 |    20.0 |    24.0 |   0.0% |   0.0%
  Thumb-2nd-pitch                |     0.0 |     1.8 |    20.0 |    24.0 |   0.0% |   0.0%
  Thumb-3rd-pitch                |     0.0 |     1.3 |    20.0 |    24.0 |   0.0% |   0.0%
  thumb_1st_yaw                  |     0.4 |     0.4 |    20.0 |    24.0 |   0.0% |   0.0%


██████████████████████████████████████████████████████████████████████
█  MOTOR SIZING VALIDATION — FINAL REPORT
██████████████████████████████████████████████████████████████████████

────────────────────────────────────────────────────────────
  Motor: IndexFinger-1st-pitch  ✅ PASS
  Spec: stall=1.8041Nm × gear=30 × eff=90% → out=48.71Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=26% (>20%)
  ✅ Thermal Load        : avg=23% (<80%)
  ✅ Tracking            : max=0.0127rad (0.73°)
  ✅ Speed Margin        : 76%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: IndexFinger-2nd-pitch  ✅ PASS
  Spec: stall=0.6184Nm × gear=30 × eff=90% → out=16.70Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=28% (>20%)
  ✅ Thermal Load        : avg=34% (<80%)
  ✅ Tracking            : max=0.0065rad (0.37°)
  ✅ Speed Margin        : 97%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: IndexFinger-3rd-pitch  ✅ PASS
  Spec: stall=0.3078Nm × gear=30 × eff=90% → out=8.31Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=31% (>20%)
  ✅ Thermal Load        : avg=6% (<80%)
  ✅ Tracking            : max=0.0111rad (0.64°)
  ✅ Speed Margin        : 95%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: IndexFinger_1st_roll  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=58% (>20%)
  ✅ Thermal Load        : avg=4% (<80%)
  ✅ Tracking            : max=0.0120rad (0.69°)
  ✅ Speed Margin        : 99%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st-pitch  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=99% (>20%)
  ✅ Thermal Load        : avg=2% (<80%)
  ✅ Tracking            : max=0.0001rad (0.01°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st_roll  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=98% (>20%)
  ✅ Thermal Load        : avg=2% (<80%)
  ✅ Tracking            : max=0.0002rad (0.01°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-2nd-pitch  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=100% (>20%)
  ✅ Thermal Load        : avg=0% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-3rd-pitch  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=100% (>20%)
  ✅ Thermal Load        : avg=0% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-1st-pitch  ✅ PASS
  Spec: stall=2.1568Nm × gear=30 × eff=90% → out=58.23Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=26% (>20%)
  ✅ Thermal Load        : avg=20% (<80%)
  ✅ Tracking            : max=0.0125rad (0.72°)
  ✅ Speed Margin        : 93%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-2nd-pitch  ✅ PASS
  Spec: stall=0.5607Nm × gear=30 × eff=90% → out=15.14Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=22% (>20%)
  ✅ Thermal Load        : avg=7% (<80%)
  ✅ Tracking            : max=0.0040rad (0.23°)
  ✅ Speed Margin        : 92%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-3rd-pitch  ✅ PASS
  Spec: stall=0.1627Nm × gear=20 × eff=90% → out=2.93Nm
  Spec: 3820RPM / gear=20 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=25% (>20%)
  ✅ Thermal Load        : avg=72% (<80%)
  ✅ Tracking            : max=0.0030rad (0.17°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger_1st_roll  ✅ PASS
  Spec: stall=0.0472Nm × gear=15 × eff=90% → out=0.64Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=32% (>20%)
  ✅ Thermal Load        : avg=4% (<80%)
  ✅ Tracking            : max=0.0175rad (1.00°)
  ✅ Speed Margin        : 99%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st-pitch  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=99% (>20%)
  ✅ Thermal Load        : avg=2% (<80%)
  ✅ Tracking            : max=0.0001rad (0.01°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st_roll  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=99% (>20%)
  ✅ Thermal Load        : avg=1% (<80%)
  ✅ Tracking            : max=0.0001rad (0.01°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-2nd-pitch  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=100% (>20%)
  ✅ Thermal Load        : avg=0% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-3rd-pitch  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=100% (>20%)
  ✅ Thermal Load        : avg=0% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: Thumb-1st-pitch  ✅ PASS
  Spec: stall=1.3966Nm × gear=30 × eff=90% → out=37.71Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=27% (>20%)
  ✅ Thermal Load        : avg=10% (<80%)
  ✅ Tracking            : max=0.0121rad (0.70°)
  ✅ Speed Margin        : 85%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: Thumb-2nd-pitch  ✅ PASS
  Spec: stall=0.7051Nm × gear=30 × eff=90% → out=19.04Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=27% (>20%)
  ✅ Thermal Load        : avg=9% (<80%)
  ✅ Tracking            : max=0.0123rad (0.71°)
  ✅ Speed Margin        : 91%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: Thumb-3rd-pitch  ✅ PASS
  Spec: stall=0.3438Nm × gear=30 × eff=90% → out=9.28Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=30% (>20%)
  ✅ Thermal Load        : avg=9% (<80%)
  ✅ Tracking            : max=0.0122rad (0.70°)
  ✅ Speed Margin        : 93%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: thumb_1st_yaw  ✅ PASS
  Spec: stall=0.1613Nm × gear=20 × eff=90% → out=2.90Nm
  Spec: 3820RPM / gear=20 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=24% (>20%)
  ✅ Thermal Load        : avg=11% (<80%)
  ✅ Tracking            : max=0.0162rad (0.93°)
  ✅ Speed Margin        : 98%
  ✅ Saturation          : 0.0%

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ✅ OVERALL: ALL MOTORS PASS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Report saved to motor_validation_report.json
CSV log saved to motor_validation_log.csv
