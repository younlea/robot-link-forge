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
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#0
▶ [T=0.50s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=1.02s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=1.75s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=2.27s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=3.03s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=3.74s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=4.42s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=4.94s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=5.66s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=6.17s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#0
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#1
▶ [T=0.72s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=1.24s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=1.74s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=2.45s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=2.95s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=3.71s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=4.43s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=4.95s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=5.72s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=6.48s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=0.25s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=0.78s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=1.52s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=2.24s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=2.76s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=3.52s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=4.28s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=5.00s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=5.53s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=6.29s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#2
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#3
▶ [T=0.51s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=1.02s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=1.75s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=2.26s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=3.01s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=3.74s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=4.25s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=5.01s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=5.77s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=6.65s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#3
▶ [T=0.24s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=0.99s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=1.72s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=2.23s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=2.97s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=3.77s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=4.51s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=5.04s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=5.80s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=6.52s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#5
▶ [T=0.75s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=1.46s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=1.96s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=2.46s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=2.99s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=3.49s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=4.01s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=4.51s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#5
▶ [T=5.02s] Worst: MiddleFinger-1st-pitch margin=7% OK | Loop#5
▶ [T=5.52s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#5
▶ [T=6.04s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=6.54s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#5
▶ [T=0.20s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#6
▶ [T=0.70s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#6
▶ [T=1.22s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#6
▶ [T=1.72s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#6
▶ [T=2.23s] Worst: MiddleFinger-1st-pitch margin=24% OK | Loop#6
▶ [T=2.74s] Worst: MiddleFinger-1st-pitch margin=28% OK | Loop#6
▶ [T=3.25s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#6
^C▶ [T=3.75s] Worst: MiddleFinger-3rd-pitch margin=35% OK | Loop#6
▶ [T=4.30s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#6
▶ [T=4.87s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#6
▶ [T=5.37s] Worst: MiddleFinger-3rd-pitch margin=29% OK | Loop#6
▶ [T=5.88s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#6

Simulation ended
CSV log: motor_validation_log.csv


██████████████████████████████████████████████████████████████████████
█  DIAGNOSTIC SUMMARY — Actual vs Spec
██████████████████████████████████████████████████████████████████████
  Joint                          | TrajMaxV |  ActMaxV | RatedSpd |   NoLoad | SatPct | TNlimit
  ──────────────────────────────────────────────────────────────────────────────────────────
  IndexFinger-1st-pitch          |     3.0 |    18.9 |    20.0 |    24.0 |   0.2% |  32.9%
  IndexFinger-2nd-pitch          |     0.0 |     4.3 |    20.0 |    24.0 |   0.0% |   0.0%
  IndexFinger-3rd-pitch          |     0.0 |     4.9 |    20.0 |    24.0 |   0.0% |   0.0%
  IndexFinger_1st_roll           |     0.0 |     0.5 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-1st-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-1st_roll          |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-2nd-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-3rd-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  MiddleFinger-1st-pitch         |     1.4 |    21.0 |    20.0 |    24.0 |   0.5% |  32.0% ⚠️
  MiddleFinger-2nd-pitch         |     0.2 |    12.6 |    20.0 |    24.0 |   0.0% |   5.6%
  MiddleFinger-3rd-pitch         |     0.0 |     1.6 |    20.0 |    24.0 |   0.0% |   0.0%
  MiddleFinger_1st_roll          |     0.0 |     0.9 |    20.9 |    25.1 |   0.3% |   0.2%
  RingFinger-1st-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-1st_roll            |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-2nd-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-3rd-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  Thumb-1st-pitch                |     1.9 |    17.7 |    20.0 |    24.0 |   0.0% |  26.3%
  Thumb-2nd-pitch                |     0.0 |     7.9 |    20.0 |    24.0 |   0.0% |   0.2%
  Thumb-3rd-pitch                |     0.0 |     5.1 |    20.0 |    24.0 |   0.0% |   0.0%
  thumb_1st_yaw                  |     0.4 |     4.0 |    20.0 |    24.0 |   0.0% |  10.4%


██████████████████████████████████████████████████████████████████████
█  MOTOR SIZING VALIDATION — FINAL REPORT
██████████████████████████████████████████████████████████████████████

────────────────────────────────────────────────────────────
  Motor: IndexFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.8041Nm × gear=30 × eff=90% → out=48.71Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=44% (<80%)
  ❌ Tracking            : max=1.0071rad (57.70°)
  ⚠️  Speed Margin        : 6% (close)
  ❌ Saturation          : 33.3% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-2nd-pitch  ✅ PASS
  Spec: stall=0.6184Nm × gear=30 × eff=90% → out=16.70Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=39% (>20%)
  ✅ Thermal Load        : avg=38% (<80%)
  ✅ Tracking            : max=0.0348rad (1.99°)
  ✅ Speed Margin        : 78%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: IndexFinger-3rd-pitch  ⚠️  WARN
  Spec: stall=0.3078Nm × gear=30 × eff=90% → out=8.31Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ⚠️  Torque Margin       : min=16% (tight!)
  ✅ Thermal Load        : avg=4% (<80%)
  ✅ Tracking            : max=0.0195rad (1.12°)
  ✅ Speed Margin        : 76%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: IndexFinger_1st_roll  ✅ PASS
  Spec: stall=0.0370Nm × gear=15 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=32% (>20%)
  ✅ Thermal Load        : avg=4% (<80%)
  ✅ Tracking            : max=0.0046rad (0.26°)
  ✅ Speed Margin        : 98%
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
  Motor: MiddleFinger-1st-pitch  ❌ FAIL
  Spec: stall=2.1568Nm × gear=30 × eff=90% → out=58.23Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=44% (<80%)
  ❌ Tracking            : max=1.5761rad (90.31°)
  ❌ Speed Margin        : -5% (EXCEEDED)
  ❌ Saturation          : 33.1% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=0.5607Nm × gear=30 × eff=90% → out=15.14Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=30% (<80%)
  ❌ Tracking            : max=0.2183rad (12.51°)
  ✅ Speed Margin        : 37%
  ⚠️  Saturation          : 6.0%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-3rd-pitch  ⚠️  WARN
  Spec: stall=0.1627Nm × gear=20 × eff=90% → out=2.93Nm
  Spec: 3820RPM / gear=20 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ⚠️  Torque Margin       : min=13% (tight!)
  ✅ Thermal Load        : avg=69% (<80%)
  ✅ Tracking            : max=0.0078rad (0.45°)
  ✅ Speed Margin        : 92%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger_1st_roll  ❌ FAIL
  Spec: stall=0.0472Nm × gear=15 × eff=90% → out=0.64Nm
  Spec: 3000RPM / gear=15 → out=20.94rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=14% (<80%)
  ✅ Tracking            : max=0.0285rad (1.63°)
  ✅ Speed Margin        : 96%
  ✅ Saturation          : 0.6%

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
  ✅ Tracking            : max=0.0001rad (0.00°)
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
  Motor: Thumb-1st-pitch  ❌ FAIL
  Spec: stall=1.3966Nm × gear=30 × eff=90% → out=37.71Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=47% (<80%)
  ❌ Tracking            : max=0.5230rad (29.96°)
  ⚠️  Speed Margin        : 12% (close)
  ❌ Saturation          : 27.0% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-2nd-pitch  ❌ FAIL
  Spec: stall=0.7051Nm × gear=30 × eff=90% → out=19.04Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=21% (<80%)
  ✅ Tracking            : max=0.0250rad (1.43°)
  ✅ Speed Margin        : 60%
  ✅ Saturation          : 0.2%

────────────────────────────────────────────────────────────
  Motor: Thumb-3rd-pitch  ✅ PASS
  Spec: stall=0.3438Nm × gear=30 × eff=90% → out=9.28Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=21% (>20%)
  ✅ Thermal Load        : avg=8% (<80%)
  ✅ Tracking            : max=0.0194rad (1.11°)
  ✅ Speed Margin        : 74%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: thumb_1st_yaw  ❌ FAIL
  Spec: stall=0.1613Nm × gear=20 × eff=90% → out=2.90Nm
  Spec: 3820RPM / gear=20 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=49% (<80%)
  ❌ Tracking            : max=0.3940rad (22.58°)
  ✅ Speed Margin        : 80%
  ⚠️  Saturation          : 11.9%

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ❌ OVERALL: SOME MOTORS NEED ATTENTION
     FAIL: IndexFinger-1st-pitch, MiddleFinger-1st-pitch, MiddleFinger-2nd-pitch, MiddleFinger_1st_roll, Thumb-1st-pitch, Thumb-2nd-pitch, thumb_1st_yaw
     WARN: IndexFinger-3rd-pitch, MiddleFinger-3rd-pitch

  💡 Recommendations:
     IndexFinger-1st-pitch: Increase stall torque or gear ratio
     IndexFinger-1st-pitch: Increase PID gains or motor torque
     IndexFinger-1st-pitch: Increase stall torque or gear ratio
     MiddleFinger-1st-pitch: Increase stall torque or gear ratio
     MiddleFinger-1st-pitch: Increase PID gains or motor torque
     MiddleFinger-1st-pitch: Increase motor speed or reduce gear ratio
     MiddleFinger-1st-pitch: Increase stall torque or gear ratio
     MiddleFinger-2nd-pitch: Increase stall torque or gear ratio
     MiddleFinger-2nd-pitch: Increase PID gains or motor torque
     MiddleFinger_1st_roll: Increase stall torque or gear ratio
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-1st-pitch: Increase PID gains or motor torque
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-2nd-pitch: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase PID gains or motor torque
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Report saved to motor_validation_report.json
CSV log saved to motor_validation_log.csv
