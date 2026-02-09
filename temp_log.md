

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
  IndexFinger-1st-pitch          |   2272.284 |    446.520 |        3.032  (gear=30:1, out=3408.4Nm, 20.0r/s, speed_margin=85%)
  IndexFinger-2nd-pitch          |   1108.465 |    218.587 |        0.000  (gear=30:1, out=1662.7Nm, 20.0r/s, speed_margin=100%)
  IndexFinger-3rd-pitch          |    439.848 |     87.802 |        0.000  (gear=30:1, out=659.8Nm, 20.0r/s, speed_margin=100%)
  IndexFinger_1st_roll           |     25.571 |      4.146 |        0.000  (gear=30:1, out=38.4Nm, 20.0r/s, speed_margin=100%)
  LittleFinger-1st-pitch         |      0.006 |      0.006 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  LittleFinger-1st_roll          |      0.007 |      0.007 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  LittleFinger-2nd-pitch         |      0.001 |      0.001 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  LittleFinger-3rd-pitch         |      0.000 |      0.000 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  MiddleFinger-1st-pitch         |   1625.948 |    296.498 |        1.444  (gear=30:1, out=2438.9Nm, 20.0r/s, speed_margin=93%)
  MiddleFinger-2nd-pitch         |    791.235 |    144.537 |        0.203  (gear=30:1, out=1186.9Nm, 20.0r/s, speed_margin=99%)
  MiddleFinger-3rd-pitch         |    284.388 |     52.456 |        0.000  (gear=30:1, out=426.6Nm, 20.0r/s, speed_margin=100%)
  MiddleFinger_1st_roll          |     34.566 |      5.762 |        0.023  (gear=30:1, out=51.8Nm, 20.0r/s, speed_margin=100%)
  RingFinger-1st-pitch           |      0.006 |      0.006 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  RingFinger-1st_roll            |      0.004 |      0.004 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  RingFinger-2nd-pitch           |      0.001 |      0.001 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  RingFinger-3rd-pitch           |      0.001 |      0.001 |        0.000  (gear=15:1, out=0.5Nm, 20.9r/s, speed_margin=100%)
  Thumb-1st-pitch                |   1974.266 |    462.425 |        1.947  (gear=30:1, out=2961.4Nm, 20.0r/s, speed_margin=90%)
  Thumb-2nd-pitch                |    997.377 |    233.615 |        0.000  (gear=30:1, out=1496.1Nm, 20.0r/s, speed_margin=100%)
  Thumb-3rd-pitch                |    486.602 |    113.633 |        0.000  (gear=30:1, out=729.9Nm, 20.0r/s, speed_margin=100%)
  thumb_1st_yaw                  |    144.802 |     31.551 |        0.362  (gear=30:1, out=217.2Nm, 20.0r/s, speed_margin=98%)

  Global defaults: stall=126.2380Nm, rated=75.7428Nm, speed=5730RPM, gear=30:1

======================================================================
  PHASE 2: Forward Simulation — Motor Physics Pipeline
  FF(100%) + PID(correction) → T-N Curve → Efficiency → Friction → MuJoCo
======================================================================
Enabled collision on 16 geoms for finger contact detection
  → solref=[0.004, 1.0], condim=4, margin=3mm for strong contact enforcement
Created 20 motor physics engines

  Per-joint motor specs:
    IndexFinger-1st-pitch         : stall=126.2380Nm × gear=30 → out=3408.4Nm, speed=20.0r/s, PID(20450.6/681.7/1363.4), traj_maxV=3.03r/s
    IndexFinger-2nd-pitch         : stall=61.5814Nm × gear=30 → out=1662.7Nm, speed=20.0r/s, PID(9976.2/332.5/665.1), traj_maxV=0.00r/s
    IndexFinger-3rd-pitch         : stall=24.4360Nm × gear=30 → out=659.8Nm, speed=20.0r/s, PID(3958.6/132.0/263.9), traj_maxV=0.00r/s
    IndexFinger_1st_roll          : stall=1.4206Nm × gear=30 → out=38.4Nm, speed=20.0r/s, PID(230.1/7.7/15.3), traj_maxV=0.00r/s
    LittleFinger-1st-pitch        : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    LittleFinger-1st_roll         : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    LittleFinger-2nd-pitch        : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    LittleFinger-3rd-pitch        : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    MiddleFinger-1st-pitch        : stall=90.3304Nm × gear=30 → out=2438.9Nm, speed=20.0r/s, PID(14633.5/487.8/975.6), traj_maxV=1.44r/s
    MiddleFinger-2nd-pitch        : stall=43.9575Nm × gear=30 → out=1186.9Nm, speed=20.0r/s, PID(7121.1/237.4/474.7), traj_maxV=0.20r/s
    MiddleFinger-3rd-pitch        : stall=15.7993Nm × gear=30 → out=426.6Nm, speed=20.0r/s, PID(2559.5/85.3/170.6), traj_maxV=0.00r/s
    MiddleFinger_1st_roll         : stall=1.9203Nm × gear=30 → out=51.8Nm, speed=20.0r/s, PID(311.1/10.4/20.7), traj_maxV=0.02r/s
    RingFinger-1st-pitch          : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    RingFinger-1st_roll           : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    RingFinger-2nd-pitch          : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    RingFinger-3rd-pitch          : stall=0.0370Nm × gear=15 → out=0.5Nm, speed=20.9r/s, PID(5.0/0.1/0.5), traj_maxV=0.00r/s
    Thumb-1st-pitch               : stall=109.6814Nm × gear=30 → out=2961.4Nm, speed=20.0r/s, PID(17768.4/592.3/1184.6), traj_maxV=1.95r/s
    Thumb-2nd-pitch               : stall=55.4098Nm × gear=30 → out=1496.1Nm, speed=20.0r/s, PID(8976.4/299.2/598.4), traj_maxV=0.00r/s
    Thumb-3rd-pitch               : stall=27.0334Nm × gear=30 → out=729.9Nm, speed=20.0r/s, PID(4379.4/146.0/292.0), traj_maxV=0.00r/s
    thumb_1st_yaw                 : stall=8.0445Nm × gear=30 → out=217.2Nm, speed=20.0r/s, PID(1303.2/43.4/86.9), traj_maxV=0.36r/s
UI created — ▶Play / ⏸Pause / Timeline slider / Hover for joint name

  SIMULATION STARTED — ⏸Pause to inspect, hover graph for joint names
  Close MuJoCo viewer to stop & see final report.
▶ [simT=0.00s wallT=0.0s] Worst: IndexFinger-1st-pitch margin=98% OK | Loop#0 | RT=1.00x | contacts=0
▶ [simT=0.32s wallT=0.5s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.63x | contacts=0
▶ [simT=0.65s wallT=1.0s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.64x | contacts=0
▶ [simT=0.95s wallT=1.5s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.63x | contacts=0
▶ [simT=1.29s wallT=2.0s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.64x | contacts=1
▶ [simT=1.62s wallT=2.5s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.64x | contacts=0
▶ [simT=1.97s wallT=3.0s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=0
▶ [simT=2.29s wallT=3.6s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=0
▶ [simT=2.62s wallT=4.1s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=0
▶ [simT=2.96s wallT=4.6s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=0
▶ [simT=3.28s wallT=5.1s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=0
▶ [simT=3.61s wallT=5.6s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=0
▶ [simT=3.94s wallT=6.1s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=1
▶ [simT=4.26s wallT=6.6s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.65x | contacts=0
▶ [simT=4.46s wallT=7.1s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0 | RT=0.63x | contacts=0

[QUIT] Stopping simulation — generating report...

Simulation ended
CSV log: motor_validation_log.csv

  💥 Total contact events: 72
  Contact pairs:
    thumb-3rd-end <-> world: 59 events, max_force=14296.930N
    index_finger-3rd-end <-> thumb-3rd-end: 13 events, max_force=5592.804N


██████████████████████████████████████████████████████████████████████
█  DIAGNOSTIC SUMMARY — Actual vs Spec
██████████████████████████████████████████████████████████████████████
  Joint                          | TrajMaxV |  ActMaxV | RatedSpd |   NoLoad | SatPct | TNlimit
  ──────────────────────────────────────────────────────────────────────────────────────────
  IndexFinger-1st-pitch          |     3.0 |  1069.1 |    20.0 |    24.0 |   0.0% |  99.7% ⚠️
  IndexFinger-2nd-pitch          |     0.0 |  1112.4 |    20.0 |    24.0 |   0.0% |  99.3% ⚠️
  IndexFinger-3rd-pitch          |     0.0 |   475.9 |    20.0 |    24.0 |   0.0% |  98.7% ⚠️
  IndexFinger_1st_roll           |     0.0 |   117.0 |    20.0 |    24.0 |   0.0% |   5.2% ⚠️
  LittleFinger-1st-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-1st_roll          |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-2nd-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  LittleFinger-3rd-pitch         |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  MiddleFinger-1st-pitch         |     1.4 |   661.5 |    20.0 |    24.0 |   0.0% |  99.6% ⚠️
  MiddleFinger-2nd-pitch         |     0.2 |   795.0 |    20.0 |    24.0 |   0.0% |  99.5% ⚠️
  MiddleFinger-3rd-pitch         |     0.0 |   314.2 |    20.0 |    24.0 |   0.0% |  95.7% ⚠️
  MiddleFinger_1st_roll          |     0.0 |    66.0 |    20.0 |    24.0 |   0.0% |   2.0% ⚠️
  RingFinger-1st-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-1st_roll            |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-2nd-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  RingFinger-3rd-pitch           |     0.0 |     0.0 |    20.9 |    25.1 |   0.0% |   0.0%
  Thumb-1st-pitch                |     1.9 |   889.9 |    20.0 |    24.0 |   0.1% |  98.9% ⚠️
  Thumb-2nd-pitch                |     0.0 |  1002.5 |    20.0 |    24.0 |   0.1% |  99.5% ⚠️
  Thumb-3rd-pitch                |     0.0 |   526.4 |    20.0 |    24.0 |   0.0% |  99.4% ⚠️
  thumb_1st_yaw                  |     0.4 |   304.6 |    20.0 |    24.0 |   0.0% |  71.5% ⚠️


██████████████████████████████████████████████████████████████████████
█  MOTOR SIZING VALIDATION — FINAL REPORT
██████████████████████████████████████████████████████████████████████

────────────────────────────────────────────────────────────
  Motor: IndexFinger-1st-pitch  ❌ FAIL
  Spec: stall=126.2380Nm × gear=30 × eff=90% → out=3408.43Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=4% (<80%)
  ❌ Tracking            : max=6.2748rad (359.52°)
  ❌ Speed Margin        : -5246% (EXCEEDED)
  ❌ Saturation          : 99.7% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-2nd-pitch  ❌ FAIL
  Spec: stall=61.5814Nm × gear=30 × eff=90% → out=1662.70Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=6% (<80%)
  ❌ Tracking            : max=3.2137rad (184.13°)
  ❌ Speed Margin        : -5462% (EXCEEDED)
  ❌ Saturation          : 99.4% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-3rd-pitch  ❌ FAIL
  Spec: stall=24.4360Nm × gear=30 × eff=90% → out=659.77Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=9% (<80%)
  ❌ Tracking            : max=1.8813rad (107.79°)
  ❌ Speed Margin        : -2279% (EXCEEDED)
  ❌ Saturation          : 98.8% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger_1st_roll  ❌ FAIL
  Spec: stall=1.4206Nm × gear=30 × eff=90% → out=38.36Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=6% (<80%)
  ❌ Tracking            : max=0.4548rad (26.06°)
  ❌ Speed Margin        : -485% (EXCEEDED)
  ⚠️  Saturation          : 5.3%

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
  Spec: stall=90.3304Nm × gear=30 × eff=90% → out=2438.92Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=4% (<80%)
  ❌ Tracking            : max=4.9985rad (286.39°)
  ❌ Speed Margin        : -3207% (EXCEEDED)
  ❌ Saturation          : 99.6% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=43.9575Nm × gear=30 × eff=90% → out=1186.85Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=7% (<80%)
  ❌ Tracking            : max=2.9142rad (166.97°)
  ❌ Speed Margin        : -3875% (EXCEEDED)
  ❌ Saturation          : 99.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-3rd-pitch  ❌ FAIL
  Spec: stall=15.7993Nm × gear=30 × eff=90% → out=426.58Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=13% (<80%)
  ❌ Tracking            : max=0.9841rad (56.38°)
  ❌ Speed Margin        : -1471% (EXCEEDED)
  ❌ Saturation          : 95.7% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger_1st_roll  ❌ FAIL
  Spec: stall=1.9203Nm × gear=30 × eff=90% → out=51.85Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=2% (<80%)
  ❌ Tracking            : max=0.4095rad (23.47°)
  ❌ Speed Margin        : -230% (EXCEEDED)
  ✅ Saturation          : 2.0%

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
  Motor: Thumb-1st-pitch  ❌ FAIL
  Spec: stall=109.6814Nm × gear=30 × eff=90% → out=2961.40Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=5% (<80%)
  ❌ Tracking            : max=4.8193rad (276.12°)
  ❌ Speed Margin        : -4350% (EXCEEDED)
  ❌ Saturation          : 98.9% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-2nd-pitch  ❌ FAIL
  Spec: stall=55.4098Nm × gear=30 × eff=90% → out=1496.06Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=8% (<80%)
  ❌ Tracking            : max=3.5162rad (201.46°)
  ❌ Speed Margin        : -4913% (EXCEEDED)
  ❌ Saturation          : 99.6% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-3rd-pitch  ❌ FAIL
  Spec: stall=27.0334Nm × gear=30 × eff=90% → out=729.90Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=10% (<80%)
  ❌ Tracking            : max=2.1787rad (124.83°)
  ❌ Speed Margin        : -2532% (EXCEEDED)
  ❌ Saturation          : 99.4% (frequent!)

────────────────────────────────────────────────────────────
  Motor: thumb_1st_yaw  ❌ FAIL
  Spec: stall=8.0445Nm × gear=30 × eff=90% → out=217.20Nm
  Spec: 5730RPM / gear=30 → out=20.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=21% (<80%)
  ❌ Tracking            : max=1.7751rad (101.70°)
  ❌ Speed Margin        : -1423% (EXCEEDED)
  ❌ Saturation          : 71.7% (frequent!)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ❌ OVERALL: SOME MOTORS NEED ATTENTION
     FAIL: IndexFinger-1st-pitch, IndexFinger-2nd-pitch, IndexFinger-3rd-pitch, IndexFinger_1st_roll, MiddleFinger-1st-pitch, MiddleFinger-2nd-pitch, MiddleFinger-3rd-pitch, MiddleFinger_1st_roll, Thumb-1st-pitch, Thumb-2nd-pitch, Thumb-3rd-pitch, thumb_1st_yaw

  💡 Recommendations:
     IndexFinger-1st-pitch: Increase stall torque or gear ratio
     IndexFinger-1st-pitch: Increase PID gains or motor torque
     IndexFinger-1st-pitch: Increase motor speed or reduce gear ratio
     IndexFinger-1st-pitch: Increase stall torque or gear ratio
     IndexFinger-2nd-pitch: Increase stall torque or gear ratio
     IndexFinger-2nd-pitch: Increase PID gains or motor torque
     IndexFinger-2nd-pitch: Increase motor speed or reduce gear ratio
     IndexFinger-2nd-pitch: Increase stall torque or gear ratio
     IndexFinger-3rd-pitch: Increase stall torque or gear ratio
     IndexFinger-3rd-pitch: Increase PID gains or motor torque
     IndexFinger-3rd-pitch: Increase motor speed or reduce gear ratio
     IndexFinger-3rd-pitch: Increase stall torque or gear ratio
     IndexFinger_1st_roll: Increase stall torque or gear ratio
     IndexFinger_1st_roll: Increase PID gains or motor torque
     IndexFinger_1st_roll: Increase motor speed or reduce gear ratio
     MiddleFinger-1st-pitch: Increase stall torque or gear ratio
     MiddleFinger-1st-pitch: Increase PID gains or motor torque
     MiddleFinger-1st-pitch: Increase motor speed or reduce gear ratio
     MiddleFinger-1st-pitch: Increase stall torque or gear ratio
     MiddleFinger-2nd-pitch: Increase stall torque or gear ratio
     MiddleFinger-2nd-pitch: Increase PID gains or motor torque
     MiddleFinger-2nd-pitch: Increase motor speed or reduce gear ratio
     MiddleFinger-2nd-pitch: Increase stall torque or gear ratio
     MiddleFinger-3rd-pitch: Increase stall torque or gear ratio
     MiddleFinger-3rd-pitch: Increase PID gains or motor torque
     MiddleFinger-3rd-pitch: Increase motor speed or reduce gear ratio
     MiddleFinger-3rd-pitch: Increase stall torque or gear ratio
     MiddleFinger_1st_roll: Increase stall torque or gear ratio
     MiddleFinger_1st_roll: Increase PID gains or motor torque
     MiddleFinger_1st_roll: Increase motor speed or reduce gear ratio
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-1st-pitch: Increase PID gains or motor torque
     Thumb-1st-pitch: Increase motor speed or reduce gear ratio
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-2nd-pitch: Increase stall torque or gear ratio
     Thumb-2nd-pitch: Increase PID gains or motor torque
     Thumb-2nd-pitch: Increase motor speed or reduce gear ratio
     Thumb-2nd-pitch: Increase stall torque or gear ratio
     Thumb-3rd-pitch: Increase stall torque or gear ratio
     Thumb-3rd-pitch: Increase PID gains or motor torque
     Thumb-3rd-pitch: Increase motor speed or reduce gear ratio
     Thumb-3rd-pitch: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase PID gains or motor torque
     thumb_1st_yaw: Increase motor speed or reduce gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Report saved to motor_validation_report.json
CSV log saved to motor_validation_log.csv


