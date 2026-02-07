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
  IndexFinger-1st-pitch          |     32.474 |     11.875 |        3.032  (gear=105:1, out=64.9Nm, 6.1r/s)
  IndexFinger-2nd-pitch          |     11.132 |      3.500 |        0.000  (gear=80:1, out=22.3Nm, 3.9r/s)
  IndexFinger-3rd-pitch          |      5.541 |      1.106 |        0.000  (gear=40:1, out=11.1Nm, 7.9r/s)
  IndexFinger_1st_roll           |      0.260 |      0.043 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  LittleFinger-1st-pitch         |      0.006 |      0.006 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  LittleFinger-1st_roll          |      0.007 |      0.007 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  LittleFinger-2nd-pitch         |      0.001 |      0.001 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  LittleFinger-3rd-pitch         |      0.000 |      0.000 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  MiddleFinger-1st-pitch         |     38.822 |     13.270 |        1.444  (gear=220:1, out=77.6Nm, 2.9r/s)
  MiddleFinger-2nd-pitch         |     10.092 |      1.929 |        0.203  (gear=75:1, out=20.2Nm, 4.2r/s)
  MiddleFinger-3rd-pitch         |      1.953 |      1.726 |        0.000  (gear=15:1, out=3.9Nm, 20.9r/s)
  MiddleFinger_1st_roll          |      0.425 |      0.071 |        0.023  (gear=10:1, out=0.8Nm, 31.4r/s)
  RingFinger-1st-pitch           |      0.006 |      0.006 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  RingFinger-1st_roll            |      0.004 |      0.004 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  RingFinger-2nd-pitch           |      0.001 |      0.001 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  RingFinger-3rd-pitch           |      0.001 |      0.001 |        0.000  (gear=10:1, out=0.5Nm, 31.4r/s)
  Thumb-1st-pitch                |     25.138 |      5.914 |        1.947  (gear=160:1, out=50.3Nm, 3.9r/s)
  Thumb-2nd-pitch                |     12.692 |      2.977 |        0.000  (gear=95:1, out=25.4Nm, 3.3r/s)
  Thumb-3rd-pitch                |      6.188 |      1.448 |        0.000  (gear=45:1, out=12.4Nm, 7.0r/s)
  thumb_1st_yaw                  |      1.936 |      0.438 |        0.362  (gear=15:1, out=3.9Nm, 20.9r/s)

  Global defaults: stall=0.6873Nm, rated=0.3436Nm, speed=6080RPM, gear=220:1

======================================================================
  PHASE 2: Forward Simulation — Motor Physics Pipeline
  FF(100%) + PID(correction) → T-N Curve → Efficiency → Friction → MuJoCo
======================================================================
Created 20 motor physics engines
UI created — ▶Play / ⏸Pause / Timeline slider / Hover for joint name

  SIMULATION STARTED — ⏸Pause to inspect, hover graph for joint names
  Close MuJoCo viewer to stop & see final report.
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=50% OK | Loop#0
▶ [T=0.71s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=1.21s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=1.92s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=2.62s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#0
▶ [T=3.13s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=3.86s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=4.57s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=5.07s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=5.58s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=6.28s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#0
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=50% OK | Loop#1
▶ [T=0.73s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=1.46s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=2.15s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=2.65s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#1
▶ [T=3.38s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=4.12s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=4.82s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=5.45s] Worst: LittleFinger-3rd-pitch margin=0% OVER! | Loop#1
▶ [T=6.05s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#1
▶ [T=6.56s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=50% OK | Loop#2
▶ [T=0.72s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=2.37s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=4.47s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
[APPLY] Updated ALL joints
▶ [T=6.41s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=98% OK | Loop#3
▶ [T=0.71s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=1.45s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=1.95s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#3
▶ [T=2.70s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=3.41s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#3
▶ [T=3.92s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#3
▶ [T=4.65s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#3
▶ [T=5.39s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#3
▶ [T=6.11s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#3
▶ [T=6.62s] Worst: LittleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=0.24s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=0.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=3.73s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=4.45s] Worst: MiddleFinger_1st_roll margin=-100% OVER! | Loop#4
▶ [T=4.97s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=5.72s] Worst: MiddleFinger_1st_roll margin=-100% OVER! | Loop#4
▶ [T=6.48s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#4
▶ [T=0.25s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#5
▶ [T=0.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=1.50s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=3.60s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#5
▶ [T=4.30s] Worst: MiddleFinger_1st_roll margin=-100% OVER! | Loop#5
▶ [T=5.03s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=5.54s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#5
▶ [T=6.24s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#5
▶ [T=6.74s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#5
▶ [T=0.27s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#6
▶ [T=0.98s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=1.48s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=2.95s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=3.47s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=99% OK | Loop#7
▶ [T=0.63s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=1.25s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=2.04s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=2.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=3.26s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=4.00s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=4.77s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#7
▶ [T=5.49s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#7
▶ [T=5.99s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#7
▶ [T=6.72s] Worst: LittleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=0.47s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=0.98s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=1.49s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=2.20s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=2.70s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=3.44s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=4.18s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#8
▶ [T=4.89s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#8
▶ [T=5.53s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#8
▶ [T=6.14s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#8
▶ [T=6.77s] Worst: LittleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=0.48s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=0.98s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=1.49s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=2.20s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=2.70s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=3.45s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9

Simulation ended
CSV log: motor_validation_log.csv


██████████████████████████████████████████████████████████████████████
█  MOTOR SIZING VALIDATION — FINAL REPORT
██████████████████████████████████████████████████████████████████████

────────────────────────────────────────────────────────────
  Motor: IndexFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=15% (<80%)
  ❌ Tracking            : max=1.1173rad (64.02°)
  ❌ Speed Margin        : -3016% (EXCEEDED)
  ❌ Saturation          : 95.3% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=50% (<80%)
  ❌ Tracking            : max=0.3995rad (22.89°)
  ❌ Speed Margin        : -7170% (EXCEEDED)
  ❌ Saturation          : 78.3% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=34% (<80%)
  ⚠️  Tracking            : max=0.1112rad (6.37°)
  ❌ Speed Margin        : -2806% (EXCEEDED)
  ❌ Saturation          : 65.1% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger_1st_roll  ⚠️  WARN
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ⚠️  Torque Margin       : min=5% (tight!)
  ✅ Thermal Load        : avg=12% (<80%)
  ✅ Tracking            : max=0.0001rad (0.00°)
  ✅ Speed Margin        : 94%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=12% (<80%)
  ✅ Tracking            : max=0.0088rad (0.50°)
  ❌ Speed Margin        : -385% (EXCEEDED)
  ✅ Saturation          : 3.3%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st_roll  ✅ PASS
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=98% (>20%)
  ✅ Thermal Load        : avg=1% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=26% (<80%)
  ✅ Tracking            : max=0.0437rad (2.51°)
  ❌ Speed Margin        : -1917% (EXCEEDED)
  ⚠️  Saturation          : 18.9%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=44% (<80%)
  ⚠️  Tracking            : max=0.0937rad (5.37°)
  ❌ Speed Margin        : -1767% (EXCEEDED)
  ❌ Saturation          : 42.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=14% (<80%)
  ❌ Tracking            : max=1.9033rad (109.05°)
  ❌ Speed Margin        : -2637% (EXCEEDED)
  ❌ Saturation          : 93.9% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=22% (<80%)
  ❌ Tracking            : max=0.3974rad (22.77°)
  ❌ Speed Margin        : -2890% (EXCEEDED)
  ❌ Saturation          : 90.6% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=23% (<80%)
  ✅ Tracking            : max=0.0069rad (0.40°)
  ✅ Speed Margin        : 32%
  ⚠️  Saturation          : 6.1%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger_1st_roll  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=59% (<80%)
  ⚠️  Tracking            : max=0.0511rad (2.93°)
  ❌ Speed Margin        : -737% (EXCEEDED)
  ❌ Saturation          : 47.6% (frequent!)

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=20% (<80%)
  ✅ Tracking            : max=0.0208rad (1.19°)
  ❌ Speed Margin        : -604% (EXCEEDED)
  ⚠️  Saturation          : 8.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st_roll  ✅ PASS
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=99% (>20%)
  ✅ Thermal Load        : avg=0% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=32% (<80%)
  ⚠️  Tracking            : max=0.0540rad (3.09°)
  ❌ Speed Margin        : -1676% (EXCEEDED)
  ❌ Saturation          : 29.7% (frequent!)

────────────────────────────────────────────────────────────
  Motor: RingFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=45% (<80%)
  ⚠️  Tracking            : max=0.1131rad (6.48°)
  ❌ Speed Margin        : -2391% (EXCEEDED)
  ❌ Saturation          : 42.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-1st-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=16% (<80%)
  ❌ Tracking            : max=0.7906rad (45.30°)
  ❌ Speed Margin        : -3030% (EXCEEDED)
  ❌ Saturation          : 95.3% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-2nd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=15% (<80%)
  ❌ Tracking            : max=0.1654rad (9.48°)
  ❌ Speed Margin        : -3092% (EXCEEDED)
  ❌ Saturation          : 87.7% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-3rd-pitch  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=25% (<80%)
  ⚠️  Tracking            : max=0.1263rad (7.24°)
  ❌ Speed Margin        : -2755% (EXCEEDED)
  ❌ Saturation          : 56.6% (frequent!)

────────────────────────────────────────────────────────────
  Motor: thumb_1st_yaw  ❌ FAIL
  Spec: stall=1.4304Nm × gear=265 × eff=90% → out=341.14Nm
  Spec: 5820RPM / gear=265 → out=2.30rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=58% (<80%)
  ❌ Tracking            : max=0.4694rad (26.89°)
  ❌ Speed Margin        : -2164% (EXCEEDED)
  ❌ Saturation          : 91.0% (frequent!)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ❌ OVERALL: SOME MOTORS NEED ATTENTION
     FAIL: IndexFinger-1st-pitch, IndexFinger-2nd-pitch, IndexFinger-3rd-pitch, LittleFinger-1st-pitch, LittleFinger-2nd-pitch, LittleFinger-3rd-pitch, MiddleFinger-1st-pitch, MiddleFinger-2nd-pitch, MiddleFinger-3rd-pitch, MiddleFinger_1st_roll, RingFinger-1st-pitch, RingFinger-2nd-pitch, RingFinger-3rd-pitch, Thumb-1st-pitch, Thumb-2nd-pitch, Thumb-3rd-pitch, thumb_1st_yaw
     WARN: IndexFinger_1st_roll

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
     IndexFinger-3rd-pitch: Increase motor speed or reduce gear ratio
     IndexFinger-3rd-pitch: Increase stall torque or gear ratio
     LittleFinger-1st-pitch: Increase stall torque or gear ratio
     LittleFinger-1st-pitch: Increase motor speed or reduce gear ratio
     LittleFinger-2nd-pitch: Increase stall torque or gear ratio
     LittleFinger-2nd-pitch: Increase motor speed or reduce gear ratio
     LittleFinger-3rd-pitch: Increase stall torque or gear ratio
     LittleFinger-3rd-pitch: Increase motor speed or reduce gear ratio
     LittleFinger-3rd-pitch: Increase stall torque or gear ratio
     MiddleFinger-1st-pitch: Increase stall torque or gear ratio
     MiddleFinger-1st-pitch: Increase PID gains or motor torque
     MiddleFinger-1st-pitch: Increase motor speed or reduce gear ratio
     MiddleFinger-1st-pitch: Increase stall torque or gear ratio
     MiddleFinger-2nd-pitch: Increase stall torque or gear ratio
     MiddleFinger-2nd-pitch: Increase PID gains or motor torque
     MiddleFinger-2nd-pitch: Increase motor speed or reduce gear ratio
     MiddleFinger-2nd-pitch: Increase stall torque or gear ratio
     MiddleFinger-3rd-pitch: Increase stall torque or gear ratio
     MiddleFinger_1st_roll: Increase stall torque or gear ratio
     MiddleFinger_1st_roll: Increase motor speed or reduce gear ratio
     MiddleFinger_1st_roll: Increase stall torque or gear ratio
     RingFinger-1st-pitch: Increase stall torque or gear ratio
     RingFinger-1st-pitch: Increase motor speed or reduce gear ratio
     RingFinger-2nd-pitch: Increase stall torque or gear ratio
     RingFinger-2nd-pitch: Increase motor speed or reduce gear ratio
     RingFinger-2nd-pitch: Increase stall torque or gear ratio
     RingFinger-3rd-pitch: Increase stall torque or gear ratio
     RingFinger-3rd-pitch: Increase motor speed or reduce gear ratio
     RingFinger-3rd-pitch: Increase stall torque or gear ratio
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-1st-pitch: Increase PID gains or motor torque
     Thumb-1st-pitch: Increase motor speed or reduce gear ratio
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-2nd-pitch: Increase stall torque or gear ratio
     Thumb-2nd-pitch: Increase PID gains or motor torque
     Thumb-2nd-pitch: Increase motor speed or reduce gear ratio
     Thumb-2nd-pitch: Increase stall torque or gear ratio
     Thumb-3rd-pitch: Increase stall torque or gear ratio
     Thumb-3rd-pitch: Increase motor speed or reduce gear ratio
     Thumb-3rd-pitch: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase PID gains or motor torque
     thumb_1st_yaw: Increase motor speed or reduce gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Report saved to motor_validation_report.json
CSV log saved to motor_validation_log.csv
