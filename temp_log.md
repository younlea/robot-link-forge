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
  IndexFinger-1st-pitch          |     32.474 |     11.875 |        3.032  (gear=125:1, out=48.7Nm, 5.0r/s)
  IndexFinger-2nd-pitch          |     11.132 |      3.500 |        0.000  (gear=60:1, out=16.7Nm, 5.2r/s)
  IndexFinger-3rd-pitch          |      5.541 |      1.106 |        0.000  (gear=30:1, out=8.3Nm, 10.5r/s)
  IndexFinger_1st_roll           |      0.260 |      0.043 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  LittleFinger-1st-pitch         |      0.006 |      0.006 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  LittleFinger-1st_roll          |      0.007 |      0.007 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  LittleFinger-2nd-pitch         |      0.001 |      0.001 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  LittleFinger-3rd-pitch         |      0.000 |      0.000 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  MiddleFinger-1st-pitch         |     38.822 |     13.270 |        1.444  (gear=125:1, out=58.2Nm, 5.0r/s)
  MiddleFinger-2nd-pitch         |     10.092 |      1.929 |        0.203  (gear=55:1, out=15.1Nm, 5.7r/s)
  MiddleFinger-3rd-pitch         |      1.953 |      1.726 |        0.000  (gear=10:1, out=2.9Nm, 31.4r/s)
  MiddleFinger_1st_roll          |      0.425 |      0.071 |        0.023  (gear=5:1, out=0.6Nm, 62.8r/s)
  RingFinger-1st-pitch           |      0.006 |      0.006 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  RingFinger-1st_roll            |      0.004 |      0.004 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  RingFinger-2nd-pitch           |      0.001 |      0.001 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  RingFinger-3rd-pitch           |      0.001 |      0.001 |        0.000  (gear=5:1, out=0.5Nm, 62.8r/s)
  Thumb-1st-pitch                |     25.138 |      5.914 |        1.947  (gear=125:1, out=37.7Nm, 5.0r/s)
  Thumb-2nd-pitch                |     12.692 |      2.977 |        0.000  (gear=70:1, out=19.0Nm, 5.0r/s)
  Thumb-3rd-pitch                |      6.188 |      1.448 |        0.000  (gear=35:1, out=9.3Nm, 9.0r/s)
  thumb_1st_yaw                  |      1.936 |      0.438 |        0.362  (gear=10:1, out=2.9Nm, 31.4r/s)

  Global defaults: stall=0.5176Nm, rated=0.2877Nm, speed=5968RPM, gear=125:1

======================================================================
  PHASE 2: Forward Simulation — Motor Physics Pipeline
  FF(100%) + PID(correction) → T-N Curve → Efficiency → Friction → MuJoCo
======================================================================
Created 20 motor physics engines

  Per-joint motor specs:
    IndexFinger-1st-pitch         : stall=0.4330Nm × gear=125 → out=48.7Nm, 5.0r/s
    IndexFinger-2nd-pitch         : stall=0.3092Nm × gear=60 → out=16.7Nm, 5.2r/s
    IndexFinger-3rd-pitch         : stall=0.3078Nm × gear=30 → out=8.3Nm, 10.5r/s
    IndexFinger_1st_roll          : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    LittleFinger-1st-pitch        : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    LittleFinger-1st_roll         : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    LittleFinger-2nd-pitch        : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    LittleFinger-3rd-pitch        : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    MiddleFinger-1st-pitch        : stall=0.5176Nm × gear=125 → out=58.2Nm, 5.0r/s
    MiddleFinger-2nd-pitch        : stall=0.3058Nm × gear=55 → out=15.1Nm, 5.7r/s
    MiddleFinger-3rd-pitch        : stall=0.3255Nm × gear=10 → out=2.9Nm, 31.4r/s
    MiddleFinger_1st_roll         : stall=0.1416Nm × gear=5 → out=0.6Nm, 62.8r/s
    RingFinger-1st-pitch          : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    RingFinger-1st_roll           : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    RingFinger-2nd-pitch          : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    RingFinger-3rd-pitch          : stall=0.1111Nm × gear=5 → out=0.5Nm, 62.8r/s
    Thumb-1st-pitch               : stall=0.3352Nm × gear=125 → out=37.7Nm, 5.0r/s
    Thumb-2nd-pitch               : stall=0.3022Nm × gear=70 → out=19.0Nm, 5.0r/s
    Thumb-3rd-pitch               : stall=0.2947Nm × gear=35 → out=9.3Nm, 9.0r/s
    thumb_1st_yaw                 : stall=0.3226Nm × gear=10 → out=2.9Nm, 31.4r/s
UI created — ▶Play / ⏸Pause / Timeline slider / Hover for joint name

  SIMULATION STARTED — ⏸Pause to inspect, hover graph for joint names
  Close MuJoCo viewer to stop & see final report.
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#0
▶ [T=0.74s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=1.27s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=2.03s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#0
▶ [T=2.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=3.27s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=3.79s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=4.53s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=5.06s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#0
▶ [T=5.83s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#0
▶ [T=6.57s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=0.24s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#1
▶ [T=0.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=1.48s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=2.00s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#1
▶ [T=2.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=3.50s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=4.02s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=4.68s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=5.19s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#1
▶ [T=5.74s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#1
▶ [T=6.48s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#2
▶ [T=0.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=1.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=2.24s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#2
▶ [T=2.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=3.49s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=4.21s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=4.74s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=5.26s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=6.00s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#2
▶ [T=6.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=0.27s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#3
▶ [T=1.04s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=1.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=2.29s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#3
▶ [T=3.04s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=3.76s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=4.29s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=5.06s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#3
▶ [T=5.83s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#3
▶ [T=6.57s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=0.25s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#4
▶ [T=1.00s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=1.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=2.50s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=3.00s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=3.77s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=4.50s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=5.02s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#4
▶ [T=5.80s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#4
▶ [T=6.57s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=0.24s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#5
▶ [T=0.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=1.53s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=2.30s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#5
▶ [T=3.02s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=3.53s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=4.30s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=5.04s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#5
▶ [T=5.57s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#5
▶ [T=6.37s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#5
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#6
▶ [T=0.51s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=1.24s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=1.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=2.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=3.27s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=3.99s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=4.51s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=5.27s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=6.02s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#6
▶ [T=6.56s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#7
▶ [T=0.73s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=1.27s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=2.05s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#7
▶ [T=2.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=3.29s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=4.05s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=4.82s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=5.55s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#7
▶ [T=6.07s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=6.57s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=0.24s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#8
▶ [T=0.99s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=1.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=2.47s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=2.98s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=3.74s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=4.47s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=4.99s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#8
▶ [T=5.52s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=6.27s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#8
▶ [T=6.81s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=0.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=1.28s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=2.03s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#9
▶ [T=2.54s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=3.30s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=4.03s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=4.55s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=5.32s] Worst: RingFinger-1st-pitch margin=0% OVER! | Loop#9
▶ [T=5.82s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#9
▶ [T=6.35s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#9
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#10
▶ [T=0.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=1.02s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=1.54s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=2.07s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#10
▶ [T=2.80s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=3.33s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=4.09s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=4.84s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=5.37s] Worst: RingFinger-1st-pitch margin=0% OVER! | Loop#10
▶ [T=5.91s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#10
▶ [T=6.65s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=0.24s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#11
▶ [T=0.99s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=1.74s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=2.26s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#11
▶ [T=2.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=3.49s] Worst: Thumb-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=4.00s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=4.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=5.49s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#11
▶ [T=6.03s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#11
▶ [T=6.56s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=0.25s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#12
▶ [T=0.76s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=1.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=2.26s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#12
▶ [T=2.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=3.28s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=4.00s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=4.53s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=5.03s] Worst: Thumb-2nd-pitch margin=-100% OVER! | Loop#12
▶ [T=5.56s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#12
▶ [T=6.32s] Worst: LittleFinger-2nd-pitch margin=0% OVER! | Loop#12
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=34% OK | Loop#13
▶ [T=0.72s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#13
▶ [T=1.25s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#13
▶ [T=1.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#13

Simulation ended
CSV log: motor_validation_log.csv


██████████████████████████████████████████████████████████████████████
█  MOTOR SIZING VALIDATION — FINAL REPORT
██████████████████████████████████████████████████████████████████████

────────────────────────────────────────────────────────────
  Motor: IndexFinger-1st-pitch  ❌ FAIL
  Spec: stall=0.4330Nm × gear=125 × eff=90% → out=48.71Nm
  Spec: 5968RPM / gear=125 → out=5.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=31% (<80%)
  ❌ Tracking            : max=0.9688rad (55.51°)
  ❌ Speed Margin        : -126% (EXCEEDED)
  ❌ Saturation          : 96.2% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-2nd-pitch  ❌ FAIL
  Spec: stall=0.3092Nm × gear=60 × eff=90% → out=16.70Nm
  Spec: 3000RPM / gear=60 → out=5.24rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=173% (OVERLOAD)
  ✅ Tracking            : max=0.0075rad (0.43°)
  ✅ Speed Margin        : 40%
  ❌ Saturation          : 74.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-3rd-pitch  ❌ FAIL
  Spec: stall=0.3078Nm × gear=30 × eff=90% → out=8.31Nm
  Spec: 3000RPM / gear=30 → out=10.47rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=158% (OVERLOAD)
  ✅ Tracking            : max=0.0051rad (0.29°)
  ✅ Speed Margin        : 42%
  ❌ Saturation          : 77.0% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger_1st_roll  ❌ FAIL
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=54% (<80%)
  ✅ Tracking            : max=0.0001rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 1.4%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st-pitch  ❌ FAIL
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=60% (<80%)
  ✅ Tracking            : max=0.0001rad (0.01°)
  ✅ Speed Margin        : 100%
  ⚠️  Saturation          : 7.1%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st_roll  ✅ PASS
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=98% (>20%)
  ✅ Thermal Load        : avg=4% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=121% (OVERLOAD)
  ✅ Tracking            : max=0.0002rad (0.01°)
  ✅ Speed Margin        : 100%
  ❌ Saturation          : 40.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: LittleFinger-3rd-pitch  ❌ FAIL
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=148% (OVERLOAD)
  ✅ Tracking            : max=0.0002rad (0.01°)
  ✅ Speed Margin        : 100%
  ❌ Saturation          : 37.0% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-1st-pitch  ❌ FAIL
  Spec: stall=0.5176Nm × gear=125 × eff=90% → out=58.23Nm
  Spec: 5968RPM / gear=125 → out=5.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=25% (<80%)
  ❌ Tracking            : max=1.5867rad (90.91°)
  ❌ Speed Margin        : -125% (EXCEEDED)
  ❌ Saturation          : 96.2% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=0.3058Nm × gear=55 × eff=90% → out=15.14Nm
  Spec: 3000RPM / gear=55 → out=5.71rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ⚠️  Thermal Load        : avg=89% (near limit)
  ❌ Tracking            : max=0.2079rad (11.91°)
  ❌ Speed Margin        : -50% (EXCEEDED)
  ❌ Saturation          : 96.2% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-3rd-pitch  ❌ FAIL
  Spec: stall=0.3255Nm × gear=10 × eff=90% → out=2.93Nm
  Spec: 3000RPM / gear=10 → out=31.42rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ⚠️  Thermal Load        : avg=81% (near limit)
  ✅ Tracking            : max=0.0077rad (0.44°)
  ✅ Speed Margin        : 95%
  ❌ Saturation          : 23.0% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger_1st_roll  ❌ FAIL
  Spec: stall=0.1416Nm × gear=5 × eff=90% → out=0.64Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=230% (OVERLOAD)
  ✅ Tracking            : max=0.0265rad (1.52°)
  ✅ Speed Margin        : 99%
  ❌ Saturation          : 91.0% (frequent!)

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st-pitch  ❌ FAIL
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=77% (<80%)
  ✅ Tracking            : max=0.0001rad (0.01°)
  ✅ Speed Margin        : 100%
  ⚠️  Saturation          : 7.1%

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st_roll  ✅ PASS
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=99% (>20%)
  ✅ Thermal Load        : avg=2% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-2nd-pitch  ❌ FAIL
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=144% (OVERLOAD)
  ✅ Tracking            : max=0.0002rad (0.01°)
  ✅ Speed Margin        : 100%
  ❌ Saturation          : 40.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: RingFinger-3rd-pitch  ❌ FAIL
  Spec: stall=0.1111Nm × gear=5 × eff=90% → out=0.50Nm
  Spec: 3000RPM / gear=5 → out=62.83rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=133% (OVERLOAD)
  ✅ Tracking            : max=0.0002rad (0.01°)
  ✅ Speed Margin        : 100%
  ❌ Saturation          : 26.3% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-1st-pitch  ❌ FAIL
  Spec: stall=0.3352Nm × gear=125 × eff=90% → out=37.71Nm
  Spec: 5968RPM / gear=125 → out=5.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=54% (<80%)
  ❌ Tracking            : max=0.5001rad (28.65°)
  ❌ Speed Margin        : -103% (EXCEEDED)
  ❌ Saturation          : 96.2% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-2nd-pitch  ❌ FAIL
  Spec: stall=0.3022Nm × gear=70 × eff=90% → out=19.04Nm
  Spec: 3342RPM / gear=70 → out=5.00rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=57% (<80%)
  ✅ Tracking            : max=0.0201rad (1.15°)
  ❌ Speed Margin        : -103% (EXCEEDED)
  ❌ Saturation          : 91.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-3rd-pitch  ❌ FAIL
  Spec: stall=0.2947Nm × gear=35 × eff=90% → out=9.28Nm
  Spec: 3000RPM / gear=35 → out=8.98rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=123% (OVERLOAD)
  ✅ Tracking            : max=0.0070rad (0.40°)
  ✅ Speed Margin        : 24%
  ❌ Saturation          : 57.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: thumb_1st_yaw  ❌ FAIL
  Spec: stall=0.3226Nm × gear=10 × eff=90% → out=2.90Nm
  Spec: 3000RPM / gear=10 → out=31.42rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ❌ Thermal Load        : avg=218% (OVERLOAD)
  ❌ Tracking            : max=0.4015rad (23.00°)
  ✅ Speed Margin        : 89%
  ❌ Saturation          : 92.9% (frequent!)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ❌ OVERALL: SOME MOTORS NEED ATTENTION
     FAIL: IndexFinger-1st-pitch, IndexFinger-2nd-pitch, IndexFinger-3rd-pitch, IndexFinger_1st_roll, LittleFinger-1st-pitch, LittleFinger-2nd-pitch, LittleFinger-3rd-pitch, MiddleFinger-1st-pitch, MiddleFinger-2nd-pitch, MiddleFinger-3rd-pitch, MiddleFinger_1st_roll, RingFinger-1st-pitch, RingFinger-2nd-pitch, RingFinger-3rd-pitch, Thumb-1st-pitch, Thumb-2nd-pitch, Thumb-3rd-pitch, thumb_1st_yaw

  💡 Recommendations:
     IndexFinger-1st-pitch: Increase stall torque or gear ratio
     IndexFinger-1st-pitch: Increase PID gains or motor torque
     IndexFinger-1st-pitch: Increase motor speed or reduce gear ratio
     IndexFinger-1st-pitch: Increase stall torque or gear ratio
     IndexFinger-2nd-pitch: Increase stall torque or gear ratio
     IndexFinger-2nd-pitch: Increase rated torque (larger motor)
     IndexFinger-2nd-pitch: Increase stall torque or gear ratio
     IndexFinger-3rd-pitch: Increase stall torque or gear ratio
     IndexFinger-3rd-pitch: Increase rated torque (larger motor)
     IndexFinger-3rd-pitch: Increase stall torque or gear ratio
     IndexFinger_1st_roll: Increase stall torque or gear ratio
     LittleFinger-1st-pitch: Increase stall torque or gear ratio
     LittleFinger-2nd-pitch: Increase stall torque or gear ratio
     LittleFinger-2nd-pitch: Increase rated torque (larger motor)
     LittleFinger-2nd-pitch: Increase stall torque or gear ratio
     LittleFinger-3rd-pitch: Increase stall torque or gear ratio
     LittleFinger-3rd-pitch: Increase rated torque (larger motor)
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
     MiddleFinger-3rd-pitch: Increase stall torque or gear ratio
     MiddleFinger_1st_roll: Increase stall torque or gear ratio
     MiddleFinger_1st_roll: Increase rated torque (larger motor)
     MiddleFinger_1st_roll: Increase stall torque or gear ratio
     RingFinger-1st-pitch: Increase stall torque or gear ratio
     RingFinger-2nd-pitch: Increase stall torque or gear ratio
     RingFinger-2nd-pitch: Increase rated torque (larger motor)
     RingFinger-2nd-pitch: Increase stall torque or gear ratio
     RingFinger-3rd-pitch: Increase stall torque or gear ratio
     RingFinger-3rd-pitch: Increase rated torque (larger motor)
     RingFinger-3rd-pitch: Increase stall torque or gear ratio
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-1st-pitch: Increase PID gains or motor torque
     Thumb-1st-pitch: Increase motor speed or reduce gear ratio
     Thumb-1st-pitch: Increase stall torque or gear ratio
     Thumb-2nd-pitch: Increase stall torque or gear ratio
     Thumb-2nd-pitch: Increase motor speed or reduce gear ratio
     Thumb-2nd-pitch: Increase stall torque or gear ratio
     Thumb-3rd-pitch: Increase stall torque or gear ratio
     Thumb-3rd-pitch: Increase rated torque (larger motor)
     Thumb-3rd-pitch: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase rated torque (larger motor)
     thumb_1st_yaw: Increase PID gains or motor torque
     thumb_1st_yaw: Increase stall torque or gear ratio
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Report saved to motor_validation_report.json
CSV log saved to motor_validation_log.csv
