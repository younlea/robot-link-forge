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
  IndexFinger-1st-pitch          |     32.474 |     11.875 |        3.032  (gear=300:1)
  IndexFinger-2nd-pitch          |     11.132 |      3.500 |        0.000  (gear=210:1)
  IndexFinger-3rd-pitch          |      5.541 |      1.106 |        0.000  (gear=100:1)
  IndexFinger_1st_roll           |      0.260 |      0.043 |        0.000  (gear=30:1)
  LittleFinger-1st-pitch         |      0.006 |      0.006 |        0.000  (gear=30:1)
  LittleFinger-1st_roll          |      0.007 |      0.007 |        0.000  (gear=30:1)
  LittleFinger-2nd-pitch         |      0.001 |      0.001 |        0.000  (gear=30:1)
  LittleFinger-3rd-pitch         |      0.000 |      0.000 |        0.000  (gear=30:1)
  MiddleFinger-1st-pitch         |     38.822 |     13.270 |        1.444  (gear=300:1)
  MiddleFinger-2nd-pitch         |     10.092 |      1.929 |        0.203  (gear=190:1)
  MiddleFinger-3rd-pitch         |      1.953 |      1.726 |        0.000  (gear=40:1)
  MiddleFinger_1st_roll          |      0.425 |      0.071 |        0.023  (gear=30:1)
  RingFinger-1st-pitch           |      0.006 |      0.006 |        0.000  (gear=30:1)
  RingFinger-1st_roll            |      0.004 |      0.004 |        0.000  (gear=30:1)
  RingFinger-2nd-pitch           |      0.001 |      0.001 |        0.000  (gear=30:1)
  RingFinger-3rd-pitch           |      0.001 |      0.001 |        0.000  (gear=30:1)
  Thumb-1st-pitch                |     25.138 |      5.914 |        1.947  (gear=300:1)
  Thumb-2nd-pitch                |     12.692 |      2.977 |        0.000  (gear=240:1)
  Thumb-3rd-pitch                |      6.188 |      1.448 |        0.000  (gear=110:1)
  thumb_1st_yaw                  |      1.936 |      0.438 |        0.362  (gear=40:1)

  Global defaults: stall=0.7189Nm, rated=0.4314Nm, speed=43426RPM, gear=300:1

======================================================================
  PHASE 2: Forward Simulation — Motor Physics Pipeline
  FF(100%) + PID(correction) → T-N Curve → Efficiency → Friction → MuJoCo
======================================================================
Created 20 motor physics engines
UI created — ▶Play / ⏸Pause / Timeline slider / Hover for joint name

  SIMULATION STARTED — ⏸Pause to inspect, hover graph for joint names
  Close MuJoCo viewer to stop & see final report.
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=80% OK | Loop#0
▶ [T=0.64s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=1.16s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=1.89s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=2.40s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=3.16s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=3.89s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=4.40s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=5.17s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=5.93s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=6.67s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#0
▶ [T=0.24s] Worst: MiddleFinger_1st_roll margin=0% OVER! | Loop#1
▶ [T=0.99s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=1.71s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=2.22s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=2.97s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=3.70s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=4.41s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=4.92s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=5.67s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=6.41s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#1
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=80% OK | Loop#2
▶ [T=0.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=2.89s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=3.59s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=5.41s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#2
▶ [T=6.14s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#2
▶ [T=0.00s] Worst: MiddleFinger-3rd-pitch margin=80% OK | Loop#3
▶ [T=0.51s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=1.03s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
[APPLY] Updated ALL joints
▶ [T=2.37s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=2.88s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=3.39s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#3
▶ [T=4.10s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=4.61s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=5.40s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#3
▶ [T=6.14s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#3
▶ [T=6.67s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#3
▶ [T=0.28s] Worst: MiddleFinger-1st-pitch margin=0% OVER! | Loop#4
▶ [T=1.01s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=1.53s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=2.29s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#4
▶ [T=3.05s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#4
▶ [T=3.78s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#4
▶ [T=4.31s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#4
▶ [T=5.07s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#4
▶ [T=5.81s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#4
▶ [T=6.33s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#4
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=99% OK | Loop#5
▶ [T=0.62s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=1.15s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=4.37s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#5
▶ [T=4.88s] Worst: MiddleFinger_1st_roll margin=-100% OVER! | Loop#5
▶ [T=5.38s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#5
▶ [T=5.92s] Worst: MiddleFinger_1st_roll margin=-100% OVER! | Loop#5
▶ [T=6.68s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#5
▶ [T=0.48s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=1.01s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=1.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=2.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=3.23s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#6
▶ [T=3.75s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#6
▶ [T=4.51s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#6
▶ [T=5.24s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#6
▶ [T=5.76s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#6
▶ [T=6.29s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#6
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=99% OK | Loop#7
▶ [T=0.53s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=1.03s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=4.09s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=4.60s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#7
▶ [T=5.33s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=5.86s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=6.63s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#7
▶ [T=0.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=1.25s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=1.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=2.54s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#8
▶ [T=3.30s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#8
▶ [T=3.80s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#8
[APPLY] Updated ALL joints
▶ [T=5.47s] Worst: IndexFinger-3rd-pitch margin=-100% OVER! | Loop#8
▶ [T=5.99s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#8
▶ [T=6.77s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#8
▶ [T=0.48s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=0.99s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#9
▶ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
[PAUSE] t=1.66s
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
[APPLY] Updated ALL joints
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
⏸ [T=1.66s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#9
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[APPLY] Updated ALL joints
[PLAY] t=1.66s
▶ [T=6.76s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#9
▶ [T=0.51s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=1.27s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=1.98s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#10
▶ [T=2.49s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=3.24s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=3.96s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#10
▶ [T=4.48s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#10
▶ [T=5.24s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#10
▶ [T=6.01s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#10
▶ [T=6.74s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#10
▶ [T=0.23s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#11
▶ [T=0.98s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=1.70s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#11
▶ [T=2.44s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=2.95s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=3.70s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#11
▶ [T=4.44s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#11
▶ [T=4.95s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#11
▶ [T=5.72s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#11
▶ [T=6.44s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#11
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=99% OK | Loop#12
▶ [T=0.75s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=1.45s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=1.97s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#12
▶ [T=2.47s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=3.19s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#12
▶ [T=3.71s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#12
▶ [T=4.46s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#12
▶ [T=5.19s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#12
▶ [T=5.71s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#12
▶ [T=6.47s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#12
▶ [T=0.27s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#13
▶ [T=0.99s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#13
▶ [T=1.51s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#13
▶ [T=2.26s] Worst: IndexFinger-1st-pitch margin=-100% OVER! | Loop#13
▶ [T=2.98s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#13
▶ [T=3.49s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#13
▶ [T=4.24s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#13
▶ [T=4.96s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#13
▶ [T=5.47s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#13
▶ [T=5.99s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#13
▶ [T=6.73s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#13
▶ [T=0.23s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#14
▶ [T=0.98s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#14
▶ [T=1.70s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#14
▶ [T=2.20s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#14
▶ [T=2.71s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#14
▶ [T=3.85s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#14
▶ [T=4.37s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#14
▶ [T=5.14s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#14
▶ [T=5.87s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#14
▶ [T=6.40s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#14
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=99% OK | Loop#15
▶ [T=0.73s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#15
▶ [T=1.24s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#15
▶ [T=1.99s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#15
▶ [T=2.70s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#15
▶ [T=3.21s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#15
▶ [T=3.97s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#15
▶ [T=4.73s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#15
▶ [T=5.46s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#15
▶ [T=5.98s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#15
▶ [T=6.76s] Worst: LittleFinger-2nd-pitch margin=-100% OVER! | Loop#15
▶ [T=0.25s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#16
▶ [T=0.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#16
▶ [T=1.54s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#16
▶ [T=2.26s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#16
▶ [T=2.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#16
▶ [T=3.53s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#16
▶ [T=4.26s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#16
▶ [T=4.92s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#16
▶ [T=5.70s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#16
▶ [T=6.48s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#16
▶ [T=0.25s] Worst: IndexFinger-1st-pitch margin=0% OVER! | Loop#17
▶ [T=0.77s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#17
▶ [T=1.52s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#17
▶ [T=2.23s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#17
▶ [T=2.74s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#17
▶ [T=3.50s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#17
▶ [T=4.27s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#17
▶ [T=5.00s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#17
▶ [T=5.52s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#17
▶ [T=6.28s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#17
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=99% OK | Loop#18
▶ [T=0.51s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#18
▶ [T=1.25s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#18
▶ [T=1.99s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#18
▶ [T=2.70s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#18
▶ [T=3.22s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#18
▶ [T=3.99s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#18
▶ [T=4.62s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#18
▶ [T=5.14s] Worst: RingFinger-2nd-pitch margin=-100% OVER! | Loop#18
▶ [T=5.64s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#18
▶ [T=6.17s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#18
▶ [T=0.00s] Worst: IndexFinger-2nd-pitch margin=99% OK | Loop#19
▶ [T=0.71s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#19
▶ [T=1.22s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#19
▶ [T=1.98s] Worst: MiddleFinger-2nd-pitch margin=-100% OVER! | Loop#19
▶ [T=2.84s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#19
▶ [T=3.51s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#19
▶ [T=4.03s] Worst: MiddleFinger-1st-pitch margin=-100% OVER! | Loop#19
▶ [T=4.76s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
[PAUSE] t=5.04s
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19
⏸ [T=5.04s] Worst: LittleFinger-3rd-pitch margin=-100% OVER! | Loop#19

Simulation ended
CSV log: motor_validation_log.csv


██████████████████████████████████████████████████████████████████████
█  MOTOR SIZING VALIDATION — FINAL REPORT
██████████████████████████████████████████████████████████████████████

────────────────────────────────────────────────────────────
  Motor: IndexFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=10% (<80%)
  ❌ Tracking            : max=1.2083rad (69.23°)
  ❌ Speed Margin        : -330% (EXCEEDED)
  ❌ Saturation          : 95.8% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=15% (<80%)
  ❌ Tracking            : max=0.2257rad (12.93°)
  ❌ Speed Margin        : -349% (EXCEEDED)
  ❌ Saturation          : 81.7% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=11% (<80%)
  ❌ Tracking            : max=0.4354rad (24.95°)
  ❌ Speed Margin        : -801% (EXCEEDED)
  ❌ Saturation          : 69.4% (frequent!)

────────────────────────────────────────────────────────────
  Motor: IndexFinger_1st_roll  ✅ PASS
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=56% (>20%)
  ✅ Thermal Load        : avg=3% (<80%)
  ✅ Tracking            : max=0.0001rad (0.01°)
  ✅ Speed Margin        : 99%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st-pitch  ⚠️  WARN
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=31% (>20%)
  ✅ Thermal Load        : avg=7% (<80%)
  ✅ Tracking            : max=0.0099rad (0.57°)
  ⚠️  Speed Margin        : 10% (close)
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-1st_roll  ✅ PASS
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=99% (>20%)
  ✅ Thermal Load        : avg=0% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=13% (<80%)
  ⚠️  Tracking            : max=0.1060rad (6.07°)
  ❌ Speed Margin        : -251% (EXCEEDED)
  ⚠️  Saturation          : 18.5%

────────────────────────────────────────────────────────────
  Motor: LittleFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=19% (<80%)
  ❌ Tracking            : max=0.2118rad (12.14°)
  ❌ Speed Margin        : -504% (EXCEEDED)
  ❌ Saturation          : 38.3% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=12% (<80%)
  ❌ Tracking            : max=1.7866rad (102.37°)
  ❌ Speed Margin        : -395% (EXCEEDED)
  ❌ Saturation          : 93.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=11% (<80%)
  ❌ Tracking            : max=0.9867rad (56.54°)
  ❌ Speed Margin        : -1071% (EXCEEDED)
  ❌ Saturation          : 85.2% (frequent!)

────────────────────────────────────────────────────────────
  Motor: MiddleFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=0% (SATURATED)
  ✅ Thermal Load        : avg=6% (<80%)
  ✅ Tracking            : max=0.0028rad (0.16°)
  ✅ Speed Margin        : 93%
  ✅ Saturation          : 0.6%

────────────────────────────────────────────────────────────
  Motor: MiddleFinger_1st_roll  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=25% (<80%)
  ⚠️  Tracking            : max=0.0742rad (4.25°)
  ❌ Speed Margin        : -111% (EXCEEDED)
  ⚠️  Saturation          : 17.3%

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=9% (<80%)
  ✅ Tracking            : max=0.0445rad (2.55°)
  ❌ Speed Margin        : -63% (EXCEEDED)
  ✅ Saturation          : 3.5%

────────────────────────────────────────────────────────────
  Motor: RingFinger-1st_roll  ✅ PASS
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ✅ Torque Margin       : min=99% (>20%)
  ✅ Thermal Load        : avg=0% (<80%)
  ✅ Tracking            : max=0.0000rad (0.00°)
  ✅ Speed Margin        : 100%
  ✅ Saturation          : 0.0%

────────────────────────────────────────────────────────────
  Motor: RingFinger-2nd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=16% (<80%)
  ❌ Tracking            : max=0.2375rad (13.61°)
  ❌ Speed Margin        : -587% (EXCEEDED)
  ❌ Saturation          : 31.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: RingFinger-3rd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=17% (<80%)
  ❌ Tracking            : max=0.2800rad (16.04°)
  ❌ Speed Margin        : -442% (EXCEEDED)
  ❌ Saturation          : 36.5% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-1st-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=16% (<80%)
  ❌ Tracking            : max=0.8684rad (49.75°)
  ❌ Speed Margin        : -436% (EXCEEDED)
  ❌ Saturation          : 92.1% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-2nd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=11% (<80%)
  ❌ Tracking            : max=0.3750rad (21.49°)
  ❌ Speed Margin        : -794% (EXCEEDED)
  ❌ Saturation          : 90.0% (frequent!)

────────────────────────────────────────────────────────────
  Motor: Thumb-3rd-pitch  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=9% (<80%)
  ❌ Tracking            : max=0.4875rad (27.93°)
  ❌ Speed Margin        : -870% (EXCEEDED)
  ❌ Saturation          : 60.0% (frequent!)

────────────────────────────────────────────────────────────
  Motor: thumb_1st_yaw  ❌ FAIL
  Spec: stall=1.4098Nm × gear=300 × eff=90% → out=380.66Nm
  Spec: 62099RPM / gear=300 → out=21.68rad/s
────────────────────────────────────────────────────────────
  ❌ Torque Margin       : min=-100% (SATURATED)
  ✅ Thermal Load        : avg=27% (<80%)
  ❌ Tracking            : max=0.5310rad (30.43°)
  ❌ Speed Margin        : -313% (EXCEEDED)
  ❌ Saturation          : 72.1% (frequent!)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ❌ OVERALL: SOME MOTORS NEED ATTENTION
     FAIL: IndexFinger-1st-pitch, IndexFinger-2nd-pitch, IndexFinger-3rd-pitch, LittleFinger-2nd-pitch, LittleFinger-3rd-pitch, MiddleFinger-1st-pitch, MiddleFinger-2nd-pitch, MiddleFinger-3rd-pitch, MiddleFinger_1st_roll, RingFinger-1st-pitch, RingFinger-2nd-pitch, RingFinger-3rd-pitch, Thumb-1st-pitch, Thumb-2nd-pitch, Thumb-3rd-pitch, thumb_1st_yaw
     WARN: LittleFinger-1st-pitch

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
     LittleFinger-2nd-pitch: Increase stall torque or gear ratio
     LittleFinger-2nd-pitch: Increase motor speed or reduce gear ratio
     LittleFinger-3rd-pitch: Increase stall torque or gear ratio
     LittleFinger-3rd-pitch: Increase PID gains or motor torque
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
     RingFinger-1st-pitch: Increase stall torque or gear ratio
     RingFinger-1st-pitch: Increase motor speed or reduce gear ratio
     RingFinger-2nd-pitch: Increase stall torque or gear ratio
     RingFinger-2nd-pitch: Increase PID gains or motor torque
     RingFinger-2nd-pitch: Increase motor speed or reduce gear ratio
     RingFinger-2nd-pitch: Increase stall torque or gear ratio
     RingFinger-3rd-pitch: Increase stall torque or gear ratio
     RingFinger-3rd-pitch: Increase PID gains or motor torque
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
     Thumb-3rd-pitch: Increase PID gains or motor torque
     Thumb-3rd-pitch: Increase motor speed or reduce gear ratio
     Thumb-3rd-pitch: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
     thumb_1st_yaw: Increase PID gains or motor torque
     thumb_1st_yaw: Increase motor speed or reduce gear ratio
     thumb_1st_yaw: Increase stall torque or gear ratio
