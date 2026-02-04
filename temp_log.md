~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh 
Installing dependencies (mujoco, matplotlib, numpy)...
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

========================================
Enter choice [0 to auto-optimize, 2 for manual tuning]: 2
Starting Motor Validation...
Loading model: direct_hand_parm.xml
Loaded Recording 1768623534448. Mode: Motor Validation

=== Configuration ===
Control Params (global): kp=200.0, kv=20.0
Motor Specs (default): gear=50.0, forcelim=300.0
Total joints: 20, Total actuators: 20

First 3 joints:
  LittleFinger-1st_roll: gear=50.0, forcelim=300.0
  LittleFinger-1st-pitch: gear=50.0, forcelim=300.0
  LittleFinger-2nd-pitch: gear=50.0, forcelim=300.0

======================================================================
TRAJECTORY ANALYSIS
======================================================================
Total joints in model: 20
Joints in recording: 20

Recorded joints: ['IndexFinger-1st-pitch', 'IndexFinger-2nd-pitch', 'IndexFinger-3rd-pitch', 'IndexFinger_1st_roll', 'LittleFinger-1st-pitch', 'LittleFinger-1st_roll', 'LittleFinger-2nd-pitch', 'LittleFinger-3rd-pitch', 'MiddleFinger-1st-pitch', 'MiddleFinger-2nd-pitch', 'MiddleFinger-3rd-pitch', 'MiddleFinger_1st_roll', 'RingFinger-1st-pitch', 'RingFinger-1st_roll', 'RingFinger-2nd-pitch', 'RingFinger-3rd-pitch', 'Thumb-1st-pitch', 'Thumb-2nd-pitch', 'Thumb-3rd-pitch', 'thumb_1st_yaw']
  LittleFinger-1st_roll: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  LittleFinger-1st-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  LittleFinger-2nd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  LittleFinger-3rd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  RingFinger-1st_roll: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  RingFinger-1st-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  RingFinger-2nd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  RingFinger-3rd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  MiddleFinger_1st_roll: RECORDED - start=0.0000, range=[0.0000, 0.0255], delta=0.0255
  MiddleFinger-1st-pitch: RECORDED - start=0.0000, range=[-1.5708, 0.0000], delta=1.5708
  MiddleFinger-2nd-pitch: RECORDED - start=0.0000, range=[-0.2208, 0.0000], delta=0.2208
  MiddleFinger-3rd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  IndexFinger_1st_roll: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  IndexFinger-1st-pitch: RECORDED - start=0.0000, range=[-1.5708, 0.0000], delta=1.5708
  IndexFinger-2nd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  IndexFinger-3rd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  thumb_1st_yaw: RECORDED - start=0.0000, range=[-0.3936, 0.0000], delta=0.3936
  Thumb-1st-pitch: RECORDED - start=0.0000, range=[-0.9408, 0.0000], delta=0.9408
  Thumb-2nd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  Thumb-3rd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
======================================================================

Creating UI window...
Showing UI window...
UI window created successfully!
Figure has 22 axes

=== Motor Validation Mode Started ===
Duration: 6.83s | Joints: 20 | CSV: motor_validation_log.csv
Joints in recording: 20 / 20
UI: Adjust Control (top), then Motor specs per joint (middle)
    Click 'Apply Control to All' to update controller globally
    Select joint, adjust motor specs, click 'Apply Motor' for that joint

Initializing robot to first keyframe position...
WARNING: Nan, Inf or huge value in QACC at DOF 14. The simulation is unstable. Time = 0.0080.

Robot initialized and stabilized
DISPLAY: :0
MUJOCO_GL: NOT SET (using default)
MuJoCo version: 3.4.0
Attempting to create viewer window...
Starting motor validation simulation...

[T=0.00s] Step 0/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=6.217, error=-6.217, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=19.096, error=-19.096, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-4.436, error=4.436, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-5.032, error=5.032, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=1.890, error=-1.890, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=4.964, error=-4.964, ctrl=0.000, torque=300.0Nm

=== First Step Debug (after mj_step) ===
  LittleFinger-1st_roll:
    target=0.0000, actual=-5.0321, error=5.0321
    force=300.0000, kp=200.0
    expected_force = kp * error = 1006.4111
  LittleFinger-1st-pitch:
    target=0.0000, actual=1.8904, error=-1.8904
    force=-300.0000, kp=200.0
    expected_force = kp * error = -378.0739
  LittleFinger-2nd-pitch:
    target=0.0000, actual=4.9640, error=-4.9640
    force=300.0000, kp=200.0
    expected_force = kp * error = -992.7901


[T=0.11s] Step 113/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=5.502, error=-5.502, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=3.866, error=-3.866, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=59.418, error=-59.418, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-2.596, error=2.596, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.585, error=1.585, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=3.361, error=-3.361, ctrl=0.000, torque=-300.0Nm

[T=0.31s] Step 309/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=0.049, error=-0.049, torque=0.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    MiddleFinger-3rd-pitch: target=0.000, actual=-0.232, error=0.232, ctrl=0.000, torque=-7.0Nm
    IndexFinger-2nd-pitch: target=0.000, actual=-0.322, error=0.322, ctrl=0.000, torque=-10.5Nm

[T=0.50s] Step 499/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=5.502, error=-5.502, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=3.866, error=-3.866, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=59.418, error=-59.418, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-2.596, error=2.596, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.585, error=1.585, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=3.361, error=-3.361, ctrl=0.000, torque=-300.0Nm

[T=0.60s] Step 600/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.157, error=0.157, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=5.693, error=-5.693, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.021, actual=-3.697, error=3.677, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-4.002, error=4.002, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=3.000, error=-3.000, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=8.124, error=-8.124, ctrl=0.000, torque=300.0Nm

[T=0.71s] Step 706/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=6.217, error=-6.217, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=19.096, error=-19.096, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.131, actual=-4.436, error=4.305, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-5.032, error=5.032, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=1.890, error=-1.890, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=4.964, error=-4.964, ctrl=0.000, torque=300.0Nm

[T=0.81s] Step 807/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.347, error=1.347, torque=-135.9Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.400, error=1.400, torque=-68.5Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.236, actual=-5.471, error=5.235, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    MiddleFinger-3rd-pitch: target=0.000, actual=9.621, error=-9.621, ctrl=0.000, torque=300.0Nm
    IndexFinger-2nd-pitch: target=0.000, actual=10.055, error=-10.055, ctrl=0.000, torque=300.0Nm
    IndexFinger-3rd-pitch: target=0.000, actual=-4.089, error=4.089, ctrl=0.000, torque=-123.5Nm

[T=0.91s] Step 910/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.157, error=0.157, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=5.693, error=-5.693, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.343, actual=-3.697, error=3.355, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-4.002, error=4.002, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=3.000, error=-3.000, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=8.124, error=-8.124, ctrl=0.000, torque=300.0Nm

[T=1.01s] Step 1012/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=4.892, error=-4.892, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-9.478, error=9.478, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.449, actual=-74.984, error=74.536, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.500, error=0.500, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=1.708, error=-1.708, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=2.273, error=-2.273, ctrl=0.000, torque=300.0Nm

[T=1.11s] Step 1113/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=4.492, error=-4.492, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-13.080, error=13.080, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.553, actual=1962.080, error=-1962.633, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=1.191, error=-1.191, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.290, error=1.290, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=1.670, error=-1.670, ctrl=0.000, torque=300.0Nm

[T=1.21s] Step 1214/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=6.217, error=-6.217, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=19.096, error=-19.096, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.658, actual=-4.436, error=3.778, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-5.032, error=5.032, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=1.890, error=-1.890, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=4.964, error=-4.964, ctrl=0.000, torque=300.0Nm

[T=1.32s] Step 1318/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=5.502, error=-5.502, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=3.866, error=-3.866, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.766, actual=59.418, error=-60.184, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-2.596, error=2.596, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.585, error=1.585, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=3.361, error=-3.361, ctrl=0.000, torque=-300.0Nm

[T=1.51s] Step 1514/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=2.782, error=-2.782, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=12.600, error=-12.600, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.970, actual=6.338, error=-7.308, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-7.786, error=7.786, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=5.737, error=-5.737, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.510, error=1.510, ctrl=0.000, torque=-300.0Nm

[T=1.62s] Step 1615/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.157, error=0.157, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=5.693, error=-5.693, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.075, actual=-3.697, error=2.623, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-4.002, error=4.002, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=3.000, error=-3.000, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=8.124, error=-8.124, ctrl=0.000, torque=300.0Nm

[T=1.73s] Step 1731/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=2.782, error=-2.782, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=12.600, error=-12.600, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.195, actual=6.338, error=-7.533, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-7.786, error=7.786, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=5.737, error=-5.737, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.510, error=1.510, ctrl=0.000, torque=-300.0Nm

[T=1.91s] Step 1913/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.384, actual=0.049, error=-1.433, torque=0.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    MiddleFinger-3rd-pitch: target=0.000, actual=-0.232, error=0.232, ctrl=0.000, torque=-7.0Nm
    IndexFinger-2nd-pitch: target=0.000, actual=-0.322, error=0.322, ctrl=0.000, torque=-10.5Nm

[T=2.02s] Step 2015/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=6.217, error=-6.217, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=19.096, error=-19.096, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.490, actual=-4.436, error=2.946, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-5.032, error=5.032, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=1.890, error=-1.890, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=4.964, error=-4.964, ctrl=0.000, torque=300.0Nm

[T=2.12s] Step 2120/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.543, actual=0.049, error=-1.592, torque=0.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    MiddleFinger-3rd-pitch: target=0.000, actual=-0.232, error=0.232, ctrl=0.000, torque=-7.0Nm
    IndexFinger-2nd-pitch: target=0.000, actual=-0.322, error=0.322, ctrl=0.000, torque=-10.5Nm

[T=2.22s] Step 2224/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=4.492, error=-4.492, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-13.080, error=13.080, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.436, actual=1962.080, error=-1963.516, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=1.191, error=-1.191, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.290, error=1.290, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=1.670, error=-1.670, ctrl=0.000, torque=300.0Nm

[T=2.33s] Step 2326/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.330, actual=0.049, error=-1.379, torque=0.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    MiddleFinger-3rd-pitch: target=0.000, actual=-0.232, error=0.232, ctrl=0.000, torque=-7.0Nm
    IndexFinger-2nd-pitch: target=0.000, actual=-0.322, error=0.322, ctrl=0.000, torque=-10.5Nm

[T=2.43s] Step 2427/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.000, error=-0.000, torque=0.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.226, actual=0.049, error=-1.275, torque=0.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    MiddleFinger-3rd-pitch: target=0.000, actual=-0.232, error=0.232, ctrl=0.000, torque=-7.0Nm
    IndexFinger-2nd-pitch: target=0.000, actual=-0.322, error=0.322, ctrl=0.000, torque=-10.5Nm

[T=2.53s] Step 2532/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.157, error=0.157, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=5.693, error=-5.693, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.117, actual=-3.697, error=2.580, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-4.002, error=4.002, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=3.000, error=-3.000, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=8.124, error=-8.124, ctrl=0.000, torque=300.0Nm

[T=2.73s] Step 2729/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.157, error=0.157, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=5.693, error=-5.693, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.914, actual=-3.697, error=2.783, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-4.002, error=4.002, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=3.000, error=-3.000, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=8.124, error=-8.124, ctrl=0.000, torque=300.0Nm
Motor validation simulation completed successfully
Motor validation complete. Log saved to motor_validation_log.csv
