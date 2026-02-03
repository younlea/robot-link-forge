~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh 
Creating virtual environment...
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
  IndexFinger-1st-pitch: RECORDED - start=0.0000, range=[-1.5698, 0.0000], delta=1.5698
  IndexFinger-2nd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  IndexFinger-3rd-pitch: RECORDED - start=0.0000, range=[0.0000, 0.0000], delta=0.0000
  thumb_1st_yaw: RECORDED - start=0.0000, range=[-0.3936, 0.0000], delta=0.3936
  Thumb-1st-pitch: RECORDED - start=0.0000, range=[-0.9402, 0.0000], delta=0.9402
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

Initialized robot to first keyframe position
DISPLAY: :0
MUJOCO_GL: NOT SET (using default)
MuJoCo version: 3.4.0
Attempting to create viewer window...
Starting motor validation simulation...

[T=0.00s] Step 0/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.001, error=0.001, torque=0.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.004, error=-0.004, torque=0.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=0.003, error=-0.003, torque=0.0Nm (range=1.570)

=== First Step Debug (after mj_step) ===
  LittleFinger-1st_roll:
    target=0.0000, actual=-0.0000, error=0.0000
    force=0.0000, kp=200.0
    expected_force = kp * error = 0.0030
  LittleFinger-1st-pitch:
    target=0.0000, actual=0.0000, error=-0.0000
    force=0.0000, kp=200.0
    expected_force = kp * error = -0.0060
  LittleFinger-2nd-pitch:
    target=0.0000, actual=0.0000, error=-0.0000
    force=0.0000, kp=200.0
    expected_force = kp * error = -0.0025


[T=0.10s] Step 50/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.583, error=0.583, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.854, error=0.854, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.773, error=1.773, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.179, error=0.179, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.978, error=0.978, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.183, error=0.183, ctrl=0.000, torque=-300.0Nm

[T=0.20s] Step 101/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.918, error=0.918, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.250, error=1.250, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.461, error=1.461, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-1.347, error=1.347, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-0.133, error=0.133, ctrl=0.000, torque=300.0Nm
    RingFinger-1st-pitch: target=0.000, actual=-1.590, error=1.590, ctrl=0.000, torque=-300.0Nm

[T=0.31s] Step 153/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.454, error=0.454, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.776, error=1.776, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.501, error=1.501, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.188, error=-0.188, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.477, error=1.477, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.622, error=1.622, ctrl=0.000, torque=300.0Nm

[T=0.41s] Step 204/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.185, error=0.185, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.670, error=1.670, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.947, error=0.947, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.229, error=-0.229, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=2.350, error=-2.350, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-3.236, error=3.236, ctrl=0.000, torque=-300.0Nm

[T=0.51s] Step 256/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.266, error=0.266, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.318, error=1.318, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.674, error=0.674, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.220, error=-0.220, ctrl=0.000, torque=96.6Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.737, error=0.737, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-0.083, error=0.083, ctrl=0.000, torque=-300.0Nm
WARNING: Nan, Inf or huge value in QACC at DOF 13. The simulation is unstable. Time = 0.2520.


[T=0.62s] Step 307/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.008, error=0.008, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.008, error=0.008, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.035, actual=-0.108, error=0.073, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-0.191, error=0.191, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.334, error=0.334, ctrl=0.000, torque=-300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.845, error=-0.845, ctrl=0.000, torque=300.0Nm

[T=0.72s] Step 359/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.796, error=0.796, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.480, error=1.480, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.143, actual=-1.766, error=1.623, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.135, error=0.135, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.998, error=0.998, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.784, error=0.784, ctrl=0.000, torque=-300.0Nm

[T=0.82s] Step 409/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.693, error=0.693, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.679, error=1.679, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.247, actual=-1.022, error=0.775, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.135, error=-0.135, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.187, error=1.187, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.710, error=0.710, ctrl=0.000, torque=-300.0Nm

[T=0.92s] Step 460/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.339, error=0.339, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.534, error=1.534, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.353, actual=-0.143, error=-0.210, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.223, error=-0.223, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.325, error=1.325, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.678, error=1.678, ctrl=0.000, torque=-300.0Nm

[T=1.02s] Step 511/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.220, error=0.220, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.680, error=1.680, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.459, actual=-0.469, error=0.010, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.193, error=-0.193, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=1.794, error=-1.794, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.759, error=0.759, ctrl=0.000, torque=-300.0Nm

[T=1.13s] Step 563/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.166, error=0.166, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.949, error=0.949, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.567, actual=-1.078, error=0.512, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.213, error=-0.213, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.455, error=0.455, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.550, error=1.550, ctrl=0.000, torque=300.0Nm

[T=1.23s] Step 614/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.055, error=-0.055, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-3.659, error=3.659, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.673, actual=-1.169, error=0.497, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.247, error=-0.247, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.583, error=1.583, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.398, error=1.398, ctrl=0.000, torque=300.0Nm

[T=1.33s] Step 666/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.214, error=0.214, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.914, error=1.914, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.781, actual=-0.533, error=-0.248, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.200, error=-0.200, ctrl=0.000, torque=67.9Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.099, error=1.099, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.626, error=0.626, ctrl=0.000, torque=-300.0Nm

[T=1.44s] Step 718/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.689, error=0.689, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.071, error=1.071, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.889, actual=0.003, error=-0.891, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.104, error=-0.104, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.776, error=0.776, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.495, error=0.495, ctrl=0.000, torque=-300.0Nm

