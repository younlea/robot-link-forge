younleakim@younleakim-400TEA-400SEA:~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh 
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

Initialized robot to first keyframe position
DISPLAY: :0
MUJOCO_GL: NOT SET (using default)
MuJoCo version: 3.4.0
Attempting to create viewer window...
Starting motor validation simulation...

[T=0.00s] Step 0/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.000, error=0.000, torque=0.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.001, error=-0.001, torque=0.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=0.001, error=-0.001, torque=0.0Nm (range=1.571)

=== First Step Debug (after mj_step) ===
  LittleFinger-1st_roll:
    target=0.0000, actual=-0.0000, error=0.0000
    force=0.0000, kp=200.0
    expected_force = kp * error = 0.0008
  LittleFinger-1st-pitch:
    target=0.0000, actual=0.0000, error=-0.0000
    force=0.0000, kp=200.0
    expected_force = kp * error = -0.0015
  LittleFinger-2nd-pitch:
    target=0.0000, actual=0.0000, error=-0.0000
    force=0.0000, kp=200.0
    expected_force = kp * error = -0.0006


[T=0.10s] Step 101/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.018, error=0.018, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.345, error=0.345, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.255, error=0.255, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-0.115, error=0.115, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=0.058, error=-0.058, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-1.236, error=1.236, ctrl=0.000, torque=-300.0Nm

[T=0.20s] Step 202/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.004, error=-0.004, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.900, error=0.900, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.365, error=0.365, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.074, error=-0.074, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.215, error=0.215, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.619, error=0.619, ctrl=0.000, torque=300.0Nm

[T=0.31s] Step 305/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.010, error=0.010, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.522, error=1.522, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.490, error=0.490, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.112, error=-0.112, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.288, error=0.288, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.179, error=1.179, ctrl=0.000, torque=-300.0Nm

[T=0.41s] Step 410/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.044, error=0.044, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.582, error=1.582, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.361, error=0.361, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.164, error=-0.164, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.527, error=0.527, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.043, error=1.043, ctrl=0.000, torque=300.0Nm

[T=0.51s] Step 514/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.002, error=0.002, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.089, error=1.089, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.476, error=0.476, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.177, error=-0.177, ctrl=0.000, torque=245.7Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.635, error=0.635, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.403, error=1.403, ctrl=0.000, torque=300.0Nm

[T=0.62s] Step 616/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.004, error=-0.004, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.772, error=0.772, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.037, actual=-0.540, error=0.503, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.178, error=-0.178, ctrl=0.000, torque=185.1Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.609, error=0.609, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.284, error=1.284, ctrl=0.000, torque=-300.0Nm

[T=0.72s] Step 716/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.012, error=0.012, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.893, error=0.893, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.141, actual=-0.521, error=0.380, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.176, error=-0.176, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.744, error=0.744, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.854, error=0.854, ctrl=0.000, torque=-300.0Nm

[T=0.82s] Step 820/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.038, error=0.038, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.158, error=1.158, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.249, actual=-0.464, error=0.215, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.180, error=-0.180, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.824, error=0.824, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.417, error=0.417, ctrl=0.000, torque=-300.0Nm

[T=0.92s] Step 923/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.027, error=0.027, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.308, error=1.308, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.356, actual=-0.442, error=0.085, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.175, error=-0.175, ctrl=0.000, torque=154.9Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.737, error=0.737, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.816, error=0.816, ctrl=0.000, torque=300.0Nm

[T=1.03s] Step 1025/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.050, error=0.050, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.030, error=1.030, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.462, actual=-0.438, error=-0.024, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.167, error=-0.167, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.658, error=0.658, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.546, error=1.546, ctrl=0.000, torque=-300.0Nm

[T=1.13s] Step 1128/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.038, error=0.038, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.987, error=0.987, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.569, actual=-0.464, error=-0.105, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.144, error=-0.144, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.706, error=0.706, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.418, error=1.418, ctrl=0.000, torque=300.0Nm

[T=1.23s] Step 1231/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.050, error=0.050, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.968, error=0.968, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.676, actual=-0.414, error=-0.262, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.133, error=-0.133, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.625, error=0.625, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.177, error=1.177, ctrl=0.000, torque=300.0Nm

[T=1.33s] Step 1334/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.102, error=0.102, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.949, error=0.949, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.783, actual=-0.347, error=-0.436, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.115, error=-0.115, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.554, error=0.554, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.666, error=1.666, ctrl=0.000, torque=-300.0Nm

[T=1.43s] Step 1434/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.080, error=0.080, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.394, error=1.394, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.887, actual=-0.332, error=-0.554, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.095, error=-0.095, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.582, error=0.582, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.354, error=1.354, ctrl=0.000, torque=-300.0Nm

[T=1.54s] Step 1538/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.208, error=0.208, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.185, error=1.185, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.995, actual=-0.090, error=-0.905, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.081, error=-0.081, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.456, error=0.456, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.253, error=1.253, ctrl=0.000, torque=-300.0Nm

[T=1.64s] Step 1642/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.004, error=0.004, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.466, error=1.466, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.103, actual=-0.084, error=-1.018, torque=-300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.058, error=-0.058, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.433, error=0.433, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.295, error=1.295, ctrl=0.000, torque=-300.0Nm

[T=1.74s] Step 1742/6835
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.017, error=0.017, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.471, error=1.471, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.206, actual=0.004, error=-1.210, torque=300.0Nm (range=1.571)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-0.317, error=0.317, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.468, error=1.468, ctrl=0.000, torque=-300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-0.078, error=0.078, ctrl=0.000, torque=300.0Nm
^CTraceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 596, in <module>
    viewer.sync()
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/viewer.py", line 236, in sync
    sim.sync(state_only)  # locks internally
KeyboardInterrupt
