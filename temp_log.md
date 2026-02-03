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


[T=0.10s] Step 51/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.583, error=0.583, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.854, error=0.854, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.263, error=1.263, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.179, error=0.179, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.978, error=0.978, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.183, error=0.183, ctrl=0.000, torque=-300.0Nm

[T=0.21s] Step 102/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.915, error=0.915, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.878, error=0.878, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.365, error=1.365, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-1.394, error=1.394, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=0.486, error=-0.486, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-1.227, error=1.227, ctrl=0.000, torque=-300.0Nm

[T=0.31s] Step 154/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.585, error=0.585, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.678, error=1.678, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.509, error=0.509, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.190, error=-0.190, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.429, error=1.429, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.837, error=1.837, ctrl=0.000, torque=-300.0Nm

[T=0.41s] Step 206/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.748, error=0.748, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.717, error=0.717, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=0.685, error=-0.685, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.234, error=-0.234, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=2.424, error=-2.424, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-3.658, error=3.658, ctrl=0.000, torque=-300.0Nm

[T=0.52s] Step 258/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.041, error=1.041, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.626, error=1.626, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.711, error=0.711, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.222, error=-0.222, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=0.201, error=-0.201, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-1.013, error=1.013, ctrl=0.000, torque=-300.0Nm

[T=0.62s] Step 308/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.002, error=1.002, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.664, error=1.664, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.037, actual=-0.777, error=0.740, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.164, error=-0.164, ctrl=0.000, torque=98.3Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.306, error=1.306, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.138, error=2.138, ctrl=0.000, torque=300.0Nm

[T=0.72s] Step 359/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.957, error=0.957, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.653, error=1.653, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.143, actual=-0.626, error=0.483, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.225, error=0.225, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.323, error=1.323, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.279, error=1.279, ctrl=0.000, torque=300.0Nm

[T=0.82s] Step 410/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.102, error=1.102, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.563, error=1.563, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.249, actual=-0.463, error=0.214, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-1.123, error=1.123, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.184, error=1.184, ctrl=0.000, torque=-300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.578, error=-0.578, ctrl=0.000, torque=300.0Nm

[T=0.92s] Step 461/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.943, error=0.943, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.654, error=1.654, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.355, actual=0.024, error=-0.379, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.188, error=-0.188, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.005, error=1.005, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.185, error=1.185, ctrl=0.000, torque=300.0Nm

[T=1.03s] Step 514/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.616, error=0.616, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.231, error=1.231, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.465, actual=-0.298, error=-0.167, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.146, error=-0.146, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.344, error=1.344, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.653, error=1.653, ctrl=0.000, torque=-300.0Nm

[T=1.13s] Step 565/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.399, error=0.399, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.558, error=1.558, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.571, actual=-0.006, error=-0.565, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.223, error=-0.223, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.054, error=1.054, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.727, error=1.727, ctrl=0.000, torque=300.0Nm

[T=1.24s] Step 617/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.097, error=0.097, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.220, error=1.220, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.679, actual=-0.178, error=-0.501, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-0.564, error=0.564, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.276, error=1.276, ctrl=0.000, torque=-300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-0.615, error=0.615, ctrl=0.000, torque=-300.0Nm

[T=1.34s] Step 668/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.197, error=0.197, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.092, error=1.092, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.785, actual=-0.240, error=-0.545, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.192, error=0.192, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.758, error=0.758, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.636, error=2.636, ctrl=0.000, torque=300.0Nm

[T=1.44s] Step 719/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.494, error=0.494, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.245, error=1.245, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.891, actual=-1.235, error=0.344, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.141, error=0.141, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.215, error=0.215, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=4.016, error=-4.016, ctrl=0.000, torque=-300.0Nm

[T=1.54s] Step 771/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.351, error=0.351, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.865, error=1.865, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.999, actual=-0.925, error=-0.074, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.158, error=0.158, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=4.875, error=-4.875, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.337, error=-0.337, ctrl=0.000, torque=-300.0Nm

[T=1.65s] Step 823/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.151, error=0.151, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.552, error=1.552, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.107, actual=0.024, error=-1.131, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.217, error=-0.217, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.129, error=0.129, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=2.133, error=-2.133, ctrl=0.000, torque=300.0Nm

[T=1.75s] Step 875/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.117, error=0.117, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.656, error=1.656, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.215, actual=-0.910, error=-0.305, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.173, error=-0.173, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-2.244, error=2.244, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.580, error=2.580, ctrl=0.000, torque=-300.0Nm

[T=1.85s] Step 926/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.349, error=0.349, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.922, error=0.922, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.321, actual=-0.074, error=-1.246, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.146, error=-0.146, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.298, error=1.298, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.760, error=1.760, ctrl=0.000, torque=-300.0Nm

[T=1.96s] Step 978/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.170, error=0.170, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.721, error=1.721, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.429, actual=-0.285, error=-1.144, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.154, error=-0.154, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.202, error=0.202, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=0.090, error=-0.090, ctrl=0.000, torque=300.0Nm

[T=2.06s] Step 1029/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.155, error=0.155, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.758, error=1.758, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.534, actual=-0.172, error=-1.363, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.134, error=-0.134, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.493, error=0.493, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=0.056, error=-0.056, ctrl=0.000, torque=300.0Nm

[T=2.16s] Step 1080/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.124, error=0.124, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.622, error=0.622, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.502, actual=-0.038, error=-1.464, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.106, error=-0.106, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.274, error=0.274, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.161, error=0.161, ctrl=0.000, torque=300.0Nm

[T=2.27s] Step 1132/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.068, error=0.068, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.033, error=0.033, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.394, actual=-0.048, error=-1.346, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.056, error=-0.056, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.282, error=0.282, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.840, error=1.840, ctrl=0.000, torque=-300.0Nm

[T=2.37s] Step 1185/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.574, error=0.574, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.717, error=0.717, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.285, actual=-0.066, error=-1.219, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.055, error=0.055, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.474, error=0.474, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.656, error=0.656, ctrl=0.000, torque=300.0Nm

[T=2.47s] Step 1235/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.198, error=1.198, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.422, error=1.422, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.181, actual=0.075, error=-1.257, torque=149.3Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.140, error=0.140, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.744, error=0.744, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.644, error=1.644, ctrl=0.000, torque=-300.0Nm

[T=2.57s] Step 1286/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.002, error=1.002, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.794, error=1.794, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.076, actual=-0.302, error=-0.774, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.082, error=-0.082, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.637, error=0.637, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.685, error=1.685, ctrl=0.000, torque=-300.0Nm

[T=2.67s] Step 1336/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.076, error=1.076, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.403, error=0.403, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.973, actual=-0.513, error=-0.460, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.144, error=0.144, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.169, error=0.169, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.165, error=2.165, ctrl=0.000, torque=-300.0Nm

[T=2.78s] Step 1387/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.690, error=0.690, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.573, error=1.573, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.868, actual=0.024, error=-0.891, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.165, error=0.165, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.867, error=1.867, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.424, error=-0.424, ctrl=0.000, torque=300.0Nm

[T=2.88s] Step 1439/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-1.224, error=1.224, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.051, error=1.051, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.760, actual=0.056, error=-0.816, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.106, error=0.106, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.200, error=0.200, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.027, error=2.027, ctrl=0.000, torque=-300.0Nm

[T=2.98s] Step 1491/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.533, error=0.533, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.841, error=1.841, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.653, actual=-0.576, error=-0.077, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-0.343, error=0.343, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.561, error=0.561, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=-0.593, error=0.593, ctrl=0.000, torque=300.0Nm

[T=3.09s] Step 1543/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.093, error=-0.093, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.728, error=0.728, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.545, actual=-0.795, error=0.250, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.092, error=0.092, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.125, error=0.125, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.584, error=1.584, ctrl=0.000, torque=-300.0Nm

[T=3.19s] Step 1595/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.373, error=0.373, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.321, error=0.321, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.438, actual=-0.273, error=-0.165, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.099, error=0.099, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.107, error=0.107, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.123, error=1.123, ctrl=0.000, torque=300.0Nm

[T=3.29s] Step 1647/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.700, error=0.700, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.341, error=1.341, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.330, actual=-0.265, error=-0.066, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.091, error=0.091, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=0.100, error=-0.100, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.760, error=1.760, ctrl=0.000, torque=300.0Nm
Motor validation simulation completed successfully
Motor validation complete. Log saved to motor_validation_log.csv
