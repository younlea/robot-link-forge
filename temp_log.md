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
