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
    MiddleFinger-1st-pitch: target=0.000, actual=-0.684, error=0.684, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.438, error=1.438, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.784, error=1.784, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.167, error=0.167, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.104, error=1.104, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=0.147, error=-0.147, ctrl=0.000, torque=-300.0Nm

[T=0.21s] Step 103/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.703, error=0.703, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.724, error=1.724, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.402, error=1.402, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.102, error=-0.102, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.225, error=1.225, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.348, error=0.348, ctrl=0.000, torque=-300.0Nm

[T=0.31s] Step 154/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.373, error=0.373, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.663, error=1.663, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-1.366, error=1.366, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.234, error=-0.234, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.292, error=1.292, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.695, error=1.695, ctrl=0.000, torque=-300.0Nm

[T=0.41s] Step 206/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.210, error=0.210, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.702, error=1.702, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=-0.864, error=0.864, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.193, error=-0.193, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=1.794, error=-1.794, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.759, error=0.759, ctrl=0.000, torque=-300.0Nm

[T=0.51s] Step 256/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.422, error=0.422, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.852, error=0.852, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=0.000, actual=0.484, error=-0.484, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.212, error=-0.212, ctrl=0.000, torque=240.9Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.640, error=0.640, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.181, error=1.181, ctrl=0.000, torque=300.0Nm
WARNING: Nan, Inf or huge value in QACC at DOF 13. The simulation is unstable. Time = 0.2520.


[T=0.62s] Step 308/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.141, error=0.141, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.914, error=0.914, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.037, actual=-0.690, error=0.653, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.082, error=0.082, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.580, error=0.580, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.498, error=0.498, ctrl=0.000, torque=300.0Nm

[T=0.72s] Step 358/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.804, error=0.804, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.581, error=1.581, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.141, actual=-1.620, error=1.479, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.063, error=0.063, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.213, error=1.213, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.283, error=0.283, ctrl=0.000, torque=-300.0Nm

[T=0.82s] Step 408/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.624, error=0.624, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.512, error=1.512, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.245, actual=-0.301, error=0.056, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.165, error=-0.165, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.310, error=1.310, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.895, error=1.895, ctrl=0.000, torque=-300.0Nm

[T=0.92s] Step 459/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.281, error=0.281, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.572, error=1.572, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.351, actual=-0.405, error=0.054, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.192, error=-0.192, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=2.447, error=-2.447, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-4.333, error=4.333, ctrl=0.000, torque=300.0Nm

[T=1.02s] Step 510/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.186, error=0.186, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.097, error=1.097, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.457, actual=-0.976, error=0.519, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.227, error=-0.227, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=0.532, error=-0.532, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.141, error=0.141, ctrl=0.000, torque=300.0Nm

[T=1.12s] Step 560/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.172, error=0.172, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.625, error=1.625, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.561, actual=-0.939, error=0.379, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.213, error=-0.213, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.308, error=1.308, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.046, error=2.046, ctrl=0.000, torque=-300.0Nm

[T=1.22s] Step 611/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.010, error=-0.010, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.689, error=1.689, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.667, actual=-0.858, error=0.191, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-1.397, error=1.397, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.013, error=1.013, ctrl=0.000, torque=300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.316, error=-0.316, ctrl=0.000, torque=300.0Nm

[T=1.33s] Step 662/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.435, error=0.435, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.752, error=1.752, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.772, actual=-0.449, error=-0.323, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.179, error=0.179, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.488, error=0.488, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.565, error=1.565, ctrl=0.000, torque=300.0Nm

[T=1.43s] Step 713/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.536, error=0.536, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.634, error=1.634, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.878, actual=-0.286, error=-0.592, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.064, error=0.064, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=2.105, error=-2.105, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-3.955, error=3.955, ctrl=0.000, torque=-300.0Nm

[T=1.53s] Step 763/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.399, error=0.399, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.681, error=1.681, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.982, actual=-0.115, error=-0.867, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.190, error=-0.190, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=0.637, error=-0.637, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.380, error=1.380, ctrl=0.000, torque=-300.0Nm

[T=1.63s] Step 814/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.375, error=0.375, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.491, error=0.491, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.088, actual=-0.236, error=-0.852, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-1.030, error=1.030, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.969, error=1.969, ctrl=0.000, torque=-300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.911, error=-0.911, ctrl=0.000, torque=300.0Nm

[T=1.73s] Step 866/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.406, error=0.406, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.472, error=0.472, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.196, actual=-0.065, error=-1.131, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.187, error=0.187, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.892, error=0.892, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.131, error=0.131, ctrl=0.000, torque=300.0Nm

[T=1.84s] Step 918/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.708, error=-0.708, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-3.511, error=3.511, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.304, actual=-0.022, error=-1.282, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.236, error=0.236, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.159, error=1.159, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.831, error=1.831, ctrl=0.000, torque=-300.0Nm

[T=1.94s] Step 970/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.686, error=-0.686, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.490, error=1.490, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.412, actual=-0.300, error=-1.112, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st-pitch: target=0.000, actual=-0.463, error=0.463, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-3.012, error=3.012, ctrl=0.000, torque=-300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.364, error=-0.364, ctrl=0.000, torque=300.0Nm

[T=2.04s] Step 1021/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.388, error=0.388, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.472, error=1.472, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.518, actual=-0.137, error=-1.381, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.119, error=-0.119, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.205, error=0.205, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.779, error=2.779, ctrl=0.000, torque=-300.0Nm

[T=2.14s] Step 1071/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.377, error=0.377, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.513, error=1.513, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.520, actual=-0.405, error=-1.115, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.123, error=-0.123, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.361, error=0.361, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.288, error=1.288, ctrl=0.000, torque=-300.0Nm

[T=2.25s] Step 1122/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.127, error=0.127, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.715, error=1.715, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.415, actual=-0.359, error=-1.056, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.132, error=-0.132, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.468, error=0.468, ctrl=0.000, torque=-300.0Nm
    LittleFinger-3rd-pitch: target=0.000, actual=0.330, error=-0.330, ctrl=0.000, torque=300.0Nm

[T=2.35s] Step 1173/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.020, error=0.020, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.421, error=1.421, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.310, actual=-0.212, error=-1.097, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=0.111, error=-0.111, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.482, error=0.482, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-2.959, error=2.959, ctrl=0.000, torque=-300.0Nm

[T=2.45s] Step 1225/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.352, error=0.352, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.786, error=1.786, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.202, actual=-0.203, error=-0.999, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.051, error=0.051, ctrl=0.000, torque=-300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.579, error=0.579, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.862, error=0.862, ctrl=0.000, torque=300.0Nm

[T=2.55s] Step 1276/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.497, error=0.497, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.134, error=-0.134, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-1.097, actual=-0.066, error=-1.031, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.121, error=0.121, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.467, error=0.467, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.853, error=0.853, ctrl=0.000, torque=-300.0Nm

[T=2.65s] Step 1327/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.506, error=0.506, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=0.181, error=-0.181, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.991, actual=-0.269, error=-0.723, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.182, error=0.182, ctrl=0.000, torque=31.2Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.716, error=0.716, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.819, error=0.819, ctrl=0.000, torque=-300.0Nm

[T=2.76s] Step 1378/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.319, error=0.319, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.536, error=0.536, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.886, actual=0.065, error=-0.951, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.220, error=0.220, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=0.313, error=-0.313, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=3.675, error=-3.675, ctrl=0.000, torque=-300.0Nm

[T=2.86s] Step 1428/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.011, error=0.011, torque=-300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-0.997, error=0.997, torque=300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.783, actual=-0.319, error=-0.464, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.137, error=0.137, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.892, error=1.892, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.346, error=0.346, ctrl=0.000, torque=300.0Nm

[T=2.96s] Step 1480/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=-0.225, error=0.225, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=-1.588, error=1.588, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.675, actual=-0.185, error=-0.491, torque=-300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.166, error=0.166, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-0.638, error=0.638, ctrl=0.000, torque=-300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-0.513, error=0.513, ctrl=0.000, torque=-300.0Nm

[T=3.06s] Step 1530/3418
  Moving joints (should track trajectory):
    MiddleFinger-1st-pitch: target=0.000, actual=0.004, error=-0.004, torque=300.0Nm (range=1.571)
    MiddleFinger-2nd-pitch: target=0.000, actual=1.224, error=-1.224, torque=-300.0Nm (range=0.221)
    IndexFinger-1st-pitch: target=-0.572, actual=-0.170, error=-0.403, torque=300.0Nm (range=1.570)
  ⚠️ Static joints with large error (should stay at 0):
    LittleFinger-1st_roll: target=0.000, actual=-0.064, error=0.064, ctrl=0.000, torque=300.0Nm
    LittleFinger-1st-pitch: target=0.000, actual=-1.167, error=1.167, ctrl=0.000, torque=300.0Nm
    LittleFinger-2nd-pitch: target=0.000, actual=-1.342, error=1.342, ctrl=0.000, torque=-300.0Nm


