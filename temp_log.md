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
Enter choice [0 to auto-optimize, 2 for manual tuning]: 0
Running automatic parameter optimization...
======================================================================
🔧 AUTOMATIC MOTOR PARAMETER OPTIMIZATION
======================================================================
Model: direct_hand_parm.xml

Step 1: Testing default parameters (kp=200, kv=40, forcelim=80)...
----------------------------------------------------------------------
  Tracking Error: 6.58° (max: 12.05°)
  Saturation: 74.0%
  Stability: 0.66

❌ Default parameters not optimal. Starting automatic search...
   Testing 210 parameter combinations...
   (kp: 6 values, kv: 5 values, forcelim: 7 values)

  Progress: 10/210 tests completed...
  Progress: 20/210 tests completed...
  Progress: 30/210 tests completed...
  Progress: 40/210 tests completed...
  Progress: 50/210 tests completed...
  Progress: 60/210 tests completed...
  Progress: 70/210 tests completed...
  Progress: 80/210 tests completed...
  Progress: 90/210 tests completed...
  Progress: 100/210 tests completed...
  Progress: 110/210 tests completed...
  Progress: 120/210 tests completed...
  Progress: 130/210 tests completed...
  Progress: 140/210 tests completed...
  Progress: 150/210 tests completed...
  Progress: 160/210 tests completed...
  Progress: 170/210 tests completed...
  Progress: 180/210 tests completed...
  Progress: 190/210 tests completed...
  Progress: 200/210 tests completed...
  Progress: 210/210 tests completed...

======================================================================
⚠️  NO WORKING PARAMETERS FOUND
======================================================================

Best attempt (still failing):
  kp=150, kv=40, forcelim=80
  Error: 5.05° (limit: 8.594366926962348°)
  Saturation: 74.0% (limit: 20.0%)

🔴 DIAGNOSIS: TRAJECTORY IS TOO AGGRESSIVE

The recorded motion is physically impossible for this robot:

Possible causes:
  1. ⚡ Motion is too fast
     - Joints accelerate/decelerate too quickly
     - Physics simulation cannot keep up

  2. 📏 Joint angles exceed safe limits
     - Motion tries to move joints beyond physical range
     - Check joint limits in MJCF/URDF

  3. ⚖️  Model mass/inertia is too high
     - Heavy links require more force to move
     - Reduce link mass in the model

  4. 🎯 Trajectory has sudden jumps
     - Not enough keyframes for smooth interpolation
     - Add more intermediate keyframes

Suggested solutions:
  ✓ Re-record motion at 50-70% speed
  ✓ Add more keyframes (smoother transitions)
  ✓ Check and adjust joint limits
  ✓ Reduce link mass/inertia in model editor
  ✓ Use simpler, slower movements for testing

⚠️  Parameter tuning CANNOT solve this issue.
    The trajectory itself must be modified.

======================================================================
