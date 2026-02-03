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

Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/validate_motor_params.py", line 412, in <module>
    success = optimize_parameters()
  File "/home/younleakim/Downloads/direct_hand_parm/validate_motor_params.py", line 289, in optimize_parameters
    q_interp = np.interp(trajectory_times, kf_times, y_points)
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/numpy/lib/function_base.py", line 1599, in interp
    return interp_func(x, xp, fp, left, right)
ValueError: fp and xp are not of the same length.
