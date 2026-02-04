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
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 32, in <module>
    model = mujoco.MjModel.from_xml_path(MODEL_XML)
ValueError: Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix
Element name 'new_link_8', id 5, line 45
