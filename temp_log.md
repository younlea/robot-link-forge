export할때 로그 
Processing STL with trimesh: /tmp/tmp2moj8b7p
  Faces: 184620
  Face count 184620 exceeds safety limit 150000. Decimating...
  Decimated to: 150000 faces
Processing STL with trimesh: /tmp/tmp0sjla0eb
  Faces: 40378
Processing STL with trimesh: /tmp/tmplghohlfh
  Faces: 46982
Processing STL with trimesh: /tmp/tmpljn7w2zc
  Faces: 15918
Processing STL with trimesh: /tmp/tmp_s2xze78
  Faces: 40378
Processing STL with trimesh: /tmp/tmpbgnpmcgr
  Faces: 46982
Processing STL with trimesh: /tmp/tmpctpi0o30
  Faces: 15918
Processing STL with trimesh: /tmp/tmpnrnoaqde
  Faces: 39574
Processing STL with trimesh: /tmp/tmpy7v0rp_a
  Faces: 47050
Processing STL with trimesh: /tmp/tmpu3e7dlag
  Faces: 15918
Processing STL with trimesh: /tmp/tmpw8eufnbi
  Faces: 40378
Processing STL with trimesh: /tmp/tmphnc2tcu5
  Faces: 46982
Processing STL with trimesh: /tmp/tmp0qnattak
  Faces: 15918
Processing STL with trimesh: /tmp/tmplhc7w7xy
  Faces: 40188
Processing STL with trimesh: /tmp/tmph8kuug7i
  Faces: 46982
Processing STL with trimesh: /tmp/tmp8cpomlnk
  Faces: 15918
Processing STL with trimesh: /tmp/tmpqorfced4
  Faces: 5096
Processing STL with trimesh: /tmp/tmpp5ilscyu
  Faces: 5096
Processing STL with trimesh: /tmp/tmpgm4ne7w0
  Faces: 5096
Processing STL with trimesh: /tmp/tmpo8ybzd8o
  Faces: 5096
Processing STL with trimesh: /tmp/tmpbv342j1n
  Faces: 5584
[DEBUG_MJCF] Body: little_finger-3rd-end | Leaf: True | Direct: True | Standard: False
[DEBUG_MJCF]   -> Config FOUND for Little
[DEBUG_MJCF] Body: ring_finger-3rd-end | Leaf: True | Direct: True | Standard: False
[DEBUG_MJCF]   -> Config FOUND for Ring
[DEBUG_MJCF] Body: middle_finger-3rd-end | Leaf: True | Direct: True | Standard: False
[DEBUG_MJCF]   -> Config FOUND for Middle
[DEBUG_MJCF] Body: index_finger-3rd-end | Leaf: True | Direct: True | Standard: False
[DEBUG_MJCF]   -> Config FOUND for index
[DEBUG_MJCF] Body: thumb-link | Leaf: False | Direct: False | Standard: False
[DEBUG_MJCF] Body: thumb-3rd-end | Leaf: True | Direct: True | Standard: False
[DEBUG_MJCF]   -> Config FOUND for thumb

======================================================================
MJCF EXPORT VALIDATION
======================================================================
✓ fixed_world anchor found
✗ WARNING: Joint found inside fixed_world - robot NOT fixed!
Total bodies: 22, Total joints: 20

First 5 lines after <worldbody>:
      <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
      <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>
      <!-- Fixed world anchor - prevents robot from falling -->
      <body name="fixed_world" pos="0 0 0.5" mocap="false">
        <!-- Robot base attached here with no joints = welded -->
======================================================================

INFO:     127.0.0.1:52798 - "POST /api/export-mujoco-mjcf HTTP/1.1" 200 OK




실행시 로그 
./run_torque_replay_0_recording_1768623534448.sh 
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
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 32, in <module>
    model = mujoco.MjModel.from_xml_path(MODEL_XML)
ValueError: Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix
Element name 'new_link_8', id 5, line 45
