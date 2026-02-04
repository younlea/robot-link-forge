Processing STL with trimesh: /tmp/tmps09_blf9
  Faces: 184620
  Face count 184620 exceeds safety limit 150000. Decimating...
  Decimated to: 150000 faces
Processing STL with trimesh: /tmp/tmpe78f3brq
  Faces: 40378
Processing STL with trimesh: /tmp/tmp6tqh5a8s
  Faces: 46982
Processing STL with trimesh: /tmp/tmpc7a90e8k
  Faces: 15918
Processing STL with trimesh: /tmp/tmppsfqqizm
  Faces: 40378
Processing STL with trimesh: /tmp/tmpq1upmf6e
  Faces: 46982
Processing STL with trimesh: /tmp/tmpt4bn2kvc
  Faces: 15918
Processing STL with trimesh: /tmp/tmp_rzsc50b
  Faces: 39574
Processing STL with trimesh: /tmp/tmpdnzlefmm
  Faces: 47050
Processing STL with trimesh: /tmp/tmppic7mlnr
  Faces: 15918
Processing STL with trimesh: /tmp/tmp1aa1zlyu
  Faces: 40378
Processing STL with trimesh: /tmp/tmpu5is4cjd
  Faces: 46982
Processing STL with trimesh: /tmp/tmp9ildjng5
  Faces: 15918
Processing STL with trimesh: /tmp/tmp9fnvb_nd
  Faces: 40188
Processing STL with trimesh: /tmp/tmpcb4m9kbz
  Faces: 46982
Processing STL with trimesh: /tmp/tmp59szk6rq
  Faces: 15918
Processing STL with trimesh: /tmp/tmpe604w6bm
  Faces: 5096
Processing STL with trimesh: /tmp/tmp1b32djdw
  Faces: 5096
Processing STL with trimesh: /tmp/tmp4sj6jt9w
  Faces: 5096
Processing STL with trimesh: /tmp/tmpx7c1uvz6
  Faces: 5096
Processing STL with trimesh: /tmp/tmpdom0x5rd
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
✓ Robot base welded to fixed_world (no joint) - STABLE
Total bodies: 22, Total joints: 20

First 5 lines after <worldbody>:
      <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
      <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>
      <!-- Fixed world anchor - prevents robot from falling -->
      <body name="fixed_world" pos="0 0 0.5" mocap="false">
        <!-- Robot base attached here with no joints = welded -->
======================================================================

INFO:     127.0.0.1:46364 - "POST /api/export-mujoco-mjcf HTTP/1.1" 200 OK




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
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 32, in <module>
    model = mujoco.MjModel.from_xml_path(MODEL_XML)
ValueError: Error: mass and inertia of moving bodies must be larger than mjMINVAL
Element name 'little_finger-3rd-end', id 6, line 49


xml
<mujoco model="direct_hand_parm">
  <compiler angle="radian" meshdir="meshes" balanceinertia="true"/>
  <option timestep="0.001" iterations="100" solver="Newton" tolerance="1e-10" gravity="0 0 -9.81"/>
  <asset>
    <mesh name="direct_hand_parm_0" file="direct_hand_parm.stl" scale="0.01 0.01 0.01"/>
    <mesh name="new_link_5_1" file="new_link_5.stl" scale="0.01 0.01 0.01"/>
    <mesh name="new_link_8_2" file="new_link_8.stl" scale="0.01 0.01 0.01"/>
    <mesh name="little_finger-3rd-end_3" file="little_finger-3rd-end.stl" scale="0.01 0.01 0.01"/>
    <mesh name="new_link_6_4" file="new_link_6.stl" scale="0.01 0.01 0.01"/>
    <mesh name="new_link_9_5" file="new_link_9.stl" scale="0.01 0.01 0.01"/>
    <mesh name="ring_finger-3rd-end_6" file="ring_finger-3rd-end.stl" scale="0.01 0.01 0.01"/>
    <mesh name="new_link_7_7" file="new_link_7.stl" scale="0.01 0.01 0.01"/>
    <mesh name="new_link_10_8" file="new_link_10.stl" scale="0.01 0.01 0.01"/>
    <mesh name="middle_finger-3rd-end_9" file="middle_finger-3rd-end.stl" scale="0.01 0.01 0.01"/>
    <mesh name="index_finger-1st-pitch_10" file="index_finger-1st-pitch.stl" scale="0.01 0.01 0.01"/>
    <mesh name="index_finger-2st-pitch_11" file="index_finger-2st-pitch.stl" scale="0.01 0.01 0.01"/>
    <mesh name="index_finger-3rd-end_12" file="index_finger-3rd-end.stl" scale="0.01 0.01 0.01"/>
    <mesh name="thumb-link_13" file="thumb-link.stl" scale="0.01 0.01 0.01"/>
    <mesh name="new_link_11_14" file="new_link_11.stl" scale="0.01 0.01 0.01"/>
    <mesh name="thumb-3rd-end_15" file="thumb-3rd-end.stl" scale="0.01 0.01 0.01"/>
    <mesh name="little_finger-1st_roll_join_16" file="little_finger-1st_roll_join.stl" scale="0.01 0.01 0.01"/>
    <mesh name="ring_finger-1st_roll_join_17" file="ring_finger-1st_roll_join.stl" scale="0.01 0.01 0.01"/>
    <mesh name="middle_finger_1st_roll_join_18" file="middle_finger_1st_roll_join.stl" scale="0.01 0.01 0.01"/>
    <mesh name="index_finger_1st_roll_join_19" file="index_finger_1st_roll_join.stl" scale="0.01 0.01 0.01"/>
    <mesh name="thumb_1st_yaw_join_20" file="thumb_1st_yaw_join.stl" scale="0.01 0.01 0.01"/>
    <material name="gray" rgba="0.5 0.5 0.5 1"/>
    <material name="collision" rgba="1 0 0 0.5"/>
  </asset>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>
    <!-- Fixed world anchor - prevents robot from falling -->
    <body name="fixed_world" pos="0 0 0.5" mocap="false">
      <!-- Robot base attached here with no joints = welded -->
      <body name="direct_hand_parm" pos="0 0 0" euler="0 0 0">
        <geom type="mesh" mesh="direct_hand_parm_0" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="1.5707963267948966 -3.141592653589793 1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
        <body name="new_link" pos="0.16 0.369 0.843" euler="-0.17453292519943295 0.0 0.0">
          <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001" />
          <joint name="LittleFinger-1st_roll" type="hinge" axis="1 0 0" range="-0.17453292519943295 0.17453292519943295" damping="0.5" armature="0.001" />
          <geom type="mesh" mesh="little_finger-1st_roll_join_16" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" group="1" contype="0" conaffinity="0" />
          <body name="new_link_5" pos="0.0 0.0 0.0" euler="0.0 0.0 0.0">
            <inertial pos="0.019521 0.002047 -0.000167" mass="0.008566" diaginertia="0.000001 0.000002 0.000002" />
            <joint name="LittleFinger-1st-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
            <geom type="mesh" mesh="new_link_5_1" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
            <body name="new_link_8" pos="0.035 0.0 0.41" euler="0.0 0.0 0.0">
              <inertial pos="0.013176 0.000843 -0.000035" mass="0.004325" diaginertia="0.000000 0.000001 0.000000" />
              <joint name="LittleFinger-2nd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
              <geom type="mesh" mesh="new_link_8_2" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
              <body name="little_finger-3rd-end" pos="0.008 0.0 0.209" euler="0.0 0.0 0.0">
                <inertial pos="0.013969 0.000611 -0.000000" mass="0.003689" diaginertia="0.000000 0.000000 0.000000" />
                <joint name="LittleFinger-3rd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
                <geom type="mesh" mesh="little_finger-3rd-end_3" group="1" contype="1" conaffinity="1" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
                <site name="little_finger-3rd-end_sensor_0_1" pos="0.03   0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_0_2" pos="0.03   0      0.28 " size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_0_3" pos="0.03   -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_1_1" pos="0      0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_1_2" pos="0      0      0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_1_3" pos="0      -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_2_1" pos="-0.03  0.03   0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_2_2" pos="-0.03  0      0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_2_3" pos="-0.03  -0.03  0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_3_1" pos="-0.045 0.03   0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_3_2" pos="-0.045 0      0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_3_3" pos="-0.045 -0.03  0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_4_1" pos="-0.05  0.03   0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_4_2" pos="-0.05  0      0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_4_3" pos="-0.05  -0.03  0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_5_1" pos="-0.06  0.03   0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_5_2" pos="-0.06  0      0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_5_3" pos="-0.06  -0.03  0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_6_1" pos="-0.065 0.03   0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_6_2" pos="-0.065 0      0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="little_finger-3rd-end_sensor_6_3" pos="-0.065 -0.03  0.14" size="0.01" rgba="1 0 0 1"/>
              </body>
            </body>
          </body>
        </body>
        <body name="new_link_1" pos="0.16 0.14800000000000002 0.9229999999999999" euler="-0.08726646259971647 0.0 0.0">
          <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001" />
          <joint name="RingFinger-1st_roll" type="hinge" axis="1 0 0" range="-0.17453292519943295 0.17453292519943295" damping="0.5" armature="0.001" />
          <geom type="mesh" mesh="ring_finger-1st_roll_join_17" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" group="1" contype="0" conaffinity="0" />
          <body name="new_link_6" pos="0.0 0.0 0.0" euler="0.0 0.0 0.0">
            <inertial pos="0.019527 0.002202 -0.000167" mass="0.008566" diaginertia="0.000001 0.000002 0.000002" />
            <joint name="RingFinger-1st-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
            <geom type="mesh" mesh="new_link_6_4" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
            <body name="new_link_9" pos="0.035 0.0 0.41" euler="0.0 0.0 0.0">
              <inertial pos="0.013176 0.000845 -0.000035" mass="0.004325" diaginertia="0.000000 0.000001 0.000000" />
              <joint name="RingFinger-2nd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
              <geom type="mesh" mesh="new_link_9_5" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
              <body name="ring_finger-3rd-end" pos="0.008 0.0 0.24" euler="0.0 0.0 0.0">
                <inertial pos="0.013969 0.000613 -0.000000" mass="0.003689" diaginertia="0.000000 0.000000 0.000000" />
                <joint name="RingFinger-3rd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
                <geom type="mesh" mesh="ring_finger-3rd-end_6" group="1" contype="1" conaffinity="1" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
                <site name="ring_finger-3rd-end_sensor_0_1" pos="0.03   0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_0_2" pos="0.03   0      0.28 " size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_0_3" pos="0.03   -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_1_1" pos="0      0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_1_2" pos="0      0      0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_1_3" pos="0      -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_2_1" pos="-0.03  0.03   0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_2_2" pos="-0.03  0      0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_2_3" pos="-0.03  -0.03  0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_3_1" pos="-0.045 0.03   0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_3_2" pos="-0.045 0      0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_3_3" pos="-0.045 -0.03  0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_4_1" pos="-0.05  0.03   0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_4_2" pos="-0.05  0      0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_4_3" pos="-0.05  -0.03  0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_5_1" pos="-0.06  0.03   0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_5_2" pos="-0.06  0      0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_5_3" pos="-0.06  -0.03  0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_6_1" pos="-0.065 0.03   0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_6_2" pos="-0.065 0      0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="ring_finger-3rd-end_sensor_6_3" pos="-0.065 -0.03  0.14" size="0.01" rgba="1 0 0 1"/>
              </body>
            </body>
          </body>
        </body>
        <body name="new_link_2" pos="0.16 -0.08 0.955" euler="0.0 0.0 0.0">
          <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001" />
          <joint name="MiddleFinger_1st_roll" type="hinge" axis="1 0 0" range="-0.17453292519943295 0.17453292519943295" damping="0.5" armature="0.001" />
          <geom type="mesh" mesh="middle_finger_1st_roll_join_18" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" group="1" contype="0" conaffinity="0" />
          <body name="new_link_7" pos="0.0 0.0 0.0" euler="0.0 0.0 0.0">
            <inertial pos="0.021197 0.002243 -0.000200" mass="0.009018" diaginertia="0.000001 0.000003 0.000003" />
            <joint name="MiddleFinger-1st-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
            <geom type="mesh" mesh="new_link_7_7" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
            <body name="new_link_10" pos="0.035 0.0 0.44" euler="0.0 0.0 0.0">
              <inertial pos="0.014628 0.000752 -0.000034" mass="0.004985" diaginertia="0.000000 0.000001 0.000001" />
              <joint name="MiddleFinger-2nd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
              <geom type="mesh" mesh="new_link_10_8" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
              <body name="middle_finger-3rd-end" pos="0.008 0.0 0.27" euler="0.0 0.0 0.0">
                <inertial pos="0.013966 0.000607 -0.000006" mass="0.003689" diaginertia="0.000000 0.000000 0.000000" />
                <joint name="MiddleFinger-3rd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 -0.03490658503988659" damping="0.5" armature="0.001" />
                <geom type="mesh" mesh="middle_finger-3rd-end_9" group="1" contype="1" conaffinity="1" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
                <site name="middle_finger-3rd-end_sensor_0_1" pos="0.03   0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_0_2" pos="0.03   0      0.28 " size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_0_3" pos="0.03   -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_1_1" pos="0      0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_1_2" pos="0      0      0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_1_3" pos="0      -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_2_1" pos="-0.03  0.03   0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_2_2" pos="-0.03  0      0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_2_3" pos="-0.03  -0.03  0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_3_1" pos="-0.045 0.03   0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_3_2" pos="-0.045 0      0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_3_3" pos="-0.045 -0.03  0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_4_1" pos="-0.05  0.03   0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_4_2" pos="-0.05  0      0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_4_3" pos="-0.05  -0.03  0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_5_1" pos="-0.06  0.03   0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_5_2" pos="-0.06  0      0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_5_3" pos="-0.06  -0.03  0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_6_1" pos="-0.065 0.03   0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_6_2" pos="-0.065 0      0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="middle_finger-3rd-end_sensor_6_3" pos="-0.065 -0.03  0.14" size="0.01" rgba="1 0 0 1"/>
              </body>
            </body>
          </body>
        </body>
        <body name="new_link_3" pos="0.16 -0.31 0.9229999999999999" euler="0.08726646259971647 0.0 0.0">
          <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001" />
          <joint name="IndexFinger_1st_roll" type="hinge" axis="1 0 0" range="-0.17453292519943295 0.17453292519943295" damping="0.5" armature="0.001" />
          <geom type="mesh" mesh="index_finger_1st_roll_join_19" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" group="1" contype="0" conaffinity="0" />
          <body name="index_finger-1st-pitch" pos="0.0 0.0 0.0" euler="0.0 0.0 0.0">
            <inertial pos="0.019527 0.002202 -0.000167" mass="0.008566" diaginertia="0.000001 0.000002 0.000002" />
            <joint name="IndexFinger-1st-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
            <geom type="mesh" mesh="index_finger-1st-pitch_10" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
            <body name="index_finger-2st-pitch" pos="0.035 0.0 0.41" euler="0.0 0.0 0.0">
              <inertial pos="0.013176 0.000843 -0.000035" mass="0.004325" diaginertia="0.000000 0.000001 0.000000" />
              <joint name="IndexFinger-2nd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 -0.05235987755982988" damping="0.5" armature="0.001" />
              <geom type="mesh" mesh="index_finger-2st-pitch_11" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
              <body name="index_finger-3rd-end" pos="0.008 0.0 0.24" euler="0.0 0.0 0.0">
                <inertial pos="0.013969 0.000611 -0.000000" mass="0.003689" diaginertia="0.000000 0.000000 0.000000" />
                <joint name="IndexFinger-3rd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
                <geom type="mesh" mesh="index_finger-3rd-end_12" group="1" contype="1" conaffinity="1" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 0.0 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
                <site name="index_finger-3rd-end_sensor_0_1" pos="0.03   0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_0_2" pos="0.03   0      0.28 " size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_0_3" pos="0.03   -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_1_1" pos="0      0.03   0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_1_2" pos="0      0      0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_1_3" pos="0      -0.03  0.275" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_2_1" pos="-0.03  0.03   0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_2_2" pos="-0.03  0      0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_2_3" pos="-0.03  -0.03  0.26" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_3_1" pos="-0.045 0.03   0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_3_2" pos="-0.045 0      0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_3_3" pos="-0.045 -0.03  0.23" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_4_1" pos="-0.05  0.03   0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_4_2" pos="-0.05  0      0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_4_3" pos="-0.05  -0.03  0.2" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_5_1" pos="-0.06  0.03   0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_5_2" pos="-0.06  0      0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_5_3" pos="-0.06  -0.03  0.17" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_6_1" pos="-0.065 0.03   0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_6_2" pos="-0.065 0      0.14" size="0.01" rgba="1 0 0 1"/>
                <site name="index_finger-3rd-end_sensor_6_3" pos="-0.065 -0.03  0.14" size="0.01" rgba="1 0 0 1"/>
              </body>
            </body>
          </body>
        </body>
        <body name="new_link_4" pos="-0.07 -0.317 0.267" euler="0.0 0.0 3.141592653589793">
          <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001" />
          <joint name="thumb_1st_yaw" type="hinge" axis="0 0 1" range="-0.5235987755982988 1.5707963267948966" damping="0.5" armature="0.001" />
          <geom type="mesh" mesh="thumb_1st_yaw_join_20" pos="0.0 0.0 0.0" euler="1.5707963267948966 0.0 1.5707963267948966" rgba="0.941 0.941 0.941 1.0" group="1" contype="0" conaffinity="0" />
          <body name="thumb-link" pos="0.0 0.0 0.0" euler="0.0 0.0 0.0">
            <inertial pos="-0.001958 -0.021758 -0.001149" mass="0.009483" diaginertia="0.000003 0.000001 0.000002" />
            <joint name="Thumb-1st-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
            <geom type="mesh" mesh="thumb-link_13" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="-1.5707963267948966 -3.141592653589793 -1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
            <body name="new_link_11" pos="0.44 0.0 -0.035" euler="0.0 0.0 0.0">
              <inertial pos="-0.000843 -0.013176 -0.000015" mass="0.004325" diaginertia="0.000001 0.000000 0.000000" />
              <joint name="Thumb-2nd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
              <geom type="mesh" mesh="new_link_11_14" group="1" contype="0" conaffinity="0" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="1.5707963267948966 0.0 1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
              <body name="thumb-3rd-end" pos="0.24 0.0 -0.008" euler="0.0 0.0 0.0">
                <inertial pos="-0.000855 -0.013956 0.000000" mass="0.003689" diaginertia="0.000000 0.000000 0.000000" />
                <joint name="Thumb-3rd-pitch" type="hinge" axis="0 1 0" range="-1.5707963267948966 0.0" damping="0.5" armature="0.001" />
                <geom type="mesh" mesh="thumb-3rd-end_15" group="1" contype="1" conaffinity="1" condim="3" margin="0.002" pos="0.0 0.0 0.0" euler="1.5707963267948966 0.0 1.5707963267948966" rgba="0.941 0.941 0.941 1.0" />
                <site name="thumb-3rd-end_sensor_0_1" pos="0.275 0.03   -0.03" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_0_2" pos="0.28     0   -0.03" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_0_3" pos="0.275 -0.03  -0.03" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_1_1" pos="0.275 0.03   0" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_1_2" pos="0.275 0      0" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_1_3" pos="0.275 -0.03  0" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_2_1" pos="0.26 0.03   0.03" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_2_2" pos="0.26 0      0.03" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_2_3" pos="0.26 -0.03  0.03" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_3_1" pos="0.23 0.03   0.045" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_3_2" pos="0.23 0      0.045" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_3_3" pos="0.23 -0.03   0.045" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_4_1" pos="0.2 0.03   0.05" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_4_2" pos="0.2 0      0.05" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_4_3" pos="0.2 -0.03  0.05" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_5_1" pos="0.17 0.03  0.06" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_5_2" pos="0.17 0     0.06" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_5_3" pos="0.17 -0.03 0.06" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_6_1" pos="0.14 0.03  0.065" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_6_2" pos="0.14 0     0.065" size="0.01" rgba="1 0 0 1"/>
                <site name="thumb-3rd-end_sensor_6_3" pos="0.14 -0.03 0.065" size="0.01" rgba="1 0 0 1"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>  <!-- End fixed_world -->
  </worldbody>
  <actuator>
            <position name="LittleFinger-1st_roll_act" joint="LittleFinger-1st_roll" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-0.17453292519943295 0.17453292519943295"/>
              <position name="LittleFinger-1st-pitch_act" joint="LittleFinger-1st-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                <position name="LittleFinger-2nd-pitch_act" joint="LittleFinger-2nd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                  <position name="LittleFinger-3rd-pitch_act" joint="LittleFinger-3rd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
            <position name="RingFinger-1st_roll_act" joint="RingFinger-1st_roll" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-0.17453292519943295 0.17453292519943295"/>
              <position name="RingFinger-1st-pitch_act" joint="RingFinger-1st-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                <position name="RingFinger-2nd-pitch_act" joint="RingFinger-2nd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                  <position name="RingFinger-3rd-pitch_act" joint="RingFinger-3rd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
            <position name="MiddleFinger_1st_roll_act" joint="MiddleFinger_1st_roll" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-0.17453292519943295 0.17453292519943295"/>
              <position name="MiddleFinger-1st-pitch_act" joint="MiddleFinger-1st-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                <position name="MiddleFinger-2nd-pitch_act" joint="MiddleFinger-2nd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                  <position name="MiddleFinger-3rd-pitch_act" joint="MiddleFinger-3rd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 -0.03490658503988659"/>
            <position name="IndexFinger_1st_roll_act" joint="IndexFinger_1st_roll" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-0.17453292519943295 0.17453292519943295"/>
              <position name="IndexFinger-1st-pitch_act" joint="IndexFinger-1st-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                <position name="IndexFinger-2nd-pitch_act" joint="IndexFinger-2nd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 -0.05235987755982988"/>
                  <position name="IndexFinger-3rd-pitch_act" joint="IndexFinger-3rd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
            <position name="thumb_1st_yaw_act" joint="thumb_1st_yaw" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-0.5235987755982988 1.5707963267948966"/>
              <position name="Thumb-1st-pitch_act" joint="Thumb-1st-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                <position name="Thumb-2nd-pitch_act" joint="Thumb-2nd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
                  <position name="Thumb-3rd-pitch_act" joint="Thumb-3rd-pitch" kp="200" kv="20" gear="50" forcelimited="true" forcerange="-300 300" ctrlrange="-1.5707963267948966 0.0"/>
  </actuator>
  <sensor>
    <touch name="sensor_little_finger-3rd-end_sensor_0_1" site="little_finger-3rd-end_sensor_0_1" />
    <touch name="sensor_little_finger-3rd-end_sensor_0_2" site="little_finger-3rd-end_sensor_0_2" />
    <touch name="sensor_little_finger-3rd-end_sensor_0_3" site="little_finger-3rd-end_sensor_0_3" />
    <touch name="sensor_little_finger-3rd-end_sensor_1_1" site="little_finger-3rd-end_sensor_1_1" />
    <touch name="sensor_little_finger-3rd-end_sensor_1_2" site="little_finger-3rd-end_sensor_1_2" />
    <touch name="sensor_little_finger-3rd-end_sensor_1_3" site="little_finger-3rd-end_sensor_1_3" />
    <touch name="sensor_little_finger-3rd-end_sensor_2_1" site="little_finger-3rd-end_sensor_2_1" />
    <touch name="sensor_little_finger-3rd-end_sensor_2_2" site="little_finger-3rd-end_sensor_2_2" />
    <touch name="sensor_little_finger-3rd-end_sensor_2_3" site="little_finger-3rd-end_sensor_2_3" />
    <touch name="sensor_little_finger-3rd-end_sensor_3_1" site="little_finger-3rd-end_sensor_3_1" />
    <touch name="sensor_little_finger-3rd-end_sensor_3_2" site="little_finger-3rd-end_sensor_3_2" />
    <touch name="sensor_little_finger-3rd-end_sensor_3_3" site="little_finger-3rd-end_sensor_3_3" />
    <touch name="sensor_little_finger-3rd-end_sensor_4_1" site="little_finger-3rd-end_sensor_4_1" />
    <touch name="sensor_little_finger-3rd-end_sensor_4_2" site="little_finger-3rd-end_sensor_4_2" />
    <touch name="sensor_little_finger-3rd-end_sensor_4_3" site="little_finger-3rd-end_sensor_4_3" />
    <touch name="sensor_little_finger-3rd-end_sensor_5_1" site="little_finger-3rd-end_sensor_5_1" />
    <touch name="sensor_little_finger-3rd-end_sensor_5_2" site="little_finger-3rd-end_sensor_5_2" />
    <touch name="sensor_little_finger-3rd-end_sensor_5_3" site="little_finger-3rd-end_sensor_5_3" />
    <touch name="sensor_little_finger-3rd-end_sensor_6_1" site="little_finger-3rd-end_sensor_6_1" />
    <touch name="sensor_little_finger-3rd-end_sensor_6_2" site="little_finger-3rd-end_sensor_6_2" />
    <touch name="sensor_little_finger-3rd-end_sensor_6_3" site="little_finger-3rd-end_sensor_6_3" />
    <touch name="sensor_ring_finger-3rd-end_sensor_0_1" site="ring_finger-3rd-end_sensor_0_1" />
    <touch name="sensor_ring_finger-3rd-end_sensor_0_2" site="ring_finger-3rd-end_sensor_0_2" />
    <touch name="sensor_ring_finger-3rd-end_sensor_0_3" site="ring_finger-3rd-end_sensor_0_3" />
    <touch name="sensor_ring_finger-3rd-end_sensor_1_1" site="ring_finger-3rd-end_sensor_1_1" />
    <touch name="sensor_ring_finger-3rd-end_sensor_1_2" site="ring_finger-3rd-end_sensor_1_2" />
    <touch name="sensor_ring_finger-3rd-end_sensor_1_3" site="ring_finger-3rd-end_sensor_1_3" />
    <touch name="sensor_ring_finger-3rd-end_sensor_2_1" site="ring_finger-3rd-end_sensor_2_1" />
    <touch name="sensor_ring_finger-3rd-end_sensor_2_2" site="ring_finger-3rd-end_sensor_2_2" />
    <touch name="sensor_ring_finger-3rd-end_sensor_2_3" site="ring_finger-3rd-end_sensor_2_3" />
    <touch name="sensor_ring_finger-3rd-end_sensor_3_1" site="ring_finger-3rd-end_sensor_3_1" />
    <touch name="sensor_ring_finger-3rd-end_sensor_3_2" site="ring_finger-3rd-end_sensor_3_2" />
    <touch name="sensor_ring_finger-3rd-end_sensor_3_3" site="ring_finger-3rd-end_sensor_3_3" />
    <touch name="sensor_ring_finger-3rd-end_sensor_4_1" site="ring_finger-3rd-end_sensor_4_1" />
    <touch name="sensor_ring_finger-3rd-end_sensor_4_2" site="ring_finger-3rd-end_sensor_4_2" />
    <touch name="sensor_ring_finger-3rd-end_sensor_4_3" site="ring_finger-3rd-end_sensor_4_3" />
    <touch name="sensor_ring_finger-3rd-end_sensor_5_1" site="ring_finger-3rd-end_sensor_5_1" />
    <touch name="sensor_ring_finger-3rd-end_sensor_5_2" site="ring_finger-3rd-end_sensor_5_2" />
    <touch name="sensor_ring_finger-3rd-end_sensor_5_3" site="ring_finger-3rd-end_sensor_5_3" />
    <touch name="sensor_ring_finger-3rd-end_sensor_6_1" site="ring_finger-3rd-end_sensor_6_1" />
    <touch name="sensor_ring_finger-3rd-end_sensor_6_2" site="ring_finger-3rd-end_sensor_6_2" />
    <touch name="sensor_ring_finger-3rd-end_sensor_6_3" site="ring_finger-3rd-end_sensor_6_3" />
    <touch name="sensor_middle_finger-3rd-end_sensor_0_1" site="middle_finger-3rd-end_sensor_0_1" />
    <touch name="sensor_middle_finger-3rd-end_sensor_0_2" site="middle_finger-3rd-end_sensor_0_2" />
    <touch name="sensor_middle_finger-3rd-end_sensor_0_3" site="middle_finger-3rd-end_sensor_0_3" />
    <touch name="sensor_middle_finger-3rd-end_sensor_1_1" site="middle_finger-3rd-end_sensor_1_1" />
    <touch name="sensor_middle_finger-3rd-end_sensor_1_2" site="middle_finger-3rd-end_sensor_1_2" />
    <touch name="sensor_middle_finger-3rd-end_sensor_1_3" site="middle_finger-3rd-end_sensor_1_3" />
    <touch name="sensor_middle_finger-3rd-end_sensor_2_1" site="middle_finger-3rd-end_sensor_2_1" />
    <touch name="sensor_middle_finger-3rd-end_sensor_2_2" site="middle_finger-3rd-end_sensor_2_2" />
    <touch name="sensor_middle_finger-3rd-end_sensor_2_3" site="middle_finger-3rd-end_sensor_2_3" />
    <touch name="sensor_middle_finger-3rd-end_sensor_3_1" site="middle_finger-3rd-end_sensor_3_1" />
    <touch name="sensor_middle_finger-3rd-end_sensor_3_2" site="middle_finger-3rd-end_sensor_3_2" />
    <touch name="sensor_middle_finger-3rd-end_sensor_3_3" site="middle_finger-3rd-end_sensor_3_3" />
    <touch name="sensor_middle_finger-3rd-end_sensor_4_1" site="middle_finger-3rd-end_sensor_4_1" />
    <touch name="sensor_middle_finger-3rd-end_sensor_4_2" site="middle_finger-3rd-end_sensor_4_2" />
    <touch name="sensor_middle_finger-3rd-end_sensor_4_3" site="middle_finger-3rd-end_sensor_4_3" />
    <touch name="sensor_middle_finger-3rd-end_sensor_5_1" site="middle_finger-3rd-end_sensor_5_1" />
    <touch name="sensor_middle_finger-3rd-end_sensor_5_2" site="middle_finger-3rd-end_sensor_5_2" />
    <touch name="sensor_middle_finger-3rd-end_sensor_5_3" site="middle_finger-3rd-end_sensor_5_3" />
    <touch name="sensor_middle_finger-3rd-end_sensor_6_1" site="middle_finger-3rd-end_sensor_6_1" />
    <touch name="sensor_middle_finger-3rd-end_sensor_6_2" site="middle_finger-3rd-end_sensor_6_2" />
    <touch name="sensor_middle_finger-3rd-end_sensor_6_3" site="middle_finger-3rd-end_sensor_6_3" />
    <touch name="sensor_index_finger-3rd-end_sensor_0_1" site="index_finger-3rd-end_sensor_0_1" />
    <touch name="sensor_index_finger-3rd-end_sensor_0_2" site="index_finger-3rd-end_sensor_0_2" />
    <touch name="sensor_index_finger-3rd-end_sensor_0_3" site="index_finger-3rd-end_sensor_0_3" />
    <touch name="sensor_index_finger-3rd-end_sensor_1_1" site="index_finger-3rd-end_sensor_1_1" />
    <touch name="sensor_index_finger-3rd-end_sensor_1_2" site="index_finger-3rd-end_sensor_1_2" />
    <touch name="sensor_index_finger-3rd-end_sensor_1_3" site="index_finger-3rd-end_sensor_1_3" />
    <touch name="sensor_index_finger-3rd-end_sensor_2_1" site="index_finger-3rd-end_sensor_2_1" />
    <touch name="sensor_index_finger-3rd-end_sensor_2_2" site="index_finger-3rd-end_sensor_2_2" />
    <touch name="sensor_index_finger-3rd-end_sensor_2_3" site="index_finger-3rd-end_sensor_2_3" />
    <touch name="sensor_index_finger-3rd-end_sensor_3_1" site="index_finger-3rd-end_sensor_3_1" />
    <touch name="sensor_index_finger-3rd-end_sensor_3_2" site="index_finger-3rd-end_sensor_3_2" />
    <touch name="sensor_index_finger-3rd-end_sensor_3_3" site="index_finger-3rd-end_sensor_3_3" />
    <touch name="sensor_index_finger-3rd-end_sensor_4_1" site="index_finger-3rd-end_sensor_4_1" />
    <touch name="sensor_index_finger-3rd-end_sensor_4_2" site="index_finger-3rd-end_sensor_4_2" />
    <touch name="sensor_index_finger-3rd-end_sensor_4_3" site="index_finger-3rd-end_sensor_4_3" />
    <touch name="sensor_index_finger-3rd-end_sensor_5_1" site="index_finger-3rd-end_sensor_5_1" />
    <touch name="sensor_index_finger-3rd-end_sensor_5_2" site="index_finger-3rd-end_sensor_5_2" />
    <touch name="sensor_index_finger-3rd-end_sensor_5_3" site="index_finger-3rd-end_sensor_5_3" />
    <touch name="sensor_index_finger-3rd-end_sensor_6_1" site="index_finger-3rd-end_sensor_6_1" />
    <touch name="sensor_index_finger-3rd-end_sensor_6_2" site="index_finger-3rd-end_sensor_6_2" />
    <touch name="sensor_index_finger-3rd-end_sensor_6_3" site="index_finger-3rd-end_sensor_6_3" />
    <touch name="sensor_thumb-3rd-end_sensor_0_1" site="thumb-3rd-end_sensor_0_1" />
    <touch name="sensor_thumb-3rd-end_sensor_0_2" site="thumb-3rd-end_sensor_0_2" />
    <touch name="sensor_thumb-3rd-end_sensor_0_3" site="thumb-3rd-end_sensor_0_3" />
    <touch name="sensor_thumb-3rd-end_sensor_1_1" site="thumb-3rd-end_sensor_1_1" />
    <touch name="sensor_thumb-3rd-end_sensor_1_2" site="thumb-3rd-end_sensor_1_2" />
    <touch name="sensor_thumb-3rd-end_sensor_1_3" site="thumb-3rd-end_sensor_1_3" />
    <touch name="sensor_thumb-3rd-end_sensor_2_1" site="thumb-3rd-end_sensor_2_1" />
    <touch name="sensor_thumb-3rd-end_sensor_2_2" site="thumb-3rd-end_sensor_2_2" />
    <touch name="sensor_thumb-3rd-end_sensor_2_3" site="thumb-3rd-end_sensor_2_3" />
    <touch name="sensor_thumb-3rd-end_sensor_3_1" site="thumb-3rd-end_sensor_3_1" />
    <touch name="sensor_thumb-3rd-end_sensor_3_2" site="thumb-3rd-end_sensor_3_2" />
    <touch name="sensor_thumb-3rd-end_sensor_3_3" site="thumb-3rd-end_sensor_3_3" />
    <touch name="sensor_thumb-3rd-end_sensor_4_1" site="thumb-3rd-end_sensor_4_1" />
    <touch name="sensor_thumb-3rd-end_sensor_4_2" site="thumb-3rd-end_sensor_4_2" />
    <touch name="sensor_thumb-3rd-end_sensor_4_3" site="thumb-3rd-end_sensor_4_3" />
    <touch name="sensor_thumb-3rd-end_sensor_5_1" site="thumb-3rd-end_sensor_5_1" />
    <touch name="sensor_thumb-3rd-end_sensor_5_2" site="thumb-3rd-end_sensor_5_2" />
    <touch name="sensor_thumb-3rd-end_sensor_5_3" site="thumb-3rd-end_sensor_5_3" />
    <touch name="sensor_thumb-3rd-end_sensor_6_1" site="thumb-3rd-end_sensor_6_1" />
    <touch name="sensor_thumb-3rd-end_sensor_6_2" site="thumb-3rd-end_sensor_6_2" />
    <touch name="sensor_thumb-3rd-end_sensor_6_3" site="thumb-3rd-end_sensor_6_3" />
  </sensor>
</mujoco>
