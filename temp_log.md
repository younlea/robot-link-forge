~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh
Creating virtual environment...
Installing dependencies (mujoco, matplotlib, numpy)...
========================================
  MuJoCo Motion Analysis Tool
========================================

Select Analysis Mode:

1. Joint Torque Visualization
   - Theoretical torque (inverse dynamics)

2. Motor Sizing Validation
   - Set motor parameters and validate

3. Fingertip Sensor Forces
   - Contact force visualization

========================================
Enter choice [1]: 1
Starting Analysis in mode: inverse...
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_with_torque.py", line 13, in <module>
    import mujoco
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/__init__.py", line 62, in <module>
    from mujoco.gl_context import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/gl_context.py", line 38, in <module>
    from mujoco.osmesa import GLContext as _GLContext
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/osmesa/__init__.py", line 31, in <module>
    from OpenGL import GL
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/GL/__init__.py", line 4, in <module>
    from OpenGL.GL.VERSION.GL_1_1 import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/GL/VERSION/GL_1_1.py", line 14, in <module>
    from OpenGL.raw.GL.VERSION.GL_1_1 import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/raw/GL/VERSION/GL_1_1.py", line 7, in <module>
    from OpenGL.raw.GL import _errors
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/raw/GL/_errors.py", line 4, in <module>
    _error_checker = _ErrorChecker( _p, _p.GL.glGetError )
AttributeError: 'NoneType' object has no attribute 'glGetError'
younleakim@younleakim-400TEA-400SEA:~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh
Installing dependencies (mujoco, matplotlib, numpy)...
========================================
  MuJoCo Motion Analysis Tool
========================================

Select Analysis Mode:

1. Joint Torque Visualization
   - Theoretical torque (inverse dynamics)

2. Motor Sizing Validation
   - Set motor parameters and validate

3. Fingertip Sensor Forces
   - Contact force visualization

========================================
Enter choice [1]: 2
Starting Motor Validation...
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 12, in <module>
    import mujoco
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/__init__.py", line 62, in <module>
    from mujoco.gl_context import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/gl_context.py", line 38, in <module>
    from mujoco.osmesa import GLContext as _GLContext
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/osmesa/__init__.py", line 31, in <module>
    from OpenGL import GL
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/GL/__init__.py", line 4, in <module>
    from OpenGL.GL.VERSION.GL_1_1 import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/GL/VERSION/GL_1_1.py", line 14, in <module>
    from OpenGL.raw.GL.VERSION.GL_1_1 import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/raw/GL/VERSION/GL_1_1.py", line 7, in <module>
    from OpenGL.raw.GL import _errors
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/raw/GL/_errors.py", line 4, in <module>
    _error_checker = _ErrorChecker( _p, _p.GL.glGetError )
AttributeError: 'NoneType' object has no attribute 'glGetError'
younleakim@younleakim-400TEA-400SEA:~/Downloads/direct_hand_parm$ ./run_torque_replay_0_recording_1768623534448.sh
Installing dependencies (mujoco, matplotlib, numpy)...
========================================
  MuJoCo Motion Analysis Tool
========================================

Select Analysis Mode:

1. Joint Torque Visualization
   - Theoretical torque (inverse dynamics)

2. Motor Sizing Validation
   - Set motor parameters and validate

3. Fingertip Sensor Forces
   - Contact force visualization

========================================
Enter choice [1]: 3
Starting Sensor Analysis in mode: sensors...
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_with_torque.py", line 13, in <module>
    import mujoco
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/__init__.py", line 62, in <module>
    from mujoco.gl_context import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/gl_context.py", line 38, in <module>
    from mujoco.osmesa import GLContext as _GLContext
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/mujoco/osmesa/__init__.py", line 31, in <module>
    from OpenGL import GL
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/GL/__init__.py", line 4, in <module>
    from OpenGL.GL.VERSION.GL_1_1 import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/GL/VERSION/GL_1_1.py", line 14, in <module>
    from OpenGL.raw.GL.VERSION.GL_1_1 import *
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/raw/GL/VERSION/GL_1_1.py", line 7, in <module>
    from OpenGL.raw.GL import _errors
  File "/home/younleakim/Downloads/direct_hand_parm/venv/lib/python3.10/site-packages/OpenGL/raw/GL/_errors.py", line 4, in <module>
    _error_checker = _ErrorChecker( _p, _p.GL.glGetError )
AttributeError: 'NoneType' object has no attribute 'glGetError'
