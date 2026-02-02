========================================
Enter choice [1]: 2
Starting Motor Validation...
Loading model: direct_hand_parm.xml
Loaded Recording 1768623534448. Mode: Motor Validation
Traceback (most recent call last):
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 324, in <module>
    on_joint_select('[GLOBAL]')
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 261, in on_joint_select
    update_info_text()
  File "/home/younleakim/Downloads/direct_hand_parm/replay_motor_validation.py", line 294, in update_info_text
    info_str += f"Time: {elapsed:.2f}s / {duration:.2f}s"
NameError: name 'elapsed' is not defined
