Step 6833000/6835001: Max torque so far = 38.40 Nm
  Step 6833500/6835001: Max torque so far = 38.40 Nm
  Step 6834000/6835001: Max torque so far = 38.40 Nm
  Step 6834500/6835001: Max torque so far = 38.40 Nm
  Step 6835000/6835001: Max torque so far = 38.40 Nm

Inverse Dynamics Results:
  Joint Name                    | Max Torque (Nm)
  ------------------------------------------------------------
  LittleFinger-1st_roll          |     0.01
  LittleFinger-1st-pitch         |     0.01
  LittleFinger-2nd-pitch         |     0.00
  LittleFinger-3rd-pitch         |     0.00
  RingFinger-1st_roll            |     0.00
  RingFinger-1st-pitch           |     0.01
  RingFinger-2nd-pitch           |     0.00
  RingFinger-3rd-pitch           |     0.00
  MiddleFinger_1st_roll          |     0.57
  MiddleFinger-1st-pitch         |    31.93
  MiddleFinger-2nd-pitch         |    15.32
  MiddleFinger-3rd-pitch         |     3.84
  IndexFinger_1st_roll           |     0.28
  IndexFinger-1st-pitch          |    38.40
  IndexFinger-2nd-pitch          |    15.61
  IndexFinger-3rd-pitch          |     7.31
  thumb_1st_yaw                  |     2.26
  Thumb-1st-pitch                |    34.48
  Thumb-2nd-pitch                |    17.28
  Thumb-3rd-pitch                |     8.42

  Overall Peak Torque: 38.40 Nm
  Average Peak Torque: 8.79 Nm

Applying 1.2x safety margin...
  Adjusted force limits: 10.54 Nm (avg), 46.08 Nm (max)

======================================================================
PHASE 2: FORWARD SIMULATION WITH COMPUTED LIMITS
======================================================================
  Actuator 0: forcelimit = ±0.01 Nm
  Actuator 1: forcelimit = ±0.01 Nm
  Actuator 2: forcelimit = ±0.00 Nm
  Actuator 3: forcelimit = ±0.00 Nm
  Actuator 4: forcelimit = ±0.00 Nm
  Actuator 5: forcelimit = ±0.01 Nm
  Actuator 6: forcelimit = ±0.00 Nm
  Actuator 7: forcelimit = ±0.00 Nm
  Actuator 8: forcelimit = ±0.69 Nm
  Actuator 9: forcelimit = ±38.31 Nm
  Actuator 10: forcelimit = ±18.38 Nm
  Actuator 11: forcelimit = ±4.61 Nm
  Actuator 12: forcelimit = ±0.34 Nm
  Actuator 13: forcelimit = ±46.08 Nm
  Actuator 14: forcelimit = ±18.73 Nm
  Actuator 15: forcelimit = ±8.78 Nm
  Actuator 16: forcelimit = ±2.71 Nm
  Actuator 17: forcelimit = ±41.38 Nm
  Actuator 18: forcelimit = ±20.74 Nm
  Actuator 19: forcelimit = ±10.10 Nm

Control gains: kp=200.0, kv=20.0

Starting forward simulation...
  T=0.00s: RMS error=0.0805 rad, Max torque=0.00 Nm
  T=1.00s: RMS error=0.5289 rad, Max torque=5677.05 Nm
  T=2.00s: RMS error=0.5857 rad, Max torque=5856.84 Nm
  T=6.00s: RMS error=0.6156 rad, Max torque=5788.22 Nm
  T=9.00s: RMS error=0.6767 rad, Max torque=5534.85 Nm
  T=9.50s: RMS error=0.6649 rad, Max torque=5792.53 Nm
  T=10.50s: RMS error=0.7100 rad, Max torque=5532.93 Nm
  T=12.00s: RMS error=0.7244 rad, Max torque=5531.02 Nm
  T=15.00s: RMS error=0.6943 rad, Max torque=5799.31 Nm
  T=19.50s: RMS error=0.7018 rad, Max torque=5802.76 Nm
  T=25.00s: RMS error=0.7083 rad, Max torque=5644.57 Nm
  T=27.00s: RMS error=0.6811 rad, Max torque=5645.55 Nm
  T=31.50s: RMS error=0.6724 rad, Max torque=5678.99 Nm
  T=33.50s: RMS error=0.7017 rad, Max torque=5648.84 Nm
  T=34.00s: RMS error=0.7003 rad, Max torque=5677.66 Nm
  T=37.00s: RMS error=0.6997 rad, Max torque=5650.77 Nm
  T=38.00s: RMS error=0.7042 rad, Max torque=5651.34 Nm
  T=40.50s: RMS error=0.6781 rad, Max torque=5652.79 Nm
  T=41.50s: RMS error=0.6797 rad, Max torque=5673.38 Nm
  T=45.00s: RMS error=0.7026 rad, Max torque=5671.27 Nm
  T=46.00s: RMS error=0.6637 rad, Max torque=5670.63 Nm
  T=46.50s: RMS error=0.6907 rad, Max torque=5670.31 Nm
  T=55.50s: RMS error=0.7002 rad, Max torque=5664.28 Nm
  T=57.00s: RMS error=0.7013 rad, Max torque=5663.60 Nm
  T=59.50s: RMS error=0.6628 rad, Max torque=5661.44 Nm
  T=62.00s: RMS error=0.6863 rad, Max torque=5667.35 Nm
  T=63.00s: RMS error=0.6854 rad, Max torque=5658.62 Nm
  T=63.50s: RMS error=0.7104 rad, Max torque=5668.49 Nm
  T=64.50s: RMS error=0.6836 rad, Max torque=5657.46 Nm
  T=67.00s: RMS error=0.6699 rad, Max torque=5655.44 Nm
  T=69.50s: RMS error=0.6782 rad, Max torque=5653.29 Nm
  T=70.50s: RMS error=0.6915 rad, Max torque=5652.47 Nm
  T=72.00s: RMS error=0.6837 rad, Max torque=5651.14 Nm
  T=75.00s: RMS error=0.6799 rad, Max torque=5648.44 Nm
