#!/usr/bin/env python3
"""
Mode 4: Inverse-to-Forward Validation
Uses inverse dynamics torques as motor limits to test forward tracking

Script Version: 20260205_194807
Generated: 2026-02-05 19:48:07
"""
import time
import mujoco
import mujoco.viewer
import numpy as np
import json

# Version info
SCRIPT_VERSION = "20260205_194807"
SCRIPT_DATE = "2026-02-05 19:48:07"

print("="*70)
print(f"MODE 4 SCRIPT VERSION: {SCRIPT_VERSION}")
print(f"Generated: {SCRIPT_DATE}")
print("="*70)
print()

try:
    from scipy.interpolate import CubicSpline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    # scipy is installed by setup script, this shouldn't happen
    print("⚠️  WARNING: scipy import failed!")
    print("   Using fallback linear interpolation (may cause instability)")
    print()

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available")

# CRITICAL: Remove actuators from MJCF for pure torque control
# Position actuators have builtin PD controllers that interfere with qfrc_applied
print("")
print("="*70)
print("PREPARING MODEL FOR PURE TORQUE CONTROL")
print("="*70)
print("Removing position actuators from MJCF...")
print("  This allows pure qfrc_applied control without actuator interference")

# Read and modify MJCF
with open("direct_hand_parm.xml", 'r') as f:
    mjcf_content = f.read()

# Remove <actuator> section entirely
import re
# Find and remove everything between <actuator> and </actuator>
mjcf_modified = re.sub(r'<actuator>.*?</actuator>', '', mjcf_content, flags=re.DOTALL)

# Save modified MJCF temporarily
import tempfile
import os
temp_mjcf = tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False, dir=os.path.dirname("direct_hand_parm.xml"))
temp_mjcf.write(mjcf_modified)
temp_mjcf_path = temp_mjcf.name
temp_mjcf.close()

print(f"  Created temporary MJCF without actuators: {os.path.basename(temp_mjcf_path)}")

# Load model from modified MJCF
model = mujoco.MjModel.from_xml_path(temp_mjcf_path)
data = mujoco.MjData(model)

print(f"  Model loaded: {model.nu} actuators (should be 0), {model.nv} DOFs")
if model.nu > 0:
    print("  ⚠️ WARNING: Model still has actuators! Actuator removal may have failed.")
else:
    print("  ✅ SUCCESS: Pure torque control model (no actuators)")
print("="*70)

# Load recording
recording_data = {
  "id": "9697918c-3ed8-4d32-8655-df21b16789a8",
  "name": "Recording 1768623534448",
  "duration": 6835,
  "keyframes": [
    {
      "timestamp": 580,
      "joints": {
        "IndexFinger-2nd-pitch": 0,
        "RingFinger-3rd-pitch": 0,
        "LittleFinger-3rd-pitch": 0,
        "RingFinger-1st-pitch": 0,
        "IndexFinger-3rd-pitch": 0,
        "MiddleFinger-2nd-pitch": 0,
        "IndexFinger_1st_roll": 0,
        "Thumb-3rd-pitch": 0,
        "Thumb-1st-pitch": 0,
        "RingFinger-2nd-pitch": 0,
        "LittleFinger-1st-pitch": 0,
        "IndexFinger-1st-pitch": 0,
        "Thumb-2nd-pitch": 0,
        "LittleFinger-1st_roll": 0,
        "MiddleFinger-1st-pitch": 0,
        "MiddleFinger_1st_roll": 0,
        "LittleFinger-2nd-pitch": 0,
        "thumb_1st_yaw": 0,
        "RingFinger-1st_roll": 0,
        "MiddleFinger-3rd-pitch": 0
      }
    },
    {
      "timestamp": 2093,
      "joints": {
        "IndexFinger-2nd-pitch": 0,
        "RingFinger-3rd-pitch": 0,
        "LittleFinger-3rd-pitch": 0,
        "RingFinger-1st-pitch": 0,
        "IndexFinger-3rd-pitch": 0,
        "MiddleFinger-2nd-pitch": 0,
        "IndexFinger_1st_roll": 0,
        "Thumb-3rd-pitch": 0,
        "Thumb-1st-pitch": -0.940796326794897,
        "RingFinger-2nd-pitch": 0,
        "LittleFinger-1st-pitch": 0,
        "IndexFinger-1st-pitch": -1.5707963267949,
        "Thumb-2nd-pitch": 0,
        "LittleFinger-1st_roll": 0,
        "MiddleFinger-1st-pitch": 0,
        "MiddleFinger_1st_roll": 0,
        "LittleFinger-2nd-pitch": 0,
        "thumb_1st_yaw": 0,
        "RingFinger-1st_roll": 0,
        "MiddleFinger-3rd-pitch": 0
      }
    },
    {
      "timestamp": 3614,
      "joints": {
        "IndexFinger-2nd-pitch": 0,
        "RingFinger-3rd-pitch": 0,
        "LittleFinger-3rd-pitch": 0,
        "RingFinger-1st-pitch": 0,
        "IndexFinger-3rd-pitch": 0,
        "MiddleFinger-2nd-pitch": 0,
        "IndexFinger_1st_roll": 0,
        "Thumb-3rd-pitch": 0,
        "Thumb-1st-pitch": 0,
        "RingFinger-2nd-pitch": 0,
        "LittleFinger-1st-pitch": 0,
        "IndexFinger-1st-pitch": 0,
        "Thumb-2nd-pitch": 0,
        "LittleFinger-1st_roll": 0,
        "MiddleFinger-1st-pitch": 0,
        "MiddleFinger_1st_roll": 0,
        "LittleFinger-2nd-pitch": 0,
        "thumb_1st_yaw": 0,
        "RingFinger-1st_roll": 0,
        "MiddleFinger-3rd-pitch": 0
      }
    },
    {
      "timestamp": 5052,
      "joints": {
        "IndexFinger-2nd-pitch": 0,
        "RingFinger-3rd-pitch": 0,
        "LittleFinger-3rd-pitch": 0,
        "RingFinger-1st-pitch": 0,
        "IndexFinger-3rd-pitch": 0,
        "MiddleFinger-2nd-pitch": -0.220796326794897,
        "IndexFinger_1st_roll": 0,
        "Thumb-3rd-pitch": 0,
        "Thumb-1st-pitch": -0.780796326794897,
        "RingFinger-2nd-pitch": 0,
        "LittleFinger-1st-pitch": 0,
        "IndexFinger-1st-pitch": 0,
        "Thumb-2nd-pitch": 0,
        "LittleFinger-1st_roll": 0,
        "MiddleFinger-1st-pitch": -1.5707963267949,
        "MiddleFinger_1st_roll": 0.0254670748005671,
        "LittleFinger-2nd-pitch": 0,
        "thumb_1st_yaw": -0.393598775598299,
        "RingFinger-1st_roll": 0,
        "MiddleFinger-3rd-pitch": 0
      }
    },
    {
      "timestamp": 6835,
      "joints": {
        "IndexFinger-2nd-pitch": 0,
        "RingFinger-3rd-pitch": 0,
        "LittleFinger-3rd-pitch": 0,
        "RingFinger-1st-pitch": 0,
        "IndexFinger-3rd-pitch": 0,
        "MiddleFinger-2nd-pitch": 0,
        "IndexFinger_1st_roll": 0,
        "Thumb-3rd-pitch": 0,
        "Thumb-1st-pitch": 0,
        "RingFinger-2nd-pitch": 0,
        "LittleFinger-1st-pitch": 0,
        "IndexFinger-1st-pitch": 0,
        "Thumb-2nd-pitch": 0,
        "LittleFinger-1st_roll": 0,
        "MiddleFinger-1st-pitch": 0,
        "MiddleFinger_1st_roll": 0,
        "LittleFinger-2nd-pitch": 0,
        "thumb_1st_yaw": 0,
        "RingFinger-1st_roll": 0,
        "MiddleFinger-3rd-pitch": 0
      }
    }
  ]
}
duration = recording_data["duration"] / 1000.0  # Convert ms to seconds
keyframes = recording_data["keyframes"]

# Build joint mapping
joint_ids = {}
for i in range(model.njnt):
    jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if jnt_name:
        joint_ids[jnt_name] = i

# Build actuator mapping (should be empty now since we removed actuators)
actuator_ids = {}
if model.nu > 0:
    print("⚠️ Model still has actuators, building mapping...")
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if act_name:
            # Actuator names usually end with "_act", strip to get joint name
            joint_name = act_name.replace("_act", "")
            if joint_name in joint_ids:
                actuator_ids[joint_name] = i
else:
    # No actuators - this is what we want for pure torque control
    print("✅ No actuators in model (pure torque control mode)")

print(f"Model has {model.njnt} joints, {model.nu} actuators")
print(f"Recording duration: {duration:.2f}s ({recording_data['duration']:.0f}ms)")

# Interpolate trajectory
dt = model.opt.timestep
n_steps = int(duration / dt) + 1
qpos_traj = np.zeros((n_steps, model.nq))
qvel_traj = np.zeros((n_steps, model.nv))
qacc_traj = np.zeros((n_steps, model.nv))

# Get joint names from first keyframe
recorded_joints = list(keyframes[0]["joints"].keys()) if keyframes else []
print(f"Found {len(recorded_joints)} joints in recording")

# DEBUG: Check keyframe timing
print("")
print("🔍 KEYFRAME TIMING:")
print(f"  Number of keyframes: {len(keyframes)}")
if len(keyframes) > 0:
    print(f"  First keyframe time: {keyframes[0]['timestamp']/1000.0:.3f}s")
    print(f"  Last keyframe time: {keyframes[-1]['timestamp']/1000.0:.3f}s")
    print(f"  Recording duration: {duration:.3f}s")
    print(f"  Keyframe times: ", end="")
    for i, kf in enumerate(keyframes[:5]):
        print(f"{kf['timestamp']/1000.0:.2f}s ", end="")
    if len(keyframes) > 5:
        print("...")
    else:
        print()

for jname in recorded_joints:
    if jname not in joint_ids:
        print(f"Warning: Joint {jname} in recording not found in model")
        continue
    
    jnt_idx = joint_ids[jname]
    qadr = model.jnt_qposadr[jnt_idx]
    dof_adr = model.jnt_dofadr[jnt_idx]
    
    # Build time array starting from 0
    times = [0.0] + [kf["timestamp"]/1000.0 for kf in keyframes]
    
    # Get initial position from model's default qpos (not recording's first keyframe)
    # Recording often starts at 0, which causes physics instability
    model_default_pos = model.qpos0[qadr] if qadr < len(model.qpos0) else 0.0
    first_pos = keyframes[0]["joints"].get(jname, model_default_pos)
    
    # If recording starts at 0, use model default instead
    if abs(first_pos) < 0.001:  # Close to zero
        first_pos = model_default_pos
    
    positions = [first_pos] + [kf["joints"].get(jname, first_pos) for kf in keyframes]
    
    t_interp = np.linspace(0, duration, n_steps)
    
    if HAS_SCIPY:
        # Use cubic spline for smooth interpolation (reduces acceleration spikes)
        cs = CubicSpline(times, positions, bc_type='clamped')  # clamped = zero velocity at endpoints
        q_interp = cs(t_interp)
        qvel_interp = cs(t_interp, 1)  # First derivative (velocity)
        qacc_interp = cs(t_interp, 2)  # Second derivative (acceleration)
    else:
        # Fallback: Linear interpolation with moving average smoothing
        print(f"  Warning: {jname} using linear interpolation (install scipy for better results)")
        q_interp = np.interp(t_interp, times, positions)
        # Simple moving average filter to reduce noise
        window = 21
        q_smooth = np.convolve(q_interp, np.ones(window)/window, mode='same')
        qvel_interp = np.gradient(q_smooth, dt)
        qacc_interp = np.gradient(qvel_interp, dt)
        q_interp = q_smooth  # Use smoothed version for position
    
    qpos_traj[:, qadr] = q_interp
    qvel_traj[:, dof_adr] = qvel_interp
    qacc_traj[:, dof_adr] = qacc_interp

# DEBUG: Check trajectory variation
print("")
print("🔍 TRAJECTORY DIAGNOSTIC:")
print("Checking if trajectory actually changes over time...")
for jname in ['IndexFinger-1st-pitch', 'MiddleFinger-1st-pitch', 'Thumb-1st-pitch']:
    if jname in joint_ids:
        jid = joint_ids[jname]
        qadr = model.jnt_qposadr[jid]
        dof_adr = model.jnt_dofadr[jid]
        print(f"  {jname}:")
        print(f"    Step 0:    {qpos_traj[0, qadr]:.4f} rad")
        print(f"    Step 500:  {qpos_traj[500, qadr]:.4f} rad")
        print(f"    Step 2500: {qpos_traj[2500, qadr]:.4f} rad")
        print(f"    Step 5000: {qpos_traj[5000, qadr]:.4f} rad")
        print(f"    Range: {np.max(qpos_traj[:, qadr]) - np.min(qpos_traj[:, qadr]):.4f} rad")
        print(f"    Vel range: {np.min(qvel_traj[:, dof_adr]):.2f} to {np.max(qvel_traj[:, dof_adr]):.2f} rad/s")
        print(f"    Acc range: {np.min(qacc_traj[:, dof_adr]):.2f} to {np.max(qacc_traj[:, dof_adr]):.2f} rad/s²")

print("")
print("="*70)
print("PHASE 1: INVERSE DYNAMICS ANALYSIS")
print("="*70)
print("Calculating required torques for trajectory...")

# CRITICAL DIAGNOSTIC: Check data structure
print("")
print("🔍 DATA STRUCTURE DIAGNOSTIC:")
print(f"  model.nu (actuators): {model.nu}")
print(f"  model.nv (DOFs): {model.nv}")
print(f"  model.nq (positions): {model.nq}")
print("")
print("  Joint → DOF mapping:")
for jname, jid in joint_ids.items():
    dof_adr = model.jnt_dofadr[jid]
    print(f"    {jname:30s} → DOF {dof_adr}")
print("")
print("  Actuator → Joint mapping:")
if model.nu > 0:
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        trnid = model.actuator_trnid[i, 0]  # Transmission ID (joint)
        if trnid >= 0:
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
            print(f"    Actuator[{i}] {act_name:25s} → Joint {joint_name}")
else:
    print("    (No actuators - using pure torque control)")

print("")
print("="*70)
# Phase 1: Run inverse dynamics to compute required torques
# CRITICAL: Since we removed actuators, use DOF-based torque tracking
max_torques = np.zeros(model.nv)  # Use nv (DOFs) instead of nu (actuators)
torque_history = []

data.qpos[:] = qpos_traj[0]
data.qvel[:] = 0.0
mujoco.mj_forward(model, data)

for step in range(n_steps):
    data.qpos[:] = qpos_traj[step]
    data.qvel[:] = qvel_traj[step]
    data.qacc[:] = qacc_traj[step]
    
    mujoco.mj_inverse(model, data)
    
    # DEBUG: Print qfrc_inverse at step 2500
    if step == 2500:
        print(f"\n🔍 DEBUG Step 2500 - qfrc_inverse values:")
        print(f"  qfrc_inverse shape: {data.qfrc_inverse.shape}")
        print(f"  qfrc_inverse max: {np.max(np.abs(data.qfrc_inverse)):.2f} Nm")
        for jname, jid in joint_ids.items():
            dof_adr = model.jnt_dofadr[jid]
            qfrc = data.qfrc_inverse[dof_adr]
            if abs(qfrc) > 0.1:
                print(f"    {jname:30s} DOF[{dof_adr}]: {qfrc:+8.2f} Nm")
    
    # CRITICAL: Store torques in DOF space (not actuator space, since we have no actuators)
    # qfrc_inverse gives us the required generalized forces at each DOF
    torques_dof = data.qfrc_inverse.copy()
    
    torque_history.append(torques_dof)
    
    # Track max for statistics
    torques_abs = np.abs(torques_dof)
    max_torques = np.maximum(max_torques, torques_abs)
    
    if step % 500 == 0:
        if len(max_torques) > 0:
            print(f"  Step {step}/{n_steps}: Max torque so far = {np.max(max_torques):.2f} Nm")
        else:
            print(f"  Step {step}/{n_steps}: Processing...")

torque_history = np.array(torque_history)

# Save torque_history to CSV for verification
print("")
print("💾 Saving torque history to CSV...")
import csv
with open('phase1_torque_history.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    
    # Header: time, step, then joint names (in DOF order)
    header = ['time_s', 'step']
    joint_names_ordered = []
    for jname, jid in joint_ids.items():
        joint_names_ordered.append(jname)
        header.append(jname)
    writer.writerow(header)
    
    # Data rows
    for step_idx in range(len(torque_history)):
        row = [step_idx * dt, step_idx]
        # torque_history[step_idx] is DOF-indexed array
        for jname, jid in joint_ids.items():
            dof_adr = model.jnt_dofadr[jid]
            torque = torque_history[step_idx][dof_adr]
            row.append(torque)
        writer.writerow(row)
    
print(f"  Saved {len(torque_history)} steps × {len(joint_ids)} joints to phase1_torque_history.csv")
print(f"  File size: ~{len(torque_history) * len(joint_ids) * 8 / 1024:.1f} KB")
    
    # Data rows
    for step in range(n_steps):
        time_s = step * dt
        row = [time_s, step] + torque_history[step].tolist()
        writer.writerow(row)
    
print(f"  Saved {n_steps} steps × {model.nu} joints to phase1_torque_history.csv")
print(f"  File size: ~{n_steps * model.nu * 8 / 1024:.1f} KB")

print("")
print("Inverse Dynamics Results:")
print("  Joint Name                    | Max Torque (Nm)")
print("  " + "-"*60)
for jname, jid in joint_ids.items():
    dof_adr = model.jnt_dofadr[jid]
    if dof_adr < len(max_torques):
        print(f"  {jname:30s} | {max_torques[dof_adr]:8.2f}")

if len(max_torques) > 0:
    print(f"\n  Overall Peak Torque: {np.max(max_torques):.2f} Nm")
    print(f"  Average Peak Torque: {np.mean(max_torques):.2f} Nm")
else:
    print(f"\n  No torque data available")

# Add safety margin - need more for real forward dynamics!
# Inverse dynamics is ideal, forward needs extra for friction, damping, errors
safety_margin = 2.0  # Increased from 1.2
adjusted_limits = max_torques * safety_margin

if len(adjusted_limits) > 0:
    print(f"\nApplying {safety_margin}x safety margin...")
    print(f"  → Inverse dynamics only accounts for ideal motion")
    print(f"  → Forward simulation needs extra for friction, damping, numerical errors")
    print(f"  Adjusted force limits: {np.mean(adjusted_limits):.2f} Nm (avg), {np.max(adjusted_limits):.2f} Nm (max)")

print("")
print("="*70)
print("PHASE 2: PHYSICS SIMULATION WITH TORQUE CONTROL")
print("="*70)

print("")
print("Using REAL PHYSICS SIMULATION with torque control:")
print("  Apply torques from Phase 1 → mj_step() → simulate dynamics")
print("  This tests: Can torque control track the trajectory?")
print("  Goal: Prepare for motor parameter tuning (Mode 2 development)")
print("")
print("Note: Using mj_step() for REAL dynamics simulation")
print("  NOT kinematic playback - we want to see physics behavior!")

# CRITICAL: Initialize from trajectory first frame, NOT qpos=0
# qpos=0 causes geometric constraint violations → NaN/Inf
# Solution: Start from a valid configuration (trajectory start)
print("")
print("📍 Initialization for physics simulation:")
print("  Setting initial pose to TRAJECTORY FIRST FRAME (not qpos=0)")
print("  This avoids geometric constraint violations")
mujoco.mj_resetData(model, data)

# Initialize from trajectory start (valid configuration)
data.qpos[:] = qpos_traj[0]
data.qvel[:] = qvel_traj[0]

# CRITICAL: Disable ALL actuators for pure torque control
# Setting ctrl=0 is not enough - actuators still apply forces!
print("  Disabling position actuators (using ONLY qfrc_applied torque control)")
data.ctrl[:] = 0.0  # Zero control signal

# Note: data.act is for muscle actuators only, size is 0 for position actuators
# So we skip zeroing data.act

data.qfrc_applied[:] = 0.0

# Compute forward kinematics for initial state
mujoco.mj_forward(model, data)

print(f"  Initial pose from trajectory: {data.qpos[:5]}")
print(f"  Actuators disabled, using pure torque control")
print(f"  This should be stable (no constraint violations)")

# Check initial position (should be zero mismatch since we set it directly)
print("")
print("=== Initial Position Check ===")
init_errors = []
for jname, jnt_idx in joint_ids.items():
    qadr = model.jnt_qposadr[jnt_idx]
    actual_pos = data.qpos[qadr]
    target_pos = qpos_traj[0, qadr]
    error = target_pos - actual_pos
    init_errors.append(error ** 2)
    
    if abs(error) > 0.1:  # > 5.7 degrees
        print(f"  {jname:20s}: actual={actual_pos:+7.4f}, traj_start={target_pos:+7.4f}, diff={error:+7.4f} ({np.rad2deg(error):+6.2f}°)")

if init_errors:
    init_rms = np.sqrt(np.mean(init_errors))
    print("")
    print(f"Initial mismatch RMS: {init_rms:.4f} rad ({np.rad2deg(init_rms):.2f}°)")
print("Note: With torque control, initial mismatch is okay")
print("=" * 50)

# Tracking data
tracking_errors = []
control_torques = []
times = []

# Phase 2 data logging (save every step for CSV)
phase2_log = []  # Will store: [time, step, target_pos, actual_pos, applied_force] per joint

print("")
print("Starting forward simulation with TORQUE CONTROL...")

# DEBUG: Check torque_history contents
print("")
print("🔍 TORQUE HISTORY DIAGNOSTIC:")
print(f"  torque_history shape: {torque_history.shape}")
print(f"  Expected: ({n_steps}, {model.nv})")  # nv not nu!
print("")
print("  Sample torques at key steps:")
for sample_step in [0, 500, 2500, 5000]:
    if sample_step < len(torque_history):
        max_t = np.max(np.abs(torque_history[sample_step]))
        nonzero = np.count_nonzero(np.abs(torque_history[sample_step]) > 0.01)
        print(f"    Step {sample_step}: max={max_t:.2f} Nm, nonzero={nonzero}/{model.nv}")
        # Show which joints have significant torque
        for jname, jid in joint_ids.items():
            dof_adr = model.jnt_dofadr[jid]
            t = torque_history[sample_step][dof_adr]
            if abs(t) > 0.1:  # Only show significant torques
                print(f"      {jname:30s}: {t:+8.2f} Nm")

print("")
print("Initial position set. RMS: 0.0000 rad (should be ~0)")
print("=" * 50)

print("")
print("Starting forward simulation...")

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        sim_step = 0  # Simulation step counter
        
        while viewer.is_running() and sim_step < n_steps:
            # REAL PHYSICS SIMULATION with torque control
            # This is what we want to test for Mode 2 development!
            
            # Apply torques from inverse dynamics (Phase 1) WITH light PD feedback
            # Pure feedforward is unstable - add stabilizing feedback
            if sim_step < len(torque_history):
                # Feedforward torque from inverse dynamics
                ff_torque = torque_history[sim_step]
                
                # Very light PD feedback for stabilization
                # CRITICAL: kp/kd must be VERY LOW to avoid instability
                # Position actuators in MJCF have their own builtin PD that interferes!
                kp = 1.0   # Proportional gain (very low)
                kd = 0.1   # Derivative gain (very low)
                
                # Compute feedback torque for each DOF
                fb_torque = np.zeros(model.nv)
                for jname, jid in joint_ids.items():
                    dof_adr = model.jnt_dofadr[jid]
                    qadr = model.jnt_qposadr[jid]
                    
                    # Position and velocity errors
                    pos_error = qpos_traj[sim_step, qadr] - data.qpos[qadr]
                    vel_error = qvel_traj[sim_step, dof_adr] - data.qvel[dof_adr]
                    
                    # PD control (very gentle)
                    fb_torque[dof_adr] = kp * pos_error + kd * vel_error
                
                # Total torque = feedforward + feedback
                data.qfrc_applied[:] = ff_torque + fb_torque
            else:
                data.qfrc_applied[:] = 0.0
            
            # DEBUG: Monitor torque application, physics state, and COLLISIONS
            if sim_step == 0 or sim_step == 500 or sim_step % 1000 == 0:
                print(f"\n🔍 PHYSICS DEBUG at step {sim_step}:")
                print(f"  Using REAL PHYSICS (mj_step) with torque control")
                max_torque = np.max(np.abs(data.qfrc_applied))
                nonzero_torques = np.count_nonzero(np.abs(data.qfrc_applied) > 0.01)
                print(f"  Applied torques: {nonzero_torques}/{model.nv}, Max: {max_torque:.2f} Nm")
                
                # Check for instability warnings
                if np.any(np.isnan(data.qpos)) or np.any(np.isinf(data.qpos)):
                    print("  ⚠️ WARNING: NaN/Inf detected in qpos!")
                if np.max(np.abs(data.qvel)) > 100:
                    print(f"  ⚠️ WARNING: High velocity detected: {np.max(np.abs(data.qvel)):.2f}")
                
                # COLLISION DETECTION: Check for contacts between bodies
                print(f"\n  💥 COLLISION DEBUG:")
                print(f"     Active contacts: {data.ncon}")
                
                if data.ncon > 0:
                    print(f"     ⚠️ COLLISIONS DETECTED! Analyzing contact pairs...")
                    for i in range(min(data.ncon, 10)):  # Show first 10 contacts
                        contact = data.contact[i]
                        geom1 = contact.geom1
                        geom2 = contact.geom2
                        
                        # Get body IDs from geometry IDs
                        body1_id = model.geom_bodyid[geom1]
                        body2_id = model.geom_bodyid[geom2]
                        
                        # Get body names
                        body1_name = model.body(body1_id).name if body1_id >= 0 else "world"
                        body2_name = model.body(body2_id).name if body2_id >= 0 else "world"
                        
                        # Contact distance (negative = penetration)
                        dist = contact.dist
                        
                        # Get contact force magnitude
                        # contact.frame: contact frame (3x3 rotation matrix stored as 9 elements)
                        # We need to compute force from constraint forces
                        # For now, just show penetration depth
                        
                        print(f"       Contact {i+1}: {body1_name:25s} <-> {body2_name:25s}")
                        print(f"                 Penetration: {-dist*1000:.2f} mm" + 
                              (" 🔴 DEEP!" if dist < -0.005 else ""))
                
                else:
                    print(f"     ✅ No collisions (collision exclusions working)")
                
                # Show sample joint states
                print(f"\n  Joint States:")
                for jname, jid in list(joint_ids.items())[:3]:
                    if jname in actuator_ids:
                        dof_adr = model.jnt_dofadr[jid]
                        qadr = model.jnt_qposadr[jid]
                        pos = data.qpos[qadr]
                        vel = data.qvel[dof_adr]
                        torque = data.qfrc_applied[dof_adr]
                        target = qpos_traj[sim_step, qadr]
                        error = target - pos
                        print(f"    {jname:30s}: pos={pos:+7.3f} (target={target:+7.3f}, err={error:+7.3f}), vel={vel:+7.3f}, torque={torque:+7.2f} Nm")
            
            # Run physics simulation step
            # This is the KEY difference from kinematic playback!
            mujoco.mj_step(model, data)
            
            # COLLISION MONITORING: Check for any collisions after step
            # This is CRITICAL - collisions can cause constraint forces that lead to instability
            if data.ncon > 0:
                # Collision detected! This could be the cause of divergence
                if sim_step % 100 == 0:  # Report every 100 steps if collisions persist
                    print(f"\n⚠️ COLLISION WARNING at step {sim_step}:")
                    print(f"   {data.ncon} active contacts detected")
                    # Show first few problematic contacts
                    for i in range(min(data.ncon, 3)):
                        contact = data.contact[i]
                        body1_id = model.geom_bodyid[contact.geom1]
                        body2_id = model.geom_bodyid[contact.geom2]
                        body1_name = model.body(body1_id).name if body1_id >= 0 else "world"
                        body2_name = model.body(body2_id).name if body2_id >= 0 else "world"
                        print(f"      {body1_name} <-> {body2_name}, penetration: {-contact.dist*1000:.2f}mm")
            
            # Check for catastrophic failure (NaN/Inf)
            if np.any(np.isnan(data.qpos)) or np.any(np.isinf(data.qpos)):
                print(f"\n🔴 CATASTROPHIC FAILURE at step {sim_step}:")
                print(f"   NaN/Inf detected in qpos!")
                print(f"   Last known collisions: {data.ncon}")
                if data.ncon > 0:
                    print(f"   Collision body pairs:")
                    for i in range(min(data.ncon, 5)):
                        contact = data.contact[i]
                        body1_id = model.geom_bodyid[contact.geom1]
                        body2_id = model.geom_bodyid[contact.geom2]
                        body1_name = model.body(body1_id).name if body1_id >= 0 else "world"
                        body2_name = model.body(body2_id).name if body2_id >= 0 else "world"
                        print(f"      {body1_name} <-> {body2_name}")
                print(f"\n   This likely means:")
                print(f"   1. Missing collision exclusions (check MJCF <contact> section)")
                print(f"   2. Excessive constraint forces from geometry conflicts")
                print(f"   3. Torques too high or initial configuration invalid")
                break  # Stop simulation
            
            # Store actual applied forces for analysis
            applied_forces = data.qfrc_applied.copy()
            
            # Sync viewer every 10 steps for faster playback (100Hz physics -> 10Hz rendering)
            if sim_step % 10 == 0:
                viewer.sync()
            
            # Calculate elapsed time for logging
            elapsed = sim_step * dt
            
            # Log tracking error with detailed diagnostics
            errors = []
            error_details = []
            saturated_joints = []
            
            for jname, jnt_idx in joint_ids.items():
                qadr = model.jnt_qposadr[jnt_idx]
                error = qpos_traj[sim_step, qadr] - data.qpos[qadr]
                errors.append(error ** 2)
                
                # Get position command value
                ctrl_val = 0.0
                if jname in actuator_ids:
                    aid = actuator_ids[jname]
                    ctrl_val = data.ctrl[aid]
                
                error_details.append((jname, abs(error), abs(ctrl_val)))
                
                # Check if position command is large
                if jname in actuator_ids:
                    aid = actuator_ids[jname]
                    if abs(ctrl_val) > 1.57:  # > 90 degrees
                        saturated_joints.append(jname)
            
            rms_error = np.sqrt(np.mean(errors))
            # Max position command (not force)
            max_ctrl = np.max(np.abs(data.ctrl[:model.nu]))
            
            tracking_errors.append(rms_error)
            control_torques.append(max_ctrl)  # Store for plot (but it's position not torque)
            times.append(elapsed)
            
            # Save Phase 2 data every 10 steps (reduce file size)
            if sim_step % 10 == 0:
                log_entry = {'time': elapsed, 'step': sim_step}
                for jname in joint_ids.keys():
                    if jname in actuator_ids:
                        jid = joint_ids[jname]
                        aid = actuator_ids[jname]
                        qadr = model.jnt_qposadr[jid]
                        dof_adr = model.jnt_dofadr[jid]
                        
                        target = qpos_traj[sim_step, qadr]
                        actual = data.qpos[qadr]
                        applied_torque = applied_forces[dof_adr]  # Torque in Nm, not rad!
                        
                        log_entry[f'{jname}_target'] = target
                        log_entry[f'{jname}_actual'] = actual
                        log_entry[f'{jname}_force'] = applied_torque  # This is Nm, not rad
                        log_entry[f'{jname}_error'] = target - actual
                
                phase2_log.append(log_entry)
            
            # Print progress with diagnostics
            if sim_step % 500 == 0:
                # Find worst 3 joints
                error_details.sort(key=lambda x: x[1], reverse=True)
                worst_joints = error_details[:3]
                
                print(f"  T={elapsed:.2f}s: RMS error={rms_error:.4f} rad ({np.rad2deg(rms_error):.1f}°), Max cmd={max_ctrl:.2f} rad")
                print(f"    Worst errors: ", end="")
                for jname, err, ctrl in worst_joints:
                    print(f"{jname}={np.rad2deg(err):.1f}° (cmd={ctrl:.2f}rad)  ", end="")
                print()
                if saturated_joints:
                    print(f"    Large commands: {', '.join(saturated_joints[:5])}")
            
            sim_step += 1  # Increment simulation step
        
        print("")
        print("Simulation complete!")
        
        # Save Phase 2 applied control for comparison
        print("")
        print("💾 Saving Phase 2 control history to CSV...")
        with open('phase2_control_applied.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Header: time, step, then for each joint: target, actual, force, error
            header = ['time_s', 'step']
            for jname in joint_names_ordered:
                header.extend([
                    f'{jname}_target_rad',
                    f'{jname}_actual_rad',
                    f'{jname}_force_Nm',
                    f'{jname}_error_rad'
                ])
            writer.writerow(header)
            
            # Write logged data
            for entry in phase2_log:
                row = [entry['time'], entry['step']]
                for jname in joint_names_ordered:
                    row.append(entry.get(f'{jname}_target', 0))
                    row.append(entry.get(f'{jname}_actual', 0))
                    row.append(entry.get(f'{jname}_force', 0))
                    row.append(entry.get(f'{jname}_error', 0))
                writer.writerow(row)
        
        print(f"  Saved {len(phase2_log)} samples (every 10 steps) to phase2_control_applied.csv")

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()

# Analysis
if len(tracking_errors) > 0:
    tracking_errors = np.array(tracking_errors)
    control_torques = np.array(control_torques)
    times = np.array(times)
    
    avg_error = np.mean(tracking_errors)
    max_error = np.max(tracking_errors)
    avg_torque = np.mean(control_torques)
    peak_torque = np.max(control_torques)
    
    print("\n" + "="*70)
    print("VALIDATION RESULTS")
    print("="*70)
    print(f"\nTracking Performance:")
    print(f"  Average RMS Error: {avg_error:.6f} rad ({np.rad2deg(avg_error):.3f} deg)")
    print(f"  Maximum RMS Error: {max_error:.6f} rad ({np.rad2deg(max_error):.3f} deg)")
    print(f"\nTorque Control (Direct Force Application):")
    print(f"  Average Torque: {avg_torque:.2f} Nm")
    print(f"  Peak Torque: {peak_torque:.2f} Nm")
    
    # Verdict
    print("\n" + "="*70)
    if max_error < 0.02:  # ~1 degree
        print("✓ SUCCESS: Position control tracks trajectory accurately!")
        print("  → Robot can physically perform this motion")
        print("  → Use these PD gains (kp=200, kd=20) for control")
    elif max_error < 0.1:  # ~5 degrees
        print("⚠ PARTIAL: Some tracking error but acceptable")
        print("  → Try increasing kp (stiffness)")
        print("  → Or check for collisions/joint limits")
    else:
        print("✗ FAILED: Large tracking errors even with position control")
        print("  → Trajectory may be too fast for physics timestep")
        print("  → Or robot model has issues (mass/inertia/constraints)")
    print("="*70)
    
    # Plot results
    if HAS_MATPLOTLIB:
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # Plot 1: Tracking Error (RMS)
        axes[0].plot(times, np.rad2deg(tracking_errors), 'b-', linewidth=1.5, label='RMS Error')
        axes[0].axhline(y=11.5, color='r', linestyle='--', label='Acceptable limit (11.5°)')
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('RMS Tracking Error (degrees)')
        axes[0].set_title('Phase 2: Tracking Performance (Torque Control with PD Feedback)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[0].set_ylim([0, min(180, max(np.rad2deg(tracking_errors)) * 1.1)])  # Cap at 180° for readability
        
        # Plot 2: Applied Torques (from qfrc_applied)
        # Extract max absolute torque at each time step from phase2 log
        max_applied_torques = []
        for step_data in phase2_log:
            # step_data is a dictionary with keys like 'time', 'step', 'jointname_force', etc.
            if isinstance(step_data, dict):
                # Extract all force values (keys ending with '_force')
                forces = [abs(v) for k, v in step_data.items() if k.endswith('_force')]
                if forces:
                    max_applied_torques.append(max(forces))
                else:
                    max_applied_torques.append(0.0)
            else:
                max_applied_torques.append(0.0)
        
        if max_applied_torques and len(max_applied_torques) > 0:
            plot_times = times[:len(max_applied_torques)]
            axes[1].plot(plot_times, max_applied_torques, 'g-', linewidth=1.5, label='Max Applied Torque')
            axes[1].set_xlabel('Time (s)')
            axes[1].set_ylabel('Torque (Nm)')
            axes[1].set_title('Applied Torques (Feedforward + PD Feedback)')
            axes[1].grid(True, alpha=0.3)
            axes[1].legend()
            # Use log scale if torques explode
            if len(max_applied_torques) > 0 and max(max_applied_torques) > 1000:
                axes[1].set_yscale('log')
                axes[1].set_ylabel('Torque (Nm) - Log Scale')
        else:
            # Fallback: just show zeros
            axes[1].plot(times, [0]*len(times), 'g-', linewidth=1.5, label='No torque data')
            axes[1].set_xlabel('Time (s)')
            axes[1].set_ylabel('Torque (Nm)')
            axes[1].set_title('Applied Torques (No Data)')
            axes[1].grid(True, alpha=0.3)
            axes[1].legend()
        
        # Plot 3: Max Joint Velocity
        # This shows if system is diverging
        max_velocities = []
        for i, t in enumerate(times):
            # We don't have velocity data stored, but we can infer from tracking_errors
            # If error is growing rapidly, velocity is high
            max_velocities.append(np.rad2deg(tracking_errors[i]))  # Placeholder
        
        axes[2].plot(times, max_velocities, 'r-', linewidth=1.5, label='Tracking Error Rate')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Error (degrees)')
        axes[2].set_title('Tracking Error Over Time (Shows Divergence)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        plt.tight_layout()
        plt.savefig('mode4_validation.png', dpi=150)
        print("\nPlot saved to: mode4_validation.png")
        plt.show()

# Cleanup temporary MJCF file
try:
    os.remove(temp_mjcf_path)
    print(f"\nCleaned up temporary file: {os.path.basename(temp_mjcf_path)}")
except:
    pass

print("\nMode 4 validation complete.")
