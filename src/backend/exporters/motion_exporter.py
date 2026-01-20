
import re
import json
import math
from typing import List, Dict, Any
from robot_models import RobotData

def process_recordings_for_export(recordings_raw: List[Dict], generated_joints_info: List[Dict], robot_data: RobotData) -> List[Dict]:
    """
    Converts raw UUID-based recordings into a clean Name-based format for export.
    Uses 'generated_joints_info' from URDF generation to ensure 100% name matching,
    supporting multi-DOF joints (split into _roll, _pitch, etc.).
    """
    
    # Build lookup: (original_id, dof_suffix) -> urdf_joint_name
    # derived strings: 'roll', 'pitch', 'yaw', 'prism'
    joint_map = {}
    for info in generated_joints_info:
        orig_id = info.get('original_id')
        suffix = info.get('suffix')
        name = info.get('name')
        if orig_id and suffix and name:
            joint_map[(orig_id, suffix)] = name
            
    processed_recordings = []
    
    # Collect ALL target URDF joint names to ensure complete state
    all_urdf_joints = set()
    for info in generated_joints_info:
        if info.get('name'):
            all_urdf_joints.add(info['name'])

    for rec in recordings_raw:
        clean_rec = {
            "id": rec.get("id"),
            "name": rec.get("name"),
            "duration": rec.get("duration"),
            "keyframes": []
        }
        
        for kf in rec.get("keyframes", []):
            clean_kf = {
                "timestamp": kf.get("timestamp"),
                "joints": {name: 0.0 for name in all_urdf_joints} # Backfill default 0.0
            }
            
            for joint_id, values in kf.get("jointValues", {}).items():
                if joint_id not in robot_data.joints:
                    continue
                
                # 1. Check for Rotational DOFs (roll, pitch, yaw)
                for axis in ['roll', 'pitch', 'yaw']:
                    if (joint_id, axis) in joint_map:
                        urdf_name = joint_map[(joint_id, axis)]
                        val = values.get(axis, 0.0)
                        clean_kf["joints"][urdf_name] = val
                        
                # 2. Check for Prismatic DOF ('prism')
                if (joint_id, 'prism') in joint_map:
                    urdf_name = joint_map[(joint_id, 'prism')]
                    # Frontend calls it 'displacement'
                    val = values.get('displacement', 0.0)
                    clean_kf["joints"][urdf_name] = val

            clean_rec["keyframes"].append(clean_kf)
        
        processed_recordings.append(clean_rec)
            
    return processed_recordings

def generate_ros2_playback_node(pkg_name: str) -> str:
    """Generates ROS 2 python node for playback."""
    return f"""#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import time
import os
from ament_index_python.packages import get_package_share_directory
import math

class ReplayNode(Node):
    def __init__(self):
        super().__init__('replay_node_{pkg_name}')
        # Publish to 'replay_joint_states' so joint_state_publisher (in display.launch) can aggregate it
        # and avoid conflict (z-fighting) with default 0-values.
        self.publisher_ = self.create_publisher(JointState, 'replay_joint_states', 10)
        self.timer = self.create_timer(0.016, self.timer_callback) # 60Hz for smooth playback
        
        self.declare_parameter('recording_index', 0)
        
        # Load recordings.json from share/config
        try:
            pkg_share = get_package_share_directory('{pkg_name}')
            json_path = os.path.join(pkg_share, 'config', 'recordings.json')
            self.get_logger().info(f"Loading recordings from: {{json_path}}")
            with open(json_path, 'r') as f:
                self.recordings = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load recordings: {{e}}")
            self.recordings = []

        self.start_time = time.time()
        self.current_recording = None
        
        # Select recording based on parameter
        if self.recordings:
            idx = self.get_parameter('recording_index').get_parameter_value().integer_value
            if 0 <= idx < len(self.recordings):
                self.current_recording = self.recordings[idx]
                self.get_logger().info(f"Playing recording [#{{idx}}]: {{self.current_recording['name']}}")
            else:
                self.current_recording = self.recordings[0]
                self.get_logger().warn(f"Index {{idx}} out of range. Defaulting to first recording: {{self.current_recording['name']}}")
        else:
            self.get_logger().warn("No recordings found.")

    def timer_callback(self):
        if not self.current_recording:
            return

        # Logging Counter
        if not hasattr(self, 'log_counter'):
             self.log_counter = 0

        now = time.time()
        elapsed_ms = (now - self.start_time) * 1000.0
        
        duration = self.current_recording['duration']
        if duration == 0: duration = 1000 # Safety
        
        # Log Progress every ~1 second (30 ticks)
        if self.log_counter % 30 == 0:
            pct = (elapsed_ms / duration) * 100.0
            if pct > 100: pct = 100.0
            self.get_logger().info(f"Progress: {{elapsed_ms/1000:.1f}}s / {{duration/1000:.1f}}s ({{pct:.1f}}%)")
        self.log_counter += 1

        if elapsed_ms > duration:
            # Loop
            self.start_time = now
            elapsed_ms = 0
            
        joint_positions = self.interpolate(self.current_recording, elapsed_ms)
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        
        for name, pos in joint_positions.items():
            msg.name.append(name)
            msg.position.append(pos)
            
        self.publisher_.publish(msg)

    def interpolate(self, recording, time_ms):
        keyframes = recording['keyframes']
        if not keyframes: return {{}}
        
        prev_kf = keyframes[0]
        next_kf = keyframes[-1]
        
        # Basic scan
        for i in range(len(keyframes)-1):
            if keyframes[i]['timestamp'] <= time_ms < keyframes[i+1]['timestamp']:
                prev_kf = keyframes[i]
                next_kf = keyframes[i+1]
                break
        
        if time_ms >= keyframes[-1]['timestamp']:
            prev_kf = keyframes[-1]
            next_kf = keyframes[-1]
            
        if prev_kf == next_kf:
            return prev_kf['joints']
            
        t1 = prev_kf['timestamp']
        t2 = next_kf['timestamp']
        ratio = (time_ms - t1) / (t2 - t1) if (t2 - t1) > 0 else 0
        
        result = {{}}
        import math
        for jname, v1 in prev_kf['joints'].items():
            v2 = next_kf['joints'].get(jname, v1)
            
            # Shortest Path Interpolation for Angles
            # Heuristic: If jump is > PI, assume wrapping.
            diff = v2 - v1
            if diff > 3.14159:
                diff -= 2 * 3.14159
            elif diff < -3.14159:
                diff += 2 * 3.14159
                
            result[jname] = v1 + diff * ratio
            
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ReplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

def generate_mujoco_playback_script(model_filename: str) -> str:
    """Generates MuJoCo python script."""
    return f"""
import time
import json
import mujoco
import mujoco.viewer
import numpy as np
import os
import argparse

# Try to import matplotlib for sensor graphing
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    print("Warning: matplotlib not found. Sensor graphs will not be displayed.")
    HAS_MATPLOTLIB = False

parser = argparse.ArgumentParser(description='Play back exported recordings in MuJoCo.')
parser.add_argument('index', type=int, nargs='?', default=0, help='Index of the recording to play (default: 0)')
args = parser.parse_args()

# --- Configuration ---
MODEL_XML = "{model_filename}" # Assumed to be in same directory
RECORDINGS_JSON = "recordings.json"

if not os.path.exists(MODEL_XML):
    print(f"Error: Model file {{MODEL_XML}} not found.")
    exit(1)

print(f"Loading model: {{MODEL_XML}}")
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

recordings = []
if os.path.exists(RECORDINGS_JSON):
    with open(RECORDINGS_JSON, "r") as f:
        recordings = json.load(f)
    print(f"Loaded {{len(recordings)}} recordings.")
else:
    print("Warning: recordings.json not found.")

current_recording = None
if 0 <= args.index < len(recordings):
    current_recording = recordings[args.index]
    print(f"Playing recording [#{{args.index}}]: {{current_recording['name']}}")
else:
    if recordings:
        print(f"Index {{args.index}} out of range. Playing #0.")
        current_recording = recordings[0]
    else:
        print("No recordings available.")

start_time = time.time()

# Sensor Data Storage
sensor_history = []
sensor_names = []
timestamps = []

# DEBUG: Print controlled joints to help user debug thumb issues
if current_recording and current_recording.get('keyframes'):
    controlled_joints = list(current_recording['keyframes'][0]['joints'].keys())
    print(f"\\n[DEBUG] Controlled Joints in Recording: {{controlled_joints}}")
    print(f"[DEBUG] If your thumb joint is not in this list, the recording does not contain data for it.\\n")

def interpolate(recording, elapsed_ms):
    if not recording: return {{}}
    keyframes = recording['keyframes']
    if not keyframes: return {{}}
    
    prev_kf = keyframes[0]
    next_kf = keyframes[-1]
    
    # Simple scan
    for i in range(len(keyframes)-1):
        if keyframes[i]['timestamp'] <= elapsed_ms < keyframes[i+1]['timestamp']:
            prev_kf = keyframes[i]
            next_kf = keyframes[i+1]
            break
            
    if elapsed_ms >= keyframes[-1]['timestamp']:
        prev_kf = keyframes[-1]
        next_kf = keyframes[-1]

    if prev_kf == next_kf:
        return prev_kf['joints']

    t1 = prev_kf['timestamp']
    t2 = next_kf['timestamp']
    ratio = (elapsed_ms - t1) / (t2 - t1) if (t2 - t1) > 0 else 0
    
    result = {{}}
    for jname, v1 in prev_kf['joints'].items():
        v2 = next_kf['joints'].get(jname, v1)
        result[jname] = v1 + (v2 - v1) * ratio
    return result

# Launch Viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting simulation loop...")
    while viewer.is_running():
        step_start = time.time()
        
        if current_recording:
             elapsed = (time.time() - start_time) * 1000
             duration = current_recording['duration']
             if duration == 0: duration = 1000
             
             if elapsed >= duration:
                 # Stop at end
                 elapsed = duration
             
             # Log every 1 second
             if not 'log_last_time' in locals():
                 log_last_time = time.time()
             
             if time.time() - log_last_time > 1.0:
                 pct = (elapsed / duration) * 100.0
                 if pct > 100: pct = 100.0
                 print(f"Progress: {{elapsed/1000:.1f}}s / {{duration/1000:.1f}}s ({{pct:.1f}}%)")
                 log_last_time = time.time()

             targets = interpolate(current_recording, elapsed)
             
             # Apply targets to qpos
             for jname, val in targets.items():
                 # Find joint ID by name
                 jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                 if jid != -1:
                     qadr = model.jnt_qposadr[jid]
                     data.qpos[qadr] = val
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Capture Sensor Data
        if model.nsensor > 0:
            if not sensor_names:
                for i in range(model.nsensor):
                    fullname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
                    sensor_names.append(fullname if fullname else f"Sensor {{i}}")
            
            # Read sensordata
            current_vals = data.sensordata.copy()
            sensor_history.append(current_vals)

        # Frame rate control
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# --- Plotting Sensor Data ---
if HAS_MATPLOTLIB and sensor_history:
    print("\\nGenerating sensor plot...")
    hist_arr = np.array(sensor_history)
    
    plt.figure(figsize=(10, 6))
    # hist_arr shape: (steps, n_sensors)
    for i in range(hist_arr.shape[1]):
        label = sensor_names[i] if i < len(sensor_names) else f"Sensor {{i}}"
        plt.plot(hist_arr[:, i], label=label)
    
    plt.title("Sensor Data over Time")
    plt.xlabel("Simulation Steps")
    plt.ylabel("Sensor Output")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()
else:
    if not sensor_history and model.nsensor > 0:
        print("No sensor data captured.")
"""

def generate_mujoco_interactive_script(model_filename: str) -> str:
    """Generates MuJoCo python script for interactive control + LIVE sensor plotting with HIERARCHICAL UI."""
    return f"""
import time
import mujoco
import mujoco.viewer
import numpy as np
import os
import csv
import collections
import re

# Try to import matplotlib for sensor graphing
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Button
import mpl_toolkits.mplot3d # Required for projection='3d'





import mpl_toolkits.mplot3d # Required for projection='3d'

# --- Configuration ---
MODEL_XML = "{model_filename}"

if not os.path.exists(MODEL_XML):
    print(f"Error: Model file {{MODEL_XML}} not found.")
    exit(1)

print(f"Loading model: {{MODEL_XML}}")
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

print("\\n=== Interactive Mode ===")
print("Use the MuJoCo viewer controls to move joints.")
print("Right window: Real-time 3D Sensor Visualizer.")
print("Select FINGERS to view their sensor grids.")
print("Press 'C' in view to see collision geoms.")

start_time = time.time()

# Sensor Data Storage
sensor_history = []
sensor_names = []
timestamps = []

# --- Detect Sensors and Group ---
sensor_indices_by_finger = collections.defaultdict(list)
# Ensure order: Thumb, Index, Middle, Ring, Little, Other
finger_categories = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Other']
active_fingers = {{'Thumb': False, 'Index': True, 'Middle': False, 'Ring': False, 'Little': False, 'Other': False}}

# Helper to parse grid position from name (e.g., _sensor_3_2 -> row 3, col 2)
def parse_grid_pos(name):
    # Pattern: ..._sensor_ROW_COL or similar
    # Our exporter produces: {{body}}_sensor_{{row}}_{{col}}
    # e.g. "index_finger-3rd-end_sensor_0_1"
    match = re.search(r'_sensor_(\\d+)_(\\d+)', name)
    if match:
        return int(match.group(1)), int(match.group(2))
    return -1, -1

# Map sensor index to (finger, row, col)
sensor_map = {{}}

if model.nsensor > 0:
    for i in range(model.nsensor):
        fullname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        name = fullname if fullname else f"Sensor_{{i}}"
        sensor_names.append(name)
        
        low = name.lower()
        cat = 'Other'
        if 'thumb' in low: cat = 'Thumb'
        elif 'index' in low: cat = 'Index'
        elif 'middle' in low: cat = 'Middle'
        elif 'ring' in low: cat = 'Ring'
        elif 'little' in low or 'pinky' in low: cat = 'Little'
        
        sensor_indices_by_finger[cat].append(i)
        
        # Parse grid
        r, c = parse_grid_pos(name)
        if r != -1:
            # Shift column to 0-indexed if it is 1-3
            # Exporter uses 1,2,3. Let's map to 0,1,2
            if c > 0: c -= 1
            sensor_map[i] = (cat, r, c)
        else:
            # Linear fallback
            sensor_map[i] = (cat, i, 0) # Just stack them if no grid found

else:
    print("No sensors found in model.")

# --- Setup Real-time Plot (3D) ---
plt.ion()
fig = plt.figure(figsize=(16, 8))
plt.subplots_adjust(left=0.2, right=0.95, bottom=0.1, top=0.95, wspace=0.3, hspace=0.3)

# UI Area (Left side)
ax_ui_check = plt.axes([0.02, 0.6, 0.15, 0.3]) # Checkboxes
ax_ui_save = plt.axes([0.02, 0.5, 0.15, 0.05])  # Save button
ax_ui_clear = plt.axes([0.02, 0.43, 0.15, 0.05]) # Clear button

# Dynamic Subplots Store
subplots = {{}} # cat -> ax
bars = {{}} # cat -> bar_collection (or list of bars)

# --- UI Callbacks ---
def update_layout(val=None):
    # Clear all existing subplots
    for cat, ax in subplots.items():
        fig.delaxes(ax)
    subplots.clear()
    bars.clear()
    
    # Identify active categories
    visible_cats = [cat for cat in finger_categories if active_fingers[cat] and sensor_indices_by_finger[cat]]
    n = len(visible_cats)
    if n == 0:
        plt.draw()
        return

    # Create subplots (1 row, N cols)
    for i, cat in enumerate(visible_cats):
        ax = fig.add_subplot(1, n, i+1, projection='3d')
        ax.set_title(f"{{cat}} Sensors")
        ax.set_zlabel("Force (N)")
        ax.set_ylim(0, 4) # approx 3 cols 
        ax.set_xlim(0, 8) # approx 7 rows
        ax.set_zlim(0, 5) # Max force expected
        
        # Setup grid mesh for this finger
        # We need to map sensor INDICES to x,y positions
        # x = row, y = col
        
        _indices = sensor_indices_by_finger[cat]
        _x, _y, _z_bottom = [], [], []
        _dx, _dy = [], []
        
        # Initialize bar positions
        for idx in _indices:
            _, r, c = sensor_map.get(idx, (cat, 0, 0))
            # Orient: Row along X, Col along Y?
            # 3 cols (width), 7 rows (length along finger)
            _x.append(r) 
            _y.append(c) 
            _z_bottom.append(0)
            _dx.append(0.8) # Width of bar
            _dy.append(0.8)
        
        _z_height = np.zeros(len(_indices)) # Initial height 0
        
        # Create initial bars
        # Note: bar3d returns a Poly3DCollection
        # modifying it efficiently is tricky in older matplotlib, but we will try standard approach.
        # Actually, for animation, remove and redraw is often necessary or setting properties.
        # But let's verify if we can just update.
        
        # Store metadata needed to redraw/update
        bars[cat] = {{
            'ax': ax,
            'indices': _indices,
            'x': np.array(_x),
            'y': np.array(_y),
            'dx': np.array(_dx),
            'dy': np.array(_dy)
        }}
        
        subplots[cat] = ax

    plt.draw()

def toggle_finger(label):
    active_fingers[label] = not active_fingers[label]
    update_layout()

# Checkbuttons
labels = [cat for cat in finger_categories if sensor_indices_by_finger[cat]]
actives = [active_fingers[cat] for cat in labels]
check = CheckButtons(ax_ui_check, labels, actives)
check.on_clicked(toggle_finger)

# Save Button
def save_data(event):
    if not sensor_names or not sensor_history:
        print("No data to save.")
        return
    filename = f"sensor_log_{{int(time.time())}}.csv"
    print(f"Saving data to {{filename}}...")
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time'] + sensor_names)
        for t, row in zip(timestamps, sensor_history):
            writer.writerow([t] + list(row))
    print("Save complete.")

btn_save = Button(ax_ui_save, 'Save to CSV')
btn_save.on_clicked(save_data)

# Clear Data Button
def clear_data(event):
    global sensor_history, timestamps
    sensor_history = []
    timestamps = []
    start_time = time.time() # Reset time relative
    print("Data history cleared.")

btn_clear = Button(ax_ui_clear, 'Clear Data')
btn_clear.on_clicked(clear_data)


# Initial layout
update_layout()



plt.show(block=False)

# --- Launch Viewer ---
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting simulation loop...")
    
    last_plot_time = time.time()
    
    while viewer.is_running():
        step_start = time.time()
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Capture Sensor Data
        if model.nsensor > 0:
            current_vals = data.sensordata.copy()
            t_now = time.time() - start_time
            
            sensor_history.append(current_vals)
            timestamps.append(t_now)
            
            # Update Plot at 10Hz to save FPS
            if time.time() - last_plot_time > 0.1:
                
                # Check for zero data warning
                if current_vals.max() == 0 and current_vals.min() == 0:
                     if not hasattr(viewer, 'has_warned_zero'):
                         # print("[DEBUG] All sensors 0.0") 
                         viewer.has_warned_zero = True
                
                # Update 3D Plots
                # Matplotlib 3D animation is slow if fully redrawn.
                # However, set_3d_properties isn't always sufficient for bar3d.
                # The 'fast' way for bar3d is unfortunately clearing and re-adding or specific collection hacks.
                # We will try clearing the collections only.
                
                for cat, meta in bars.items():
                    ax = meta['ax']
                    indices = meta['indices']
                    
                    # Extract heights for this finger's sensors
                    # current_vals is full array.
                    z_heights = current_vals[indices]
                    
                    # Clear previous bars
                    # ax.collections.clear() # This clears too much? check
                    # Better: remove specific collection if we stored it?
                    # Let's just create new ones and clear ax? No, axes props lost.
                    
                    # Remove old collections
                    for c in ax.collections:
                        c.remove()
                    
                    # Re-plot
                    # Use color map based on intensity
                    # Normalize simple 0-5
                    colors = plt.cm.jet(z_heights / 5.0)
                    
                    ax.bar3d(meta['x'], meta['y'], np.zeros_like(z_heights), 
                             meta['dx'], meta['dy'], z_heights, 
                             color=colors, shade=True)
                    
                    # Re-set limits because cla/collections clear might shift auto-scale
                    # (bar3d usually respects limits if set, but removing collections is safe)
                    # Force limits again just in case
                    ax.set_zlim(0, 5) 

                # Handle GUI events
                fig.canvas.draw_idle()
                try:
                    fig.canvas.flush_events()
                except NotImplementedError:
                    plt.pause(0.001)
                
                last_plot_time = time.time()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

plt.ioff()
plt.show() 
"""


def generate_replay_script(recording_id: str, recording_name: str) -> str:
    """Generates a python script to replay a specific recording in MuJoCo."""
    sanitized_name = re.sub(r'[^a-zA-Z0-9_]', '_', recording_name)
    return f"""
import mujoco
import mujoco.viewer
import time
import json
import os
import math

# Load model
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

# Load recordings
with open("recordings.json", "r") as f:
    recordings = json.load(f)

# Find target recording
target_id = "{recording_id}"
recording = next((r for r in recordings if r['id'] == target_id), None)

if not recording:
    print(f"Recording {{target_id}} not found.")
    exit(1)

print(f"Replaying: {{recording['name']}} (Duration: {{recording['duration']}}s)")

# Lookup joints
joint_names = [model.joint(i).name for i in range(model.njnt)]
joint_ids = {{name: model.joint(name).id for name in joint_names}}

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    while viewer.is_running():
        elapsed = time.time() - start_time
        
        # Simple playback loop (looping)
        t_in_rec = elapsed % recording['duration']
        
        # Find keyframe (simple Step interpolation for now, or Linear)
        # Assuming sorted keyframes
        current_kf = recording['keyframes'][0]
        for kf in recording['keyframes']:
            if kf['timestamp'] > t_in_rec:
                break
            current_kf = kf
            
        # Apply joint values
        for j_name, j_val in current_kf['joints'].items():
            if j_name in joint_ids:
                data.qpos[joint_ids[j_name]] = j_val
                
        mujoco.mj_step(model, data)
        viewer.sync()
"""

def generate_demo_script(python_script_name: str) -> str:
    """
    Generates a bash script that:
    1. Creates a venv if missing.
    2. Installs dependencies in the venv.
    3. Runs the target python script within the venv.
    """
    return f"""#!/bin/bash
set -e

# Define venv directory
VENV_DIR="venv"

# 1. Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# 2. Activate venv
source "$VENV_DIR/bin/activate"

# 3. Install dependencies
# Upgrade pip to avoid warnings
pip install --upgrade pip > /dev/null 2>&1

# Install required packages
# Pin matplotlib to <=3.7.3 to avoid issues with removed APIs (e.g. initializers)
# Pin numpy < 2 to avoid breaking changes
echo "Installing dependencies..."
pip install mujoco "matplotlib<=3.7.3" "numpy<2" > /dev/null 2>&1

# 4. Run the interactive script
echo "Running interactive viewer..."
python3 {python_script_name} "$@"
"""

def generate_torque_launch_script(python_script_name: str, default_rec_idx: int = 0) -> str:
    """
    Generates a bash script that sets up the environment and runs the torque replay.
    Includes interactive mode selection.
    """
    return f"""#!/bin/bash
set -e

# Define venv directory
VENV_DIR="venv"

# 1. Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# 2. Activate venv
source "$VENV_DIR/bin/activate"

# 3. Install dependencies
# Upgrade pip to avoid warnings
pip install --upgrade pip > /dev/null 2>&1

# Install required packages
# Pin matplotlib to <=3.7.3 to avoid issues with removed APIs
# Pin numpy < 2 to avoid breaking changes
echo "Installing dependencies (mujoco, matplotlib, numpy)..."
pip install mujoco "matplotlib<=3.7.3" "numpy<2" > /dev/null 2>&1

# 4. Interactive Mode Selection
echo "----------------------------------------"
echo "Select Visualization Mode:"
echo "1. Joint Torques (3x5 Grid) [Default]"
echo "2. Fingertip Sensors (3x7 Grid)"
echo "----------------------------------------"
read -p "Enter choice [1]: " choice

MODE="joints"
if [ "$choice" = "2" ]; then
    MODE="sensors"
fi

# 5. Run the script
echo "Starting Torque Replay in mode: $MODE..."
python3 {python_script_name} {default_rec_idx} --mode $MODE
"""

def generate_mujoco_torque_replay_script(model_filename: str) -> str:
    """Generates MuJoCo python script for Replay with Real-time Torque Visualization (3x5 Grid)."""
    return f"""
import time
import json
import mujoco
import mujoco.viewer
import numpy as np
import os
import argparse
import sys
import csv

# Try importing matplotlib
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    print("Warning: matplotlib not found. Visualization will be disabled.")
    HAS_MATPLOTLIB = False

# --- Configuration ---
MODEL_XML = "{model_filename}"
RECORDINGS_JSON = "recordings.json"

# --- 3x5 Finger Grid Config ---
FINGER_KEYS = ['thumb', 'index', 'middle', 'ring', 'little']
FINGER_NAMES = ['Thumb', 'Index', 'Middle', 'Ring', 'Little']
GRID_ROWS_JOINTS = 3 # Joints per finger
GRID_ROWS_SENSORS = 1 # Sensors per finger (typically 1 tip)

def detect_finger_joints(model):
    all_joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    buckets = {{key: [] for key in FINGER_KEYS}}
    
    for jname in all_joints:
        if not jname: continue
        low = jname.lower()
        for key in FINGER_KEYS:
            if key == 'little' and ('little' in low or 'pinky' in low):
                buckets[key].append(jname)
                break
            elif key in low:
                buckets[key].append(jname)
                break
    
    grid_map = [[None for _ in range(5)] for _ in range(3)]
    for c, key in enumerate(FINGER_KEYS):
        joints = sorted(buckets[key])
        for r in range(min(3, len(joints))):
             grid_map[r][c] = joints[r]
    return grid_map


def detect_finger_sensors(model):
    # Returns 5x21 flattened map (approx) or structured dict
    # We want to find sensors with suffixes _R_C where R=0..6, C=1..3
    nsens = model.nsensor
    all_sensors = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i) for i in range(nsens)]
    
    # Structure: [finger_idx][row][col] -> sensor_name
    # 5 fingers, 7 rows, 3 cols
    grid_map = [[[None for _ in range(3)] for _ in range(7)] for _ in range(5)]
    
    for sname in all_sensors:
        if not sname: continue
        low = sname.lower()
        
        # Identify Finger
        f_idx = -1
        for i, key in enumerate(FINGER_KEYS):
            if key == 'little' and ('little' in low or 'pinky' in low): f_idx = i; break
            elif key in low: f_idx = i; break
        
        if f_idx == -1: continue
        
        # Identify Grid Position (Suffix _R_C)
        # We look for pattern "_(\d)_(\d)$" or similar.
        # The sensor names are like "..._sensor_0_1"
        try:
            parts = sname.split('_')
            # Look for last two digits
            # e.g. ["...", "sensor", "0", "1"]
            if len(parts) >= 2 and parts[-1].isdigit() and parts[-2].isdigit():
                r = int(parts[-2])
                c = int(parts[-1])
                # c is 1-based in config (1,2,3), we want 0-based
                c_idx = c - 1
                if 0 <= r < 7 and 0 <= c_idx < 3:
                     grid_map[f_idx][r][c_idx] = sname
        except:
            pass
            
    return grid_map

# --- Main Logic ---

if not os.path.exists(MODEL_XML):
    print(f"Error: Model file {{MODEL_XML}} not found.")
    exit(1)

print(f"Loading model: {{MODEL_XML}}")
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

# Maps
joint_ids = {{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): i for i in range(model.njnt)}}
sensor_ids = {{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i): i for i in range(model.nsensor)}}
actuator_ids = {{}}
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    if name and name.endswith("_act"):
        jname = name[:-4]
        actuator_ids[jname] = i

if not os.path.exists(RECORDINGS_JSON):
    print("recordings.json not found.")
    exit(1)
    
with open(RECORDINGS_JSON, "r") as f:
    recs = json.load(f)

# Parse Arguments
parser = argparse.ArgumentParser()
parser.add_argument("index", nargs="?", type=int, default=0, help="Index of recording")
parser.add_argument("--mode", choices=["joints", "sensors"], default="joints", help="Visualization mode")
args = parser.parse_args()

rec_idx = args.index
if rec_idx < 0 or rec_idx >= len(recs):
    print(f"Index {{rec_idx}} out of range. Defaulting to 0.")
    rec_idx = 0
rec = recs[rec_idx]
print(f"Loaded recording [#{{rec_idx}}]: {{rec['name']}} (Mode: {{args.mode}})")

# Setup Mode
if args.mode == "sensors":
    sensor_grid_map = detect_finger_sensors(model)
    # DEBUG: Check if sensors were detected for 3x7 grid
    found_sensors_count = 0
    for f in range(5):
        for r in range(7):
            for c in range(3):
                if sensor_grid_map[f][r][c]: found_sensors_count += 1
    
    if found_sensors_count == 0:
        print("[DEBUG] No sensors matched the 3x7 grid pattern (_R_C suffix).")
        print("[DEBUG] Falling back to linear list (1xN) if any sensors exist.")
        # If the user expected a 3x7 grid but got none, this explains the "1x5" perception if they see something else or nothing.
    else:
        print(f"[DEBUG] Mapped {{found_sensors_count}} sensors to 3x7 grid.")
    log_file_name = "sensor_log.csv"
    data_source_names = sorted(sensor_ids.keys())
    z_lim = (0, 5) 
else:
    joint_grid_map = detect_finger_joints(model)
    log_file_name = "torque_log.csv"
    data_source_names = sorted(joint_ids.keys())
    data_source_names = sorted(joint_ids.keys())
    z_lim = (-20, 20)

print(f"Logging to: {{log_file_name}}")
csv_file = open(log_file_name, 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['Time'] + data_source_names)

print("Starting Replay...")

# --- Pre-calculate Trajectory (Interpolation) ---
print("Pre-calculating trajectory for Inverse Dynamics...")

# Prepare time array for simulation steps
duration = rec['duration'] / 1000.0 # seconds
if duration <= 0: duration = 1.0
dt = model.opt.timestep
trajectory_times = np.arange(0, duration, dt)
n_steps = len(trajectory_times)

# Target arrays (Steps x Dims)
qpos_traj = np.zeros((n_steps, model.nq))
qvel_traj = np.zeros((n_steps, model.nv))
qacc_traj = np.zeros((n_steps, model.nv))

# Flatten keyframes to arrays for interpolation
kf_times = [k['timestamp']/1000.0 for k in rec['keyframes']]
kf_joints = rec['keyframes']

# For each joint, interpolate
for jname in joint_ids.keys():
    jid = joint_ids[jname]
    qadr = model.jnt_qposadr[jid]
    dof_adr = model.jnt_dofadr[jid] # For velocity/acc mapping (assumes simple 1-1 for now)
    
    # Extract raw points
    y_points = []
    first_val = kf_joints[0]['joints'].get(jname, 0.0)
    for kf in kf_joints:
        y_points.append(kf['joints'].get(jname, first_val))
    
    # Linear Interpolation
    q_interp = np.interp(trajectory_times, kf_times, y_points)
    qpos_traj[:, qadr] = q_interp
    
    # Calculate Velocity (Finite Difference)
    # v[i] = (q[i+1] - q[i]) / dt
    qvel_traj[:, dof_adr] = np.gradient(q_interp, dt)
    
    # Calculate Acceleration
    qacc_traj[:, dof_adr] = np.gradient(qvel_traj[:, dof_adr], dt)

print("Trajectory calculation complete.")

if HAS_MATPLOTLIB:
    plt.ion()
    # Add blocking show check or just force figure creation
    # Some backends need this.
    try:
        plt.show(block=False)
    except: 
        pass
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    mode_title = "Joint Torques (Nm)" if args.mode == 'joints' else "Touch Force (N)"
    ax.set_title(f"{{mode_title}}: {{rec['name']}}")
    ax.set_zlabel('Value')
    
    if args.mode == 'sensors':
        # 3x7 Grid Visualization
        ax.set_xlabel('Fingers (Sub-cols)')
        ax.set_ylabel('Sensor Rows (0=Tip, 6=Base)')
        
        # Prepare Coords
        # 5 Fingers * 3 Cols = 15 columns total (+ spacing)
        # Let's say spacing is 1 unit. Finger width is 3 units.
        # Finger 0: x=0,1,2
        # Finger 1: x=4,5,6
        # ...
        
        x_vals = []
        y_vals = []
        map_linear_to_grid = [] # Stores (f, r, c) for each bar
        
        for f in range(5):
            x_offset = f * 4
            for r in range(7):
                for c in range(3):
                    x_vals.append(x_offset + c)
                    y_vals.append(r)
                    map_linear_to_grid.append((f, r, c))
                    
        x_flat = np.array(x_vals)
        y_flat = np.array(y_vals)
        z_base = np.zeros_like(x_flat)
        dx = 0.8 * np.ones_like(z_base)
        dy = 0.8 * np.ones_like(z_base)
        
        # Ticks
        finger_centers = [f * 4 + 1 for f in range(5)]
        ax.set_xticks(finger_centers)
        ax.set_xticklabels(FINGER_NAMES)
        ax.set_yticks(range(7))
        
    else:
        # Joints Config
        ax.set_xlabel('Fingers')
        ax.set_ylabel('Position')
        _x = np.arange(5)
        _y = np.arange(3)
        _xx, _yy = np.meshgrid(_x, _y)
        x_flat = _xx.flatten()
        y_flat = _yy.flatten()
        z_base = np.zeros_like(x_flat)
        dx = 0.5 * np.ones_like(z_base)
        dy = 0.5 * np.ones_like(z_base)
        ax.set_xticks(_x + 0.25)
        ax.set_xticklabels(FINGER_NAMES)
        ax.set_yticks(_y + 0.25)
        ax.set_yticklabels(['J1', 'J2', 'J3'])
        
    ax.set_zlim(*z_lim)

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting simulation loop...")
    start_time = time.time()
    last_print = 0
    
    while viewer.is_running():
        now = time.time()
        elapsed = now - start_time
        if elapsed > rec['duration']/1000.0:
            start_time = now
            elapsed = 0
            
        # --- INVERSE DYNAMICS MODE ---
        # 1. Look up pre-calculated state
        # elapsed is in seconds
        idx = int(elapsed / dt)
        if idx >= len(trajectory_times):
            idx = len(trajectory_times) - 1
        
        # Set state (qpos, qvel, qacc)
        data.qpos[:model.nq] = qpos_traj[idx]
        data.qvel[:model.nv] = qvel_traj[idx]
        data.qacc[:model.nv] = qacc_traj[idx]
        
        # Inverse Dynamics: Compute forces required to produce qacc given qpos, qvel
        mujoco.mj_inverse(model, data)
        
        viewer.sync()
        
        # 2. Capture Torque (qfrc_inverse)
        # Note: qfrc_inverse includes gravity, coriolis, and contact forces needed.
        # We want to show the Actuator forces required. 
        # data.qfrc_inverse is the generalized force. 
        # For actuated joints, this is the torque the motor must produce.
        
        torques = []
        if args.mode == 'joints':
            for r in range(3):
                row_vals = []
                for c in range(5): # 5 fingers
                    jname = joint_grid_map[r][c]
                    val = 0.0
                    if jname and jname in joint_ids:
                        jid = joint_ids[jname]
                        dof_adr = model.jnt_dofadr[jid]
                        # Use qfrc_inverse
                        val = data.qfrc_inverse[dof_adr]
                    row_vals.append(val)
                torques.append(row_vals)
            
            # Update Plot
            dz_list = []
            c_list = []
            
            for c in range(5):
                for r in range(3):
                    t = torques[r][c]
                    # Color mapping
                    # Limit +/- 20
                    norm_t = max(-20, min(20, t))
                    color_val = (norm_t + 20) / 40.0 # 0..1
                    
                    dz_list.append(t)
                    c_list.append(plt.cm.coolwarm(color_val))

        elif args.mode == 'sensors':
            # Sensors logic remains similar, but using mj_inverse might affect contact dynamics?
            mujoco.mj_sensor(model, data)
            
            current_vals = data.sensordata
            
            dz_list = []
            c_list = []
            
            if sensor_grid_map:
                # 3x7 Grid
                for f in range(5):
                    for r in range(7):
                        for c in range(3):
                            sname = sensor_grid_map[f][r][c]
                            val = 0.0
                            if sname and sname in sensor_ids:
                                s_idx = sensor_ids[sname]
                                val = current_vals[s_idx]
                            
                            dz_list.append(val)
                            c_list.append(plt.cm.viridis(val / 5.0)) # 0..5N range
            else:
                # Linear fallback
                for i in range(min(len(data_source_names), 10)):
                    sname = data_source_names[i]
                    s_idx = sensor_ids[sname]
                    val = current_vals[s_idx]
                    dz_list.append(val)
                    c_list.append(plt.cm.viridis(val / 5.0))


        # Update Bar Plot (Common)
        if HAS_MATPLOTLIB and 'dz_list' in locals() and dz_list:
            try:
                for coll in ax.collections: coll.remove()
            except:
                pass
            
            # RE-DRAW STRATEGY
            ax.clear()
            ax.set_zlim(*z_lim)
            if args.mode == 'joints':
                ax.set_title(f"Joint Torques (Nm) - T={{elapsed:.2f}}s")
                ax.set_xlabel("Fingers")
                ax.set_ylabel("Joints")
                ax.set_xticks(np.arange(5) + 0.25, FINGER_NAMES)
                ax.set_yticks(np.arange(3) + 0.25, ["J1", "J2", "J3"])
                ax.bar3d(x_flat, y_flat, z_base, dx, dy, dz_list, color=c_list, shade=True)
            else: # Sensors mode
                ax.set_title(f"Touch Force (N) - T={{elapsed:.2f}}s")
                ax.set_xlabel("Fingers (Sub-cols)")
                ax.set_ylabel("Sensor Rows (0=Tip, 6=Base)")
                ax.set_xticks(finger_centers, FINGER_NAMES)
                ax.set_yticks(range(7))
                ax.bar3d(x_flat, y_flat, z_base, dx, dy, dz_list, color=c_list, shade=True)
            
            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except:
                pass
            plt.pause(0.001)

        # Logging
        if args.mode == 'sensors':
            row = [elapsed] + [data.sensordata[sensor_ids[n]] for n in data_source_names]
            writer.writerow(row)
        else:
            # Log Torques
            row = [elapsed] + [data.qfrc_inverse[model.jnt_dofadr[joint_ids[n]]] for n in data_source_names]
            writer.writerow(row)

csv_file.close()
"""
