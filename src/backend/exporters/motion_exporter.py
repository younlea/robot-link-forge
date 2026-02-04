import re
import json
import math
from typing import List, Dict, Any
from robot_models import RobotData


def process_recordings_for_export(
    recordings_raw: List[Dict], generated_joints_info: List[Dict], robot_data: RobotData
) -> List[Dict]:
    """
    Converts raw UUID-based recordings into a clean Name-based format for export.
    Uses 'generated_joints_info' from URDF generation to ensure 100% name matching,
    supporting multi-DOF joints (split into _roll, _pitch, etc.).
    """

    # Build lookup: (original_id, dof_suffix) -> urdf_joint_name
    # derived strings: 'roll', 'pitch', 'yaw', 'prism'
    joint_map = {}
    for info in generated_joints_info:
        orig_id = info.get("original_id")
        suffix = info.get("suffix")
        name = info.get("name")
        if orig_id and suffix and name:
            joint_map[(orig_id, suffix)] = name

    processed_recordings = []

    # Collect ALL target URDF joint names to ensure complete state
    all_urdf_joints = set()
    for info in generated_joints_info:
        if info.get("name"):
            all_urdf_joints.add(info["name"])

    for rec in recordings_raw:
        clean_rec = {
            "id": rec.get("id"),
            "name": rec.get("name"),
            "duration": rec.get("duration"),
            "keyframes": [],
        }

        for kf in rec.get("keyframes", []):
            clean_kf = {
                "timestamp": kf.get("timestamp"),
                "joints": {
                    name: 0.0 for name in all_urdf_joints
                },  # Backfill default 0.0
            }

            for joint_id, values in kf.get("jointValues", {}).items():
                if joint_id not in robot_data.joints:
                    continue

                # 1. Check for Rotational DOFs (roll, pitch, yaw)
                for axis in ["roll", "pitch", "yaw"]:
                    if (joint_id, axis) in joint_map:
                        urdf_name = joint_map[(joint_id, axis)]
                        val = values.get(axis, 0.0)
                        clean_kf["joints"][urdf_name] = val

                # 2. Check for Prismatic DOF ('prism')
                if (joint_id, "prism") in joint_map:
                    urdf_name = joint_map[(joint_id, "prism")]
                    # Frontend calls it 'displacement'
                    val = values.get("displacement", 0.0)
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
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Button, Slider, RadioButtons, TextBox
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
    sanitized_name = re.sub(r"[^a-zA-Z0-9_]", "_", recording_name)
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


def generate_motor_analysis_script() -> str:
    """Generates post-processing analysis script for motor validation data"""
    return """
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def analyze_motor_validation(csv_file='motor_validation_log.csv'):
    \"\"\"
    Analyze motor validation log and generate comprehensive report
    \"\"\"
    if not os.path.exists(csv_file):
        print(f"Error: {{csv_file}} not found. Run motor validation first (Mode 2).")
        return
    
    print(f"Loading data from {{csv_file}}...")
    df = pd.read_csv(csv_file)
    
    # Extract metrics
    time = df['Time'].values
    rms_error = df['RMS_tracking_error'].values
    max_torque = df['Max_torque'].values
    num_saturated = df['Num_saturated'].values
    
    # Analysis
    avg_rms_error = np.mean(rms_error)
    max_rms_error = np.max(rms_error)
    avg_torque = np.mean(max_torque)
    peak_torque = np.max(max_torque)
    saturation_events = np.sum(num_saturated > 0)
    saturation_pct = 100 * saturation_events / len(time) if len(time) > 0 else 0
    
    # Thermal load (simplified RMS torque)
    thermal_rms = np.sqrt(np.mean(max_torque ** 2))
    
    print("\\n" + "="*60)
    print("MOTOR VALIDATION ANALYSIS REPORT")
    print("="*60)
    print(f"\\nTracking Performance:")
    print(f"  Average RMS Error: {{avg_rms_error:.6f}} rad ({{np.rad2deg(avg_rms_error):.3f}} deg)")
    print(f"  Maximum RMS Error: {{max_rms_error:.6f}} rad ({{np.rad2deg(max_rms_error):.3f}} deg)")
    print(f"\\nTorque Analysis:")
    print(f"  Average Torque: {{avg_torque:.2f}} Nm")
    print(f"  Peak Torque: {{peak_torque:.2f}} Nm")
    print(f"  RMS Thermal Load: {{thermal_rms:.2f}} Nm")
    print(f"\\nSaturation:")
    print(f"  Saturation Events: {{saturation_events}} / {{len(time)}} samples ({{saturation_pct:.1f}}%)")
    
    if saturation_pct > 10:
        print(f"  [WARNING] High saturation rate! Consider increasing force limit.")
    elif saturation_pct > 0:
        print(f"  [CAUTION] Some saturation detected.")
    else:
        print(f"  [OK] No saturation detected.")
    
    print("\\n" + "="*60)
    
    # Generate plots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Tracking error
    axes[0].plot(time, rms_error, 'b-', linewidth=1.5)
    axes[0].set_ylabel('RMS Tracking Error (rad)')
    axes[0].set_title('Tracking Performance Over Time')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=avg_rms_error, color='r', linestyle='--', label=f'Average: {{avg_rms_error:.6f}}')
    axes[0].legend()
    
    # Torque
    axes[1].plot(time, max_torque, 'r-', linewidth=1.5, label='Max Torque')
    axes[1].axhline(y=avg_torque, color='orange', linestyle='--', label=f'Average: {{avg_torque:.2f}} Nm')
    axes[1].set_ylabel('Torque (Nm)')
    axes[1].set_title('Actuator Torque')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    
    # Saturation
    axes[2].plot(time, num_saturated, 'orange', linewidth=1.5)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Number of Saturated Actuators')
    axes[2].set_title('Torque Saturation Events')
    axes[2].grid(True, alpha=0.3)
    axes[2].fill_between(time, 0, num_saturated, alpha=0.3, color='orange')
    
    plt.tight_layout()
    
    # Save figure
    output_file = csv_file.replace('.csv', '_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\\nAnalysis plot saved to: {{output_file}}")
    
    plt.show()
    
    return {{
        'avg_rms_error': avg_rms_error,
        'peak_torque': peak_torque,
        'thermal_rms': thermal_rms,
        'saturation_pct': saturation_pct
    }}

if __name__ == '__main__':
    import sys
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'motor_validation_log.csv'
    analyze_motor_validation(csv_file)
"""


def generate_inverse_to_forward_validation_script(model_file: str, recording_data: dict) -> str:
    """Generate Mode 4: Inverse-to-Forward Validation Script
    
    Phase 1: Run inverse dynamics to find required torques
    Phase 2: Use those torques as motor limits and test forward tracking
    """
    rec_json = json.dumps(recording_data, indent=2)
    
    return f'''#!/usr/bin/env python3
"""
Mode 4: Inverse-to-Forward Validation
Uses inverse dynamics torques as motor limits to test forward tracking
"""
import time
import mujoco
import mujoco.viewer
import numpy as np
import json

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available")

# Load model
model = mujoco.MjModel.from_xml_path("{model_file}")
data = mujoco.MjData(model)

# Load recording
recording_data = {rec_json}
duration = recording_data["duration"] / 1000.0  # Convert ms to seconds
keyframes = recording_data["keyframes"]

# Build joint mapping
joint_ids = {{}}
for i in range(model.njnt):
    jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if jnt_name:
        joint_ids[jnt_name] = i

# Build actuator mapping (CRITICAL for Mode 4)
actuator_ids = {{}}
for i in range(model.nu):
    act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    if act_name:
        # Actuator names usually end with "_act", strip to get joint name
        joint_name = act_name.replace("_act", "")
        if joint_name in joint_ids:
            actuator_ids[joint_name] = i

print(f"Model has {{model.njnt}} joints, {{model.nu}} actuators")
print(f"Recording duration: {{duration:.2f}}s ({{recording_data['duration']:.0f}}ms)")

# Interpolate trajectory
dt = model.opt.timestep
n_steps = int(duration / dt) + 1
qpos_traj = np.zeros((n_steps, model.nq))
qvel_traj = np.zeros((n_steps, model.nv))
qacc_traj = np.zeros((n_steps, model.nv))

# Get joint names from first keyframe
recorded_joints = list(keyframes[0]["joints"].keys()) if keyframes else []
print(f"Found {{len(recorded_joints)}} joints in recording")

for jname in recorded_joints:
    if jname not in joint_ids:
        print(f"Warning: Joint {{jname}} in recording not found in model")
        continue
    
    jnt_idx = joint_ids[jname]
    qadr = model.jnt_qposadr[jnt_idx]
    dof_adr = model.jnt_dofadr[jnt_idx]
    
    # Build time array starting from 0
    times = [0.0] + [kf["timestamp"]/1000.0 for kf in keyframes]
    
    # Get initial position (first keyframe value)
    first_pos = keyframes[0]["joints"].get(jname, 0.0)
    positions = [first_pos] + [kf["joints"].get(jname, first_pos) for kf in keyframes]
    
    t_interp = np.linspace(0, duration, n_steps)
    q_interp = np.interp(t_interp, times, positions)
    
    qpos_traj[:, qadr] = q_interp
    qvel_traj[:, dof_adr] = np.gradient(q_interp, dt)
    qacc_traj[:, dof_adr] = np.gradient(qvel_traj[:, dof_adr], dt)

# DEBUG: Check trajectory variation
print("\\n🔍 TRAJECTORY DIAGNOSTIC:")
print("Checking if trajectory actually changes over time...")
for jname in ['IndexFinger-1st-pitch', 'MiddleFinger-1st-pitch', 'Thumb-1st-pitch']:
    if jname in joint_ids:
        jid = joint_ids[jname]
        qadr = model.jnt_qposadr[jid]
        print(f"  {{jname}}:")
        print(f"    Step 0:    {{qpos_traj[0, qadr]:.4f}} rad")
        print(f"    Step 500:  {{qpos_traj[500, qadr]:.4f}} rad")
        print(f"    Step 2500: {{qpos_traj[2500, qadr]:.4f}} rad")
        print(f"    Step 5000: {{qpos_traj[5000, qadr]:.4f}} rad")
        print(f"    Range: {{np.max(qpos_traj[:, qadr]) - np.min(qpos_traj[:, qadr]):.4f}} rad")

print("\\n" + "="*70)
print("PHASE 1: INVERSE DYNAMICS ANALYSIS")
print("="*70)
print("Calculating required torques for trajectory...")

# CRITICAL DIAGNOSTIC: Check data structure
print("\\n🔍 DATA STRUCTURE DIAGNOSTIC:")
print(f"  model.nu (actuators): {{model.nu}}")
print(f"  model.nv (DOFs): {{model.nv}}")
print(f"  model.nq (positions): {{model.nq}}")
print("\\n  Joint → DOF mapping:")
for jname, jid in joint_ids.items():
    dof_adr = model.jnt_dofadr[jid]
    print(f"    {{jname:30s}} → DOF {{dof_adr}}")
print("\\n  Actuator → Joint mapping:")
for i in range(model.nu):
    act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    trnid = model.actuator_trnid[i, 0]  # Transmission ID (joint)
    if trnid >= 0:
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
        print(f"    Actuator[{{i}}] {{act_name:25s}} → Joint {{joint_name}}")

# Phase 1: Run inverse dynamics to compute required torques
max_torques = np.zeros(model.nu)
torque_history = []

data.qpos[:] = qpos_traj[0]
data.qvel[:] = 0.0
mujoco.mj_forward(model, data)

print("\\n" + "="*70)
for step in range(n_steps):
    data.qpos[:] = qpos_traj[step]
    data.qvel[:] = qvel_traj[step]
    data.qacc[:] = qacc_traj[step]
    
    mujoco.mj_inverse(model, data)
    
    # DEBUG: Print qfrc_inverse at step 2500
    if step == 2500:
        print(f"\\n🔍 DEBUG Step 2500 - qfrc_inverse values:")
        print(f"  qfrc_inverse shape: {{data.qfrc_inverse.shape}}")
        print(f"  qfrc_inverse max: {{np.max(np.abs(data.qfrc_inverse)):.2f}} Nm")
        for jname, jid in joint_ids.items():
            dof_adr = model.jnt_dofadr[jid]
            qfrc = data.qfrc_inverse[dof_adr]
            if abs(qfrc) > 0.1:
                print(f"    {{jname:30s}} DOF[{{dof_adr}}]: {{qfrc:+8.2f}} Nm")
    
    # CRITICAL: qfrc_inverse is in DOF space, not actuator space!
    # We need to map DOF forces to actuator controls
    # Method: For each actuator, find its joint and read qfrc_inverse at that DOF
    
    torques_actual = np.zeros(model.nu)
    for jname, jid in joint_ids.items():
        dof_adr = model.jnt_dofadr[jid]
        joint_force = data.qfrc_inverse[dof_adr]
        
        # Find corresponding actuator
        if jname in actuator_ids:
            aid = actuator_ids[jname]
            # For motor actuators: ctrl = force / gear
            # But we're testing direct force, so just copy
            torques_actual[aid] = joint_force
    
    torque_history.append(torques_actual)
    
    # Track max for statistics
    torques_abs = np.abs(torques_actual)
    max_torques = np.maximum(max_torques, torques_abs)
    
    if step % 500 == 0:
        print(f"  Step {{step}}/{{n_steps}}: Max torque so far = {{np.max(max_torques):.2f}} Nm")

torque_history = np.array(torque_history)

# Save torque_history to CSV for verification
print("\\n💾 Saving torque history to CSV...")
import csv
with open('phase1_torque_history.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    
    # Header: time, then joint names
    header = ['time_s', 'step']
    joint_names_ordered = []
    for i in range(model.nu):
        # Find joint name for this actuator
        for jname, aid in actuator_ids.items():
            if aid == i:
                joint_names_ordered.append(jname)
                header.append(jname)
                break
    writer.writerow(header)
    
    # Data rows
    for step in range(n_steps):
        time_s = step * dt
        row = [time_s, step] + torque_history[step].tolist()
        writer.writerow(row)
    
print(f"  Saved {{n_steps}} steps × {{model.nu}} joints to phase1_torque_history.csv")
print(f"  File size: ~{{n_steps * model.nu * 8 / 1024:.1f}} KB")

print("\\nInverse Dynamics Results:")
print("  Joint Name                    | Max Torque (Nm)")
print("  " + "-"*60)
for i, jname in enumerate(joint_ids.keys()):
    if i < model.nu:
        print(f"  {{jname:30s}} | {{max_torques[i]:8.2f}}")

print(f"\\n  Overall Peak Torque: {{np.max(max_torques):.2f}} Nm")
print(f"  Average Peak Torque: {{np.mean(max_torques):.2f}} Nm")

# Add safety margin - need more for real forward dynamics!
# Inverse dynamics is ideal, forward needs extra for friction, damping, errors
safety_margin = 2.0  # Increased from 1.2
adjusted_limits = max_torques * safety_margin

print(f"\\nApplying {{safety_margin}}x safety margin...")
print(f"  → Inverse dynamics only accounts for ideal motion")
print(f"  → Forward simulation needs extra for friction, damping, numerical errors")
print(f"  Adjusted force limits: {{np.mean(adjusted_limits):.2f}} Nm (avg), {{np.max(adjusted_limits):.2f}} Nm (max)")

print("\\n" + "="*70)
print("PHASE 2: FORWARD SIMULATION WITH COMPUTED TORQUES")
print("="*70)

print("\\nUsing pure feedforward control:")
print("  data.ctrl = torque_history[step]")
print("  (Direct application of inverse dynamics forces)")
print("\\nThis tests: Can the actuators track trajectory with computed forces?")

# Reset simulation - DO NOT stabilize with torques!
# Inverse dynamics torques are for MOTION, not for holding still
data.qpos[:] = qpos_traj[0]
data.qvel[:] = 0.0
data.qacc[:] = 0.0
data.ctrl[:] = 0.0  # Zero control initially

# Just let physics settle without control
print("\\nLetting physics settle at initial pose...")
for i in range(50):
    mujoco.mj_forward(model, data)

# Check initial error
init_errors = []
for jname, jnt_idx in joint_ids.items():
    if jnt_idx >= model.nu:
        continue
    qadr = model.jnt_qposadr[jnt_idx]
    error = qpos_traj[0, qadr] - data.qpos[qadr]
    init_errors.append(error ** 2)
    if abs(error) > 0.1:  # > 5.7 degrees
        print(f"  Warning: {{jname}} has {{np.rad2deg(abs(error)):.1f}}° initial error")

init_rms = np.sqrt(np.mean(init_errors))
print(f"Initial RMS error after stabilization: {{init_rms:.4f}} rad ({{np.rad2deg(init_rms):.2f}}°)")

if init_rms > 0.2:
    print("  ⚠️  Large initial error! Physics may be unstable or gains too weak")

# Tracking data
tracking_errors = []
control_torques = []
times = []

# Phase 2 data logging (save every step for CSV)
phase2_log = []  # Will store: [time, step, target_pos, actual_pos, ctrl] per joint

print("\\nStarting forward simulation...")

# DEBUG: Check torque_history contents
print("\\n🔍 TORQUE HISTORY DIAGNOSTIC:")
print(f"  torque_history shape: {{torque_history.shape}}")
print(f"  Expected: ({{n_steps}}, {{model.nu}})")
print("\\n  Sample torques at key steps:")
for sample_step in [0, 500, 2500, 5000]:
    if sample_step < len(torque_history):
        max_t = np.max(np.abs(torque_history[sample_step]))
        nonzero = np.count_nonzero(np.abs(torque_history[sample_step]) > 0.01)
        print(f"    Step {{sample_step}}: max={{max_t:.2f}} Nm, nonzero={{nonzero}}/{{model.nu}}")
        # Show which joints have significant torque
        for jname, jid in joint_ids.items():
            if jname in actuator_ids:
                aid = actuator_ids[jname]
                t = torque_history[sample_step][aid]
                if abs(t) > 0.1:  # Only show significant torques
                    print(f"      {{jname:30s}}: {{t:+8.2f}} Nm")

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        step = 0
        
        while viewer.is_running() and step < n_steps:
            now = time.time()
            elapsed = now - start_time
            
            if elapsed > duration:
                break
            
            step = int(elapsed / dt)
            if step >= n_steps:
                step = n_steps - 1
            
            # CRITICAL: We have position actuators, not motor actuators!
            # data.ctrl expects target positions, not forces
            # But we want to apply forces directly from inverse dynamics
            
            # Workaround: Disable actuators and apply forces directly to qfrc_applied
            data.ctrl[:] = 0.0  # Disable position control
            
            # Apply computed forces directly to joints
            for jname, jid in joint_ids.items():
                if jname in actuator_ids:
                    aid = actuator_ids[jname]
                    dof_adr = model.jnt_dofadr[jid]
                    
                    # Apply force directly to the DOF
                    force = torque_history[step][aid]
                    data.qfrc_applied[dof_adr] = force
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # Log tracking error with detailed diagnostics
            errors = []
            error_details = []
            saturated_joints = []
            
            for jname, jnt_idx in joint_ids.items():
                qadr = model.jnt_qposadr[jnt_idx]
                error = qpos_traj[step, qadr] - data.qpos[qadr]
                errors.append(error ** 2)
                
                # Get actuator control value (use actuator_ids, not joint_ids!)
                ctrl_val = 0.0
                if jname in actuator_ids:
                    aid = actuator_ids[jname]
                    ctrl_val = data.ctrl[aid]
                
                error_details.append((jname, abs(error), abs(ctrl_val)))
                
                # Check if saturated
                if jname in actuator_ids:
                    aid = actuator_ids[jname]
                    limit = adjusted_limits[aid]
                    if abs(data.ctrl[aid]) >= limit * 0.99:
                        saturated_joints.append(jname)
            
            rms_error = np.sqrt(np.mean(errors))
            max_ctrl = np.max(np.abs(data.ctrl[:model.nu]))
            
            tracking_errors.append(rms_error)
            control_torques.append(max_ctrl)
            times.append(elapsed)
            
            # Save Phase 2 data every 10 steps (reduce file size)
            if step % 10 == 0:
                log_entry = {{'time': elapsed, 'step': step}}
                for jname in joint_ids.keys():
                    if jname in actuator_ids:
                        jid = joint_ids[jname]
                        aid = actuator_ids[jname]
                        qadr = model.jnt_qposadr[jid]
                        
                        target = qpos_traj[step, qadr]
                        actual = data.qpos[qadr]
                        ctrl = data.ctrl[aid]
                        
                        log_entry[f'{{jname}}_target'] = target
                        log_entry[f'{{jname}}_actual'] = actual
                        log_entry[f'{{jname}}_ctrl'] = ctrl
                        log_entry[f'{{jname}}_error_rad'] = target - actual
                
                phase2_log.append(log_entry)
            
            # Print progress with diagnostics
            if step % 500 == 0:
                # Find worst 3 joints
                error_details.sort(key=lambda x: x[1], reverse=True)
                worst_joints = error_details[:3]
                
                print(f"  T={{elapsed:.2f}}s: RMS error={{rms_error:.4f}} rad, Max torque={{max_ctrl:.2f}} Nm")
                print(f"    Worst errors: ", end="")
                for jname, err, ctrl in worst_joints:
                    print(f"{{jname}}={{np.rad2deg(err):.1f}}° ({{ctrl:.1f}}Nm)  ", end="")
                print()
                if saturated_joints:
                    print(f"    Saturated: {{', '.join(saturated_joints[:5])}}")
        
        print("\\nSimulation complete!")
        
        # Save Phase 2 applied control for comparison
        print("\\n💾 Saving Phase 2 control history to CSV...")
        with open('phase2_control_applied.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Header: time, step, then for each joint: target, actual, ctrl, error
            header = ['time_s', 'step']
            for jname in joint_names_ordered:
                header.extend([
                    f'{{jname}}_target_rad',
                    f'{{jname}}_actual_rad',
                    f'{{jname}}_ctrl_Nm',
                    f'{{jname}}_error_rad'
                ])
            writer.writerow(header)
            
            # Write logged data
            for entry in phase2_log:
                row = [entry['time'], entry['step']]
                for jname in joint_names_ordered:
                    row.append(entry.get(f'{{jname}}_target', 0))
                    row.append(entry.get(f'{{jname}}_actual', 0))
                    row.append(entry.get(f'{{jname}}_ctrl', 0))
                    row.append(entry.get(f'{{jname}}_error_rad', 0))
                writer.writerow(row)
        
        print(f"  Saved {{len(phase2_log)}} samples (every 10 steps) to phase2_control_applied.csv")

except Exception as e:
    print(f"ERROR: {{e}}")
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
    
    print("\\n" + "="*70)
    print("VALIDATION RESULTS")
    print("="*70)
    print(f"\\nTracking Performance:")
    print(f"  Average RMS Error: {{avg_error:.6f}} rad ({{np.rad2deg(avg_error):.3f}} deg)")
    print(f"  Maximum RMS Error: {{max_error:.6f}} rad ({{np.rad2deg(max_error):.3f}} deg)")
    print(f"\\nTorque Usage:")
    print(f"  Average Torque: {{avg_torque:.2f}} Nm")
    print(f"  Peak Torque: {{peak_torque:.2f}} Nm")
    print(f"  Computed Limit: {{np.max(adjusted_limits):.2f}} Nm")
    print(f"  Usage: {{100 * peak_torque / np.max(adjusted_limits):.1f}}%")
    
    # Verdict
    print("\\n" + "="*70)
    if max_error < 0.2:  # ~11 degrees
        print("✓ SUCCESS: Physics can track trajectory with computed torques!")
        print("  → Motor matching should work")
        print("  → Problem was likely in Mode 2 parameter tuning")
    elif max_error < 0.5:  # ~28 degrees
        print("⚠ PARTIAL: Tracking has some error but reasonable")
        print("  → Try adjusting kp/kv gains")
        print("  → Or increase safety margin")
    else:
        print("✗ FAILED: Cannot track trajectory even with computed torques")
        print("  → Problem is in physics model or timestep")
        print("  → Check: mass, inertia, timestep, solver settings")
    print("="*70)
    
    # Plot results
    if HAS_MATPLOTLIB:
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        axes[0].plot(times, np.rad2deg(tracking_errors), 'b-', linewidth=1.5, label='RMS Error')
        axes[0].axhline(y=11.5, color='r', linestyle='--', label='Acceptable limit (11.5°)')
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('RMS Tracking Error (degrees)')
        axes[0].set_title('Tracking Performance (Mode 4: Inverse-to-Forward)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        axes[1].plot(times, control_torques, 'g-', linewidth=1.5, label='Control Torque')
        axes[1].axhline(y=np.max(adjusted_limits), color='r', linestyle='--', label='Computed Limit')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Torque (Nm)')
        axes[1].set_title('Torque Usage')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        plt.tight_layout()
        plt.savefig('mode4_validation.png', dpi=150)
        print("\\nPlot saved to: mode4_validation.png")
        plt.show()

print("\\nMode 4 validation complete.")
'''


def generate_validation_script(model_file: str, recording_data: dict) -> str:
    """Generate validation script with automatic parameter optimization"""
    rec_json = json.dumps(recording_data, indent=2)

    return f'''#!/usr/bin/env python3
"""
Automatic Motor Parameter Optimization and Validation
Finds optimal parameters or diagnoses trajectory issues
"""
import mujoco
import numpy as np
import json
import os
from itertools import product

# Validation thresholds
MAX_ACCEPTABLE_ERROR = 0.20  # rad (~11.5 degrees, very relaxed for aggressive trajectories)
MAX_SATURATION_PCT = 50.0    # Allow up to 50% force saturation
MIN_STABILITY_SCORE = 0.4    # Stability metric (0-1)

# Parameter search ranges - PHYSICS CORRECTED!
# Key insight: High gear → SLOW speed! Must use moderate gear for trajectory tracking
# Torque = gear × (kp × error + kv × velocity)
# Speed ∝ 1/gear (inverse relationship!)
KP_RANGE = [100, 200, 400]  # Moderate position gains
KV_RANGE = [10, 20, 40]  # 10% damping ratio
GEAR_RANGE = [20, 50, 100, 150]  # REALISTIC gear ratios for fast movement
FORCELIM_RANGE = [200, 400, 600]  # Torque = gear × ~6Nm motor

def simulate_with_params(model_file, qpos_traj, joint_ids, actuator_ids, kp, kv, gear, forcelim, n_steps):
    """Run simulation with given parameters and return metrics"""
    try:
        model = mujoco.MjModel.from_xml_path(model_file)
        data = mujoco.MjData(model)
        
        # Set parameters
        for i in range(model.nu):
            model.actuator_gainprm[i][0] = kp  # Position gain
            model.actuator_biasprm[i][1] = -kv  # Velocity damping (CRITICAL: index 1, not 2!)
            model.actuator_gear[i][0] = gear  # Gear ratio for torque amplification
            model.actuator_forcerange[i] = [-forcelim, forcelim]
        
        # CRITICAL FIX: Initialize qpos to trajectory start
        data.qpos[:] = qpos_traj[0]
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        
        # Warmup: Let robot settle at start position (reduced from 50 to 20)
        for _ in range(20):
            for jname, jid in joint_ids.items():
                if jname in actuator_ids:
                    data.ctrl[actuator_ids[jname]] = qpos_traj[0, model.jnt_qposadr[jid]]
            mujoco.mj_step(model, data)
        
        errors = []
        forces = []
        saturated_count = 0
        
        # Run simulation (use half steps for speed)
        test_steps = min(n_steps, 250)
        for step_idx in range(test_steps):
            # Set control targets
            for jname, jid in joint_ids.items():
                if jname in actuator_ids:
                    target_q = qpos_traj[step_idx, model.jnt_qposadr[jid]]
                    data.ctrl[actuator_ids[jname]] = target_q
            
            # Step
            mujoco.mj_step(model, data)
            
            # Collect metrics
            step_errors = []
            step_forces = []
            for jname, jid in joint_ids.items():
                if jname in actuator_ids:
                    qadr = model.jnt_qposadr[jid]
                    target = qpos_traj[step_idx, qadr]
                    actual = data.qpos[qadr]
                    error = abs(target - actual)
                    step_errors.append(error)
                    
                    force = abs(data.actuator_force[actuator_ids[jname]])
                    step_forces.append(force)
                    if force >= forcelim * 0.95:
                        saturated_count += 1
            
            if step_errors:
                errors.append(np.mean(step_errors))
            if step_forces:
                forces.append(np.max(step_forces))
        
        # Calculate metrics
        avg_error = np.mean(errors) if errors else 999
        max_error = np.max(errors) if errors else 999
        saturation_pct = 100 * saturated_count / (len(errors) * len(joint_ids)) if errors else 100
        error_std = np.std(errors) if errors else 999
        stability_score = 1.0 / (1.0 + error_std * 10)
        
        return {{
            'avg_error': avg_error,
            'max_error': max_error,
            'saturation_pct': saturation_pct,
            'stability_score': stability_score,
            'success': avg_error < MAX_ACCEPTABLE_ERROR and saturation_pct < MAX_SATURATION_PCT and stability_score > MIN_STABILITY_SCORE
        }}
    except Exception as e:
        return {{'avg_error': 999, 'success': False, 'error': str(e)}}

def optimize_parameters(model_file='{model_file}'):
    """Automatically find optimal parameters or diagnose issues"""
    
    print("="*70)
    print("[MODE 0] AUTOMATIC MOTOR PARAMETER OPTIMIZATION")
    print("="*70)
    print(f"Model: {{model_file}}")
    print()
    
    # Load model and data
    model = mujoco.MjModel.from_xml_path(model_file)
    data = mujoco.MjData(model)
    
    # Load recording
    rec = {rec_json}
    
    # Build joint mappings - get joint names from first keyframe or model
    joint_ids = {{}}
    actuator_ids = {{}}
    
    # Get joint names from recording's first keyframe
    joint_names = []
    if rec.get('keyframes') and len(rec['keyframes']) > 0:
        first_kf = rec['keyframes'][0]
        if 'joints' in first_kf:
            joint_names = list(first_kf['joints'].keys())
    
    # If no joints found in recording, try to get from model
    if not joint_names:
        print("[WARNING] No joints in recording, using all model joints")
        joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) 
                      for i in range(model.njnt)]
    
    # Map joint names to IDs
    for jname in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            joint_ids[jname] = jid
            act_name = f"{{jname}}_act"
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if aid >= 0:
                actuator_ids[jname] = aid
    
    if not joint_ids:
        print("❌ ERROR: No joints found in model!")
        return False
    
    # Pre-calculate trajectory
    duration = min(rec['duration'] / 1000.0, 5.0)
    dt = model.opt.timestep
    trajectory_times = np.arange(0, duration, dt)
    n_steps = len(trajectory_times)
    
    kf_joints = rec['keyframes']
    # Build time array: start at 0.0, then add all keyframe timestamps
    kf_times_raw = [kf['timestamp'] / 1000.0 for kf in kf_joints]
    kf_times = np.array([0.0] + kf_times_raw)
    
    # Build trajectory
    qpos_traj = np.zeros((n_steps, model.nq))
    for jname, jid in joint_ids.items():
        qadr = model.jnt_qposadr[jid]
        y_points = []
        first_val = 0.0
        if kf_joints and jname in kf_joints[0]['joints']:
            first_val = kf_joints[0]['joints'][jname]
        y_points.append(first_val)
        prev_val = first_val
        for kf in kf_joints:
            val = kf['joints'].get(jname, prev_val)
            y_points.append(val)
            prev_val = val
        q_interp = np.interp(trajectory_times, kf_times, y_points)
        qpos_traj[:, qadr] = q_interp
    
    print("Step 1: Testing default parameters (kp=200, kv=20, gear=50, forcelim=300)...")
    print("-" * 70)
    default_result = simulate_with_params(model_file, qpos_traj, joint_ids, actuator_ids, 
                                         200, 20, 50, 300, n_steps)
    
    print(f"  Tracking Error: {{np.rad2deg(default_result['avg_error']):.2f}}deg (max: {{np.rad2deg(default_result['max_error']):.2f}}deg)")
    print(f"  Saturation: {{default_result['saturation_pct']:.1f}}%")
    print(f"  Stability: {{default_result['stability_score']:.2f}}")
    print()
    
    if default_result['success']:
        print("="*70)
        print("[SUCCESS] DEFAULT PARAMETERS WORK PERFECTLY!")
        print("="*70)
        print("No optimization needed.")
        print()
        
        # Ask user if they want to proceed to Mode 2
        while True:
            response = input("\\n>>  Would you like to open Mode 2 (Interactive Motor Tuning)? [Y/n]: ").strip().lower()
            if response in ['', 'y', 'yes']:
                print("\\n" + "="*70)
                print("[MODE 2] LAUNCHING INTERACTIVE MOTOR TUNING")
                print("="*70)
                print("You can now fine-tune parameters per-joint...\\n")
                # Return special code to launch Mode 2
                return 'launch_mode2'
            elif response in ['n', 'no']:
                print("\\nOptimization complete. Exiting...")
                return True
            else:
                print("Please enter 'y' or 'n'")
        return True
    
    # Start optimization
    print("[X] Default parameters not optimal. Starting automatic search...")
    total_tests = len(KP_RANGE) * len(KV_RANGE) * len(GEAR_RANGE) * len(FORCELIM_RANGE)
    print(f"   Testing {{total_tests}} parameter combinations...")
    print(f"   (kp: {{len(KP_RANGE)}} values, kv: {{len(KV_RANGE)}} values, gear: {{len(GEAR_RANGE)}} values, forcelim: {{len(FORCELIM_RANGE)}} values)")
    print()
    
    best_result = None
    best_params = None
    test_count = 0
    
    for kp, kv, gear, forcelim in product(KP_RANGE, KV_RANGE, GEAR_RANGE, FORCELIM_RANGE):
        test_count += 1
        result = simulate_with_params(model_file, qpos_traj, joint_ids, actuator_ids,
                                     kp, kv, gear, forcelim, n_steps)
        
        # Show progress every 10 tests
        if test_count % 10 == 0:
            print(f"  Progress: {{test_count}}/{{total_tests}} tests completed...")
        
        # Update best result (prioritize success, then lowest error)
        if best_result is None or (result['success'] and not best_result['success']) or \\
           (result['success'] == best_result['success'] and result['avg_error'] < best_result['avg_error']):
            best_result = result
            best_params = {{'kp': kp, 'kv': kv, 'gear': gear, 'forcelim': forcelim}}
            
            # If we found a working combination, we can stop early
            if result['success']:
                print(f"  [SUCCESS] Found working parameters at test {{test_count}}/{{total_tests}}!")
                break
    
    print()
    print("="*70)
    
    if best_result and best_result['success']:
        print("[SUCCESS] OPTIMAL PARAMETERS FOUND!")
        print("="*70)
        print()
        print("Recommended parameters:")
        print(f"  kp (position gain):     {{best_params['kp']}}")
        print(f"  kv (velocity damping):  {{best_params['kv']}}")
        print(f"  gear (torque amplif.):  {{best_params['gear']}} <-- KEY PARAMETER!")
        print(f"  forcelim (force limit): {{best_params['forcelim']}} Nm")
        print()
        print("Performance with these parameters:")
        print(f"  Tracking Error: {{np.rad2deg(best_result['avg_error']):.2f}}deg (target: < {{np.rad2deg(MAX_ACCEPTABLE_ERROR)}}deg)")
        print(f"  Saturation: {{best_result['saturation_pct']:.1f}}% (target: < {{MAX_SATURATION_PCT}}%)")
        print(f"  Stability: {{best_result['stability_score']:.2f}} (target: > {{MIN_STABILITY_SCORE}})")
        print()
        print("Next steps:")
        print("  1. Update these values in src/backend/exporters/mjcf_exporter.py:")
        print(f"     - Line ~250: kp=\\"{{best_params['kp']}}\\" kv=\\"{{best_params['kv']}}\\" gear=\\"{{best_params['gear']}}\\"")
        print(f"     - Line ~250: forcerange=\\"-{{best_params['forcelim']}} {{best_params['forcelim']}}\\"")
        print("  2. Re-export your robot model")
        print("  3. Or use Mode 2 to apply these per-joint")
        print()
        print("="*70)
        
        # Ask user if they want to proceed to Mode 2
        while True:
            response = input("\\n>>  Would you like to open Mode 2 (Interactive Motor Tuning)? [Y/n]: ").strip().lower()
            if response in ['', 'y', 'yes']:
                print("\\n" + "="*70)
                print("[MODE 2] LAUNCHING INTERACTIVE MOTOR TUNING")
                print("="*70)
                print("Applying optimized parameters as defaults...\\n")
                # Return special code with best params
                return ('launch_mode2', best_params)
            elif response in ['n', 'no']:
                print("\\nOptimization complete. Exiting...")
                return True
            else:
                print("Please enter 'y' or 'n'")
        return True
    else:
        # No working parameters found
        print("[WARNING] NO WORKING PARAMETERS FOUND")
        print("="*70)
        print()
        print("Best attempt (still failing):")
        if best_params:
            print(f"  kp={{best_params['kp']}}, kv={{best_params['kv']}}, forcelim={{best_params['forcelim']}}")
            print(f"  Error: {{np.rad2deg(best_result['avg_error']):.2f}}deg (limit: {{np.rad2deg(MAX_ACCEPTABLE_ERROR)}}deg)")
            print(f"  Saturation: {{best_result['saturation_pct']:.1f}}% (limit: {{MAX_SATURATION_PCT}}%)")
        print()
        print("[DIAGNOSIS] TRAJECTORY IS TOO AGGRESSIVE")
        print()
        print("The recorded motion is physically impossible for this robot:")
        print()
        print("Possible causes:")
        print("  1. Motion is too fast")
        print("     - Joints accelerate/decelerate too quickly")
        print("     - Physics simulation cannot keep up")
        print()
        print("  2. Joint angles exceed safe limits")
        print("     - Motion tries to move joints beyond physical range")
        print("     - Check joint limits in MJCF/URDF")
        print()
        print("  3. Model mass/inertia is too high")
        print("     - Heavy links require more force to move")
        print("     - Reduce link mass in the model")
        print()
        print("  4. Trajectory has sudden jumps")
        print("     - Not enough keyframes for smooth interpolation")
        print("     - Add more intermediate keyframes")
        print()
        print("Suggested solutions:")
        print("  - Re-record motion at 50-70% speed")
        print("  - Add more keyframes (smoother transitions)")
        print("  - Check and adjust joint limits")
        print("  - Reduce link mass/inertia in model editor")
        print("  - Use simpler, slower movements for testing")
        print()
        print("[WARNING] Parameter tuning CANNOT solve this issue.")
        print("    The trajectory itself must be modified.")
        print()
        print("="*70)
        return False

if __name__ == '__main__':
    import sys
    result = optimize_parameters()
    # result can be: True, False, 'launch_mode2', or ('launch_mode2', params_dict)
    if result == 'launch_mode2' or (isinstance(result, tuple) and result[0] == 'launch_mode2'):
        sys.exit(42)  # Special exit code to launch Mode 2
    elif result:
        sys.exit(0)
    else:
        sys.exit(1)
'''

    return f'''#!/usr/bin/env python3
"""
Quick validation of default motor parameters.
Tests if robot can track trajectory with current settings.
"""
import mujoco
'''


def generate_torque_launch_script(
    python_script_name: str, default_rec_idx: int = 0
) -> str:
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
# Pin matplotlib to <=3.7.3
echo "Installing dependencies (mujoco, matplotlib, numpy)..."
pip install mujoco "matplotlib<=3.7.3" "numpy<2" > /dev/null 2>&1

# 4. Interactive Mode Selection
echo "========================================"
echo "  MuJoCo Motion Analysis Tool"
echo "========================================"
echo ""
echo "Select Analysis Mode:"
echo ""
echo "0. Auto Parameter Optimization (NEW)"
echo "   - Finds optimal motor parameters automatically"
echo "   - Or diagnoses trajectory issues"
echo ""
echo "1. Joint Torque Visualization"
echo "   - Theoretical torque (inverse dynamics)"
echo ""
echo "2. Motor Sizing Validation"  
echo "   - Set motor parameters and validate"
echo ""
echo "3. Fingertip Sensor Forces"
echo "   - Contact force visualization"
echo ""
echo "4. Inverse-to-Forward Validation (NEW)"
echo "   - Use Mode 1 torques as motor limits"
echo "   - Test if physics can actually track trajectory"
echo ""
echo "========================================"
read -p "Enter choice [0/1/2/3/4]: " choice

MODE="inverse"
SCRIPT="replay_with_torque.py"

if [ "$choice" = "0" ]; then
    # Mode 0: Auto optimization
    echo "Running automatic parameter optimization..."
    python3 validate_motor_params.py
    exit_code=$?
    echo "Exit code: $exit_code"
    
    # Check for Mode 2 launch request (exit code 42 = launch Mode 2)
    if [ $exit_code -eq 42 ]; then
        echo ""
        echo "===================="
        echo "Launching Mode 2..."
        echo "===================="
        sleep 1
        python3 replay_motor_validation.py {default_rec_idx}
        exit 0
    elif [ $exit_code -eq 0 ]; then
        echo ""
        echo "Optimization complete."
        exit 0
    else
        echo "Optimization failed with exit code: $exit_code"
        exit $exit_code
    fi
elif [ "$choice" = "2" ]; then
    # Mode 2 uses separate script
    SCRIPT="replay_motor_validation.py"
    echo "Starting Motor Validation..."
    python3 $SCRIPT {default_rec_idx}
elif [ "$choice" = "3" ]; then
    # Mode 3 uses original script with sensors mode
    MODE="sensors"
    echo "Starting Sensor Analysis in mode: $MODE..."
    python3 $SCRIPT {default_rec_idx} --mode $MODE
elif [ "$choice" = "4" ]; then
    # Mode 4: Inverse-to-Forward validation
    SCRIPT="inverse_to_forward_validation.py"
    echo "Starting Inverse-to-Forward Validation..."
    python3 $SCRIPT {default_rec_idx}
else
    # Mode 1 (default) uses inverse dynamics
    echo "Starting Analysis in mode: $MODE..."
    python3 $SCRIPT {default_rec_idx} --mode $MODE
fi
"""


def generate_mujoco_torque_replay_script(model_filename: str) -> str:
    """Generates MuJoCo python script for Replay with Real-time Torque Visualization and Interactive Tuning."""
    return f"""
import time
import json
import os
import argparse
import sys
import csv
import mujoco
import mujoco.viewer
import numpy as np
import warnings
warnings.filterwarnings('ignore', category=UserWarning)

# Try importing matplotlib
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib.widgets import Slider, RadioButtons, CheckButtons, TextBox
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
    nsens = model.nsensor
    all_sensors = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i) for i in range(nsens)]
    grid_map = [[[None for _ in range(3)] for _ in range(7)] for _ in range(5)]
    
    for sname in all_sensors:
        if not sname: continue
        low = sname.lower()
        f_idx = -1
        for i, key in enumerate(FINGER_KEYS):
            if key == 'little' and ('little' in low or 'pinky' in low): f_idx = i; break
            elif key in low: f_idx = i; break
        
        if f_idx == -1: continue
        try:
            parts = sname.split('_')
            if len(parts) >= 2 and parts[-1].isdigit() and parts[-2].isdigit():
                r = int(parts[-2])
                c = int(parts[-1])
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
parser.add_argument("--mode", choices=["inverse", "sensors"], default="inverse", help="Analysis Mode")
args = parser.parse_args()

rec_idx = args.index
if rec_idx < 0 or rec_idx >= len(recs):
    rec_idx = 0
rec = recs[rec_idx]

# Map logic
joint_grid_map = detect_finger_joints(model)
sensor_grid_map = detect_finger_sensors(model)
data_source_names = sorted(joint_ids.keys()) if args.mode != 'sensors' else sorted(sensor_ids.keys())

# Visualization Limits
if args.mode == 'sensors':
    z_lim_default = (0, 5)
    log_file_name = "sensor_log.csv"
else:
    z_lim_default = (-20, 20)
    log_file_name = f"torque_log_{{args.mode}}.csv"

print(f"Loaded {{rec['name']}}. Mode: {{args.mode}}. Logging to {{log_file_name}}")
csv_file = open(log_file_name, 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['Time'] + data_source_names)

# --- Pre-calculate Trajectory (Interpolation) ---
duration = rec['duration'] / 1000.0 # seconds
if duration <= 0: duration = 1.0
# Pre-compute Grids
joint_grid_map = detect_finger_joints(model)
sensor_grid_map = detect_finger_sensors(model)

dt = model.opt.timestep
trajectory_times = np.arange(0, duration, dt)
n_steps = len(trajectory_times)

# Arrays
qpos_traj = np.zeros((n_steps, model.nq))
qvel_traj = np.zeros((n_steps, model.nv))
qacc_traj = np.zeros((n_steps, model.nv))

kf_times = [k['timestamp']/1000.0 for k in rec['keyframes']]
kf_joints = rec['keyframes']

# Fill Trajectory
for jname, jid in joint_ids.items():
    qadr = model.jnt_qposadr[jid]
    dof_adr = model.jnt_dofadr[jid]
    
    y_points = []
    first_val = kf_joints[0]['joints'].get(jname, 0.0)
    prev_val = first_val
    for kf in kf_joints:
        val = kf['joints'].get(jname, prev_val)
        y_points.append(val)
        prev_val = val
    
    q_interp = np.interp(trajectory_times, kf_times, y_points)
    qpos_traj[:, qadr] = q_interp
    qvel_traj[:, dof_adr] = np.gradient(q_interp, dt)
    qacc_traj[:, dof_adr] = np.gradient(qvel_traj[:, dof_adr], dt)

# Interactive Plot (Mode 1 and 3 only use simple visualization)
if HAS_MATPLOTLIB:
    plt.ion()
    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(111, projection='3d')

# Debug: Print environment info
print(f"DISPLAY: {{os.environ.get('DISPLAY', 'NOT SET')}}")
print(f"MUJOCO_GL: {{os.environ.get('MUJOCO_GL', 'NOT SET (using default)')}}")
print(f"MuJoCo version: {{mujoco.__version__ if hasattr(mujoco, '__version__') else 'unknown'}}")
print("Attempting to create viewer window...")

# Main Loop
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Starting simulation loop...")
        start_time = time.time()
        last_print = 0
        
        while viewer.is_running():
            now = time.time()
            elapsed = now - start_time
            if elapsed > duration:
                start_time = now
                elapsed = 0
                
            step_idx = int(elapsed / dt)
            if step_idx >= n_steps: step_idx = n_steps - 1
                
            # --- PHYSICS STEP ---
            if args.mode == 'inverse' or args.mode == 'sensors':
                # 1. INVERSE DYNAMICS / KINEMATIC Mode
                # "Mode 1" Logic: Forced State
                # This ensures EXACT trajectory following for Sensors Mode too.
                data.qpos[:model.nq] = qpos_traj[step_idx]
                data.qvel[:model.nv] = qvel_traj[step_idx]
                data.qacc[:model.nv] = qacc_traj[step_idx]
                mujoco.mj_inverse(model, data)
                
                # Note: For sensors to work in Inverse Mode, relying on overlap?
                # mj_inverse -> mj_collision -> mj_sensor?
                # Actually mj_inverse calculates forces. Sensors usually depend on FWD pipeline (mj_step).
                # If we just force qpos, we can call mj_fwdPosition -> mj_sensor?
                # But mj_inverse does checks.
                # To get valid sensor data (Contacts), we might need `mj_forward` AFTER setting qpos?
                # No, `mj_inverse` is full inverse.
                # Let's try `mujoco.mj_step` but OVERRIDING position?
                # No, that's unstable.
                # Best for Sensors is often: `data.qpos = target; mujoco.mj_forward(model, data)`
                # This ignores inertia but computes static forces/contacts at that pose.
                if args.mode == 'sensors':
                    # Force kinematic state, then compute sensors/contacts statically
                    mujoco.mj_forward(model, data)

            viewer.sync()

            
            # --- VISUALIZATION & LOGGING ---
            # Collect Data
            vals_to_plot = []
            
            if args.mode == 'inverse':
                # Plot qfrc_inverse (Actuator Force required)
                vals_to_plot = [data.qfrc_inverse[model.jnt_dofadr[joint_ids[n]]] for n in data_source_names]
                
                # Log for Mode 1 (inverse dynamics)
                row = [elapsed] + vals_to_plot
                writer.writerow(row)
                
            elif args.mode == 'sensors':
                # Plot Sensor Data
                vals_to_plot = [data.sensordata[sensor_ids[n]] for n in data_source_names]

                # Log
                row = [elapsed] + vals_to_plot
                writer.writerow(row)
            
            # Draw Plot (for ALL modes with matplotlib)
            if HAS_MATPLOTLIB and (now - last_print > 0.1):
                ax.clear()
                
                if args.mode == 'sensors':
                    ax.set_zlim(0, 5)
                    ax.set_title(f"Sensor Forces (N) T={{elapsed:.2f}}s")
                    # 3x7 Logic
                    dz_list = []
                    c_list = []
                    
                    for f in range(5):
                        for r in range(7):
                            for c in range(3):
                                sname = sensor_grid_map[f][r][c]
                                val = 0.0
                                if sname and sname in sensor_ids:
                                    idx_map = data_source_names.index(sname)
                                    val = vals_to_plot[idx_map]
                                dz_list.append(val)
                                c_list.append(plt.cm.viridis(val / 5.0))
                    
                    # Finger tick logic
                    x_flat_arr = []
                    y_flat_arr = []
                    for f in range(5):
                        for r in range(7):
                            for c in range(3):
                                x_flat_arr.append(f*4 + c)
                                y_flat_arr.append(r)
                                
                    ax.bar3d(x_flat_arr, y_flat_arr, np.zeros_like(x_flat_arr), 0.8, 0.8, dz_list, color=c_list, shade=True)
                    
                    # Sensor Graph Labels
                    tick_locs = []
                    for i in range(5):
                        tick_locs.append(i*4 + 1)
                    ax.set_xticks(tick_locs)
                    ax.set_xticklabels(FINGER_NAMES, fontsize=9)

                elif args.mode in ['inverse', 'forward']:
                    # Bar Plot for Torques (2D Grid: Finger vs Joint Rank)
                    ax.set_title(f"Joint Torques (Nm) T={{elapsed:.2f}}s")
                    ax.set_zlim(-50, 50)
                    
                    # Setup Grid
                    # X: Fingers (0..4) 'Thumb', 'Index' ...
                    # Y: Joints (0..2) '1st', '2nd', '3rd'
                    
                    x_bar = []
                    y_bar = []
                    dz_bar = []
                    c_bar = []
                    
                    for c in range(5):
                        for r in range(3):
                            jname = joint_grid_map[r][c]
                            val = 0.0
                            if jname and jname in joint_ids:
                                 # Find value
                                 if jname in data_source_names:
                                     idx_map = data_source_names.index(jname)
                                     val = vals_to_plot[idx_map]
                            
                            x_bar.append(c)
                            y_bar.append(r)
                            dz_bar.append(val)
                            
                            # Color Logic (Coolwarm)
                            # Map -50..50 to 0..1
                            norm_val = np.clip(val, -50, 50)
                            # Shift to 0..1: (val + 50) / 100
                            c_bar.append(plt.cm.coolwarm((norm_val + 50) / 100.0))

                    # Plot
                    # Use narrow bars
                    ax.bar3d(x_bar, y_bar, np.zeros_like(x_bar), 0.5, 0.5, dz_bar, color=c_bar, shade=True)
                    
                    # Labels
                    ax.set_xticks(np.arange(5) + 0.25)
                    ax.set_xticklabels(FINGER_NAMES, rotation=0, fontsize=9)
                    ax.set_yticks(np.arange(3) + 0.25)
                    ax.set_yticklabels(['1st', '2nd', '3rd'], rotation=0, fontsize=9)
                    
                try:
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                except: pass
                last_print = now

    print("Simulation completed successfully")
except Exception as e:
    import traceback
    print(f"ERROR: Failed to create or run viewer: {{e}}")
    print(f"Error type: {{type(e).__name__}}")
    traceback.print_exc()
    print("\\nTroubleshooting:")
    print("1. Check if X11 display is available: echo $DISPLAY")
    print("2. Try setting MUJOCO_GL environment: export MUJOCO_GL=glfw")
    print("3. Check MuJoCo installation: pip show mujoco")
    import sys
    sys.exit(1)

csv_file.close()
"""


def generate_mujoco_motor_validation_script(model_filename: str) -> str:
    """Generates Mode 2: Motor Sizing Validation Script with Global + Per-Joint Parameters"""
    return f"""
import time
import json
import os
import argparse
import csv
import mujoco
import mujoco.viewer
import numpy as np
import warnings
warnings.filterwarnings('ignore', category=UserWarning)

# Try importing matplotlib
try:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button
    HAS_MATPLOTLIB = True
except ImportError:
    print("Warning: matplotlib not found. Visualization will be disabled.")
    HAS_MATPLOTLIB = False

# --- Configuration ---
MODEL_XML = "{model_filename}"
RECORDINGS_JSON = "recordings.json"

# --- Load Model ---
if not os.path.exists(MODEL_XML):
    print(f"Error: Model file {{MODEL_XML}} not found.")
    exit(1)

print(f"Loading model: {{MODEL_XML}}")
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

# Maps
joint_ids = {{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): i for i in range(model.njnt)}}
actuator_ids = {{}}
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    if name and name.endswith("_act"):
        jname = name[:-4]
        actuator_ids[jname] = i

# --- Load Recordings ---
if not os.path.exists(RECORDINGS_JSON):
    print("recordings.json not found.")
    exit(1)
    
with open(RECORDINGS_JSON, "r") as f:
    recs = json.load(f)

# Parse Arguments
parser = argparse.ArgumentParser()
parser.add_argument("index", nargs="?", type=int, default=0, help="Index of recording")
args = parser.parse_args()

rec_idx = args.index
if rec_idx < 0 or rec_idx >= len(recs):
    rec_idx = 0
rec = recs[rec_idx]

print(f"Loaded {{rec['name']}}. Mode: Motor Validation")

# --- Motor Parameter Management ---
# Control parameters: applied globally to all actuators (controller tuning)
GLOBAL_CONTROL_PARAMS = {{
    'kp': 200.0,  # Moderate position gain
    'kv': 20.0,   # 10% damping ratio (kv = 0.1 × kp)
}}

# Motor specifications: can be different per joint (hardware characteristics)
GLOBAL_MOTOR_PARAMS = {{
    'gear': 50.0,  # REALISTIC gear ratio - allows fast movement (speed ∝ 1/gear)
    'forcelim': 300.0,  # Motor force limit (gear × ~6Nm motor = 300Nm)
    'ctrlrange_max': 10.0,  # Maximum velocity (rad/s or m/s)
    'armature': 0.001,
    'frictionloss': 0.1,
    'damping': 0.5
}}

# Per-joint motor parameter storage (overrides global motor specs if set)
per_joint_params = {{}}

motor_params_file = "motor_parameters.json"
if os.path.exists(motor_params_file):
    try:
        with open(motor_params_file, 'r') as f:
            loaded = json.load(f)
            if 'control' in loaded:
                GLOBAL_CONTROL_PARAMS.update(loaded['control'])
            if 'motor_global' in loaded:
                GLOBAL_MOTOR_PARAMS.update(loaded['motor_global'])
            if 'per_joint' in loaded:
                per_joint_params = loaded['per_joint']
        print(f"Loaded motor parameters from {{motor_params_file}}")
    except Exception as e:
        print(f"Warning: Could not load motor parameters: {{e}}")

def get_joint_motor_params(jname):
    \"\"\"Get effective motor parameters for a joint (per-joint override or global)\"\"\"
    if jname in per_joint_params:
        return per_joint_params[jname]
    return GLOBAL_MOTOR_PARAMS

def apply_control_params_all():
    \"\"\"Apply control parameters (kp, kv) to all actuators\"\"\"
    for jname in joint_ids.keys():
        if jname not in actuator_ids: 
            continue
        aid = actuator_ids[jname]
        
        model.actuator_gainprm[aid, 0] = GLOBAL_CONTROL_PARAMS['kp']
        model.actuator_biasprm[aid, 1] = -GLOBAL_CONTROL_PARAMS['kv']

def apply_motor_params_all():
    \"\"\"Apply motor specifications to all joints\"\"\"
    for jname in joint_ids.keys():
        if jname not in actuator_ids: 
            continue
        aid = actuator_ids[jname]
        jid = joint_ids[jname]
        
        params = get_joint_motor_params(jname)
        
        model.actuator_gear[aid, 0] = params['gear']
        model.actuator_forcerange[aid, :] = [-params['forcelim'], params['forcelim']]
        model.dof_armature[model.jnt_dofadr[jid]] = params['armature']
        model.dof_frictionloss[model.jnt_dofadr[jid]] = params['frictionloss']
        model.dof_damping[model.jnt_dofadr[jid]] = params.get('damping', 0.5)

def apply_motor_params_joint(jname):
    \"\"\"Apply motor parameters to a specific joint\"\"\"
    if jname not in actuator_ids: 
        return
    aid = actuator_ids[jname]
    jid = joint_ids[jname]
    
    params = get_joint_motor_params(jname)
    
    model.actuator_gear[aid, 0] = params['gear']
    model.actuator_forcerange[aid, :] = [-params['forcelim'], params['forcelim']]
    model.dof_armature[model.jnt_dofadr[jid]] = params['armature']
    model.dof_frictionloss[model.jnt_dofadr[jid]] = params['frictionloss']
    model.dof_damping[model.jnt_dofadr[jid]] = params.get('damping', 0.5)

def save_motor_params():
    \"\"\"Save current control and motor parameters to file\"\"\"
    with open(motor_params_file, 'w') as f:
        json.dump({{
            'control': GLOBAL_CONTROL_PARAMS,
            'motor_global': GLOBAL_MOTOR_PARAMS,
            'per_joint': per_joint_params
        }}, f, indent=2)
    print(f"Saved motor parameters to {{motor_params_file}}")

# Apply initial parameters
apply_control_params_all()
apply_motor_params_all()

# Debug: Print configuration
print("\\n=== Configuration ===\")
print(f"Control Params (global): kp={{GLOBAL_CONTROL_PARAMS['kp']:.1f}}, kv={{GLOBAL_CONTROL_PARAMS['kv']:.1f}}\")
print(f"Motor Specs (default): gear={{GLOBAL_MOTOR_PARAMS['gear']:.1f}}, forcelim={{GLOBAL_MOTOR_PARAMS['forcelim']:.1f}}\")
print(f"Total joints: {{len(joint_ids)}}, Total actuators: {{len(actuator_ids)}}\")
print("\\nFirst 3 joints:")
for jname in list(joint_ids.keys())[:3]:  # Print first 3 joints
    if jname in actuator_ids:
        aid = actuator_ids[jname]
        params = get_joint_motor_params(jname)
        print(f"  {{jname}}: gear={{params['gear']:.1f}}, forcelim={{params['forcelim']:.1f}}\")

# --- Pre-calculate Trajectory ---
duration = rec['duration'] / 1000.0
if duration <= 0: duration = 1.0

dt = model.opt.timestep
trajectory_times = np.arange(0, duration, dt)
n_steps = len(trajectory_times)

qpos_traj = np.zeros((n_steps, model.nq))
qvel_traj = np.zeros((n_steps, model.nv))
qacc_traj = np.zeros((n_steps, model.nv))

kf_times = [k['timestamp']/1000.0 for k in rec['keyframes']]
kf_joints = rec['keyframes']

# Track which joints are actually in the recording
joints_in_recording = set()
for kf in kf_joints:
    joints_in_recording.update(kf['joints'].keys())

print("\\n" + "="*70)
print("TRAJECTORY ANALYSIS")
print("="*70)
print(f"Total joints in model: {{len(joint_ids)}}")
print(f"Joints in recording: {{len(joints_in_recording)}}")
print(f"\\nRecorded joints: {{sorted(joints_in_recording)}}")

joints_not_in_recording = set(joint_ids.keys()) - joints_in_recording
if joints_not_in_recording:
    print(f"\\nJoints NOT in recording (should stay still): {{sorted(joints_not_in_recording)}}")

for jname, jid in joint_ids.items():
    qadr = model.jnt_qposadr[jid]
    dof_adr = model.jnt_dofadr[jid]
    
    # CRITICAL: Only interpolate joints that are in the recording!
    if jname in joints_in_recording:
        y_points = []
        first_val = kf_joints[0]['joints'].get(jname, 0.0)
        prev_val = first_val
        for kf in kf_joints:
            val = kf['joints'].get(jname, prev_val)
            y_points.append(val)
            prev_val = val
        
        q_interp = np.interp(trajectory_times, kf_times, y_points)
        qpos_traj[:, qadr] = q_interp
        qvel_traj[:, dof_adr] = np.gradient(q_interp, dt)
        qacc_traj[:, dof_adr] = np.gradient(qvel_traj[:, dof_adr], dt)
        
        # Log trajectory stats for recorded joints
        traj_min = np.min(qpos_traj[:, qadr])
        traj_max = np.max(qpos_traj[:, qadr])
        traj_range = traj_max - traj_min
        print(f"  {{jname}}: RECORDED - start={{first_val:.4f}}, range=[{{traj_min:.4f}}, {{traj_max:.4f}}], delta={{traj_range:.4f}}")
    else:
        # Joint not in recording: use model's default qpos (usually 0)
        default_qpos = model.qpos0[qadr] if hasattr(model, 'qpos0') else 0.0
        qpos_traj[:, qadr] = default_qpos
        qvel_traj[:, dof_adr] = 0.0
        qacc_traj[:, dof_adr] = 0.0
        print(f"  {{jname}}: NOT RECORDED - fixed at {{default_qpos:.4f}}")

print("="*70)

# Store initial qpos for reset
initial_qpos = qpos_traj[0, :].copy()

# --- Interactive Motor Parameter UI ---
if HAS_MATPLOTLIB:
    import matplotlib
    matplotlib.use('TkAgg')  # Use non-blocking backend
    plt.ion()
    fig = plt.figure(figsize=(18, 10))
    fig.canvas.manager.window.attributes('-topmost', False)  # Prevent window from staying on top
    print("\\nCreating UI window...")
    
    # Create layout: info (top), sliders (left), plots (right)
    gs = fig.add_gridspec(3, 2, height_ratios=[0.5, 2, 1.5], width_ratios=[1, 1.5], 
                          hspace=0.35, wspace=0.35, left=0.08, right=0.97, top=0.95, bottom=0.08)
    
    ax_info = fig.add_subplot(gs[0, :])
    ax_sliders_container = fig.add_subplot(gs[1, 0])
    ax_joint_selector_container = fig.add_subplot(gs[2, 0])
    ax_plot_tracking = fig.add_subplot(gs[1, 1])
    ax_plot_torque = fig.add_subplot(gs[2, 1])
    
    ax_info.axis('off')
    ax_sliders_container.axis('off')
    ax_joint_selector_container.axis('off')
    
    info_text = ax_info.text(0.5, 0.5, "Motor Validation", ha='center', va='center', fontsize=11)
    
    # State
    selected_joint = None
    current_mode = 'global'  # 'global' or 'per_joint'
    joint_page = 0
    joints_per_page = 8  # Reduced from 20 to prevent text overlap
    
    # Real-time data storage
    plot_history = {{
        'time': [],
        'tracking_error': [],
        'max_torque': [],
        'saturated_count': 0
    }}
    
    # Joint selector with pagination
    from matplotlib.widgets import RadioButtons, Button
    
    joint_list_full = ['[GLOBAL]'] + sorted(joint_ids.keys())
    total_pages = (len(joint_list_full) + joints_per_page - 1) // joints_per_page
    
    def get_current_page_joints():
        start = joint_page * joints_per_page
        end = min(start + joints_per_page, len(joint_list_full))
        return joint_list_full[start:end]
    
    # === GLOBAL CONTROL SETTINGS (top) ===
    control_specs = [
        ('kp', 'Control Kp (Gain)', 10, 1000, 200),  # Default 200
        ('kv', 'Control Kv (Damping)', 2, 100, 20),  # 10% damping
    ]
    
    control_sliders = {{}}
    y_start = 0.88
    for i, (key, label, vmin, vmax, vinit) in enumerate(control_specs):
        ax_slider = plt.axes([0.10, y_start - i*0.05, 0.28, 0.02])
        slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit, valstep=(vmax-vmin)/1000.0)
        control_sliders[key] = slider
    
    # Control apply button
    ax_btn_apply_control = plt.axes([0.10, y_start - 0.12, 0.28, 0.03])
    btn_apply_control = Button(ax_btn_apply_control, 'Apply Control to All')
    
    # === MOTOR SPECIFICATIONS (middle) ===
    # Physics: gear ↑ → torque ↑, speed ↓ (inverse relationship!)
    motor_specs = [
        ('gear', 'Motor Gear Ratio', 1, 200, 50),  # Default 50, max 200 (higher = SLOWER)
        ('forcelim', 'Motor Force Limit (Nm)', 50, 1000, 300),  # gear × 6Nm motor
        ('ctrlrange_max', 'Max Velocity (rad/s)', 0, 50, 10),
        ('armature', 'Motor Armature', 0, 0.01, 0.001),
        ('frictionloss', 'Motor Friction', 0, 1, 0.1),
        ('damping', 'Joint Damping', 0, 5, 0.5)
    ]
    
    motor_sliders = {{}}
    y_start = 0.68
    for i, (key, label, vmin, vmax, vinit) in enumerate(motor_specs):
        ax_slider = plt.axes([0.10, y_start - i*0.045, 0.28, 0.018])
        slider = Slider(ax_slider, label, vmin, vmax, valinit=vinit, valstep=(vmax-vmin)/1000.0)
        motor_sliders[key] = slider
    
    # Motor buttons (all in one row below motor sliders)
    button_y = 0.38
    button_height = 0.03
    button_width = 0.09
    button_spacing = 0.005
    
    ax_btn_apply = plt.axes([0.09, button_y, button_width, button_height])
    btn_apply = Button(ax_btn_apply, 'Apply')
    
    ax_btn_reset_joint = plt.axes([0.09 + button_width + button_spacing, button_y, button_width, button_height])
    btn_reset_joint = Button(ax_btn_reset_joint, 'Reset')
    
    ax_btn_restart = plt.axes([0.09 + 2*(button_width + button_spacing), button_y, button_width, button_height])
    btn_restart = Button(ax_btn_restart, 'Restart')
    
    ax_btn_save = plt.axes([0.09 + 3*(button_width + button_spacing), button_y, button_width, button_height])
    btn_save = Button(ax_btn_save, 'Save')
    
    # RadioButtons area - below buttons, wider and taller (increased from 0.23 to 0.28)
    ax_radio_joints = plt.axes([0.09, 0.08, 0.29, 0.28])
    radio_joints = RadioButtons(ax_radio_joints, get_current_page_joints(), activecolor='blue')
    
    # Pagination buttons (moved up to accommodate larger radio area)
    ax_btn_prev = plt.axes([0.09, 0.03, 0.08, 0.025])
    btn_prev = Button(ax_btn_prev, '< Prev')
    
    ax_page_text_ax = plt.axes([0.18, 0.03, 0.06, 0.025])
    ax_page_text_ax.axis('off')
    page_text = ax_page_text_ax.text(0.5, 0.5, f'{{joint_page+1}}/{{total_pages}}', ha='center', va='center', fontsize=9)
    
    ax_btn_next = plt.axes([0.25, 0.03, 0.08, 0.025])
    btn_next = Button(ax_btn_next, 'Next >')
    
    def update_control_sliders():
        \"\"\"Update control slider positions from global params\"\"\"
        for key in control_sliders:
            if key in GLOBAL_CONTROL_PARAMS:
                control_sliders[key].set_val(GLOBAL_CONTROL_PARAMS[key])
    
    def update_motor_sliders(params):
        \"\"\"Update motor slider positions from parameter dict\"\"\"
        for key in motor_sliders:
            if key in params:
                motor_sliders[key].set_val(params[key])
    
    def get_current_motor_values():
        \"\"\"Get current motor slider values as dict\"\"\"
        return {{key: motor_sliders[key].val for key in motor_sliders}}
    
    def update_radio_buttons():
        \"\"\"Recreate RadioButtons with current page joints\"\"\"
        global radio_joints, ax_radio_joints
        ax_radio_joints.clear()
        ax_radio_joints.set_position([0.09, 0.08, 0.29, 0.28])  # Match new size
        radio_joints = RadioButtons(ax_radio_joints, get_current_page_joints(), activecolor='blue')
        radio_joints.on_clicked(on_joint_select)
        page_text.set_text(f'{{joint_page+1}}/{{total_pages}}')
        fig.canvas.draw_idle()
    
    def on_prev_page(event):
        global joint_page
        if joint_page > 0:
            joint_page -= 1
            update_radio_buttons()
    
    def on_next_page(event):
        global joint_page
        if joint_page < total_pages - 1:
            joint_page += 1
            update_radio_buttons()
    
    def on_apply_control(event):
        \"\"\"Apply control parameters to all actuators\"\"\"
        GLOBAL_CONTROL_PARAMS['kp'] = control_sliders['kp'].val
        GLOBAL_CONTROL_PARAMS['kv'] = control_sliders['kv'].val
        apply_control_params_all()
        update_info_text()
    
    def on_joint_select(label):
        global selected_joint, current_mode
        if label == '[GLOBAL]':
            current_mode = 'global'
            selected_joint = None
            update_motor_sliders(GLOBAL_MOTOR_PARAMS)
        else:
            current_mode = 'per_joint'
            selected_joint = label
            params = get_joint_motor_params(label)
            update_motor_sliders(params)
        update_info_text()
    
    def on_apply(event):
        \"\"\"Apply motor parameters to selected joint or global\"\"\"
        params = get_current_motor_values()
        if current_mode == 'global':
            GLOBAL_MOTOR_PARAMS.update(params)
            apply_motor_params_all()
        else:
            per_joint_params[selected_joint] = params.copy()
            apply_motor_params_joint(selected_joint)
        update_info_text()
    
    def on_reset_joint(event):
        \"\"\"Reset selected joint to global motor specs\"\"\"
        if current_mode == 'per_joint' and selected_joint:
            if selected_joint in per_joint_params:
                del per_joint_params[selected_joint]
            apply_motor_params_joint(selected_joint)
            update_motor_sliders(GLOBAL_MOTOR_PARAMS)
            update_info_text()
    
    def on_restart(event):
        \"\"\"Restart replay from beginning - reset time and robot position\"\"\"
        global start_time
        start_time = time.time()
        
        # Reset robot to initial trajectory position
        data.qpos[:] = qpos_traj[0]
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        
        # Clear history for fresh plots
        plot_history['time'].clear()
        plot_history['tracking_error'].clear()
        plot_history['max_torque'].clear()
        plot_history['saturated_count'] = 0
        update_info_text()
        print(\"[RESTART] Replay restarted from t=0\")
    
    def on_save(event):
        save_motor_params()
        update_info_text()
    
    # Connect event handlers
    btn_apply_control.on_clicked(on_apply_control)
    radio_joints.on_clicked(on_joint_select)
    btn_apply.on_clicked(on_apply)
    btn_reset_joint.on_clicked(on_reset_joint)
    btn_restart.on_clicked(on_restart)
    btn_save.on_clicked(on_save)
    btn_prev.on_clicked(on_prev_page)
    btn_next.on_clicked(on_next_page)
    
    def update_info_text():
        mode_str = "GLOBAL Motor" if current_mode == 'global' else f"Joint: {{selected_joint}}"
        override_count = len(per_joint_params)
        total_joints = len(joint_ids)
        info_str = f"Motor Validation | Control: kp={{GLOBAL_CONTROL_PARAMS['kp']:.0f}} kv={{GLOBAL_CONTROL_PARAMS['kv']:.0f}} | "
        info_str += f"{{mode_str}} | Overrides: {{override_count}}/{{total_joints}} | "
        info_str += f"Time: {{elapsed:.2f}}s / {{duration:.2f}}s"
        if plot_history['saturated_count'] > 0:
            info_str += f" | [SAT] {{plot_history['saturated_count']}}"
        info_text.set_text(info_str)

    
    def update_plots():
        # Tracking Error Plot
        ax_plot_tracking.clear()
        if len(plot_history['time']) > 0:
            ax_plot_tracking.plot(plot_history['time'], plot_history['tracking_error'], 'b-', linewidth=2)
            ax_plot_tracking.set_xlabel('Time (s)')
            ax_plot_tracking.set_ylabel('RMS Tracking Error (rad)')
            ax_plot_tracking.set_title('Tracking Performance')
            ax_plot_tracking.grid(True, alpha=0.3)
            ax_plot_tracking.set_xlim(max(0, elapsed - 5), elapsed + 0.5)
        
        # Torque Plot
        ax_plot_torque.clear()
        if len(plot_history['time']) > 0:
            ax_plot_torque.plot(plot_history['time'], plot_history['max_torque'], 'r-', linewidth=2, label='Max Torque')
            ax_plot_torque.axhline(y=GLOBAL_MOTOR_PARAMS['forcelim'], color='orange', linestyle='--', label='Force Limit')
            ax_plot_torque.set_xlabel('Time (s)')
            ax_plot_torque.set_ylabel('Torque (Nm)')
            ax_plot_torque.set_title('Actuator Force')
            ax_plot_torque.legend(loc='upper right')
            ax_plot_torque.grid(True, alpha=0.3)
            ax_plot_torque.set_xlim(max(0, elapsed - 5), elapsed + 0.5)
            # Auto-scale Y axis but ensure it shows 0
            max_val = max(plot_history['max_torque']) if plot_history['max_torque'] else 10
            ax_plot_torque.set_ylim(0, max(max_val * 1.2, 10))
    
    # Initialize elapsed before UI setup
    elapsed = 0.0
    
    # Initialize UI - load global motor defaults into sliders
    update_control_sliders()
    on_joint_select('[GLOBAL]')
    
    print("Showing UI window...")
    plt.show(block=False)
    plt.pause(0.5)  # Longer pause to ensure full render
    fig.canvas.draw()
    fig.canvas.flush_events()
    print("UI window created successfully!")
    print(f"Figure has {{len(fig.axes)}} axes")
else:
    print("\\nMatplotlib not available - running without UI")

print("\\n=== Motor Validation Mode Started ===\")
print(f"Duration: {{duration:.2f}}s | Joints: {{len(joint_ids)}} | CSV: motor_validation_log.csv\")
print(f"Joints in recording: {{len(joints_in_recording)}} / {{len(joint_ids)}}\")
print("UI: Adjust Control (top), then Motor specs per joint (middle)\")
print("    Click 'Apply Control to All' to update controller globally\")
print("    Select joint, adjust motor specs, click 'Apply Motor' for that joint\")

# CRITICAL: Initialize qpos to first keyframe before simulation
data.qpos[:] = initial_qpos
data.qvel[:] = 0.0
data.qacc[:] = 0.0
data.ctrl[:] = 0.0
mujoco.mj_forward(model, data)
print(f"\\nRobot initialized to first keyframe\")

# --- Main Simulation Loop ---
log_file_name = "motor_validation_log.csv"
csv_file = open(log_file_name, 'w', newline='')
writer = csv.writer(csv_file)

# Extended CSV header with analysis data
header = ['Time']
for jname in joint_ids.keys():
    header.extend([
        f'{{jname}}_target_pos',
        f'{{jname}}_actual_pos',
        f'{{jname}}_error',
        f'{{jname}}_velocity',
        f'{{jname}}_torque',
        f'{{jname}}_ctrl_signal',
        f'{{jname}}_saturated'
    ])
header.extend(['RMS_tracking_error', 'Max_torque', 'Num_saturated'])
writer.writerow(header)

# Debug: Print environment info
print(f"DISPLAY: {{os.environ.get('DISPLAY', 'NOT SET')}}")
print(f"MUJOCO_GL: {{os.environ.get('MUJOCO_GL', 'NOT SET (using default)')}}")
print(f"MuJoCo version: {{mujoco.__version__ if hasattr(mujoco, '__version__') else 'unknown'}}")
print("Attempting to create viewer window...")

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Starting motor validation simulation...")
        start_time = time.time()
        last_print = 0
        last_plot_update = 0
        first_step_debug = True
        
        while viewer.is_running():
            now = time.time()
            elapsed = now - start_time
            if elapsed > duration:
                start_time = now
                elapsed = 0
                # Reset to initial position
                data.qpos[:] = initial_qpos
                data.qvel[:] = 0.0
                data.qacc[:] = 0.0
                data.ctrl[:] = 0.0
                mujoco.mj_forward(model, data)
                if HAS_MATPLOTLIB:
                    plot_history['time'].clear()
                    plot_history['tracking_error'].clear()
                    plot_history['max_torque'].clear()
                    plot_history['saturated_count'] = 0
                
            step_idx = int(elapsed / dt)
            if step_idx >= n_steps: 
                step_idx = n_steps - 1
            
            # Set control targets from trajectory
            # CRITICAL: Only control joints that are in the recording!
            for jname, jid in joint_ids.items():
                if jname in actuator_ids:
                    target_q = qpos_traj[step_idx, model.jnt_qposadr[jid]]
                    data.ctrl[actuator_ids[jname]] = target_q
                    # Note: Joints not in recording keep their qpos0 (handled in trajectory generation)
            
            # Step simulation
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # Runtime monitoring every 0.1 second (10Hz)
            if now - last_print >= 0.1:
                print(f"\\n[T={{elapsed:.2f}}s] Step {{step_idx}}/{{n_steps}}")
                
                # Show sample of moving joints (delta > 0.1 rad)
                moving_joints = []
                static_joints = []
                for jname, jid in joint_ids.items():
                    qadr = model.jnt_qposadr[jid]
                    target_pos = qpos_traj[step_idx, qadr]
                    actual_pos = data.qpos[qadr]
                    error = target_pos - actual_pos
                    
                    # Determine if joint should be moving based on trajectory
                    traj_range = np.max(qpos_traj[:, qadr]) - np.min(qpos_traj[:, qadr])
                    
                    if traj_range > 0.1:  # Should be moving (>5.7 degrees range)
                        moving_joints.append((jname, target_pos, actual_pos, error, traj_range))
                    elif abs(error) > 0.05:  # Should be static but has error
                        static_joints.append((jname, target_pos, actual_pos, error))
                
                # Show first 3 moving joints
                if moving_joints:
                    print("  Moving joints (should track trajectory):")
                    for jname, target, actual, error, traj_range in moving_joints[:3]:
                        if jname in actuator_ids:
                            aid = actuator_ids[jname]
                            ctrl = data.ctrl[aid]
                            torque = data.actuator_force[aid]
                            print(f"    {{jname}}: target={{target:.3f}}, actual={{actual:.3f}}, "
                                  f"error={{error:.3f}}, torque={{torque:.1f}}Nm (range={{traj_range:.3f}})")
                
                # Show problematic static joints
                if static_joints:
                    print("  ⚠️ Static joints with large error (should stay at 0):")
                    for jname, target, actual, error in static_joints[:3]:
                        if jname in actuator_ids:
                            aid = actuator_ids[jname]
                            ctrl = data.ctrl[aid]
                            torque = data.actuator_force[aid]
                            print(f"    {{jname}}: target={{target:.3f}}, actual={{actual:.3f}}, "
                                  f"error={{error:.3f}}, ctrl={{ctrl:.3f}}, torque={{torque:.1f}}Nm")
                
                last_print = now
            
            # Debug first step with detailed actuator info
            if first_step_debug:
                first_step_debug = False
                print("\\n=== First Step Debug (after mj_step) ===\")
                for jname in list(joint_ids.keys())[:3]:
                    if jname in actuator_ids:
                        aid = actuator_ids[jname]
                        jid = joint_ids[jname]
                        qadr = model.jnt_qposadr[jid]
                        target = data.ctrl[aid]
                        actual = data.qpos[qadr]
                        error = target - actual
                        force = data.actuator_force[aid]
                        print(f"  {{jname}}:")
                        print(f"    target={{target:.4f}}, actual={{actual:.4f}}, error={{error:.4f}}\")
                        print(f"    force={{force:.4f}}, kp={{model.actuator_gainprm[aid,0]:.1f}}\")
                        print(f"    expected_force = kp * error = {{model.actuator_gainprm[aid,0] * error:.4f}}\")
                print()
            
            # Collect analysis data (ALWAYS, for plots)
            tracking_errors = []
            torques = []
            saturated_count = 0
            row_data = [elapsed]
            
            for jname, jid in joint_ids.items():
                qadr = model.jnt_qposadr[jid]
                dof_adr = model.jnt_dofadr[jid]
                
                target_pos = qpos_traj[step_idx, qadr]
                actual_pos = data.qpos[qadr]
                error = target_pos - actual_pos
                velocity = data.qvel[dof_adr]
                
                tracking_errors.append(error ** 2)
                
                # Get actuator force/torque
                torque = 0.0
                ctrl_signal = 0.0
                is_saturated = 0
                if jname in actuator_ids:
                    aid = actuator_ids[jname]
                    ctrl_signal = data.ctrl[aid]
                    torque = data.actuator_force[aid]
                    torques.append(abs(torque))
                    
                    # Check saturation
                    if abs(torque) >= GLOBAL_MOTOR_PARAMS['forcelim'] * 0.95:
                        is_saturated = 1
                        saturated_count += 1
                
                row_data.extend([target_pos, actual_pos, error, velocity, torque, ctrl_signal, is_saturated])
            
            # Calculate metrics
            rms_error = np.sqrt(np.mean(tracking_errors)) if tracking_errors else 0.0
            max_torque = max(torques) if torques else 0.0
            
            row_data.extend([rms_error, max_torque, saturated_count])
            
            # Write CSV at 10Hz
            if now - last_print > 0.1:
                writer.writerow(row_data)
                last_print = now
            
            # Update plots at 5Hz (smoother without performance hit)
            if HAS_MATPLOTLIB and now - last_plot_update > 0.2:
                plot_history['time'].append(elapsed)
                plot_history['tracking_error'].append(rms_error)
                plot_history['max_torque'].append(max_torque)
                plot_history['saturated_count'] = saturated_count
                
                # Keep only last 500 data points for smooth plotting
                if len(plot_history['time']) > 500:
                    plot_history['time'] = plot_history['time'][-500:]
                    plot_history['tracking_error'] = plot_history['tracking_error'][-500:]
                    plot_history['max_torque'] = plot_history['max_torque'][-500:]
                
                update_info_text()
                update_plots()
                
                try:
                    fig.canvas.draw_idle()  # Use draw_idle instead of draw for better performance
                    fig.canvas.flush_events()
                    # Removed plt.pause() to prevent window from grabbing focus
                except Exception as e:
                    print(f"Warning: Plot update failed: {{e}}")
                
                last_plot_update = now

        print("Motor validation simulation completed successfully")
except Exception as e:
    import traceback
    print(f"ERROR: Failed to create or run viewer: {{e}}")
    print(f"Error type: {{type(e).__name__}}")
    traceback.print_exc()
    print("\\nTroubleshooting:")
    print("1. Check if X11 display is available: echo $DISPLAY")
    print("2. Try setting MUJOCO_GL environment: export MUJOCO_GL=glfw")
    print("3. Check MuJoCo installation: pip show mujoco")
    import sys
    sys.exit(1)

csv_file.close()
print(f"Motor validation complete. Log saved to {{log_file_name}}")
"""
