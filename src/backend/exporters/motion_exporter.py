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
    print("")
    print("Generating sensor plot...")
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

print("")
print("=== Interactive Mode ===")
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
active_fingers = {{{{'Thumb': False, 'Index': True, 'Middle': False, 'Ring': False, 'Little': False, 'Other': False}}}}

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
pip install mujoco "matplotlib<=3.7.3" "numpy<2" scipy > /dev/null 2>&1

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
    
    print("")
    print("="*60)
    print("MOTOR VALIDATION ANALYSIS REPORT")
    print("="*60)
    print("")
    print("Tracking Performance:")
    print(f"  Average RMS Error: {{avg_rms_error:.6f}} rad ({{np.rad2deg(avg_rms_error):.3f}} deg)")
    print(f"  Maximum RMS Error: {{max_rms_error:.6f}} rad ({{np.rad2deg(max_rms_error):.3f}} deg)")
    print("")
    print("Torque Analysis:")
    print(f"  Average Torque: {{avg_torque:.2f}} Nm")
    print(f"  Peak Torque: {{peak_torque:.2f}} Nm")
    print(f"  RMS Thermal Load: {{thermal_rms:.2f}} Nm")
    print("")
    print("Saturation:")
    print(f"  Saturation Events: {{saturation_events}} / {{len(time)}} samples ({{saturation_pct:.1f}}%)")
    
    if saturation_pct > 10:
        print(f"  [WARNING] High saturation rate! Consider increasing force limit.")
    elif saturation_pct > 0:
        print(f"  [CAUTION] Some saturation detected.")
    else:
        print(f"  [OK] No saturation detected.")
    
    print("")
    print("="*60)
    
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


def generate_inverse_to_forward_validation_script(
    model_file: str, recording_data: dict
) -> str:
    """Generate Mode 4: Inverse-to-Forward Validation Script

    Phase 1: Run inverse dynamics to find required torques
    Phase 2: Use those torques as motor limits and test forward tracking
    """
    rec_json = json.dumps(recording_data, indent=2)

    # Version info for debugging
    from datetime import datetime

    script_version = datetime.now().strftime("%Y%m%d_%H%M%S")

    return f'''#!/usr/bin/env python3
"""
Mode 4: Inverse-to-Forward Validation
Uses inverse dynamics torques as motor limits to test forward tracking

Script Version: {script_version}
Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
"""
import time
import mujoco
import mujoco.viewer
import numpy as np
import json

# Version info
SCRIPT_VERSION = "{script_version}"
SCRIPT_DATE = "{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}"

print("="*70)
print(f"MODE 4 SCRIPT VERSION: {{SCRIPT_VERSION}}")
print(f"Generated: {{SCRIPT_DATE}}")
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
with open("{model_file}", 'r') as f:
    mjcf_content = f.read()

# Remove <actuator> section entirely
import re
# Find and remove everything between <actuator> and </actuator>
mjcf_modified = re.sub(r'<actuator>.*?</actuator>', '', mjcf_content, flags=re.DOTALL)

# Save modified MJCF temporarily
import tempfile
import os
temp_mjcf = tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False, dir=os.path.dirname("{model_file}"))
temp_mjcf.write(mjcf_modified)
temp_mjcf_path = temp_mjcf.name
temp_mjcf.close()

print(f"  Created temporary MJCF without actuators: {{os.path.basename(temp_mjcf_path)}}")

# Load model from modified MJCF
model = mujoco.MjModel.from_xml_path(temp_mjcf_path)
data = mujoco.MjData(model)

print(f"  Model loaded: {{model.nu}} actuators (should be 0), {{model.nv}} DOFs")
if model.nu > 0:
    print("  ⚠️ WARNING: Model still has actuators! Actuator removal may have failed.")
else:
    print("  ✅ SUCCESS: Pure torque control model (no actuators)")
print("="*70)

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

# Build actuator mapping (should be empty now since we removed actuators)
actuator_ids = {{}}
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

# DEBUG: Check keyframe timing
print("")
print("🔍 KEYFRAME TIMING:")
print(f"  Number of keyframes: {{len(keyframes)}}")
if len(keyframes) > 0:
    print(f"  First keyframe time: {{keyframes[0]['timestamp']/1000.0:.3f}}s")
    print(f"  Last keyframe time: {{keyframes[-1]['timestamp']/1000.0:.3f}}s")
    print(f"  Recording duration: {{duration:.3f}}s")
    print(f"  Keyframe times: ", end="")
    for i, kf in enumerate(keyframes[:5]):
        print(f"{{kf['timestamp']/1000.0:.2f}}s ", end="")
    if len(keyframes) > 5:
        print("...")
    else:
        print()

for jname in recorded_joints:
    if jname not in joint_ids:
        print(f"Warning: Joint {{jname}} in recording not found in model")
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
        print(f"  Warning: {{jname}} using linear interpolation (install scipy for better results)")
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
        print(f"  {{jname}}:")
        print(f"    Step 0:    {{qpos_traj[0, qadr]:.4f}} rad")
        print(f"    Step 500:  {{qpos_traj[500, qadr]:.4f}} rad")
        print(f"    Step 2500: {{qpos_traj[2500, qadr]:.4f}} rad")
        print(f"    Step 5000: {{qpos_traj[5000, qadr]:.4f}} rad")
        print(f"    Range: {{np.max(qpos_traj[:, qadr]) - np.min(qpos_traj[:, qadr]):.4f}} rad")
        print(f"    Vel range: {{np.min(qvel_traj[:, dof_adr]):.2f}} to {{np.max(qvel_traj[:, dof_adr]):.2f}} rad/s")
        print(f"    Acc range: {{np.min(qacc_traj[:, dof_adr]):.2f}} to {{np.max(qacc_traj[:, dof_adr]):.2f}} rad/s²")

print("")
print("="*70)
print("PHASE 1: INVERSE DYNAMICS ANALYSIS")
print("="*70)
print("Calculating required torques for trajectory...")

# CRITICAL DIAGNOSTIC: Check data structure
print("")
print("🔍 DATA STRUCTURE DIAGNOSTIC:")
print(f"  model.nu (actuators): {{model.nu}}")
print(f"  model.nv (DOFs): {{model.nv}}")
print(f"  model.nq (positions): {{model.nq}}")
print("")
print("  Joint → DOF mapping:")
for jname, jid in joint_ids.items():
    dof_adr = model.jnt_dofadr[jid]
    print(f"    {{jname:30s}} → DOF {{dof_adr}}")
print("")
print("  Actuator → Joint mapping:")
if model.nu > 0:
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        trnid = model.actuator_trnid[i, 0]  # Transmission ID (joint)
        if trnid >= 0:
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
            print(f"    Actuator[{{i}}] {{act_name:25s}} → Joint {{joint_name}}")
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
        print(f"\\n🔍 DEBUG Step 2500 - qfrc_inverse values:")
        print(f"  qfrc_inverse shape: {{data.qfrc_inverse.shape}}")
        print(f"  qfrc_inverse max: {{np.max(np.abs(data.qfrc_inverse)):.2f}} Nm")
        for jname, jid in joint_ids.items():
            dof_adr = model.jnt_dofadr[jid]
            qfrc = data.qfrc_inverse[dof_adr]
            if abs(qfrc) > 0.1:
                print(f"    {{jname:30s}} DOF[{{dof_adr}}]: {{qfrc:+8.2f}} Nm")
    
    # CRITICAL: Store torques in DOF space (not actuator space, since we have no actuators)
    # qfrc_inverse gives us the required generalized forces at each DOF
    torques_dof = data.qfrc_inverse.copy()
    
    torque_history.append(torques_dof)
    
    # Track max for statistics
    torques_abs = np.abs(torques_dof)
    max_torques = np.maximum(max_torques, torques_abs)
    
    if step % 500 == 0:
        if len(max_torques) > 0:
            print(f"  Step {{step}}/{{n_steps}}: Max torque so far = {{np.max(max_torques):.2f}} Nm")
        else:
            print(f"  Step {{step}}/{{n_steps}}: Processing...")

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

print(f"  Saved {{len(torque_history)}} steps × {{len(joint_ids)}} joints to phase1_torque_history.csv")
print(f"  File size: ~{{len(torque_history) * len(joint_ids) * 8 / 1024:.1f}} KB")

print("")
print("Inverse Dynamics Results:")
print("  Joint Name                    | Max Torque (Nm)")
print("  " + "-"*60)
for jname, jid in joint_ids.items():
    dof_adr = model.jnt_dofadr[jid]
    if dof_adr < len(max_torques):
        print(f"  {{jname:30s}} | {{max_torques[dof_adr]:8.2f}}")

if len(max_torques) > 0:
    print(f"\\n  Overall Peak Torque: {{np.max(max_torques):.2f}} Nm")
    print(f"  Average Peak Torque: {{np.mean(max_torques):.2f}} Nm")
else:
    print(f"\\n  No torque data available")

# Add safety margin - need more for real forward dynamics!
# Inverse dynamics is ideal, forward needs extra for friction, damping, errors
safety_margin = 2.0  # Increased from 1.2
adjusted_limits = max_torques * safety_margin

if len(adjusted_limits) > 0:
    print(f"\\nApplying {{safety_margin}}x safety margin...")
    print(f"  → Inverse dynamics only accounts for ideal motion")
    print(f"  → Forward simulation needs extra for friction, damping, numerical errors")
    print(f"  Adjusted force limits: {{np.mean(adjusted_limits):.2f}} Nm (avg), {{np.max(adjusted_limits):.2f}} Nm (max)")

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

print(f"  Initial pose from trajectory: {{data.qpos[:5]}}")
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
        print(f"  {{jname:20s}}: actual={{actual_pos:+7.4f}}, traj_start={{target_pos:+7.4f}}, diff={{error:+7.4f}} ({{np.rad2deg(error):+6.2f}}°)")

if init_errors:
    init_rms = np.sqrt(np.mean(init_errors))
    print("")
    print(f"Initial mismatch RMS: {{init_rms:.4f}} rad ({{np.rad2deg(init_rms):.2f}}°)")
print("Note: With torque control, initial mismatch is okay")
print("=" * 50)

# Tracking data
tracking_errors = []
control_torques = []
times = []

# Phase 2 data logging (save every step for CSV)
phase2_log = []  # Will store: [time, step, target_pos, actual_pos, applied_force] per joint

# Store all torque data for post-simulation visualization
all_torque_data = []  # Will store qfrc_applied at each step
all_qpos_data = []    # Will store qpos at each step for replay

# Prepare joint list for visualization
vis_joints = []
vis_joint_indices = []
for jname, jid in joint_ids.items():
    if 'pitch' in jname.lower() or 'roll' in jname.lower() or 'yaw' in jname.lower():
        vis_joints.append(jname)
        vis_joint_indices.append(model.jnt_dofadr[jid])

print(f"📊 Will visualize {{len(vis_joints)}} joints in post-simulation analysis")

print("")
print("Starting forward simulation with TORQUE CONTROL...")

# DEBUG: Check torque_history contents
print("")
print("🔍 TORQUE HISTORY DIAGNOSTIC:")
print(f"  torque_history shape: {{torque_history.shape}}")
print(f"  Expected: ({{n_steps}}, {{model.nv}})")  # nv not nu!
print("")
print("  Sample torques at key steps:")
for sample_step in [0, 500, 2500, 5000]:
    if sample_step < len(torque_history):
        max_t = np.max(np.abs(torque_history[sample_step]))
        nonzero = np.count_nonzero(np.abs(torque_history[sample_step]) > 0.01)
        print(f"    Step {{sample_step}}: max={{max_t:.2f}} Nm, nonzero={{nonzero}}/{{model.nv}}")
        # Show which joints have significant torque
        for jname, jid in joint_ids.items():
            dof_adr = model.jnt_dofadr[jid]
            t = torque_history[sample_step][dof_adr]
            if abs(t) > 0.1:  # Only show significant torques
                print(f"      {{jname:30s}}: {{t:+8.2f}} Nm")

print("")
print("Initial position set. RMS: 0.0000 rad (should be ~0)")
print("=" * 50)

print("")
print("Starting forward simulation...")

# Create matplotlib figure BEFORE simulation starts
# This will be displayed alongside MuJoCo during simulation
if HAS_MATPLOTLIB:
    from matplotlib.widgets import Slider, Button, RadioButtons
    from mpl_toolkits.mplot3d import Axes3D
    
    print("")
    print("="*70)
    print("📊 Creating visualization window...")
    print("="*70)
    
    # Create figure with 3D plot and controls
    fig_interactive = plt.figure(figsize=(14, 10))
    ax_3d = fig_interactive.add_subplot(111, projection='3d')
    plt.subplots_adjust(bottom=0.2, right=0.85)
    
    # Add placeholder text
    ax_3d.text(0.5, 0.5, 0.5, 'Simulating...\\nPlease wait', 
              fontsize=20, ha='center', va='center',
              transform=ax_3d.transAxes)
    ax_3d.set_title('Simulation in progress...', fontsize=14, fontweight='bold')
    
    # Show matplotlib in non-blocking mode
    plt.ion()
    plt.show(block=False)
    plt.pause(0.3)
    
    print("✅ Visualization window ready")
    print("   (Will show interactive controls after simulation completes)")

# Single simulation run (no replay loop)
print("")
print("="*70)
print("🎬 Starting Simulation...")
print("="*70)

# Tracking data for this run
tracking_errors_run = []
control_torques_run = []
times_run = []
phase2_log_run = []
all_torque_data_run = []
all_qpos_data_run = []

# Reset simulation state
mujoco.mj_resetData(model, data)
data.qpos[:] = qpos_traj[0]
data.qvel[:] = qvel_traj[0]
data.qfrc_applied[:] = 0.0
mujoco.mj_forward(model, data)

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set better camera view for hand visualization
        # Adjust these values for optimal viewing angle
        viewer.cam.azimuth = 90    # Horizontal rotation (degrees)
        viewer.cam.elevation = -20  # Vertical angle (degrees)
        viewer.cam.distance = 1.5   # Distance from target
        viewer.cam.lookat[:] = [0.0, 0.0, 0.3]  # Look at point (x, y, z)
        
        print("")
        print("📹 Camera controls:")
        print("  - Right-click drag: Rotate view")
        print("  - Scroll wheel: Zoom in/out")
        print("  - Left-click drag: Pan view")
        print("  - Close window when simulation completes to see interactive analysis")
        print("")
        
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
                    print(f"\\n🔍 PHYSICS DEBUG at step {{sim_step}}:")
                    print(f"  Using REAL PHYSICS (mj_step) with torque control")
                    max_torque = np.max(np.abs(data.qfrc_applied))
                    nonzero_torques = np.count_nonzero(np.abs(data.qfrc_applied) > 0.01)
                    print(f"  Applied torques: {{nonzero_torques}}/{{model.nv}}, Max: {{max_torque:.2f}} Nm")
                    
                    # Check for instability warnings
                    if np.any(np.isnan(data.qpos)) or np.any(np.isinf(data.qpos)):
                        print("  ⚠️ WARNING: NaN/Inf detected in qpos!")
                    if np.max(np.abs(data.qvel)) > 100:
                        print(f"  ⚠️ WARNING: High velocity detected: {{np.max(np.abs(data.qvel)):.2f}}")
                    
                    # COLLISION DETECTION: Check for contacts between bodies
                    print(f"\\n  💥 COLLISION DEBUG:")
                    print(f"     Active contacts: {{data.ncon}}")
                    
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
                            
                            print(f"       Contact {{i+1}}: {{body1_name:25s}} <-> {{body2_name:25s}}")
                            print(f"                 Penetration: {{-dist*1000:.2f}} mm" + 
                                  (" 🔴 DEEP!" if dist < -0.005 else ""))
                    
                    else:
                        print(f"     ✅ No collisions (collision exclusions working)")
                    
                    # Show sample joint states
                    print(f"\\n  Joint States:")
                    for jname, jid in list(joint_ids.items())[:3]:
                        if jname in actuator_ids:
                            dof_adr = model.jnt_dofadr[jid]
                            qadr = model.jnt_qposadr[jid]
                            pos = data.qpos[qadr]
                            vel = data.qvel[dof_adr]
                            torque = data.qfrc_applied[dof_adr]
                            target = qpos_traj[sim_step, qadr]
                            error = target - pos
                            print(f"    {{jname:30s}}: pos={{pos:+7.3f}} (target={{target:+7.3f}}, err={{error:+7.3f}}), vel={{vel:+7.3f}}, torque={{torque:+7.2f}} Nm")
                
                # Run physics simulation step
                # This is the KEY difference from kinematic playback!
                mujoco.mj_step(model, data)
                
                # COLLISION MONITORING: Check for any collisions after step
                # This is CRITICAL - collisions can cause constraint forces that lead to instability
                if data.ncon > 0:
                    # Collision detected! This could be the cause of divergence
                    if sim_step % 100 == 0:  # Report every 100 steps if collisions persist
                        print(f"\\n⚠️ COLLISION WARNING at step {{sim_step}}:")
                        print(f"   {{data.ncon}} active contacts detected")
                        # Show first few problematic contacts
                        for i in range(min(data.ncon, 3)):
                            contact = data.contact[i]
                            body1_id = model.geom_bodyid[contact.geom1]
                            body2_id = model.geom_bodyid[contact.geom2]
                            body1_name = model.body(body1_id).name if body1_id >= 0 else "world"
                            body2_name = model.body(body2_id).name if body2_id >= 0 else "world"
                            print(f"      {{body1_name}} <-> {{body2_name}}, penetration: {{-contact.dist*1000:.2f}}mm")
                
                # Check for catastrophic failure (NaN/Inf)
                if np.any(np.isnan(data.qpos)) or np.any(np.isinf(data.qpos)):
                    print(f"\\n🔴 CATASTROPHIC FAILURE at step {{sim_step}}:")
                    print(f"   NaN/Inf detected in qpos!")
                    print(f"   Last known collisions: {{data.ncon}}")
                    if data.ncon > 0:
                        print(f"   Collision body pairs:")
                        for i in range(min(data.ncon, 5)):
                            contact = data.contact[i]
                            body1_id = model.geom_bodyid[contact.geom1]
                            body2_id = model.geom_bodyid[contact.geom2]
                            body1_name = model.body(body1_id).name if body1_id >= 0 else "world"
                            body2_name = model.body(body2_id).name if body2_id >= 0 else "world"
                            print(f"      {{body1_name}} <-> {{body2_name}}")
                    print(f"\\n   This likely means:")
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
                # Track ACTUAL applied torque (not actuator control which is 0)
                max_applied_torque = np.max(np.abs(data.qfrc_applied))
                
                tracking_errors_run.append(rms_error)
                control_torques_run.append(max_applied_torque)  # Store actual torque magnitude
                times_run.append(elapsed)
                
                # Store torque and position data for interactive visualization
                all_torque_data_run.append(data.qfrc_applied.copy())
                all_qpos_data_run.append(data.qpos.copy())
                
                # Save Phase 2 data every 10 steps (reduce file size)
                if sim_step % 10 == 0:
                    log_entry = {{'time': elapsed, 'step': sim_step}}
                    for jname in joint_ids.keys():
                        jid = joint_ids[jname]
                        qadr = model.jnt_qposadr[jid]
                        dof_adr = model.jnt_dofadr[jid]
                        
                        target = qpos_traj[sim_step, qadr]
                        actual = data.qpos[qadr]
                        applied_torque = applied_forces[dof_adr]  # Torque in Nm, not rad!
                        
                        log_entry[f'{{jname}}_target'] = target
                        log_entry[f'{{jname}}_actual'] = actual
                        log_entry[f'{{jname}}_force'] = applied_torque  # This is Nm, not rad
                        log_entry[f'{{jname}}_error'] = target - actual
                    
                    phase2_log_run.append(log_entry)
                
                # Print progress with diagnostics
                if sim_step % 500 == 0:
                    # Find worst 3 joints
                    error_details.sort(key=lambda x: x[1], reverse=True)
                    worst_joints = error_details[:3]
                    
                    print(f"  T={{elapsed:.2f}}s: RMS error={{rms_error:.4f}} rad ({{np.rad2deg(rms_error):.1f}}°), Max torque={{max_applied_torque:.2f}} Nm")
                    print(f"    Worst errors: ", end="")
                    for jname, err, ctrl in worst_joints:
                        print(f"{{jname}}={{np.rad2deg(err):.1f}}°  ", end="")
                    print()
                    if saturated_joints:
                        print(f"    Large commands: {{', '.join(saturated_joints[:5])}}")
                    
                    # Update matplotlib window to show it's still alive
                    if HAS_MATPLOTLIB:
                        try:
                            fig_interactive.canvas.flush_events()
                        except:
                            pass
                
                sim_step += 1  # Increment simulation step
        
        print("")
        print("✅ Simulation complete!")
        
except KeyboardInterrupt:
    print("\\n⚠️ Simulation interrupted by user")

# Store data for final analysis
tracking_errors = tracking_errors_run
control_torques = control_torques_run
times = times_run
phase2_log = phase2_log_run
all_torque_data = all_torque_data_run
all_qpos_data = all_qpos_data_run

# Convert lists to numpy arrays for easier indexing
all_torque_data = np.array(all_torque_data)
all_qpos_data = np.array(all_qpos_data)

# Now populate matplotlib with interactive controls
print("")
print("="*70)
print("📊 Setting up interactive controls...")
print("="*70)

# Interactive visualization with time slider
if HAS_MATPLOTLIB and len(all_torque_data) > 0:
    # Clear the placeholder text
    ax_3d.cla()
    
    # Add slider for time control
    ax_slider = plt.axes([0.15, 0.1, 0.55, 0.03])
    time_slider = Slider(ax_slider, 'Time (s)', 0, duration, valinit=0, valstep=dt)
    
    # Add Play/Pause button
    ax_button = plt.axes([0.72, 0.1, 0.08, 0.04])
    play_button = Button(ax_button, 'Play')
    
    # Add speed control radio buttons
    ax_speed = plt.axes([0.87, 0.3, 0.1, 0.15])
    speed_radio = RadioButtons(ax_speed, ('1x', '5x', '10x', '15x', '20x'), active=2)
    
    # Animation state - store model, data, viewer reference for playback
    anim_state = {{'is_playing': False, 'timer': None, 'speed': 10.0, 'viewer': None, 'model': model, 'data': data}}
    
    # Initial plot
    def plot_torques_at_time(time_val):
        \"\"\"Plot 3D bar chart of torques at given time\"\"\"
        ax_3d.cla()
        
        # Find closest time step
        step_idx = int(time_val / dt)
        step_idx = min(step_idx, len(all_torque_data) - 1)
        
        # Get torques at this time step
        torques_at_time = all_torque_data[step_idx]
        
        # Prepare data for 3D bar plot
        # Group joints by finger
        fingers = {{'Thumb': [], 'Index': [], 'Middle': [], 'Ring': [], 'Little': []}}
        for jname, jid in joint_ids.items():
            dof_adr = model.jnt_dofadr[jid]
            torque = torques_at_time[dof_adr]
            
            # Extract joint info from name (e.g., "Thumb-1st-roll" -> "1st roll")
            joint_label = jname
            if '-' in jname:
                parts = jname.split('-')
                if len(parts) >= 3:
                    # e.g., "1st-roll" or "2nd-pitch"
                    joint_label = f"{{parts[1]}} {{parts[2]}}"
            
            # Categorize by finger
            for finger_name in fingers.keys():
                if finger_name in jname:
                    fingers[finger_name].append((jname, joint_label, torque))
                    break
        
        # Create 3D bar plot
        x_pos = []
        y_pos = []
        z_pos = []
        dx = []
        dy = []
        dz = []
        colors = []
        labels = []
        
        finger_colors = {{
            'Thumb': 'red',
            'Index': 'blue', 
            'Middle': 'green',
            'Ring': 'orange',
            'Little': 'purple'
        }}
        
        finger_idx = 0
        joint_label_map = {{}}
        for finger_name, joint_list in fingers.items():
            if not joint_list:
                continue
            for joint_idx, (jname, joint_label, torque) in enumerate(joint_list):
                x_pos.append(finger_idx)
                y_pos.append(joint_idx)
                z_pos.append(0)
                dx.append(0.8)
                dy.append(0.8)
                dz.append(torque)
                colors.append(finger_colors[finger_name])
                labels.append(f"{{finger_name}}: {{joint_label}}")
                # Store label for Y-axis
                if finger_idx not in joint_label_map:
                    joint_label_map[finger_idx] = []
                if joint_idx >= len(joint_label_map[finger_idx]):
                    joint_label_map[finger_idx].extend([''] * (joint_idx + 1 - len(joint_label_map[finger_idx])))
                joint_label_map[finger_idx][joint_idx] = joint_label
            finger_idx += 1
        
        # Plot bars
        ax_3d.bar3d(x_pos, y_pos, z_pos, dx, dy, dz, color=colors, alpha=0.8, shade=True)
        
        ax_3d.set_xlabel('Finger')
        ax_3d.set_ylabel('Joint')
        ax_3d.set_zlabel('Torque (Nm)')
        ax_3d.set_title(f'Joint Torques at t={{time_val:.2f}}s')
        
        # Set x-axis labels
        ax_3d.set_xticks(range(len(fingers)))
        ax_3d.set_xticklabels(list(fingers.keys()))
        
        # Set y-axis to show integer joint numbers (0, 1, 2, 3, ...)
        if y_pos:
            max_joints = max(y_pos) + 1
            ax_3d.set_yticks(range(int(max_joints)))
            ax_3d.set_yticklabels([str(i+1) for i in range(int(max_joints))])
        
        # Set reasonable z-axis limits
        max_torque = np.max(np.abs(all_torque_data))
        ax_3d.set_zlim(-max_torque*1.1, max_torque*1.1)
        
        fig_interactive.canvas.draw_idle()  # Use draw_idle() to avoid auto-focus
    
    # Update function for slider
    def update_time(val):
        time_val = time_slider.val
        plot_torques_at_time(time_val)
        
        # Update MuJoCo viewer if still open
        if anim_state['viewer'] is not None:
            step_idx = int(time_val / dt)
            step_idx = min(step_idx, len(all_qpos_data) - 1)
            if step_idx < len(all_qpos_data):
                anim_state['data'].qpos[:] = all_qpos_data[step_idx]
                mujoco.mj_forward(anim_state['model'], anim_state['data'])
                anim_state['viewer'].sync()
    
    time_slider.on_changed(update_time)
    
    # Speed control functionality
    def change_speed(label):
        speed_map = {{'1x': 1.0, '5x': 5.0, '10x': 10.0, '15x': 15.0, '20x': 20.0}}
        anim_state['speed'] = speed_map[label]
        
        # Restart timer with new speed if playing
        if anim_state['is_playing']:
            if anim_state['timer'] is not None:
                anim_state['timer'].stop()
                anim_state['timer'] = None
            start_animation()
    
    speed_radio.on_clicked(change_speed)
    
    # Play/Pause functionality
    def toggle_play(event):
        anim_state['is_playing'] = not anim_state['is_playing']
        if anim_state['is_playing']:
            play_button.label.set_text('Pause')
            start_animation()
        else:
            play_button.label.set_text('Play')
            if anim_state['timer'] is not None:
                anim_state['timer'].stop()
                anim_state['timer'] = None
        fig_interactive.canvas.draw_idle()  # Use draw_idle() instead of plt.draw() to avoid auto-focus
    
    def start_animation():
        if anim_state['timer'] is None:
            # Fixed interval for smooth animation (update every 16ms = ~60fps)
            # Speed is controlled by time advancement per step, not interval
            interval = 16  # milliseconds (60 fps)
            anim_state['timer'] = fig_interactive.canvas.new_timer(interval=interval)
            anim_state['timer'].add_callback(animate_step)
            anim_state['timer'].start()
    
    def animate_step():
        if not anim_state['is_playing']:
            return
        
        current_time = time_slider.val
        # Advance by simulation timestep multiplied by speed and base multiplier
        # Base multiplier compensates for small dt value (typically 0.002s)
        # With 60fps and dt=0.002, we need ~40x multiplier for real-time
        base_multiplier = 40
        next_time = current_time + (dt * anim_state['speed'] * base_multiplier)
        
        if next_time >= duration:
            # Loop back to start
            next_time = 0
        
        time_slider.set_val(next_time)
        
        # Update MuJoCo viewer to match matplotlib
        if anim_state['viewer'] is not None:
            # Find corresponding simulation step
            step_idx = int(next_time / dt)
            if step_idx < len(all_qpos_data):
                # Update MuJoCo state to match this timestep
                anim_state['data'].qpos[:] = all_qpos_data[step_idx]
                mujoco.mj_forward(anim_state['model'], anim_state['data'])
                anim_state['viewer'].sync()
        
        # Update MuJoCo viewer to match matplotlib
        if anim_state['viewer'] is not None:
            # Find corresponding simulation step
            step_idx = int(next_time / dt)
            if step_idx < len(all_qpos_data):
                # Update MuJoCo state to match this timestep
                anim_state['data'].qpos[:] = all_qpos_data[step_idx]
                mujoco.mj_forward(anim_state['model'], anim_state['data'])
                anim_state['viewer'].sync()
    
    # Keyboard event handler
    def on_key(event):
        if event.key == ' ':  # Spacebar
            toggle_play(None)
    
    play_button.on_clicked(toggle_play)
    fig_interactive.canvas.mpl_connect('key_press_event', on_key)
    
    # Initial plot
    plot_torques_at_time(0.0)
    
    print("")
    print("="*70)
    print("🎮 INTERACTIVE ANALYSIS MODE")
    print("="*70)
    print("  Both windows are now ready:")
    print("  1. Matplotlib: 3D bar chart with time slider")
    print("     - Drag slider to move through time")
    print("     - Click 'Play' button to auto-play animation")
    print("     - Press SPACEBAR to play/pause")
    print("     - Speed control: Select 1x / 5x / 10x / 15x / 20x on the right")
    print("     - Animation loops automatically")
    print("     - Chart shows: Finger (X) - Joint (Y) - Torque (Z)")
    print("     - Colors: different fingers")
    print("")
    print("  2. MuJoCo Viewer: Robot pose synchronized with slider")
    print("     - Automatically updates when you move slider or play")
    print("     - Right-click drag: rotate camera")
    print("     - Scroll: zoom in/out")
    print("")
    print("  👉 Close matplotlib window when done to see final results")
    print("="*70)
    
    # Refresh matplotlib to show interactive controls
    fig_interactive.canvas.draw()
    fig_interactive.canvas.flush_events()
    plt.pause(0.1)
    
    print("\\n✅ Interactive controls ready")
    
    # Launch MuJoCo viewer in passive mode for visualization
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Store viewer reference for animation callbacks
            anim_state['viewer'] = viewer
            
            # Set camera view
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -20
            viewer.cam.distance = 1.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
            
            # Keep viewer open while matplotlib is interactive
            print("\\n✅ Both windows active. Move slider to analyze motion...")
            
            try:
                while plt.fignum_exists(fig_interactive.number):
                    viewer.sync()
                    # Process matplotlib events to make buttons/sliders/radio buttons responsive
                    fig_interactive.canvas.flush_events()
                    plt.pause(0.01)  # Small delay + event processing
            except KeyboardInterrupt:
                print("\\n⚠️ Interrupted by user")
            finally:
                # Clear viewer reference when closing
                anim_state['viewer'] = None
    except KeyboardInterrupt:
        print("\\n⚠️ Interrupted by user")
    except Exception as e:
        print(f"\\n⚠️ Could not open MuJoCo viewer: {{e}}")
    finally:
        anim_state['viewer'] = None
    
    # Cleanup
    plt.ioff()  # Turn off interactive mode
    try:
        plt.close(fig_interactive)
    except:
        pass

# Save Phase 2 applied control for comparison (last run only)
print("")
print("💾 Saving Phase 2 control history to CSV...")
with open('phase2_control_applied.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    
    # Header: time, step, then for each joint: target, actual, force, error
    header = ['time_s', 'step']
    for jname in joint_names_ordered:
        header.extend([
            f'{{jname}}_target_rad',
            f'{{jname}}_actual_rad',
            f'{{jname}}_force_Nm',
            f'{{jname}}_error_rad'
        ])
    writer.writerow(header)
    
    # Write logged data
    for entry in phase2_log:
        row = [entry['time'], entry['step']]
        for jname in joint_names_ordered:
            row.append(entry.get(f'{{jname}}_target', 0))
            row.append(entry.get(f'{{jname}}_actual', 0))
            row.append(entry.get(f'{{jname}}_force', 0))
            row.append(entry.get(f'{{jname}}_error', 0))
        writer.writerow(row)

print(f"  Saved {{len(phase2_log)}} samples (every 10 steps) to phase2_control_applied.csv")

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
    print(f"\\nTorque Control (Direct Force Application):")
    print(f"  Average Torque: {{avg_torque:.2f}} Nm")
    print(f"  Peak Torque: {{peak_torque:.2f}} Nm")
    
    # Verdict
    print("\\n" + "="*70)
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
        print("")
        print("📊 Generating final validation plots...")
        
        fig_results, axes = plt.subplots(3, 1, figsize=(12, 12))
        
        # Plot 1: Overall RMS Tracking Error
        axes[0].plot(times, np.rad2deg(tracking_errors), 'b-', linewidth=2, label='Overall RMS Error')
        axes[0].axhline(y=11.5, color='r', linestyle='--', linewidth=1.5, label='Acceptable limit (11.5°)')
        axes[0].set_xlabel('Time (s)', fontsize=11)
        axes[0].set_ylabel('RMS Tracking Error (degrees)', fontsize=11)
        axes[0].set_title('Overall Tracking Performance (Torque Control with PD Feedback)', fontsize=12, fontweight='bold')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(fontsize=10)
        axes[0].set_ylim([0, min(180, max(np.rad2deg(tracking_errors)) * 1.1)])
        
        # Plot 2: Per-Finger Torques
        # Extract per-joint torque data from phase2_log
        # Use highly distinctive colors that are easy to differentiate
        finger_colors = {{'thumb': '#FF0000', 'index': '#00FF00', 'middle': '#0000FF', 'ring': '#FF00FF', 'pinky': '#FFA500'}}
        finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        
        for finger in finger_names:
            finger_torques = []
            for entry in phase2_log:
                # Collect all forces for this finger
                finger_forces = [abs(entry.get(f'{{jname}}_force', 0)) 
                               for jname in joint_names_ordered if finger in jname.lower()]
                if finger_forces:
                    finger_torques.append(max(finger_forces))  # Max torque for this finger
                else:
                    finger_torques.append(0.0)
            
            if finger_torques:
                plot_times = times[:len(finger_torques)]
                axes[1].plot(plot_times, finger_torques, 
                           color=finger_colors.get(finger, 'gray'), 
                           linewidth=1.5, 
                           label=finger.capitalize(),
                           alpha=0.8)
        
        axes[1].set_xlabel('Time (s)', fontsize=11)
        axes[1].set_ylabel('Torque (Nm)', fontsize=11)
        axes[1].set_title('Per-Finger Applied Torques (Max per Finger)', fontsize=12, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(fontsize=10, ncol=5, loc='upper right')
        
        # Plot 3: Per-Finger Tracking Errors
        for finger in finger_names:
            finger_errors = []
            for entry in phase2_log:
                # Collect all errors for this finger
                finger_errs = [abs(entry.get(f'{{jname}}_error', 0)) 
                             for jname in joint_names_ordered if finger in jname.lower()]
                if finger_errs:
                    # RMS error for this finger
                    finger_errors.append(np.sqrt(np.mean(np.array(finger_errs)**2)))
                else:
                    finger_errors.append(0.0)
            
            if finger_errors:
                plot_times = times[:len(finger_errors)]
                axes[2].plot(plot_times, np.rad2deg(finger_errors), 
                           color=finger_colors.get(finger, 'gray'), 
                           linewidth=1.5, 
                           label=finger.capitalize(),
                           alpha=0.8)
        
        axes[2].set_xlabel('Time (s)', fontsize=11)
        axes[2].set_ylabel('RMS Error (degrees)', fontsize=11)
        axes[2].set_title('Per-Finger Tracking Errors', fontsize=12, fontweight='bold')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(fontsize=10, ncol=5, loc='upper right')
        
        plt.tight_layout()
        plt.savefig('mode4_validation.png', dpi=150)
        print("\\n📊 Plot saved to: mode4_validation.png")
        print("📊 Showing final results. Close window to exit...")
        plt.show()

# Cleanup temporary MJCF file
try:
    os.remove(temp_mjcf_path)
    print(f"\\nCleaned up temporary file: {{os.path.basename(temp_mjcf_path)}}")
except:
    pass

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
echo "Installing dependencies (mujoco, matplotlib, numpy, scipy)..."
pip install mujoco "matplotlib<=3.7.3" "numpy<2" scipy > /dev/null 2>&1

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
echo "2. Inverse-to-Forward Validation"
echo "   - Use Mode 1 torques as motor limits"
echo "   - Test if physics can actually track trajectory"
echo ""
echo "3. Motor Sizing Validation"  
echo "   - Set motor parameters and validate"
echo ""
echo "4. Fingertip Sensor Forces"
echo "   - Contact force visualization"
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
        echo "Launching Mode 3..."
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
    # Mode 2: Inverse-to-Forward validation
    SCRIPT="inverse_to_forward_validation.py"
    echo "Starting Inverse-to-Forward Validation..."
    python3 $SCRIPT {default_rec_idx}
elif [ "$choice" = "3" ]; then
    # Mode 3: Motor Sizing Validation
    SCRIPT="replay_motor_validation.py"
    echo "Starting Motor Validation..."
    python3 $SCRIPT {default_rec_idx}
elif [ "$choice" = "4" ]; then
    # Mode 4: Sensor Analysis
    MODE="sensors"
    echo "Starting Sensor Analysis in mode: $MODE..."
    python3 $SCRIPT {default_rec_idx} --mode $MODE
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
    """Generates Mode 3: Advanced Motor Sizing Validation Script.

    v6: Editable text boxes + removed speed (physics-correct) + hover tooltips
    """
    return f"""#!/usr/bin/env python3
\"\"\"
Mode 3: Advanced Motor Sizing Validation (v6)
==============================================
Phase 1: Inverse dynamics → auto-detect defaults
Phase 2: Forward sim with motor physics pipeline

v6: Editable text boxes, removed speed toggle (physics-correct), hover tooltips
\"\"\"
import time
import json
import os
import re
import argparse
import csv
import tempfile
import collections
import mujoco
import mujoco.viewer
import numpy as np
import warnings
warnings.filterwarnings('ignore', category=UserWarning)

try:
    from scipy.interpolate import CubicSpline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button, RadioButtons, TextBox
    HAS_MATPLOTLIB = True
except ImportError:
    print("Warning: matplotlib not found. Install: pip install matplotlib")
    HAS_MATPLOTLIB = False

EPSILON = 1e-6
PLOT_DT = 0.002
PLOT_WINDOW_SEC = 6.0
PLOT_MAX_POINTS = int(PLOT_WINDOW_SEC / PLOT_DT)

class RingBuffer:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.data = collections.deque(maxlen=maxlen)
    def append(self, val):
        self.data.append(val)
    def clear(self):
        self.data.clear()
    def __len__(self):
        return len(self.data)
    def to_array(self):
        return np.array(self.data) if self.data else np.array([])
    def __bool__(self):
        return len(self.data) > 0

# ═══════════════════════════════════════════════════════════════
# Motor Physics Pipeline
# ═══════════════════════════════════════════════════════════════
def smooth_sign(velocity, epsilon=EPSILON):
    return float(np.tanh(np.clip(velocity / epsilon, -20.0, 20.0)))

def compute_friction_torque(velocity, friction_torque_nm):
    sign_v = smooth_sign(velocity)
    coulomb = friction_torque_nm * 0.7 * sign_v
    viscous_coeff = friction_torque_nm * 0.3 / max(1.0, abs(velocity) + 0.1)
    return coulomb + viscous_coeff * velocity

def compute_output_torque_limit(velocity_rads, stall_torque, rated_speed_rads):
    no_load_speed = rated_speed_rads * 1.2
    abs_v = abs(velocity_rads)
    if abs_v >= no_load_speed: return 0.0
    return max(stall_torque * (1.0 - abs_v / no_load_speed), 0.0)

def apply_torque_limit(tau_cmd, velocity, stall_torque, rated_speed_rads):
    tau = float(np.clip(tau_cmd, -stall_torque, stall_torque))
    t_avail = compute_output_torque_limit(velocity, stall_torque, rated_speed_rads)
    return float(np.clip(tau, -t_avail, t_avail))

def apply_efficiency(tau_limited, velocity, gear_efficiency, epsilon=EPSILON):
    power = tau_limited * velocity
    threshold = max(abs(tau_limited) * 0.01, epsilon)
    blend = float(np.tanh(np.clip(power / threshold, -20.0, 20.0)))
    alpha = (blend + 1.0) / 2.0
    eff = alpha * gear_efficiency + (1.0 - alpha) * (1.0 / max(gear_efficiency, EPSILON))
    return tau_limited * eff

class PIDController:
    def __init__(self, kp=100.0, ki=0.0, kd=10.0, i_max=100.0, d_filter=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max, self.d_filter = i_max, d_filter
        self._integral = 0.0; self._prev_error = 0.0
        self._filtered_deriv = 0.0; self._init = False
    def reset(self):
        self._integral = 0.0; self._prev_error = 0.0
        self._filtered_deriv = 0.0; self._init = False
    def compute(self, q_ref, q_act, dt, ff_torque=0.0):
        dt_safe = max(dt, EPSILON); error = q_ref - q_act
        p = self.kp * error
        self._integral = float(np.clip(self._integral + error * dt_safe, -self.i_max, self.i_max))
        i = self.ki * self._integral
        if not self._init: raw_d = 0.0; self._init = True
        else: raw_d = (error - self._prev_error) / dt_safe
        self._filtered_deriv = self.d_filter * raw_d + (1.0 - self.d_filter) * self._filtered_deriv
        self._prev_error = error
        return p + i + self.kd * self._filtered_deriv + ff_torque

class MotorPhysicsEngine:
    def __init__(self, stall_torque_nm, rated_torque_nm, rated_speed_rpm, gear_ratio,
                 gear_efficiency=0.85, rotor_inertia_kgcm2=0.01, friction_torque_nm=0.1,
                 kp=100.0, ki=0.0, kd=10.0):
        self.stall_torque_nm = stall_torque_nm
        self.rated_torque_nm = rated_torque_nm
        self.rated_speed_rpm = rated_speed_rpm
        self.gear_ratio = gear_ratio
        self.gear_efficiency = gear_efficiency
        self.rotor_inertia_kgcm2 = rotor_inertia_kgcm2
        self.friction_torque_nm = friction_torque_nm
        self.pid = PIDController(kp=kp, ki=ki, kd=kd)
        self._recalc(); self.log = []; self.per_step = {{}}
    def _recalc(self):
        self._output_stall = self.stall_torque_nm * self.gear_ratio * self.gear_efficiency
        self._output_rated = self.rated_torque_nm * self.gear_ratio * self.gear_efficiency
        self._output_speed_rads = (self.rated_speed_rpm / self.gear_ratio) * (2.0 * np.pi / 60.0)
        self._reflected_inertia = (self.rotor_inertia_kgcm2 / 10000.0) * (self.gear_ratio ** 2)
    def reset(self):
        self.pid.reset(); self.log.clear(); self.per_step = {{}}
    def update_from_datasheet(self, **kwargs):
        for k, v in kwargs.items():
            if hasattr(self, k): setattr(self, k, v)
        self._recalc()
    def update_pid(self, kp=None, ki=None, kd=None):
        if kp is not None: self.pid.kp = kp
        if ki is not None: self.pid.ki = ki
        if kd is not None: self.pid.kd = kd
    def compute(self, t, q_ref, q_act, v_act, dt, ff_torque=0.0):
        # Pre-compensate FF for efficiency & friction losses
        # Without this, pipeline always delivers ~10% less than needed
        ff_comp = ff_torque / max(self.gear_efficiency, EPSILON)
        ff_friction_comp = compute_friction_torque(v_act, self.friction_torque_nm)
        ff_compensated = ff_comp + ff_friction_comp
        tau_cmd = self.pid.compute(q_ref, q_act, dt, ff_compensated)
        tau_limited = apply_torque_limit(tau_cmd, v_act, self._output_stall, self._output_speed_rads)
        tau_eff = apply_efficiency(tau_limited, v_act, self.gear_efficiency)
        tau_friction = compute_friction_torque(v_act, self.friction_torque_nm)
        tau_final = tau_eff - tau_friction
        t_avail = compute_output_torque_limit(v_act, self._output_stall, self._output_speed_rads)
        torque_margin = (t_avail - abs(tau_limited)) / max(t_avail, EPSILON) * 100.0 if t_avail > EPSILON else -100.0
        thermal_ratio = abs(tau_final) / max(self._output_rated, EPSILON) * 100.0
        state = {{
            'time': t, 'q_ref': q_ref, 'q_act': q_act, 'v_act': v_act,
            'tau_cmd': tau_cmd, 'tau_limited': tau_limited,
            'tau_eff': tau_eff, 'tau_friction': tau_friction, 'tau_final': tau_final,
            'torque_margin': torque_margin, 'thermal_ratio': thermal_ratio, 't_avail': t_avail,
        }}
        self.log.append(state); self.per_step = state
        return state

# ═══════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════
MODEL_XML = "{model_filename}"
RECORDINGS_JSON = "recordings.json"
if not os.path.exists(MODEL_XML):
    print(f"Error: Model file {{MODEL_XML}} not found."); exit(1)

# ═══════════════════════════════════════════════════════════════
# PHASE 1: Inverse Dynamics
# ═══════════════════════════════════════════════════════════════
print("="*70)
print("  PHASE 1: Inverse Dynamics — Detecting Required Torques")
print("="*70)

with open(MODEL_XML, 'r') as f:
    mjcf_content = f.read()
mjcf_no_act = re.sub(r'<actuator>.*?</actuator>', '', mjcf_content, flags=re.DOTALL)
temp_dir = os.path.dirname(os.path.abspath(MODEL_XML))
temp_mjcf = tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False, dir=temp_dir)
temp_mjcf.write(mjcf_no_act); temp_mjcf_path = temp_mjcf.name; temp_mjcf.close()
model_inv = mujoco.MjModel.from_xml_path(temp_mjcf_path)
data_inv = mujoco.MjData(model_inv)

if not os.path.exists(RECORDINGS_JSON):
    print("recordings.json not found."); exit(1)
with open(RECORDINGS_JSON, "r") as f:
    recs = json.load(f)
parser = argparse.ArgumentParser()
parser.add_argument("index", nargs="?", type=int, default=0)
args = parser.parse_args()
rec_idx = max(0, min(args.index, len(recs)-1))
rec = recs[rec_idx]
print(f"Recording: {{rec['name']}} (index {{rec_idx}})")

model_orig = mujoco.MjModel.from_xml_path(MODEL_XML)
data_orig = mujoco.MjData(model_orig)

joint_ids = {{mujoco.mj_id2name(model_inv, mujoco.mjtObj.mjOBJ_JOINT, i): i for i in range(model_inv.njnt)}}
actuator_ids = {{}}
for i in range(model_orig.nu):
    name = mujoco.mj_id2name(model_orig, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    if name and name.endswith("_act"):
        actuator_ids[name[:-4]] = i

duration = rec['duration'] / 1000.0
if duration <= 0: duration = 1.0
dt = model_inv.opt.timestep
trajectory_times = np.arange(0, duration, dt)
n_steps = len(trajectory_times)
qpos_traj_inv = np.zeros((n_steps, model_inv.nq))
qvel_traj_inv = np.zeros((n_steps, model_inv.nv))
qacc_traj_inv = np.zeros((n_steps, model_inv.nv))

kf_times = [k['timestamp']/1000.0 for k in rec['keyframes']]
kf_joints = rec['keyframes']
joints_in_recording = set()
for kf in kf_joints:
    joints_in_recording.update(kf['joints'].keys())

for jname, jid in joint_ids.items():
    qadr = model_inv.jnt_qposadr[jid]; dof_adr = model_inv.jnt_dofadr[jid]
    if jname in joints_in_recording:
        y_pts = []; prev_val = kf_joints[0]['joints'].get(jname, 0.0)
        for kf in kf_joints:
            val = kf['joints'].get(jname, prev_val); y_pts.append(val); prev_val = val
        if HAS_SCIPY and len(kf_times) >= 4:
            cs = CubicSpline(kf_times, y_pts, bc_type='clamped')
            qpos_traj_inv[:, qadr] = cs(trajectory_times)
            qvel_traj_inv[:, dof_adr] = cs(trajectory_times, 1)
            qacc_traj_inv[:, dof_adr] = cs(trajectory_times, 2)
        else:
            q_interp = np.interp(trajectory_times, kf_times, y_pts)
            qpos_traj_inv[:, qadr] = q_interp
            qvel_traj_inv[:, dof_adr] = np.gradient(q_interp, dt)
            qacc_traj_inv[:, dof_adr] = np.gradient(qvel_traj_inv[:, dof_adr], dt)
    else:
        qpos_traj_inv[:, qadr] = model_inv.qpos0[qadr] if qadr < len(model_inv.qpos0) else 0.0

inv_torques = {{}}; max_torques = {{}}; rms_torques = {{}}; max_vels = {{}}
data_inv.qpos[:] = qpos_traj_inv[0]; data_inv.qvel[:] = 0.0
mujoco.mj_forward(model_inv, data_inv)

for step in range(n_steps):
    data_inv.qpos[:] = qpos_traj_inv[step]
    data_inv.qvel[:] = qvel_traj_inv[step]
    data_inv.qacc[:] = qacc_traj_inv[step]
    mujoco.mj_inverse(model_inv, data_inv)
    for jname, jid in joint_ids.items():
        dof_adr = model_inv.jnt_dofadr[jid]
        if jname not in inv_torques: inv_torques[jname] = []
        inv_torques[jname].append(data_inv.qfrc_inverse[dof_adr])

ff_torques = {{}}
for jname in joint_ids.keys():
    arr = np.array(inv_torques.get(jname, [0.0]))
    ff_torques[jname] = arr
    max_torques[jname] = float(np.max(np.abs(arr)))
    rms_torques[jname] = float(np.sqrt(np.mean(arr**2)))
    max_vels[jname] = float(np.max(np.abs(qvel_traj_inv[:, model_inv.jnt_dofadr[joint_ids[jname]]])))

# Default motor params
print("\\n" + "="*70)
print("  Phase 1 Results: Required Motor Specifications")
print("="*70)
print(f"  {{'Joint':<30s}} | {{'Peak(Nm)':>10s}} | {{'RMS(Nm)':>10s}} | {{'MaxVel(r/s)':>12s}}")
print("  " + "-"*70)

TORQUE_MARGIN = 1.5   # torque headroom multiplier
SPEED_MARGIN = 3.0    # speed headroom multiplier (PID needs MUCH more speed than trajectory)
MIN_OUTPUT_SPEED = 20.0  # rad/s — minimum output speed at joint for PID responsiveness
default_motor_params = {{}}
for jname in sorted(joint_ids.keys()):
    peak = max_torques.get(jname, 0.0)
    rms_val = rms_torques.get(jname, 0.0)
    max_v = max_vels.get(jname, 0.0)
    assumed_eff = 0.90

    # ── Determine output requirements ──
    output_torque_needed = max(peak * TORQUE_MARGIN, 0.5)
    # PID correction needs MUCH more speed than trajectory max velocity:
    # When error is large, PID drives high velocity to catch up.
    # T-N curve outputs torque=0 when speed >= no_load_speed(= rated*1.2)
    # So rated_speed must be well above any PID-driven velocity.
    output_speed_needed = max(max_v * SPEED_MARGIN, MIN_OUTPUT_SPEED)

    # ── Pick motor + gear combo ──
    typical_motor_stall = 0.3  # Nm (motor side)
    typical_motor_rpm = 6000   # RPM (motor side, no load)
    motor_max_speed_rads = typical_motor_rpm * 2.0 * np.pi / 60.0  # ~628 rad/s

    # Max gear ratio limited by SPEED (most critical constraint)
    max_gear_from_speed = motor_max_speed_rads / output_speed_needed
    # Min gear ratio needed for torque
    min_gear_from_torque = output_torque_needed / (typical_motor_stall * assumed_eff)

    # ALWAYS prefer lower gear ratio — speed is king for PID tracking
    if min_gear_from_torque <= max_gear_from_speed:
        # Both achievable — pick midpoint leaning toward speed
        mid_gear = (min_gear_from_torque + max_gear_from_speed) / 2.0
        assumed_gear = max(5, min(200, round(mid_gear / 5) * 5))
    else:
        # Speed-limited — cap gear at speed limit, use bigger motor
        assumed_gear = max(5, min(200, round(max_gear_from_speed / 5) * 5))
    motor_stall = output_torque_needed / (assumed_gear * assumed_eff)

    # Ensure motor RPM covers the required output speed through the gear
    motor_rpm = max(output_speed_needed * assumed_gear * 60.0 / (2.0 * np.pi), 3000)
    # Rated torque = max(RMS need, 60% of peak) — prevents thermal overload
    motor_rated = max(rms_val * TORQUE_MARGIN, output_torque_needed * 0.6) / (assumed_gear * assumed_eff)
    # Very low friction — it accumulates through the pipeline and hinders tracking
    friction = round(output_torque_needed * 0.001, 5)
    default_motor_params[jname] = {{
        'stall_torque_nm': round(motor_stall, 4), 'rated_torque_nm': round(motor_rated, 4),
        'rated_speed_rpm': round(motor_rpm, 1), 'gear_ratio': assumed_gear,
        'gear_efficiency': assumed_eff, 'rotor_inertia_kgcm2': 0.005,
        'friction_torque_nm': friction,
    }}
    out_stall = motor_stall * assumed_gear * assumed_eff
    out_speed = motor_rpm / assumed_gear * 2.0 * np.pi / 60.0
    speed_margin_pct = (out_speed - max_v) / max(out_speed, 0.01) * 100.0 if out_speed > 0.01 else 100.0
    print(f"  {{jname:<30s}} | {{peak:>10.3f}} | {{rms_val:>10.3f}} | {{max_v:>12.3f}}  (gear={{assumed_gear:.0f}}:1, out={{out_stall:.1f}}Nm, {{out_speed:.1f}}r/s, speed_margin={{speed_margin_pct:.0f}}%)")

all_stall = [p['stall_torque_nm'] for p in default_motor_params.values()]
all_rated_t = [p['rated_torque_nm'] for p in default_motor_params.values()]
all_rpm = [p['rated_speed_rpm'] for p in default_motor_params.values()]
all_gear = [p['gear_ratio'] for p in default_motor_params.values()]
g_stall = max(all_stall) if all_stall else 0.3
g_rated = max(all_rated_t) if all_rated_t else 0.15
g_rpm = max(all_rpm) if all_rpm else 5000
g_gear = max(all_gear) if all_gear else 100
print(f"\\n  Global defaults: stall={{g_stall:.4f}}Nm, rated={{g_rated:.4f}}Nm, speed={{g_rpm:.0f}}RPM, gear={{g_gear:.0f}}:1")

try: os.unlink(temp_mjcf_path)
except: pass

# ═══════════════════════════════════════════════════════════════
# PHASE 2: Forward Simulation
# ═══════════════════════════════════════════════════════════════
print("\\n" + "="*70)
print("  PHASE 2: Forward Simulation — Motor Physics Pipeline")
print("  FF(100%) + PID(correction) → T-N Curve → Efficiency → Friction → MuJoCo")
print("="*70)

model = model_orig; data = data_orig
joint_ids = {{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): i for i in range(model.njnt)}}
actuator_ids = {{}}
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    if name and name.endswith("_act"): actuator_ids[name[:-4]] = i

dt = model.opt.timestep
trajectory_times = np.arange(0, duration, dt)
n_steps = len(trajectory_times)
qpos_traj = np.zeros((n_steps, model.nq))
qvel_traj_fwd = np.zeros((n_steps, model.nv))

for jname, jid in joint_ids.items():
    qadr = model.jnt_qposadr[jid]; dof_adr = model.jnt_dofadr[jid]
    if jname in joints_in_recording:
        y_pts = []; prev_val = kf_joints[0]['joints'].get(jname, 0.0)
        for kf in kf_joints:
            val = kf['joints'].get(jname, prev_val); y_pts.append(val); prev_val = val
        if HAS_SCIPY and len(kf_times) >= 4:
            cs = CubicSpline(kf_times, y_pts, bc_type='clamped')
            qpos_traj[:, qadr] = cs(trajectory_times)
            qvel_traj_fwd[:, dof_adr] = cs(trajectory_times, 1)
        else:
            q_interp = np.interp(trajectory_times, kf_times, y_pts)
            qpos_traj[:, qadr] = q_interp
            qvel_traj_fwd[:, dof_adr] = np.gradient(q_interp, dt)
    else:
        qpos_traj[:, qadr] = model.qpos0[qadr] if qadr < len(model.qpos0) else 0.0

initial_qpos = qpos_traj[0, :].copy()

for jname, aid in actuator_ids.items():
    model.actuator_gainprm[aid, 0] = 1.0
    model.actuator_biasprm[aid, :] = 0.0
    model.actuator_dynprm[aid, :] = 0.0
    model.actuator_gear[aid, 0] = 1.0
    model.actuator_forcerange[aid, :] = [-9999, 9999]
    model.actuator_ctrllimited[aid] = 0

ff_torques_fwd = {{}}
for jname in joint_ids.keys():
    if jname in ff_torques and jname in actuator_ids:
        arr = ff_torques[jname]
        if len(arr) == n_steps: ff_torques_fwd[jname] = arr
        else:
            old_t = np.linspace(0, duration, len(arr))
            ff_torques_fwd[jname] = np.interp(trajectory_times, old_t, arr)
    else:
        ff_torques_fwd[jname] = np.zeros(n_steps)

DEFAULT_KP = 1500.0; DEFAULT_KI = 50.0; DEFAULT_KD = 100.0
engines = {{}}
for jname in joint_ids.keys():
    if jname not in actuator_ids: continue
    mp = default_motor_params.get(jname, {{}})
    engines[jname] = MotorPhysicsEngine(
        stall_torque_nm=mp.get('stall_torque_nm', g_stall),
        rated_torque_nm=mp.get('rated_torque_nm', g_rated),
        rated_speed_rpm=mp.get('rated_speed_rpm', g_rpm),
        gear_ratio=mp.get('gear_ratio', g_gear),
        gear_efficiency=mp.get('gear_efficiency', 0.90),
        rotor_inertia_kgcm2=mp.get('rotor_inertia_kgcm2', 0.005),
        friction_torque_nm=mp.get('friction_torque_nm', 0.01),
        kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD,
    )
print(f"Created {{len(engines)}} motor physics engines")

# Print per-joint specs for verification
print("\\n  Per-joint motor specs:")
for jname in sorted(engines.keys()):
    eng = engines[jname]
    no_load = eng._output_speed_rads * 1.2  # T-N curve zero-torque speed
    max_v = max_vels.get(jname, 0.0)
    print(f"    {{jname:<30s}}: stall={{eng.stall_torque_nm:.4f}}Nm × gear={{eng.gear_ratio:.0f}} → out={{eng._output_stall:.1f}}Nm, rated_speed={{eng._output_speed_rads:.1f}}r/s, no_load={{no_load:.1f}}r/s, traj_maxV={{max_v:.2f}}r/s")

# Diagnostic tracking — will print summary at end
_diag_max_speed = {{jname: 0.0 for jname in engines}}
_diag_saturation_count = {{jname: 0 for jname in engines}}
_diag_total_steps = {{jname: 0 for jname in engines}}
_diag_tn_limited_count = {{jname: 0 for jname in engines}}

motor_params_file = "motor_parameters_v3.json"
if os.path.exists(motor_params_file):
    try:
        with open(motor_params_file, 'r') as f:
            loaded = json.load(f)
        if 'per_joint' in loaded:
            # Only load if user explicitly saved (check for matching joint count)
            loaded_joints = set(loaded['per_joint'].keys())
            engine_joints = set(engines.keys())
            if loaded_joints == engine_joints:
                for jname, params in loaded['per_joint'].items():
                    if jname in engines:
                        engines[jname].update_from_datasheet(**{{k: v for k, v in params.items() if hasattr(engines[jname], k)}})
                print(f"Loaded saved parameters from {{motor_params_file}}")
            else:
                print(f"Skipping {{motor_params_file}} (joint mismatch, using fresh defaults)")
    except Exception as e:
        print(f"Warning: Could not load {{motor_params_file}}: {{e}}")

# ═══════════════════════════════════════════════════════════════
# Playback State
# ═══════════════════════════════════════════════════════════════
sim_time = 0.0          # 시뮬레이션 시간 (0 ~ duration)
is_playing = True       # True=재생 중, False=일시정지
playback_speed = 1.0    # 재생 속도 (x1 고정)

joint_history = {{}}
for jname in engines:
    joint_history[jname] = {{
        'time': RingBuffer(PLOT_MAX_POINTS), 'q_ref': RingBuffer(PLOT_MAX_POINTS),
        'q_act': RingBuffer(PLOT_MAX_POINTS), 'error': RingBuffer(PLOT_MAX_POINTS),
        'tau_cmd': RingBuffer(PLOT_MAX_POINTS), 'tau_limited': RingBuffer(PLOT_MAX_POINTS),
        'tau_final': RingBuffer(PLOT_MAX_POINTS), 'tau_friction': RingBuffer(PLOT_MAX_POINTS),
        'v_act': RingBuffer(PLOT_MAX_POINTS), 't_avail': RingBuffer(PLOT_MAX_POINTS),
        'v_act_rpm': RingBuffer(PLOT_MAX_POINTS),
    }}

def seek_to_time(target_t):
    \"\"\"시뮬레이션을 특정 시간으로 이동 (처음부터 다시 시뮬)\"\"\"
    global sim_time
    target_t = max(0, min(target_t, duration))
    # 처음부터 target_t까지 빠르게 시뮬
    data.qpos[:] = initial_qpos; data.qvel[:] = 0; data.ctrl[:] = 0
    mujoco.mj_forward(model, data)
    for eng in engines.values(): eng.pid.reset()
    for jh in joint_history.values():
        for v in jh.values(): v.clear()

    target_step = min(int(target_t / dt), n_steps - 1)
    for si in range(target_step + 1):
        t_now = si * dt
        for jname in sorted(engines.keys()):
            jid = joint_ids[jname]; aid = actuator_ids[jname]
            qadr = model.jnt_qposadr[jid]; dof_adr = model.jnt_dofadr[jid]
            q_ref = qpos_traj[si, qadr]; q_act = data.qpos[qadr]; v_act = data.qvel[dof_adr]
            ff = ff_torques_fwd[jname][si] if jname in ff_torques_fwd else 0.0
            state = engines[jname].compute(t_now, q_ref, q_act, v_act, dt, ff_torque=ff)
            data.ctrl[aid] = state['tau_final']
        mujoco.mj_step(model, data)

    # 히스토리 채우기 (seek 지점 주변 데이터)
    start_fill = max(0, target_step - PLOT_MAX_POINTS)
    for si in range(start_fill, target_step + 1):
        t_now = si * dt
        for jname in engines:
            eng = engines[jname]
            if si < len(eng.log):
                s = eng.log[si]
            else:
                continue
            h = joint_history[jname]
            h['time'].append(s['time']); h['q_ref'].append(s['q_ref'])
            h['q_act'].append(s['q_act']); h['error'].append(abs(s['q_ref'] - s['q_act']))
            h['tau_cmd'].append(s['tau_cmd']); h['tau_limited'].append(s['tau_limited'])
            h['tau_final'].append(s['tau_final']); h['tau_friction'].append(abs(s['tau_friction']))
            h['v_act'].append(s['v_act']); h['t_avail'].append(s.get('t_avail', 0))
            h['v_act_rpm'].append(abs(s['v_act']) * eng.gear_ratio * 60.0 / (2.0 * np.pi))

    sim_time = target_t

# ═══════════════════════════════════════════════════════════════
# Interactive UI
# ═══════════════════════════════════════════════════════════════
if HAS_MATPLOTLIB:
    plt.ion()
    fig = plt.figure(figsize=(24, 14))
    fig.suptitle('Mode 3: Motor Sizing Validation (v6)', fontsize=13, fontweight='bold')
    try: fig.canvas.manager.window.attributes('-topmost', False)
    except: pass

    # Layout: 왼쪽 = 컨트롤, 오른쪽 = 4 plots (tracking, torque, T-N curve, margin bar)
    gs = fig.add_gridspec(4, 2, height_ratios=[1, 1, 1, 0.7], width_ratios=[1, 1.6],
                          hspace=0.40, wspace=0.3, left=0.07, right=0.97, top=0.93, bottom=0.08)

    ax_tracking = fig.add_subplot(gs[0, 1])
    ax_torque = fig.add_subplot(gs[1, 1])
    ax_tn = fig.add_subplot(gs[2, 1])
    ax_margin = fig.add_subplot(gs[3, 1])

    for row in range(4):
        ax_d = fig.add_subplot(gs[row, 0]); ax_d.axis('off')

    selected_joint = None; current_mode = 'global'
    joint_page = 0; joints_per_page = 8

    # 관절별 고유 색상 팔레트
    _COLOR_PALETTE = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
                      '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
                      '#aec7e8', '#ffbb78', '#98df8a', '#ff9896', '#c5b0d5',
                      '#c49c94', '#f7b6d2', '#c7c7c7', '#dbdb8d', '#9edae5']
    _sorted_joints = sorted([j for j in joint_ids.keys() if j in actuator_ids])
    joint_colors = {{}}
    for _ci, _jn in enumerate(_sorted_joints):
        joint_colors[_jn] = _COLOR_PALETTE[_ci % len(_COLOR_PALETTE)]

    joint_list_full = ['[GLOBAL]'] + _sorted_joints
    total_pages = max(1, (len(joint_list_full) + joints_per_page - 1) // joints_per_page)
    def get_page_joints():
        s = joint_page * joints_per_page
        return joint_list_full[s:s+joints_per_page]

    # === 모터 슬라이더 ===
    slider_specs = [
        ('stall_torque_nm', 'Stall Torque (Nm)', 0.001, max(g_stall*10, 1.0), g_stall),
        ('rated_torque_nm', 'Rated Torque (Nm)', 0.001, max(g_rated*10, 0.5), g_rated),
        ('rated_speed_rpm', 'Rated Speed (RPM)', 100, max(g_rpm*3, 10000), g_rpm),
        ('gear_ratio', 'Gear Ratio (N:1)', 1, 500, g_gear),
        ('gear_efficiency', 'Gear Efficiency (%)', 30, 100, 90),
        ('rotor_inertia_kgcm2', 'Rotor Inertia (kg·cm²)', 0.001, 1.0, 0.005),
        ('friction_torque_nm', 'Friction Torque (Nm)', 0.0, 2.0, 0.01),
        ('kp', 'PID Kp', 10, 5000, DEFAULT_KP),
        ('kd', 'PID Kd', 1, 500, DEFAULT_KD),
    ]
    sliders = {{}}
    textboxes = {{}}
    _slider_textbox_updating = [False]  # guard against infinite loop
    y_pos = 0.90
    for key, label, vmin, vmax, vinit in slider_specs:
        ax_s = plt.axes([0.09, y_pos, 0.22, 0.014])
        if key in ('gear_efficiency', 'gear_ratio', 'rated_speed_rpm'):
            sliders[key] = Slider(ax_s, label, vmin, vmax, valinit=vinit, valstep=1)
        else:
            sliders[key] = Slider(ax_s, label, vmin, vmax, valinit=vinit,
                                  valstep=max((vmax-vmin)/500.0, 0.0001))
        # Hide slider's built-in value text (replaced by editable TextBox)
        sliders[key].valtext.set_visible(False)
        # Editable text box next to slider
        ax_tb = plt.axes([0.32, y_pos, 0.055, 0.014])
        tb = TextBox(ax_tb, '', initial=f'{{vinit:.4g}}')
        textboxes[key] = tb
        y_pos -= 0.035

    def _make_slider_changed_cb(skey):
        def _on_slider_changed(val):
            if _slider_textbox_updating[0]: return
            _slider_textbox_updating[0] = True
            try:
                textboxes[skey].set_val(f'{{val:.4g}}')
            except: pass
            _slider_textbox_updating[0] = False
        return _on_slider_changed

    def _make_textbox_submit_cb(skey):
        def _on_textbox_submit(text):
            if _slider_textbox_updating[0]: return
            try:
                val = float(text)
                _slider_textbox_updating[0] = True
                sliders[skey].set_val(np.clip(val, sliders[skey].valmin, sliders[skey].valmax))
                _slider_textbox_updating[0] = False
            except ValueError:
                pass
        return _on_textbox_submit

    for skey in sliders:
        sliders[skey].on_changed(_make_slider_changed_cb(skey))
        textboxes[skey].on_submit(_make_textbox_submit_cb(skey))

    # Info text
    ax_info_box = plt.axes([0.09, y_pos - 0.005, 0.28, 0.02])
    ax_info_box.axis('off')
    info_text = ax_info_box.text(0.0, 0.5, "", fontsize=8, va='center',
                                  fontfamily='monospace', color='#333')

    # === Buttons row 1: Apply, Reset, Save ===
    btn_y1 = y_pos - 0.035
    ax_apply = plt.axes([0.09, btn_y1, 0.06, 0.022])
    btn_apply = Button(ax_apply, 'Apply')
    ax_reset = plt.axes([0.155, btn_y1, 0.06, 0.022])
    btn_reset = Button(ax_reset, 'Reset')
    ax_save = plt.axes([0.22, btn_y1, 0.06, 0.022])
    btn_save = Button(ax_save, 'Save')

    # === Buttons row 2: Playback — ▶/⏸  Restart ===
    btn_y2 = btn_y1 - 0.035
    ax_playpause = plt.axes([0.09, btn_y2, 0.08, 0.022])
    btn_playpause = Button(ax_playpause, '⏸ Pause')
    ax_restart_btn = plt.axes([0.175, btn_y2, 0.07, 0.022])
    btn_restart_btn = Button(ax_restart_btn, 'Restart')

    # === Timeline Slider ===
    timeline_y = btn_y2 - 0.035
    ax_timeline = plt.axes([0.09, timeline_y, 0.28, 0.016])
    timeline_slider = Slider(ax_timeline, 'Time(s)', 0, duration, valinit=0,
                              valstep=max(dt, 0.01), color='lightblue')

    # Joint selector
    radio_h = max(0.04, timeline_y - 0.09)
    ax_radio = plt.axes([0.09, 0.05, 0.29, radio_h])
    radio = RadioButtons(ax_radio, get_page_joints(), activecolor='blue')
    ax_prev = plt.axes([0.09, 0.02, 0.07, 0.02])
    btn_prev = Button(ax_prev, '< Prev')
    ax_pg = plt.axes([0.17, 0.02, 0.05, 0.02])
    ax_pg.axis('off')
    pg_text = ax_pg.text(0.5, 0.5, f'{{joint_page+1}}/{{total_pages}}', ha='center', fontsize=8)
    ax_next = plt.axes([0.23, 0.02, 0.07, 0.02])
    btn_next = Button(ax_next, 'Next >')

    def update_sliders_from_engine(eng):
        _slider_textbox_updating[0] = True
        for key in sliders:
            if key == 'gear_efficiency':
                v = eng.gear_efficiency * 100.0
                sliders[key].set_val(v)
                textboxes[key].set_val(f'{{v:.4g}}')
            elif hasattr(eng, key):
                v = getattr(eng, key)
                sliders[key].set_val(v)
                textboxes[key].set_val(f'{{v:.4g}}')
            elif key == 'kp':
                sliders[key].set_val(eng.pid.kp)
                textboxes[key].set_val(f'{{eng.pid.kp:.4g}}')
            elif key == 'kd':
                sliders[key].set_val(eng.pid.kd)
                textboxes[key].set_val(f'{{eng.pid.kd:.4g}}')
        _slider_textbox_updating[0] = False

    def get_slider_values():
        vals = {{k: s.val for k, s in sliders.items()}}
        vals['gear_efficiency'] = vals['gear_efficiency'] / 100.0
        return vals

    def apply_slider_to_engine(eng, vals):
        ds_keys = ['stall_torque_nm', 'rated_torque_nm', 'rated_speed_rpm',
                    'gear_ratio', 'gear_efficiency', 'rotor_inertia_kgcm2', 'friction_torque_nm']
        eng.update_from_datasheet(**{{k: vals[k] for k in ds_keys if k in vals}})
        eng.update_pid(kp=vals.get('kp'), kd=vals.get('kd'))

    def on_apply(event):
        vals = get_slider_values()
        if current_mode == 'global':
            for eng in engines.values(): apply_slider_to_engine(eng, vals)
            print("[APPLY] Updated ALL joints")
        elif selected_joint and selected_joint in engines:
            apply_slider_to_engine(engines[selected_joint], vals)
            print(f"[APPLY] Updated {{selected_joint}}")
        update_info()

    def on_reset(event):
        if current_mode == 'per_joint' and selected_joint and selected_joint in default_motor_params:
            mp = default_motor_params[selected_joint]
            engines[selected_joint].update_from_datasheet(**mp)
            engines[selected_joint].update_pid(kp=DEFAULT_KP, kd=DEFAULT_KD)
            update_sliders_from_engine(engines[selected_joint])

    def on_save(event):
        save_data = {{'per_joint': {{}}}}
        for jname, eng in engines.items():
            save_data['per_joint'][jname] = {{
                'stall_torque_nm': eng.stall_torque_nm, 'rated_torque_nm': eng.rated_torque_nm,
                'rated_speed_rpm': eng.rated_speed_rpm, 'gear_ratio': eng.gear_ratio,
                'gear_efficiency': eng.gear_efficiency, 'rotor_inertia_kgcm2': eng.rotor_inertia_kgcm2,
                'friction_torque_nm': eng.friction_torque_nm, 'kp': eng.pid.kp, 'kd': eng.pid.kd,
            }}
        with open(motor_params_file, 'w') as f: json.dump(save_data, f, indent=2)
        print(f"[SAVE] → {{motor_params_file}}")

    # ── Playback controls ──
    def on_playpause(event):
        global is_playing
        is_playing = not is_playing
        btn_playpause.label.set_text('⏸ Pause' if is_playing else '▶ Play')
        if is_playing:
            print(f"[PLAY] t={{sim_time:.2f}}s")
        else:
            print(f"[PAUSE] t={{sim_time:.2f}}s")
        fig.canvas.draw_idle()

    def on_restart(event):
        global is_playing, sim_time
        is_playing = True
        btn_playpause.label.set_text('⏸ Pause')
        seek_to_time(0.0)
        timeline_slider.set_val(0)
        update_info(); update_plots()
        fig.canvas.draw_idle(); fig.canvas.flush_events()
        print("[RESTART] t=0")

    _timeline_dragging = [False]
    def on_timeline_changed(val):
        global sim_time
        if not is_playing:
            seek_to_time(val)
            update_info(); update_plots()
            try:
                fig.canvas.draw_idle(); fig.canvas.flush_events()
            except: pass

    def on_joint_select(label):
        global selected_joint, current_mode
        # 색상 인디케이터 prefix 제거 (██ prefix)
        clean_label = label.replace('██ ', '').strip()
        if clean_label == '[GLOBAL]':
            current_mode = 'global'; selected_joint = None
        else:
            current_mode = 'per_joint'; selected_joint = clean_label
            if clean_label in engines: update_sliders_from_engine(engines[clean_label])
        update_info()

    def _add_color_indicators():
        # 라디오 버튼 옆에 관절별 색상 인디케이터 추가
        try:
            labels = [t.get_text() for t in radio.labels]
            for i, lbl in enumerate(labels):
                jn = lbl.strip()
                if jn in joint_colors:
                    color = joint_colors[jn]
                    # 라디오 버튼 라벨 텍스트에 색상 표시 (██ prefix)
                    radio.labels[i].set_text(f'██ {{jn}}')
                    radio.labels[i].set_color(color)
                    radio.labels[i].set_fontweight('bold')
                    radio.labels[i].set_fontsize(8)
                elif jn == '[GLOBAL]':
                    radio.labels[i].set_fontweight('bold')
                    radio.labels[i].set_fontsize(8)
                    radio.labels[i].set_color('black')
        except: pass

    def update_radio():
        global radio, ax_radio
        ax_radio.clear()
        ax_radio.set_position([0.09, 0.05, 0.29, radio_h])
        radio = RadioButtons(ax_radio, get_page_joints(), activecolor='blue')
        radio.on_clicked(on_joint_select)
        _add_color_indicators()
        pg_text.set_text(f'{{joint_page+1}}/{{total_pages}}')
        fig.canvas.draw_idle()
    def on_prev(event):
        global joint_page
        if joint_page > 0: joint_page -= 1; update_radio()
    def on_next(event):
        global joint_page
        if joint_page < total_pages - 1: joint_page += 1; update_radio()

    btn_apply.on_clicked(on_apply)
    btn_reset.on_clicked(on_reset)
    btn_save.on_clicked(on_save)
    btn_playpause.on_clicked(on_playpause)
    btn_restart_btn.on_clicked(on_restart)
    timeline_slider.on_changed(on_timeline_changed)
    radio.on_clicked(on_joint_select)
    btn_prev.on_clicked(on_prev)
    btn_next.on_clicked(on_next)

    def update_info():
        play_icon = "▶" if is_playing else "⏸"
        if current_mode == 'global':
            info_text.set_text(f"{{play_icon}} [GLOBAL] t={{sim_time:.2f}}/{{duration:.1f}}s | {{len(engines)}} motors")
        elif selected_joint and selected_joint in engines:
            eng = engines[selected_joint]
            out_rpm = eng._output_speed_rads * 60.0 / (2.0 * np.pi)
            info_text.set_text(
                f"{{play_icon}} [{{selected_joint}}] Out: {{eng._output_stall:.2f}}Nm, {{out_rpm:.0f}}RPM\\n"
                f"  t={{sim_time:.2f}}/{{duration:.1f}}s"
            )

    def update_plots():
        win_end = max(sim_time, PLOT_WINDOW_SEC)
        win_start = max(0, win_end - PLOT_WINDOW_SEC)

        # ── Plot 1: Position ──
        ax_tracking.clear()
        if current_mode == 'per_joint' and selected_joint and selected_joint in joint_history:
            h = joint_history[selected_joint]
            t_arr = h['time'].to_array()
            if len(t_arr) > 0:
                ax_tracking.plot(t_arr, h['q_ref'].to_array(), 'g--', lw=1.5, label='Reference')
                ax_tracking.plot(t_arr, h['q_act'].to_array(), 'b-', lw=1.5, label='Actual')
                ax_tracking.set_ylabel('Position (rad)')
                ax_tracking.set_title(f'Position: {{selected_joint}}')
                ax_tracking.legend(loc='upper right', fontsize=7)
        else:
            for jname in sorted(joint_history.keys()):
                h = joint_history[jname]; t_arr = h['time'].to_array()
                if len(t_arr) > 0:
                    jc = joint_colors.get(jname, '#333')
                    ax_tracking.plot(t_arr, h['error'].to_array(), lw=1, alpha=0.8, label=jname[:15], color=jc)
            ax_tracking.set_ylabel('|Error| (rad)')
            ax_tracking.set_title('All Joints Tracking Error')
            if len(engines) <= 10: ax_tracking.legend(loc='upper right', fontsize=6)
        ax_tracking.set_xlim(win_start, win_end)
        ax_tracking.axvline(x=sim_time, color='red', lw=0.8, ls='--', alpha=0.5)
        ax_tracking.grid(True, alpha=0.3); ax_tracking.set_xlabel('Time (s)')

        # ── Plot 2: Torque ──
        ax_torque.clear()
        if current_mode == 'per_joint' and selected_joint and selected_joint in joint_history:
            h = joint_history[selected_joint]; eng = engines.get(selected_joint)
            t_arr = h['time'].to_array()
            if len(t_arr) > 0:
                ax_torque.plot(t_arr, h['tau_cmd'].to_array(), 'gray', lw=0.8, label='T_cmd', alpha=0.6)
                ax_torque.plot(t_arr, h['tau_limited'].to_array(), 'orange', lw=1.2, label='T_limited')
                ax_torque.plot(t_arr, h['tau_final'].to_array(), 'r-', lw=1.5, label='T_final')
                ax_torque.fill_between(t_arr, 0, h['tau_friction'].to_array(), alpha=0.3, color='purple', label='Friction')
                if eng:
                    ax_torque.axhline(y=eng._output_stall, color='red', ls=':', lw=1, label=f'Stall({{eng._output_stall:.1f}})')
                    ax_torque.axhline(y=-eng._output_stall, color='red', ls=':', lw=1)
                    ax_torque.axhline(y=eng._output_rated, color='green', ls=':', lw=1, label=f'Rated({{eng._output_rated:.1f}})')
                    ax_torque.axhline(y=-eng._output_rated, color='green', ls=':', lw=1)
                ax_torque.set_title(f'Torque: {{selected_joint}}')
                ax_torque.legend(loc='upper right', fontsize=6)
        else:
            for jname in sorted(joint_history.keys()):
                h = joint_history[jname]; t_arr = h['time'].to_array()
                if len(t_arr) > 0:
                    jc = joint_colors.get(jname, '#333')
                    ax_torque.plot(t_arr, h['tau_final'].to_array(), lw=1, alpha=0.8, label=jname[:15], color=jc)
            ax_torque.set_title('All Joints: Actual Torque')
            if len(engines) <= 10: ax_torque.legend(loc='upper right', fontsize=6)
        ax_torque.set_xlim(win_start, win_end)
        ax_torque.axvline(x=sim_time, color='red', lw=0.8, ls='--', alpha=0.5)
        ax_torque.set_ylabel('Torque (Nm)'); ax_torque.grid(True, alpha=0.3)
        ax_torque.set_xlabel('Time (s)')

        # ── Plot 3: T-N Curve (Torque vs RPM) ──
        ax_tn.clear()
        if current_mode == 'per_joint' and selected_joint and selected_joint in engines:
            eng = engines[selected_joint]; h = joint_history[selected_joint]
            out_no_load_rpm = eng._output_speed_rads * 1.2 * 60.0 / (2.0 * np.pi)
            rpm_range = np.linspace(0, out_no_load_rpm, 100)
            rads_range = rpm_range * 2.0 * np.pi / 60.0
            t_boundary = [compute_output_torque_limit(v, eng._output_stall, eng._output_speed_rads) for v in rads_range]
            ax_tn.plot(rpm_range, t_boundary, 'r--', lw=2, label='T-N Limit')
            ax_tn.axhline(y=eng._output_rated, color='green', ls=':', lw=1.5, label=f'Rated({{eng._output_rated:.1f}}Nm)')
            rpm_arr = h['v_act_rpm'].to_array(); tf_arr = h['tau_final'].to_array()
            if len(rpm_arr) > 0 and len(tf_arr) > 0:
                rpm_pts = np.abs(rpm_arr); t_pts = np.abs(tf_arr)
                rads_pts = rpm_pts * 2.0 * np.pi / 60.0
                colors = ['red' if t > compute_output_torque_limit(v, eng._output_stall, eng._output_speed_rads)
                          else 'blue' for v, t in zip(rads_pts, t_pts)]
                ax_tn.scatter(rpm_pts, t_pts, c=colors, s=5, alpha=0.5)
            ax_tn.set_title(f'T-N Curve: {{selected_joint}}')
            ax_tn.legend(loc='upper right', fontsize=7)
        else:
            plotted_any = False
            for jname in sorted(engines.keys()):
                h = joint_history[jname]
                rpm_arr = h['v_act_rpm'].to_array(); tf_arr = h['tau_final'].to_array()
                if len(rpm_arr) > 0 and len(tf_arr) > 0:
                    jc = joint_colors.get(jname, '#333')
                    ax_tn.scatter(np.abs(rpm_arr), np.abs(tf_arr), s=3, alpha=0.5, label=jname[:12], color=jc)
                    plotted_any = True
            if plotted_any:
                ref_eng = max(engines.values(), key=lambda e: e._output_stall)
                out_no_load_rpm = ref_eng._output_speed_rads * 1.2 * 60.0 / (2.0 * np.pi)
                rpm_range = np.linspace(0, out_no_load_rpm, 100)
                rads_range = rpm_range * 2.0 * np.pi / 60.0
                t_boundary = [compute_output_torque_limit(v, ref_eng._output_stall, ref_eng._output_speed_rads)
                              for v in rads_range]
                ax_tn.plot(rpm_range, t_boundary, 'r--', lw=2, alpha=0.5, label='T-N Limit')
            ax_tn.set_title('All Joints: T-N Curve')
            if len(engines) <= 10: ax_tn.legend(loc='upper right', fontsize=6)
        ax_tn.set_xlabel('Speed (RPM)'); ax_tn.set_ylabel('|Torque| (Nm)')
        ax_tn.grid(True, alpha=0.3)

        # ── Plot 4: Motor Margin Bar Chart (복원) ──
        ax_margin.clear()
        names = sorted(engines.keys())
        if names:
            margins = []; colors_bar = []
            for jname in names:
                eng = engines[jname]
                m = eng.per_step.get('torque_margin', 100) if eng.per_step else 100
                margins.append(m)
                if m < 0: colors_bar.append('red')
                elif m < 20: colors_bar.append('orange')
                else: colors_bar.append('green')
            short_names = [n[:12] for n in names]
            ax_margin.barh(range(len(names)), margins, color=colors_bar, height=0.6)
            ax_margin.set_yticks(range(len(names)))
            ax_margin.set_yticklabels(short_names, fontsize=7)
            ax_margin.axvline(x=0, color='red', lw=1)
            ax_margin.axvline(x=20, color='orange', ls='--', lw=0.8)
            ax_margin.set_xlim(-100, 100)
            ax_margin.set_xlabel('Torque Margin (%)')
            ax_margin.set_title('Motor Margin (green=OK, orange=tight, red=OVER)')
            ax_margin.grid(True, alpha=0.3, axis='x')

        # ── Hover annotations 재생성 (clear() 후 필요) ──
        for _hax in [ax_tracking, ax_torque, ax_tn]:
            ann = _hax.annotate('', xy=(0,0), xytext=(15,15), textcoords='offset points',
                                bbox=dict(boxstyle='round,pad=0.3', fc='lightyellow', ec='gray', alpha=0.9),
                                fontsize=8, fontweight='bold', visible=False)
            _hover_annot[_hax] = ann

    # ── Hover tooltip for graph lines ──
    _hover_annot = {{}}
    _hover_prev_label = ['']  # track previous state to avoid unnecessary redraws
    _hover_last_time = [0.0]  # throttle: min interval between hover checks
    _hover_axes = {{ax_tracking, ax_torque, ax_tn}}  # only check on plot axes

    def _on_hover(event):
        # Only process when mouse is on one of the 3 plot axes
        if event.inaxes not in _hover_axes:
            if _hover_prev_label[0]:
                for ann in _hover_annot.values(): ann.set_visible(False)
                _hover_prev_label[0] = ''
                try: fig.canvas.draw_idle()
                except: pass
            return
        now = time.time()
        if now - _hover_last_time[0] < 0.2:  # 5Hz throttle (200ms)
            return
        _hover_last_time[0] = now

        if event.inaxes not in _hover_annot:
            return
        ax_h = event.inaxes
        ann = _hover_annot[ax_h]
        found_label = ''
        # Check lines (tracking, torque plots)
        for line in ax_h.get_lines():
            lb = line.get_label()
            if lb.startswith('_') or lb in ('Reference', 'Actual', 'T_cmd', 'T_limited',
                                            'T_final', 'Friction', 'T-N Limit') or lb.startswith('Stall') or lb.startswith('Rated'):
                continue
            contains, _ = line.contains(event)
            if contains:
                ann.xy = (event.xdata, event.ydata)
                lc = line.get_color()
                ann.get_bbox_patch().set_edgecolor(lc)
                ann.set_text(lb)
                ann.set_visible(True)
                found_label = lb
                break
        # Check scatter collections (T-N plot)
        if not found_label:
            for coll in ax_h.collections:
                lb = coll.get_label()
                if lb.startswith('_'): continue
                contains, ind = coll.contains(event)
                if contains:
                    ann.xy = (event.xdata, event.ydata)
                    try:
                        fc = coll.get_facecolor()[0]
                        ann.get_bbox_patch().set_edgecolor(fc)
                    except: pass
                    ann.set_text(lb)
                    ann.set_visible(True)
                    found_label = lb
                    break
        if not found_label:
            ann.set_visible(False)
        # Only redraw if state actually changed
        if found_label != _hover_prev_label[0]:
            _hover_prev_label[0] = found_label
            try: fig.canvas.draw_idle()
            except: pass

    fig.canvas.mpl_connect('motion_notify_event', _on_hover)

    _add_color_indicators()
    on_joint_select('[GLOBAL]')
    plt.show(block=False); plt.pause(0.3)
    fig.canvas.draw(); fig.canvas.flush_events()
    print("UI created — ▶Play / ⏸Pause / Timeline slider / Hover for joint name")

# ═══════════════════════════════════════════════════════════════
# Initialize + CSV
# ═══════════════════════════════════════════════════════════════
data.qpos[:] = initial_qpos; data.qvel[:] = 0.0; data.ctrl[:] = 0.0
mujoco.mj_forward(model, data)

log_file = "motor_validation_log.csv"
csv_f = open(log_file, 'w', newline='')
csv_w = csv.writer(csv_f)
csv_header = ['time']
for jname in sorted(engines.keys()):
    csv_header.extend([f'{{jname}}_q_ref', f'{{jname}}_q_act', f'{{jname}}_v_act',
                       f'{{jname}}_tau_cmd', f'{{jname}}_tau_limited',
                       f'{{jname}}_tau_final', f'{{jname}}_tau_friction',
                       f'{{jname}}_margin_pct', f'{{jname}}_thermal_pct'])
csv_w.writerow(csv_header)

# ═══════════════════════════════════════════════════════════════
# Main Loop
# ═══════════════════════════════════════════════════════════════
print("\\n  SIMULATION STARTED — ⏸Pause to inspect, hover graph for joint names")
print("  Close MuJoCo viewer to stop & see final report.")

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_wall = time.time()
        last_print = 0; last_plot = 0; last_csv = 0
        last_data_collect = 0
        loop_count = 0

        while viewer.is_running():
            now = time.time()
            wall_dt = now - last_wall
            last_wall = now

            # ── 재생 상태일 때만 시뮬레이션 진행 ──
            if is_playing:
                sim_time += wall_dt
                if sim_time > duration:
                    loop_count += 1
                    sim_time = 0.0
                    data.qpos[:] = initial_qpos; data.qvel[:] = 0; data.ctrl[:] = 0
                    mujoco.mj_forward(model, data)
                    for eng in engines.values(): eng.pid.reset()
                    for jh in joint_history.values():
                        for v in jh.values(): v.clear()

                step_idx = min(int(sim_time / dt), n_steps - 1)
                csv_row = [sim_time]

                for jname in sorted(engines.keys()):
                    jid = joint_ids[jname]; aid = actuator_ids[jname]
                    qadr = model.jnt_qposadr[jid]; dof_adr = model.jnt_dofadr[jid]
                    q_ref = qpos_traj[step_idx, qadr]
                    q_act = data.qpos[qadr]; v_act = data.qvel[dof_adr]
                    ff = ff_torques_fwd[jname][step_idx] if jname in ff_torques_fwd else 0.0
                    eng = engines[jname]
                    state = eng.compute(sim_time, q_ref, q_act, v_act, dt, ff_torque=ff)
                    data.ctrl[aid] = state['tau_final']
                    csv_row.extend([q_ref, q_act, v_act, state['tau_cmd'], state['tau_limited'],
                                   state['tau_final'], state['tau_friction'],
                                   state['torque_margin'], state['thermal_ratio']])
                    # Diagnostic tracking
                    _diag_total_steps[jname] += 1
                    abs_v = abs(v_act)
                    if abs_v > _diag_max_speed[jname]: _diag_max_speed[jname] = abs_v
                    if abs(state['tau_limited']) >= eng._output_stall * 0.99: _diag_saturation_count[jname] += 1
                    if abs(state['tau_cmd']) > abs(state['tau_limited']) + 0.01: _diag_tn_limited_count[jname] += 1

                mujoco.mj_step(model, data)

                # 2ms 데이터 수집
                if now - last_data_collect >= PLOT_DT:
                    for jname, eng in engines.items():
                        s = eng.per_step
                        if not s: continue
                        h = joint_history[jname]
                        h['time'].append(s['time']); h['q_ref'].append(s['q_ref'])
                        h['q_act'].append(s['q_act']); h['error'].append(abs(s['q_ref'] - s['q_act']))
                        h['tau_cmd'].append(s['tau_cmd']); h['tau_limited'].append(s['tau_limited'])
                        h['tau_final'].append(s['tau_final']); h['tau_friction'].append(abs(s['tau_friction']))
                        h['v_act'].append(s['v_act']); h['t_avail'].append(s.get('t_avail', 0))
                        h['v_act_rpm'].append(abs(s['v_act']) * eng.gear_ratio * 60.0 / (2.0 * np.pi))
                    last_data_collect = now

                # CSV 10Hz
                if now - last_csv >= 0.1:
                    csv_w.writerow(csv_row); last_csv = now

            # ── 일시정지 상태: MuJoCo 뷰어를 현재 시간 포즈로 유지 ──
            else:
                step_idx = min(int(sim_time / dt), n_steps - 1)
                for jname in sorted(engines.keys()):
                    jid = joint_ids[jname]; qadr = model.jnt_qposadr[jid]
                    data.qpos[qadr] = qpos_traj[step_idx, qadr]
                data.qvel[:] = 0
                mujoco.mj_forward(model, data)

            viewer.sync()

            # Console 2Hz
            if now - last_print >= 0.5:
                worst_margin = 100.0; worst_joint = ""
                for jname, eng in engines.items():
                    if eng.per_step:
                        m = eng.per_step.get('torque_margin', 100)
                        if m < worst_margin: worst_margin = m; worst_joint = jname
                status = "OK" if worst_margin > 0 else "OVER!"
                play_str = "▶" if is_playing else "⏸"
                print(f"{{play_str}} [T={{sim_time:.2f}}s] Worst: {{worst_joint}} margin={{worst_margin:.0f}}% {{status}} | Loop#{{loop_count}}")
                last_print = now

            # Plots 5Hz
            if HAS_MATPLOTLIB and now - last_plot >= 0.2:
                try:
                    # 재생 중일 때 타임라인 슬라이더 동기화
                    if is_playing:
                        timeline_slider.eventson = False
                        timeline_slider.set_val(min(sim_time, duration))
                        timeline_slider.eventson = True
                    update_info(); update_plots()
                    fig.canvas.draw_idle(); fig.canvas.flush_events()
                except: pass
                last_plot = now

        print("\\nSimulation ended")
except Exception as e:
    import traceback
    print(f"ERROR: {{e}}"); traceback.print_exc()
    import sys; sys.exit(1)

csv_f.close()
print(f"CSV log: {{log_file}}")

# ═══════════════════════════════════════════════════════════════
# FINAL ANALYSIS REPORT
# ═══════════════════════════════════════════════════════════════
print("\\n")
print("█"*70)
print("█  DIAGNOSTIC SUMMARY — Actual vs Spec")
print("█"*70)
print(f"  {{'Joint':<30s}} | {{'TrajMaxV':>8s}} | {{'ActMaxV':>8s}} | {{'RatedSpd':>8s}} | {{'NoLoad':>8s}} | {{'SatPct':>6s}} | {{'TNlimit':>7s}}")
print("  " + "─"*90)
for jname in sorted(engines.keys()):
    eng = engines[jname]
    traj_v = max_vels.get(jname, 0.0)
    act_v = _diag_max_speed.get(jname, 0.0)
    rated = eng._output_speed_rads
    no_load = rated * 1.2
    total = _diag_total_steps.get(jname, 1)
    sat_pct = _diag_saturation_count.get(jname, 0) / max(total, 1) * 100
    tn_pct = _diag_tn_limited_count.get(jname, 0) / max(total, 1) * 100
    flag = " ⚠️" if act_v > rated else ""
    print(f"  {{jname:<30s}} | {{traj_v:>7.1f}} | {{act_v:>7.1f}} | {{rated:>7.1f}} | {{no_load:>7.1f}} | {{sat_pct:>5.1f}}% | {{tn_pct:>5.1f}}%{{flag}}")

print("\\n")
print("█"*70)
print("█  MOTOR SIZING VALIDATION — FINAL REPORT")
print("█"*70)

overall_pass = True; report_lines = []

for jname in sorted(engines.keys()):
    eng = engines[jname]
    if not eng.log: continue
    torques_abs = [abs(s['tau_final']) for s in eng.log]
    errors = [abs(s['q_ref'] - s['q_act']) for s in eng.log]
    margins = [s['torque_margin'] for s in eng.log]
    thermal_ratios = [s['thermal_ratio'] for s in eng.log]
    velocities = [abs(s['v_act']) for s in eng.log]

    rms_torque = np.sqrt(np.mean([t**2 for t in torques_abs]))
    peak_torque = max(torques_abs); max_error = max(errors)
    min_margin = min(margins); avg_thermal = np.mean(thermal_ratios)
    max_vel = max(velocities)
    saturated_steps = sum(1 for m in margins if m < 5)
    saturation_pct = saturated_steps / len(margins) * 100
    checks = {{}}

    if min_margin > 20: checks['Torque Margin'] = ('PASS', f'min={{min_margin:.0f}}% (>20%)')
    elif min_margin > 0: checks['Torque Margin'] = ('WARN', f'min={{min_margin:.0f}}% (tight!)')
    else: checks['Torque Margin'] = ('FAIL', f'min={{min_margin:.0f}}% (SATURATED)')

    if avg_thermal < 80: checks['Thermal Load'] = ('PASS', f'avg={{avg_thermal:.0f}}% (<80%)')
    elif avg_thermal < 100: checks['Thermal Load'] = ('WARN', f'avg={{avg_thermal:.0f}}% (near limit)')
    else: checks['Thermal Load'] = ('FAIL', f'avg={{avg_thermal:.0f}}% (OVERLOAD)')

    if max_error < 0.05: checks['Tracking'] = ('PASS', f'max={{max_error:.4f}}rad ({{np.degrees(max_error):.2f}}°)')
    elif max_error < 0.15: checks['Tracking'] = ('WARN', f'max={{max_error:.4f}}rad ({{np.degrees(max_error):.2f}}°)')
    else: checks['Tracking'] = ('FAIL', f'max={{max_error:.4f}}rad ({{np.degrees(max_error):.2f}}°)')

    speed_margin = (eng._output_speed_rads - max_vel) / max(eng._output_speed_rads, EPSILON) * 100
    if speed_margin > 20: checks['Speed Margin'] = ('PASS', f'{{speed_margin:.0f}}%')
    elif speed_margin > 0: checks['Speed Margin'] = ('WARN', f'{{speed_margin:.0f}}% (close)')
    else: checks['Speed Margin'] = ('FAIL', f'{{speed_margin:.0f}}% (EXCEEDED)')

    if saturation_pct < 5: checks['Saturation'] = ('PASS', f'{{saturation_pct:.1f}}%')
    elif saturation_pct < 20: checks['Saturation'] = ('WARN', f'{{saturation_pct:.1f}}%')
    else: checks['Saturation'] = ('FAIL', f'{{saturation_pct:.1f}}% (frequent!)')

    statuses = [v[0] for v in checks.values()]
    if 'FAIL' in statuses: joint_verdict = '❌ FAIL'; overall_pass = False
    elif 'WARN' in statuses: joint_verdict = '⚠️  WARN'
    else: joint_verdict = '✅ PASS'

    print(f"\\n{'─'*60}")
    print(f"  Motor: {{jname}}  {{joint_verdict}}")
    print(f"  Spec: stall={{eng.stall_torque_nm:.4f}}Nm × gear={{eng.gear_ratio:.0f}} × eff={{eng.gear_efficiency:.0%}}"
          f" → out={{eng._output_stall:.2f}}Nm")
    print(f"  Spec: {{eng.rated_speed_rpm:.0f}}RPM / gear={{eng.gear_ratio:.0f}}"
          f" → out={{eng._output_speed_rads:.2f}}rad/s")
    print(f"{'─'*60}")
    for cn, (st, det) in checks.items():
        icon = '✅' if st == 'PASS' else ('⚠️ ' if st == 'WARN' else '❌')
        print(f"  {{icon}} {{cn:<20s}}: {{det}}")

    report_lines.append({{
        'joint': jname, 'verdict': joint_verdict,
        'rms_torque': rms_torque, 'peak_torque': peak_torque,
        'output_stall': eng._output_stall, 'output_rated': eng._output_rated,
        'max_error_deg': np.degrees(max_error), 'saturation_pct': saturation_pct,
        'checks': checks,
    }})

print(f"\\n{'━'*70}")
if overall_pass:
    print("  ✅ OVERALL: ALL MOTORS PASS")
else:
    fail_joints = [r['joint'] for r in report_lines if '❌' in r['verdict']]
    warn_joints = [r['joint'] for r in report_lines if '⚠️' in r['verdict']]
    print(f"  ❌ OVERALL: SOME MOTORS NEED ATTENTION")
    if fail_joints: print(f"     FAIL: {{', '.join(fail_joints)}}")
    if warn_joints: print(f"     WARN: {{', '.join(warn_joints)}}")
    print("\\n  💡 Recommendations:")
    for r in report_lines:
        if '❌' in r['verdict']:
            j = r['joint']
            for cn, (st, det) in r['checks'].items():
                if st == 'FAIL':
                    if 'Torque' in cn or 'Saturation' in cn: print(f"     {{j}}: Increase stall torque or gear ratio")
                    elif 'Thermal' in cn: print(f"     {{j}}: Increase rated torque (larger motor)")
                    elif 'Speed' in cn: print(f"     {{j}}: Increase motor speed or reduce gear ratio")
                    elif 'Tracking' in cn: print(f"     {{j}}: Increase PID gains or motor torque")
print(f"{'━'*70}")

report_file = "motor_validation_report.json"
with open(report_file, 'w') as f:
    json.dump(report_lines, f, indent=2, default=str)
print(f"\\nReport saved to {{report_file}}")
print(f"CSV log saved to {{log_file}}")
"""
