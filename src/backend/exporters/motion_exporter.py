
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
# Pin matplotlib to <=3.7.3
echo "Installing dependencies (mujoco, matplotlib, numpy)..."
pip install mujoco "matplotlib<=3.7.3" "numpy<2" > /dev/null 2>&1

# 4. Interactive Mode Selection
echo "----------------------------------------"
echo "Select Analysis Mode:"
echo "1. Inverse Dynamics (Theoretical Torque) [Default]"
echo "   - Calculates exact torque needed to follow trajectory."
echo "   - Ignores collisions (forced motion)."
echo "2. Forward Dynamics (Validation & Tuning)"
echo "   - Uses PID Control to track trajectory."
echo "   - Respects collisions (stops on contact)."
echo "   - Interactive Sliders to tune Kp/Kv."
echo "3. Fingertip Sensors (3x7 Grid)"
echo "----------------------------------------"
read -p "Enter choice [1]: " choice

MODE="inverse"
if [ "$choice" = "2" ]; then
    MODE="forward"
elif [ "$choice" = "3" ]; then
    MODE="sensors"
fi

# 5. Run the script
echo "Starting Analysis in mode: $MODE..."
python3 {python_script_name} {default_rec_idx} --mode $MODE
"""

def generate_mujoco_torque_replay_script(model_filename: str) -> str:
    """Generates MuJoCo python script for Replay with Real-time Torque Visualization and Interactive Tuning."""
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
    from matplotlib.widgets import Slider, RadioButtons, CheckButtons
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
parser.add_argument("--mode", choices=["inverse", "forward", "sensors"], default="inverse", help="Analysis Mode")
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

# Tunable Parameters (Forward Mode)
current_kp = 100.0 # Low initial stiffness to prevent start-up shock
current_kv = 10.0   # Moderate damping

# Global Scope for Sliders
scope_selection = 'All' # Default
scope_map = {{
    'All': [],
    'Thumb': ['thumb'],
    'Index': ['index'],
    'Middle': ['middle'],
    'Ring': ['ring'],
    'Little': ['little', 'pinky']
}}

def update_actuator_gains(kp, kv):
    # Determine target actuators based on scope
    # Iterating over actuator_ids (name -> id)
    # We want to match names that contain the scope keywords
    
    target_indices = []
    
    if scope_selection == 'All':
        target_indices = range(model.nu)
    else:
        # Filter
        keywords = scope_map.get(scope_selection, [])
        for name, aid in actuator_ids.items():
            name_low = name.lower()
            if any(k in name_low for k in keywords):
                target_indices.append(aid)
    
    # Apply
    # Note: target_indices is a list of INTs
    if target_indices:
        # We need to set them individually or via mask?
        # Creating a mask is safer for numpy indexing
        mask = np.zeros(model.nu, dtype=bool)
        for i in target_indices: mask[i] = True
        
        model.actuator_gainprm[mask, 0] = kp
        model.actuator_biasprm[mask, 1] = -kv

# Interactive Plot
if HAS_MATPLOTLIB and args.mode == 'forward':
    plt.ion()
    # Layout
    fig = plt.figure(figsize=(14, 10))
    plt.subplots_adjust(bottom=0.45) # Huge Space for Detail UI
    ax = fig.add_subplot(111, projection='3d')
    
    # UI Areas
    # Left: Radio for Scope
    ax_radio = plt.axes([0.02, 0.1, 0.12, 0.25])
    radio = RadioButtons(ax_radio, list(scope_map.keys()))

    # Right: Detail Slots for Selected Scope (Max 4 slots)
    # We create static axes for slots
    slots = []
    
    # Helper to clean labels
    def clean_jname(n):
        return n.replace('joint', '').replace('_', ' ').strip()
        
    class JointControl:
        def __init__(self, idx, y_pos):
            self.idx = idx
            self.ax_lbl = plt.axes([0.18, y_pos, 0.15, 0.03])
            self.ax_kp_box = plt.axes([0.35, y_pos, 0.1, 0.03])
            self.ax_kv_box = plt.axes([0.47, y_pos, 0.1, 0.03])
            self.ax_kp_slide = plt.axes([0.60, y_pos+0.015, 0.35, 0.015])
            self.ax_kv_slide = plt.axes([0.60, y_pos, 0.35, 0.015])
            
            self.box_kp = TextBox(self.ax_kp_box, 'Kp ', initial='0')
            self.box_kv = TextBox(self.ax_kv_box, 'Kv ', initial='0')
            self.slide_kp = Slider(self.ax_kp_slide, '', 0.1, 2000.0, valinit=100)
            self.slide_kv = Slider(self.ax_kv_slide, '', 0.1, 200.0, valinit=10)
            
            self.target_actuator_id = -1
            
            # Callbacks
            self.box_kp.on_submit(self.on_box_submit)
            self.box_kv.on_submit(self.on_box_submit)
            self.slide_kp.on_changed(self.on_slide_change)
            self.slide_kv.on_changed(self.on_slide_change)
            
            self.is_updating = False
            
            # Hide initially
            self.set_visible(False)

        def set_visible(self, vis):
            self.ax_lbl.set_visible(vis)
            self.ax_kp_box.ax.set_visible(vis)
            self.ax_kv_box.ax.set_visible(vis)
            self.ax_kp_slide.ax.set_visible(vis)
            self.ax_kv_slide.ax.set_visible(vis)
            if not vis:
                self.ax_lbl.axis('off')

        def set_target(self, name, aid):
            self.target_actuator_id = aid
            self.is_updating = True
            
            # Label
            self.ax_lbl.clear()
            self.ax_lbl.text(0, 0, clean_jname(name), fontsize=9)
            self.ax_lbl.axis('off')
            
            # Read current values
            curr_kp = model.actuator_gainprm[aid, 0]
            curr_kv = -model.actuator_biasprm[aid, 1]
            
            self.box_kp.set_val(str(curr_kp))
            self.box_kv.set_val(str(curr_kv))
            self.slide_kp.set_val(curr_kp)
            self.slide_kv.set_val(curr_kv)
            
            self.set_visible(True)
            self.is_updating = False

        def on_box_submit(self, val):
            if self.target_actuator_id == -1 or self.is_updating: return
            try:
                v = float(val)
                self.is_updating = True
                # Update model
                # Check which box triggered? Hard to tell, assuming we just re-read both or update specific
                # TextBox callback passes val.
                # Just update both from box values to be safe
                kp = float(self.box_kp.text)
                kv = float(self.box_kv.text)
                
                model.actuator_gainprm[self.target_actuator_id, 0] = kp
                model.actuator_biasprm[self.target_actuator_id, 1] = -kv
                
                # Sync sliders
                self.slide_kp.set_val(kp)
                self.slide_kv.set_val(kv)
                self.is_updating = False
            except: pass

        def on_slide_change(self, val):
            if self.target_actuator_id == -1 or self.is_updating: return
            self.is_updating = True
            kp = self.slide_kp.val
            kv = self.slide_kv.val
            
            model.actuator_gainprm[self.target_actuator_id, 0] = kp
            model.actuator_biasprm[self.target_actuator_id, 1] = -kv
            
            self.box_kp.set_val(f"{kp:.1f}")
            self.box_kv.set_val(f"{kv:.1f}")
            self.is_updating = False

    # Create 4 Slots
    for i in range(4):
        slots.append(JointControl(i, 0.35 - (i * 0.08)))

    def update_scope(label):
        global scope_selection
        scope_selection = label
        
        # Identify actuators in this scope
        target_aids = [] # (name, aid)
        
        keywords = scope_map.get(scope_selection, [])
        if scope_selection == 'All':
            # Too many for details? Show first 4?
            # Or just show a note "Select specific finger for details"
            pass
        
        # Sort by joint name to get 1, 2, 3 ordered
        # actuator_ids is name -> id
        sorted_acts = sorted(actuator_ids.items())
        
        count = 0
        for name, aid in sorted_acts:
            name_low = name.lower()
            if any(k in name_low for k in keywords):
                if count < 4:
                    slots[count].set_target(name, aid)
                    count += 1
        
        # Hide unused
        for i in range(count, 4):
            slots[i].set_visible(False)
            
        fig.canvas.draw_idle()

    radio.on_clicked(update_scope)
    # Init default
    update_scope(list(scope_map.keys())[0] if scope_map else 'All')
    
elif HAS_MATPLOTLIB:
    # Fallback/General Plot
    plt.ion()
    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(111, projection='3d')


# Main Loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting simulation loop...")
    start_time = time.time()
    last_print = 0
    
    # Set initial physics params
    update_sliders(0)
    
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
            
        else: # 'forward'
            # 2. FORWARD DYNAMICS (Validation)
            # Set Target for Actuators (Position Control)
            for jname, jid in joint_ids.items():
                 target_q = qpos_traj[step_idx, model.jnt_qposadr[jid]]
                 if jname in actuator_ids:
                     data.ctrl[actuator_ids[jname]] = target_q
            
            # Run Physics
            mujoco.mj_step(model, data)

        viewer.sync()

        
# --- VISUALIZATION & LOGGING ---
        # Collect Data
        vals_to_plot = []
        
        if args.mode == 'inverse':
            # Plot qfrc_inverse (Actuator Force required)
            vals_to_plot = [data.qfrc_inverse[model.jnt_dofadr[joint_ids[n]]] for n in data_source_names]
            
        elif args.mode == 'forward':
            # Plot Actuator Force (applied)
            vals_to_plot = [data.qfrc_actuator[model.jnt_dofadr[joint_ids[n]]] for n in data_source_names]
            
        elif args.mode == 'sensors':
            # Plot Sensor Data
            vals_to_plot = [data.sensordata[sensor_ids[n]] for n in data_source_names]

        # Log
        row = [elapsed] + vals_to_plot
        writer.writerow(row)
        
        # User requested: "Finger name, 1st, 2nd, 3rd" labels
        def get_pretty_label(name):
            # Parse standard robot names
            # e.g. "index_finger_joint_1" -> "Index 1st"
            low = name.lower()
            finger = "Unknown"
            if "thumb" in low: finger = "Thumb"
            elif "index" in low: finger = "Index"
            elif "middle" in low: finger = "Middle"
            elif "ring" in low: finger = "Ring"
            elif "little" in low or "pinky" in low: finger = "Little"
            
            # Find index
            idx_str = ""
            if "1" in name or "proximal" in name: idx_str = "1st"
            elif "2" in name or "medial" in name: idx_str = "2nd"
            elif "3" in name or "distal" in name: idx_str = "3rd"
            elif "tip" in name: idx_str = "Tip"
            
            if finger == "Unknown": return name
            return f"{finger}\n{idx_str}"

        # Draw Plot (Throttle)
        if HAS_MATPLOTLIB and (now - last_print > 0.1):
            ax.clear()
            ax.set_zlim(*z_lim_default)
            
            # Determine mapping
            dz_list = []
            
            if args.mode == 'sensors':
                 # ... existing sensor logic ...
                 # (Not modifying sensor grid logic in this request, assuming it's stable)
                 pass
            
            elif args.mode in ['inverse', 'forward']:
                # Bar Plot for Torques
                ax.set_title(f"Joint Torques (Nm) T={{elapsed:.2f}}s")
                
                # Make labels readable
                pretty_labels = [get_pretty_label(n) for n in data_source_names]
                
                x = np.arange(len(vals_to_plot))
                y = np.zeros_like(x)
                z = np.zeros_like(x)
                dx = np.ones_like(x) * 0.5
                dy = np.ones_like(x) * 0.5
                dz = vals_to_plot
                
                # Colors
                colors = []
                for v in dz:
                    if abs(v) > 10.0: colors.append('red') # Warning
                    elif abs(v) > 5.0: colors.append('orange')
                    else: colors.append('green')
                    
                ax.bar3d(x, y, z, dx, dy, dz, color=colors)
                
                # Set X ticks
                ax.set_xticks(x + 0.25)
                ax.set_xticklabels(pretty_labels, rotation=45, fontsize=8)

                ax.set_title(f"Sensor Forces (N) T={{elapsed:.2f}}s")
                # 3x7 Logic
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
                
                # Finger tick logic (same as before)
                # ...
                x_flat_arr = []
                y_flat_arr = []
                for f in range(5):
                    for r in range(7):
                        for c in range(3):
                            x_flat_arr.append(f*4 + c)
                            y_flat_arr.append(r)
                
                ax.bar3d(x_flat_arr, y_flat_arr, np.zeros_like(x_flat_arr), 0.8, 0.8, dz_list, color=c_list, shade=True)
                
            else:
                ax.set_title(f"Torques (Nm) T={{elapsed:.2f}}s Modes: Inverse/Forward")
                # 3x5 Logic
                for c in range(5):
                    for r in range(3):
                        jname = joint_grid_map[r][c]
                        val = 0.0
                        if jname and jname in joint_ids:
                             idx_map = data_source_names.index(jname)
                             val = vals_to_plot[idx_map]
                        dz_list.append(val)
                        norm_val = np.clip(val, -20, 20)
                        c_list.append(plt.cm.coolwarm((norm_val+20)/40.0))
                
                # Joints Mesh grid logic
                _x = np.arange(5)
                _y = np.arange(3)
                _xx, _yy = np.meshgrid(_x, _y)
                
                ax.bar3d(_xx.flatten(), _yy.flatten(), np.zeros_like(_xx.flatten()), 0.5, 0.5, dz_list, color=c_list, shade=True)
                
            try:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except: pass
            last_print = now

csv_file.close()
"""
