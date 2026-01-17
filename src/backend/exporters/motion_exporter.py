
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

# Try to import matplotlib for sensor graphing
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Button

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
print("Right window: Real-time sensor graph system.")
print("Select a FINGER category to see its sensors.")
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

if model.nsensor > 0:
    for i in range(model.nsensor):
        fullname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        name = fullname if fullname else f"Sensor_{{i}}"
        sensor_names.append(name)
        
        low = name.lower()
        if 'thumb' in low: sensor_indices_by_finger['Thumb'].append(i)
        elif 'index' in low: sensor_indices_by_finger['Index'].append(i)
        elif 'middle' in low: sensor_indices_by_finger['Middle'].append(i)
        elif 'ring' in low: sensor_indices_by_finger['Ring'].append(i)
        elif 'little' in low or 'pinky' in low: sensor_indices_by_finger['Little'].append(i)
        else: sensor_indices_by_finger['Other'].append(i)
else:
    print("No sensors found in model.")

# --- UI State ---
current_view = "HOME" # HOME or DETAIL
current_finger = None

# --- Setup Real-time Plot ---
plt.ion()
fig, ax = plt.subplots(figsize=(14, 7))
plt.subplots_adjust(left=0.3) # Make lots of room for UI

# Initially create ALL lines but hide them
lines = []
for i, name in enumerate(sensor_names):
    ln, = ax.plot([], [], label=name, visible=False) # Start hidden
    lines.append(ln)

ax.set_title("Real-time Sensor Data")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Force")
ax.set_ylim(-0.1, 5.0) # Approx range, will autoscale

# --- UI Elements ---
# We will dynamically clear/rebuild the widgets axes

ui_axes = [] # Track axes to clear them

def clear_ui():
    global ui_axes, buttons, checkbuttons
    for a in ui_axes:
        a.remove()
    ui_axes = []
    buttons = {{}} # Keep references
    checkbuttons = None
    plt.draw()

buttons = {{}}
checkbuttons = None

def show_home_menu(event=None):
    global current_view
    current_view = "HOME"
    clear_ui()
    ax.set_title("Select a Finger Group")
    
    # Create Buttons for each non-empty category
    y_pos = 0.8
    for cat in finger_categories:
        if not sensor_indices_by_finger[cat]: continue
        
        # Axis for button
        b_ax = plt.axes([0.05, y_pos, 0.2, 0.08]) # Left panel
        btn = Button(b_ax, f"{{cat}} ({{len(sensor_indices_by_finger[cat])}})")
        # Closure to capture category
        def make_callback(c):
            return lambda event: show_detail_menu(c)
        btn.on_clicked(make_callback(cat))
        
        buttons[cat] = btn # Store ref
        ui_axes.append(b_ax)
        y_pos -= 0.1

def show_detail_menu(finger):
    global current_view, current_finger, checkbuttons
    current_view = "DETAIL"
    current_finger = finger
    clear_ui()
    ax.set_title(f"Sensors for {{finger}}")
    
    # Back Button
    b_ax = plt.axes([0.05, 0.85, 0.2, 0.08])
    btn = Button(b_ax, "< BACK")
    btn.on_clicked(show_home_menu)
    buttons['back'] = btn
    ui_axes.append(b_ax)
    
    # Checkboxes for sensors in this group
    indices = sensor_indices_by_finger[finger]
    labels = [sensor_names[i] for i in indices]
    # Check visibility state
    actives = [lines[i].get_visible() for i in indices]
    
    # Scrollable? No, just squeeze them in for now. 18 sensors is a lot for 80% height.
    # We might need small font.
    c_ax = plt.axes([0.05, 0.1, 0.2, 0.7]) # large vertical area
    checkbuttons = CheckButtons(c_ax, labels, actives)
    
    def on_check(label):
        # Find index 
        # CAUTION: label is non-unique if we removed prefix? No, names are unique.
        idx = -1
        for i in indices:
            if sensor_names[i] == label:
                idx = i
                break
        if idx != -1:
            lines[idx].set_visible(not lines[idx].get_visible())
            plt.draw()
            
    checkbuttons.on_clicked(on_check)
    ui_axes.append(c_ax)
    
    # Force initial visibility update (if needed)
    plt.draw()

# Start at Home
show_home_menu()

plt.show()

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
            
            # Update Plot at 10Hz
            if time.time() - last_plot_time > 0.1:
                # DEBUG: Verify sensor data is non-zero
                if current_vals.max() == 0 and current_vals.min() == 0:
                     if not hasattr(viewer, 'has_warned_zero'):
                         print("[DEBUG] Warning: All sensor values are 0.0. Check collision contacts!")
                         viewer.has_warned_zero = True
                else:
                     if hasattr(viewer, 'has_warned_zero'):
                         print(f"[DEBUG] Sensor data detected! Max: {current_vals.max():.4f}")
                         del viewer.has_warned_zero # Reset warning

                limit = 500
                x_data = timestamps[-limit:]
                # But we also need to update data for lines that might become visible? 
                # Yes, update data for all, but drawing only happens for visible.
                
                # Check for autoscale
                max_val = 0
                has_visible = False
                
                raw_history = np.array(sensor_history[-limit:])
                
                for i, ln in enumerate(lines):
                    # Always set data so it's ready when checked
                    y_data = raw_history[:, i]
                    ln.set_data(x_data, y_data)
                    
                    if ln.get_visible():
                        has_visible = True
                        m = np.max(y_data) if len(y_data) > 0 else 0
                        if m > max_val: max_val = m
                
                ax.set_xlim(x_data[0], x_data[-1] + 1)
                
                # Dynamic Y-scale if visible
                if has_visible:
                    # Smoothing scale
                    cur_ylim = ax.get_ylim()[1]
                    target_ylim = max(0.5, max_val * 1.2)
                    # Simple interpolation or set
                    ax.set_ylim(-0.1, target_ylim)
                
                # fig.canvas.draw() # Expensive
                # fig.canvas.flush_events()
                # Use blit if possible, but flush_events is safer for UI
                plt.pause(0.001) 
                
                last_plot_time = time.time()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("Saving data to sensor_log.csv...")
if sensor_names and sensor_history:
    with open('sensor_log.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time'] + sensor_names)
        for t, row in zip(timestamps, sensor_history):
            writer.writerow([t] + list(row))
    print("Saved.")

plt.ioff()
plt.show() 
"""
