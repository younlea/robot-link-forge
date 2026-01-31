# Robot Link Forge 🤖

**Robot Link Forge** is an intuitive, web-based tool for designing, animating, and exporting robot models. It Bridges the gap between visual design and simulation by allowing you to build URDF models interactively, record motions using sliders or webcam hand tracking, and export everything directly to **ROS 2** and **MuJoCo**.

---

## 🚀 Key Features

*   **Visual Editor**: Build robot models by adding links and joints. Drag & drop STL meshes or use primitive shapes.
*   **Motion Recording**:
    *   **Timeline Editor**: precise keyframe control and interpolation modification.
    *   **Hand Control**: Control your robot in real-time using your webcam (MediaPipe Hand Tracking).
*   **Export Ready**:
    *   **ROS 2**: Generates a full ament_cmake package with valid URDF, RViz config, and replay scripts.
    *   **MuJoCo**: Generates MJCF XML with calculated inertia, sensors, and simulation scripts.
*   **Replay Support**: Automatically generates Python scripts to replay your recorded motions in the target environment.

---

## 🛠️ Installation & Run

### Prerequisites
*   **Node.js** (v18+)
*   **Python** (v3.8+)
*   **ROS 2** (Humble/Iron/Jazzy) - *Optional, for export verification*

### 1. Backend Setup
The backend handles file operations and URDF generation.
```bash
cd src/backend
# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install fastapi uvicorn python-multipart jinja2 numpy

# Run Server
uvicorn main:app --reload --port 8000
```

### 2. Frontend Setup
The frontend is the visual editor interface.
```bash
cd src/frontend
npm install
npm run dev
```
Open your browser to `http://localhost:5173`.

---

## 📖 Usage Guide

### 1. Modeling
*   Use the **Sidebar** to add Links and Joints.
*   **Upload Meshes**: Supports STL files for detailed visuals.
*   **Configurations**: Adjust joint limits, axis (Roll/Pitch/Yaw/Prismatic), and parent-child relationships.

### 2. Motion Recording
*   Click **"Recording"** in the sidebar.
*   **Slider Mode**: Move joints manually and click **"Capture Keyframe"**.
*   **Timeline**: Drag keyframes to adjust timing. Modify transition bars to change speed.
*   **Webcam Mode**: Toggle "Camera" to control the robot with your hand.

### 3. Export & Replay (ROS 2)
When you export to ROS 2, you get a ZIP file containing a complete ROS 2 package.

#### Setup
1.  Extract the ZIP to your ROS 2 workspace `src/` folder.
2.  Build:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

#### Running Replay (Split-Terminal Workflow)
To avoid conflicts between the viewer and the replay controller, use two terminals:

**Terminal 1: Viewer**
Runs RViz to visualize the robot.
```bash
./build_and_launch.sh
# Select Option 2) Passive (Disable Sliders)
```

**Terminal 2: Replay Controller**
Runs the replay node to move the robot.
```bash
./replay_0_my_recording.sh
```
*   `replay_0`, `replay_1`, etc. correspond to your saved recordings.
*   Progress will be logged to the terminal: `Progress: 1.5s / 10.0s (15.0%)`.

### 4. Export & Replay (MuJoCo)

The MuJoCo export includes powerful analysis tools for motor sizing and validation.

#### Setup
1. Extract the ZIP.
2. Run any of the shell scripts - they will automatically create a virtual environment and install dependencies.

#### Running Analysis

**Basic Replay:**
```bash
./replay_0_my_recording.sh
```

**Advanced Analysis (with Torque/Sensor Visualization):**
```bash
./run_torque_replay.sh
```

This will show an interactive menu:

```
========================================
  MuJoCo Motion Analysis Tool
========================================

Select Analysis Mode:

1. Joint Torque Visualization (Inverse Dynamics)
   - Shows theoretical torque needed for recorded motion
   - 3x5 grid for finger joints
   - Ignores motor limitations (kinematic mode)

2. Motor Sizing Validation (Forward Dynamics) ⭐ NEW
   - Verify if selected motors can perform the motion
   - Set motor parameters (torque, speed, inertia, etc.)
   - Real-time validation with 4 graphs:
     ① Torque Saturation Check
     ② Position Tracking Error
     ③ Speed-Torque Curve (T-N Curve)
     ④ RMS Torque & Pass/Fail Judgment

3. Fingertip Sensor Forces
   - 3x7 grid for contact sensors
   - Shows force magnitude at each sensor

========================================
```

#### Mode 2: Motor Sizing Validation (Detailed Guide)

This mode helps mechanical engineers validate motor selection by simulating realistic motor constraints.

**What You'll See:**
- **Left Window**: MuJoCo 3D viewer showing robot motion
- **Right Window**: Motor parameter GUI with validation graphs

**Workflow:**

1. **Select a Joint**
   - Click on any joint in the list (left panel)
   - Modified joints show ✓, default joints show ○

2. **Enter Motor Parameters** (from datasheet):
   - **Max Torque (forcelim)**: Stall Torque × Gear Ratio × Efficiency (0.8)
   - **Gear Ratio**: e.g., 100 for 100:1 reduction
   - **Max Speed**: No-load RPM × 0.1047 (convert to rad/s)
   - **Rotor Inertia (armature)**: Rotor inertia × (gear ratio)²
   - **Friction Loss**: ~5-10% of rated torque
   - **Kp/Kv**: PD controller gains (start with 50/1)

3. **Click "Apply"**
   - Parameters are applied to the model
   - Simulation restarts automatically

4. **Watch Validation**
   - Robot moves in MuJoCo viewer
   - 4 graphs update in real-time:
   
   **① Torque Saturation**: 
   - Blue line: actual torque
   - Red line: motor limit
   - ❌ If line clips → Motor too weak
   
   **② Tracking Error**:
   - Shows position error over time
   - ❌ If error > 0.1 rad → Can't follow trajectory
   
   **③ T-N Curve**:
   - Speed vs Torque scatter plot
   - Red lines: motor operating limits
   - ❌ Points outside box → Impossible to achieve
   
   **④ Pass/Fail Summary**:
   - RMS Torque (thermal load)
   - Max torque utilization
   - Overall verdict: ✅ PASS or ❌ FAIL

5. **Save Parameters**
   - Click "Save All Params" to save to `motor_parameters.json`
   - Parameters will be reloaded next time

**Tips:**
- Start with conservative default values
- Test one finger at a time
- If validation fails, increase `forcelim` or reduce motion speed
- RMS < 70% of max torque is recommended for safety

**Output Files:**
- `motor_parameters.json`: Saved motor configurations
- `torque_log_forward.csv`: Detailed data for each timestep

#### Running
```bash
# Visualize Model
python3 visualize_urdf.py

# Replay Recording
python3 replay_recording.py 0
```
---

## 🤝 Contributing
Feel free to open issues or submit PRs to improve the tool!