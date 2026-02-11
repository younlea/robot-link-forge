# Robot Link Forge 🤖

**Robot Link Forge** is an intuitive, web-based tool for designing, animating, and exporting robot models. It Bridges the gap between visual design and simulation by allowing you to build URDF models interactively, record motions using sliders or webcam hand tracking, and export everything directly to **ROS 2** and **MuJoCo**.

---

## 🚀 Key Features

*   **Visual Editor**: Build robot models by adding links and joints. Drag & drop STL meshes or use primitive shapes.
*   **Advanced Joint Types**:
    *   **Rolling Contact Joints**: Model curvature-based contact (convex/concave surfaces) with friction.
    *   **Tendon-Driven Systems**: Active (motor-driven) and passive (spring) tendons with 3D click-based routing.
*   **Simulation Environment**:
    *   **Obstacles**: Add fixed-position obstacles (box/sphere/cylinder) for contact simulation.
    *   **Sensors**: Place touch and force sensors on link surfaces with 3D click placement.
*   **Motion Recording**:
    *   **Timeline Editor**: precise keyframe control and interpolation modification.
    *   **Hand Control**: Control your robot in real-time using your webcam (MediaPipe Hand Tracking).
*   **Export Ready**:
    *   **ROS 2**: Generates a full ament_cmake package with valid URDF, RViz config, and replay scripts.
    *   **MuJoCo**: Generates MJCF XML with calculated inertia, tendons, obstacles, sensors, and simulation scripts.
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
*   **Joint Types**:
    *   **Fixed**: No movement, used to glue parts together.
    *   **Rotational**: Revolute joints with 1-3 DoF (Roll/Pitch/Yaw).
    *   **Prismatic**: Linear sliding motion.
    *   **Rolling Contact**: Models rolling surfaces with curvature radius and contact friction.

### 2. Advanced Simulation Features

#### Tendon System 🧵
*   Add **Active Tendons** (motor-driven) or **Passive Tendons** (spring-based).
*   **3D Click Routing**: Enter routing mode, then click on link surfaces to define the tendon path.
*   Configure stiffness, damping, rest length, and moment arm.
*   Exports to MuJoCo `<tendon><spatial>` with automatic site generation.

#### Obstacles 🪨
*   Add fixed-position obstacles (Box, Sphere, Cylinder) for contact simulation.
*   Adjust position, rotation, dimensions, color, and physics parameters (friction, solref, solimp).
*   Toggle enabled/disabled without deleting.
*   Exports to MuJoCo as fixed `<body>` elements with contact properties.

#### Sensors 📡
*   **3D Click Placement**: Enter sensor placement mode and click on link surfaces.
*   Automatically creates touch sensors at precise locations.
*   Exports to MuJoCo `<sensor><touch>` with link-local coordinates.

### 3. Motion Recording
*   Click **"Recording"** in the sidebar.
*   **Slider Mode**: Move joints manually and click **"Capture Keyframe"**.
*   **Timeline**: Drag keyframes to adjust timing. Modify transition bars to change speed.
*   **Webcam Mode**: Toggle "Camera" to control the robot with your hand.
*   **Tendon-Driven Recording**: For joints controlled by active tendons, use the "Tendon Input" slider in the link inspector. This records both joint positions and tendon input values for realistic MuJoCo simulation.

### 4. Export & Replay (ROS 2)
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

### 5. Export & Replay (MuJoCo)

MuJoCo export includes motor sizing validation tools for mechanical engineers, plus advanced features like tendons, obstacles, and sensors.

#### Setup
1. Extract the ZIP file
2. Run any shell script - virtual environment and dependencies are installed automatically

#### Running Analysis
```bash
# Basic replay
./replay_0_my_recording.sh

# Advanced analysis with visualization
./run_torque_replay.sh
```

**Analysis Modes:**
- **Mode 1**: Joint Torque Visualization (inverse dynamics, 3x5 grid)
- **Mode 2**: Motor Sizing Validation (set motor params, check performance)
- **Mode 3**: Fingertip Sensor Forces (contact visualization, 3x7 grid)

**Mode 2 Features:**
- Set motor parameters per joint (forcelim, gear, velocity, armature, frictionloss)
- Real-time validation: torque saturation, tracking error, T-N curve, RMS thermal load
- Parameters saved to `motor_parameters.json` for reuse

---

## 🤝 Contributing
Feel free to open issues or submit PRs to improve the tool!