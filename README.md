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
#### Setup
1.  Extract the ZIP.
2.  Install `mujoco` python package (`pip install mujoco`).

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