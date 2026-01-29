"""motion_exporter.py

Helpers to process recordings and produce small script/templates that are
placed inside exported packages. The functions below are intentionally
lightweight so they can be written into the package and edited by users.
"""

from typing import List, Dict, Any
import json
import os


def process_recordings_for_export(recordings_raw: List[Dict[str, Any]],
                                  generated_joints_info: List[Dict[str, Any]],
                                  robot_data: Any) -> List[Dict[str, Any]]:
    """Normalize recordings into a compact form suitable for the replay UI.

    Each recording is transformed so that every keyframe contains a flat
    mapping from exported joint name -> float. Missing joints default to 0.0.
    """
    joint_map = {}
    for info in (generated_joints_info or []):
        orig = info.get('original_id')
        suf = info.get('suffix')
        name = info.get('name')
        if orig and suf and name:
            joint_map[(orig, suf)] = name

    all_names = [info.get('name') for info in (generated_joints_info or []) if info.get('name')]

    out: List[Dict[str, Any]] = []
    for rec in (recordings_raw or []):
        clean = {
            'id': rec.get('id'),
            'name': rec.get('name'),
            'duration': rec.get('duration'),
            'keyframes': []
        }
        for kf in rec.get('keyframes', []):
            joints = {n: 0.0 for n in all_names}
            for jid, vals in (kf.get('jointValues') or {}).items():
                for suf in ('roll', 'pitch', 'yaw', 'prism'):
                    key = (jid, suf)
                    if key in joint_map:
                        v = vals.get('displacement' if suf == 'prism' else suf, 0.0)
                        try:
                            joints[joint_map[key]] = float(v or 0.0)
                        except Exception:
                            joints[joint_map[key]] = 0.0
            clean['keyframes'].append({'timestamp': kf.get('timestamp'), 'joints': joints})
        out.append(clean)
    return out


def generate_demo_script(python_script_name: str) -> str:
    return (
        "#!/bin/bash\n"
        "set -e\n"
        "VENV_DIR=\"venv\"\n"
        "if [ ! -d \"$VENV_DIR\" ]; then\n"
        "  python3 -m venv \"$VENV_DIR\"\n"
        "fi\n"
        "source \"$VENV_DIR/bin/activate\"\n"
        "python3 -m pip install --upgrade pip > /dev/null 2>&1\n"
        "python3 -m pip install flask numpy<2 > /dev/null 2>&1\n"
        f"python3 {python_script_name} \"$@\"\n"
    )


def generate_torque_launch_script(python_script_name: str, default_rec_idx: int = 0) -> str:
    return (
        "#!/bin/bash\n"
        "echo \"Select Visualization Mode:\"\n"
        "echo \"1. Joint Torques (Motion with Torque Values)\"\n"
        "echo \"2. Motion with Motor Parameter Adjustment\"\n"
        "echo \"3. Fingertip Sensors (Motion with Sensor Values)\"\n"
        "echo \"4. Exit\"\n"
        "read -p \"Enter choice [1]: \" choice\n"
        "\n"
        "if [ \"$choice\" = \"1\" ]; then\n"
        "    python3 {python_script_name} {default_rec_idx} --mode joints\n"
        "elif [ \"$choice\" = \"2\" ]; then\n"
        "    python3 motor_parameter_adjustment.py\n"
        "elif [ \"$choice\" = \"3\" ]; then\n"
        "    python3 {python_script_name} {default_rec_idx} --mode sensors\n"
        "elif [ \"$choice\" = \"4\" ]; then\n"
        "    echo \" Exiting...\"\n"
        "    exit 0\n"
        "else\n"
        "    echo \"Invalid choice. Exiting...\"\n"
        "    exit 1\n"
        "fi\n"
    )


def generate_mujoco_torque_replay_script(model_filename: str) -> str:
    # Template with a placeholder for MODEL_XML that we replace here.
    tpl = (
        "#!/usr/bin/env python3\n"
        "from flask import Flask, jsonify, request\n"
        "import json, os, threading, time\n"
        "\n"
        "MODEL_XML = '__MODEL_XML__'\n"
        "RECORDINGS_PATH = os.path.join('config', 'recordings.json')\n"
        "JOINTS_PATH = os.path.join('config', 'generated_joints_info.json')\n"
        "app = Flask(__name__)\n"
        "DIAG = []\n"
        "DIAG_LOCK = threading.Lock()\n"
        "SIM_STATE = {'running': False, 'index': 0}\n"
        "SIM_STOP = threading.Event()\n"
        "\n"
        "try:\n"
        "    import mujoco, numpy as np\n"
        "    HAS_MUJOCO = True\n"
        "except Exception:\n"
        "    mujoco = None\n"
        "    np = None\n"
        "    HAS_MUJOCO = False\n"
        "\n"
        "def load_json(path):\n"
        "    try:\n"
        "        with open(path,'r') as f:\n"
        "            return json.load(f)\n"
        "    except Exception:\n"
        "        return []\n"
        "\n"
        "@app.route('/')\n"
        "def index():\n"
        "    return '<html><body><h3>Torque Replay UI (minimal)</h3><p>Use /api/joints and /api/recordings</p></body></html>'\n"
        "\n"
        "@app.route('/api/recordings')\n"
        "def api_recordings():\n"
        "    return jsonify(load_json(RECORDINGS_PATH))\n"
        "\n"
        "@app.route('/api/joints')\n"
        "def api_joints():\n"
        "    return jsonify(load_json(JOINTS_PATH))\n"
        "\n"
        "@app.route('/api/apply_params', methods=['POST'])\n"
        "def api_apply_params():\n"
        "    payload = request.get_json() or []\n"
        "    with open('applied_params.json','w') as f:\n"
        "        json.dump(payload,f,indent=2)\n"
        "    return ('',204)\n"
        "\n"
        "@app.route('/api/start', methods=['POST'])\n"
        "def api_start():\n"
        "    body = request.get_json() or {}\n"
        "    SIM_STATE['index'] = int(body.get('index',0))\n"
        "    SIM_STATE['running'] = True\n"
        "    return ('',204)\n"
        "\n"
        "@app.route('/api/stop', methods=['POST'])\n"
        "def api_stop():\n"
        "    SIM_STATE['running'] = False\n"
        "    return ('',204)\n"
        "\n"
        "@app.route('/api/diag')\n"
        "def api_diag():\n"
        "    with DIAG_LOCK:\n"
        "        if not DIAG:\n"
        "            return jsonify({'torque_rms':0.0,'error_rms':0.0})\n"
        "        return jsonify(DIAG[-1])\n"
        "\n"
        "def sim_loop():\n"
        "    if not HAS_MUJOCO:\n"
        "        while not SIM_STOP.is_set():\n"
        "            if SIM_STATE.get('running'):\n"
        "                with DIAG_LOCK:\n"
        "                    DIAG.append({'torque_rms':0.0,'error_rms':0.0})\n"
        "            time.sleep(0.5)\n"
        "        return\n"
        "\n"
        "    try:\n"
        "        if not os.path.exists(MODEL_XML):\n"
        "            print('Model XML not found:', MODEL_XML)\n"
        "            while not SIM_STOP.is_set():\n"
        "                if SIM_STATE.get('running'):\n"
        "                    with DIAG_LOCK:\n"
        "                        DIAG.append({'torque_rms':0.0,'error_rms':0.0})\n"
        "                time.sleep(0.5)\n"
        "            return\n"
        "        model = mujoco.MjModel.from_xml_path(MODEL_XML)\n"
        "        data = mujoco.MjData(model)\n"
        "    except Exception as e:\n"
        "        print('Failed to initialize mujoco model:',e)\n"
        "        while not SIM_STOP.is_set():\n"
        "            if SIM_STATE.get('running'):\n"
        "                with DIAG_LOCK:\n"
        "                    DIAG.append({'torque_rms':0.0,'error_rms':0.0})\n"
        "            time.sleep(0.5)\n"
        "        return\n"
        "\n"
        "    while not SIM_STOP.is_set():\n"
        "        if SIM_STATE.get('running'):\n"
        "            try:\n"
        "                mujoco.mj_step(model,data)\n"
        "                forces = data.actuator_force.tolist() if hasattr(data,'actuator_force') else []\n"
        "                torque = float(np.sqrt(np.mean(np.square(forces)))) if forces else 0.0\n"
        "                err = float(np.linalg.norm(data.qpos)) if hasattr(data,'qpos') else 0.0\n"
        "                with DIAG_LOCK:\n"
        "                    DIAG.append({'torque_rms':torque,'error_rms':err})\n"
        "            except Exception:\n"
        "                with DIAG_LOCK:\n"
        "                    DIAG.append({'torque_rms':0.0,'error_rms':0.0})\n"
        "        time.sleep(0.05)\n"
        "\n"
        "if __name__=='__main__':\n"
        "    import threading\n"
        "    t = threading.Thread(target=sim_loop,daemon=True)\n"
        "    t.start()\n"
        "    try:\n"
        "        app.run(host='127.0.0.1',port=5000)\n"
        "    finally:\n"
        "        SIM_STOP.set()\n"
    )

    return tpl.replace('__MODEL_XML__', model_filename)


def generate_ros2_playback_node(package_name: str) -> str:
    # Use a simple placeholder with a safe replace token to avoid nested braces
    tpl = (
        "#!/usr/bin/env python3\n"
        "import rclpy\n"
        "from rclpy.node import Node\n"
        "from sensor_msgs.msg import JointState\n"
        "import json, os\n"
        "\n"
        "RECORDINGS_PATH = os.path.join('share','__PKG__','config','recordings.json')\n"
        "\n"
        "class ReplayNode(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('replay_node')\n"
        "        self.pub = self.create_publisher(JointState,'replay_joint_states',10)\n"
        "        self.timer = self.create_timer(0.02,self.timer_cb)\n"
        "        self.recordings = self.load_recordings()\n"
        "\n"
        "    def load_recordings(self):\n"
        "        try:\n"
        "            with open(RECORDINGS_PATH,'r') as f:\n"
        "                return json.load(f)\n"
        "        except Exception:\n"
        "            return []\n"
        "\n"
        "    def timer_cb(self):\n"
        "        if not self.recordings:\n"
        "            return\n"
        "        rec = self.recordings[0] if self.recordings else None\n"
        "        if not rec:\n"
        "            return\n"
        "        kfs = rec.get('keyframes',[])\n"
        "        if not kfs:\n"
        "            return\n"
        "        js = JointState()\n"
        "        j0 = kfs[0]\n"
        "        joints = j0.get('joints',{})\n"
        "        js.name = list(joints.keys())\n"
        "        js.position = [float(joints[n]) for n in js.name]\n"
        "        js.header.stamp = self.get_clock().now().to_msg()\n"
        "        self.pub.publish(js)\n"
        "\n"
        "def main(args=None):\n"
        "    rclpy.init(args=args)\n"
        "    node = ReplayNode()\n"
        "    try:\n"
        "        rclpy.spin(node)\n"
        "    except KeyboardInterrupt:\n"
        "        pass\n"
        "    node.destroy_node()\n"
        "    rclpy.shutdown()\n"
        "\n"
        "if __name__=='__main__':\n"
        "    main()\n"
    )

    return tpl.replace('__PKG__', package_name)


def generate_mujoco_playback_script(robot_name: str) -> str:
    return """#!/usr/bin/env python3
print('This is a placeholder MuJoCo playback script for %s')
""" % robot_name


def generate_mujoco_interactive_script(robot_name: str) -> str:
    return """#!/usr/bin/env python3
print('This is a placeholder MuJoCo interactive script for %s')
""" % robot_name


def generate_replay_script(robot_name: str) -> str:
    return """#!/usr/bin/env python3
print('This is a small replay helper for %s')
""" % robot_name

