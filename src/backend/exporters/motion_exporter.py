"""motion_exporter.py

Minimal helpers used by the backend exporter to write a small Flask-based
diagnostic replay app and simple launcher scripts into an exported package.
This module intentionally keeps templates small and uses placeholder
replacement to avoid f-string/brace escaping issues.
"""

import json
import os
from typing import List, Dict

try:
    from robot_models import RobotData
except Exception:
    class RobotData:  # pragma: no cover - fallback for editor/tests
        pass


def process_recordings_for_export(recordings_raw: List[Dict], generated_joints_info: List[Dict], robot_data: RobotData) -> List[Dict]:
    """motion_exporter.py

    Minimal helpers used by the backend exporter to write a small Flask-based
    diagnostic replay app and simple launcher scripts into an exported package.
    This module intentionally keeps templates small and uses placeholder
    replacement to avoid f-string/brace escaping issues.
    """

    import json
    import os
    from typing import List, Dict

    try:
        from robot_models import RobotData
    except Exception:
        class RobotData:  # pragma: no cover - fallback for editor/tests
            pass


    def process_recordings_for_export(recordings_raw: List[Dict], generated_joints_info: List[Dict], robot_data: RobotData) -> List[Dict]:
        """Normalize recordings: map recorded joint ids+suffixes to exported joint names.

        Returns a list of recordings where each keyframe has a 'joints' dict keyed by
        exported joint name with float values.
        """
        joint_map = {}
        for info in generated_joints_info:
            orig = info.get('original_id')
            suf = info.get('suffix')
            name = info.get('name')
            if orig and suf and name:
                joint_map[(orig, suf)] = name

        all_names = [info.get('name') for info in generated_joints_info if info.get('name')]

        out = []
        for rec in recordings_raw:
            clean = {'id': rec.get('id'), 'name': rec.get('name'), 'duration': rec.get('duration'), 'keyframes': []}
            for kf in rec.get('keyframes', []):
                joints = {n: 0.0 for n in all_names}
                for jid, vals in (kf.get('jointValues') or {}).items():
                    for suf in ('roll', 'pitch', 'yaw', 'prism'):
                        key = (jid, suf)
                        if key in joint_map:
                            v = vals.get('displacement' if suf == 'prism' else suf, 0.0)
                            joints[joint_map[key]] = v
                clean['keyframes'].append({'timestamp': kf.get('timestamp'), 'joints': joints})
            out.append(clean)
        return out


    def generate_demo_script(python_script_name: str) -> str:
        """Return a small bash script that creates a venv and runs the given python script."""
        return (
            """#!/bin/bash
    set -e
    VENV_DIR=\"venv\"
    if [ ! -d "$VENV_DIR" ]; then
      python3 -m venv "$VENV_DIR"
    fi
    source "$VENV_DIR/bin/activate"
    python3 -m pip install --upgrade pip > /dev/null 2>&1
    python3 -m pip install flask numpy<2 > /dev/null 2>&1
    python3 %s "$@"
    """ % python_script_name
        )


    def generate_torque_launch_script(python_script_name: str, default_rec_idx: int = 0) -> str:
        """Return a launcher that creates a venv, installs Flask and runs the replay app."""
        return (
            """#!/bin/bash
    set -e
    VENV_DIR=\"venv\"
    if [ ! -d "$VENV_DIR" ]; then
      python3 -m venv "$VENV_DIR"
    fi
    source "$VENV_DIR/bin/activate"
    python3 -m pip install --upgrade pip > /dev/null 2>&1
    python3 -m pip install flask > /dev/null 2>&1
    python3 %s %d
    """ % (python_script_name, default_rec_idx)
        )


    def generate_mujoco_torque_replay_script(model_filename: str) -> str:
        """Return the contents of a minimal Flask-based replay/diagnostic app.

        The returned script provides a minimal index page and a few JSON
        endpoints used by the tuning UI. It gracefully handles missing MuJoCo
        by running a no-op diagnostics loop that emits zeros.
        """
        template = (
            '#!/usr/bin/env python3\n'
            'from flask import Flask, jsonify, request\n'
            'import json\n'
            'import os\n'
            'import threading\n'
            'import time\n'
            '\n'
            'MODEL_XML = "__MODEL_XML__"\n'
            "RECORDINGS_PATH = os.path.join('config', 'recordings.json')\n"
            "JOINTS_PATH = os.path.join('config', 'generated_joints_info.json')\n"
            '\n'
            'app = Flask(__name__)\n'
            'DIAG = []\n'
            'DIAG_LOCK = threading.Lock()\n'
            "SIM_STATE = {'running': False, 'index': 0}\n"
            'SIM_STOP = threading.Event()\n'
            '\n'
            'try:\n'
            '    import mujoco\n'
            '    import numpy as np\n'
            '    HAS_MUJOCO = True\n'
            'except Exception:\n'
            '    mujoco = None\n'
            '    np = None\n'
            '    HAS_MUJOCO = False\n'
            '\n'
            'def load_json(path):\n'
            '    try:\n'
            '        with open(path, "r") as f:\n'
            '            return json.load(f)\n'
            '    except Exception:\n'
            '        return []\n'
            '\n'
            "@app.route('/')\n"
            'def index():\n'
            "    return '<html><body><h3>Torque Replay UI (minimal)</h3><p>Use /api/joints and /api/recordings</p></body></html>'\n"
            '\n'
            "@app.route('/api/recordings')\n"
            'def api_recordings():\n'
            '    return jsonify(load_json(RECORDINGS_PATH))\n'
            '\n'
            "@app.route('/api/joints')\n"
            'def api_joints():\n'
            '    return jsonify(load_json(JOINTS_PATH))\n'
            '\n'
            "@app.route('/api/apply_params', methods=['POST'])\n"
            'def api_apply_params():\n'
            '    payload = request.get_json() or []\n'
            "    with open('applied_params.json', 'w') as f:\n"
            '        json.dump(payload, f, indent=2)\n'
            "    return ('', 204)\n"
            '\n'
            "@app.route('/api/start', methods=['POST'])\n"
            'def api_start():\n'
            "    body = request.get_json() or {}\n"
            "    SIM_STATE['index'] = int(body.get('index', 0))\n"
            "    SIM_STATE['running'] = True\n"
            "    return ('', 204)\n"
            '\n'
            "@app.route('/api/stop', methods=['POST'])\n"
            'def api_stop():\n'
            "    SIM_STATE['running'] = False\n"
            "    return ('', 204)\n"
            '\n'
            "@app.route('/api/diag')\n"
            'def api_diag():\n'
            '    with DIAG_LOCK:\n'
            "        if not DIAG:\n"
            "            return jsonify({'torque_rms': 0.0, 'error_rms': 0.0})\n"
            '        return jsonify(DIAG[-1])\n'
            '\n'
            'def sim_loop():\n'
            '    if not HAS_MUJOCO:\n'
            '        while not SIM_STOP.is_set():\n'
            '            if SIM_STATE.get("running"):\n'
            '                with DIAG_LOCK:\n'
            "                    DIAG.append({'torque_rms': 0.0, 'error_rms': 0.0})\n"
            '            time.sleep(0.5)\n'
            '        return\n'
            '\n'
            '    try:\n'
            '        if not os.path.exists(MODEL_XML):\n'
            "            print('Model XML not found:', MODEL_XML)\n"
            '            while not SIM_STOP.is_set():\n'
            '                if SIM_STATE.get("running"):\n'
            '                    with DIAG_LOCK:\n'
            "                        DIAG.append({'torque_rms': 0.0, 'error_rms': 0.0})\n"
            '                time.sleep(0.5)\n'
            '            return\n'
            '\n'
            '        model = mujoco.MjModel.from_xml_path(MODEL_XML)\n'
            '        data = mujoco.MjData(model)\n'
            '    except Exception as e:\n'
            "        print('Failed to initialize mujoco model:', e)\n"
            '        while not SIM_STOP.is_set():\n'
            '            if SIM_STATE.get("running"):\n'
            '                with DIAG_LOCK:\n'
            "                    DIAG.append({'torque_rms': 0.0, 'error_rms': 0.0})\n"
            '            time.sleep(0.5)\n'
            '        return\n'
            '\n'
            '    while not SIM_STOP.is_set():\n'
            '        if SIM_STATE.get("running"):\n'
            '            try:\n'
            '                mujoco.mj_step(model, data)\n'
            '                forces = data.actuator_force.tolist() if hasattr(data, "actuator_force") else []\n'
            '                torque = float(np.sqrt(np.mean(np.square(forces)))) if forces else 0.0\n'
            '                err = float(np.linalg.norm(data.qpos)) if hasattr(data, "qpos") else 0.0\n'
            '                with DIAG_LOCK:\n'
            "                    DIAG.append({'torque_rms': torque, 'error_rms': err})\n"
            '            except Exception:\n'
            "                with DIAG_LOCK:\n"
            "                    DIAG.append({'torque_rms': 0.0, 'error_rms': 0.0})\n"
            '        time.sleep(0.05)\n'
            '\n'
            'if __name__ == "__main__":\n'
            '    t = threading.Thread(target=sim_loop, daemon=True)\n'
            '    t.start()\n'
            '    try:\n'
            '        app.run(host="127.0.0.1", port=5000)\n'
            '    finally:\n'
            '        SIM_STOP.set()\n'
        )

        return template.replace('__MODEL_XML__', model_filename)
