#!/usr/bin/env python3
from flask import Flask, jsonify, request
import json, os, threading, time

MODEL_XML = '__MODEL_XML__'
RECORDINGS_PATH = os.path.join('config', 'recordings.json')
JOINTS_PATH = os.path.join('config', 'generated_joints_info.json')
app = Flask(__name__)
DIAG = []
DIAG_LOCK = threading.Lock()
SIM_STATE = {'running': False, 'index': 0}
SIM_STOP = threading.Event()

try:
    import mujoco, numpy as np
    HAS_MUJOCO = True
except Exception:
    mujoco = None
    np = None
    HAS_MUJOCO = False

def load_json(path):
    try:
        with open(path,'r') as f:
            return json.load(f)
    except Exception:
        return []

@app.route('/')
def index():
    return '<html><body><h3>Torque Replay UI (minimal)</h3><p>Use /api/joints and /api/recordings</p></body></html>'

@app.route('/api/recordings')
def api_recordings():
    return jsonify(load_json(RECORDINGS_PATH))

@app.route('/api/joints')
def api_joints():
    return jsonify(load_json(JOINTS_PATH))

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)