import React, { useEffect, useRef, useState } from 'react';
import { HandLandmarker, FilesetResolver } from '@mediapipe/tasks-vision';
import { useRobotStore } from '../store';
import { XCircle, Video, Play, Pause } from 'lucide-react';

/*
  HandControl.tsx
  - Handles Webcam Stream
  - Runs MediaPipe Hand Landmark Detection
  - Ports Python 'calculate_finger_curl' logic to TypeScript
  - Updates Robot Store directly
*/

// --- Type Definitions for MediaPipe ---
// (Implicitly handled by @mediapipe/tasks-vision types, but helpful for reference)

const HandControl = ({ onClose }: { onClose: () => void }) => {
    const videoRef = useRef<HTMLVideoElement>(null);
    const canvasRef = useRef<HTMLCanvasElement>(null);

    // MediaPipe State
    const [handLandmarker, setHandLandmarker] = useState<HandLandmarker | null>(null);
    const [webcamRunning, setWebcamRunning] = useState(false);
    const [isLoading, setIsLoading] = useState(true);
    const [statusMsg, setStatusMsg] = useState('Initializing AI Model...');
    const [fps, setFps] = useState(0);

    // Store Access
    const { updateJoint, links, joints } = useRobotStore();

    // --- 1. Initialize MediaPipe ---
    useEffect(() => {
        const initMediaPipe = async () => {
            try {
                const vision = await FilesetResolver.forVisionTasks(
                    "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.2/wasm"
                );
                const landmarker = await HandLandmarker.createFromOptions(vision, {
                    baseOptions: {
                        modelAssetPath: `https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task`,
                        delegate: "GPU"
                    },
                    runningMode: "VIDEO",
                    numHands: 1
                });
                setHandLandmarker(landmarker);
                setIsLoading(false);
                setStatusMsg('Ready to Start');
                startWebcam(); // Auto-start for convenience
            } catch (error) {
                console.error("Failed to load MediaPipe:", error);
                setStatusMsg('Failed to load AI Model.');
                setIsLoading(false);
            }
        };

        initMediaPipe();

        return () => {
            // Cleanup: Stop webcam
            stopWebcam();
        };
    }, []);

    // --- 2. Webcam Handling ---
    const startWebcam = async () => {
        if (!handLandmarker) return;
        try {
            const stream = await navigator.mediaDevices.getUserMedia({ video: true });
            if (videoRef.current) {
                videoRef.current.srcObject = stream;
                videoRef.current.addEventListener("loadeddata", predictWebcam);
            }
            setWebcamRunning(true);
            setStatusMsg('Tracking Active');
        } catch (err) {
            console.error("Webcam Error:", err);
            setStatusMsg('Camera Permission Denied');
        }
    };

    const stopWebcam = () => {
        if (videoRef.current && videoRef.current.srcObject) {
            const stream = videoRef.current.srcObject as MediaStream;
            stream.getTracks().forEach(track => track.stop());
            videoRef.current.srcObject = null;
        }
        setWebcamRunning(false);
        setStatusMsg('Paused');
    };

    // --- 3. Prediction Loop ---
    const lastVideoTimeRef = useRef(-1);
    const requestRef = useRef<number>();
    const fpsTimeRef = useRef(Date.now());
    const fpsCountRef = useRef(0);

    const predictWebcam = async () => {
        if (!videoRef.current || !handLandmarker) return;

        // FPS Calculation
        const now = Date.now();
        fpsCountRef.current++;
        if (now - fpsTimeRef.current >= 1000) {
            setFps(fpsCountRef.current);
            fpsCountRef.current = 0;
            fpsTimeRef.current = now;
        }

        const video = videoRef.current;
        if (video.currentTime !== lastVideoTimeRef.current) {
            const startTimeMs = performance.now();
            const results = handLandmarker.detectForVideo(video, startTimeMs);
            lastVideoTimeRef.current = video.currentTime;

            if (results.landmarks && results.landmarks.length > 0) {
                // Visualize and Control
                drawLandmarks(results.landmarks[0]);
                updateRobotControl(results.landmarks[0], results.handedness[0]);
            } else {
                if (canvasRef.current) {
                    const ctx = canvasRef.current.getContext('2d');
                    if (ctx) ctx.clearRect(0, 0, canvasRef.current.width, canvasRef.current.height);
                }
            }
        }

        if (webcamRunning) {
            requestRef.current = requestAnimationFrame(predictWebcam);
        }
    };

    useEffect(() => {
        if (webcamRunning) {
            // Re-trigger loop if state changes to running and loop stopped
            // Actually `loadeddata` event triggers it initially.
            // If we toggle play/pause, we might need to kick it.
            if (videoRef.current && videoRef.current.readyState >= 2) {
                predictWebcam();
            }
        } else {
            if (requestRef.current) cancelAnimationFrame(requestRef.current);
        }
    }, [webcamRunning]);


    // --- 4. Logic Port: calculate_finger_curl ---
    const calculateFingerCurl = (landmarks: any[], fingerName: string) => {
        // Landmarks indexing (MediaPipe Standard)
        const WRIST = 0;
        const THUMB_CMC = 1, THUMB_MCP = 2, THUMB_IP = 3, THUMB_TIP = 4;
        const INDEX_MCP = 5, INDEX_PIP = 6, INDEX_DIP = 7, INDEX_TIP = 8;
        const MIDDLE_MCP = 9, MIDDLE_PIP = 10, MIDDLE_DIP = 11, MIDDLE_TIP = 12;
        const RING_MCP = 13, RING_PIP = 14, RING_DIP = 15, RING_TIP = 16;
        const PINKY_MCP = 17, PINKY_PIP = 18, PINKY_DIP = 19, PINKY_TIP = 20;

        const wrist = landmarks[WRIST];

        const dist = (p1: any, p2: any) => Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));

        if (fingerName === 'thumb') {
            const tip = landmarks[THUMB_TIP];
            const mcp = landmarks[INDEX_MCP]; // Distance to Index MCP
            const d = dist(tip, mcp);
            // Heuristic from Python: curl = clip((0.2 - dist) / 0.15)
            return Math.min(Math.max((0.2 - d) / 0.15, 0.0), 1.0);
        } else {
            let tipId, mcpId;
            if (fingerName === 'index') { tipId = INDEX_TIP; mcpId = INDEX_MCP; }
            else if (fingerName === 'middle') { tipId = MIDDLE_TIP; mcpId = MIDDLE_MCP; }
            else if (fingerName === 'ring') { tipId = RING_TIP; mcpId = RING_MCP; }
            else { tipId = PINKY_TIP; mcpId = PINKY_MCP; }

            const tip = landmarks[tipId];
            const mcp = landmarks[mcpId];

            const dist_tip = dist(tip, wrist);
            const dist_mcp = dist(mcp, wrist);

            // Heuristic from Python: ratio = dist_tip / dist_mcp. curl = clip((1.8 - ratio)/1.0)
            const ratio = dist_tip / (dist_mcp + 1e-6);
            return Math.min(Math.max((1.8 - ratio) / 1.0, 0.0), 1.0);
        }
    };


    // --- 5. Robot Store Update ---
    const updateRobotControl = (landmarks: any[], handedness: any) => {
        // Only process Right Hand if preferred, or generic.
        // Python code filtered for "Right". Here we can just take the first hand.

        const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];
        const curls: Record<string, number> = {};

        fingers.forEach(f => {
            curls[f] = calculateFingerCurl(landmarks, f);
        });

        // Loop through joints and map curls
        // Similar heuristics to Python patch_main.py:
        // "thumb" -> thumb curl
        // "index" -> index curl...
        // Control Direction: Thumb Positive, others Negative (from Python v3 fix)
        // Gain: In frontend we don't simulate torque (yet), we set POSITION (angle).
        // So we need to map Curl (0~1) to Angle limit range.

        // Strategy:
        // 1. Find all joints.
        // 2. Identify which finger they belong to.
        // 3. Map Curl 0.0 -> LowerLimit, Curl 1.0 -> UpperLimit (or usage of Gain).
        // In Python, Curl (0-1) became CMD. CMD -> Torque.
        // But here we are setting `currentValues` which is Position.
        // Ideally: Curl 0 (Open) -> 0 Angle. Curl 1 (Closed) -> 90 Angle (1.57 rad).

        // Thumb: Positive rotation (Open=0, Closed=Positive?)
        // Python "Control Direction Inversion": 
        // Thumb: Cmd = Curl * 2.0 (Pos)
        // Others: Cmd = Curl * -2.0 (Neg)

        // Interpretation for Position Control:
        // Thumb: Curl 0 -> 0 rad. Curl 1 -> +2.0 rad (approx).
        // Others: Curl 0 -> 0 rad. Curl 1 -> -2.0 rad (approx).

        Object.values(joints).forEach(joint => {
            const lowerName = joint.name.toLowerCase();

            // Skip 1st joint roll if implemented (Python logic: '1st' in lower and 'pitch' not in lower -> continue)
            // Here `dof` handles channels. If pitch is enabled, we drive pitch.
            // If roll is enabled, and it's 1st joint, we should probably keep it 0.

            let matchedFinger = fingers.find(f => lowerName.includes(f));
            if (!matchedFinger) {
                // Fallback or 'general'
                return;
            }

            // FILTER: 1st joint only PITCH
            // If joint name has "1st" (e.g. index_1st_knuckle), we only want Pitch.
            const isFirst = lowerName.includes('1st');

            const curl = curls[matchedFinger];

            // Determine Target Angle based on Python gains
            // Thumb: +2.0 * curl
            // Others: -2.0 * curl
            const gain = (matchedFinger === 'thumb') ? 2.0 : -2.0;
            const targetAngle = curl * gain;

            // Apply to JOINT
            // We assume mostly Rotational joints using Pitch or Roll depending on setup.
            // Usually Finger Flexion is Pitch (Y) or Roll (X) depending on axis.
            // Let's assume Pitch based on previous context.

            if (joint.type === 'rotational') {
                // Apply to Pitch
                if (joint.dof.pitch) {
                    updateJoint(joint.id, 'currentValues.pitch', targetAngle);
                }
                // If setup uses Roll for flexion (common in some URDFs), apply there too?
                // Let's stick to Python filter logic: "1st joint ignore non-pitch".
                if (isFirst) {
                    // Only Pitch Allowed for 1st
                    if (joint.dof.roll) updateJoint(joint.id, 'currentValues.roll', 0); // Lock Roll
                    if (joint.dof.yaw) updateJoint(joint.id, 'currentValues.yaw', 0);
                } else {
                    // 2nd/3rd joints: Usually one DOF (Pitch).
                    // If they have Roll enabled (unlikely), maybe apply same angle?
                    // Safe bet: Apply to main axis.
                    if (joint.dof.roll && !joint.dof.pitch) {
                        // If only Roll exists, use it.
                        updateJoint(joint.id, 'currentValues.roll', targetAngle);
                    }
                }
            }
        });
    };

    // --- Helper: Draw ---
    const drawLandmarks = (landmarks: any[]) => {
        const canvas = canvasRef.current;
        const video = videoRef.current;
        if (!canvas || !video) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        ctx.fillStyle = "#00FF00";
        landmarks.forEach(lm => {
            const x = lm.x * canvas.width;
            const y = lm.y * canvas.height;
            ctx.beginPath();
            ctx.arc(x, y, 3, 0, 2 * Math.PI);
            ctx.fill();
        });
    };

    return (
        <div className="absolute bottom-4 right-4 w-72 bg-gray-900 rounded-lg shadow-2xl border border-gray-700 overflow-hidden flex flex-col z-50">
            {/* Header */}
            <div className="bg-gray-800 p-2 px-3 flex justify-between items-center border-b border-gray-700">
                <div className="flex items-center space-x-2">
                    <Video size={16} className="text-green-500" />
                    <span className="text-sm font-bold text-gray-200">Hand Control</span>
                </div>
                <div className="flex items-center space-x-2">
                    <span className="text-xs text-gray-500">{fps} FPS</span>
                    <button onClick={onClose} className="text-gray-400 hover:text-white">
                        <XCircle size={16} />
                    </button>
                </div>
            </div>

            {/* Video Area */}
            <div className="relative aspect-video bg-black">
                {isLoading && (
                    <div className="absolute inset-0 flex items-center justify-center text-xs text-gray-500">
                        Loading model...
                    </div>
                )}
                <video ref={videoRef} className="absolute inset-0 w-full h-full object-cover transform -scale-x-100" autoPlay playsInline muted />
                <canvas ref={canvasRef} className="absolute inset-0 w-full h-full transform -scale-x-100" />
            </div>

            {/* Controls */}
            <div className="p-2 flex justify-between items-center bg-gray-800 text-xs">
                <span className={`px-2 py-0.5 rounded ${webcamRunning ? 'bg-green-900 text-green-300' : 'bg-yellow-900 text-yellow-300'}`}>
                    {statusMsg}
                </span>

                <button
                    onClick={webcamRunning ? stopWebcam : startWebcam}
                    className="p-1.5 bg-gray-700 hover:bg-gray-600 rounded text-white"
                >
                    {webcamRunning ? <Pause size={14} /> : <Play size={14} />}
                </button>
            </div>
        </div>
    );
};

export default HandControl;
