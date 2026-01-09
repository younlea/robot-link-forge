import React, { useEffect, useRef, useState } from 'react';
import { HandLandmarker, FilesetResolver } from '@mediapipe/tasks-vision';
import { useRobotStore } from '../store';
import { XCircle, Video, Play, Pause } from 'lucide-react';

/*
  HandControl.tsx
  - Handles Webcam Stream
  - Runs MediaPipe Hand Landmark Detection
  - Robust Init & Logging
*/

const HandControl = ({ onClose }: { onClose: () => void }) => {
    const videoRef = useRef<HTMLVideoElement>(null);
    const canvasRef = useRef<HTMLCanvasElement>(null);

    // MediaPipe State
    const [handLandmarker, setHandLandmarker] = useState<HandLandmarker | null>(null);
    const [webcamRunning, setWebcamRunning] = useState(false);
    const [enableAI, setEnableAI] = useState(true);
    const enableAIRef = useRef(true); // Ref to avoid closure staleness in loop

    // Sync Ref
    useEffect(() => { enableAIRef.current = enableAI; }, [enableAI]);

    const [isLoading, setIsLoading] = useState(true);
    const [fps, setFps] = useState(0);
    const [logs, setLogs] = useState<string[]>(['Initializing component...']);

    // Store Access
    const { updateJoint, links, joints } = useRobotStore();

    // Logger
    const addLog = (msg: string) => {
        setLogs(prev => [`[${new Date().toLocaleTimeString().split(' ')[0]}] ${msg}`, ...prev.slice(0, 4)]);
    };

    // --- 1. Initialize MediaPipe ---
    useEffect(() => {
        let isMounted = true;

        const initMediaPipe = async () => {
            try {
                addLog("Loading MediaPipe Vision (Local)...");
                const vision = await FilesetResolver.forVisionTasks(
                    "/mediapipe"
                );

                if (!isMounted) return;

                const landmarker = await HandLandmarker.createFromOptions(vision, {
                    baseOptions: {
                        modelAssetPath: `/mediapipe/hand_landmarker.task`,
                        delegate: "CPU" // Linux compatible
                    },
                    runningMode: "VIDEO",
                    numHands: 1
                });

                if (!isMounted) return;

                console.log("MediaPipe Loaded Successfully");
                addLog("MediaPipe Ready.");
                setHandLandmarker(landmarker);
                setIsLoading(false);

                // Auto-start webcam if ready
                startWebcam();
            } catch (error) {
                console.error("Failed to load MediaPipe:", error);
                const errorMsg = error instanceof Error ? error.message : JSON.stringify(error);
                addLog(`Error: ${errorMsg}`);
                setIsLoading(false);
            }
        };

        // Delay slightly to ensure render
        setTimeout(initMediaPipe, 100);

        return () => {
            isMounted = false;
            stopWebcam();
        };
    }, []);

    // --- 2. Webcam Handling ---
    const startWebcam = async () => {
        try {
            addLog("Requesting Camera...");
            const stream = await navigator.mediaDevices.getUserMedia({ video: true });

            if (videoRef.current) {
                videoRef.current.srcObject = stream;
                videoRef.current.addEventListener("loadeddata", () => {
                    addLog("Camera Stream Started");
                    predictWebcam();
                });
            }
            setWebcamRunning(true);
        } catch (err) {
            console.error("Webcam Error:", err);
            addLog("Camera Permission Denied!");
        }
    };

    const stopWebcam = () => {
        if (videoRef.current && videoRef.current.srcObject) {
            const stream = videoRef.current.srcObject as MediaStream;
            stream.getTracks().forEach(track => track.stop());
            videoRef.current.srcObject = null;
        }
        setWebcamRunning(false);
        addLog("Camera Stopped");
    };

    // --- 3. Prediction Loop ---
    const lastVideoTimeRef = useRef(-1);
    const requestRef = useRef<number>();
    const fpsTimeRef = useRef(Date.now());
    const fpsCountRef = useRef(0);

    const predictWebcam = async () => {
        // Stop if component unmounted or webcam stopped
        if (!videoRef.current || !videoRef.current.videoWidth) {
            if (webcamRunning) requestRef.current = requestAnimationFrame(predictWebcam);
            return;
        }

        // FPS Calculation
        const now = Date.now();
        fpsCountRef.current++;
        if (now - fpsTimeRef.current >= 1000) {
            setFps(fpsCountRef.current);
            fpsCountRef.current = 0;
            fpsTimeRef.current = now;
        }

        const video = videoRef.current;

        // Debug Preconditions (Throttle this log?)
        if (enableAIRef.current && !handLandmarker && fpsCountRef.current % 60 === 0) {
            addLog("AI: Waiting for model to load...");
        }

        // Only run AI if enabled and model loaded
        if (enableAIRef.current && handLandmarker && video.currentTime !== lastVideoTimeRef.current) {
            const startTimeMs = performance.now();
            try {
                const results = handLandmarker.detectForVideo(video, startTimeMs);
                lastVideoTimeRef.current = video.currentTime;

                if (results.landmarks && results.landmarks.length > 0) {
                    drawLandmarks(results.landmarks[0]);
                    updateRobotControl(results.landmarks[0], results.handedness[0]);
                } else {
                    clearCanvas();
                }
            } catch (e) {
                console.error(e);
                const msg = e instanceof Error ? e.message : JSON.stringify(e);
                // Only log unique errors to avoid flooding
                if (!logs[0]?.includes(msg.substring(0, 20))) {
                    addLog(`AI Error: ${msg}`);
                }
            }
        } else if (!enableAIRef.current) {
            clearCanvas();
        }

        if (webcamRunning) {
            requestRef.current = requestAnimationFrame(predictWebcam);
        }
    };

    const clearCanvas = () => {
        if (canvasRef.current) {
            const ctx = canvasRef.current.getContext('2d');
            if (ctx) ctx.clearRect(0, 0, canvasRef.current.width, canvasRef.current.height);
        }
    };

    useEffect(() => {
        if (webcamRunning) {
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

            const ratio = dist_tip / (dist_mcp + 1e-6);
            return Math.min(Math.max((1.8 - ratio) / 1.0, 0.0), 1.0);
        }
    };


    // --- 5. Robot Store Update ---
    const updateRobotControl = (landmarks: any[], handedness: any) => {
        const fingers = ['thumb', 'index', 'middle', 'ring', 'pinky'];
        const curls: Record<string, number> = {};

        fingers.forEach(f => {
            curls[f] = calculateFingerCurl(landmarks, f);
        });

        Object.values(joints).forEach(joint => {
            const lowerName = joint.name.toLowerCase();
            let matchedFinger = fingers.find(f => lowerName.includes(f));

            // Debug: Periodic log of un-matched joints?
            // if (fpsCountRef.current % 120 === 0 && !matchedFinger) {
            //      console.log(`Joint ${joint.name} not controlled (no finger match)`);
            // }

            if (!matchedFinger) return;

            const isFirst = lowerName.includes('1st');
            const curl = curls[matchedFinger];
            const gain = (matchedFinger === 'thumb') ? 2.0 : -2.0;
            const targetAngle = curl * gain;

            if (joint.type === 'rotational') {
                if (joint.dof.pitch) {
                    const currentPitch = (joint.currentValues as any).pitch || 0;
                    // Optimization: Only update if change is significant (> 0.05 rad ~ 3 deg)
                    if (Math.abs(currentPitch - targetAngle) > 0.05) {
                        if (matchedFinger === 'index' && isFirst && fpsCountRef.current % 60 === 0) {
                            addLog(`Ctrl: ${joint.name} -> ${targetAngle.toFixed(2)}`);
                        }
                        updateJoint(joint.id, 'currentValues.pitch', targetAngle);
                    }
                }
                if (isFirst) {
                    // Locks
                    if (joint.dof.roll && Math.abs((joint.currentValues as any).roll || 0) > 0.01) updateJoint(joint.id, 'currentValues.roll', 0);
                    if (joint.dof.yaw && Math.abs((joint.currentValues as any).yaw || 0) > 0.01) updateJoint(joint.id, 'currentValues.yaw', 0);
                } else {
                    if (joint.dof.roll && !joint.dof.pitch) {
                        const currentRoll = (joint.currentValues as any).roll || 0;
                        if (Math.abs(currentRoll - targetAngle) > 0.05) {
                            updateJoint(joint.id, 'currentValues.roll', targetAngle);
                        }
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
                    <Video size={16} className={`${webcamRunning ? 'text-green-500' : 'text-gray-400'}`} />
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

            {/* Logs Area */}
            <div className="bg-gray-950 p-2 h-20 overflow-y-auto text-[10px] font-mono border-t border-gray-800">
                {logs.map((log, i) => (
                    <div key={i} className="text-gray-400 truncate">{log}</div>
                ))}
            </div>

            {/* Controls */}
            <div className="p-2 flex justify-between items-center bg-gray-800 text-xs">
                <label className="flex items-center space-x-2 cursor-pointer">
                    <input
                        type="checkbox"
                        checked={enableAI}
                        onChange={(e) => setEnableAI(e.target.checked)}
                        className="form-checkbox text-blue-500 rounded h-3 w-3"
                    />
                    <span className="text-gray-300">Enable AI</span>
                </label>

                <button
                    onClick={webcamRunning ? stopWebcam : startWebcam}
                    className={`p-1.5 rounded text-white ${webcamRunning ? 'bg-red-900 hover:bg-red-800' : 'bg-green-700 hover:bg-green-600'}`}
                >
                    {webcamRunning ? <Pause size={14} /> : <Play size={14} />}
                </button>
            </div>
        </div>
    );
};

export default HandControl;
