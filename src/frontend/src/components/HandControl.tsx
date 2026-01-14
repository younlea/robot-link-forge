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

    // Draggable Logic
    const [position, setPosition] = useState({ x: 16, y: 80 }); // Default top-left
    const isDragging = useRef(false);
    const dragOffset = useRef({ x: 0, y: 0 });

    const handleMouseDown = (e: React.MouseEvent) => {
        isDragging.current = true;
        dragOffset.current = {
            x: e.clientX - position.x,
            y: e.clientY - position.y
        };
    };

    useEffect(() => {
        const handleMouseMove = (e: MouseEvent) => {
            if (isDragging.current) {
                setPosition({
                    x: e.clientX - dragOffset.current.x,
                    y: e.clientY - dragOffset.current.y
                });
            }
        };

        const handleMouseUp = () => {
            isDragging.current = false;
        };

        window.addEventListener('mousemove', handleMouseMove);
        window.addEventListener('mouseup', handleMouseUp);

        return () => {
            window.removeEventListener('mousemove', handleMouseMove);
            window.removeEventListener('mouseup', handleMouseUp);
        };
    }, []);

    // Store Access
    const { updateJoint, links, joints, isRecording, recordingMode, captureKeyframe } = useRobotStore();

    // Init Refs for Recording to avoid loop staleness
    const isRecordingRef = useRef(isRecording);
    const recordingModeRef = useRef(recordingMode);

    useEffect(() => { isRecordingRef.current = isRecording; }, [isRecording]);
    useEffect(() => { recordingModeRef.current = recordingMode; }, [recordingMode]);

    // Logger
    const addLog = (msg: string) => {
        setLogs(prev => [`[${new Date().toLocaleTimeString().split(' ')[0]}] ${msg}`, ...prev].slice(0, 50));
    };

    // --- 1. Initialize MediaPipe ---
    // --- 1. Initialize MediaPipe ---
    const initInProgress = useRef(false);
    const isMountedRef = useRef(true); // Track mount status
    const webcamRunningRef = useRef(false); // Track webcam status in ref for loop

    useEffect(() => {
        isMountedRef.current = true;
        return () => { isMountedRef.current = false; };
    }, []);

    useEffect(() => {
        webcamRunningRef.current = webcamRunning;
    }, [webcamRunning]);

    const initMediaPipe = async () => {
        if (initInProgress.current) return;
        initInProgress.current = true;

        try {
            if (!isMountedRef.current) return;

            setLogs(prev => [`[${new Date().toLocaleTimeString()}] Init...`, ...prev]);
            addLog("Loading MediaPipe Vision (Local)...");

            // Close existing if any (manual restart case)
            if (handLandmarker) {
                handLandmarker.close();
                setHandLandmarker(null);
            }

            const vision = await FilesetResolver.forVisionTasks(
                "/mediapipe"
            );

            if (!isMountedRef.current) return;

            const landmarker = await HandLandmarker.createFromOptions(vision, {
                baseOptions: {
                    modelAssetPath: `/mediapipe/hand_landmarker.task`,
                    delegate: "CPU" // Linux compatible
                },
                runningMode: "VIDEO",
                numHands: 1
            });

            if (!isMountedRef.current) {
                landmarker.close();
                return;
            }

            console.log("MediaPipe Loaded Successfully");
            addLog("MediaPipe Ready.");
            setHandLandmarker(landmarker);
            setIsLoading(false);

            // Auto-start webcam if ready
            if (!webcamRunning && isMountedRef.current) startWebcam();
        } catch (error) {
            console.error("Failed to load MediaPipe:", error);
            const errorMsg = error instanceof Error ? error.message : JSON.stringify(error);
            addLog(`Error: ${errorMsg}`);
            setIsLoading(false);
        } finally {
            initInProgress.current = false;
        }
    };

    useEffect(() => {
        initMediaPipe();

        return () => {
            stopWebcam();
        };
    }, []);

    // Track landmarker in ref for cleanup
    const landmarkerRef = useRef<HandLandmarker | null>(null);
    useEffect(() => { landmarkerRef.current = handLandmarker; }, [handLandmarker]);

    // Cleanup on unmount
    useEffect(() => {
        return () => {
            if (landmarkerRef.current) {
                console.log("Closing MediaPipe Landmarker...");
                landmarkerRef.current.close();
            }
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
    const lastLoopLogTimeRef = useRef(0);

    const predictWebcam = async () => {
        try {
            // Loop Log (Throttled 5s) - Moved to top for debug
            const now = Date.now();
            if (now - lastLoopLogTimeRef.current >= 5000) {
                addLog("Loop Alive");
                lastLoopLogTimeRef.current = now;
            }

            // Stop if component unmounted or webcam stopped
            // FIX: Use Refs for safe access in closure
            if (!isMountedRef.current || !webcamRunningRef.current) {
                return;
            }

            if (!videoRef.current || !videoRef.current.videoWidth) {
                // Retry if valid but not ready
                if (webcamRunningRef.current && isMountedRef.current) {
                    requestRef.current = requestAnimationFrame(predictWebcam);
                }
                return;
            }

            // FPS Calculation
            fpsCountRef.current++;
            if (now - fpsTimeRef.current >= 1000) {
                setFps(fpsCountRef.current);
                fpsCountRef.current = 0;
                fpsTimeRef.current = now;
            }

            const video = videoRef.current;

            // Debug Preconditions (Throttle this log?)
            // Only run AI if enabled and model loaded
            // FIX: Use ref to avoid closure staleness
            const landmarkerStart = landmarkerRef.current;

            if (enableAIRef.current && landmarkerStart && video.currentTime !== lastVideoTimeRef.current) {
                const startTimeMs = performance.now();
                try {
                    const results = landmarkerStart.detectForVideo(video, startTimeMs);
                    lastVideoTimeRef.current = video.currentTime;

                    if (results.landmarks && results.landmarks.length > 0) {
                        if (fpsCountRef.current % 60 === 0) {
                            addLog(`Hands: ${results.landmarks.length}`);
                        }
                        drawLandmarks(results.landmarks[0]);
                        updateRobotControl(results.landmarks[0], results.handedness[0]);

                        // --- Camera Recording Capture ---
                        if (isRecordingRef.current && recordingModeRef.current === 'camera') {
                            captureKeyframe();
                        }
                        // --------------------------------
                    } else {
                        if (fpsCountRef.current % 120 === 0) {
                            addLog("No hands detected");
                        }
                        clearCanvas();
                    }
                } catch (e) {
                    // Inner catch for AI errors
                    console.error(e);
                    const msg = e instanceof Error ? e.message : JSON.stringify(e);
                    if (!logs[0]?.includes(msg.substring(0, 20))) {
                        addLog(`AI Error: ${msg}`);
                    }
                }
            } else if (!enableAIRef.current) {
                clearCanvas();
            }

            if (webcamRunningRef.current && isMountedRef.current) {
                requestRef.current = requestAnimationFrame(predictWebcam);
            }
        } catch (globalError) {
            console.error("CRITICAL LOOP ERROR:", globalError);
            addLog(`Loop Crash: ${globalError}`);
            // Attempt recovery
            if (webcamRunningRef.current && isMountedRef.current) {
                requestRef.current = requestAnimationFrame(predictWebcam);
            }
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
            // Geometric Angle Calculation (2D Projection - Ignore Z for stability)
            const cmc = landmarks[THUMB_CMC];
            const mcp = landmarks[THUMB_MCP];
            const ip = landmarks[THUMB_IP];

            // Vec 1: CMC -> MCP (XY only)
            const v1 = { x: mcp.x - cmc.x, y: mcp.y - cmc.y };
            // Vec 2: MCP -> IP (XY only)
            const v2 = { x: ip.x - mcp.x, y: ip.y - mcp.y };

            const dot = v1.x * v2.x + v1.y * v2.y;
            const mag1 = Math.sqrt(v1.x * v1.x + v1.y * v1.y);
            const mag2 = Math.sqrt(v2.x * v2.x + v2.y * v2.y);

            const cos = Math.min(Math.max(dot / (mag1 * mag2), -1), 1);
            const angle = Math.acos(cos); // Radians

            // Normalize 0..PI/2 -> 0..1
            return Math.min(Math.max(angle / (Math.PI / 2), 0.0), 1.0);
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

            // Uniform Sensitivity for all non-thumb fingers (User reported mismatch)
            // Previously tuned to 1.6 for pinky, reverting to 1.8 to match others.
            const maxRatio = 1.8;

            return Math.min(Math.max((maxRatio - ratio) / 1.0, 0.0), 1.0);
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
            if (!matchedFinger) {
                if (lowerName.includes('little')) matchedFinger = 'pinky';
            }

            // Debug: Periodic log of un-matched joints?
            // if (fpsCountRef.current % 120 === 0 && !matchedFinger) {
            //      console.log(`Joint ${joint.name} not controlled (no finger match)`);
            // }

            if (!matchedFinger) return;

            const isFirst = lowerName.includes('1st');
            const curl = curls[matchedFinger];

            // Smart Gain Direction based on Limits
            let dir = -1.0;
            let currentLimit = null; // Store active limit for clamping

            if (joint.limits) {
                // Determine active axis
                if (joint.dof.pitch) {
                    currentLimit = joint.limits.pitch;
                    // Fallback: If Pitch limits are default symmetric [-PI, PI], 
                    // but joint is physically Yaw (axis Z) with restricted Yaw limits, use Yaw.
                    if (currentLimit.lower < -3 && currentLimit.upper > 3 && joint.limits.yaw.upper !== undefined) {
                        // Check if Yaw is restricted
                        if (Math.abs(joint.limits.yaw.lower) < 3 || Math.abs(joint.limits.yaw.upper) < 3) {
                            currentLimit = joint.limits.yaw;
                        }
                    }
                }
                else if (joint.dof.yaw) currentLimit = joint.limits.yaw;
                else if (joint.dof.roll) currentLimit = joint.limits.roll;

                if (currentLimit) {
                    if (currentLimit.lower >= 0) dir = 1.0;
                    else if (currentLimit.upper <= 0) dir = -1.0;
                }
            }

            const gain = 1.57 * dir;
            let targetAngle = curl * gain;

            // Explicit Clamping to Limits
            if (currentLimit) {
                targetAngle = Math.max(currentLimit.lower, Math.min(currentLimit.upper, targetAngle));
            }

            // STRICT Direction Clamping (Safety Net for "Jumping")
            // If we decided direction is Negative, NEVER allow Positive.
            // If Positive, NEVER allow Negative.
            if (dir < 0) targetAngle = Math.min(0, targetAngle);
            if (dir > 0) targetAngle = Math.max(0, targetAngle);

            // Debug Thumb Glitches
            if (matchedFinger === 'thumb' && fpsCountRef.current % 30 === 0) {
                // console.log(`Thumb ${joint.name}: Curl=${curl.toFixed(2)} Dir=${dir} Tgt=${targetAngle.toFixed(2)}`);
            }

            if (joint.type === 'rotational') {
                if (joint.dof.pitch) {
                    const currentPitch = (joint.currentValues as any).pitch || 0;
                    // Optimization: Only update if change is significant (> 0.05 rad ~ 3 deg)
                    if (Math.abs(currentPitch - targetAngle) > 0.05) {
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
        if (!ctx) {
            console.error("Canvas Context Lost");
            return;
        }

        // Sync local dimensions to video source dimensions
        if (canvas.width !== video.videoWidth || canvas.height !== video.videoHeight) {
            canvas.width = video.videoWidth;
            canvas.height = video.videoHeight;
        }

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Debug: Draw a visible constant marker (Top-Left)
        // ctx.fillStyle = "red";
        // ctx.fillRect(10, 10, 50, 50);

        ctx.fillStyle = "#00FF00";
        landmarks.forEach(lm => {
            const x = lm.x * canvas.width;
            const y = lm.y * canvas.height;
            ctx.beginPath();
            ctx.arc(x, y, 5, 0, 2 * Math.PI); // Increased size 3->5
            ctx.fill();
        });

        // Debug Log Data
        // if (fpsCountRef.current % 60 === 0) {
        //    console.log("Landmark 0:", landmarks[0]);
        // }
    };

    const restartAI = async () => {
        addLog("Restarting AI...");
        await initMediaPipe();
        // Kickstart loop if it died
        if (webcamRunningRef.current && isMountedRef.current) {
            addLog("Restarting Loop...");
            predictWebcam();
        }
    };

    return (
        <div
            className="fixed w-72 bg-gray-900 rounded-lg shadow-2xl border border-gray-700 overflow-hidden flex flex-col z-50"
            style={{ left: position.x, top: position.y }}
        >
            {/* Header */}
            <div
                className="bg-gray-800 p-2 px-3 flex justify-between items-center border-b border-gray-700 cursor-move"
                onMouseDown={handleMouseDown}
            >
                <div className="flex items-center space-x-2 pointer-events-none">
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
            <div className="bg-gray-950 p-2 h-32 overflow-y-auto text-[10px] font-mono border-t border-gray-800 flex flex-col-reverse">
                {logs.map((log, i) => (
                    <div key={i} className="text-gray-400 break-words whitespace-pre-wrap border-b border-gray-900 last:border-0 py-0.5">
                        {log}
                    </div>
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
                    onClick={restartAI}
                    className="ml-2 px-2 py-0.5 bg-blue-900 text-blue-200 rounded text-[10px] hover:bg-blue-800"
                >
                    Restart AI
                </button>

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
