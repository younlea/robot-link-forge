import { useState, useEffect, useRef } from 'react';
import { useRobotStore } from '../store';
import { RecordingMode } from '../types';
import {
    Video,
    Camera,
    SlidersHorizontal,
    Hand,
    Gamepad2,
    Circle,
    Square,
    Play,
    Pause,
    SkipBack,
    Trash2,
    Plus,
    X,
    Clock,
    RefreshCw,
    Pencil,
    Save,
    CalendarClock,
    HardDrive,
    Cloud,
    Download,
    Upload
} from 'lucide-react';
import TimelineEditor from './TimelineEditor';

interface RecordingPanelProps {
    onClose: () => void;
}

const RecordingPanel = ({ onClose }: RecordingPanelProps) => {
    const {
        recordingMode,
        isRecording,
        currentRecording,
        recordings,
        playbackState,
        joints,
        setRecordingMode,
        startRecording,
        stopRecording,
        captureKeyframe,
        deleteKeyframe,
        updateKeyframeTiming,
        playRecording,
        pausePlayback,
        stopPlayback,
        seekPlayback,
        deleteRecording,
        renameRecording,
        updateJoint,
        updateKeyframePose,
        loadKeyframePose,
        editRecording,
        saveRecording,
        cancelEditRecording,
        updateKeyframeTransition,
        updateRecordingMetadata,
        deleteRecordingFromServer,

        // Persistence
        saveRecordingsLocal,
        loadRecordingsLocal,
        fetchRecordingList,
        saveRecordingsToServer,
        loadRecordingsFromServer,
    } = useRobotStore();

    const [recordingName, setRecordingName] = useState('');
    const [renamingId, setRenamingId] = useState<string | null>(null);
    const playbackRef = useRef<number | null>(null);

    const [showTimeline, setShowTimeline] = useState(true);

    // Draggable Logic
    const [position, setPosition] = useState({ x: 16, y: window.innerHeight - 300 }); // Default nearby bottom-left
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

    // Catmull-Rom Spline Interpolation
    const catmullRom = (p0: number, p1: number, p2: number, p3: number, t: number): number => {
        const t2 = t * t;
        const t3 = t2 * t;
        return 0.5 * (
            (2 * p1) +
            (-p0 + p2) * t +
            (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
            (-p0 + 3 * p1 - 3 * p2 + p3) * t3
        );
    };

    const interpolateJointValues = (recording: typeof currentRecording, timeMs: number) => {
        if (!recording || recording.keyframes.length < 2) return;

        const keyframes = recording.keyframes;

        // Find surrounding keyframes
        let idx = 0;
        for (let i = 0; i < keyframes.length - 1; i++) {
            if (timeMs >= keyframes[i].timestamp && timeMs <= keyframes[i + 1].timestamp) {
                idx = i;
                break;
            }
        }

        const kf0 = keyframes[Math.max(0, idx - 1)];
        const kf1 = keyframes[idx];
        const kf2 = keyframes[Math.min(keyframes.length - 1, idx + 1)];
        const kf3 = keyframes[Math.min(keyframes.length - 1, idx + 2)];

        const segmentDuration = kf2.timestamp - kf1.timestamp;
        const moveDuration = kf1.transitionDuration ?? segmentDuration;
        const localTime = timeMs - kf1.timestamp;

        // If we exceeded moveDuration, we hold at kf2 (t=1). 
        // Note: For Catmull-Rom, t=1 yields p2 (end point).
        const t = moveDuration > 0 ? Math.min(1, localTime / moveDuration) : 1;

        // Apply interpolated values to each joint
        Object.keys(joints).forEach(jointId => {
            const v0 = kf0.jointValues[jointId];
            const v1 = kf1.jointValues[jointId];
            const v2 = kf2.jointValues[jointId];
            const v3 = kf3.jointValues[jointId];

            if (!v1 || !v2) return;

            // Interpolate each DOF
            ['roll', 'pitch', 'yaw', 'displacement'].forEach(dof => {
                const p0 = v0?.[dof as keyof typeof v0] ?? v1[dof as keyof typeof v1];
                const p1 = v1[dof as keyof typeof v1];
                const p2 = v2[dof as keyof typeof v2];
                const p3 = v3?.[dof as keyof typeof v3] ?? v2[dof as keyof typeof v2];

                const interpolated = catmullRom(p0, p1, p2, p3, Math.max(0, Math.min(1, t)));
                updateJoint(jointId, `currentValues.${dof}`, interpolated);
            });
        });
    };

    // Playback animation loop
    useEffect(() => {
        if (playbackState.isPlaying && playbackState.recordingId) {
            // Prefer currentRecording if it matches the ID (actively editing), otherwise find in saved list
            const recording = (currentRecording && currentRecording.id === playbackState.recordingId)
                ? currentRecording
                : recordings.find(r => r.id === playbackState.recordingId);

            if (!recording || recording.keyframes.length < 2) {
                stopPlayback();
                return;
            }

            const startTime = Date.now();
            const initialTime = playbackState.currentTime;

            const animate = () => {
                const elapsed = Date.now() - startTime;
                const newTime = initialTime + elapsed;

                if (newTime >= recording.duration) {
                    // Apply final keyframe
                    interpolateJointValues(recording, recording.duration);
                    stopPlayback();
                    return;
                }

                seekPlayback(newTime);
                interpolateJointValues(recording, newTime);

                playbackRef.current = requestAnimationFrame(animate);
            };

            playbackRef.current = requestAnimationFrame(animate);

            return () => {
                if (playbackRef.current) {
                    cancelAnimationFrame(playbackRef.current);
                }
            };
        }
    }, [playbackState.isPlaying, playbackState.recordingId]);

    // Live update when stopped (scrubbing or editing)
    useEffect(() => {
        if (!playbackState.isPlaying && currentRecording && !isRecording) {
            interpolateJointValues(currentRecording, playbackState.currentTime);
        }
    }, [playbackState.isPlaying, playbackState.currentTime, currentRecording, isRecording]);

    const modes: { id: RecordingMode; label: string; icon: any; disabled: boolean }[] = [
        { id: 'slider', label: 'Slider', icon: Video, disabled: false },
        { id: 'camera', label: 'Camera', icon: Camera, disabled: false },
        { id: 'input_device', label: 'Glove', icon: Gamepad2, disabled: true },
    ];

    const handleStartRecording = () => {
        if (!recordingMode) {
            setRecordingMode('slider');
        }
        startRecording(recordingName || undefined);
        setRecordingName('');
    };

    const formatTime = (ms: number) => {
        const secs = Math.floor(ms / 1000);
        const millis = ms % 1000;
        return `${secs}.${millis.toString().padStart(3, '0')}s`;
    };

    // New state and handlers for storage menu
    const [showStorageMenu, setShowStorageMenu] = useState(false);

    // Server Dialog State
    const [serverDialogMode, setServerDialogMode] = useState<'none' | 'save' | 'load'>('none');
    const [serverFiles, setServerFiles] = useState<string[]>([]);
    const [serverFilename, setServerFilename] = useState('');

    // Fetch list and open dialog
    const openServerDialog = async (mode: 'save' | 'load') => {
        const files = await fetchRecordingList();
        setServerFiles(files);
        setServerDialogMode(mode);
        if (mode === 'save') {
            // Default filename suggestion
            setServerFilename(`recordings_${new Date().toISOString().slice(0, 10)}`);
        }
        setShowStorageMenu(false); // Close the small menu
    };

    const handleServerSaveConfirm = async () => {
        if (!serverFilename.trim()) return;
        if (serverFiles.includes(`${serverFilename}.json`) || serverFiles.includes(serverFilename)) {
            if (!confirm(`Overwrite existing file "${serverFilename}"?`)) return;
        }
        await saveRecordingsToServer(serverFilename);
        setServerDialogMode('none');
    };

    const handleServerLoadConfirm = async (file: string) => {
        if (confirm(`Replace all current recordings with content from "${file}"?`)) {
            await loadRecordingsFromServer(file);
            setServerDialogMode('none');
        }
    };

    const handleDeleteFile = async (file: string) => {
        if (confirm(`Are you sure you want to delete "${file}" from the server?`)) {
            await deleteRecordingFromServer(file);
            // Refresh list
            const list = await fetchRecordingList();
            setServerFiles(list);
        }
    };

    const handleLocalUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.files && e.target.files[0]) {
            loadRecordingsLocal(e.target.files[0]);
            setShowStorageMenu(false);
        }
    };

    return (
        <>
            {/* Unified Server Dialog Overlay */}
            {serverDialogMode !== 'none' && (
                <div className="fixed inset-0 bg-black/50 z-[100] flex items-center justify-center" onClick={() => setServerDialogMode('none')}>
                    <div className="bg-gray-900 border border-gray-700 rounded-lg shadow-2xl w-80 max-h-[80vh] flex flex-col p-4 space-y-4" onClick={e => e.stopPropagation()}>
                        <div className="flex justify-between items-center border-b border-gray-700 pb-2">
                            <h3 className="text-sm font-bold text-white uppercase">
                                {serverDialogMode === 'save' ? 'Save to Server' : 'Load from Server'}
                            </h3>
                            <button onClick={() => setServerDialogMode('none')} className="text-gray-400 hover:text-white">
                                <X size={16} />
                            </button>
                        </div>

                        {/* File List */}
                        <div className="flex-1 overflow-y-auto space-y-1 min-h-[100px] border border-gray-800 rounded bg-gray-950/50 p-2">
                            {serverFiles.length > 0 ? serverFiles.map(f => (
                                <div key={f} className="flex items-center space-x-1 group hover:bg-gray-800 rounded px-2 py-1 cursor-pointer">
                                    <div
                                        className="flex-1 text-xs text-gray-300 truncate"
                                        onClick={() => {
                                            if (serverDialogMode === 'save') {
                                                // Strip .json for input if needed, or just set it
                                                setServerFilename(f.replace('.json', ''));
                                            } else {
                                                handleServerLoadConfirm(f);
                                            }
                                        }}
                                    >
                                        {f}
                                    </div>
                                    <button
                                        onClick={(e) => {
                                            e.stopPropagation();
                                            handleDeleteFile(f);
                                        }}
                                        className="text-gray-600 hover:text-red-400 opacity-0 group-hover:opacity-100 transition-opacity"
                                        title="Delete"
                                    >
                                        <Trash2 size={12} />
                                    </button>
                                </div>
                            )) : (
                                <div className="text-gray-600 text-xs italic text-center mt-4">No files found on server</div>
                            )}
                        </div>

                        {/* Save Mode Input Area */}
                        {serverDialogMode === 'save' && (
                            <div className="space-y-2">
                                <label className="text-xs text-gray-400">Filename</label>
                                <div className="flex space-x-2">
                                    <input
                                        type="text"
                                        value={serverFilename}
                                        onChange={(e) => setServerFilename(e.target.value)}
                                        className="flex-1 bg-gray-800 border border-gray-700 rounded px-2 py-1 text-sm text-white"
                                        placeholder="filename"
                                    />
                                    <button
                                        onClick={handleServerSaveConfirm}
                                        disabled={!serverFilename.trim()}
                                        className="bg-blue-600 hover:bg-blue-500 text-white px-3 py-1 rounded text-sm disabled:opacity-50 disabled:cursor-not-allowed"
                                    >
                                        Save
                                    </button>
                                </div>
                                <div className="text-[10px] text-gray-500 text-right">
                                    Saving {recordings.length} recordings:
                                </div>
                                <div className="max-h-20 overflow-y-auto bg-gray-950/30 rounded p-1 border border-gray-800">
                                    {recordings.map(r => (
                                        <div key={r.id} className="text-[10px] text-gray-400 truncate">
                                            • {r.name} ({formatTime(r.duration)})
                                        </div>
                                    ))}
                                </div>
                            </div>
                        )}
                        {serverDialogMode === 'load' && (
                            <div className="text-[10px] text-gray-500 text-center">
                                Click a file to replace current recordings.
                            </div>
                        )}
                    </div>
                </div>
            )}

            <div
                style={{ top: position.y, left: position.x }}
                className={`cursor-move fixed w-72 bg-gray-900 border border-gray-700 rounded-lg shadow-xl flex flex-col z-40 transition-shadow hover:shadow-2xl ${isDragging.current ? 'opacity-90' : 'opacity-100'}`}
            >
                {/* Header */}
                <div
                    className="flex items-center justify-between p-3 bg-gray-800 border-b border-gray-700 rounded-t-lg"
                    onMouseDown={handleMouseDown}
                >
                    <div className="flex items-center gap-2">
                        <span className="font-bold text-gray-100">Motion Recording</span>
                        {isRecording && <span className="animate-pulse text-red-500 text-xs font-bold">● REC</span>}
                    </div>

                    <div className="flex items-center gap-1">
                        <button
                            onClick={() => setShowStorageMenu(!showStorageMenu)}
                            className={`p-1 rounded hover:bg-gray-700 ${showStorageMenu ? 'text-blue-400' : 'text-gray-400'}`}
                            title="Storage Options"
                            onMouseDown={(e) => e.stopPropagation()}
                        >
                            <HardDrive size={16} />
                        </button>
                        <button onClick={onClose} className="text-gray-400 hover:text-white">
                            <X size={16} />
                        </button>
                    </div>
                </div>

                {/* Storage Menu Overlay (Small Popup) */}
                {showStorageMenu && (
                    <div className="bg-gray-800 border-b border-gray-700 p-2 text-sm space-y-2 animate-in slide-in-from-top-2">
                        <div className="text-xs text-gray-500 font-bold uppercase">Server Storage</div>
                        <div className="flex gap-2">
                            <button onClick={() => openServerDialog('save')} className="flex-1 flex items-center justify-center gap-1 bg-blue-900/50 hover:bg-blue-800 p-1.5 rounded text-blue-200 border border-blue-800">
                                <Cloud size={14} /> Save...
                            </button>
                            <button onClick={() => openServerDialog('load')} className="flex-1 flex items-center justify-center gap-1 bg-gray-700 hover:bg-gray-600 p-1.5 rounded text-gray-200 border border-gray-600">
                                <Cloud size={14} /> Load...
                            </button>
                        </div>

                        <div className="text-xs text-gray-500 font-bold uppercase mt-2">Local File (JSON)</div>
                        <div className="flex gap-2">
                            <button onClick={saveRecordingsLocal} className="flex-1 flex items-center justify-center gap-1 bg-green-900/50 hover:bg-green-800 p-1.5 rounded text-green-200 border border-green-800">
                                <Download size={14} /> Export
                            </button>
                            <label className="flex-1 flex items-center justify-center gap-1 bg-gray-700 hover:bg-gray-600 p-1.5 rounded text-gray-200 border border-gray-600 cursor-pointer">
                                <Upload size={14} /> Import
                                <input type="file" accept=".json" onChange={handleLocalUpload} className="hidden" />
                            </label>
                        </div>
                    </div>
                )}

                {/* Mode Selection */}
                <div className="p-3 border-b border-gray-700">
                    <div className="text-xs text-gray-500 mb-2">Recording Mode</div>
                    <div className="flex space-x-2">
                        <button
                            onClick={() => setRecordingMode('slider')}
                            disabled={isRecording}
                            className={`flex-1 py-2 px-3 rounded text-xs flex flex-col items-center space-y-1 transition-colors ${recordingMode === 'slider'
                                ? 'bg-blue-600 text-white'
                                : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                                }`}
                        >
                            <SlidersHorizontal size={16} />
                            <span>Sliders</span>
                        </button>
                        <button
                            onClick={() => setRecordingMode('live')}
                            disabled={isRecording}
                            className={`flex-1 py-2 px-3 rounded text-xs flex flex-col items-center space-y-1 transition-colors ${recordingMode === 'live'
                                ? 'bg-red-900/50 text-red-200'
                                : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                                }`}
                        >
                            <Camera size={16} />
                            <span>Live</span>
                        </button>
                        <button
                            onClick={() => setRecordingMode('glove')}
                            disabled={isRecording}
                            className={`flex-1 py-2 px-3 rounded text-xs flex flex-col items-center space-y-1 transition-colors ${recordingMode === 'glove'
                                ? 'bg-purple-900/50 text-purple-200'
                                : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                                }`}
                        >
                            <Hand size={16} />
                            <span>Glove</span>
                        </button>
                    </div>
                </div>

                {/* Recording Controls */}
                <div className="p-3 border-b border-gray-700">
                    <div className="flex flex-col space-y-2 mb-2">
                        {/* Name Input */}
                        <input
                            type="text"
                            placeholder="Recording name..."
                            value={recordingName}
                            onChange={(e) => {
                                setRecordingName(e.target.value);
                                // Sync with store immediately if editing
                                if (currentRecording) {
                                    updateRecordingMetadata({ name: e.target.value });
                                }
                            }}
                            disabled={isRecording}
                            className="w-full bg-gray-800 border border-gray-600 rounded px-2 py-1 text-xs text-gray-200 placeholder-gray-500"
                        />

                        {/* Duration Controls (Only when editing/stopped) */}
                        {!isRecording && currentRecording && (
                            <div className="flex items-center space-x-2">
                                <label className="text-xs text-gray-400">Duration (ms):</label>
                                <input
                                    type="number"
                                    value={Math.round(currentRecording.duration || 0)}
                                    onChange={(e) => {
                                        const val = Math.max(0, parseInt(e.target.value) || 0);
                                        updateRecordingMetadata({ duration: val });
                                    }}
                                    className="w-20 bg-gray-800 border border-gray-600 rounded px-2 py-1 text-xs text-gray-200"
                                />
                                <button
                                    onClick={() => {
                                        if (currentRecording.keyframes.length > 0) {
                                            const maxTime = Math.max(...currentRecording.keyframes.map(k => k.timestamp));
                                            updateRecordingMetadata({ duration: maxTime });
                                        }
                                    }}
                                    className="bg-gray-700 hover:bg-gray-600 text-white px-2 py-1 rounded text-xs"
                                    title="Trim duration to end at the last keyframe"
                                >
                                    Trim to End
                                </button>
                            </div>
                        )}
                    </div>

                    <div className="flex space-x-2">
                        {!isRecording ? (
                            <>
                                {currentRecording && (
                                    <>
                                        <button
                                            onClick={saveRecording}
                                            className="flex-1 bg-green-600 hover:bg-green-500 text-white py-2 rounded text-xs flex items-center justify-center space-x-1"
                                            title="Save edited recording"
                                            disabled={!currentRecording.name.trim()}
                                        >
                                            <Save size={12} />
                                            <span>Save</span>
                                        </button>
                                        <button
                                            onClick={() => setShowTimeline(!showTimeline)}
                                            className={`flex-1 ${showTimeline ? 'bg-blue-600 hover:bg-blue-500' : 'bg-gray-600 hover:bg-gray-500'} text-white py-2 rounded text-xs flex items-center justify-center space-x-1`}
                                            title="Toggle Timeline Editor"
                                        >
                                            <CalendarClock size={12} />
                                            <span>Timeline</span>
                                        </button>
                                        <button
                                            onClick={cancelEditRecording}
                                            className="flex-1 bg-gray-600 hover:bg-gray-500 text-white py-2 rounded text-xs flex items-center justify-center space-x-1"
                                            title="Cancel Editing (Exit)"
                                        >
                                            <X size={12} />
                                            <span>Exit</span>
                                        </button>
                                    </>
                                )}
                                <button
                                    onClick={handleStartRecording}
                                    disabled={!recordingMode}
                                    className="flex-1 bg-red-600 hover:bg-red-500 disabled:bg-gray-700 disabled:text-gray-500 text-white py-2 rounded text-xs flex items-center justify-center space-x-1"
                                >
                                    <Circle size={12} fill="currentColor" />
                                    <span>{currentRecording ? 'New Recording' : 'Start Recording'}</span>
                                </button>
                            </>
                        ) : (
                            <>
                                <button
                                    onClick={stopRecording}
                                    className="flex-1 bg-gray-600 hover:bg-gray-500 text-white py-2 rounded text-xs flex items-center justify-center space-x-1"
                                >
                                    <Square size={12} fill="currentColor" />
                                    <span>Stop</span>
                                </button>
                                {recordingMode === 'slider' && (
                                    <button
                                        onClick={captureKeyframe}
                                        className="flex-1 bg-green-600 hover:bg-green-500 text-white py-2 rounded text-xs flex items-center justify-center space-x-1"
                                    >
                                        <Plus size={12} />
                                        <span>Capture</span>
                                    </button>
                                )}
                            </>
                        )}
                    </div>
                </div>

                {/* Current Recording Keyframes */}
                {currentRecording && currentRecording.keyframes.length > 0 && (
                    <div className="p-3 border-b border-gray-700 max-h-40 overflow-y-auto">
                        <div className="text-xs text-gray-500 mb-2">
                            Keyframes ({currentRecording.keyframes.length})
                        </div>
                        <div className="space-y-1">
                            {currentRecording.keyframes.map((kf, idx) => (
                                <div
                                    key={kf.id}
                                    className="flex flex-col bg-gray-800 rounded px-2 py-1 space-y-1"
                                >
                                    <div className="flex items-center justify-between">
                                        <div className="flex items-center space-x-2 cursor-pointer hover:text-blue-400" onClick={() => loadKeyframePose(kf.id)} title="Click to Load Pose">
                                            <span className="text-xs text-gray-400">#{idx + 1}</span>
                                            <div className="flex items-center space-x-1">
                                                <Clock size={10} className="text-gray-500" />
                                                <input
                                                    type="number"
                                                    value={kf.timestamp}
                                                    onChange={(e) => updateKeyframeTiming(kf.id, Number(e.target.value))}
                                                    onClick={(e) => e.stopPropagation()}
                                                    className="w-16 bg-gray-700 border border-gray-600 rounded px-1 py-0.5 text-xs text-gray-200"
                                                />
                                                <span className="text-[10px] text-gray-500">ms</span>
                                            </div>
                                            {/* Transition Duration (Optional) */}
                                            {idx < currentRecording.keyframes.length - 1 && (
                                                <div className="flex items-center space-x-1">
                                                    <span className="text-[9px] text-blue-400 w-3 text-center">Dur</span>
                                                    <input
                                                        type="number"
                                                        value={kf.transitionDuration ?? ''}
                                                        placeholder="Full"
                                                        onChange={(e) => updateKeyframeTransition(kf.id, e.target.value ? Number(e.target.value) : undefined)}
                                                        onClick={(e) => e.stopPropagation()}
                                                        className="w-16 bg-gray-700 border border-gray-600 rounded px-1 py-0.5 text-[10px] text-blue-200 placeholder-gray-500"
                                                        title="Motion Duration (leave empty for full time)"
                                                    />
                                                    <span className="text-[10px] text-gray-500">ms</span>
                                                </div>
                                            )}
                                        </div>
                                        <div className="flex items-center space-x-1">
                                            <button
                                                onClick={() => updateKeyframePose(kf.id)}
                                                className="text-blue-500 hover:text-blue-400 p-1"
                                                title="Update Pose with Current Sliders"
                                            >
                                                <RefreshCw size={12} />
                                            </button>
                                            <button
                                                onClick={() => deleteKeyframe(kf.id)}
                                                className="text-red-500 hover:text-red-400 p-1"
                                            >
                                                <Trash2 size={12} />
                                            </button>
                                        </div>
                                    </div>
                                </div>
                            ))}
                        </div>
                    </div>
                )}

                {/* Playback Controls */}
                {(currentRecording?.keyframes.length || 0) >= 2 && (
                    <div className="p-3 border-b border-gray-700">
                        <div className="text-xs text-gray-500 mb-2">Preview</div>
                        <div className="flex items-center space-x-2">
                            <button
                                onClick={stopPlayback}
                                className="p-1.5 bg-gray-700 hover:bg-gray-600 rounded text-gray-300"
                            >
                                <SkipBack size={14} />
                            </button>
                            {playbackState.isPlaying ? (
                                <button
                                    onClick={pausePlayback}
                                    className="p-1.5 bg-yellow-600 hover:bg-yellow-500 rounded text-white"
                                >
                                    <Pause size={14} />
                                </button>
                            ) : (
                                <button
                                    onClick={() => playRecording()}
                                    className="p-1.5 bg-green-600 hover:bg-green-500 rounded text-white"
                                >
                                    <Play size={14} />
                                </button>
                            )}
                            <div className="flex-1 text-xs text-gray-400 text-center">
                                {formatTime(playbackState.currentTime)} / {formatTime(currentRecording?.duration || 0)}
                            </div>
                        </div>
                    </div>
                )}

                {/* Saved Recordings */}
                {recordings.length > 0 && (
                    <div className="p-3 max-h-40 overflow-y-auto">
                        <div className="text-xs text-gray-500 mb-2">Saved Recordings</div>
                        <div className="space-y-1">
                            {recordings.map(rec => {
                                const isPlayingThis = playbackState.isPlaying && playbackState.recordingId === rec.id;
                                return (
                                    <div
                                        key={rec.id}
                                        className="flex items-center justify-between bg-gray-800 rounded px-2 py-1.5"
                                    >
                                        <div className="flex-1">
                                            {renamingId === rec.id ? (
                                                <div className="flex items-center space-x-1 mr-2">
                                                    <input
                                                        type="text"
                                                        autoFocus
                                                        className="w-full bg-gray-700 border border-blue-500 rounded px-1 py-0.5 text-xs text-white"
                                                        defaultValue={rec.name}
                                                        onKeyDown={(e) => {
                                                            if (e.key === 'Enter') {
                                                                renameRecording(rec.id, e.currentTarget.value);
                                                                setRenamingId(null);
                                                            } else if (e.key === 'Escape') {
                                                                setRenamingId(null);
                                                            }
                                                        }}
                                                        onBlur={() => setRenamingId(null)}
                                                    />
                                                </div>
                                            ) : (
                                                <>
                                                    <div
                                                        className="text-xs text-gray-200 cursor-pointer hover:text-blue-400 truncate"
                                                        onDoubleClick={() => setRenamingId(rec.id)}
                                                        title="Double-click to rename"
                                                    >
                                                        {rec.name}
                                                    </div>
                                                    <div className="text-[10px] text-gray-500">
                                                        {rec.keyframes.length} keyframes · {formatTime(rec.duration)}
                                                    </div>
                                                </>
                                            )}
                                        </div>
                                        <div className="flex items-center space-x-1">
                                            <button
                                                onClick={() => isPlayingThis ? pausePlayback() : playRecording(rec.id)}
                                                className="p-1 text-gray-400 hover:text-white"
                                                title={isPlayingThis ? "Pause" : "Play"}
                                            >
                                                {isPlayingThis ? <Pause size={12} /> : <Play size={12} />}
                                            </button>
                                            <button
                                                onClick={() => editRecording(rec.id)}
                                                className="p-1 text-blue-400 hover:text-blue-300"
                                                title="Edit Recording"
                                            >
                                                <Pencil size={12} />
                                            </button>
                                            <button
                                                onClick={() => deleteRecording(rec.id)}
                                                className="p-1 text-red-500 hover:text-red-400"
                                                title="Delete"
                                            >
                                                <Trash2 size={12} />
                                            </button>
                                        </div>
                                    </div>
                                );
                            })}
                        </div>
                    </div>
                )}
                {/* Timeline Editor Overlay */}
                {currentRecording && !isRecording && showTimeline && (
                    <TimelineEditor
                        recording={currentRecording}
                        onUpdateTiming={updateKeyframeTiming}
                        onUpdateTransition={updateKeyframeTransition}
                        onClose={() => setShowTimeline(false)}
                        currentTime={playbackState.currentTime}
                        onSeek={seekPlayback}
                    />
                )}
            </div>
        </>
    );
};

export default RecordingPanel;
