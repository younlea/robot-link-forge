import React, { useState, useRef, useEffect } from 'react';
import { MotionRecording, MotionKeyframe } from '../types';
import { ZoomIn, ZoomOut, X } from 'lucide-react';

interface TimelineEditorProps {
    recording: MotionRecording;
    onUpdateTiming: (id: string, timestamp: number) => void;
    onUpdateTransition: (id: string, duration: number | undefined) => void;
    onClose: () => void;
    onSeek: (time: number) => void;
    currentTime: number;
}

const TimelineEditor: React.FC<TimelineEditorProps> = ({
    recording,
    onUpdateTiming,
    onUpdateTransition,
    onClose,
    onSeek,
    currentTime
}) => {
    const [zoom, setZoom] = useState(100); // pixels per second
    const containerRef = useRef<HTMLDivElement>(null);

    // Drag States
    const [draggingId, setDraggingId] = useState<string | null>(null);
    const [velocityDragId, setVelocityDragId] = useState<string | null>(null);
    const [isPanning, setIsPanning] = useState(false);

    // Mouse Tracking for Pan vs Click
    const dragStart = useRef<{ x: number, y: number, scrollLeft: number } | null>(null);
    const [dragType, setDragType] = useState<'keyframe' | 'velocity' | 'pan' | null>(null);

    const msToPx = (ms: number) => (ms / 1000) * zoom;
    const pxToMs = (px: number) => (px / zoom) * 1000;

    const handleWheel = (e: React.WheelEvent) => {
        // Direct Zoom with Wheel
        e.preventDefault();
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        setZoom(z => Math.max(10, Math.min(500, z * delta)));
    };

    // Drag Logic
    const handleMouseMove = (e: MouseEvent) => {
        if (!containerRef.current) return;
        const rect = containerRef.current.getBoundingClientRect();

        if (dragType === 'keyframe' && draggingId) {
            const relativeX = e.clientX - rect.left + containerRef.current.scrollLeft;
            const newTime = Math.max(0, pxToMs(relativeX - 20)); // -20 padding
            onUpdateTiming(draggingId, Math.round(newTime));
        }
        else if (dragType === 'velocity' && velocityDragId) {
            const kfIndex = recording.keyframes.findIndex(k => k.id === velocityDragId);
            if (kfIndex === -1) return;
            const kf = recording.keyframes[kfIndex];
            const nextKf = recording.keyframes[kfIndex + 1];
            if (!nextKf) return;

            const segmentDuration = nextKf.timestamp - kf.timestamp;
            // Calculations relative to raised baseline
            // Baseline is at bottom: 40px (approx) inside the track area. 
            // We use clientY delta or relative Y from bottom of rect.
            const relativeY = rect.bottom - e.clientY - 60; // Baseline raised to ~60px

            // Map Y to Speed Factor. 
            const speedFactor = Math.max(1, 1 + (relativeY / 15)); // 15px per speed unit roughly
            const newDuration = Math.max(10, segmentDuration / speedFactor);
            const finalDuration = (newDuration > segmentDuration * 0.9) ? undefined : Math.round(newDuration);
            onUpdateTransition(velocityDragId, finalDuration);
        }
        else if (dragType === 'pan' && dragStart.current) {
            // Panning Logic: Move scroll based on delta
            const deltaX = e.clientX - dragStart.current.x;
            containerRef.current.scrollLeft = dragStart.current.scrollLeft - deltaX;

            // If moved significantly, mark as panning to prevent Seek on MouseUp
            if (Math.abs(deltaX) > 5) {
                setIsPanning(true);
            }
        }
    };

    const handleMouseUp = (e: MouseEvent) => {
        // Handle Seek if it was a Click (Pan with no movement)
        if (dragType === 'pan' && !isPanning && containerRef.current) {
            const rect = containerRef.current.getBoundingClientRect();
            const x = e.clientX - rect.left + containerRef.current.scrollLeft;
            onSeek(pxToMs(x - 20));
        }

        setDraggingId(null);
        setVelocityDragId(null);
        setDragType(null);
        setIsPanning(false);
        dragStart.current = null;
    };

    useEffect(() => {
        if (dragType) {
            window.addEventListener('mousemove', handleMouseMove);
            window.addEventListener('mouseup', handleMouseUp);
            return () => {
                window.removeEventListener('mousemove', handleMouseMove);
                window.removeEventListener('mouseup', handleMouseUp);
            };
        }
    }, [dragType, draggingId, velocityDragId, zoom, recording.keyframes, isPanning]);

    return (
        <div
            className="fixed bottom-4 left-[340px] right-[340px] bg-gray-900 border border-gray-700 rounded-lg shadow-xl flex flex-col z-50 h-72 select-none"
        >
            {/* Timeline Track Area */}
            <div
                ref={containerRef}
                className="flex-1 overflow-x-auto overflow-y-hidden relative bg-gray-900 p-4 cursor-grab active:cursor-grabbing"
                onWheel={handleWheel}
                onMouseDown={(e) => {
                    // Start Panning on background click
                    if ((e.target as HTMLElement) === e.currentTarget || (e.target as HTMLElement).classList.contains('timeline-bg')) {
                        setDragType('pan');
                        dragStart.current = {
                            x: e.clientX,
                            y: e.clientY,
                            scrollLeft: e.currentTarget.scrollLeft
                        };
                        setIsPanning(false);
                    }
                }}
            >
                {/* Background Click Target (Full Area) - Implicitly represented by container, 
                    but we ensure elements don't block it unless interactive */}

                {/* Playhead */}
                <div
                    className="absolute top-0 bottom-0 w-0.5 bg-red-500 z-10 pointer-events-none"
                    style={{ left: 20 + msToPx(currentTime) }}
                />

                <div className="relative h-full mt-10 timeline-bg" style={{ width: Math.max(2000, msToPx(recording.duration + 5000)) }}>
                    {/* Base Line (Time Axis) */}
                    <div className="absolute bottom-12 left-0 right-0 h-0.5 bg-gray-700 pointer-events-none" />

                    {recording.keyframes.map((kf, i) => {
                        const nextKf = recording.keyframes[i + 1];
                        const x = msToPx(kf.timestamp);
                        const nextX = nextKf ? msToPx(nextKf.timestamp) : x;
                        const segmentDuration = nextKf ? nextKf.timestamp - kf.timestamp : 0;
                        const duration = kf.transitionDuration ?? segmentDuration;

                        const speed = segmentDuration > 0 ? (segmentDuration / duration) : 1;
                        const height = (speed - 1) * 15;

                        return (
                            <React.Fragment key={kf.id}>
                                {nextKf && (
                                    <>
                                        {/* Wait Time Indicator (Base) */}
                                        <div
                                            className="absolute bottom-12 h-1 bg-gray-800"
                                            style={{ left: x, width: nextX - x }}
                                        />

                                        {/* Velocity Bar */}
                                        <div
                                            className={`absolute h-1 cursor-ns-resize z-20 hover:bg-blue-400 ${velocityDragId === kf.id ? 'bg-blue-400' : 'bg-blue-600'}`}
                                            style={{
                                                left: x,
                                                width: msToPx(duration),
                                                bottom: 30 + height, // Raised Baseline (12 + 18 padding approx) -> 30+
                                                transition: velocityDragId === kf.id ? 'none' : 'bottom 0.1s'
                                            }}
                                            onMouseDown={(e) => {
                                                e.stopPropagation();
                                                setVelocityDragId(kf.id);
                                                setDragType('velocity');
                                            }}
                                            title={`Speed: ${speed.toFixed(1)}x\nDrag Up/Down to change`}
                                        >
                                            <div className="absolute top-1 left-0 w-px bg-blue-600/30" style={{ height: height + 10 }} />
                                            <div className="absolute top-1 right-0 w-px bg-blue-600/30" style={{ height: height + 10 }} />
                                        </div>
                                    </>
                                )}

                                {/* Keyframe Node */}
                                <div
                                    className={`absolute bottom-10 w-4 h-4 -ml-2 rounded-full border-2 cursor-pointer transition-colors z-30 ${draggingId === kf.id ? 'bg-blue-500 border-white scale-110' : 'bg-gray-800 border-blue-400 hover:bg-blue-400'
                                        }`}
                                    style={{ left: x }}
                                    onMouseDown={(e) => {
                                        e.stopPropagation();
                                        setDraggingId(kf.id);
                                        setDragType('keyframe');
                                    }}
                                    title={`Keyframe #${i + 1}\n${kf.timestamp}ms`}
                                >
                                    <div className="absolute -top-6 left-1/2 -translate-x-1/2 text-[10px] text-gray-400 whitespace-nowrap pointer-events-none">
                                        {(kf.timestamp / 1000).toFixed(1)}s
                                    </div>
                                </div>
                            </React.Fragment>
                        );
                    })}
                </div>
            </div>

            {/* Footer / Controls (Moved from Top) */}
            <div className="flex items-center justify-between p-2 bg-gray-800 border-t border-gray-700 rounded-b-lg">
                <span className="text-xs font-medium text-gray-400 px-2">
                    {recording.name || 'Untitled Recording'}
                </span>
                <div className="flex items-center space-x-2">
                    <span className="text-[10px] text-gray-500 mr-2">
                        Scroll/Drag to Pan • Wheel to Zoom
                    </span>
                    <button onClick={() => setZoom(z => Math.max(10, z * 0.8))} className="p-1 hover:text-white text-gray-400"><ZoomOut size={14} /></button>
                    <span className="text-xs text-gray-500 w-10 text-center">{Math.round(zoom)}%</span>
                    <button onClick={() => setZoom(z => Math.min(500, z * 1.2))} className="p-1 hover:text-white text-gray-400"><ZoomIn size={14} /></button>
                    <div className="w-px h-3 bg-gray-600 mx-2" />
                    <button onClick={onClose} className="p-1 hover:text-red-400 text-gray-400"><X size={14} /></button>
                </div>
            </div>
        </div>
    );
};

export default TimelineEditor;
