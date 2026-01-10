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
    const [draggingId, setDraggingId] = useState<string | null>(null);
    const [dragType, setDragType] = useState<'keyframe' | 'velocity'>('keyframe');
    const [velocityDragId, setVelocityDragId] = useState<string | null>(null);

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

        if (draggingId && dragType === 'keyframe') {
            const relativeX = e.clientX - rect.left + containerRef.current.scrollLeft;
            const newTime = Math.max(0, pxToMs(relativeX - 20)); // -20 padding
            onUpdateTiming(draggingId, Math.round(newTime));
        } else if (velocityDragId && dragType === 'velocity') {
            const kfIndex = recording.keyframes.findIndex(k => k.id === velocityDragId);
            if (kfIndex === -1) return;
            const kf = recording.keyframes[kfIndex];
            const nextKf = recording.keyframes[kfIndex + 1];
            if (!nextKf) return;

            const segmentDuration = nextKf.timestamp - kf.timestamp;
            const relativeY = rect.bottom - e.clientY - 40; // Approx height from bottom track
            // Map Y to Speed Factor. 
            // Y=0 -> Speed 1x (Duration = Segment)
            // Y=50 -> Speed Max (Duration = Min)

            const maxSpeed = 5; // Max 5x speed
            const speedFactor = Math.max(1, 1 + (relativeY / 15)); // 15px per speed unit roughly

            const newDuration = Math.max(10, segmentDuration / speedFactor);
            // If newDuration is close to segmentDuration (e.g. speed < 1.1), snap to segmentDuration
            const finalDuration = (newDuration > segmentDuration * 0.9) ? undefined : Math.round(newDuration);

            onUpdateTransition(velocityDragId, finalDuration);
        }
    };

    const handleMouseUp = () => {
        setDraggingId(null);
        setVelocityDragId(null);
    };

    useEffect(() => {
        if (draggingId || velocityDragId) {
            window.addEventListener('mousemove', handleMouseMove);
            window.addEventListener('mouseup', handleMouseUp);
            return () => {
                window.removeEventListener('mousemove', handleMouseMove);
                window.removeEventListener('mouseup', handleMouseUp);
            };
        }
    }, [draggingId, velocityDragId, zoom, recording.keyframes]);

    return (
        <div
            className="fixed bottom-4 left-4 right-4 bg-gray-900 border border-gray-700 rounded-lg shadow-xl flex flex-col z-50 h-64 select-none"
        >
            {/* Header */}
            <div className="flex items-center justify-between p-2 bg-gray-800 border-b border-gray-700 rounded-t-lg">
                <span className="text-sm font-bold text-gray-200">Timeline Editor: {recording.name || 'Untitled'}</span>
                <div className="flex items-center space-x-2">
                    <button onClick={() => setZoom(z => Math.max(10, z * 0.8))} className="p-1 hover:text-white text-gray-400"><ZoomOut size={16} /></button>
                    <span className="text-xs text-gray-500 w-12 text-center">{Math.round(zoom)}%</span>
                    <button onClick={() => setZoom(z => Math.min(500, z * 1.2))} className="p-1 hover:text-white text-gray-400"><ZoomIn size={16} /></button>
                    <div className="w-px h-4 bg-gray-600 mx-2" />
                    <button onClick={onClose} className="p-1 hover:text-red-400 text-gray-400"><X size={16} /></button>
                </div>
            </div>

            {/* Timeline Track */}
            <div
                ref={containerRef}
                className="flex-1 overflow-x-auto overflow-y-hidden relative bg-gray-900 p-4"
                onWheel={handleWheel}
                onClick={(e) => {
                    // Only seek if not dragging
                    if (draggingId || velocityDragId) return;
                    const rect = e.currentTarget.getBoundingClientRect();
                    const x = e.clientX - rect.left + e.currentTarget.scrollLeft;
                    onSeek(pxToMs(x - 20));
                }}
            >
                {/* Ruler markings could go here */}

                {/* Playhead */}
                <div
                    className="absolute top-0 bottom-0 w-0.5 bg-red-500 z-10 pointer-events-none"
                    style={{ left: 20 + msToPx(currentTime) }}
                />

                <div className="relative h-full mt-10" style={{ width: Math.max(2000, msToPx(recording.duration + 5000)) }}>
                    {/* Base Line */}
                    <div className="absolute bottom-6 left-0 right-0 h-0.5 bg-gray-700" />

                    {recording.keyframes.map((kf, i) => {
                        const nextKf = recording.keyframes[i + 1];
                        const x = msToPx(kf.timestamp);
                        const nextX = nextKf ? msToPx(nextKf.timestamp) : x;
                        const segmentDuration = nextKf ? nextKf.timestamp - kf.timestamp : 0;
                        const duration = kf.transitionDuration ?? segmentDuration;

                        // Calculate Height based on Speed
                        // Speed = Segment / Duration
                        // H = (Speed - 1) * Scale
                        const speed = segmentDuration > 0 ? (segmentDuration / duration) : 1;
                        const height = (speed - 1) * 15; // 15px per 1x speed

                        return (
                            <React.Fragment key={kf.id}>
                                {/* Transition Bar (Velocity Handle) */}
                                {nextKf && (
                                    <>
                                        {/* Wait Time Indicator (Base) */}
                                        <div
                                            className="absolute bottom-6 h-1 bg-gray-800"
                                            style={{ left: x, width: nextX - x }}
                                        />

                                        {/* Velocity Bar */}
                                        <div
                                            className={`absolute h-1 cursor-ns-resize z-10 hover:bg-blue-400 ${velocityDragId === kf.id ? 'bg-blue-400' : 'bg-blue-600'}`}
                                            style={{
                                                left: x,
                                                width: msToPx(duration),
                                                bottom: 24 + height, // Base 24px + Variable Height
                                                transition: velocityDragId === kf.id ? 'none' : 'bottom 0.2s'
                                            }}
                                            onMouseDown={(e) => {
                                                e.stopPropagation();
                                                setVelocityDragId(kf.id);
                                                setDragType('velocity');
                                            }}
                                            title={`Speed: ${speed.toFixed(1)}x\nDrag Up/Down to change`}
                                        >
                                            {/* Connector Lines */}
                                            <div className="absolute top-1 left-0 w-px bg-blue-600/30" style={{ height: height + 10 }} />
                                            <div className="absolute top-1 right-0 w-px bg-blue-600/30" style={{ height: height + 10 }} />
                                        </div>
                                    </>
                                )}

                                {/* Keyframe Node */}
                                <div
                                    className={`absolute bottom-4 w-4 h-4 -ml-2 rounded-full border-2 cursor-pointer transition-colors z-20 ${draggingId === kf.id ? 'bg-blue-500 border-white scale-110' : 'bg-gray-800 border-blue-400 hover:bg-blue-400'
                                        }`}
                                    style={{ left: x }}
                                    onMouseDown={(e) => {
                                        e.stopPropagation();
                                        setDraggingId(kf.id);
                                        setDragType('keyframe');
                                    }}
                                    title={`Keyframe #${i + 1}\n${kf.timestamp}ms`}
                                >
                                    <div className="absolute -top-6 left-1/2 -translate-x-1/2 text-[10px] text-gray-500 whitespace-nowrap">
                                        {(kf.timestamp / 1000).toFixed(1)}s
                                    </div>
                                </div>
                            </React.Fragment>
                        );
                    })}
                </div>
            </div>
        </div>
    );
};

export default TimelineEditor;
