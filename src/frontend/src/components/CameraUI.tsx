// src/frontend/src/components/CameraUI.tsx
import React from 'react';
import { useRobotStore } from '../store';
import { Plus, Minus, Move, RotateCw } from 'lucide-react';

export const CameraUI: React.FC = () => {
    const { cameraMode, setCameraMode, zoomIn, zoomOut } = useRobotStore();

    const uiStyle: React.CSSProperties = {
        position: 'fixed',
        bottom: '20px',
        left: '20px',
        width: '12rem', // Equivalent to w-48
        zIndex: 100,
    };

    return (
        <div style={uiStyle}>
            <div className="bg-gray-800 bg-opacity-80 p-1 rounded-lg flex justify-around items-center text-white">
                <button 
                    onClick={() => setCameraMode("pan")} 
                    className={`p-2 rounded ${cameraMode === 'pan' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`}
                    title="Pan Mode (Arrow Keys)"
                >
                    <Move size={16} />
                </button>
                <button 
                    onClick={() => setCameraMode("rotate")} 
                    className={`p-2 rounded ${cameraMode === 'rotate' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`}
                    title="Rotate Mode (Arrow Keys)"
                >
                    <RotateCw size={16} />
                </button>
                <div className="w-px h-6 bg-gray-500 mx-1"></div>
                <button onClick={(e) => { e.stopPropagation(); zoomIn(); }} className="p-2 rounded bg-gray-700 hover:bg-gray-600" title="Zoom In">
                    <Plus size={16} />
                </button>
                <button onClick={(e) => { e.stopPropagation(); zoomOut(); }} className="p-2 rounded bg-gray-700 hover:bg-gray-600" title="Zoom Out">
                    <Minus size={16} />
                </button>
            </div>
        </div>
    );
};
