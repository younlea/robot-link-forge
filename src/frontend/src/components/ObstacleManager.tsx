import React, { useState } from 'react';
import { useRobotStore } from '../store';
import { Obstacle } from '../types';

const NumberInput = ({ label, value, onChange, step = 0.01 }: { label: string, value: number, onChange: (val: number) => void, step?: number }) => (
    <div className="flex items-center justify-between mb-1">
        <label className="text-gray-400 text-xs w-1/3">{label}</label>
        <input type="number" step={step} value={value}
            onChange={(e) => onChange(parseFloat(e.target.value) || 0)}
            onFocus={(e) => e.target.select()}
            className="w-2/3 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500" />
    </div>
);

const Vec3Input = ({ label, value, onChange }: { label: string, value: [number, number, number], onChange: (v: [number, number, number]) => void }) => (
    <div className="mb-1">
        <p className="text-xs text-gray-400 mb-1">{label}</p>
        <div className="flex space-x-1">
            {['X', 'Y', 'Z'].map((axis, i) => (
                <div key={axis} className="flex-1">
                    <input type="number" step={0.01} value={value[i]}
                        onChange={(e) => {
                            const newVal = [...value] as [number, number, number];
                            newVal[i] = parseFloat(e.target.value) || 0;
                            onChange(newVal);
                        }}
                        onFocus={(e) => e.target.select()}
                        className="w-full bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500"
                        title={axis}
                    />
                </div>
            ))}
        </div>
    </div>
);

const ObstacleManager = () => {
    const { obstacles, addObstacle, updateObstacle, deleteObstacle, setInteractionMode, interactionMode } = useRobotStore();
    const [isExpanded, setIsExpanded] = useState(false);
    const obstacleList = Object.values(obstacles);

    const toggleDragMode = () => {
        if (interactionMode === 'obstacle-drag') {
            setInteractionMode('select');
        } else {
            setInteractionMode('obstacle-drag');
        }
    };

    return (
        <div className="mt-4 border-t border-gray-700 pt-4">
            <button
                onClick={() => setIsExpanded(!isExpanded)}
                className="flex items-center justify-between w-full text-sm font-bold text-gray-300 hover:text-white"
            >
                <span>🪨 Obstacles ({obstacleList.length})</span>
                <span className="text-xs">{isExpanded ? '▲' : '▼'}</span>
            </button>

            {isExpanded && (
                <div className="mt-2 space-y-3">
                    {/* Add Obstacle Buttons */}
                    <div className="flex space-x-1">
                        <button onClick={() => addObstacle('box')}
                            className="flex-1 bg-red-800 hover:bg-red-700 p-1.5 rounded text-xs text-white">
                            + Box
                        </button>
                        <button onClick={() => addObstacle('sphere')}
                            className="flex-1 bg-red-800 hover:bg-red-700 p-1.5 rounded text-xs text-white">
                            + Sphere
                        </button>
                        <button onClick={() => addObstacle('cylinder')}
                            className="flex-1 bg-red-800 hover:bg-red-700 p-1.5 rounded text-xs text-white">
                            + Cylinder
                        </button>
                    </div>

                    {/* Drag Mode Toggle */}
                    <div className="flex items-center justify-between">
                        <span className="text-xs text-gray-400">3D Drag Mode</span>
                        <button
                            onClick={toggleDragMode}
                            className={`px-3 py-1 rounded text-xs font-semibold ${
                                interactionMode === 'obstacle-drag'
                                    ? 'bg-blue-600 text-white'
                                    : 'bg-gray-700 hover:bg-gray-600 text-gray-300'
                            }`}
                        >
                            {interactionMode === 'obstacle-drag' ? 'Active' : 'Inactive'}
                        </button>
                    </div>

                    <p className="text-[10px] text-gray-500">
                        Fixed obstacles for contact simulation. Toggle enabled/disabled without deleting.
                        {interactionMode === 'obstacle-drag' && ' Click and drag obstacles in 3D view.'}
                    </p>

                    {/* Obstacle List */}
                    {obstacleList.map(obs => (
                        <ObstacleItem key={obs.id} obstacle={obs} />
                    ))}

                    {obstacleList.length === 0 && (
                        <p className="text-xs text-gray-500 text-center py-2">
                            No obstacles yet. Add one above.
                        </p>
                    )}
                </div>
            )}
        </div>
    );
};

const ObstacleItem = ({ obstacle }: { obstacle: Obstacle }) => {
    const { updateObstacle, deleteObstacle } = useRobotStore();
    const [isExpanded, setIsExpanded] = useState(false);

    return (
        <div className={`p-2 rounded text-xs space-y-2 ${obstacle.enabled ? 'bg-gray-900/50' : 'bg-gray-900/30 opacity-60'}`}>
            {/* Header */}
            <div className="flex items-center justify-between">
                <button
                    onClick={() => setIsExpanded(!isExpanded)}
                    className="flex items-center space-x-2 text-left flex-1"
                >
                    <span className="font-semibold">{obstacle.name}</span>
                    <span className="text-gray-500 text-[10px]">({obstacle.shape})</span>
                </button>
                <div className="flex items-center space-x-1">
                    <label className="cursor-pointer" title={obstacle.enabled ? 'Disable' : 'Enable'}>
                        <input
                            type="checkbox"
                            checked={obstacle.enabled}
                            onChange={(e) => updateObstacle(obstacle.id, 'enabled', e.target.checked)}
                            className="h-3 w-3 rounded"
                        />
                    </label>
                    <button
                        onClick={() => { if (confirm(`Delete "${obstacle.name}"?`)) deleteObstacle(obstacle.id); }}
                        className="text-red-400 hover:text-red-300 px-1"
                    >✕</button>
                </div>
            </div>

            {isExpanded && (
                <div className="space-y-2 pl-1">
                    {/* Name */}
                    <div className="flex items-center justify-between mb-1">
                        <label className="text-gray-400 text-xs">Name</label>
                        <input
                            type="text" value={obstacle.name}
                            onChange={(e) => updateObstacle(obstacle.id, 'name', e.target.value)}
                            className="w-2/3 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500"
                        />
                    </div>

                    {/* Transform */}
                    <Vec3Input label="Position (m)" value={obstacle.position as [number, number, number]}
                        onChange={(v) => updateObstacle(obstacle.id, 'position', v)} />
                    <Vec3Input label="Rotation RPY (rad)" value={obstacle.rotation as [number, number, number]}
                        onChange={(v) => updateObstacle(obstacle.id, 'rotation', v)} />
                    <Vec3Input label="Dimensions (m)" value={obstacle.dimensions as [number, number, number]}
                        onChange={(v) => updateObstacle(obstacle.id, 'dimensions', v)} />

                    {/* Color */}
                    <div className="flex items-center justify-between">
                        <label className="text-gray-400 text-xs">Color</label>
                        <input
                            type="color" value={obstacle.color}
                            onChange={(e) => updateObstacle(obstacle.id, 'color', e.target.value)}
                            className="w-12 h-6 bg-gray-900 rounded border border-gray-700 cursor-pointer"
                        />
                    </div>

                    {/* Physics */}
                    <div className="border-t border-gray-800 pt-2">
                        <p className="text-gray-400 font-semibold mb-1">Physics</p>
                        <NumberInput label="Friction" value={obstacle.physics.friction}
                            onChange={(v) => updateObstacle(obstacle.id, 'physics.friction', v)} step={0.1} />

                        <div className="flex items-center justify-between mb-1">
                            <label className="text-gray-400 text-xs w-1/3">Solref</label>
                            <div className="flex space-x-1 w-2/3">
                                <input type="number" step={0.01} value={obstacle.physics.solref[0]}
                                    onChange={(e) => updateObstacle(obstacle.id, 'physics.solref[0]', parseFloat(e.target.value) || 0)}
                                    className="w-1/2 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500" />
                                <input type="number" step={0.1} value={obstacle.physics.solref[1]}
                                    onChange={(e) => updateObstacle(obstacle.id, 'physics.solref[1]', parseFloat(e.target.value) || 0)}
                                    className="w-1/2 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500" />
                            </div>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
};

export default ObstacleManager;
