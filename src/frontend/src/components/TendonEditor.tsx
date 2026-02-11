import React, { useState } from 'react';
import { useRobotStore } from '../store';
import { Tendon } from '../types';

const TendonEditor = () => {
    const {
        tendons, activeTendonId,
        addTendon, updateTendon, deleteTendon, removeTendonRoutingPoint,
        setActiveTendonId, setInteractionMode, interactionMode, links
    } = useRobotStore();

    const [isExpanded, setIsExpanded] = useState(false);
    const tendonList = Object.values(tendons);

    const handleStartRouting = (tendonId: string) => {
        setActiveTendonId(tendonId);
        setInteractionMode('tendon-routing');
    };

    const handleStopRouting = () => {
        setInteractionMode('select');
    };

    return (
        <div className="mt-4 border-t border-gray-700 pt-4">
            <button
                onClick={() => setIsExpanded(!isExpanded)}
                className="flex items-center justify-between w-full text-sm font-bold text-gray-300 hover:text-white"
            >
                <span>🧵 Tendon System ({tendonList.length})</span>
                <span className="text-xs">{isExpanded ? '▲' : '▼'}</span>
            </button>

            {isExpanded && (
                <div className="mt-2 space-y-3">
                    {/* Add Tendon Buttons */}
                    <div className="flex space-x-2">
                        <button
                            onClick={() => addTendon('active')}
                            className="flex-1 bg-orange-600 hover:bg-orange-700 p-2 rounded text-xs text-white"
                        >
                            + Active Tendon
                        </button>
                        <button
                            onClick={() => addTendon('passive')}
                            className="flex-1 bg-blue-600 hover:bg-blue-700 p-2 rounded text-xs text-white"
                        >
                            + Passive Tendon
                        </button>
                    </div>

                    {/* Active Routing Mode Indicator */}
                    {interactionMode === 'tendon-routing' && activeTendonId && (
                        <div className="bg-orange-900/40 border border-orange-600 rounded p-2">
                            <p className="text-xs text-orange-300 font-semibold">
                                🎯 Routing Mode Active
                            </p>
                            <p className="text-xs text-gray-400 mt-1">
                                Click on link surfaces in the 3D view to add routing points.
                            </p>
                            <button
                                onClick={handleStopRouting}
                                className="mt-2 w-full bg-gray-700 hover:bg-gray-600 p-1 rounded text-xs"
                            >
                                ✓ Done Routing
                            </button>
                        </div>
                    )}

                    {/* Tendon List */}
                    {tendonList.map(tendon => (
                        <TendonItem
                            key={tendon.id}
                            tendon={tendon}
                            isActive={activeTendonId === tendon.id}
                            onStartRouting={() => handleStartRouting(tendon.id)}
                            onStopRouting={handleStopRouting}
                            isRouting={interactionMode === 'tendon-routing' && activeTendonId === tendon.id}
                        />
                    ))}

                    {tendonList.length === 0 && (
                        <p className="text-xs text-gray-500 text-center py-2">
                            No tendons yet. Add one above.
                        </p>
                    )}
                </div>
            )}
        </div>
    );
};

const TendonItem = ({
    tendon, isActive, onStartRouting, onStopRouting, isRouting
}: {
    tendon: Tendon;
    isActive: boolean;
    onStartRouting: () => void;
    onStopRouting: () => void;
    isRouting: boolean;
}) => {
    const { updateTendon, deleteTendon, removeTendonRoutingPoint, links, setActiveTendonId } = useRobotStore();
    const [isExpanded, setIsExpanded] = useState(false);

    const getLinkName = (linkId: string) => links[linkId]?.name || linkId.slice(0, 8);

    return (
        <div className={`p-2 rounded text-xs space-y-2 ${isActive ? 'bg-gray-700 border border-orange-600' : 'bg-gray-900/50'
            }`}>
            {/* Header */}
            <div className="flex items-center justify-between">
                <button
                    onClick={() => { setActiveTendonId(tendon.id); setIsExpanded(!isExpanded); }}
                    className="flex items-center space-x-2 text-left flex-1"
                >
                    <span className={`w-3 h-3 rounded-full ${tendon.type === 'active' ? 'bg-orange-500' : 'bg-blue-500'}`}></span>
                    <span className="font-semibold">{tendon.name}</span>
                    <span className="text-gray-500 text-[10px]">({tendon.type})</span>
                </button>
                <button
                    onClick={() => { if (confirm(`Delete tendon "${tendon.name}"?`)) deleteTendon(tendon.id); }}
                    className="text-red-400 hover:text-red-300 px-1"
                >✕</button>
            </div>

            {isExpanded && (
                <div className="space-y-2 pl-2">
                    {/* Name */}
                    <div className="flex items-center justify-between">
                        <label className="text-gray-400">Name</label>
                        <input
                            type="text"
                            value={tendon.name}
                            onChange={(e) => updateTendon(tendon.id, 'name', e.target.value)}
                            className="w-2/3 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500"
                        />
                    </div>

                    {/* Physics Parameters */}
                    <div className="flex items-center justify-between">
                        <label className="text-gray-400">Stiffness (N/m)</label>
                        <input
                            type="number" step={1} value={tendon.stiffness}
                            onChange={(e) => updateTendon(tendon.id, 'stiffness', parseFloat(e.target.value) || 0)}
                            className="w-2/3 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500"
                        />
                    </div>

                    <div className="flex items-center justify-between">
                        <label className="text-gray-400">Damping (N·s/m)</label>
                        <input
                            type="number" step={0.01} value={tendon.damping}
                            onChange={(e) => updateTendon(tendon.id, 'damping', parseFloat(e.target.value) || 0)}
                            className="w-2/3 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500"
                        />
                    </div>

                    <div className="flex items-center justify-between">
                        <label className="text-gray-400">Rest Length (m)</label>
                        <input
                            type="number" step={0.01} value={tendon.restLength}
                            onChange={(e) => updateTendon(tendon.id, 'restLength', parseFloat(e.target.value) || 0)}
                            className="w-2/3 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500"
                        />
                    </div>

                    {tendon.type === 'active' && (
                        <div className="flex items-center justify-between">
                            <label className="text-gray-400">Moment Arm (m)</label>
                            <input
                                type="number" step={0.001} value={tendon.momentArm ?? 0.01}
                                onChange={(e) => updateTendon(tendon.id, 'momentArm', parseFloat(e.target.value) || 0)}
                                className="w-2/3 bg-gray-900 rounded p-1 text-xs focus:outline-none focus:ring-1 focus:ring-blue-500"
                            />
                        </div>
                    )}

                    {/* Routing Points */}
                    <div className="border-t border-gray-800 pt-2">
                        <div className="flex justify-between items-center mb-1">
                            <span className="text-gray-400 font-semibold">Routing Points ({tendon.routingPoints.length})</span>
                            {!isRouting ? (
                                <button
                                    onClick={onStartRouting}
                                    className="bg-orange-600 hover:bg-orange-700 px-2 py-0.5 rounded text-[10px]"
                                >
                                    + Add Points
                                </button>
                            ) : (
                                <button
                                    onClick={onStopRouting}
                                    className="bg-green-600 hover:bg-green-700 px-2 py-0.5 rounded text-[10px]"
                                >
                                    ✓ Done
                                </button>
                            )}
                        </div>

                        {tendon.routingPoints.map((point, idx) => (
                            <div key={point.id} className="flex items-center justify-between bg-gray-900 rounded p-1 mb-1">
                                <span className="text-gray-500">{idx + 1}.</span>
                                <span className="text-gray-300 flex-1 ml-1 truncate" title={point.linkId}>
                                    {getLinkName(point.linkId)}
                                </span>
                                <span className="text-gray-500 text-[10px] mx-1">
                                    [{point.localPosition.map(v => v.toFixed(3)).join(', ')}]
                                </span>
                                <button
                                    onClick={() => removeTendonRoutingPoint(tendon.id, point.id)}
                                    className="text-red-400 hover:text-red-300 ml-1"
                                >✕</button>
                            </div>
                        ))}

                        {tendon.routingPoints.length === 0 && (
                            <p className="text-gray-600 text-[10px] text-center py-1">
                                Click "Add Points" then click links in 3D view
                            </p>
                        )}
                    </div>

                    {/* Color */}
                    <div className="flex items-center justify-between">
                        <label className="text-gray-400">Color</label>
                        <input
                            type="color"
                            value={tendon.color}
                            onChange={(e) => updateTendon(tendon.id, 'color', e.target.value)}
                            className="w-12 h-6 bg-gray-900 rounded border border-gray-700 cursor-pointer"
                        />
                    </div>
                </div>
            )}
        </div>
    );
};

export default TendonEditor;
