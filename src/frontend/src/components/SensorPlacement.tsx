import React, { useState } from 'react';
import { useRobotStore } from '../store';
import { SensorDef } from '../types';

const SensorPlacement = () => {
    const { sensors, deleteSensor, setInteractionMode, interactionMode, links, updateSensor } = useRobotStore();
    const [isExpanded, setIsExpanded] = useState(false);
    const [expandedSensors, setExpandedSensors] = useState<Set<string>>(new Set());
    const sensorList = Object.values(sensors);

    const handleStartPlacement = () => {
        setInteractionMode('sensor-placement');
    };

    const handleStopPlacement = () => {
        setInteractionMode('select');
    };

    const toggleSensorExpanded = (sensorId: string) => {
        const newExpanded = new Set(expandedSensors);
        if (newExpanded.has(sensorId)) {
            newExpanded.delete(sensorId);
        } else {
            newExpanded.add(sensorId);
        }
        setExpandedSensors(newExpanded);
    };

    // Helper function to get link name
    const getLinkName = (linkId: string) => {
        return links[linkId]?.name || linkId.slice(0, 8);
    };

    return (
        <div className="mt-4 border-t border-gray-700 pt-4">
            <button
                onClick={() => setIsExpanded(!isExpanded)}
                className="flex items-center justify-between w-full text-sm font-bold text-gray-300 hover:text-white"
            >
                <span>📡 Sensors ({sensorList.length})</span>
                <span className="text-xs">{isExpanded ? '▲' : '▼'}</span>
            </button>

            {isExpanded && (
                <div className="mt-2 space-y-3">
                    {/* Placement Mode */}
                    {interactionMode === 'sensor-placement' ? (
                        <div className="bg-green-900/40 border border-green-600 rounded p-2">
                            <p className="text-xs text-green-300 font-semibold">
                                🎯 Sensor Placement Mode
                            </p>
                            <p className="text-xs text-gray-400 mt-1">
                                Click on link surfaces in the 3D view to place touch sensors.
                            </p>
                            <button
                                onClick={handleStopPlacement}
                                className="mt-2 w-full bg-gray-700 hover:bg-gray-600 p-1 rounded text-xs"
                            >
                                ✓ Done Placing
                            </button>
                        </div>
                    ) : (
                        <button
                            onClick={handleStartPlacement}
                            className="w-full bg-green-700 hover:bg-green-600 p-2 rounded text-xs text-white"
                        >
                            + Place Touch Sensor (Click on Link)
                        </button>
                    )}

                    {/* Sensor List */}
                    {sensorList.map(sensor => {
                        const isSensorExpanded = expandedSensors.has(sensor.id);
                        return (
                            <div key={sensor.id} className="p-2 bg-gray-900/50 rounded text-xs">
                                <div className="flex items-center justify-between">
                                    <div className="flex items-center space-x-2 flex-1">
                                        <span className={`w-2 h-2 rounded-full ${sensor.type === 'touch' ? 'bg-green-500' : 'bg-yellow-500'}`}></span>
                                        <span className="font-semibold">{sensor.siteName}</span>
                                        <button
                                            onClick={() => deleteSensor(sensor.id)}
                                            className="text-red-400 hover:text-red-300 px-1"
                                        >✕</button>
                                    </div>
                                    <div className="text-gray-500 text-[10px] mt-1">
                                        Link: {getLinkName(sensor.linkId)} |
                                        Pos: [{sensor.localPosition.map(v => v.toFixed(3)).join(', ')}]
                                    </div>

                                    {isSensorExpanded && (
                                        <div className="mt-3 space-y-3 border-t border-gray-700 pt-3">
                                            <div>
                                                <label className="block text-gray-400 text-[10px] mb-1">
                                                    Size: {(sensor.size || 0.01).toFixed(3)} m
                                                </label>
                                                <input
                                                    type="range"
                                                    min="0.001"
                                                    max="0.1"
                                                    step="0.001"
                                                    value={sensor.size || 0.01}
                                                    onChange={(e) => updateSensor(sensor.id, { size: parseFloat(e.target.value) })}
                                                    className="w-full h-1 bg-gray-700 rounded-lg appearance-none cursor-pointer slider"
                                                />
                                            </div>
                                            <div>
                                                <label className="block text-gray-400 text-[10px] mb-1">
                                                    Range: {(sensor.range || 0.02).toFixed(3)} m
                                                </label>
                                                <input
                                                    type="range"
                                                    min="0.005"
                                                    max="0.2"
                                                    step="0.005"
                                                    value={sensor.range || 0.02}
                                                    onChange={(e) => updateSensor(sensor.id, { range: parseFloat(e.target.value) })}
                                                    className="w-full h-1 bg-gray-700 rounded-lg appearance-none cursor-pointer slider"
                                                />
                                            </div>
                                        </div>
                                    )}
                                </div>
                            </div>
                        );
                    })}

                    {sensorList.length === 0 && (
                        <p className="text-xs text-gray-500 text-center py-2">
                            No sensors placed. Use the button above to start.
                        </p>
                    )}
                </div>
            )}
        </div>
    );
};

export default SensorPlacement;
