import React, { useState } from 'react';
import { useRobotStore } from '../store';
import { SensorDef } from '../types';

const SensorPlacement = () => {
    const { sensors, deleteSensor, setInteractionMode, interactionMode, links } = useRobotStore();
    const [isExpanded, setIsExpanded] = useState(false);
    const sensorList = Object.values(sensors);

    const handleStartPlacement = () => {
        setInteractionMode('sensor-placement');
    };

    const handleStopPlacement = () => {
        setInteractionMode('select');
    };

    const getLinkName = (linkId: string) => links[linkId]?.name || linkId.slice(0, 8);

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
                    {sensorList.map(sensor => (
                        <div key={sensor.id} className="p-2 bg-gray-900/50 rounded text-xs">
                            <div className="flex items-center justify-between">
                                <div className="flex items-center space-x-2">
                                    <span className={`w-2 h-2 rounded-full ${sensor.type === 'touch' ? 'bg-green-500' : 'bg-yellow-500'}`}></span>
                                    <span className="font-semibold">{sensor.siteName}</span>
                                </div>
                                <button
                                    onClick={() => deleteSensor(sensor.id)}
                                    className="text-red-400 hover:text-red-300 px-1"
                                >✕</button>
                            </div>
                            <div className="text-gray-500 text-[10px] mt-1">
                                Link: {getLinkName(sensor.linkId)} |
                                Pos: [{sensor.localPosition.map(v => v.toFixed(3)).join(', ')}]
                            </div>
                        </div>
                    ))}

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
