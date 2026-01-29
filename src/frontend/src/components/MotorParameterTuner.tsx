import React, { useState } from 'react';
import axios from 'axios';
import { Line } from 'react-chartjs-2';

interface MotorParameterTunerProps {
  onParametersUpdated?: (kp: number, kv: number) => void;
}

const MotorParameterTuner: React.FC<MotorParameterTunerProps> = ({ onParametersUpdated }) => {
  const [kp, setKp] = useState(100);
  const [kv, setKv] = useState(10);
  const [simulationData, setSimulationData] = useState({
    torque: [],
    speed: [],
    error: [],
    torque_saturation: 0,
    tracking_error: 0,
    rms_torque: 0,
  });

  const handleUpdate = async () => {
    try {
      await axios.post('/api/apply_params', { kp, kv });
      if (onParametersUpdated) {
        onParametersUpdated(kp, kv);
      }
    } catch (error) {
      console.error('Failed to update parameters:', error);
    }
  };

  const handleSimulate = async () => {
    try {
      const response = await axios.post('/api/simulate', {
        forcelim: kp * 2, // Example mapping
        gear: kv * 1.5, // Example mapping
        velocity: 50, // Placeholder value
        armature: 0.1, // Placeholder value
        frictionloss: 0.05, // Placeholder value
      });
      setSimulationData(response.data);
    } catch (error) {
      console.error('Simulation failed:', error);
    }
  };

  return (
    <div className="bg-gray-800 p-4 rounded shadow-md">
      <h3 className="text-xl font-bold text-white mb-4">Motor Parameter Tuner</h3>
      <div className="mb-4">
        <label className="block text-gray-300 mb-2">Kp (Stiffness)</label>
        <input
          type="range"
          min="0"
          max="200"
          value={kp}
          onChange={(e) => setKp(Number(e.target.value))}
          className="w-full"
        />
        <span className="text-gray-400">{kp}</span>
      </div>
      <div className="mb-4">
        <label className="block text-gray-300 mb-2">Kv (Damping)</label>
        <input
          type="range"
          min="0"
          max="50"
          value={kv}
          onChange={(e) => setKv(Number(e.target.value))}
          className="w-full"
        />
        <span className="text-gray-400">{kv}</span>
      </div>
      <button
        onClick={handleUpdate}
        className="bg-green-500 text-white px-4 py-2 rounded hover:bg-green-600"
      >
        Apply Parameters
      </button>
      <button
        onClick={handleSimulate}
        className="bg-blue-500 text-white px-4 py-2 rounded hover:bg-blue-600"
      >
        Simulate
      </button>

      <div className="mt-6">
        <h4 className="text-lg font-bold text-white mb-2">Simulation Results</h4>
        <Line
          data={{
            labels: simulationData.torque.map((_, index) => `Step ${index + 1}`),
            datasets: [
              {
                label: 'Torque',
                data: simulationData.torque,
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(75, 192, 192, 0.2)',
              },
              {
                label: 'Speed',
                data: simulationData.speed,
                borderColor: 'rgba(255, 99, 132, 1)',
                backgroundColor: 'rgba(255, 99, 132, 0.2)',
              },
              {
                label: 'Error',
                data: simulationData.error,
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(54, 162, 235, 0.2)',
              },
            ],
          }}
          options={{
            responsive: true,
            plugins: {
              legend: {
                position: 'top',
              },
            },
          }}
        />
        <div className="mt-4">
          <p className="text-gray-300">Torque Saturation: {simulationData.torque_saturation}</p>
          <p className="text-gray-300">Tracking Error: {simulationData.tracking_error}</p>
          <p className="text-gray-300">RMS Torque: {simulationData.rms_torque}</p>
        </div>
      </div>
    </div>
  );
};

export default MotorParameterTuner;