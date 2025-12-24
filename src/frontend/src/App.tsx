import * as THREE from 'three';
import React from 'react';
import { Canvas, ThreeEvent } from '@react-three/fiber';
import { Grid } from '@react-three/drei';
import RobotVisualizer from './components/RobotVisualizer';
import Sidebar from './components/Sidebar';
import { useRobotStore } from './store';
import { CameraManager } from './components/CameraManager';
import { CameraUI } from './components/CameraUI';

function App() {
  const { selectItem } = useRobotStore();

  return (
    <div className="h-screen w-screen bg-gray-900">
      <Canvas
        shadows
        camera={{ position: [-4, -4, 4], fov: 50, up: [0, 0, 1] }}
        onCreated={({ scene, camera }) => {
          scene.up.set(0, 0, 1);
          camera.up.set(0, 0, 1);
        }}
        onPointerMissed={(e) => {
          // Only deselect if it was a click (not a drag release)
          if (e.type === 'click') {
            selectItem(null, null);
          }
        }}
      >
        <ambientLight intensity={0.7} />
        <directionalLight
          position={[5, 5, 10]}
          intensity={1.5}
          castShadow
          shadow-mapSize-width={2048}
          shadow-mapSize-height={2048}
        />
        <Grid rotation={[Math.PI / 2, 0, 0]} args={[20, 20]} infiniteGrid fadeDistance={40} fadeStrength={5} onClick={(e) => { e.stopPropagation(); selectItem(null, null); }} />
        <CameraManager />
        <RobotVisualizer />
      </Canvas>
      <CameraUI />
      <Sidebar />
    </div>
  );
}

export default App;
