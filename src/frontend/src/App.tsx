// src/frontend/src/App.tsx
import { Canvas, ThreeEvent } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import RobotVisualizer from './components/RobotVisualizer';
import Sidebar from './components/Sidebar';
import { useRobotStore } from './store';

function App() {
  const { selectItem } = useRobotStore.getState();

  // This handler now correctly deselects objects when clicking on the background.
  const handleCanvasClick = (event: ThreeEvent<MouseEvent>) => {
    // Stop propagation to prevent this from firing when an object is clicked.
    // Object clicks are handled in RobotVisualizer.
    event.stopPropagation();
    
    // Check if the click was on the background (missed all objects)
    if (event.intersections.length === 0) {
      selectItem(null, null);
    }
  };

  return (
    <div className="h-screen w-screen bg-gray-900">
      <Canvas 
        shadows 
        // User wants to see XY plane, looking from a 45-degree angle.
        // This position is looking from above and angled.
        camera={{ position: [-4, -4, 4], fov: 50, up: [0, 0, 1] }}
        onCreated={({ scene }) => {
          // Set Z as the 'up' axis for the scene and controls.
          // This is redundant with camera.up but reinforces the coordinate system.
          scene.up.set(0, 0, 1);
        }}
        // Use onClick instead of onPointerMissed for more reliable deselection.
        onClick={handleCanvasClick}
      >
        <ambientLight intensity={0.7} />
        <directionalLight 
            position={[5, 5, 10]} // Higher Z for better shadow angles
            intensity={1.5} 
            castShadow
            shadow-mapSize-width={2048}
            shadow-mapSize-height={2048}
        />
        {/* The grid is now correctly on the XY plane. */}
        <Grid rotation={[Math.PI / 2, 0, 0]} args={[20, 20]} infiniteGrid fadeDistance={40} fadeStrength={5} />
        
        {/* OrbitControls need to know Z is up. */}
        <OrbitControls makeDefault />
        
        <RobotVisualizer />
      </Canvas>
      <Sidebar />
    </div>
  );
}

export default App;
