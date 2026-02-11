// src/frontend/src/components/CameraManager.tsx
import React, { useRef, useEffect } from 'react';
import { CameraControls } from '@react-three/drei';
import { useThree } from '@react-three/fiber';
import { useRobotStore } from '../store';
import { Vector3 } from 'three';

const PAN_SPEED = 0.5; // units per second
const ROTATE_SPEED = 1.0; // radians per second

export const CameraManager: React.FC = () => {
    const controlsRef = useRef<CameraControls | null>(null);
    const { scene } = useThree();
    const { cameraMode, setCameraControls } = useRobotStore();

    useEffect(() => {
        if (controlsRef.current) {
            setCameraControls(controlsRef.current);
        }
        
        const controls = controlsRef.current;
        if (!controls) return;

        // Set initial camera position and target
        const pos = new Vector3(-3, 4, 5);
        const target = new Vector3(0, 0, 0);
        controls.setLookAt(pos.x, pos.y, pos.z, target.x, target.y, target.z, false);
        
        return () => {
            setCameraControls(null);
        }
    }, [scene, setCameraControls]);

    useEffect(() => {
        const controls = controlsRef.current;
        if (!controls) return;

        // Mouse middle button drag for rotation
        const handleMouseDown = (e: MouseEvent) => {
            if (e.button === 1) { // Middle mouse button
                document.addEventListener('mousemove', handleMouseMove);
                document.addEventListener('mouseup', handleMouseUp);
            }
        };

        const handleMouseMove = (e: MouseEvent) => {
            const deltaX = e.movementX * 0.005; // Adjust sensitivity
            const deltaY = e.movementY * 0.005;
            controls.rotate(deltaX, deltaY, true);
        };

        const handleMouseUp = () => {
            document.removeEventListener('mousemove', handleMouseMove);
            document.removeEventListener('mouseup', handleMouseUp);
        };

        window.addEventListener('mousedown', handleMouseDown);

        return () => {
            window.removeEventListener('mousedown', handleMouseDown);
        };
    }, [controlsRef]);

    useEffect(() => {
        const controls = controlsRef.current;
        if (!controls) return;

        const handleKeyDown = (e: KeyboardEvent) => {
            if (document.activeElement && document.activeElement.tagName === 'INPUT') {
                return;
            }

            let dx = 0, dy = 0;
            const delta = 0.1;

            switch (e.key) {
                case 'ArrowUp': dx = -delta * 2; break; // Pitch up
                case 'ArrowDown': dx = delta * 2; break; // Pitch down
                case 'ArrowLeft': dy = -delta * 2; break; // Yaw left
                case 'ArrowRight': dy = delta * 2; break; // Yaw right
            }
            if (dx !== 0 || dy !== 0) {
                controls.rotate(dy, dx, true); // Note: dy for yaw, dx for pitch
            }
        };

        window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [cameraMode, controlsRef]);

    return <CameraControls ref={controlsRef} />;
};