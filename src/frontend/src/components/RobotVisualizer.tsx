


// src/frontend/src/components/RobotVisualizer.tsx

import React, { useRef, useEffect, useState, useMemo } from 'react';
import { useRobotStore } from '../store';
import { Box, Cylinder, Sphere, TransformControls, Html } from '@react-three/drei';
import * as THREE from 'three';
import { useFrame, useThree } from '@react-three/fiber';
import { Move, RotateCw } from 'lucide-react';
import { STLMesh } from './STLMesh'; // Import the new component

const HIGHLIGHT_COLOR = 'hotpink';

type RefMap = Map<string, THREE.Object3D>;
type RegisterRef = (id: string, ref: THREE.Object3D) => void;
type GizmoMode = "translate" | "rotate";

// --- JointWrapper ---
const JointWrapper: React.FC<{ jointId: string; registerRef: RegisterRef }> = ({ jointId, registerRef }) => {
  const joint = useRobotStore((state) => state.joints[jointId]);
  const { selectedItem, selectItem } = useRobotStore();
  
  const originGroupRef = useRef<THREE.Group>(null!);
  const motionGroupRef = useRef<THREE.Group>(null!);

  useEffect(() => {
    const ref = originGroupRef.current;
    if (ref) {
      registerRef(jointId, ref);
      return () => registerRef(jointId, null!);
    }
  }, [jointId, registerRef]);
  
  useFrame(() => {
    if (!originGroupRef.current || !motionGroupRef.current) return;

    originGroupRef.current.position.set(...(joint.origin?.xyz || [0,0,0]));
    originGroupRef.current.rotation.set(...(joint.origin?.rpy || [0,0,0]));

    const motionGroup = motionGroupRef.current;
    if (joint.type === 'rotational') {
        const euler = new THREE.Euler(
            joint.dof.roll ? joint.currentValues.roll : 0,
            joint.dof.pitch ? joint.currentValues.pitch : 0,
            joint.dof.yaw ? joint.currentValues.yaw : 0,
            'ZYX' 
        );
        motionGroup.quaternion.setFromEuler(euler);
        motionGroup.position.set(0, 0, 0);
    } else if (joint.type === 'prismatic') {
        const axis = new THREE.Vector3(...joint.axis).normalize();
        motionGroup.position.copy(axis).multiplyScalar(joint.currentValues.displacement);
        motionGroup.quaternion.set(0,0,0,1);
    } else {
        motionGroup.position.set(0, 0, 0);
        motionGroup.quaternion.set(0,0,0,1);
    }
  });

  const axes = useMemo(() => new THREE.AxesHelper(0.2), []);

  return (
    <group ref={originGroupRef}>
      <group ref={motionGroupRef}>
        <primitive object={axes} />
        <Sphere args={[0.02, 16, 16]} onClick={(e) => { e.stopPropagation(); selectItem(joint.id, 'joint'); }}>
            <meshStandardMaterial color={selectedItem.id === jointId ? HIGHLIGHT_COLOR : 'yellow'} />
        </Sphere>
        {joint.childLinkId && <RecursiveLink linkId={joint.childLinkId} registerRef={registerRef} />}
      </group>
    </group>
  );
};


// --- RecursiveLink ---
const RecursiveLink: React.FC<{ linkId: string; registerRef: RegisterRef }> = ({ linkId, registerRef }) => {
  const link = useRobotStore((state) => state.links[linkId]);
  const getJoint = useRobotStore((state) => (id: string) => state.joints[id]);
  const { selectedItem, selectItem } = useRobotStore();
  const groupRef = useRef<THREE.Group>(null!);

  useEffect(() => {
    const ref = groupRef.current;
    if (ref) {
      registerRef(linkId, ref);
      return () => registerRef(linkId, null!);
    }
  }, [linkId, registerRef]);

  if (!link) return null;

  const renderVisual = () => {
    const { type, dimensions, color, meshUrl, meshScale, meshOrigin } = link.visual;
    if (type === 'none') return null;

    const isSelected = selectedItem.id === linkId;
    const materialColor = isSelected ? HIGHLIGHT_COLOR : color;
    
    const clickHandler = (e: any) => { e.stopPropagation(); selectItem(link.id, 'link'); };

    // Handle mesh type first
    if (type === 'mesh') {
        if (!meshUrl) return null; // Don't render if no URL
        return (
            // Wrap STLMesh in a group to handle the click, as STLMesh uses Suspense
            <group onClick={clickHandler}>
                <STLMesh
                    linkId={linkId}
                    url={meshUrl}
                    scale={meshScale}
                    origin={meshOrigin}
                    color={materialColor}
                />
            </group>
        );
    }

    const props = {
        onClick: clickHandler,
    };

    switch (type) {
      case 'box': 
        // Box is centered at origin by default, this is usually fine.
        return <Box {...props} args={dimensions as [number, number, number]} >
            <meshStandardMaterial color={materialColor} />
        </Box>;

      case 'cylinder': {
        // This is a connecting link in a chain.
        if (link.childJoints.length === 1) {
            const childJoint = getJoint(link.childJoints[0]);
            if (childJoint) {
                const start = new THREE.Vector3(0, 0, 0);
                const end = new THREE.Vector3(...childJoint.origin.xyz);
                
                if (end.lengthSq() < 0.0001) return null; // Avoid zero-length cylinder

                const JOINT_VISUAL_RADIUS = 0.02; // Matches the Sphere args in JointWrapper
                const GAP_OFFSET = JOINT_VISUAL_RADIUS * 1.5; // Create a visible gap

                const originalLength = start.distanceTo(end);
                const length = Math.max(0, originalLength - (GAP_OFFSET * 2)); // Don't allow negative length
                
                // If the link is too short to be visible, don't render it.
                if (length <= 0) return null;

                const midPoint = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);

                const orientation = new THREE.Quaternion();
                const up = new THREE.Vector3(0, 1, 0); // Default Cylinder axis is Y
                const direction = new THREE.Vector3().subVectors(end, start).normalize();
                
                orientation.setFromUnitVectors(up, direction);

                const radius = dimensions[0] || 0.05;
                const cylinderArgs: [number, number, number, number] = [radius, radius, length, 16];

                return (
                    <group position={midPoint} quaternion={orientation}>
                        <Cylinder args={cylinderArgs} {...props}>
                            <meshStandardMaterial color={materialColor} />
                        </Cylinder>
                    </group>
                );
            }
        }
        
        // Fallback for terminal or branching links.
        const radius = dimensions[0] || 0.05;
        const length = dimensions[1] || 0.5;
        const cylinderProps: [number, number, number, number] = [radius, radius, length, 16];
        // For a terminal cylinder, stand it on the XY plane (Y-up). Its base is at y=0.
        return <Cylinder {...props} args={cylinderProps} position={[0, length / 2, 0]} >
            <meshStandardMaterial color={materialColor} />
        </Cylinder>;
      }
        
      case 'sphere':
        // Sphere is centered at origin.
        return <Sphere {...props} args={[(dimensions[0] || 0.1), 32, 32]} >
             <meshStandardMaterial color={materialColor} />
        </Sphere>;

      default: return null;
    }
  };

  return (
    <group ref={groupRef}>
      {renderVisual()}
      {link.childJoints.map((jointId) => (
        <JointWrapper key={jointId} jointId={jointId} registerRef={registerRef} />
      ))}
    </group>
  );
};


// --- Main Visualizer with Gizmo Logic ---
const RobotVisualizer: React.FC = () => {
  const { baseLinkId, selectedItem, updateJoint, cameraMode, setCameraMode } = useRobotStore();
  // We get the camera controls via the new manager, not useThree, to avoid conflicts
  // const orbitControls = useThree(state => state.controls) as any; 
  const { scene } = useThree();
  const transformControlsRef = useRef<any>(null!);
  const [objectRefs] = useState(() => new Map<string, THREE.Object3D>());

  const registerRef: RegisterRef = (id, ref) => {
    if (ref) objectRefs.set(id, ref);
    else objectRefs.delete(id);
  };

  useEffect(() => {
    const controls = transformControlsRef.current;
    const target = (selectedItem.type === 'joint' && selectedItem.id) ? objectRefs.get(selectedItem.id) : null;

    if (target) {
        controls.attach(target);
        controls.visible = true;
    } else {
        controls.detach();
        controls.visible = false;
    }
    
    // This logic to disable camera controls during object dragging is now more complex
    // because the controls are managed in CameraManager. We'll handle this with a new state.
    // For now, we focus on the main task. A simple way is to check the `dragging` property.
    if (controls) {
        // Guard against race condition: cameraControls might not be set yet.
        const cameraControls = (scene.userData.cameraControls as any);
        if (!cameraControls) {
          return;
        }

        const callback = (event: any) => {
            if (cameraControls) cameraControls.enabled = !event.value
        };
        controls.addEventListener('dragging-changed', callback);
        return () => controls.removeEventListener('dragging-changed', callback);
    }
  }, [selectedItem, objectRefs, scene]);

  const handleObjectChange = () => {
    const controls = transformControlsRef.current;
    if (!controls || !controls.object || !selectedItem.id || selectedItem.type !== 'joint') return;

    const { object } = controls;
    const newPos = object.position.toArray();
    const newRot = [object.rotation.x, object.rotation.y, object.rotation.z];

    updateJoint(selectedItem.id, 'origin.xyz', newPos);
    updateJoint(selectedItem.id, 'origin.rpy', newRot);
  }

  return (
    <>
      <Html position={[-4, 2, 0]} wrapperClass="w-32">
        <div className="bg-gray-800 bg-opacity-80 p-1 rounded-lg flex justify-around">
            {/* These buttons now control the CAMERA mode, not the transform gizmo */}
            <button onClick={() => setCameraMode("pan")} className={`p-2 rounded ${cameraMode==='pan' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`}> <Move size={16}/> </button>
            <button onClick={() => setCameraMode("rotate")} className={`p-2 rounded ${cameraMode==='rotate' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`}> <RotateCw size={16}/> </button>
        </div>
      </Html>
      {baseLinkId && <RecursiveLink linkId={baseLinkId} registerRef={registerRef} />}
      {/* The transform gizmo is independent. Let's default it to translate. The user can cycle with 'e' key. */}
      <TransformControls ref={transformControlsRef} onObjectChange={handleObjectChange} mode={"translate"} />
    </>
  );
};

export default RobotVisualizer;
