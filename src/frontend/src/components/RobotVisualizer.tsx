// src/frontend/src/components/RobotVisualizer.tsx

import React, { useRef, useEffect, useState, useMemo } from 'react';
import { useRobotStore } from '../store';
import { Box, Cylinder, Sphere, TransformControls, Html } from '@react-three/drei';
import * as THREE from 'three';
import { useFrame, useThree } from '@react-three/fiber';
import { Move, RotateCw } from 'lucide-react';

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
    const { type, dimensions, color } = link.visual;
    if (type === 'none') return null;

    const isSelected = selectedItem.id === linkId;
    const materialColor = isSelected ? HIGHLIGHT_COLOR : color;
    
    const clickHandler = (e: any) => { e.stopPropagation(); selectItem(link.id, 'link'); };

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

                const length = start.distanceTo(end);
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
  const { baseLinkId, selectedItem, updateJoint } = useRobotStore();
  const orbitControls = useThree(state => state.controls) as any;

  const [objectRefs] = useState(() => new Map<string, THREE.Object3D>());
  const transformControlsRef = useRef<any>(null!);
  const [gizmoMode, setGizmoMode] = useState<GizmoMode>("translate");
  
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
    
    if (controls) {
        const callback = (event: any) => {
            if (orbitControls) orbitControls.enabled = !event.value
        };
        controls.addEventListener('dragging-changed', callback);
        return () => controls.removeEventListener('dragging-changed', callback);
    }
  }, [selectedItem, objectRefs, orbitControls]);

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
            <button onClick={() => setGizmoMode("translate")} className={`p-2 rounded ${gizmoMode==='translate' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`}> <Move size={16}/> </button>
            <button onClick={() => setGizmoMode("rotate")} className={`p-2 rounded ${gizmoMode==='rotate' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`}> <RotateCw size={16}/> </button>
        </div>
      </Html>
      {baseLinkId && <RecursiveLink linkId={baseLinkId} registerRef={registerRef} />}
      <TransformControls ref={transformControlsRef} onObjectChange={handleObjectChange} mode={gizmoMode} />
    </>
  );
};

export default RobotVisualizer;
