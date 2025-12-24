


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

    originGroupRef.current.position.set(...(joint.origin?.xyz || [0, 0, 0]));
    originGroupRef.current.rotation.set(...(joint.origin?.rpy || [0, 0, 0]));

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
      motionGroup.quaternion.set(0, 0, 0, 1);
    } else {
      motionGroup.position.set(0, 0, 0);
      motionGroup.quaternion.set(0, 0, 0, 1);
    }
  });

  const axes = useMemo(() => new THREE.AxesHelper(0.2), []);

  const renderVisual = () => {
    // Default small handle if no visual is set
    if (!joint.visual || joint.visual.type === 'none') {
      return (
        <Sphere args={[0.02, 16, 16]} onClick={(e) => { e.stopPropagation(); selectItem(joint.id, 'joint'); }}>
          <meshStandardMaterial color={selectedItem.id === jointId ? HIGHLIGHT_COLOR : 'yellow'} />
        </Sphere>
      );
    }

    const { type, dimensions, color, meshUrl, meshScale, meshOrigin } = joint.visual;
    const isSelected = selectedItem.id === jointId;
    const materialColor = isSelected ? HIGHLIGHT_COLOR : (color || '#888888');
    const clickHandler = (e: any) => { e.stopPropagation(); selectItem(joint.id, 'joint'); };

    if (type === 'mesh') {
      if (!meshUrl) return null;
      return (
        <group onClick={clickHandler}>
          <STLMesh
            linkId={jointId} // passing jointId as linkId for caching purposes is fine
            url={meshUrl}
            scale={meshScale}
            origin={meshOrigin}
            color={materialColor}
          />
        </group>
      );
    }

    const props = { onClick: clickHandler };
    const dims = dimensions || [0.05, 0.05, 0.05];

    switch (type) {
      case 'box':
        return <Box {...props} args={dims as [number, number, number]}><meshStandardMaterial color={materialColor} /></Box>;
      case 'cylinder':
        return <Cylinder {...props} args={[dims[0], dims[0], dims[1], 16]}><meshStandardMaterial color={materialColor} /></Cylinder>;
      case 'sphere':
        return <Sphere {...props} args={[dims[0], 16, 16]}><meshStandardMaterial color={materialColor} /></Sphere>;
      default: return null;
    }
  };

  return (
    <group ref={originGroupRef}>
      <group ref={motionGroupRef}>
        <primitive object={axes} />
        {renderVisual()}
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

  // Helper: Clamp value within limits
  const clamp = (val: number, min?: number, max?: number) => {
    let v = val;
    if (min !== undefined) v = Math.max(min, v);
    if (max !== undefined) v = Math.min(max, v);
    return v;
  };

  const registerRef: RegisterRef = (id, ref) => {
    if (ref) objectRefs.set(id, ref);
    else objectRefs.delete(id);
  };

  // Helper to find parent joint for a link
  const getParentJointId = (linkId: string): string | null => {
    const joints = useRobotStore.getState().joints;
    const entry = Object.values(joints).find(j => j.childLinkId === linkId);
    return entry ? entry.id : null;
  };

  useEffect(() => {
    const controls = transformControlsRef.current;

    let targetId = null;
    if (selectedItem.id) {
      if (selectedItem.type === 'joint') {
        targetId = selectedItem.id;
      } else if (selectedItem.type === 'link') {
        // If checking link, find its parent joint
        const parentJointId = getParentJointId(selectedItem.id);
        if (parentJointId) {
          targetId = parentJointId;
        }
      }
    }

    const target = targetId ? objectRefs.get(targetId) : null;

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
  }, [selectedItem, objectRefs, scene]); // Added dependencies for reliability

  const handleObjectChange = () => {
    const controls = transformControlsRef.current;

    // Determine the actual Joint ID being modified
    let actualJointId: string | null = null;
    if (selectedItem.type === 'joint') {
      actualJointId = selectedItem.id;
    } else if (selectedItem.type === 'link') {
      actualJointId = getParentJointId(selectedItem.id || "");
    }

    if (!controls || !controls.object || !actualJointId) return;

    // Fetch latest state to check limits and types
    const state = useRobotStore.getState();
    const joint = state.joints[actualJointId];
    if (!joint) return;

    if (joint.type === 'fixed') {
      // Fixed joints cannot be moved
      return;
    }

    const { object } = controls;

    if (joint.type === 'rotational') {
      // Read rotation from object
      // NOTE: The object is the motionGroup. Its default order might be XYZ but we prefer ZYX for Roll-Pitch-Yaw
      const euler = new THREE.Euler().setFromQuaternion(object.quaternion, 'ZYX');

      // We only update the DOFs that are enabled
      const newValues = { ...joint.currentValues };

      if (joint.dof.yaw) newValues.yaw = clamp(euler.z, joint.limits.yaw.lower, joint.limits.yaw.upper);
      if (joint.dof.pitch) newValues.pitch = clamp(euler.y, joint.limits.pitch.lower, joint.limits.pitch.upper);
      if (joint.dof.roll) newValues.roll = clamp(euler.x, joint.limits.roll.lower, joint.limits.roll.upper);

      // Update Store
      updateJoint(actualJointId, 'currentValues', newValues);

    } else if (joint.type === 'prismatic') {
      // Read position
      const pos = object.position;
      const axis = new THREE.Vector3(...joint.axis).normalize();

      // Project position onto the axis to find displacement (scalar)
      // displacement = pos dot axis
      const displacement = pos.dot(axis);

      const clampedDisp = clamp(displacement, joint.limits.displacement?.lower, joint.limits.displacement?.upper);

      const newValues = { ...joint.currentValues, displacement: clampedDisp };
      updateJoint(actualJointId, 'currentValues', newValues);
    }
  }

  // Determine TransformControls Props based on Selection
  const getGizmoProps = () => {
    let targetJointId: string | null = null;
    if (selectedItem.type === 'joint') targetJointId = selectedItem.id;
    else if (selectedItem.type === 'link') targetJointId = getParentJointId(selectedItem.id || "");

    if (!targetJointId) return { mode: 'translate' as const, visible: false };

    const joint = useRobotStore.getState().joints[targetJointId];
    if (!joint || joint.type === 'fixed') return { mode: 'translate' as const, visible: false };

    if (joint.type === 'rotational') {
      return {
        mode: 'rotate' as const,
        visible: true,
        showX: joint.dof.roll,
        showY: joint.dof.pitch,
        showZ: joint.dof.yaw,
        space: 'local' as const
      };
    } else if (joint.type === 'prismatic') {
      // For prismatic, we ideally want to show only the arrow along the axis.
      // TransformControls doesn't support arbitrary axis arrows easily without rotation.
      // BUT, we are in 'local' space of the joint.
      // The JointWrapper sets the MOTION GROUP position/rotation relative to origin.
      // The Prismatic logic: `motionGroup.position.copy(axis).multiplyScalar(disp)`.
      // If axis is [0,0,1], we slide Z. If axis is [1,0,0], we slide X.
      // So we can enable X/Y/Z based on the axis vector components? 
      // Roughly: if abs(component) > 0.1, show it.
      const axis = joint.axis;
      return {
        mode: 'translate' as const,
        visible: true,
        showX: Math.abs(axis[0]) > 0.01,
        showY: Math.abs(axis[1]) > 0.01,
        showZ: Math.abs(axis[2]) > 0.01,
        space: 'local' as const
      };
    }

    return { mode: 'translate' as const, visible: false };
  };

  const gizmoProps = getGizmoProps();

  return (
    <>
      <Html position={[-4, 2, 0]} wrapperClass="w-32">
        <div className="bg-gray-800 bg-opacity-80 p-1 rounded-lg flex justify-around">
          <button onClick={() => setCameraMode("pan")} className={`p-2 rounded ${cameraMode === 'pan' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`} title="Camera Pan"> <Move size={16} /> </button>
          <button onClick={() => setCameraMode("rotate")} className={`p-2 rounded ${cameraMode === 'rotate' ? 'bg-blue-600' : 'bg-gray-700'} hover:bg-blue-500`} title="Camera Rotate"> <RotateCw size={16} /> </button>
        </div>
      </Html>
      {baseLinkId && <RecursiveLink linkId={baseLinkId} registerRef={registerRef} />}

      <TransformControls
        ref={transformControlsRef}
        onObjectChange={handleObjectChange}
        mode={gizmoProps.mode}
        visible={gizmoProps.visible && !!((selectedItem.id && selectedItem.type === 'joint') || (selectedItem.type === 'link' && getParentJointId(selectedItem.id || "")))}
        showX={gizmoProps.showX}
        showY={gizmoProps.showY}
        showZ={gizmoProps.showZ}
        space={gizmoProps.space}
        size={0.7}
      />
    </>
  );
};

export default RobotVisualizer;
