


// src/frontend/src/components/RobotVisualizer.tsx

import React, { useRef, useEffect, useState, useMemo } from 'react';
import { useRobotStore } from '../store';
import { Box, Cylinder, Sphere, TransformControls, Html } from '@react-three/drei';
import * as THREE from 'three';
import { useFrame, useThree } from '@react-three/fiber';
import { Move, RotateCw } from 'lucide-react';
import { STLMesh } from './STLMesh';
import { computeBoundsTree, disposeBoundsTree, acceleratedRaycast } from 'three-mesh-bvh';

// Extend BufferGeometry to include BVH properties
THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
(THREE.Mesh.prototype as any).raycast = acceleratedRaycast;

const HIGHLIGHT_COLOR = 'hotpink';
const COLLISION_COLOR = '#FF0000'; // Red for collision

type RefMap = Map<string, THREE.Object3D>;
type RegisterRef = (id: string, ref: THREE.Object3D) => void;

// --- JointWrapper ---
const JointWrapper: React.FC<{ jointId: string; registerRef: RegisterRef; isColliding: boolean }> = ({ jointId, registerRef, isColliding }) => {
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
        <Sphere args={[0.02, 16, 16]} onClick={(e) => { e.stopPropagation(); selectItem(joint.id, 'joint'); }} userData={{ isVisual: true, ownerId: jointId }}>
          <meshStandardMaterial color={isColliding ? COLLISION_COLOR : (selectedItem.id === jointId ? HIGHLIGHT_COLOR : 'yellow')} side={THREE.DoubleSide} />
        </Sphere>
      );
    }

    const { type, dimensions, color, meshUrl, meshScale, meshOrigin } = joint.visual;
    const isSelected = selectedItem.id === jointId;
    const isHighlighted = useRobotStore(s => s.highlightedItem.id === jointId);

    // Highlight logic: if highlighted, override color to Cyan/White mixer
    const displayColor = isHighlighted ? '#00FFFF' : (isColliding ? COLLISION_COLOR : (isSelected ? HIGHLIGHT_COLOR : (color || '#888888')));

    const clickHandler = (e: any) => { e.stopPropagation(); selectItem(joint.id, 'joint'); };

    if (type === 'mesh') {
      if (!meshUrl) return null;
      return (
        <group onClick={clickHandler} userData={{ isVisual: true, ownerId: jointId }}>
          <STLMesh
            linkId={jointId}
            url={meshUrl}
            scale={meshScale}
            origin={meshOrigin}
            color={displayColor}
          />
        </group>
      );
    }

    const props = { onClick: clickHandler, userData: { isVisual: true, ownerId: jointId } };
    const dims = dimensions || [0.05, 0.05, 0.05];

    switch (type) {
      case 'box':
        return <Box {...props} args={dims as [number, number, number]}><meshStandardMaterial color={displayColor} side={THREE.DoubleSide} /></Box>;
      case 'cylinder':
        return <Cylinder {...props} args={[dims[0], dims[0], dims[1], 16]}><meshStandardMaterial color={displayColor} side={THREE.DoubleSide} /></Cylinder>;
      case 'sphere':
        return <Sphere {...props} args={[dims[0], 16, 16]}><meshStandardMaterial color={displayColor} side={THREE.DoubleSide} /></Sphere>;
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

  // We need to inject the collision state here. 
  // However, RobotVisualizer manages the collision state. 
  // We can't easily pass props down recursively without refactoring the whole recursion.
  // A cleaner way for the recursive pattern is to use a Context or a separate tiny store (or select purely from store if we put collision in global store).
  // But wait! We are inside RobotVisualizer, and RecursiveLink renders RecursiveLink...
  // Actually, let's use a simplistic approach: subscribe to a 'collidingLinks' set passed via Context?
  // OR, simpler: useRobotStore with a selector for "is this link colliding?"
  // But we didn't put collidingLinks in the main store to avoid global re-renders on every drag frame.
  // Let's modify RecursiveLink to take `collidingLinks` set as a prop? No, it's recursive.

  // SOLUTION: We will lift the `collidingLinks` set to a React Context to avoid prop drilling through recursion.
  const isColliding = useCollisionContext(linkId);

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
    if (!link.visual || link.visual.type === 'none') return null;
    const { type, dimensions, color, meshUrl, meshScale, meshOrigin } = link.visual;

    const isSelected = selectedItem.id === linkId;
    const materialColor = isColliding ? COLLISION_COLOR : (isSelected ? HIGHLIGHT_COLOR : color);

    const clickHandler = (e: any) => { e.stopPropagation(); selectItem(link.id, 'link'); };

    // Handle mesh type first
    if (type === 'mesh') {
      if (!meshUrl) return null;
      return (
        <group onClick={clickHandler} userData={{ isVisual: true, ownerId: linkId }}>
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
      userData: { isVisual: true, ownerId: linkId }
    };

    switch (type) {
      case 'box':
        return <Box {...props} args={dimensions as [number, number, number]} >
          <meshStandardMaterial color={materialColor} side={THREE.DoubleSide} />
        </Box>;

      case 'cylinder': {
        if (link.childJoints.length === 1) {
          const childJoint = getJoint(link.childJoints[0]);
          if (childJoint) {
            const start = new THREE.Vector3(0, 0, 0);
            const end = new THREE.Vector3(...childJoint.origin.xyz);
            if (end.lengthSq() < 0.0001) return null;

            const JOINT_VISUAL_RADIUS = 0.02;
            const GAP_OFFSET = JOINT_VISUAL_RADIUS * 1.5;
            const originalLength = start.distanceTo(end);
            const length = Math.max(0, originalLength - (GAP_OFFSET * 2));
            if (length <= 0) return null;

            const midPoint = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
            const orientation = new THREE.Quaternion();
            const up = new THREE.Vector3(0, 1, 0);
            const direction = new THREE.Vector3().subVectors(end, start).normalize();
            orientation.setFromUnitVectors(up, direction);

            const radius = dimensions[0] || 0.05;
            const cylinderArgs: [number, number, number, number] = [radius, radius, length, 16];

            return (
              <group position={midPoint} quaternion={orientation}>
                <Cylinder args={cylinderArgs} {...props}>
                  <meshStandardMaterial color={materialColor} side={THREE.DoubleSide} />
                </Cylinder>
              </group>
            );
          }
        }
        const radius = dimensions[0] || 0.05;
        const length = dimensions[1] || 0.5;
        const cylinderProps: [number, number, number, number] = [radius, radius, length, 16];
        return <Cylinder {...props} args={cylinderProps} position={[0, length / 2, 0]} >
          <meshStandardMaterial color={materialColor} side={THREE.DoubleSide} />
        </Cylinder>;
      }

      case 'sphere':
        return <Sphere {...props} args={[(dimensions[0] || 0.1), 32, 32]} >
          <meshStandardMaterial color={materialColor} side={THREE.DoubleSide} />
        </Sphere>;

      default: return null;
    }
  };

  return (
    <group ref={groupRef}>
      {renderVisual()}
      {link.childJoints.map((jointId) => (
        <JointWrapper key={jointId} jointId={jointId} registerRef={registerRef} isColliding={useCollisionContext(jointId)} />
      ))}
    </group>
  );
};

// --- Collision Context ---
// Simple Context to pass down the colliding set without drillin props deep
const CollisionContext = React.createContext<Set<string>>(new Set());
const useCollisionContext = (id: string) => {
  const set = React.useContext(CollisionContext);
  return set.has(id);
};

// --- Helper to create Proxy Cylinder Geometry ---
const createProxyCylinder = (mesh: THREE.Mesh, scale: number): THREE.BufferGeometry | null => {
  if (!mesh.geometry) return null;
  if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox();
  const box = mesh.geometry.boundingBox;
  if (!box) return null;

  const size = new THREE.Vector3();
  box.getSize(size);
  const center = new THREE.Vector3();
  box.getCenter(center);

  // Determine Axis
  let radius = 0;
  let height = 0;
  let axis = 'z';

  if (size.z >= size.x && size.z >= size.y) { // Z-up
    height = size.z;
    radius = Math.max(size.x, size.y) / 2;
    axis = 'z';
  } else if (size.y >= size.x && size.y >= size.z) { // Y-up
    height = size.y;
    radius = Math.max(size.x, size.z) / 2;
    axis = 'y';
  } else { // X-up
    height = size.x;
    radius = Math.max(size.y, size.z) / 2;
    axis = 'x';
  }

  radius *= scale;
  height *= scale;

  // Create Cylinder (defaults to Y-aligned)
  const geometry = new THREE.CylinderGeometry(radius, radius, height, 12);

  // Align Axis
  if (axis === 'z') {
    geometry.rotateX(Math.PI / 2);
  } else if (axis === 'x') {
    geometry.rotateZ(Math.PI / 2);
  }
  // Offset to center
  geometry.translate(center.x, center.y, center.z);

  return geometry;
};

// --- Helper to visualize the collision cylinder (Proxy) ---
const CollisionCylinderHelper: React.FC<{ objectRef: THREE.Object3D; scale: number; visible: boolean; color: string }> = ({ objectRef, scale, visible, color }) => {
  const meshRef = useRef<THREE.Mesh>(null!);

  useFrame(() => {
    if (!visible || !objectRef || !meshRef.current) return;

    let targetMesh: THREE.Mesh | undefined;
    if ((objectRef as THREE.Mesh).isMesh) targetMesh = objectRef as THREE.Mesh;
    else {
      objectRef.traverse((child) => {
        if ((child as THREE.Mesh).isMesh && !targetMesh) targetMesh = child as THREE.Mesh;
      });
    }

    if (targetMesh) {
      const proxyGeom = createProxyCylinder(targetMesh, scale);
      if (proxyGeom) {
        if (meshRef.current.geometry) meshRef.current.geometry.dispose();
        meshRef.current.geometry = proxyGeom;

        // Match Transform
        targetMesh.updateMatrixWorld();
        meshRef.current.matrix.copy(targetMesh.matrixWorld);
        // We must disable auto-update since we manually set the matrix
        meshRef.current.matrixAutoUpdate = false;
      }
    }
  });

  if (!visible) return null;

  return (
    <mesh ref={meshRef}>
      <meshBasicMaterial color={color} wireframe />
    </mesh>
  );
};

// --- Main Visualizer with Gizmo Logic ---
const RobotVisualizer: React.FC = () => {
  const { baseLinkId, selectedItem, updateJoint, cameraMode, setCameraMode, collisionMode, joints, collisionBoxScale } = useRobotStore();
  const transformControlsRef = useRef<any>(null!);
  const [objectRefs] = useState(() => new Map<string, THREE.Object3D>());
  const [collidingLinks, setCollidingLinks] = useState<Set<string>>(new Set());

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

  const getParentJointId = (linkId: string): string | null => {
    // using local `joints` from store hook
    const entry = Object.values(joints).find(j => j.childLinkId === linkId);
    return entry ? entry.id : null;
  };

  // --- Collision Logic ---
  const checkCollisions = () => {
    if (collisionMode !== 'off') console.log("running collision check", collisionMode);

    if (collisionMode === 'off') {
      if (collidingLinks.size > 0) setCollidingLinks(new Set());
      return false;
    }

    const collisionSet = new Set<string>();

    // We need to check every visible object against every other visible object
    // Filter objects that are "meshable" (have geometry)
    // objectRefs contains Groups. We need to find the Mesh children within those groups.
    const items = Array.from(objectRefs.entries());

    // Pre-calculate adjacency (Ignored Pairs)
    // Parent Link <-> Child Joint <-> Child Link
    // We ignore (Parent Link, Child Joint) and (Parent Link, Child Link)?? 
    // Usually physically connected parts should be ignored.
    const ignoredPairs = new Set<string>();
    Object.values(joints).forEach(joint => {
      if (joint.parentLinkId) {
        // Parent Link <-> Joint
        ignoredPairs.add(`${joint.parentLinkId}+${joint.id}`);
        ignoredPairs.add(`${joint.id}+${joint.parentLinkId}`);

        if (joint.childLinkId) {
          // Joint <-> Child Link
          ignoredPairs.add(`${joint.id}+${joint.childLinkId}`);
          ignoredPairs.add(`${joint.childLinkId}+${joint.id}`);

          // Parent Link <-> Child Link (Directly connected via joint)
          ignoredPairs.add(`${joint.parentLinkId}+${joint.childLinkId}`);
          ignoredPairs.add(`${joint.childLinkId}+${joint.parentLinkId}`);
        }
      }
    });

    for (let i = 0; i < items.length; i++) {
      for (let j = i + 1; j < items.length; j++) {
        const [id1, group1] = items[i];
        const [id2, group2] = items[j];

        if (ignoredPairs.has(`${id1}+${id2}`)) continue;

        // Traverse to find meshes strictly owned by this ID
        let mesh1: THREE.Mesh | undefined;
        let mesh2: THREE.Mesh | undefined;

        // Custom Traverse to find the visual owned by this ID
        group1.traverse((child) => {
          if (child.userData?.ownerId === id1) {
            if ((child as THREE.Mesh).isMesh) mesh1 = child as THREE.Mesh;
            // If it's a group (STL loader wrapper), check its children
            if (child.type === 'Group') {
              child.traverse((subChild) => {
                if ((subChild as THREE.Mesh).isMesh) mesh1 = subChild as THREE.Mesh;
              });
            }
          }
        });

        group2.traverse((child) => {
          if (child.userData?.ownerId === id2) {
            if ((child as THREE.Mesh).isMesh) mesh2 = child as THREE.Mesh;
            if (child.type === 'Group') {
              child.traverse((subChild) => {
                if ((subChild as THREE.Mesh).isMesh) mesh2 = subChild as THREE.Mesh;
              });
            }
          }
        });

        if (!mesh1 || !mesh2) continue;

        let intersection = false;

        // Ensure matrices are up to date
        // Note: during drag, React frame might not have updated yet, but the ThreeJS scene graph IS updated by TransformControls.
        // We must ensure WorldMatrices are current.
        mesh1.updateMatrixWorld();
        mesh2.updateMatrixWorld();

        // Optimisation: Broadphase Bounding Sphere check first?
        const sphere1 = mesh1.geometry.boundingSphere?.clone().applyMatrix4(mesh1.matrixWorld);
        const sphere2 = mesh2.geometry.boundingSphere?.clone().applyMatrix4(mesh2.matrixWorld);

        // Safety check to avoid crash if bounding sphere is null
        if (sphere1 && sphere2 && !sphere1.intersectsSphere(sphere2)) continue;

        try {
          if (collisionMode === 'box') {
            // CYLINDER MODE (Internally mapped to 'box' for now)
            const geom1 = createProxyCylinder(mesh1, collisionBoxScale);
            const geom2 = createProxyCylinder(mesh2, collisionBoxScale);

            if (geom1 && geom2) {
              if (!geom1.boundsTree) geom1.computeBoundsTree!();
              if (!geom2.boundsTree) geom2.computeBoundsTree!();

              // Check intersection
              // geom1 is in Local Space of mesh1.
              // We need to check (mesh1 * geom1) vs (mesh2 * geom2).
              // mesh1.matrixWorld transforms geom1 to World.
              // matrix2to1 = mesh1.inv * mesh2.
              const matrix2to1 = new THREE.Matrix4().copy(mesh1.matrixWorld).invert().multiply(mesh2.matrixWorld);
              if (geom1.boundsTree && geom2.boundsTree) {
                // intersection
                if (geom1.boundsTree.intersectsGeometry(geom2, matrix2to1)) {
                  intersection = true;
                }
              }
              // Cleanup
              geom1.dispose();
              geom1.disposeBoundsTree!();
              geom2.dispose();
              geom2.disposeBoundsTree!();
            }

          } else if (collisionMode === 'mesh') {
            if (!mesh1.geometry.boundsTree) mesh1.geometry.computeBoundsTree!();
            if (!mesh2.geometry.boundsTree) mesh2.geometry.computeBoundsTree!();

            if (mesh1.geometry.boundsTree && mesh2.geometry.boundsTree) {
              const matrix2to1 = new THREE.Matrix4().copy(mesh1.matrixWorld).invert().multiply(mesh2.matrixWorld);
              if (mesh1.geometry.boundsTree.intersectsGeometry(mesh2.geometry, matrix2to1)) {
                intersection = true;
              }
            }
          }
        } catch (e) {
          console.error("Collision check failed", e);
        }

        if (intersection) {
          console.warn(`Collision detected between ${id1} and ${id2}`);
          collisionSet.add(id1);
          collisionSet.add(id2);
        }
      }
    }

    // Update state only if changed
    if (collisionSet.size !== collidingLinks.size || ![...collisionSet].every(x => collidingLinks.has(x))) {
      setCollidingLinks(collisionSet);
    }

    return collisionSet.size > 0;
  };

  // Effect to check collisions whenever joints change (covers Sidebar/Undo/Redo)
  useEffect(() => {
    if (collisionMode !== 'off') {
      checkCollisions();
    }
  }, [joints, collisionMode, objectRefs, collisionBoxScale]);

  useEffect(() => {
    const controls = transformControlsRef.current;
    let targetId = null;
    if (selectedItem.id) {
      if (selectedItem.type === 'joint') {
        targetId = selectedItem.id;
      } else if (selectedItem.type === 'link') {
        const parentJointId = getParentJointId(selectedItem.id);
        if (parentJointId) targetId = parentJointId;
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

    if (controls) {
      const callback = (event: any) => {
        const cameraControls = useRobotStore.getState().cameraControls;
        if (cameraControls) {
          cameraControls.enabled = !event.value;
        }
      };
      controls.addEventListener('dragging-changed', callback);
      return () => controls.removeEventListener('dragging-changed', callback);
    }
  }, [selectedItem, objectRefs]);

  const handleObjectChange = () => {
    const controls = transformControlsRef.current;
    let actualJointId: string | null = null;
    if (selectedItem.type === 'joint') actualJointId = selectedItem.id;
    else if (selectedItem.type === 'link') actualJointId = getParentJointId(selectedItem.id || "");

    if (!controls || !controls.object || !actualJointId) return;

    const state = useRobotStore.getState();
    const joint = state.joints[actualJointId];
    if (!joint || joint.type === 'fixed') return;

    const { object } = controls;
    let proposedValues = { ...joint.currentValues };

    // 1. Calculate Proposed Values
    if (joint.type === 'rotational') {
      const euler = new THREE.Euler().setFromQuaternion(object.quaternion, 'ZYX');

      const axis = controls.axis; // 'X', 'Y', 'Z' corresponding to Roll, Pitch, Yaw in local space (roughly)
      // Note: TransformControls axis: X=Roll, Y=Pitch, Z=Yaw when aligned.
      // We only update the value corresponding to the dragged handle to prevent Euler decomposition
      // from flipping other values (gimbal lock/interaction issues).

      if (axis === 'X' && joint.dof.roll) proposedValues.roll = clamp(euler.x, joint.limits.roll.lower, joint.limits.roll.upper);
      if (axis === 'Y' && joint.dof.pitch) proposedValues.pitch = clamp(euler.y, joint.limits.pitch.lower, joint.limits.pitch.upper);
      if (axis === 'Z' && joint.dof.yaw) proposedValues.yaw = clamp(euler.z, joint.limits.yaw.lower, joint.limits.yaw.upper);

      // Fallback: If axis is null or weird (e.g. freemove), update all enabled
      if (!axis || axis === 'XYZ') {
        if (joint.dof.roll) proposedValues.roll = clamp(euler.x, joint.limits.roll.lower, joint.limits.roll.upper);
        if (joint.dof.pitch) proposedValues.pitch = clamp(euler.y, joint.limits.pitch.lower, joint.limits.pitch.upper);
        if (joint.dof.yaw) proposedValues.yaw = clamp(euler.z, joint.limits.yaw.lower, joint.limits.yaw.upper);
      }

    } else if (joint.type === 'prismatic') {
      const pos = object.position;
      const axis = new THREE.Vector3(...joint.axis).normalize();
      const displacement = pos.dot(axis);
      const clampedDisp = clamp(displacement, joint.limits.displacement?.lower, joint.limits.displacement?.upper);
      proposedValues = { ...joint.currentValues, displacement: clampedDisp };
    }

    // 2. We need to physically APPLY these values to the scene graph nodes *before* checking collision.
    // The TransformControls has already moved the 'motionGroup' of the joint we are editing.
    // However, the standard `useFrame` in JointWrapper will overwrite this on next frame.
    // AND, importantly, the TransformControls only moves the specific node it is attached to.
    // If we have a chain, the children move with it automatically because they are children in ThreeJS graph.
    // So the visual state IS theoretically correct for testing right now.

    // BUT we need to ensure local matrix updates have propagated if we rely on World Matrix.
    // object.updateMatrixWorld(true); // force update down the tree?

    // 3. Check Collision
    // We run the check to update the visual feedback (red color), but we DO NOT block movement.
    // Blocking movement causes detailed deadlocks (can't move out of collision).
    checkCollisions();

    // Commit the values
    updateJoint(actualJointId, 'currentValues', proposedValues);
  }

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
    <CollisionContext.Provider value={collidingLinks}>
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

      {/* Debug render for collision boxes */}
      {collisionMode === 'box' && Array.from(objectRefs.entries()).map(([id, ref]) => (
        <CollisionCylinderHelper
          key={id}
          objectRef={ref}
          scale={collisionBoxScale}
          visible={true}
          color={collidingLinks.has(id) ? '#ff0000' : '#ffff00'}
        />
      ))}

    </CollisionContext.Provider>
  );
};

export default RobotVisualizer;
