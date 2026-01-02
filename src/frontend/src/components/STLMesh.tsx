// src/frontend/src/components/STLMesh.tsx
import React, { Suspense, useEffect } from 'react';
import { useLoader } from '@react-three/fiber';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import * as THREE from 'three';
import { Euler } from 'three';
import { useRobotStore } from '../store';

interface STLMeshProps {
  linkId: string; // Required to update the correct link
  url: string;
  scale?: [number, number, number];
  origin?: {
    xyz: [number, number, number];
    rpy: [number, number, number];
  };
  color?: string;
}

function LoadedMesh({ url, scale, origin, color, linkId }: STLMeshProps) {
  const { updateLink } = useRobotStore();
  const geom = useLoader(STLLoader, url);

  useEffect(() => {
    if (geom) {
      // This effect runs when the geometry is loaded.
      // We compute the bounding box and store its size in our global state.
      // This is crucial for the "Fit to Link" functionality.
      geom.computeBoundingBox();
      geom.computeBoundsTree(); // Compute BVH for accelerated raycasting
      const box = geom.boundingBox;
      if (box) {
        const size = box.getSize(new THREE.Vector3());
        updateLink(linkId, 'visual.meshBoundingBox', [size.x, size.y, size.z]);
      }
    }
  }, [geom, linkId, updateLink]); // Effect depends on the loaded geometry

  // The group will handle the user-defined origin transform.
  const groupPosition = origin?.xyz || [0, 0, 0];
  const groupRotation = new Euler(...(origin?.rpy || [0, 0, 0]));

  return (
    <group position={groupPosition} rotation={groupRotation} scale={scale || [1, 1, 1]}>
      <mesh>
        <primitive object={geom} attach="geometry" />
        <meshStandardMaterial color={color || '#ff8800'} side={THREE.DoubleSide} />
      </mesh>
    </group>
  );
}

export function STLMesh(props: STLMeshProps) {
  // A fallback while the mesh is loading.
  // You can replace this with a more sophisticated placeholder.
  const fallback = (
    <mesh scale={props.scale || [1, 1, 1]}>
      <boxGeometry args={[0.1, 0.1, 0.1]} />
      <meshBasicMaterial wireframe color="orange" />
    </mesh>
  );

  return (
    <Suspense fallback={fallback}>
      {props.url && <LoadedMesh {...props} />}
    </Suspense>
  );
}
