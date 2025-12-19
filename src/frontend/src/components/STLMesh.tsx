// src/frontend/src/components/STLMesh.tsx
import React, { Suspense } from 'react';
import { useLoader } from '@react-three/fiber';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import * as THREE from 'three';
import { Euler } from 'three';

interface STLMeshProps {
  url: string;
  scale?: [number, number, number];
  origin?: {
    xyz: [number, number, number];
    rpy: [number, number, number];
  };
  color?: string;
}

function LoadedMesh({ url, scale, origin, color }: STLMeshProps) {
  const geom = useLoader(STLLoader, url);

  // The STLLoader returns a BufferGeometry.
  // We need to create a mesh from it.
  const mesh = new THREE.Mesh(geom);

  // We should compute the bounding box to center the geometry
  // so that the origin transform is applied correctly.
  const box = new THREE.Box3().setFromObject(mesh);
  const center = box.getCenter(new THREE.Vector3());
  
  // The position of the mesh itself will be the negation of its center,
  // effectively moving its geometric center to the group's origin (0,0,0).
  const position = [-center.x, -center.y, -center.z];

  // The group will handle the user-defined origin transform.
  const groupPosition = origin?.xyz || [0, 0, 0];
  const groupRotation = new Euler(...(origin?.rpy || [0, 0, 0]));

  return (
    <group position={groupPosition} rotation={groupRotation} scale={scale || [1, 1, 1]}>
      <mesh position={position}>
        <primitive object={geom} attach="geometry" />
        <meshStandardMaterial color={color || '#ff8800'} />
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
