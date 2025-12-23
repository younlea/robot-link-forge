// src/frontend/src/types.ts
import type { CameraControls } from '@react-three/drei';

export type Visual = {
  type: 'box' | 'cylinder' | 'sphere' | 'mesh' | 'none';
  dimensions: [number, number, number];
  color: string;
  // For meshes
  meshUrl?: string | null;
  meshScale?: [number, number, number];
  meshBoundingBox?: [number, number, number]; // Stores the computed size of the mesh
  meshOrigin?: {
    xyz: [number, number, number];
    rpy: [number, number, number];
  };
};

export interface RobotLink {
  id: string;
  name: string;
  visual: Visual;
  childJoints: string[];
}

// --- New Comprehensive Joint Model ---

export type JointType = 'fixed' | 'rotational' | 'prismatic';

export type RotationalDof = {
  roll: boolean; // x-axis
  pitch: boolean; // y-axis
  yaw: boolean; // z-axis
};

export type JointValues = {
  roll: number;
  pitch: number;
  yaw: number;
  displacement: number;
}

export type Limit = {
  lower: number;
  upper: number;
};

export interface RobotJoint {
  id: string;
  name: string;
  parentLinkId: string;
  childLinkId: string | null; // A joint can now exist without a child link

  // Motion properties
  type: JointType;

  // For 'rotational'
  dof: RotationalDof;

  // For 'prismatic'
  axis: [number, number, number];

  // Current values for sliders
  currentValues: JointValues;

  // Limits for each potential degree of freedom
  limits: {
    roll: Limit;
    pitch: Limit;
    yaw: Limit;
    displacement: Limit;
  };

  // Physical relationship to parent
  origin: {
    xyz: [number, number, number];
    rpy: [number, number, number];
  };
}

export type SelectedItem = {
  id: string | null;
  type: 'link' | 'joint' | null;
}

export interface RobotState {
  links: Record<string, RobotLink>;
  joints: Record<string, RobotJoint>;
  baseLinkId: string;
  selectedItem: SelectedItem;
  cameraMode: 'rotate' | 'pan';
  cameraControls: CameraControls | null;
  serverProjects: string[];
}

export interface RobotActions {
  addJoint: (parentLinkId: string) => void;
  addChainedJoint: (parentJointId: string) => void;
  uploadAndSetMesh: (itemId: string, itemType: 'link' | 'joint', file: File) => Promise<void>;
  updateMeshTransform: (
    itemId: string,
    itemType: 'link' | 'joint',
    transform: {
      scale?: [number, number, number];
      origin?: {
        xyz?: [number, number, number];
        rpy?: [number, number, number];
      };
    }
  ) => void;
  fitMeshToLink: (linkId: string) => void;
  resetJointsToZero: () => void;

  // Generic update actions
  updateJoint: (id: string, path: string, value: any) => void;
  updateLink: (id: string, path: string, value: any) => void;

  // Selection
  selectItem: (id: string | null, type: 'link' | 'joint' | null) => void;

  // Camera Control
  setCameraMode: (mode: 'rotate' | 'pan') => void;
  setCameraControls: (controls: CameraControls | null) => void;
  zoomIn: () => void;
  zoomOut: () => void;

  // Export
  exportURDF: (robotName: string) => Promise<void>;
  exportURDF_ROS2: (robotName: string) => Promise<void>;

  // App State Management
  saveRobot: () => Promise<void>; // Local Download
  loadRobot: (file: File) => Promise<void>; // Local Upload

  // Server-side Persistence
  getProjectList: () => Promise<void>;
  saveProjectToServer: (name: string) => Promise<void>;
  loadProjectFromServer: (filename: string) => Promise<void>;

  deleteItem: (id: string, type: 'link' | 'joint') => void;
  resetProject: () => void;
}
