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

  // Passive Mode Config
  isPassive?: boolean;
  equation?: string;

  // Current values for sliders
  currentValues: JointValues;

  // Visual representation of the joint (e.g. motor housing)
  // Legacy single visual (defaults to last DOF or generic)
  visual: Visual;

  // New: Multiple visuals keyed by DOF ('roll', 'pitch', 'yaw', 'displacement')
  // Allows attaching specific meshes to specific axes of rotation/motion
  visuals?: Record<string, Visual>;

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

// --- Motion Recording Types ---

export type RecordingMode = 'slider' | 'camera' | 'input_device';

export interface MotionKeyframe {
  id: string;
  timestamp: number;  // ms from recording start
  transitionDuration?: number; // Optional: effective duration for movement. If < (next - curr), we wait.
  jointValues: Record<string, JointValues>;  // jointId -> values snapshot
}

export interface MotionRecording {
  id: string;
  name: string;
  mode: RecordingMode;
  keyframes: MotionKeyframe[];
  duration: number;  // total duration in ms
  createdAt: number;  // Date.now() when created
}

export interface PlaybackState {
  isPlaying: boolean;
  currentTime: number;  // ms
  recordingId: string | null;
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
  highlightedItem: SelectedItem; // Visual highlight for helper UI
  cameraMode: 'rotate' | 'pan';
  cameraControls: CameraControls | null;
  serverProjects: string[];
  importUnit: 'm' | 'cm' | 'mm';
  collisionMode: 'box' | 'mesh' | 'off';
  collisionBoxScale: number;

  // Motion Recording State
  recordingMode: RecordingMode | null;
  isRecording: boolean;
  recordingStartTime: number | null;
  currentRecording: MotionRecording | null;
  recordings: MotionRecording[];
  playbackState: PlaybackState;
}

export interface RobotActions {
  addJoint: (parentLinkId: string) => void;
  addChainedJoint: (parentJointId: string) => void;
  uploadAndSetMesh: (itemId: string, itemType: 'link' | 'joint', file: File, dofKey?: string) => Promise<void>;
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
  setHighlightedItem: (id: string | null, type: 'link' | 'joint' | null) => void;

  // Camera Control
  setCameraMode: (mode: 'rotate' | 'pan') => void;
  setCameraControls: (controls: CameraControls | null) => void;
  zoomIn: () => void;
  zoomOut: () => void;

  // Export
  exportURDF: (robotName: string) => Promise<void>;
  exportURDF_ROS2: (robotName: string) => Promise<void>;
  exportMujocoURDF: (robotName: string) => Promise<void>;
  exportMujocoMJCF: (robotName: string, useMeshCollision?: boolean) => Promise<void>;
  exportGazebo: (robotName: string) => Promise<void>;
  exportGazeboROS2: (robotName: string) => Promise<void>;

  // App State Management
  saveRobot: () => Promise<void>; // Local Download
  loadRobot: (file: File) => Promise<void>; // Local Upload

  // Server-side Persistence
  getProjectList: () => Promise<void>;
  saveProjectToServer: (name: string) => Promise<void>;
  loadProjectFromServer: (filename: string) => Promise<void>;
  deleteProjectFromServer: (filename: string) => Promise<void>;

  deleteItem: (id: string, type: 'link' | 'joint') => void;
  resetProject: () => void;

  // Settings
  setImportUnit: (unit: 'm' | 'cm' | 'mm') => void;
  setCollisionMode: (mode: 'box' | 'mesh' | 'off') => void;
  setCollisionBoxScale: (scale: number) => void;

  // Motion Recording Actions
  setRecordingMode: (mode: RecordingMode | null) => void;
  startRecording: (name?: string) => void;
  stopRecording: () => void;
  captureKeyframe: () => void;  // For Slider mode
  deleteKeyframe: (keyframeId: string) => void;
  updateKeyframeTiming: (keyframeId: string, newTimestamp: number) => void;
  updateKeyframePose: (keyframeId: string) => void;
  updateKeyframeTransition: (keyframeId: string, duration: number | undefined) => void;

  // Persistence
  saveRecordingsLocal: () => void;
  loadRecordingsLocal: (file: File) => Promise<void>;
  fetchRecordingList: () => Promise<string[]>;
  saveRecordingsToServer: (filename: string) => Promise<void>;
  loadRecordingsFromServer: (filename: string) => Promise<void>;
  loadKeyframePose: (keyframeId: string) => void;
  editRecording: (recordingId: string) => void;
  saveRecording: () => void;
  cancelEditRecording: () => void;
  playRecording: (recordingId?: string) => void;
  pausePlayback: () => void;
  stopPlayback: () => void;
  seekPlayback: (timeMs: number) => void;
  deleteRecording: (id: string) => void;
  renameRecording: (id: string, newName: string) => void;
}
