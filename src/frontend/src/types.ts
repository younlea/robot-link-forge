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

export type JointType = 'fixed' | 'rotational' | 'prismatic' | 'rolling';

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
  visual: Visual;

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

  // Rolling Contact Joint properties (only used when type === 'rolling')
  rollingParams?: RollingContactParams;
}

// --- Rolling Contact Joint ---

export interface RollingContactParams {
  curvatureRadius: number;     // Curvature radius (m)
  contactFriction: number;     // Contact friction coefficient
  surfaceType: 'convex' | 'concave';
}

// --- Tendon System (independent entity, NOT a JointType) ---

export interface TendonRoutingPoint {
  id: string;
  linkId: string;                          // Which link this point is on
  localPosition: [number, number, number]; // Position in link-local coordinates
  worldPosition?: [number, number, number]; // Cached world position (for visualization)
}

export interface Tendon {
  id: string;
  name: string;
  type: 'active' | 'passive';

  // Routing path through links
  routingPoints: TendonRoutingPoint[];

  // Physics parameters
  stiffness: number;     // N/m (required for passive tendons)
  damping: number;       // N·s/m
  restLength: number;    // Natural length (m)

  // Active tendon: motor reference
  actuatorMotorId?: string;  // Which motor drives this tendon
  momentArm?: number;        // Effective moment arm (m)

  // Visualization
  color: string;
  width: number;           // Line thickness (visualization only)
}

// --- Obstacle System (fixed-position objects for contact simulation) ---

export interface ObstaclePhysics {
  friction: number;
  solref: [number, number];           // MuJoCo contact solver parameters
  solimp: [number, number, number];   // MuJoCo impedance parameters
}

export interface Obstacle {
  id: string;
  name: string;
  shape: 'box' | 'sphere' | 'cylinder';
  dimensions: [number, number, number];
  position: [number, number, number];
  rotation: [number, number, number];  // RPY
  color: string;
  physics: ObstaclePhysics;
  enabled: boolean;  // Toggle on/off without deleting
}

// --- Sensor System ---

export interface SensorDef {
  id: string;
  type: 'touch' | 'force';
  linkId: string;
  localPosition: [number, number, number];
  localRotation: [number, number, number];
  siteName: string;  // For MJCF export
}

// --- Interaction Modes ---

export type InteractionMode = 'select' | 'tendon-routing' | 'sensor-placement';

// --- Motion Recording Types ---

export type RecordingMode = 'slider' | 'live' | 'glove';

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
  selectionCandidates: string[]; // List of IDs intersecting with click

  cameraMode: 'rotate' | 'pan';
  cameraControls: CameraControls | null;
  serverProjects: string[];
  importUnit: 'm' | 'cm' | 'mm';
  collisionMode: 'box' | 'mesh' | 'off';
  collisionBoxScale: number;

  // --- Advanced Simulation Systems ---
  tendons: Record<string, Tendon>;
  obstacles: Record<string, Obstacle>;
  sensors: Record<string, SensorDef>;
  interactionMode: InteractionMode;
  activeTendonId: string | null;  // Currently editing tendon (for routing mode)

  // Motion Recording State
  recordingMode: RecordingMode | null;
  isRecording: boolean;
  recordingStartTime: number | null;
  currentRecording: MotionRecording | null;
  recordings: MotionRecording[];
  recordingInterval: any | null;
  playbackState: PlaybackState;
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
  setHighlightedItem: (id: string | null, type: 'link' | 'joint' | null) => void;
  setSelectionCandidates: (ids: string[]) => void;

  // Camera Control
  setCameraMode: (mode: 'rotate' | 'pan') => void;
  setCameraControls: (controls: CameraControls | null) => void;
  zoomIn: () => void;
  zoomOut: () => void;

  // Export
  exportURDF: (robotName: string) => Promise<void>;
  exportURDF_ROS2: (robotName: string) => Promise<void>;
  exportMujocoURDF: (robotName: string) => Promise<void>;
  exportMujocoMJCF: (robotName: string, useMeshCollision?: boolean, directHand?: boolean) => Promise<void>;
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

  // --- Tendon Actions ---
  addTendon: (type: 'active' | 'passive') => void;
  updateTendon: (id: string, path: string, value: any) => void;
  deleteTendon: (id: string) => void;
  addTendonRoutingPoint: (tendonId: string, linkId: string, localPosition: [number, number, number]) => void;
  removeTendonRoutingPoint: (tendonId: string, pointId: string) => void;
  setActiveTendonId: (id: string | null) => void;

  // --- Obstacle Actions ---
  addObstacle: (shape: 'box' | 'sphere' | 'cylinder') => void;
  updateObstacle: (id: string, path: string, value: any) => void;
  deleteObstacle: (id: string) => void;

  // --- Sensor Actions ---
  addSensor: (type: 'touch' | 'force', linkId: string, localPosition: [number, number, number]) => void;
  deleteSensor: (id: string) => void;

  // --- Interaction Mode ---
  setInteractionMode: (mode: InteractionMode) => void;

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
  deleteRecordingFromServer: (filename: string) => Promise<void>;
  loadKeyframePose: (keyframeId: string) => void;
  updateRecordingMetadata: (updates: Partial<{ name: string; duration: number }>) => void;
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
