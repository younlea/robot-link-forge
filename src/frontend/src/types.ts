// src/frontend/src/types.ts

export type Visual = {
  type: 'box' | 'cylinder' | 'sphere' | 'none';
  dimensions: [number, number, number];
  color: string;
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

  // Limits
  limit?: {
    lower: number;
    upper: number;
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
}

export interface RobotActions {
  addJoint: (parentLinkId: string) => void;
  addChainedJoint: (parentJointId: string) => void;

  // Generic update actions
  updateJoint: (id: string, path: string, value: any) => void;
  updateLink: (id: string, path: string, value: any) => void;
  
  // Selection
  selectItem: (id: string | null, type: 'link' | 'joint' | null) => void;
}
