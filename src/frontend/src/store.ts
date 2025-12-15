// src/frontend/src/store.ts
import { create } from 'zustand';
import { v4 as uuidv4 } from 'uuid';
import { RobotState, RobotActions, RobotLink, RobotJoint, JointType } from './types';
import { set } from 'lodash';

// Helper function to update nested properties immutably.
const updateDeep = (obj: any, path: string, value: any) => {
    const newObj = JSON.parse(JSON.stringify(obj));
    set(newObj, path, value);
    return newObj;
};

const createInitialState = (): RobotState => {
  const baseLinkId = `link-${uuidv4()}`;
  return {
    links: { [baseLinkId]: { id: baseLinkId, name: 'base_link', visual: { type: 'box', dimensions: [0.1, 0.2, 0.1], color: '#cccccc' }, childJoints: [] } },
    joints: {},
    baseLinkId: baseLinkId,
    selectedItem: { id: baseLinkId, type: 'link' },
  };
};

const createDefaultJoint = (parentLinkId: string): RobotJoint => ({
    id: `joint-${uuidv4()}`,
    name: 'new_joint',
    parentLinkId,
    childLinkId: null,
    type: 'fixed',
    dof: { roll: false, pitch: false, yaw: true },
    axis: [0, 0, 1],
    currentValues: { roll: 0, pitch: 0, yaw: 0, displacement: 0 },
    limit: { lower: -Math.PI, upper: Math.PI },
    origin: { xyz: [0,0,0], rpy: [0,0,0] },
});

// This now ALWAYS creates a visible cylinder link.
const createDefaultLink = (): RobotLink => ({
    id: `link-${uuidv4()}`,
    name: 'new_link',
    visual: {
        type: 'cylinder',
        dimensions: [0.02, 0.02, 0.2],
        color: `#${Math.floor(Math.random() * 16777215).toString(16).padStart(6, '0')}`,
    },
    childJoints: [],
});

export const useRobotStore = create<RobotState & RobotActions>((setState) => ({
  ...createInitialState(),

  selectItem: (id, type) => setState({ selectedItem: { id, type } }),

  updateJoint: (id, path, value) => {
    setState((state) => {
      const originalJoint = state.joints[id];
      if (!originalJoint) return {};
      let updatedJoint = updateDeep(originalJoint, path, value);
      if (path === 'type') {
        const newType = value as JointType;
        if (newType === 'rotational') updatedJoint = updateDeep(updatedJoint, 'currentValues.displacement', 0);
        else if (newType === 'prismatic') updatedJoint = updateDeep(updatedJoint, 'currentValues', { roll: 0, pitch: 0, yaw: 0, displacement: updatedJoint.currentValues.displacement });
      }
      return { joints: { ...state.joints, [id]: updatedJoint } };
    });
  },

  updateLink: (id, path, value) => {
    setState((state) => {
      if (!state.links[id]) return {};
      const updatedLink = updateDeep(state.links[id], path, value);
      return { links: { ...state.links, [id]: updatedLink } };
    });
  },

  // Creates ONLY a joint, attached to a link. Used to start a chain.
  addJoint: (parentLinkId) => {
    setState((state) => {
      const parentLink = state.links[parentLinkId];
      if (!parentLink) return {};
      const newJoint = createDefaultJoint(parentLinkId);
      
      // Position new joint at parent's origin. User will move it.
      newJoint.origin = { xyz: [0, 0, 0], rpy: [0, 0, 0] };

      return {
        joints: { ...state.joints, [newJoint.id]: newJoint },
        links: { ...state.links, [parentLinkId]: { ...parentLink, childJoints: [...parentLink.childJoints, newJoint.id] } },
        selectedItem: { id: newJoint.id, type: 'joint' }
      };
    });
  },

  // Creates a visible link AND a new joint at its end. The core "chaining" action.
  addChainedJoint: (parentJointId) => {
    setState((state) => {
        const parentJoint = state.joints[parentJointId];
        if (!parentJoint || parentJoint.childLinkId) return {};

        // 1. Create the intermediate link (now visible by default)
        const intermediateLink = createDefaultLink();

        // 2. Create the new joint that will be at the end of the new link
        const newJoint = createDefaultJoint(intermediateLink.id);
        // Position new joint at the end of the intermediate link's geometry
        newJoint.origin = { xyz: [0, 0, intermediateLink.visual.dimensions[2]], rpy: [0, 0, 0] };
        
        // 3. Wire everything up
        intermediateLink.childJoints.push(newJoint.id);
        const updatedParentJoint = { ...parentJoint, childLinkId: intermediateLink.id };

        return {
            links: { ...state.links, [intermediateLink.id]: intermediateLink },
            joints: { ...state.joints, [parentJoint.id]: updatedParentJoint, [newJoint.id]: newJoint },
            selectedItem: { id: newJoint.id, type: 'joint' }
        }
    })
  }
}));