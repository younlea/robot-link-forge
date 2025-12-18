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
    cameraMode: 'rotate', // Default camera mode
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

export const useRobotStore = create<RobotState & RobotActions>((setState, getState) => ({
  ...createInitialState(),
  cameraControls: null,

  setCameraControls: (controls) => setState({ cameraControls: controls }),

  zoomIn: () => {
    // Dolly forward to zoom in. The value is a distance in world units.
    // A negative value moves the camera closer to the target.
    getState().cameraControls?.dolly(-0.5, true);
  },

  zoomOut: () => {
    // Dolly backward to zoom out.
    // A positive value moves the camera away from the target.
    getState().cameraControls?.dolly(0.5, true);
  },

  selectItem: (id, type) => setState({ selectedItem: { id, type } }),
  setCameraMode: (mode) => setState({ cameraMode: mode }),


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
  },

  // --- Save/Load Functionality ---
  saveRobot: async () => {
    const { links, joints, baseLinkId } = getState();
    const robotData = {
      links,
      joints,
      baseLinkId,
    };
    const jsonString = JSON.stringify(robotData, null, 2);
    const blob = new Blob([jsonString], { type: 'application/json' });

    // Modern approach: File System Access API
    if ('showSaveFilePicker' in window) {
      try {
        const handle = await window.showSaveFilePicker({
          suggestedName: 'robot-model.json',
          types: [{
            description: 'JSON Files',
            accept: { 'application/json': ['.json'] },
          }],
        });
        const writable = await handle.createWritable();
        await writable.write(blob);
        await writable.close();
        return; // Success
      } catch (err) {
        // Handle cancellation or errors
        if (err instanceof DOMException && err.name === 'AbortError') {
          console.log('Save dialog was cancelled.');
        } else {
          console.error('Error saving file:', err);
        }
        return; // Exit on error or cancellation
      }
    }

    // Fallback approach for older browsers
    const fileName = prompt("Enter a filename for your robot model:", "robot-model.json");
    if (fileName) {
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = fileName.endsWith('.json') ? fileName : `${fileName}.json`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    }
  },

  loadRobot: (file: File) => {
    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const jsonString = event.target?.result;
        if (typeof jsonString !== 'string') {
          throw new Error('File could not be read as text.');
        }
        const robotData = JSON.parse(jsonString);
        // Basic validation
        if (robotData.links && robotData.joints && robotData.baseLinkId) {
          setState({
            links: robotData.links,
            joints: robotData.joints,
            baseLinkId: robotData.baseLinkId,
            selectedItem: { id: null, type: null }, // Deselect after loading
          });
        } else {
          throw new Error('Invalid robot data format.');
        }
      } catch (error) {
        console.error("Failed to load and parse robot data:", error);
        alert("Failed to load robot file. Please check the console for details.");
      }
    };
    reader.onerror = (error) => {
        console.error("Error reading file:", error);
        alert("Error reading file.");
    }
    reader.readAsText(file);
  },
}));