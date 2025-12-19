import { create } from 'zustand';
import { v4 as uuidv4 } from 'uuid';
import * as THREE from 'three';
import { RobotState, RobotActions, RobotLink, RobotJoint, JointType } from './types';
import { set } from 'lodash';
import JSZip from 'jszip'; // Needed for advanced save/load

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
    limits: {
        roll: { lower: -Math.PI, upper: Math.PI },
        pitch: { lower: -Math.PI, upper: Math.PI },
        yaw: { lower: -Math.PI, upper: Math.PI },
        displacement: { lower: -1, upper: 1 },
    },
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

  uploadAndSetMesh: async (itemId, itemType, file) => {
    const { joints, links } = getState();
    let targetLinkId: string | null = null;

    if (itemType === 'link') {
        targetLinkId = itemId;
    } else if (itemType === 'joint') {
        const joint = joints[itemId];
        if (joint && joint.childLinkId) {
            targetLinkId = joint.childLinkId;
        } else {
            console.error("Cannot add mesh: Selected joint has no child link.");
            return;
        }
    }

    if (!targetLinkId || !links[targetLinkId]) {
        console.error("Cannot add mesh: Target link not found.");
        return;
    }

    const finalTargetLinkId = targetLinkId;

    const formData = new FormData();
    formData.append('file', file);

    try {
        const response = await fetch('http://localhost:8000/api/upload-stl', {
            method: 'POST',
            body: formData,
        });

        if (!response.ok) {
            throw new Error(`Upload failed with status: ${response.status}`);
        }

        const result = await response.json();
        const meshUrl = `http://localhost:8000${result.url}`;
        
        setState(state => {
            const linkToUpdate = state.links[finalTargetLinkId];
            if (!linkToUpdate) return {};

            const updatedLink = updateDeep(linkToUpdate, 'visual', {
                ...linkToUpdate.visual,
                type: 'mesh',
                meshUrl: meshUrl,
                meshScale: [1, 1, 1],
                meshOrigin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                meshBoundingBox: undefined, // Reset bounding box on new mesh upload
            });

            return { links: { ...state.links, [finalTargetLinkId]: updatedLink } };
        });

    } catch (error) {
        console.error("Failed to upload and set mesh:", error);
        alert(`Failed to upload mesh. Make sure the backend server is running. Error: ${error.message}`);
    }
  },
  
  fitMeshToLink: (linkId) => {
    setState(state => {
        const link = state.links[linkId];
        if (!link || link.visual.type !== 'mesh' || !link.visual.meshBoundingBox) {
            console.warn("Fit to link requires a mesh with a calculated bounding box.");
            alert("Cannot fit to length: Bounding box not calculated. Ensure the mesh is loaded correctly.");
            return {};
        }

        if (link.childJoints.length === 0) {
            alert("Cannot fit to length: The selected link has no child joint to define a length.");
            return {};
        }

        const childJoint = state.joints[link.childJoints[0]];
        if (!childJoint) return {};

        // Calculate link length based on child joint position
        const childPos = new THREE.Vector3(...childJoint.origin.xyz);
        const linkLength = childPos.length();

        if (linkLength < 0.0001) {
            alert("Cannot fit to length: Link length is effectively zero.");
            return {};
        }

        // Assume the primary axis of the STL is Y, which is a common convention for links.
        const stlLength = link.visual.meshBoundingBox[1];
        if (!stlLength || stlLength < 0.0001) {
            alert("Cannot fit to length: Invalid STL bounding box dimension. The height (Y-axis) of the mesh is zero.");
            return {};
        }

        const scaleFactor = linkLength / stlLength;
        const newScale: [number, number, number] = [scaleFactor, scaleFactor, scaleFactor];

        const updatedLink = updateDeep(link, 'visual.meshScale', newScale);
        
        return { links: { ...state.links, [linkId]: updatedLink } };
    });
  },

  updateMeshTransform: (itemId, itemType, transform) => {
    setState(state => {
        let targetLinkId: string | null = null;

        if (itemType === 'link') {
            targetLinkId = itemId;
        } else if (itemType === 'joint') {
            const joint = state.joints[itemId];
            if (joint && joint.childLinkId) {
                targetLinkId = joint.childLinkId;
            }
        }
        
        if (!targetLinkId || !state.links[targetLinkId]) return {};
        
        const linkToUpdate = state.links[targetLinkId];
        let updatedLink = linkToUpdate;

        if (transform.scale) {
            updatedLink = updateDeep(updatedLink, 'visual.meshScale', transform.scale);
        }
        if (transform.origin?.xyz) {
            updatedLink = updateDeep(updatedLink, 'visual.meshOrigin.xyz', transform.origin.xyz);
        }
        if (transform.origin?.rpy) {
            updatedLink = updateDeep(updatedLink, 'visual.meshOrigin.rpy', transform.origin.rpy);
        }

        return { links: { ...state.links, [targetLinkId]: updatedLink } };
    });
  },

  setCameraControls: (controls) => setState({ cameraControls: controls }),


  zoomIn: () => {
    getState().cameraControls?.dolly(-0.5, true);
  },

  zoomOut: () => {
    getState().cameraControls?.dolly(0.5, true);
  },

  selectItem: (id, type) => setState({ selectedItem: { id, type } }),
  setCameraMode: (mode) => setState({ cameraMode: mode }),

  resetJointsToZero: () => {
    setState(state => {
        const newJoints = { ...state.joints };
        for (const jointId in newJoints) {
            const joint = newJoints[jointId];
            // Only reset movable joints
            if (joint.type !== 'fixed') {
                const newValues = {
                    roll: 0,
                    pitch: 0,
                    yaw: 0,
                    displacement: 0,
                };
                newJoints[jointId] = updateDeep(joint, 'currentValues', newValues);
            }
        }
        return { joints: newJoints };
    });
  },

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

  addJoint: (parentLinkId) => {
    setState((state) => {
      const parentLink = state.links[parentLinkId];
      if (!parentLink) return {};
      const newJoint = createDefaultJoint(parentLinkId);
      
      newJoint.origin = { xyz: [0, 0, 0], rpy: [0, 0, 0] };

      return {
        joints: { ...state.joints, [newJoint.id]: newJoint },
        links: { ...state.links, [parentLinkId]: { ...parentLink, childJoints: [...parentLink.childJoints, newJoint.id] } },
        selectedItem: { id: newJoint.id, type: 'joint' }
      };
    });
  },

  addChainedJoint: (parentJointId) => {
    setState((state) => {
        const parentJoint = state.joints[parentJointId];
        if (!parentJoint || parentJoint.childLinkId) return {};

        const intermediateLink = createDefaultLink();
        const newJoint = createDefaultJoint(intermediateLink.id);
        newJoint.origin = { xyz: [0, 0, intermediateLink.visual.dimensions[2]], rpy: [0, 0, 0] };
        
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
    try {
        const { links, joints, baseLinkId } = getState();
        const zip = new JSZip();

        // Create a deep copy to modify URLs without affecting current state
        const linksToSave = JSON.parse(JSON.stringify(links));
        
        const meshFolder = zip.folder('meshes');
        if (!meshFolder) throw new Error("Failed to create 'meshes' folder in zip.");

        const meshUrlMap = new Map<string, string>();

        for (const link of Object.values(linksToSave as Record<string, RobotLink>)) {
            if (link.visual.type === 'mesh' && link.visual.meshUrl && link.visual.meshUrl.startsWith('http')) {
                // Check if we've already processed this URL
                if (meshUrlMap.has(link.visual.meshUrl)) {
                    link.visual.meshUrl = meshUrlMap.get(link.visual.meshUrl);
                    continue;
                }

                const response = await fetch(link.visual.meshUrl);
                if (!response.ok) {
                    console.warn(`Could not fetch mesh for ${link.name}: ${link.visual.meshUrl}`);
                    continue;
                }
                const blob = await response.blob();
                const newMeshName = `${uuidv4()}.stl`;
                const relativePath = `meshes/${newMeshName}`;
                
                meshFolder.file(newMeshName, blob);

                // Update the URL in our data-to-be-saved
                link.visual.meshUrl = relativePath;
                meshUrlMap.set(link.visual.meshUrl, relativePath); // Store original for mapping
            }
        }

        const robotData = { links: linksToSave, joints, baseLinkId };
        zip.file('robot-scene.json', JSON.stringify(robotData, null, 2));
        
        const zipBlob = await zip.generateAsync({ type: 'blob' });

        if ('showSaveFilePicker' in window) {
            const handle = await window.showSaveFilePicker({
                suggestedName: 'robot-model.zip',
                types: [{ description: 'RobotLinkForge Zip Archive', accept: { 'application/zip': ['.zip'] }}],
            });
            const writable = await handle.createWritable();
            await writable.write(zipBlob);
            await writable.close();
        } else {
            // Fallback for older browsers
            const a = document.createElement('a');
            a.href = URL.createObjectURL(zipBlob);
            a.download = 'robot-model.zip';
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(a.href);
        }

    } catch (err) {
        console.error('Error saving robot project:', err);
        alert(`Failed to save project: ${err.message}`);
    }
  },

  loadRobot: async (file: File) => {
    const processAndSetState = (robotData: any) => {
        // --- Migration Logic ---
        for (const jointId in robotData.joints) {
            const joint = robotData.joints[jointId];
            // If old `limit` property exists and `limits` does not
            if (joint.limit && !joint.limits) {
                const defaultLimit = joint.limit;
                joint.limits = {
                    roll: { ...defaultLimit },
                    pitch: { ...defaultLimit },
                    yaw: { ...defaultLimit },
                    displacement: { lower: -1, upper: 1 }, // Provide a sensible default
                };
                 // If the joint was prismatic, its old limit is used for displacement
                if (joint.type === 'prismatic') {
                    joint.limits.displacement = { ...defaultLimit };
                }
                delete joint.limit; // Remove the old property
            }
        }
        // --- End Migration Logic ---

        setState({
            links: robotData.links,
            joints: robotData.joints,
            baseLinkId: robotData.baseLinkId,
            selectedItem: { id: null, type: null },
        });
    };

    // Handle legacy .json files
    if (file.name.toLowerCase().endsWith('.json')) {
        try {
            const jsonString = await file.text();
            const robotData = JSON.parse(jsonString);
            if (!robotData.links || !robotData.joints || !robotData.baseLinkId) {
                throw new Error('Invalid robot data format in JSON file.');
            }
            processAndSetState(robotData);
        } catch (error) {
            console.error("Failed to load and parse legacy JSON file:", error);
            alert(`Failed to load JSON project. Error: ${error.message}`);
        }
        return;
    }

    // Handle new .zip archives
    if (file.name.toLowerCase().endsWith('.zip')) {
        try {
            const zip = await JSZip.loadAsync(file);
            const sceneFile = zip.file('robot-scene.json');
            if (!sceneFile) {
                throw new Error("'robot-scene.json' not found in the zip archive.");
            }
            const jsonString = await sceneFile.async('string');
            const robotData = JSON.parse(jsonString);
            if (!robotData.links || !robotData.joints || !robotData.baseLinkId) {
                throw new Error('Invalid robot data format in robot-scene.json.');
            }

            const meshFolder = zip.folder('meshes');
            if (meshFolder) {
                for (const link of Object.values(robotData.links as Record<string, RobotLink>)) {
                    if (link.visual.type === 'mesh' && link.visual.meshUrl) {
                        const meshFile = meshFolder.file(link.visual.meshUrl.replace('meshes/', ''));
                        if (meshFile) {
                            const blob = await meshFile.async('blob');
                            link.visual.meshUrl = URL.createObjectURL(blob);
                        } else {
                            console.warn(`Mesh file not found in zip: ${link.visual.meshUrl}`);
                            link.visual.meshUrl = null;
                        }
                    }
                }
            }
            processAndSetState(robotData);
        } catch (error) {
            console.error("Failed to load and parse robot data:", error);
            alert(`Failed to load robot project. Please check the console for details. Error: ${error.message}`);
        }
        return;
    }

    alert("Invalid file type. Please upload a '.zip' or '.json' file created by RobotLinkForge.");
  },

  exportURDF: async () => {
    try {
        const { links, joints, baseLinkId } = getState();

        // 1. Create a deep copy of the state to avoid mutations
        const robotData = JSON.parse(JSON.stringify({ links, joints, baseLinkId }));

        // 2. Fetch all necessary mesh files and convert them to a format the backend can handle (e.g., base64)
        const meshDataPromises = Object.values(robotData.links as Record<string, RobotLink>).map(async (link) => {
            if (link.visual.type === 'mesh' && link.visual.meshUrl && link.visual.meshUrl.startsWith('http')) {
                try {
                    const response = await fetch(link.visual.meshUrl);
                    if (!response.ok) throw new Error(`Failed to fetch ${link.visual.meshUrl}`);
                    const blob = await response.blob();
                    
                    // We need to get the original filename. Let's assume the URL's last part is the name.
                    const urlParts = link.visual.meshUrl.split('/');
                    const filename = urlParts[urlParts.length - 1];

                    return {
                        linkId: link.id,
                        filename: filename, // e.g., 'nozzle.stl'
                        blob: blob,
                    };
                } catch (e) {
                    console.error(`Error fetching mesh for ${link.name}:`, e);
                    return null; // Ignore failed fetches
                }
            }
             else if (link.visual.type === 'mesh' && link.visual.meshUrl && link.visual.meshUrl.startsWith('blob:')) {
                try {
                    const response = await fetch(link.visual.meshUrl);
                    if (!response.ok) throw new Error(`Failed to fetch ${link.visual.meshUrl}`);
                    const blob = await response.blob();
                    const filename = `${link.id}.stl`; // Create a unique name
                    return {
                        linkId: link.id,
                        filename: filename,
                        blob: blob
                    };
                } catch (e) {
                    console.error(`Error fetching mesh for ${link.name}:`, e);
                    return null;
                }
            }
            return null;
        });

        const meshDatas = (await Promise.all(meshDataPromises)).filter(m => m !== null);
        
        // Use FormData to send both JSON and files
        const formData = new FormData();
        formData.append('robot_data', JSON.stringify(robotData));
        
        // Append each mesh blob
        for (const meshData of meshDatas) {
             if (meshData) {
                // The backend will receive the file with the name `mesh_<linkId>`.
                formData.append(`mesh_${meshData.linkId}`, meshData.blob, meshData.filename);
            }
        }

        // 3. Send to the backend
        const response = await fetch('http://localhost:8000/api/export-urdf', {
            method: 'POST',
            body: formData, // FormData sets the correct 'multipart/form-data' header
        });

        if (!response.ok) {
            const errorText = await response.text();
            throw new Error(`Backend export failed: ${response.status} ${errorText}`);
        }

        // 4. Receive the zip file and trigger download
        const zipBlob = await response.blob();
        const robotName = robotData.links[robotData.baseLinkId]?.name.replace(/[^a-zA-Z0-9]/g, '_') || 'my_robot';

        if ('showSaveFilePicker' in window) {
             const handle = await window.showSaveFilePicker({
                suggestedName: `${robotName}_ros_package.zip`,
                types: [{ description: 'ROS Package (Zip)', accept: { 'application/zip': ['.zip'] }}],
            });
            const writable = await handle.createWritable();
            await writable.write(zipBlob);
            await writable.close();
        } else {
            const url = window.URL.createObjectURL(zipBlob);
            const a = document.createElement('a');
            a.style.display = 'none';
            a.href = url;
            a.download = `${robotName}_ros_package.zip`;
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
            document.body.removeChild(a);
        }
        
        alert('URDF package exported successfully!');

    } catch (err) {
        console.error('Error exporting URDF:', err);
        alert(`Failed to export URDF package. Check the console for details. Error: ${err.message}`);
    }
  },
}));