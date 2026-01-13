import { create } from 'zustand';
import { v4 as uuidv4 } from 'uuid';
import * as THREE from 'three';
import { RobotState, RobotActions, RobotLink, RobotJoint, JointType, RecordingMode, MotionKeyframe, MotionRecording, PlaybackState, JointValues } from './types';
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
        highlightedItem: { id: null, type: null },
        selectionCandidates: [],
        cameraMode: 'rotate', // Default camera mode
        cameraControls: null,
        serverProjects: [],
        importUnit: 'm', // Default to Meters
        collisionMode: 'off', // Default collision off
        collisionBoxScale: 0.8,
        // Motion Recording State
        recordingMode: null,
        isRecording: false,
        recordingStartTime: null,
        currentRecording: null,
        recordings: [],
        playbackState: { isPlaying: false, currentTime: 0, recordingId: null },
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
    visual: {
        type: 'none',
        dimensions: [0.05, 0.05, 0.05], // Default small box if enabled
        color: '#888888',
    },
    origin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
});

// This now ALWAYS creates a visible cylinder link.
const createDefaultLink = (): RobotLink => ({
    id: `link-${uuidv4()}`,
    name: 'new_link',
    visual: {
        type: 'none',
        dimensions: [0.05, 0.05, 0.05],
        color: '#888888',
    },
    childJoints: [],
});

// Helper for Zip Generation
const createRobotZip = async (state: RobotState) => {
    const { links, joints, baseLinkId } = state;
    const zip = new JSZip();

    // Create a deep copy to modify URLs without affecting current state
    const linksToSave = JSON.parse(JSON.stringify(links));
    const jointsToSave = JSON.parse(JSON.stringify(joints));

    const meshFolder = zip.folder('meshes');
    if (!meshFolder) throw new Error("Failed to create 'meshes' folder in zip.");

    const meshUrlMap = new Map<string, string>();

    const processVisual = async (item: any, name: string) => {
        if (item.visual.type === 'mesh' && item.visual.meshUrl && (item.visual.meshUrl.startsWith('http') || item.visual.meshUrl.startsWith('blob:'))) {
            // Check if we've already processed this URL
            if (meshUrlMap.has(item.visual.meshUrl)) {
                item.visual.meshUrl = meshUrlMap.get(item.visual.meshUrl);
                return;
            }

            try {
                const response = await fetch(item.visual.meshUrl);
                if (!response.ok) {
                    console.warn(`Could not fetch mesh for ${name}: ${item.visual.meshUrl}`);
                    return;
                }
                const blob = await response.blob();
                const newMeshName = `${uuidv4()}.stl`;
                const relativePath = `meshes/${newMeshName}`;

                meshFolder.file(newMeshName, blob);

                // Update the URL in our data-to-be-saved
                meshUrlMap.set(item.visual.meshUrl, relativePath); // Store original for mapping
                item.visual.meshUrl = relativePath;
            } catch (e) {
                console.warn(`Error fetching mesh during save: ${e}`);
            }
        }
    };

    for (const link of Object.values(linksToSave as Record<string, RobotLink>)) {
        await processVisual(link, link.name);
    }

    for (const joint of Object.values(jointsToSave as Record<string, RobotJoint>)) {
        await processVisual(joint, joint.name);
    }

    const robotData = { links: linksToSave, joints: jointsToSave, baseLinkId };
    zip.file('robot-scene.json', JSON.stringify(robotData, null, 2));

    return await zip.generateAsync({ type: 'blob' });
};

// Define API Base URL dynamically to support remote access
// Use relative path for API calls to support remote access via Vite Proxy
const API_BASE_URL = '';

export const useRobotStore = create<RobotState & RobotActions>((setState, getState) => ({
    ...createInitialState(),
    ...createInitialState(),
    selectionCandidates: [],
    cameraControls: null,

    uploadAndSetMesh: async (itemId, itemType, file) => {
        const { joints, links } = getState();

        // Previous logic forced joints to apply to their child link.
        // We now support applying directly to the joint (e.g. for motor housing).

        let targetId = itemId;
        let targetType = itemType;

        // Validation
        if (targetType === 'link' && !links[targetId]) {
            console.error("Cannot add mesh: Target link not found.");
            return;
        }
        if (targetType === 'joint' && !joints[targetId]) {
            console.error("Cannot add mesh: Target joint not found.");
            return;
        }

        const formData = new FormData();
        formData.append('file', file);

        try {
            const response = await fetch(`${API_BASE_URL}/api/upload-stl`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                throw new Error(`Upload failed with status: ${response.status}`);
            }

            const result = await response.json();
            // Ensure the returned URL is also relative or uses the dynamic host
            // The backend returns e.g. /static/meshes/foo.stl or similar? 
            // Currently backend returns {"url": "/path/to/file"} -> we prepend base url.
            const meshUrl = `${API_BASE_URL}${result.url}`;

            setState(state => {
                // Determine scale based on global setting
                // If setting is 'm' -> 1.0 (already meters)
                // If setting is 'cm' -> 0.01 (cm to m)
                // If setting is 'mm' -> 0.001 (mm to m)
                let scaleFactor = 1.0;
                if (state.importUnit === 'cm') scaleFactor = 0.01;
                if (state.importUnit === 'mm') scaleFactor = 0.001;

                const initialScale: [number, number, number] = [scaleFactor, scaleFactor, scaleFactor];

                if (targetType === 'link') {
                    const linkToUpdate = state.links[targetId];
                    if (!linkToUpdate) return {};

                    const updatedLink = updateDeep(linkToUpdate, 'visual', {
                        ...linkToUpdate.visual,
                        type: 'mesh',
                        meshUrl: meshUrl,
                        meshScale: initialScale,
                        meshOrigin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                        meshBoundingBox: undefined,
                        color: '#F0F0F0', // Default light gray for new meshes
                    });
                    return { links: { ...state.links, [targetId]: updatedLink } };
                } else {
                    // Joint
                    const jointToUpdate = state.joints[targetId];
                    if (!jointToUpdate) return {};

                    const updatedJoint = updateDeep(jointToUpdate, 'visual', {
                        ...jointToUpdate.visual,
                        type: 'mesh',
                        meshUrl: meshUrl,
                        meshScale: initialScale,
                        meshOrigin: { xyz: [0, 0, 0], rpy: [0, 0, 0] },
                        color: '#F0F0F0', // Default light gray for new meshes
                        // Joints don't usually need bounding box for length fitting like links do, but we can store it if we want.
                        // The Visual type has it optional.
                    });
                    return { joints: { ...state.joints, [targetId]: updatedJoint } };
                }
            });

        } catch (error: any) {
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
            let targetId = itemId;
            let targetType = itemType;

            if (targetType === 'link') {
                if (!state.links[targetId]) return {};
                let updatedLink = state.links[targetId];

                if (transform.scale) {
                    updatedLink = updateDeep(updatedLink, 'visual.meshScale', transform.scale);
                }
                if (transform.origin?.xyz) {
                    updatedLink = updateDeep(updatedLink, 'visual.meshOrigin.xyz', transform.origin.xyz);
                }
                if (transform.origin?.rpy) {
                    updatedLink = updateDeep(updatedLink, 'visual.meshOrigin.rpy', transform.origin.rpy);
                }
                return { links: { ...state.links, [targetId]: updatedLink } };

            } else {
                // Joint
                if (!state.joints[targetId]) return {};
                let updatedJoint = state.joints[targetId];

                if (transform.scale) {
                    updatedJoint = updateDeep(updatedJoint, 'visual.meshScale', transform.scale);
                }
                if (transform.origin?.xyz) {
                    updatedJoint = updateDeep(updatedJoint, 'visual.meshOrigin.xyz', transform.origin.xyz);
                }
                if (transform.origin?.rpy) {
                    updatedJoint = updateDeep(updatedJoint, 'visual.meshOrigin.rpy', transform.origin.rpy);
                }
                return { joints: { ...state.joints, [targetId]: updatedJoint } };
            }
        });
    },

    setCameraControls: (controls) => setState({ cameraControls: controls }),


    zoomIn: () => {
        getState().cameraControls?.dolly(-0.5, true);
    },

    zoomOut: () => {
        getState().cameraControls?.dolly(0.5, true);
    },

    selectItem: (id, type) => setState({ selectedItem: { id, type }, selectionCandidates: [] }),
    setHighlightedItem: (id, type) => setState({ highlightedItem: { id, type } }),
    setSelectionCandidates: (ids) => setState({ selectionCandidates: ids }),
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

            // Apply the update
            const nextJoints = { ...state.joints, [id]: updatedJoint };

            // --- Passive Joint Recalculation ---
            // We iterate ONCE over all joints to find passive ones and evaluate their equations.
            // This assumes no deeply nested chains of passive joints for now (A -> B -> C).
            // Even if we did, doing 1 pass is safe against infinite loops.

            // Context builder: Create a map of "JointName" -> { roll, pitch, yaw, displacement }
            const context: Record<string, any> = {};
            Object.values(nextJoints).forEach(j => {
                // Sanitize name for JS variable usage? simpler to just use it as key if user follows naming rules
                // But user might name joint "Arm 1".
                // We will access via `context['Arm 1'].roll` if we use a safe evaluator,
                // OR we strictly bind them.
                // For MVP, we pass the Whole JOINT object or just values.
                // Let's pass values.
                context[j.name] = { ...j.currentValues };
            });

            const evaluateEquation = (eq: string, ctx: any): number | null => {
                try {
                    // Create a function that takes the context keys as arguments and returns the result
                    const keys = Object.keys(ctx);
                    const values = Object.values(ctx);
                    // Safe-ish eval: use Function constructor.
                    // "return " + eq
                    // But we want to access variables directly like "Joint1.roll".
                    // So we wrap the execution.
                    // Note: This relies on user knowing variable names.
                    const func = new Function(...keys, `return ${eq};`);
                    const result = func(...values);
                    return typeof result === 'number' && !isNaN(result) ? result : null;
                } catch (e) {
                    // console.warn("Equation error:", e); // Squelch during typing
                    return null;
                }
            };

            Object.values(nextJoints).forEach(j => {
                if (j.isPassive && j.equation && j.id !== id) { // Don't self-update if we are editing the passive joint directly (though we block UI)
                    const result = evaluateEquation(j.equation, context);
                    if (result !== null) {
                        // Apply result to the main degree of freedom
                        let newVals = { ...j.currentValues };

                        if (j.type === 'prismatic') {
                            // Enforce limits? Maybe. Let's clamp.
                            const clamped = Math.min(Math.max(result, j.limits.displacement.lower), j.limits.displacement.upper);
                            newVals.displacement = clamped;
                        } else if (j.type === 'rotational') {
                            // Apply to available DOF. If multiple, assume Z (Yaw) or main?
                            // Standard for passive is usually 1 DOF.
                            // Let's try to infer or apply to all enabled? No, formula usually returns scalar.
                            // Let's apply to the first enabled DOF in order R P Y.
                            if (j.dof.roll) newVals.roll = Math.min(Math.max(result, j.limits.roll.lower), j.limits.roll.upper);
                            if (j.dof.pitch) newVals.pitch = Math.min(Math.max(result, j.limits.pitch.lower), j.limits.pitch.upper);
                            if (j.dof.yaw) newVals.yaw = Math.min(Math.max(result, j.limits.yaw.lower), j.limits.yaw.upper);
                        }
                        nextJoints[j.id] = { ...j, currentValues: newVals };
                    }
                }
            });

            return { joints: nextJoints };
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
    // --- Save/Load Functionality ---
    saveRobot: async () => {
        try {
            const zipBlob = await createRobotZip(getState());

            if ('showSaveFilePicker' in window) {
                const handle = await window.showSaveFilePicker({
                    suggestedName: 'robot-model.zip',
                    types: [{ description: 'RobotLinkForge Zip Archive', accept: { 'application/zip': ['.zip'] } }],
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

        } catch (err: any) {
            console.error('Error saving robot project:', err);
            alert(`Failed to save project: ${err.message}`);
        }
    },

    saveProjectToServer: async (name: string) => {
        const { serverProjects } = getState();
        const expectedFilename = `${name}.zip`;

        // Confirmation if exists
        if (serverProjects.includes(expectedFilename)) {
            if (!window.confirm(`A project named "${name}" already exists on the server. Do you want to overwrite it?`)) {
                return;
            }
        }

        try {
            const zipBlob = await createRobotZip(getState());
            const formData = new FormData();
            formData.append('file', zipBlob, `${name}.zip`);
            formData.append('project_name', name);

            const response = await fetch(`${API_BASE_URL}/api/projects`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                throw new Error(`Server returned ${response.status}`);
            }

            const result = await response.json();
            console.log('Project saved to server:', result);
            alert('Project saved to server successfully!');
            // Refresh list
            getState().getProjectList();

        } catch (err: any) {
            console.error('Error saving robot project to server:', err);
            alert(`Failed to save project to server: ${err.message}`);
        }
    },

    getProjectList: async () => {
        try {
            const response = await fetch(`${API_BASE_URL}/api/projects`);
            if (response.ok) {
                const data = await response.json();
                setState({ serverProjects: data.projects || [] });
            }
        } catch (error) {
            console.error("Failed to fetch project list:", error);
        }
    },

    loadProjectFromServer: async (filename: string) => {
        try {
            // Encode filename to handle spaces or special characters safely
            const encodedFilename = encodeURIComponent(filename);
            const response = await fetch(`${API_BASE_URL}/api/projects/${encodedFilename}`);
            if (!response.ok) {
                throw new Error(`Failed to fetch project: ${response.status} ${response.statusText}`);
            }
            const blob = await response.blob();

            // Create a File object. Note: filename here should be the original readable one for display logic
            // explicitly set lastModified to now to simulate fresh file
            const file = new File([blob], filename, {
                type: 'application/zip',
                lastModified: new Date().getTime()
            });

            await getState().loadRobot(file);
        } catch (error: any) {
            console.error("Failed to load project from server:", error);
            alert(`Failed to load project from server: ${error.message}`);
        }
    },

    setImportUnit: (unit) => setState({ importUnit: unit }),
    setCollisionMode: (mode) => setState({ collisionMode: mode }),
    setCollisionBoxScale: (scale) => setState({ collisionBoxScale: scale }),

    deleteProjectFromServer: async (filename: string) => {
        try {
            const encodedFilename = encodeURIComponent(filename);
            const response = await fetch(`${API_BASE_URL}/api/projects/${encodedFilename}`, {
                method: 'DELETE',
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(errorText);
            }

            // Refresh list
            await getState().getProjectList();
            alert(`Project "${filename}" deleted (soft delete).`);
        } catch (error: any) {
            console.error("Failed to delete project:", error);
            alert(`Failed to delete project: ${error.message}`);
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

                // Ensure visual property exists (Migration for new feature)
                if (!joint.visual) {
                    joint.visual = {
                        type: 'none',
                        dimensions: [0.05, 0.05, 0.05],
                        color: '#888888',
                    };
                }
            }
            // --- End Migration Logic ---

            setState({
                links: robotData.links,
                joints: robotData.joints,
                baseLinkId: robotData.baseLinkId,
                selectedItem: { id: null, type: null },
                highlightedItem: { id: null, type: null },
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
                    const resolveMeshUrl = async (item: any) => {
                        // Defensive check: Ensure visual exists before checking properties
                        if (item.visual && item.visual.type === 'mesh' && item.visual.meshUrl) {
                            const meshFile = meshFolder.file(item.visual.meshUrl.replace('meshes/', ''));
                            if (meshFile) {
                                const blob = await meshFile.async('blob');
                                item.visual.meshUrl = URL.createObjectURL(blob);
                            } else {
                                // Fallback for bad legacy saves or cross-origin issues
                                if (item.visual.meshUrl.includes('localhost:8000')) {
                                    console.warn(`Patching localhost URL for ${item.name}: ${item.visual.meshUrl}`);
                                    item.visual.meshUrl = item.visual.meshUrl.replace(/https?:\/\/localhost:8000/, API_BASE_URL);
                                } else {
                                    console.warn(`Mesh file not found in zip: ${item.visual.meshUrl}`);
                                    // Don't null it, leave it so user sees it broken but URL is preserved
                                }
                            }
                        }
                    };

                    for (const link of Object.values(robotData.links as Record<string, RobotLink>)) {
                        await resolveMeshUrl(link);
                    }
                    for (const joint of Object.values(robotData.joints as Record<string, RobotJoint>)) {
                        await resolveMeshUrl(joint);
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

    exportURDF: async (robotName: string) => {
        try {
            const { links, joints, baseLinkId } = getState();

            // 1. Create a deep copy of the state to avoid mutations
            const robotData = JSON.parse(JSON.stringify({ links, joints, baseLinkId }));

            // 2. Fetch all necessary mesh files and convert them to a format the backend can handle
            const meshDataPromises = Object.values(robotData.links as Record<string, RobotLink>).map(async (link) => {
                if (link.visual.type === 'mesh' && link.visual.meshUrl && (link.visual.meshUrl.startsWith('http') || link.visual.meshUrl.startsWith('blob:') || link.visual.meshUrl.startsWith('/'))) {
                    try {
                        const response = await fetch(link.visual.meshUrl);
                        if (!response.ok) throw new Error(`Failed to fetch ${link.visual.meshUrl}`);
                        const blob = await response.blob();

                        const urlParts = link.visual.meshUrl.split('/');
                        const filename = urlParts[urlParts.length - 1] || `${link.id}.stl`;

                        return {
                            linkId: link.id,
                            filename: filename,
                            blob: blob,
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
            formData.append('robot_name', robotName); // Add the user-defined robot name

            // Append each mesh blob
            for (const meshData of meshDatas) {
                if (meshData) {
                    formData.append(`files`, meshData.blob, `mesh_${meshData.linkId}`);
                }
            }

            // 3. Send to the backend
            const response = await fetch(`${API_BASE_URL}/api/export-urdf`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Backend export failed: ${response.status} ${errorText}`);
            }

            // 4. Receive the zip file and trigger download
            const zipBlob = await response.blob();
            const safeRobotName = robotName.replace(/[^a-zA-Z0-9]/g, '_');

            if ('showSaveFilePicker' in window) {
                const handle = await window.showSaveFilePicker({
                    suggestedName: `${safeRobotName}_ros_package.zip`,
                    types: [{ description: 'ROS Package (Zip)', accept: { 'application/zip': ['.zip'] } }],
                });
                const writable = await handle.createWritable();
                await writable.write(zipBlob);
                await writable.close();
            } else {
                const url = window.URL.createObjectURL(zipBlob);
                const a = document.createElement('a');
                a.style.display = 'none';
                a.href = url;
                a.download = `${safeRobotName}_ros_package.zip`;
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

    exportURDF_ROS2: async (robotName: string) => {
        try {
            const { links, joints, baseLinkId } = getState();

            const robotData = JSON.parse(JSON.stringify({ links, joints, baseLinkId }));

            const linkMeshDataPromises = Object.values(robotData.links as Record<string, RobotLink>).map(async (link) => {
                if (link.visual.type === 'mesh' && link.visual.meshUrl && (link.visual.meshUrl.startsWith('http') || link.visual.meshUrl.startsWith('blob:') || link.visual.meshUrl.startsWith('/'))) {
                    try {
                        const response = await fetch(link.visual.meshUrl);
                        if (!response.ok) throw new Error(`Failed to fetch ${link.visual.meshUrl}`);
                        const blob = await response.blob();

                        const urlParts = link.visual.meshUrl.split('/');
                        const filename = urlParts[urlParts.length - 1] || `${link.id}.stl`;

                        return {
                            id: link.id,
                            filename: filename,
                            blob: blob,
                        };
                    } catch (e) {
                        console.error(`Error fetching mesh for ${link.name}:`, e);
                        return null;
                    }
                }
                return null;
            });

            const jointMeshDataPromises = Object.values(robotData.joints as Record<string, RobotJoint>).map(async (joint) => {
                if (joint.visual.type === 'mesh' && joint.visual.meshUrl && (joint.visual.meshUrl.startsWith('http') || joint.visual.meshUrl.startsWith('blob:') || joint.visual.meshUrl.startsWith('/'))) {
                    try {
                        const response = await fetch(joint.visual.meshUrl);
                        if (!response.ok) throw new Error(`Failed to fetch ${joint.visual.meshUrl}`);
                        const blob = await response.blob();

                        const urlParts = joint.visual.meshUrl.split('/');
                        const filename = urlParts[urlParts.length - 1] || `${joint.id}.stl`;

                        return {
                            id: joint.id,
                            filename: filename,
                            blob: blob,
                        };
                    } catch (e) {
                        console.error(`Error fetching mesh for joint ${joint.name}:`, e);
                        return null;
                    }
                }
                return null;
            });

            const meshDatas = (await Promise.all([...linkMeshDataPromises, ...jointMeshDataPromises])).filter(m => m !== null);

            const formData = new FormData();
            formData.append('robot_data', JSON.stringify(robotData));
            const state = getState();
            if (state.recordings && state.recordings.length > 0) {
                formData.append('recordings', JSON.stringify(state.recordings));
            }
            formData.append('robot_name', robotName);

            for (const meshData of meshDatas) {
                if (meshData) {
                    formData.append(`files`, meshData.blob, `mesh_${meshData.id}`);
                }
            }

            const response = await fetch(`${API_BASE_URL}/api/export-urdf-ros2`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Backend export failed: ${response.status} ${errorText}`);
            }

            const zipBlob = await response.blob();
            const safeRobotName = robotName.replace(/[^a-zA-Z0-9]/g, '_');

            console.log("URDF Export: Received Blob size:", zipBlob.size);

            if (zipBlob.size === 0) {
                alert("Error: Received empty file from server.");
                return;
            }

            // safeRobotName is already declared above
            const fileName = `${safeRobotName}_ros2_package.zip`;

            console.log("URDF Export: Triggering download for", fileName);

            // Legacy Download Logic (Safest for all browsers/contexts)
            const url = window.URL.createObjectURL(zipBlob);
            const a = document.createElement('a');
            a.style.display = 'none';
            a.href = url;
            a.download = fileName;
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
            document.body.removeChild(a);

            console.log("URDF Export: Download click triggered.");
            alert(`Export Success! File size: ${zipBlob.size} bytes. check your browser downloads.`);

            alert('URDF ROS2 package exported successfully!');

        } catch (err) {
            console.error('Error exporting URDF for ROS2:', err);
            alert(`Failed to export URDF ROS2 package. Check the console for details. Error: ${err.message}`);
        }
    },

    exportMujocoURDF: async (robotName: string) => {
        try {
            const { links, joints, baseLinkId } = getState();
            const robotData = JSON.parse(JSON.stringify({ links, joints, baseLinkId }));

            // Helper to collect meshes
            const collectMeshes = async (items: any[]) => {
                const promises = items.map(async (item) => {
                    if (item.visual.type === 'mesh' && item.visual.meshUrl && (item.visual.meshUrl.startsWith('http') || item.visual.meshUrl.startsWith('blob:') || item.visual.meshUrl.startsWith('/'))) {
                        try {
                            const response = await fetch(item.visual.meshUrl);
                            if (!response.ok) throw new Error(`Failed to fetch ${item.visual.meshUrl}`);
                            const blob = await response.blob();
                            const urlParts = item.visual.meshUrl.split('/');
                            const filename = urlParts[urlParts.length - 1] || `${item.id}.stl`;
                            return { id: item.id, filename: filename, blob: blob };
                        } catch (e) {
                            console.error(`Error fetching mesh for ${item.name}:`, e);
                            return null;
                        }
                    }
                    return null;
                });
                return (await Promise.all(promises)).filter(m => m !== null);
            };

            const linkMeshes = await collectMeshes(Object.values(robotData.links));
            const jointMeshes = await collectMeshes(Object.values(robotData.joints));
            const meshDatas = [...linkMeshes, ...jointMeshes];

            const formData = new FormData();
            formData.append('robot_data', JSON.stringify(robotData));
            const state = getState();
            if (state.recordings && state.recordings.length > 0) {
                formData.append('recordings', JSON.stringify(state.recordings));
            }
            formData.append('robot_name', robotName);

            for (const meshData of meshDatas) {
                if (meshData) {
                    formData.append(`files`, meshData.blob, `mesh_${meshData.id}`);
                }
            }

            const response = await fetch(`${API_BASE_URL}/api/export-mujoco-urdf`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Backend export failed: ${response.status} ${errorText}`);
            }

            const zipBlob = await response.blob();
            const safeRobotName = robotName.replace(/[^a-zA-Z0-9]/g, '_');
            const fileName = `${safeRobotName}_mujoco_urdf.zip`;

            // Download
            const url = window.URL.createObjectURL(zipBlob);
            const a = document.createElement('a');
            a.style.display = 'none';
            a.href = url;
            a.download = fileName;
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
            document.body.removeChild(a);

            alert('MuJoCo (URDF) package exported successfully!');

        } catch (err: any) {
            console.error('Error exporting MuJoCo URDF:', err);
            alert(`Failed to export MuJoCo package: ${err.message}`);
        }
    },

    exportMujocoMJCF: async (robotName: string, useMeshCollision: boolean = false) => {
        try {
            const { links, joints, baseLinkId } = getState();
            const robotData = JSON.parse(JSON.stringify({ links, joints, baseLinkId }));

            // Helper to collect meshes (Same logic)
            const collectMeshes = async (items: any[]) => {
                const promises = items.map(async (item) => {
                    if (item.visual.type === 'mesh' && item.visual.meshUrl && (item.visual.meshUrl.startsWith('http') || item.visual.meshUrl.startsWith('blob:') || item.visual.meshUrl.startsWith('/'))) {
                        try {
                            const response = await fetch(item.visual.meshUrl);
                            if (!response.ok) throw new Error(`Failed to fetch ${item.visual.meshUrl}`);
                            const blob = await response.blob();
                            const urlParts = item.visual.meshUrl.split('/');
                            const filename = urlParts[urlParts.length - 1] || `${item.id}.stl`;
                            return { id: item.id, filename: filename, blob: blob };
                        } catch (e) {
                            console.error(`Error fetching mesh for ${item.name}:`, e);
                            return null;
                        }
                    }
                    return null;
                });
                return (await Promise.all(promises)).filter(m => m !== null);
            };

            const linkMeshes = await collectMeshes(Object.values(robotData.links));
            const jointMeshes = await collectMeshes(Object.values(robotData.joints));
            const meshDatas = [...linkMeshes, ...jointMeshes];

            const formData = new FormData();
            formData.append('robot_data', JSON.stringify(robotData));
            formData.append('robot_name', robotName);
            formData.append('mesh_collision', useMeshCollision.toString());

            for (const meshData of meshDatas) {
                if (meshData) {
                    formData.append(`files`, meshData.blob, `mesh_${meshData.id}`);
                }
            }

            const response = await fetch(`${API_BASE_URL}/api/export-mujoco-mjcf`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Backend export failed: ${response.status} ${errorText}`);
            }

            const zipBlob = await response.blob();
            const safeRobotName = robotName.replace(/[^a-zA-Z0-9]/g, '_');
            const fileName = `${safeRobotName}_mujoco_mjcf.zip`;

            // Download
            const url = window.URL.createObjectURL(zipBlob);
            const a = document.createElement('a');
            a.style.display = 'none';
            a.href = url;
            a.download = fileName;
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
            document.body.removeChild(a);

            alert('MuJoCo (MJCF) package exported successfully!');

        } catch (err: any) {
            console.error('Error exporting MuJoCo MJCF:', err);
            alert(`Failed to export MuJoCo package: ${err.message}`);
        }
    },

    exportGazebo: async (robotName: string) => {
        try {
            const { links, joints, baseLinkId } = getState();
            const robotData = JSON.parse(JSON.stringify({ links, joints, baseLinkId }));

            // Helper to collect meshes
            const collectMeshes = async (items: any[]) => {
                const promises = items.map(async (item) => {
                    if (item.visual.type === 'mesh' && item.visual.meshUrl && (item.visual.meshUrl.startsWith('http') || item.visual.meshUrl.startsWith('blob:') || item.visual.meshUrl.startsWith('/'))) {
                        try {
                            const response = await fetch(item.visual.meshUrl);
                            if (!response.ok) throw new Error(`Failed to fetch ${item.visual.meshUrl}`);
                            const blob = await response.blob();
                            const urlParts = item.visual.meshUrl.split('/');
                            const filename = urlParts[urlParts.length - 1] || `${item.id}.stl`;
                            return { id: item.id, filename: filename, blob: blob };
                        } catch (e) {
                            console.error(`Error fetching mesh for ${item.name}:`, e);
                            return null;
                        }
                    }
                    return null;
                });
                return (await Promise.all(promises)).filter(m => m !== null);
            };

            const linkMeshes = await collectMeshes(Object.values(robotData.links));
            const jointMeshes = await collectMeshes(Object.values(robotData.joints));
            const meshDatas = [...linkMeshes, ...jointMeshes];

            const formData = new FormData();
            formData.append('robot_data', JSON.stringify(robotData));
            formData.append('robot_name', robotName);

            for (const meshData of meshDatas) {
                if (meshData) {
                    formData.append(`files`, meshData.blob, `mesh_${meshData.id}`);
                }
            }

            const response = await fetch(`${API_BASE_URL}/api/export-gazebo`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Backend export failed: ${response.status} ${errorText}`);
            }

            const zipBlob = await response.blob();
            const safeRobotName = robotName.replace(/[^a-zA-Z0-9]/g, '_');
            const fileName = `${safeRobotName}_gazebo.zip`;

            // Download
            const url = window.URL.createObjectURL(zipBlob);
            const a = document.createElement('a');
            a.style.display = 'none';
            a.href = url;
            a.download = fileName;
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
            document.body.removeChild(a);

            alert('Gazebo package exported successfully!');

        } catch (err: any) {
            console.error('Error exporting Gazebo:', err);
            alert(`Failed to export Gazebo package: ${err.message}`);
        }
    },

    exportGazeboROS2: async (robotName: string) => {
        try {
            const { links, joints, baseLinkId } = getState();
            const robotData = JSON.parse(JSON.stringify({ links, joints, baseLinkId }));

            // Helper to collect meshes
            const collectMeshes = async (items: any[]) => {
                const promises = items.map(async (item) => {
                    if (item.visual.type === 'mesh' && item.visual.meshUrl && (item.visual.meshUrl.startsWith('http') || item.visual.meshUrl.startsWith('blob:') || item.visual.meshUrl.startsWith('/'))) {
                        try {
                            const response = await fetch(item.visual.meshUrl);
                            if (!response.ok) throw new Error(`Failed to fetch ${item.visual.meshUrl}`);
                            const blob = await response.blob();
                            const urlParts = item.visual.meshUrl.split('/');
                            const filename = urlParts[urlParts.length - 1] || `${item.id}.stl`;
                            return { id: item.id, filename: filename, blob: blob };
                        } catch (e) {
                            console.error(`Error fetching mesh for ${item.name}:`, e);
                            return null;
                        }
                    }
                    return null;
                });
                return (await Promise.all(promises)).filter(m => m !== null);
            };

            const linkMeshes = await collectMeshes(Object.values(robotData.links));
            const jointMeshes = await collectMeshes(Object.values(robotData.joints));
            const meshDatas = [...linkMeshes, ...jointMeshes];

            const formData = new FormData();
            formData.append('robot_data', JSON.stringify(robotData));
            formData.append('robot_name', robotName);

            for (const meshData of meshDatas) {
                if (meshData) {
                    formData.append(`files`, meshData.blob, `mesh_${meshData.id}`);
                }
            }

            const response = await fetch(`${API_BASE_URL}/api/export-gazebo-ros2`, {
                method: 'POST',
                body: formData,
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Backend export failed: ${response.status} ${errorText}`);
            }

            const zipBlob = await response.blob();
            const safeRobotName = robotName.replace(/[^a-zA-Z0-9]/g, '_');
            const fileName = `${safeRobotName}_gazebo_ros2.zip`;

            // Download
            const url = window.URL.createObjectURL(zipBlob);
            const a = document.createElement('a');
            a.style.display = 'none';
            a.href = url;
            a.download = fileName;
            document.body.appendChild(a);
            a.click();
            window.URL.revokeObjectURL(url);
            document.body.removeChild(a);

            alert('Gazebo (ROS 2) package exported successfully!');

        } catch (err: any) {
            console.error('Error exporting Gazebo ROS 2:', err);
            alert(`Failed to export Gazebo ROS 2 package: ${err.message}`);
        }
    },
    deleteItem: (id, type) => {
        setState(state => {
            // Prevent deleting the base link
            if (type === 'link' && id === state.baseLinkId) {
                alert("Cannot delete the base link.");
                return {};
            }

            const newLinks = { ...state.links };
            const newJoints = { ...state.joints };

            // Helper to recursively delete a joint and its children
            const deleteJointRecursive = (jointId: string) => {
                const joint = newJoints[jointId];
                if (!joint) return;

                // Delete the child link if it exists
                if (joint.childLinkId) {
                    deleteLinkRecursive(joint.childLinkId);
                }

                delete newJoints[jointId];
            };

            // Helper to recursively delete a link and its children
            const deleteLinkRecursive = (linkId: string) => {
                const link = newLinks[linkId];
                if (!link) return;

                // Delete all child joints
                for (const childJointId of link.childJoints) {
                    deleteJointRecursive(childJointId);
                }

                delete newLinks[linkId];
            };

            if (type === 'link') {
                const linkToDelete = newLinks[id];
                if (!linkToDelete) return {};

                // Find parent joint (if any) to remove reference
                // A link is a child of a joint. We need to find the joint that has this link as childLinkId.
                const parentJointEntry = Object.entries(newJoints).find(([_, j]) => j.childLinkId === id);

                if (parentJointEntry) {
                    const [parentJointId, parentJoint] = parentJointEntry;
                    // Deleting a link implies deleting the joint that connects to it, 
                    // OR just unsetting the childLinkId?
                    // Usually in a tree, if you delete a node, you delete the subtree.
                    // If I delete a link, the joint pointing to it becomes a "dead end".
                    // Let's delete the parent joint as well, because a joint without a child link is valid but 
                    // typically if user deletes "Link B", they want the joint connecting "Link A -> Link B" to go too,
                    // otherwise they'd just be left with a dangling joint.
                    // However, the user might want to attach a different link.
                    // Let's stick to the behavior: Delete Link -> Delete Parent Joint -> Remove Parent Joint from Parent Link.

                    // WAIT: If I delete the parent joint, I must remove it from the GrandParent Link.
                    const grandParentLinkId = parentJoint.parentLinkId;
                    const grandParentLink = newLinks[grandParentLinkId];

                    if (grandParentLink) {
                        // Remove parentJointId from grandParentLink.childJoints
                        newLinks[grandParentLinkId] = {
                            ...grandParentLink,
                            childJoints: grandParentLink.childJoints.filter(jid => jid !== parentJointId)
                        };
                    }

                    // Now delete the parent joint (which will recurse down if we hadn't started at the link)
                    // But since we are starting at the link, we can just call deleteJointRecursive on the parent joint,
                    // which will come back to delete this link.
                    // To avoid infinite loops or complexity, let's simpler:
                    // 1. Detach from GrandParent.
                    // 2. Delete Parent Joint.
                    // 3. Delete Target Link (and subtree).

                    delete newJoints[parentJointId];
                }

                deleteLinkRecursive(id);

            } else if (type === 'joint') {
                const jointToDelete = newJoints[id];
                if (!jointToDelete) return {};

                // Remove from Parent Link
                const parentLink = newLinks[jointToDelete.parentLinkId];
                if (parentLink) {
                    newLinks[jointToDelete.parentLinkId] = {
                        ...parentLink,
                        childJoints: parentLink.childJoints.filter(jid => jid !== id)
                    };
                }

                deleteJointRecursive(id);
            }

            return {
                links: newLinks,
                joints: newJoints,
                selectedItem: { id: null, type: null } // Deselect
            };
        });
    },

    resetProject: () => {
        if (confirm("Are you sure you want to create a new project? Unsaved changes will be lost.")) {
            setState({
                ...createInitialState(),
                cameraControls: getState().cameraControls // Keep camera controls
            });
        }
    },

    // --- Motion Recording Actions ---

    setRecordingMode: (mode) => {
        setState({ recordingMode: mode });
    },

    startRecording: (name = `Recording ${Date.now()}`) => {
        const state = getState();
        if (state.isRecording) return;

        const newRecording: MotionRecording = {
            id: uuidv4(),
            name,
            mode: state.recordingMode || 'slider',
            keyframes: [],
            duration: 0,
            createdAt: Date.now(),
        };

        setState({
            isRecording: true,
            recordingStartTime: Date.now(),
            currentRecording: newRecording,
        });
    },

    stopRecording: () => {
        const state = getState();
        if (!state.isRecording || !state.currentRecording) return;

        const finalRecording: MotionRecording = {
            ...state.currentRecording,
            duration: state.currentRecording.keyframes.length > 0
                ? Math.max(...state.currentRecording.keyframes.map(k => k.timestamp))
                : 0,
        };

        const existingIdx = state.recordings.findIndex(r => r.id === finalRecording.id);
        const newRecordings = [...state.recordings];
        if (existingIdx >= 0) {
            newRecordings[existingIdx] = finalRecording;
        } else {
            newRecordings.push(finalRecording);
        }

        setState({
            isRecording: false,
            recordingStartTime: null,
            currentRecording: null,
            recordings: newRecordings,
        });
    },

    captureKeyframe: () => {
        const state = getState();
        if (!state.isRecording || !state.currentRecording) return;

        // Capture current joint values
        const jointValues: Record<string, JointValues> = {};
        Object.entries(state.joints).forEach(([jointId, joint]) => {
            jointValues[jointId] = { ...joint.currentValues };
        });

        const timestamp = state.recordingStartTime
            ? Date.now() - state.recordingStartTime
            : 0;

        const newKeyframe: MotionKeyframe = {
            id: uuidv4(),
            timestamp,
            jointValues,
        };

        setState({
            currentRecording: {
                ...state.currentRecording,
                keyframes: [...state.currentRecording.keyframes, newKeyframe],
            },
        });
    },

    deleteKeyframe: (keyframeId) => {
        const state = getState();
        if (!state.currentRecording) return;

        setState({
            currentRecording: {
                ...state.currentRecording,
                keyframes: state.currentRecording.keyframes.filter(k => k.id !== keyframeId),
            },
        });
    },

    updateKeyframeTransition: (keyframeId, duration) => {
        const state = getState();
        if (!state.currentRecording) return; // Only update if recording/editing

        const kfIndex = state.currentRecording.keyframes.findIndex(k => k.id === keyframeId);
        if (kfIndex === -1) return;

        setState(state => {
            if (!state.currentRecording) return state;
            const newKeyframes = [...state.currentRecording.keyframes];
            newKeyframes[kfIndex] = {
                ...newKeyframes[kfIndex],
                transitionDuration: duration
            };
            return {
                currentRecording: {
                    ...state.currentRecording,
                    keyframes: newKeyframes
                }
            };
        });
    },

    updateKeyframeTiming: (keyframeId, newTimestamp) => {
        const state = getState();
        if (!state.currentRecording) return;

        const updatedKeyframes = state.currentRecording.keyframes.map(k =>
            k.id === keyframeId ? { ...k, timestamp: Math.max(0, newTimestamp) } : k
        ).sort((a, b) => a.timestamp - b.timestamp);

        setState({
            currentRecording: {
                ...state.currentRecording,
                keyframes: updatedKeyframes,
            },
        });
    },

    updateKeyframePose: (keyframeId) => {
        const state = getState();
        if (!state.currentRecording) return;

        // Capture current values
        const jointValues: Record<string, JointValues> = {};
        Object.entries(state.joints).forEach(([jointId, joint]) => {
            jointValues[jointId] = { ...joint.currentValues };
        });

        const updatedKeyframes = state.currentRecording.keyframes.map(k =>
            k.id === keyframeId ? { ...k, jointValues } : k
        );

        setState({
            currentRecording: {
                ...state.currentRecording,
                keyframes: updatedKeyframes,
            },
        });
    },

    cancelEditRecording: () => {
        setState({
            currentRecording: null,
            isRecording: false,
            recordingStartTime: null,
        });
    },

    loadKeyframePose: (keyframeId) => {
        const state = getState();
        const recording = state.currentRecording; // Can only edit current recording for now
        if (!recording) return;

        const keyframe = recording.keyframes.find(k => k.id === keyframeId);
        if (!keyframe) return;

        // Apply to joints
        const newJoints = { ...state.joints };
        Object.entries(keyframe.jointValues).forEach(([jointId, values]) => {
            if (newJoints[jointId]) {
                newJoints[jointId] = {
                    ...newJoints[jointId],
                    currentValues: { ...values }
                };
            }
        });

        setState({ joints: newJoints });
    },

    editRecording: (recordingId) => {
        const state = getState();
        const recording = state.recordings.find(r => r.id === recordingId);
        if (!recording) return;

        setState({
            currentRecording: JSON.parse(JSON.stringify(recording)), // Deep copy
            isRecording: false,
            recordingStartTime: null, // Not recording live
        });
    },

    saveRecording: () => {
        const state = getState();
        if (!state.currentRecording) return;

        const finalRecording = { ...state.currentRecording };
        // Recalculate duration if needed? For editing, we might keep duration or update based on max timestamp
        if (finalRecording.keyframes.length > 0) {
            const maxTime = Math.max(...finalRecording.keyframes.map(k => k.timestamp));
            finalRecording.duration = Math.max(finalRecording.duration, maxTime);
        }

        const existingIdx = state.recordings.findIndex(r => r.id === finalRecording.id);
        const newRecordings = [...state.recordings];
        if (existingIdx >= 0) {
            newRecordings[existingIdx] = finalRecording;
        } else {
            newRecordings.push(finalRecording);
        }

        setState({
            recordings: newRecordings,
            currentRecording: null, // Exit editing mode
        });
    },

    playRecording: (recordingId) => {
        const state = getState();
        const recording = recordingId
            ? state.recordings.find(r => r.id === recordingId)
            : state.currentRecording;

        if (!recording || recording.keyframes.length < 2) {
            console.warn("Need at least 2 keyframes to play");
            return;
        }

        setState({
            playbackState: {
                isPlaying: true,
                currentTime: 0,
                recordingId: recording.id,
            },
        });
    },

    pausePlayback: () => {
        setState(state => ({
            playbackState: {
                ...state.playbackState,
                isPlaying: false,
            },
        }));
    },

    stopPlayback: () => {
        setState({
            playbackState: {
                isPlaying: false,
                currentTime: 0,
                recordingId: null,
            },
        });
    },

    seekPlayback: (timeMs) => {
        setState(state => ({
            playbackState: {
                ...state.playbackState,
                currentTime: Math.max(0, timeMs),
            },
        }));
    },

    deleteRecording: (id) => {
        setState(state => ({
            recordings: state.recordings.filter(r => r.id !== id),
        }));
    },

    renameRecording: (id, newName) => {
        setState(state => ({
            recordings: state.recordings.map(r =>
                r.id === id ? { ...r, name: newName } : r
            ),
        }));
    },

    saveRecordingsLocal: () => {
        const state = getState();
        const json = JSON.stringify(state.recordings, null, 2);
        const blob = new Blob([json], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `robot_recordings_${new Date().toISOString().slice(0, 10)}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    },

    loadRecordingsLocal: async (file: File) => {
        try {
            const text = await file.text();
            const loadedRecordings = JSON.parse(text);
            if (!Array.isArray(loadedRecordings)) throw new Error('Invalid format');
            const state = getState();
            // Append or Replace? Let's Confirm. Or just Append for safety.
            // But merging by ID is better.
            const merged = [...state.recordings];
            loadedRecordings.forEach((r: any) => {
                if (!merged.find(existing => existing.id === r.id)) {
                    merged.push(r);
                } else {
                    // Update existing?
                    const idx = merged.findIndex(existing => existing.id === r.id);
                    merged[idx] = r;
                }
            });
            setState({ recordings: merged });
            alert(`Loaded ${loadedRecordings.length} recordings.`);
        } catch (e: any) {
            console.error(e);
            alert('Failed to load recordings: ' + e.message);
        }
    },

    fetchRecordingList: async () => {
        try {
            const res = await fetch(`${API_BASE_URL}/api/recordings`);
            if (!res.ok) throw new Error('Failed to fetch list');
            const data = await res.json();
            return data.files || [];
        } catch (e) {
            console.error(e);
            return [];
        }
    },

    saveRecordingsToServer: async (filename: string) => {
        const state = getState();
        try {
            const res = await fetch(`${API_BASE_URL}/api/recordings`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    filename,
                    recordings: state.recordings
                })
            });
            if (!res.ok) throw new Error('Failed to save');
            alert('Recordings saved to server!');
        } catch (e: any) {
            alert('Error saving to server: ' + e.message);
        }
    },

    loadRecordingsFromServer: async (filename: string) => {
        try {
            const res = await fetch(`${API_BASE_URL}/api/recordings/${filename}`);
            if (!res.ok) throw new Error('Failed to load');
            const data = await res.json();
            const newRecordings = data.recordings;
            if (!Array.isArray(newRecordings)) throw new Error('Invalid data');

            setState({ recordings: newRecordings }); // Replace state for server load? Or merge?
            // "Load Project" usually replaces. "Load Recordings" might replace too.
            // Let's stick to REPLACE for server load (consistency with project load)
            // or maybe confirm with user? Simpler to just replace for now as it's a "Load" action.
            alert('Recordings loaded from server!');
        } catch (e: any) {
            alert('Error loading from server: ' + e.message);
        }
    },

}));