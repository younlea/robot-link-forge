import React, { useState, useEffect, useRef } from 'react';
import { useRobotStore } from '../store';
import { RobotLink, RobotJoint, JointType } from '../types';
import { ToyBrick, PlusSquare, Link as LinkIcon, GitCommit, Move3d, Save, FolderOpen, UploadCloud, RotateCcw } from 'lucide-react';

// --- Reusable Input Components (with fixes) ---
const NumberInput = ({ label, value, onChange, step = 0.01 }: { label: string, value: number, onChange: (val: number) => void, step?: number }) => {
    const [strValue, setStrValue] = useState(value.toString());

    useEffect(() => {
        if (parseFloat(strValue) !== value) {
            setStrValue(value.toString());
        }
    }, [value]);

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const val = e.target.value;
        setStrValue(val);
        // Allow negative sign
        if (val === '' || val === '-') {
            onChange(0);
        } else {
            const num = parseFloat(val);
            if (!isNaN(num)) {
                onChange(num);
            }
        }
    };
    
    const handleBlur = () => {
        // If the user leaves the input empty or with just a '-', reset to the last valid value.
        if (strValue === '' || strValue === '-') {
            setStrValue(value.toString());
        }
    }

    return (
        <div className="flex items-center justify-between mb-2">
            <label className="text-xs text-gray-400 w-1/3">{label}</label>
            <input type="number" step={step} value={strValue} 
                onChange={handleChange} 
                onFocus={(e) => e.target.select()}
                onBlur={handleBlur}
                className="w-2/3 bg-gray-900 rounded p-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500" />
        </div>
    );
};

const Vector3Input = ({ label, value, onChange, path }: { label: string, value: [number, number, number], onChange: (path: string, val: any) => void, path: string }) => (
    <div className="p-2 bg-gray-900/50 rounded">
        <p className="text-sm font-semibold mb-1">{label}</p>
        <NumberInput label="X" value={value[0]} onChange={(v) => onChange(`${path}[0]`, v)} />
        <NumberInput label="Y" value={value[1]} onChange={(v) => onChange(`${path}[1]`, v)} />
        <NumberInput label="Z" value={value[2]} onChange={(v) => onChange(`${path}[2]`, v)} />
    </div>
);

const Checkbox = ({ label, checked, onChange }: { label: string, checked: boolean, onChange: (val: boolean) => void }) => (
    <label className="flex items-center space-x-2 cursor-pointer">
        <input type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)} className="h-4 w-4 rounded bg-gray-700 border-gray-600 text-blue-500 focus:ring-blue-500"/>
        <span className="text-sm">{label}</span>
    </label>
)

// --- Inspector for Links ---
const LinkInspector = ({ link }: { link: RobotLink }) => {
    const { updateLink, addJoint, uploadAndSetMesh, fitMeshToLink } = useRobotStore();
    const stlInputRef = useRef<HTMLInputElement>(null);

    const handleStlUploadClick = () => stlInputRef.current?.click();
    
    const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (file) {
            uploadAndSetMesh(link.id, 'link', file);
        }
        if(e.target) e.target.value = ''; // Reset input
    };

    return (
        <div className="space-y-4">
            <div className="flex items-center"> <ToyBrick className="mr-2 flex-shrink-0" />
                <input type="text" value={link.name} onChange={(e) => updateLink(link.id, 'name', e.target.value)}
                    className="text-lg font-bold bg-transparent focus:bg-gray-800 rounded p-1 -m-1 w-full"/>
            </div>

            {/* Visuals Section */}
            <div className="p-2 bg-gray-900/50 rounded space-y-3">
                <p className="text-sm font-semibold">Visuals</p>
                <div>
                    <label className="text-xs text-gray-400">Primitive Type</label>
                    <select value={link.visual.type} onChange={(e) => updateLink(link.id, 'visual.type', e.target.value)}
                        className="w-full bg-gray-700 rounded p-1 mt-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500">
                        <option value="box">Box</option> 
                        <option value="cylinder">Cylinder</option>
                        <option value="sphere">Sphere</option>
                        <option value="mesh">Mesh</option>
                        <option value="none">None (Virtual)</option>
                    </select>
                </div>

                <div className="flex items-center justify-between">
                    <label className="text-xs text-gray-400">Color</label>
                    <input 
                        type="color" 
                        value={link.visual.color} 
                        onChange={(e) => updateLink(link.id, 'visual.color', e.target.value)}
                        className="w-24 h-8 bg-gray-900 rounded p-0 border-2 border-gray-700 cursor-pointer"
                    />
                </div>

                {link.visual.type !== 'none' && link.visual.type !== 'mesh' && (
                  <Vector3Input label="Dimensions" value={link.visual.dimensions} onChange={(p, v) => updateLink(link.id, `visual.${p}`, v)} path="dimensions" />
                )}

                {/* --- STL/Mesh Controls --- */}
                <input type="file" ref={stlInputRef} onChange={handleFileChange} className="hidden" accept=".stl" />
                <button onClick={handleStlUploadClick} className="flex items-center justify-center w-full bg-purple-600 hover:bg-purple-700 p-2 rounded text-sm">
                    <UploadCloud className="mr-2 h-4 w-4" /> Upload STL
                </button>

                {link.visual.type === 'mesh' && (
                    <div className="space-y-3 pt-2">
                         <h4 className="text-md font-semibold text-gray-300 border-t border-gray-700 pt-3">Mesh Properties</h4>
                         <p className="text-xs text-gray-400 truncate">URL: {link.visual.meshUrl || 'N/A'}</p>
                         <Vector3Input label="Mesh Scale" value={link.visual.meshScale || [1,1,1]} onChange={(p, v) => updateLink(link.id, `visual.meshScale${p.substring(p.indexOf('['))}`, v)} path="meshScale" />
                         <Vector3Input label="Mesh Origin XYZ" value={link.visual.meshOrigin?.xyz || [0,0,0]} onChange={(p, v) => updateLink(link.id, `visual.meshOrigin.xyz${p.substring(p.indexOf('['))}`, v)} path="meshOrigin.xyz" />
                         <Vector3Input label="Mesh Origin RPY" value={link.visual.meshOrigin?.rpy || [0,0,0]} onChange={(p, v) => updateLink(link.id, `visual.meshOrigin.rpy${p.substring(p.indexOf('['))}`, v)} path="meshOrigin.rpy" />
                         <button onClick={() => fitMeshToLink(link.id)} title="Auto-scales the mesh to match the link's length (defined by its child joint). Assumes the mesh's main axis is Y." className="flex items-center justify-center w-full bg-teal-600 hover:bg-teal-700 p-2 rounded text-sm">
                            <LinkIcon className="mr-2 h-4 w-4" /> Fit to Link Length
                        </button>
                    </div>
                )}
            </div>

            <button onClick={() => addJoint(link.id)} className="flex items-center justify-center w-full bg-blue-600 hover:bg-blue-700 p-2 rounded text-sm">
                <PlusSquare className="mr-2 h-4 w-4" /> Add Child Joint
            </button>
        </div>
    );
};

// --- Inspector for Joints (with final workflow) ---
const JointInspector = ({ joint }: { joint: RobotJoint }) => {
    const { updateJoint, addChainedJoint, uploadAndSetMesh } = useRobotStore();
    const stlInputRef = useRef<HTMLInputElement>(null);

    const handleStlUploadClick = () => stlInputRef.current?.click();

    const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (file) {
            // Apply mesh to the joint's CHILD link
            uploadAndSetMesh(joint.id, 'joint', file);
        }
        if(e.target) e.target.value = ''; // Reset input
    };

    return (
        <div className="space-y-4">
            <div className="flex items-center"> <GitCommit className="mr-2 flex-shrink-0" />
                 <input type="text" value={joint.name} onChange={(e) => updateJoint(joint.id, 'name', e.target.value)}
                    className="text-lg font-bold bg-transparent focus:bg-gray-800 rounded p-1 -m-1 w-full"/>
            </div>
            
            <Vector3Input label="Origin XYZ" value={joint.origin?.xyz || [0,0,0]} onChange={(p, v) => updateJoint(joint.id, `origin.xyz${p.substring(p.indexOf('['))}`, v)} path="xyz" />
            <Vector3Input label="Origin RPY" value={joint.origin?.rpy || [0,0,0]} onChange={(p, v) => updateJoint(joint.id, `origin.rpy${p.substring(p.indexOf('['))}`, v)} path="rpy" />

            <div>
                <label className="text-xs text-gray-400">Joint Type</label>
                <select value={joint.type} onChange={(e) => updateJoint(joint.id, 'type', e.target.value as JointType)}
                    className="w-full bg-gray-700 rounded p-1 mt-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500">
                    <option value="fixed">Fixed</option> <option value="rotational">Rotational</option> <option value="prismatic">Prismatic</option>
                </select>
            </div>

            {joint.type === 'rotational' && (
                <div className="p-2 bg-gray-900/50 rounded">
                    <p className="text-sm font-semibold mb-2">Degrees of Freedom</p>
                    <div className="flex justify-around">
                        <Checkbox label="Roll (X)" checked={joint.dof.roll} onChange={v => updateJoint(joint.id, 'dof.roll', v)} />
                        <Checkbox label="Pitch (Y)" checked={joint.dof.pitch} onChange={v => updateJoint(joint.id, 'dof.pitch', v)} />
                        <Checkbox label="Yaw (Z)" checked={joint.dof.yaw} onChange={v => updateJoint(joint.id, 'dof.yaw', v)} />
                    </div>
                </div>
            )}
            {joint.type === 'prismatic' && (
                 <div className="p-2 bg-gray-900/50 rounded">
                    <Vector3Input label="Axis" value={joint.axis} onChange={(p, v) => updateJoint(joint.id, `axis${p.substring(p.indexOf('['))}`, v)} path="axis" />
                </div>
            )}

            {joint.type !== 'fixed' && (
                <div className="p-2 bg-gray-900/50 rounded">
                    <p className="text-sm font-semibold mb-2">Limits</p>
                    <NumberInput 
                        label="Lower" 
                        value={joint.limit?.lower ?? -Math.PI} 
                        onChange={v => updateJoint(joint.id, 'limit.lower', v)} 
                    />
                    <NumberInput 
                        label="Upper" 
                        value={joint.limit?.upper ?? Math.PI} 
                        onChange={v => updateJoint(joint.id, 'limit.upper', v)}
                    />
                </div>
            )}
            
            {/* Action Buttons */}
            <div className="space-y-2 pt-2 border-t border-gray-700">
                 {joint.childLinkId && (
                    <>
                        <input type="file" ref={stlInputRef} onChange={handleFileChange} className="hidden" accept=".stl" />
                        <button onClick={handleStlUploadClick} title="Applies a mesh to the child link of this joint" className="flex items-center justify-center w-full bg-purple-600 hover:bg-purple-700 p-2 rounded text-sm">
                            <UploadCloud className="mr-2 h-4 w-4" /> Upload STL to Child Link
                        </button>
                    </>
                )}
                {!joint.childLinkId && (
                     <button onClick={() => addChainedJoint(joint.id)} className="flex items-center justify-center w-full bg-green-600 hover:bg-green-700 p-2 rounded text-sm">
                        <LinkIcon className="mr-2 h-4 w-4" /> Add Chained Joint
                    </button>
                )}
            </div>

            {joint.type !== 'fixed' && (
                <div className="p-2 bg-gray-900/50 rounded space-y-2">
                    <p className="text-sm font-semibold">Test Driver</p>
                    {joint.type === 'rotational' && joint.dof.roll && (
                        <div><label className="text-xs">Roll</label><input type="range" min={joint.limit?.lower ?? -Math.PI} max={joint.limit?.upper ?? Math.PI} step={0.01} value={joint.currentValues.roll} onChange={e => updateJoint(joint.id, 'currentValues.roll', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                    {joint.type === 'rotational' && joint.dof.pitch && (
                        <div><label className="text-xs">Pitch</label><input type="range" min={joint.limit?.lower ?? -Math.PI} max={joint.limit?.upper ?? Math.PI} step={0.01} value={joint.currentValues.pitch} onChange={e => updateJoint(joint.id, 'currentValues.pitch', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                    {joint.type === 'rotational' && joint.dof.yaw && (
                        <div><label className="text-xs">Yaw</label><input type="range" min={joint.limit?.lower ?? -Math.PI} max={joint.limit?.upper ?? Math.PI} step={0.01} value={joint.currentValues.yaw} onChange={e => updateJoint(joint.id, 'currentValues.yaw', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                    {joint.type === 'prismatic' && (
                         <div><label className="text-xs">Displacement</label><input type="range" min={joint.limit?.lower ?? -1} max={joint.limit?.upper ?? 1} step={0.01} value={joint.currentValues.displacement} onChange={e => updateJoint(joint.id, 'currentValues.displacement', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                </div>
            )}
        </div>
    );
};

// --- Main Sidebar Component ---

// --- Global Joint Controller (Default View) ---
const GlobalJointController = () => {
    const { joints, updateJoint, selectItem, resetJointsToZero } = useRobotStore();
    const jointList = Object.values(joints).filter(j => j.type !== 'fixed');

    if (jointList.length === 0) {
        return <p className="text-gray-400">No movable joints in the robot. Add a joint to begin.</p>;
    }

    return (
        <div className="space-y-4">
            <div className="flex justify-between items-center">
                <h3 className="text-lg font-semibold text-gray-300">Robot Pose Controller</h3>
                <button 
                    onClick={resetJointsToZero} 
                    title="Reset all joint values to zero"
                    className="flex items-center text-sm bg-gray-700 hover:bg-gray-600 px-2 py-1 rounded"
                >
                    <RotateCcw className="mr-2 h-4 w-4" />
                    Reset Pose
                </button>
            </div>
            {jointList.map((joint) => (
                <div key={joint.id} className="p-2 bg-gray-900/50 rounded">
                    <p 
                        className="text-md font-semibold mb-2 cursor-pointer hover:text-blue-400"
                        onClick={() => selectItem(joint.id, 'joint')}
                        title="Click to inspect this joint"
                    >
                        {joint.name}
                    </p>
                    <div className="space-y-2">
                        {joint.type === 'rotational' && joint.dof.roll && (
                            <div><label className="text-xs flex justify-between"><span>Roll</span> <span>{joint.currentValues.roll.toFixed(2)}</span></label><input type="range" min={joint.limit?.lower ?? -Math.PI} max={joint.limit?.upper ?? Math.PI} step={0.01} value={joint.currentValues.roll} onChange={e => updateJoint(joint.id, 'currentValues.roll', parseFloat(e.target.value))} className="w-full"/></div>
                        )}
                        {joint.type === 'rotational' && joint.dof.pitch && (
                            <div><label className="text-xs flex justify-between"><span>Pitch</span> <span>{joint.currentValues.pitch.toFixed(2)}</span></label><input type="range" min={joint.limit?.lower ?? -Math.PI} max={joint.limit?.upper ?? Math.PI} step={0.01} value={joint.currentValues.pitch} onChange={e => updateJoint(joint.id, 'currentValues.pitch', parseFloat(e.target.value))} className="w-full"/></div>
                        )}
                        {joint.type === 'rotational' && joint.dof.yaw && (
                            <div><label className="text-xs flex justify-between"><span>Yaw</span> <span>{joint.currentValues.yaw.toFixed(2)}</span></label><input type="range" min={joint.limit?.lower ?? -Math.PI} max={joint.limit?.upper ?? Math.PI} step={0.01} value={joint.currentValues.yaw} onChange={e => updateJoint(joint.id, 'currentValues.yaw', parseFloat(e.target.value))} className="w-full"/></div>
                        )}
                        {joint.type === 'prismatic' && (
                             <div><label className="text-xs flex justify-between"><span>Displacement</span> <span>{joint.currentValues.displacement.toFixed(2)}</span></label><input type="range" min={joint.limit?.lower ?? -1} max={joint.limit?.upper ?? 1} step={0.01} value={joint.currentValues.displacement} onChange={e => updateJoint(joint.id, 'currentValues.displacement', parseFloat(e.target.value))} className="w-full"/></div>
                        )}
                    </div>
                </div>
            ))}
        </div>
    );
};


const Sidebar = () => {
    const { selectedItem, links, joints, selectItem, saveRobot, loadRobot } = useRobotStore();
    const selectedLink = selectedItem.type === 'link' ? links[selectedItem.id!] : null;
    const selectedJoint = selectedItem.type === 'joint' ? joints[selectedItem.id!] : null;
    
    const fileInputRef = useRef<HTMLInputElement>(null);

    const handleLoadClick = () => {
        fileInputRef.current?.click();
    };

    const handleFileChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (file) {
            loadRobot(file);
        }
        // Reset file input to allow loading the same file again
        if(event.target) {
            event.target.value = '';
        }
    };


    return (
        <div className="absolute top-0 right-0 h-screen w-80 bg-gray-800 bg-opacity-80 backdrop-blur-sm text-white p-4 border-l border-gray-700 overflow-y-auto">
            {/* File Operations */}
            <div className="pb-4 mb-4 border-b border-gray-700">
                <h2 className="text-xl font-bold">Robot Link Forge</h2>
                <div className="flex space-x-2 mt-2">
                    <button 
                        onClick={() => saveRobot()}
                        className="flex-1 flex items-center justify-center bg-indigo-600 hover:bg-indigo-700 p-2 rounded text-sm"
                    >
                       <Save className="mr-2 h-4 w-4" /> Save
                    </button>
                    <button 
                        onClick={handleLoadClick}
                        className="flex-1 flex items-center justify-center bg-gray-600 hover:bg-gray-700 p-2 rounded text-sm"
                    >
                       <FolderOpen className="mr-2 h-4 w-4" /> Load
                    </button>
                     <input
                        type="file"
                        ref={fileInputRef}
                        onChange={handleFileChange}
                        className="hidden"
                        accept=".zip,application/zip,.json,application/json"
                    />
                </div>
            </div>

            {/* Inspector / Global Controls */}
            <div className="flex justify-between items-center mb-4">
                <h2 className="text-xl font-bold">{selectedItem.id ? 'Inspector' : 'Global Controls'}</h2>
                {selectedItem.id && (
                    <button onClick={() => selectItem(null, null)} title="Deselect and show Global Controls"
                        className="flex items-center text-sm bg-gray-700 hover:bg-gray-600 p-2 rounded">
                        <Move3d className="h-4 w-4" />
                    </button>
                )}
            </div>

            {!selectedItem.id && <GlobalJointController />}
            {selectedLink && <LinkInspector link={selectedLink} />}
            {selectedJoint && <JointInspector joint={selectedJoint} />}
        </div>
    );
};

export default Sidebar;