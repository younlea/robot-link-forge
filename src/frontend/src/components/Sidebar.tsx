// src/frontend/src/components/Sidebar.tsx
import React, { useState, useEffect } from 'react';
import { useRobotStore } from '../store';
import { RobotLink, RobotJoint, JointType } from '../types';
import { ToyBrick, PlusSquare, Link as LinkIcon, GitCommit, Move3d } from 'lucide-react';

// --- Reusable Input Components (with fixes) ---
const NumberInput = ({ label, value, onChange, step = 0.01 }: { label: string, value: number, onChange: (val: number) => void, step?: number }) => {
    const [strValue, setStrValue] = useState(value.toString());

    useEffect(() => {
        if (parseFloat(strValue) !== value) {
            setStrValue(value.toString());
        }
    }, [value, strValue]);

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const val = e.target.value;
        setStrValue(val);
        if (val === '' || val === '-') {
            onChange(0);
        } else {
            const num = parseFloat(val);
            if (!isNaN(num)) {
                onChange(num);
            }
        }
    };
    
    const handleBlur = () => { setStrValue(value.toString()); }

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
    const { updateLink, addJoint } = useRobotStore();
    return (
        <div className="space-y-4">
            <div className="flex items-center"> <ToyBrick className="mr-2 flex-shrink-0" />
                <input type="text" value={link.name} onChange={(e) => updateLink(link.id, 'name', e.target.value)}
                    className="text-lg font-bold bg-transparent focus:bg-gray-800 rounded p-1 -m-1 w-full"/>
            </div>
            <div>
                <label className="text-xs text-gray-400">Visual Type</label>
                <select value={link.visual.type} onChange={(e) => updateLink(link.id, 'visual.type', e.target.value)}
                    className="w-full bg-gray-700 rounded p-1 mt-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500">
                    <option value="box">Box</option> <option value="cylinder">Cylinder</option>
                    <option value="sphere">Sphere</option> <option value="none">None (Virtual)</option>
                </select>
            </div>
            {link.visual.type !== 'none' && (
              <Vector3Input label="Dimensions" value={link.visual.dimensions} onChange={(p, v) => updateLink(link.id, `visual.${p}`, v)} path="dimensions" />
            )}
            <button onClick={() => addJoint(link.id)} className="flex items-center justify-center w-full bg-blue-600 hover:bg-blue-700 p-2 rounded text-sm">
                <PlusSquare className="mr-2 h-4 w-4" /> Add Child Joint
            </button>
        </div>
    );
};

// --- Inspector for Joints (with final workflow) ---
const JointInspector = ({ joint }: { joint: RobotJoint }) => {
    const { updateJoint, addChainedJoint } = useRobotStore();
    return (
        <div className="space-y-4">
            <div className="flex items-center"> <GitCommit className="mr-2 flex-shrink-0" />
                 <input type="text" value={joint.name} onChange={(e) => updateJoint(joint.id, 'name', e.target.value)}
                    className="text-lg font-bold bg-transparent focus:bg-gray-800 rounded p-1 -m-1 w-full"/>
            </div>
            
            <Vector3Input label="Origin XYZ" value={joint.origin?.xyz || [0,0,0]} onChange={(p, v) => updateJoint(joint.id, `origin.${p}`, v)} path="xyz" />
            <Vector3Input label="Origin RPY" value={joint.origin?.rpy || [0,0,0]} onChange={(p, v) => updateJoint(joint.id, `origin.${p}`, v)} path="rpy" />

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
                    <Vector3Input label="Axis" value={joint.axis} onChange={(p, v) => updateJoint(joint.id, p, v)} path="axis" />
                </div>
            )}

            {joint.type !== 'fixed' && (
                <div className="p-2 bg-gray-900/50 rounded space-y-2">
                    <p className="text-sm font-semibold">Test Driver</p>
                    {joint.type === 'rotational' && joint.dof.roll && (
                        <div><label className="text-xs">Roll</label><input type="range" min={-Math.PI} max={Math.PI} step={0.01} value={joint.currentValues.roll} onChange={e => updateJoint(joint.id, 'currentValues.roll', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                    {joint.type === 'rotational' && joint.dof.pitch && (
                        <div><label className="text-xs">Pitch</label><input type="range" min={-Math.PI} max={Math.PI} step={0.01} value={joint.currentValues.pitch} onChange={e => updateJoint(joint.id, 'currentValues.pitch', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                    {joint.type === 'rotational' && joint.dof.yaw && (
                        <div><label className="text-xs">Yaw</label><input type="range" min={-Math.PI} max={Math.PI} step={0.01} value={joint.currentValues.yaw} onChange={e => updateJoint(joint.id, 'currentValues.yaw', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                    {joint.type === 'prismatic' && (
                         <div><label className="text-xs">Displacement</label><input type="range" min={-1} max={1} step={0.01} value={joint.currentValues.displacement} onChange={e => updateJoint(joint.id, 'currentValues.displacement', parseFloat(e.target.value))} className="w-full"/></div>
                    )}
                </div>
            )}
            
            {!joint.childLinkId && (
                 <button onClick={() => addChainedJoint(joint.id)} className="flex items-center justify-center w-full bg-green-600 hover:bg-green-700 p-2 rounded text-sm">
                    <LinkIcon className="mr-2 h-4 w-4" /> Add Chained Joint
                </button>
            )}
        </div>
    );
};

// --- Main Sidebar Component ---
const Sidebar = () => {
    const { selectedItem, links, joints, selectItem } = useRobotStore();
    const selectedLink = selectedItem.type === 'link' ? links[selectedItem.id!] : null;
    const selectedJoint = selectedItem.type === 'joint' ? joints[selectedItem.id!] : null;

    return (
        <div className="absolute top-0 right-0 h-screen w-80 bg-gray-800 bg-opacity-80 backdrop-blur-sm text-white p-4 border-l border-gray-700 overflow-y-auto">
            <div className="flex justify-between items-center mb-4">
                <h2 className="text-xl font-bold">Inspector</h2>
                {selectedItem.id && (
                    <button onClick={() => selectItem(null, null)} title="Deselect"
                        className="flex items-center text-sm bg-gray-700 hover:bg-gray-600 p-2 rounded">
                        <Move3d className="h-4 w-4" />
                    </button>
                )}
            </div>

            {!selectedItem.id && <p className="text-gray-400">Select a Link or Joint in the 3D view.</p>}
            {selectedLink && <LinkInspector link={selectedLink} />}
            {selectedJoint && <JointInspector joint={selectedJoint} />}
        </div>
    );
};

export default Sidebar;
