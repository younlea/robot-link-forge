import React, { useState, useEffect, useRef } from 'react';
import { useRobotStore } from '../store';
import { RobotLink, RobotJoint, JointType } from '../types';
import { ToyBrick, PlusSquare, Link as LinkIcon, GitCommit, Move3d, Save, FolderOpen, Upload, RotateCcw, Trash2, FilePlus, HelpCircle, Settings } from 'lucide-react';
import HelpModal from './HelpModal';

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

const Vector3RadianDegreeInput = ({ label, value, onChange, path }: { label: string, value: [number, number, number], onChange: (path: string, val: any) => void, path: string }) => (
    <div className="p-2 bg-gray-900/50 rounded">
        <div className="flex justify-between items-center mb-1">
            <p className="text-sm font-semibold">{label}</p>
            <div className="flex w-3/4 space-x-1">
                <span className="text-xs text-gray-400 w-1/2 text-center">Rad</span>
                <span className="text-xs text-gray-400 w-1/2 text-center">Deg</span>
            </div>
        </div>
        <RadianDegreeInput label="R (X)" radValue={value[0]} onRadChange={(v) => onChange(`${path}[0]`, v)} />
        <RadianDegreeInput label="P (Y)" radValue={value[1]} onRadChange={(v) => onChange(`${path}[1]`, v)} />
        <RadianDegreeInput label="Y (Z)" radValue={value[2]} onRadChange={(v) => onChange(`${path}[2]`, v)} />
    </div>
);

const RadianDegreeInput = ({ label, radValue, onRadChange }: { label: string, radValue: number, onRadChange: (rad: number) => void }) => {
    const toDegrees = (rad: number) => (rad * 180) / Math.PI;
    const toRadians = (deg: number) => (deg * Math.PI) / 180;

    const [radStr, setRadStr] = useState(radValue.toFixed(3));
    const [degStr, setDegStr] = useState(toDegrees(radValue).toFixed(1));

    // Track which input is focused to prevent overwriting the user while typing
    const [activeInput, setActiveInput] = useState<'rad' | 'deg' | null>(null);

    useEffect(() => {
        // Only update local state if the external value changed AND we are NOT currently editing that specific field
        // If we are editing 'deg', we shouldn't overwrite 'deg' with a re-calculated value from 'rad', 
        // because it might interrupt typing (e.g. typing "180" might get rounded/formatted instantly).

        if (activeInput !== 'rad') {
            if (Math.abs(parseFloat(radStr) - radValue) > 0.0001) {
                setRadStr(radValue.toFixed(3));
            }
        }

        if (activeInput !== 'deg') {
            const degVal = toDegrees(radValue);
            const currentDegStrVal = parseFloat(degStr);
            // Allow some epsilon for float comparison to avoid unnecessary updates
            if (Math.abs(currentDegStrVal - degVal) > 0.1 || isNaN(currentDegStrVal)) {
                setDegStr(degVal.toFixed(1));
            }
        }
    }, [radValue, activeInput]); // Re-run if external value changes or focus changes

    const handleRadChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const val = e.target.value;
        setRadStr(val);
        if (val === '' || val === '-') {
            onRadChange(0);
            // Do not force update degStr immediately here if we want to keep them independent while typing,
            // but usually we want to see the conversion.
            // The useEffect will handle the "other" field update if it's not focused.
        } else {
            const num = parseFloat(val);
            if (!isNaN(num)) {
                onRadChange(num);
            }
        }
    };

    const handleDegChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const val = e.target.value;
        setDegStr(val);
        if (val === '' || val === '-') {
            onRadChange(0);
        } else {
            const num = parseFloat(val);
            if (!isNaN(num)) {
                const rad = toRadians(num);
                onRadChange(rad);
            }
        }
    };

    return (
        <div className="flex items-center justify-between mb-2">
            <label className="text-xs text-gray-400 w-1/4">{label}</label>
            <div className="flex w-3/4 space-x-1">
                <input
                    type="number"
                    step={0.01}
                    value={radStr}
                    onChange={handleRadChange}
                    onFocus={(e) => { setActiveInput('rad'); e.target.select(); }}
                    onBlur={() => setActiveInput(null)}
                    className="w-1/2 bg-gray-900 rounded p-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500"
                    title="Radians"
                />
                <input
                    type="number"
                    step={1}
                    value={degStr}
                    onChange={handleDegChange}
                    onFocus={(e) => { setActiveInput('deg'); e.target.select(); }}
                    onBlur={() => setActiveInput(null)}
                    className="w-1/2 bg-gray-900 rounded p-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500"
                    title="Degrees"
                />
            </div>
        </div>
    );
};

const Checkbox = ({ label, checked, onChange }: { label: string, checked: boolean, onChange: (val: boolean) => void }) => (
    <label className="flex items-center space-x-2 cursor-pointer">
        <input type="checkbox" checked={checked} onChange={(e) => onChange(e.target.checked)} className="h-4 w-4 rounded bg-gray-700 border-gray-600 text-blue-500 focus:ring-blue-500" />
        <span className="text-sm">{label}</span>
    </label>
)
// --- JointSliderInput (New) ---
// --- JointSliderInput (New) ---
const JointSliderInput = ({
    label,
    value,
    min,
    max,
    step = 0.01,
    onChange,
    unit = "",
    toUser = (v) => v,
    fromUser = (v) => v,
    autoFocus = false
}: {
    label: string,
    value: number,
    min: number,
    max: number,
    step?: number,
    onChange: (val: number) => void,
    unit?: string,
    toUser?: (v: number) => number,
    fromUser?: (v: number) => number,
    autoFocus?: boolean
}) => {
    // Initialize strValue based on the user-facing value
    const [strValue, setStrValue] = useState(toUser(value).toFixed(2));
    const [isEditing, setIsEditing] = useState(false);

    useEffect(() => {
        if (!isEditing) {
            setStrValue(toUser(value).toFixed(2));
        }
    }, [value, isEditing, toUser]);

    const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        setStrValue(e.target.value);
    };

    const commitValue = () => {
        let num = parseFloat(strValue);
        if (isNaN(num)) {
            setStrValue(toUser(value).toFixed(2));
        } else {
            // Convert user input back to internal value
            const internalVal = fromUser(num);
            // Clamp value (internal limits)
            const clamped = Math.min(Math.max(internalVal, min), max);
            onChange(clamped);
            // Update display string to match the clamped value
            setStrValue(toUser(clamped).toFixed(2));
        }
        setIsEditing(false);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === 'Enter') {
            commitValue();
            e.currentTarget.blur();
        }
    };

    return (
        <div className="mb-2">
            <div className="flex justify-between items-center mb-1">
                <label className="text-xs text-gray-400">{label}</label>
                <div className="flex items-center">
                    <input
                        type="text"
                        value={strValue}
                        onChange={handleInputChange}
                        onFocus={() => setIsEditing(true)}
                        onBlur={commitValue}
                        onKeyDown={handleKeyDown}
                        autoFocus={autoFocus}
                        className="w-16 bg-gray-900 rounded p-0.5 text-xs text-right focus:outline-none focus:ring-1 focus:ring-blue-500"
                    />
                    {unit && <span className="ml-1 text-xs text-gray-500">{unit}</span>}
                </div>
            </div>
            <input
                type="range"
                min={min}
                max={max}
                step={step}
                value={value}
                onChange={(e) => onChange(parseFloat(e.target.value))}
                className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
            />
        </div>
    );
};

const TextInput = ({ value, onChange, className }: { value: string, onChange: (val: string) => void, className?: string }) => {
    const [localValue, setLocalValue] = useState(value);

    useEffect(() => {
        setLocalValue(value);
    }, [value]);

    const handleBlur = () => {
        if (localValue !== value) {
            onChange(localValue);
        }
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === 'Enter') {
            e.currentTarget.blur();
        }
    };

    return (
        <input
            type="text"
            value={localValue}
            onChange={(e) => setLocalValue(e.target.value)}
            onBlur={handleBlur}
            onKeyDown={handleKeyDown}
            className={className}
        />
    );
};

// --- Inspector for Links ---
const LinkInspector = ({ link }: { link: RobotLink }) => {
    const { updateLink, addJoint, uploadAndSetMesh, fitMeshToLink, deleteItem } = useRobotStore();
    const stlInputRef = useRef<HTMLInputElement>(null);

    const handleStlUploadClick = () => stlInputRef.current?.click();

    const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (file) {
            uploadAndSetMesh(link.id, 'link', file);
        }
        if (e.target) e.target.value = ''; // Reset input
    };

    return (
        <div className="space-y-4">
            <div className="flex items-center"> <ToyBrick className="mr-2 flex-shrink-0" />
                <TextInput value={link.name} onChange={(val) => updateLink(link.id, 'name', val)}
                    className="text-lg font-bold bg-transparent focus:bg-gray-800 rounded p-1 -m-1 w-full" />
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
                    <Upload className="mr-2 h-4 w-4" /> Upload STL
                </button>

                {link.visual.type === 'mesh' && (
                    <div className="space-y-3 pt-2">
                        <h4 className="text-md font-semibold text-gray-300 border-t border-gray-700 pt-3">Mesh Properties</h4>
                        <p className="text-xs text-gray-400 truncate">URL: {link.visual.meshUrl || 'N/A'}</p>
                        <Vector3Input label="Mesh Scale" value={link.visual.meshScale || [1, 1, 1]} onChange={(p, v) => updateLink(link.id, `visual.meshScale${p.substring(p.indexOf('['))}`, v)} path="meshScale" />
                        <Vector3Input label="Mesh Origin XYZ" value={link.visual.meshOrigin?.xyz || [0, 0, 0]} onChange={(p, v) => updateLink(link.id, `visual.meshOrigin.xyz${p.substring(p.indexOf('['))}`, v)} path="xyz" />
                        <Vector3RadianDegreeInput label="Mesh Origin RPY" value={link.visual.meshOrigin?.rpy || [0, 0, 0]} onChange={(p, v) => updateLink(link.id, `visual.meshOrigin.rpy${p.substring(p.indexOf('['))}`, v)} path="rpy" />
                        <button onClick={() => fitMeshToLink(link.id)} title="Auto-scales the mesh to match the link's length (defined by its child joint). Assumes the mesh's main axis is Y." className="flex items-center justify-center w-full bg-teal-600 hover:bg-teal-700 p-2 rounded text-sm">
                            <LinkIcon className="mr-2 h-4 w-4" /> Fit to Link Length
                        </button>
                    </div>
                )}
            </div>

            <button onClick={() => addJoint(link.id)} className="flex items-center justify-center w-full bg-blue-600 hover:bg-blue-700 p-2 rounded text-sm">
                <PlusSquare className="mr-2 h-4 w-4" /> Add Child Joint
            </button>

            <button onClick={() => { if (confirm('Delete this link and all its children?')) deleteItem(link.id, 'link'); }} className="flex items-center justify-center w-full bg-red-900/50 hover:bg-red-700 p-2 rounded text-sm text-red-200 border border-red-800">
                <Trash2 className="mr-2 h-4 w-4" /> Delete Link
            </button>
        </div>
    );
};

// --- Inspector for Joints (with final workflow) ---
const JointInspector = ({ joint }: { joint: RobotJoint }) => {
    const { updateJoint, addChainedJoint, uploadAndSetMesh, deleteItem } = useRobotStore();
    const stlInputRef = useRef<HTMLInputElement>(null);

    const handleStlUploadClick = () => stlInputRef.current?.click();

    const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (file) {
            // Apply mesh to the joint's CHILD link
            uploadAndSetMesh(joint.id, 'joint', file);
        }
        if (e.target) e.target.value = ''; // Reset input
    };

    return (
        <div className="space-y-4">
            <div className="flex items-center"> <GitCommit className="mr-2 flex-shrink-0" />
                <TextInput value={joint.name} onChange={(val) => updateJoint(joint.id, 'name', val)}
                    className="text-lg font-bold bg-transparent focus:bg-gray-800 rounded p-1 -m-1 w-full" />
            </div>

            {joint.type !== 'fixed' && (
                <div className="p-2 bg-blue-900/20 border border-blue-900/50 rounded space-y-2">
                    <p className="text-xs font-bold text-blue-400 uppercase tracking-wider mb-1">Joint Control</p>
                    {joint.type === 'rotational' && joint.dof.roll && (
                        <JointSliderInput
                            label="Roll"
                            min={joint.limits.roll.lower}
                            max={joint.limits.roll.upper}
                            value={joint.currentValues.roll}
                            onChange={(v) => updateJoint(joint.id, 'currentValues.roll', v)}
                            unit="°"
                            toUser={(rad) => rad * 180 / Math.PI}
                            fromUser={(deg) => deg * Math.PI / 180}
                            autoFocus={true}
                        />
                    )}
                    {joint.type === 'rotational' && joint.dof.pitch && (
                        <JointSliderInput
                            label="Pitch"
                            min={joint.limits.pitch.lower}
                            max={joint.limits.pitch.upper}
                            value={joint.currentValues.pitch}
                            onChange={(v) => updateJoint(joint.id, 'currentValues.pitch', v)}
                            unit="°"
                            toUser={(rad) => rad * 180 / Math.PI}
                            fromUser={(deg) => deg * Math.PI / 180}
                            autoFocus={!joint.dof.roll}
                        />
                    )}
                    {joint.type === 'rotational' && joint.dof.yaw && (
                        <JointSliderInput
                            label="Yaw"
                            min={joint.limits.yaw.lower}
                            max={joint.limits.yaw.upper}
                            value={joint.currentValues.yaw}
                            onChange={(v) => updateJoint(joint.id, 'currentValues.yaw', v)}
                            unit="°"
                            toUser={(rad) => rad * 180 / Math.PI}
                            fromUser={(deg) => deg * Math.PI / 180}
                            autoFocus={!joint.dof.roll && !joint.dof.pitch}
                        />
                    )}
                    {joint.type === 'prismatic' && (
                        <JointSliderInput
                            label="Displacement"
                            min={joint.limits.displacement.lower}
                            max={joint.limits.displacement.upper}
                            value={joint.currentValues.displacement}
                            onChange={(v) => updateJoint(joint.id, 'currentValues.displacement', v)}
                            unit="mm"
                            toUser={(m) => m * 1000}
                            fromUser={(mm) => mm / 1000}
                            autoFocus={true}
                        />
                    )}
                </div>
            )}

            <Vector3Input label="Origin XYZ" value={joint.origin?.xyz || [0, 0, 0]} onChange={(p, v) => updateJoint(joint.id, `origin.xyz${p.substring(p.indexOf('['))}`, v)} path="xyz" />
            <Vector3RadianDegreeInput label="Origin RPY" value={joint.origin?.rpy || [0, 0, 0]} onChange={(p, v) => updateJoint(joint.id, `origin.rpy${p.substring(p.indexOf('['))}`, v)} path="rpy" />

            <div>
                <label className="text-xs text-gray-400">Joint Type</label>
                <select value={joint.type} onChange={(e) => updateJoint(joint.id, 'type', e.target.value as JointType)}
                    className="w-full bg-gray-700 rounded p-1 mt-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500">
                    <option value="fixed">Fixed</option> <option value="rotational">Rotational</option> <option value="prismatic">Prismatic</option>
                </select>
            </div>

            {joint.type === 'rotational' && (

                <div className="p-2 bg-gray-900/50 rounded space-y-3">

                    <div>

                        <p className="text-sm font-semibold mb-2">Degrees of Freedom</p>

                        <div className="flex justify-around">

                            <Checkbox label="Roll (X)" checked={joint.dof.roll} onChange={v => updateJoint(joint.id, 'dof.roll', v)} />

                            <Checkbox label="Pitch (Y)" checked={joint.dof.pitch} onChange={v => updateJoint(joint.id, 'dof.pitch', v)} />

                            <Checkbox label="Yaw (Z)" checked={joint.dof.yaw} onChange={v => updateJoint(joint.id, 'dof.yaw', v)} />

                        </div>

                    </div>



                    {(joint.dof.roll || joint.dof.pitch || joint.dof.yaw) && (

                        <div className="pt-2 border-t border-gray-800">

                            <div className="flex justify-between items-center mb-2">

                                <p className="text-sm font-semibold">Limits</p>

                                <div className="flex w-3/4 space-x-1">

                                    <span className="text-xs text-gray-400 w-1/2 text-center">Rad</span>

                                    <span className="text-xs text-gray-400 w-1/2 text-center">Deg</span>

                                </div>

                            </div>

                            {joint.dof.roll && <>

                                <RadianDegreeInput label="Roll Lower" radValue={joint.limits.roll.lower} onRadChange={v => updateJoint(joint.id, 'limits.roll.lower', v)} />

                                <RadianDegreeInput label="Roll Upper" radValue={joint.limits.roll.upper} onRadChange={v => updateJoint(joint.id, 'limits.roll.upper', v)} />

                            </>}

                            {joint.dof.pitch && <>

                                <RadianDegreeInput label="Pitch Lower" radValue={joint.limits.pitch.lower} onRadChange={v => updateJoint(joint.id, 'limits.pitch.lower', v)} />

                                <RadianDegreeInput label="Pitch Upper" radValue={joint.limits.pitch.upper} onRadChange={v => updateJoint(joint.id, 'limits.pitch.upper', v)} />

                            </>}

                            {joint.dof.yaw && <>

                                <RadianDegreeInput label="Yaw Lower" radValue={joint.limits.yaw.lower} onRadChange={v => updateJoint(joint.id, 'limits.yaw.lower', v)} />

                                <RadianDegreeInput label="Yaw Upper" radValue={joint.limits.yaw.upper} onRadChange={v => updateJoint(joint.id, 'limits.yaw.upper', v)} />

                            </>}

                        </div>

                    )}

                </div>

            )}

            {joint.type === 'prismatic' && (

                <div className="p-2 bg-gray-900/50 rounded space-y-3">

                    <Vector3Input label="Axis" value={joint.axis} onChange={(p, v) => updateJoint(joint.id, `axis${p.substring(p.indexOf('['))}`, v)} path="axis" />

                    <div className="pt-2 border-t border-gray-800">

                        <p className="text-sm font-semibold mb-2">Limits</p>

                        <NumberInput label="Lower" value={joint.limits.displacement.lower} onChange={v => updateJoint(joint.id, 'limits.displacement.lower', v)} />

                        <NumberInput label="Upper" value={joint.limits.displacement.upper} onChange={v => updateJoint(joint.id, 'limits.displacement.upper', v)} />

                    </div>

                </div>

            )}



            {/* --- Visuals Section (New) --- */}
            <div className="p-2 bg-gray-900/50 rounded space-y-3">
                <p className="text-sm font-semibold">Visuals (e.g. Motor Housing)</p>
                <div>
                    <label className="text-xs text-gray-400">Visual Type</label>
                    <select value={joint.visual?.type || 'none'} onChange={(e) => updateJoint(joint.id, 'visual.type', e.target.value)}
                        className="w-full bg-gray-700 rounded p-1 mt-1 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500">
                        <option value="box">Box</option>
                        <option value="cylinder">Cylinder</option>
                        <option value="sphere">Sphere</option>
                        <option value="mesh">Mesh (STL)</option>
                        <option value="none">None</option>
                    </select>
                </div>

                {joint.visual && joint.visual.type !== 'none' && (
                    <>
                        <div className="flex items-center justify-between">
                            <label className="text-xs text-gray-400">Color</label>
                            <input
                                type="color"
                                value={joint.visual.color || '#888888'}
                                onChange={(e) => updateJoint(joint.id, 'visual.color', e.target.value)}
                                className="w-24 h-8 bg-gray-900 rounded p-0 border-2 border-gray-700 cursor-pointer"
                            />
                        </div>

                        {joint.visual.type !== 'mesh' && (
                            <Vector3Input label="Dimensions" value={joint.visual.dimensions || [0.05, 0.05, 0.05]} onChange={(p, v) => updateJoint(joint.id, `visual.${p}`, v)} path="dimensions" />
                        )}

                        {/* --- STL/Mesh Controls --- */}
                        <input type="file" ref={stlInputRef} onChange={handleFileChange} className="hidden" accept=".stl" />

                        {joint.visual.type === 'mesh' && (
                            <>
                                <button onClick={handleStlUploadClick} className="flex items-center justify-center w-full bg-purple-600 hover:bg-purple-700 p-2 rounded text-sm">
                                    <Upload className="mr-2 h-4 w-4" /> Upload STL for Joint
                                </button>

                                <div className="space-y-3 pt-2">
                                    <h4 className="text-md font-semibold text-gray-300 border-t border-gray-700 pt-3">Mesh Properties</h4>
                                    <p className="text-xs text-gray-400 truncate">URL: {joint.visual.meshUrl || 'N/A'}</p>
                                    <Vector3Input label="Mesh Scale" value={joint.visual.meshScale || [1, 1, 1]} onChange={(p, v) => updateJoint(joint.id, `visual.meshScale${p.substring(p.indexOf('['))}`, v)} path="meshScale" />
                                    <Vector3Input label="Mesh Origin XYZ" value={joint.visual.meshOrigin?.xyz || [0, 0, 0]} onChange={(p, v) => updateJoint(joint.id, `visual.meshOrigin.xyz${p.substring(p.indexOf('['))}`, v)} path="xyz" />
                                    <Vector3RadianDegreeInput label="Mesh Origin RPY" value={joint.visual.meshOrigin?.rpy || [0, 0, 0]} onChange={(p, v) => updateJoint(joint.id, `visual.meshOrigin.rpy${p.substring(p.indexOf('['))}`, v)} path="rpy" />
                                </div>
                            </>
                        )}
                    </>
                )}
            </div>

            {/* Action Buttons */}

            <div className="space-y-2 pt-2 border-t border-gray-700">

                {!joint.childLinkId && (

                    <button onClick={() => addChainedJoint(joint.id)} className="flex items-center justify-center w-full bg-green-600 hover:bg-green-700 p-2 rounded text-sm">

                        <LinkIcon className="mr-2 h-4 w-4" /> Add Chained Joint

                    </button>

                )}

            </div>







            <button onClick={() => { if (confirm('Delete this joint and its children?')) deleteItem(joint.id, 'joint'); }} className="flex items-center justify-center w-full bg-red-900/50 hover:bg-red-700 p-2 rounded text-sm text-red-200 border border-red-800 mt-4">
                <Trash2 className="mr-2 h-4 w-4" /> Delete Joint
            </button>
        </div >

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
                            <JointSliderInput
                                label="Roll"
                                min={joint.limits.roll.lower}
                                max={joint.limits.roll.upper}
                                value={joint.currentValues.roll}
                                onChange={(v) => updateJoint(joint.id, 'currentValues.roll', v)}
                                unit="°"
                                toUser={(rad) => rad * 180 / Math.PI}
                                fromUser={(deg) => deg * Math.PI / 180}
                            />
                        )}

                        {joint.type === 'rotational' && joint.dof.pitch && (
                            <JointSliderInput
                                label="Pitch"
                                min={joint.limits.pitch.lower}
                                max={joint.limits.pitch.upper}
                                value={joint.currentValues.pitch}
                                onChange={(v) => updateJoint(joint.id, 'currentValues.pitch', v)}
                                unit="°"
                                toUser={(rad) => rad * 180 / Math.PI}
                                fromUser={(deg) => deg * Math.PI / 180}
                            />
                        )}

                        {joint.type === 'rotational' && joint.dof.yaw && (
                            <JointSliderInput
                                label="Yaw"
                                min={joint.limits.yaw.lower}
                                max={joint.limits.yaw.upper}
                                value={joint.currentValues.yaw}
                                onChange={(v) => updateJoint(joint.id, 'currentValues.yaw', v)}
                                unit="°"
                                toUser={(rad) => rad * 180 / Math.PI}
                                fromUser={(deg) => deg * Math.PI / 180}
                            />
                        )}

                        {joint.type === 'prismatic' && (
                            <JointSliderInput
                                label="Displacement"
                                min={joint.limits.displacement.lower}
                                max={joint.limits.displacement.upper}
                                value={joint.currentValues.displacement}
                                onChange={(v) => updateJoint(joint.id, 'currentValues.displacement', v)}
                                unit="mm"
                                toUser={(m) => m * 1000}
                                fromUser={(mm) => mm / 1000}
                            />
                        )}

                    </div>

                </div>

            ))}

        </div>

    );

};


const Sidebar = () => {
    const { selectedItem, links, joints, selectItem, saveRobot, loadRobot, exportURDF, exportURDF_ROS2, resetProject, saveProjectToServer, getProjectList, serverProjects, loadProjectFromServer, importUnit, setImportUnit, collisionMode, setCollisionMode, collisionBoxScale, setCollisionBoxScale } = useRobotStore();
    const selectedLink = selectedItem.type === 'link' ? links[selectedItem.id!] : null;
    const selectedJoint = selectedItem.type === 'joint' ? joints[selectedItem.id!] : null;

    const fileInputRef = useRef<HTMLInputElement>(null);
    const [isExportMenuOpen, setExportMenuOpen] = useState(false);
    const exportMenuRef = useRef<HTMLDivElement>(null);

    // State for the new export modal
    const [showExportModal, setShowExportModal] = useState(false);
    const [exportFormat, setExportFormat] = useState<'urdf' | 'urdf_ros2' | null>(null);
    const [robotName, setRobotName] = useState('');

    // State for Save/Load Modals
    const [showSaveModal, setShowSaveModal] = useState(false);
    const [showLoadModal, setShowLoadModal] = useState(false);
    const [showHelpModal, setShowHelpModal] = useState(false);
    const [showSettingsModal, setShowSettingsModal] = useState(false);
    const [saveProjectName, setSaveProjectName] = useState('');

    const handleLoadClick = () => {
        fileInputRef.current?.click();
    };

    const handleFileChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (file) {
            loadRobot(file);
        }
        if (event.target) event.target.value = '';
    };

    const openExportModal = (format: 'urdf' | 'urdf_ros2') => {
        const { baseLinkId, links } = useRobotStore.getState();
        const defaultName = links[baseLinkId]?.name || 'my_robot';
        setRobotName(defaultName);
        setExportFormat(format);
        setShowExportModal(true);
        setExportMenuOpen(false);
    };

    const handleConfirmExport = () => {
        if (!robotName || !exportFormat) return;

        console.log(`Exporting to ${exportFormat} with name ${robotName}`);
        if (exportFormat === 'urdf') {
            exportURDF(robotName);
        } else if (exportFormat === 'urdf_ros2') {
            exportURDF_ROS2(robotName);
        }

        // Close modal
        setShowExportModal(false);
        setExportFormat(null);
        setRobotName('');
    };

    const handleSaveToServer = async () => {
        if (!saveProjectName) return;
        await saveProjectToServer(saveProjectName);
        setShowSaveModal(false);
        setSaveProjectName('');
    };

    const handleLoadFromServer = async (filename: string) => {
        if (confirm(`Load project "${filename}"? Unsaved changes will be lost.`)) {
            await loadProjectFromServer(filename);
            setShowLoadModal(false);
        }
    };

    // Close dropdown menu when clicking outside
    useEffect(() => {
        const handleClickOutside = (event: MouseEvent) => {
            if (exportMenuRef.current && !exportMenuRef.current.contains(event.target as Node)) {
                setExportMenuOpen(false);
            }
        };
        document.addEventListener("mousedown", handleClickOutside);
        return () => {
            document.removeEventListener("mousedown", handleClickOutside);
        };
    }, [exportMenuRef]);


    return (
        <>
            {/* Help Modal */}
            {showHelpModal && <HelpModal onClose={() => setShowHelpModal(false)} />}

            {/* Settings Modal */}
            {showSettingsModal && (
                <div className="absolute inset-0 bg-black bg-opacity-50 flex items-center justify-center z-20">
                    <div className="bg-gray-800 p-6 rounded-lg shadow-xl w-80">
                        <h3 className="text-lg font-bold mb-4 flex items-center"><Settings className="mr-2" size={20} /> Settings</h3>

                        <div className="mb-6">
                            <label className="block text-sm text-gray-400 mb-2">Default Import Unit</label>
                            <p className="text-xs text-gray-500 mb-2">
                                When uploading STL files, they will be automatically scaled based on this unit to match the robot's meter-based system.
                            </p>
                            <div className="space-y-2">
                                <label className="flex items-center space-x-2 cursor-pointer bg-gray-900 p-2 rounded hover:bg-gray-700">
                                    <input
                                        type="radio"
                                        name="unit"
                                        checked={importUnit === 'm'}
                                        onChange={() => setImportUnit('m')}
                                        className="text-blue-500 focus:ring-blue-500"
                                    />
                                    <div>
                                        <span className="text-sm font-semibold">Meters (m)</span>
                                        <span className="text-xs text-gray-500 block">Scale: 1.0</span>
                                    </div>
                                </label>
                                <label className="flex items-center space-x-2 cursor-pointer bg-gray-900 p-2 rounded hover:bg-gray-700">
                                    <input
                                        type="radio"
                                        name="unit"
                                        checked={importUnit === 'cm'}
                                        onChange={() => setImportUnit('cm')}
                                        className="text-blue-500 focus:ring-blue-500"
                                    />
                                    <div>
                                        <span className="text-sm font-semibold">Centimeters (cm)</span>
                                        <span className="text-xs text-gray-500 block">Scale: 0.01</span>
                                    </div>
                                </label>
                                <label className="flex items-center space-x-2 cursor-pointer bg-gray-900 p-2 rounded hover:bg-gray-700">
                                    <input
                                        type="radio"
                                        name="unit"
                                        checked={importUnit === 'mm'}
                                        onChange={() => setImportUnit('mm')}
                                        className="text-blue-500 focus:ring-blue-500"
                                    />
                                    <div>
                                        <span className="text-sm font-semibold">Millimeters (mm)</span>
                                        <span className="text-xs text-gray-500 block">Scale: 0.001</span>
                                    </div>
                                </label>
                            </div>
                        </div>

                        <div className="border-t border-gray-700 my-4"></div>

                        <div className="mb-6">
                            <label className="block text-sm text-gray-400 mb-2">Collision Detection</label>
                            <p className="text-xs text-gray-500 mb-2">
                                Choose the accuracy of collision checks. "Mesh" is slower but more precise.
                            </p>
                            <div className="space-y-2">
                                <label className="flex items-center space-x-2 cursor-pointer bg-gray-900 p-2 rounded hover:bg-gray-700">
                                    <input
                                        type="radio"
                                        name="collision"
                                        checked={collisionMode === 'off'}
                                        onChange={() => setCollisionMode('off')}
                                        className="text-red-500 focus:ring-red-500"
                                    />
                                    <div>
                                        <span className="text-sm font-semibold">Off</span>
                                        <span className="text-xs text-gray-500 block">No checks (Fastest)</span>
                                    </div>
                                </label>
                                <label className="flex items-center space-x-2 cursor-pointer bg-gray-900 p-2 rounded hover:bg-gray-700">
                                    <input
                                        type="radio"
                                        name="collisionMode"
                                        value="box"
                                        checked={collisionMode === 'box'}
                                        onChange={() => setCollisionMode('box')}
                                        className="form-radio text-blue-600"
                                    />
                                    <div>
                                        <span className="text-sm font-semibold">Cylinder (Fast)</span>
                                        <span className="text-xs text-gray-500 block">Approximation, stable</span>
                                    </div>
                                </label>
                                <label className="flex items-center space-x-2 cursor-pointer bg-gray-900 p-2 rounded hover:bg-gray-700">
                                    <input
                                        type="radio"
                                        name="collisionMode"
                                        checked={collisionMode === 'mesh'}
                                        onChange={() => setCollisionMode('mesh')}
                                        className="text-green-500 focus:ring-green-500"
                                    />
                                    <div>
                                        <span className="text-sm font-semibold">Mesh (Precise)</span>
                                        <span className="text-xs text-gray-500 block">Accurate but slower</span>
                                    </div>
                                </label>
                            </div>

                            {collisionMode === 'box' && (
                                <div className="mt-4 p-2 bg-gray-900 rounded">
                                    <label className="block text-sm text-gray-400 mb-1">
                                        Cylinder Size Factor: {useRobotStore.getState().collisionBoxScale.toFixed(2)}x
                                    </label>
                                    <input
                                        type="range"
                                        min="0.1"
                                        max="1.2"
                                        step="0.05"
                                        value={useRobotStore.getState().collisionBoxScale}
                                        onChange={(e) => setCollisionBoxScale(parseFloat(e.target.value))}
                                        className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
                                    />
                                    <p className="text-xs text-gray-500 mt-1">Adjust to fit link skin.</p>
                                </div>
                            )}
                        </div>

                        <div className="flex justify-end">
                            <button onClick={() => setShowSettingsModal(false)} className="bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded text-sm text-white">
                                Done
                            </button>
                        </div>
                    </div>
                </div>
            )}

            {/* Export Modal */}
            {showExportModal && (
                <div className="absolute inset-0 bg-black bg-opacity-50 flex items-center justify-center z-20">
                    <div className="bg-gray-800 p-6 rounded-lg shadow-xl w-96">
                        <h3 className="text-lg font-bold mb-4">Export Robot Package</h3>
                        <p className="text-sm text-gray-400 mb-2">Enter a name for the robot package. This will be used for the folder and file names.</p>
                        <input
                            type="text"
                            value={robotName}
                            onChange={(e) => setRobotName(e.target.value)}
                            onFocus={e => e.target.select()}
                            className="w-full bg-gray-900 rounded p-2 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500"
                        />
                        <div className="flex justify-end space-x-2 mt-6">
                            <button
                                onClick={() => setShowExportModal(false)}
                                className="bg-gray-600 hover:bg-gray-700 p-2 px-4 rounded text-sm"
                            >
                                Cancel
                            </button>
                            <button
                                onClick={handleConfirmExport}
                                className="bg-green-600 hover:bg-green-700 p-2 px-4 rounded text-sm"
                            >
                                Export
                            </button>
                        </div>
                    </div>
                </div>
            )}

            {/* Save Project Modal */}
            {showSaveModal && (
                <div className="absolute inset-0 bg-black bg-opacity-50 flex items-center justify-center z-20">
                    <div className="bg-gray-800 p-6 rounded-lg shadow-xl w-96">
                        <h3 className="text-lg font-bold mb-4">Save Project</h3>

                        {/* Server Save */}
                        <div className="mb-6">
                            <label className="block text-sm text-gray-400 mb-1">Project Name (Server)</label>
                            <div className="flex space-x-2">
                                <input
                                    type="text"
                                    value={saveProjectName}
                                    onChange={(e) => setSaveProjectName(e.target.value)}
                                    placeholder="MyRobot"
                                    className="flex-1 bg-gray-900 rounded p-2 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500"
                                />
                                <button onClick={handleSaveToServer} className="bg-indigo-600 hover:bg-indigo-700 p-2 rounded text-sm whitespace-nowrap">
                                    Save to Server
                                </button>
                            </div>
                        </div>

                        <div className="border-t border-gray-700 my-4"></div>

                        {/* Local Save */}
                        <div className="mb-2">
                            <p className="text-sm text-gray-400 mb-2">Or save to your computer:</p>
                            <button onClick={() => { saveRobot(); setShowSaveModal(false); }} className="w-full bg-gray-700 hover:bg-gray-600 p-2 rounded text-sm flex items-center justify-center">
                                <Save className="mr-2 h-4 w-4" /> Download Local Zip
                            </button>
                        </div>

                        <div className="flex justify-end mt-4">
                            <button onClick={() => setShowSaveModal(false)} className="text-sm text-gray-400 hover:text-white">Cancel</button>
                        </div>
                    </div>
                </div>
            )}

            {/* Load Project Modal */}
            {showLoadModal && (
                <div className="absolute inset-0 bg-black bg-opacity-50 flex items-center justify-center z-20">
                    <div className="bg-gray-800 p-6 rounded-lg shadow-xl w-96 flex flex-col max-h-[80vh]">
                        <h3 className="text-lg font-bold mb-4">Load Project</h3>

                        {/* Server List */}
                        <div className="flex-1 overflow-y-auto min-h-[150px] mb-4 bg-gray-900/50 rounded p-2">
                            <h4 className="text-xs font-semibold text-gray-400 mb-2 uppercase">Server Projects</h4>
                            {serverProjects.length === 0 ? (
                                <p className="text-gray-500 text-sm italic">No projects found on server.</p>
                            ) : (
                                <ul className="space-y-1">
                                    {serverProjects.map(proj => (
                                        <li key={proj} className="flex justify-between items-center bg-gray-800 hover:bg-gray-700 p-2 rounded cursor-pointer group">
                                            <span className="text-sm truncate mr-2">{proj}</span>
                                            <button onClick={() => handleLoadFromServer(proj)} className="text-xs bg-indigo-600 hover:bg-indigo-500 px-2 py-1 rounded opacity-0 group-hover:opacity-100 transition-opacity">
                                                Load
                                            </button>
                                        </li>
                                    ))}
                                </ul>
                            )}
                        </div>

                        <div className="border-t border-gray-700 my-4"></div>

                        {/* Local Load */}
                        <div>
                            <p className="text-sm text-gray-400 mb-2">Or load from your computer:</p>
                            <button onClick={() => { handleLoadClick(); setShowLoadModal(false); }} className="w-full bg-gray-700 hover:bg-gray-600 p-2 rounded text-sm flex items-center justify-center">
                                <FolderOpen className="mr-2 h-4 w-4" /> Browse Local File...
                            </button>
                        </div>

                        <div className="flex justify-end mt-4">
                            <button onClick={() => setShowLoadModal(false)} className="text-sm text-gray-400 hover:text-white">Cancel</button>
                        </div>
                    </div>
                </div>
            )}

            <div className="absolute top-0 right-0 h-screen w-80 bg-gray-800 bg-opacity-80 backdrop-blur-sm text-white p-4 border-l border-gray-700 overflow-y-auto">
                {/* File Operations */}
                <div className="pb-4 mb-4 border-b border-gray-700">
                    <h2 className="text-xl font-bold">Robot Link Forge</h2>
                    <div className="flex flex-wrap gap-2 mt-2">
                        <button
                            onClick={() => { setSaveProjectName('MyRobot'); setShowSaveModal(true); }}
                            className="flex-1 min-w-[30%] flex items-center justify-center bg-indigo-600 hover:bg-indigo-700 p-2 rounded text-sm"
                        >
                            <Save className="mr-2 h-4 w-4" /> Save
                        </button>

                        <button
                            onClick={() => { getProjectList(); setShowLoadModal(true); }}
                            className="flex-1 min-w-[30%] flex items-center justify-center bg-gray-600 hover:bg-gray-700 p-2 rounded text-sm"
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
                        <div className="relative flex-1 min-w-[30%]" ref={exportMenuRef}>
                            <button
                                onClick={() => setExportMenuOpen(!isExportMenuOpen)}
                                className="w-full flex items-center justify-center bg-green-600 hover:bg-green-700 p-2 rounded text-sm"
                            >
                                <Upload className="mr-2 h-4 w-4" /> Export
                            </button>
                            {isExportMenuOpen && (
                                <div className="absolute right-0 mt-2 w-48 bg-gray-700 rounded-md shadow-lg z-10">
                                    <a href="#" onClick={(e) => { e.preventDefault(); openExportModal('urdf'); }} className="block px-4 py-2 text-sm text-white hover:bg-gray-600">Export as URDF (ROS1)</a>
                                    <a href="#" onClick={(e) => { e.preventDefault(); openExportModal('urdf_ros2'); }} className="block px-4 py-2 text-sm text-white hover:bg-gray-600">Export as URDF (ROS2)</a>
                                </div>
                            )}
                        </div>
                        <button
                            onClick={resetProject}
                            className="flex-none flex items-center justify-center bg-red-800 hover:bg-red-700 p-2 rounded text-sm"
                            title="New Project (Reset)"
                        >
                            <FilePlus className="h-4 w-4" />
                        </button>

                        <button
                            onClick={() => setShowHelpModal(true)}
                            className="flex-none flex items-center justify-center bg-blue-600 hover:bg-blue-700 p-2 rounded text-sm"
                            title="User Guide / Help"
                        >
                            <HelpCircle className="h-4 w-4" />
                        </button>

                        <button
                            onClick={() => setShowSettingsModal(true)}
                            className="flex-none flex items-center justify-center bg-gray-600 hover:bg-gray-700 p-2 rounded text-sm"
                            title="Global Settings"
                        >
                            <Settings className="h-4 w-4" />
                        </button>
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
        </>
    );
};

export default Sidebar;