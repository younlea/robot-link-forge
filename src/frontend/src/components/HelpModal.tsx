import React from 'react';
import { X } from 'lucide-react';

interface HelpModalProps {
    onClose: () => void;
}

const HelpModal: React.FC<HelpModalProps> = ({ onClose }) => {
    return (
        <div className="absolute inset-0 bg-black bg-opacity-70 flex items-center justify-center z-50 p-8">
            <div className="bg-gray-800 rounded-lg shadow-2xl w-full max-w-4xl h-full max-h-[90vh] flex flex-col relative animate-fade-in-up">
                <button onClick={onClose} className="absolute top-4 right-4 text-gray-400 hover:text-white">
                    <X className="w-6 h-6" />
                </button>

                <div className="p-6 border-b border-gray-700">
                    <h2 className="text-2xl font-bold text-white">Robot Link Forge - User Guide</h2>
                </div>

                <div className="overflow-y-auto p-8 text-gray-300 space-y-8">
                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">1. Getting Started</h3>
                        <p>
                            <strong>Adding Links & Joints:</strong> Select an existing Link (click directly on the 3D model). The Sidebar will show its properties.
                            Click the <span className="text-green-400 font-mono text-sm">Add Child Joint</span> button to extend the robot chain. A new Joint (and a default visual Link attached to it) will appear.
                        </p>
                        <ul className="list-disc ml-5 mt-2 space-y-1 text-sm text-gray-400">
                            <li><strong>Fixed Joint:</strong> Does not move. Good for glueing parts together.</li>
                            <li><strong>Revolute Joint:</strong> Rotates around an axis (Roll/Pitch/Yaw).</li>
                            <li><strong>Prismatic Joint:</strong> Slides along an axis (Displacement).</li>
                            <li><strong>Rolling Contact Joint:</strong> Models rolling surfaces with curvature radius and contact friction (convex/concave).</li>
                        </ul>
                    </section>

                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">2. Visual Customization</h3>
                        <p>
                            <strong>Link Visuals:</strong> Select a Link to change its shape (Box, Cylinder, Sphere) or color.
                        </p>
                        <p className="mt-2">
                            <strong>Uploading Meshes:</strong> You can upload <strong>STL files</strong> for both Links and Joints.
                            <ul className="list-disc ml-5 mt-1 text-sm text-gray-400">
                                <li>For <strong>Links</strong>: Upload the body/structure mesh. Use "Fit Length to Content" to auto-scale.</li>
                                <li>For <strong>Joints</strong>: Upload meshes for motors, servos, or housing.</li>
                            </ul>
                        </p>
                    </section>

                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">3. Advanced Simulation Features</h3>
                        
                        <div className="space-y-4">
                            <div className="bg-gray-900 p-4 rounded border-l-4 border-orange-500">
                                <h4 className="font-bold text-orange-400 mb-2">🧵 Tendon System</h4>
                                <p className="text-sm text-gray-300 mb-2">
                                    Model underactuated mechanisms with tendon-driven systems (like robotic hands).
                                </p>
                                <ul className="list-disc ml-5 space-y-1 text-xs text-gray-400">
                                    <li><strong>Active Tendons:</strong> Motor-driven, pulls on joints via moment arm.</li>
                                    <li><strong>Passive Tendons:</strong> Spring-based, adds compliance to the structure.</li>
                                    <li><strong>3D Routing:</strong> Click "Add Points" in tendon editor, then click on link surfaces to define the path.</li>
                                    <li>Configure stiffness, damping, rest length, and moment arm parameters.</li>
                                </ul>
                            </div>

                            <div className="bg-gray-900 p-4 rounded border-l-4 border-red-500">
                                <h4 className="font-bold text-red-400 mb-2">🪨 Obstacles</h4>
                                <p className="text-sm text-gray-300 mb-2">
                                    Add fixed-position obstacles (Box, Sphere, Cylinder) for contact simulation testing.
                                </p>
                                <ul className="list-disc ml-5 space-y-1 text-xs text-gray-400">
                                    <li>Adjust position, rotation, dimensions, and physics parameters (friction, solref, solimp).</li>
                                    <li>Toggle enabled/disabled without deleting.</li>
                                    <li>Useful for testing finger contact resistance and grasp planning.</li>
                                </ul>
                            </div>

                            <div className="bg-gray-900 p-4 rounded border-l-4 border-green-500">
                                <h4 className="font-bold text-green-400 mb-2">📡 Sensor Placement</h4>
                                <p className="text-sm text-gray-300 mb-2">
                                    Place touch and force sensors on link surfaces with precise 3D click positioning.
                                </p>
                                <ul className="list-disc ml-5 space-y-1 text-xs text-gray-400">
                                    <li>Click "Place Touch Sensor" button to enter placement mode.</li>
                                    <li>Click on any link surface in the 3D view to add a sensor at that exact location.</li>
                                    <li>Sensors export to MuJoCo with link-local coordinates for accurate simulation.</li>
                                </ul>
                            </div>
                        </div>
                    </section>

                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">4. Managing Projects</h3>
                        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mt-2">
                            <div className="bg-gray-900 p-4 rounded">
                                <h4 className="font-bold text-indigo-400 mb-1">Saving</h4>
                                <p className="text-sm">
                                    Click <strong>Save</strong>. You can name your project and save it to the <strong>Server</strong> (accessible from any PC) or download a <strong>Local Zip</strong> backup.
                                </p>
                            </div>
                            <div className="bg-gray-900 p-4 rounded">
                                <h4 className="font-bold text-gray-400 mb-1">Loading</h4>
                                <p className="text-sm">
                                    Click <strong>Load</strong> to see projects saved on the server. You can also drag-and-drop or browse for a local `.zip` or `.json` file.
                                </p>
                            </div>
                        </div>
                    </section>
                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">5. Collision Detection</h3>
                        <p className="mb-2">
                            Use the <strong>Settings</strong> button (Gear icon, bottom-left) to enable collision detection.
                        </p>
                        <div className="grid grid-cols-1 gap-4 mt-2">
                            <div className="bg-gray-900 p-4 rounded border-l-4 border-yellow-500">
                                <h4 className="font-bold text-yellow-500 mb-1">Cylinder (Fast) Mode</h4>
                                <p className="text-sm text-gray-300">
                                    Recommended. Wraps every part in a <strong>Proxy Cylinder</strong> that rotates with the part.
                                    <br />
                                    Use the slider to adjust the cylinder size (0.1x - 1.2x) to match your robot's skin tight.
                                    collisions turn the parts <span className="text-red-500 font-bold">RED</span>.
                                </p>
                            </div>
                            <div className="bg-gray-900 p-4 rounded border-l-4 border-green-500">
                                <h4 className="font-bold text-green-500 mb-1">Mesh (Precise) Mode</h4>
                                <p className="text-sm text-gray-300">
                                    Uses the exact shape of your uploaded STL files. Computationally expensive. Use only for final checks.
                                </p>
                            </div>
                        </div>
                    </section>

                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">6. Webcam Hand Control 🎬</h3>
                        <div className="bg-gray-900 p-4 rounded border-l-4 border-purple-500">
                            <div className="flex items-start gap-4">
                                <div className="text-3xl">🖐️</div>
                                <div>
                                    <h4 className="font-bold text-white mb-2">Control your Robot with AI!</h4>
                                    <p className="text-sm text-gray-300 mb-2">
                                        Click the <span className="text-purple-400 font-bold">Lego Block Icon</span> in the sidebar to open the Hand Control panel.
                                    </p>
                                    <ol className="list-decimal ml-5 space-y-1 text-sm text-gray-400">
                                        <li><strong>Camera Access:</strong> Allow browser permission to use your webcam.</li>
                                        <li><strong>Select a Joint:</strong> Click any joint on your 3D robot model.</li>
                                        <li><strong>Move your Finger:</strong> Show your hand to the camera. Move your <strong>Index Finger tip</strong> left/right or up/down.</li>
                                        <li>The selected joint will mimic your finger's movement in real-time! 🤖</li>
                                    </ol>
                                </div>
                            </div>
                        </div>
                    </section>

                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">7. Motion Recording & Timeline 🎞️</h3>
                        <p className="mb-4 text-gray-300">
                            Create complex animations by recording sequences and fine-tuning them in the timeline editor.
                        </p>

                        <div className="space-y-4">
                            {/* Recording Modes */}
                            <div className="bg-gray-900 p-4 rounded">
                                <h4 className="font-bold text-red-400 mb-2">🔴 Recording Modes</h4>
                                <ul className="space-y-2 text-sm text-gray-400">
                                    <li><strong>Slider Mode:</strong> Manually move joints using the sidebar sliders. Keyframes are saved automatically.</li>
                                    <li><strong>Camera Mode:</strong> Use Hand Control to act out movements. Press "Capture Keyframe" (or Spacebar) to save poses.</li>
                                </ul>
                            </div>

                            {/* Timeline Editor */}
                            <div className="bg-gray-900 p-4 rounded border-l-4 border-blue-500">
                                <h4 className="font-bold text-blue-400 mb-2">⏱️ Timeline Editor</h4>
                                <p className="text-sm text-gray-300 mb-2">
                                    The timeline appears at the bottom when you edit a recording.
                                </p>
                                <ul className="list-disc ml-5 space-y-1 text-sm text-gray-400">
                                    <li><strong>Zoom & Pan:</strong> Use <span className="text-white">Mouse Wheel</span> to zoom. <span className="text-white">Click & Drag</span> the background to pan left/right.</li>
                                    <li><strong>Edit Timing:</strong> Drag the <span className="text-blue-400">Blue Dots</span> to change when a move happens.</li>
                                    <li><strong>Adjust Speed:</strong> Drag the <span className="text-blue-600">Horizontal Bars</span> UP to make the move faster, or DOWN to make it slower.</li>
                                    <li><strong>Real-time Preview:</strong> Scrubbing the timeline instantly updates the robot's pose!</li>
                                </ul>
                            </div>

                            {/* Storage */}
                            <div className="bg-gray-900 p-4 rounded">
                                <h4 className="font-bold text-green-400 mb-2">💾 Saving Your Moves</h4>
                                <p className="text-sm text-gray-400">
                                    Click the <span className="text-white font-bold">Storage Icon (Hard Drive)</span> in the recording panel header.
                                    You can save your best dances to the <strong>Server cloud</strong> or download them as <strong>JSON files</strong> to your computer.
                                </p>
                            </div>
                        </div>
                    </section>
                    <section>
                        <h3 className="text-xl font-semibold text-white mb-3">8. Exporting to ROS & MuJoCo</h3>
                        <p className="mb-2">
                            Click <strong>Export</strong> to generate simulation-ready packages.
                        </p>
                        <table className="w-full text-left text-sm">
                            <thead>
                                <tr className="border-b border-gray-700">
                                    <th className="py-2 text-gray-400">Format</th>
                                    <th className="py-2 text-gray-400">Description</th>
                                </tr>
                            </thead>
                            <tbody>
                                <tr className="border-b border-gray-800">
                                    <td className="py-2 font-mono text-yellow-500">URDF (ROS1)</td>
                                    <td className="py-2">Standard URDF file with relative mesh paths. Good for classic ROS / Gazebo.</td>
                                </tr>
                                <tr className="border-b border-gray-800">
                                    <td className="py-2 font-mono text-orange-500">URDF (ROS2)</td>
                                    <td className="py-2">Includes `package.xml`, `CMakeLists.txt`, and Python launch files. Ready to `colcon build`.</td>
                                </tr>
                                <tr>
                                    <td className="py-2 font-mono text-purple-500">MJCF (MuJoCo)</td>
                                    <td className="py-2">Full MuJoCo XML with tendons, obstacles, sensors, rolling joints, and motor validation tools.</td>
                                </tr>
                            </tbody>
                        </table>
                    </section>
                </div>

                <div className="p-4 border-t border-gray-700 bg-gray-900 rounded-b-lg flex justify-end">
                    <button onClick={onClose} className="bg-indigo-600 hover:bg-indigo-700 text-white px-6 py-2 rounded font-semibold">
                        Close Guide
                    </button>
                </div>
            </div>
        </div >
    );
};

export default HelpModal;
