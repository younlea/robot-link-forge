import React, { useState } from 'react';
import { X, ArrowLeft } from 'lucide-react';

interface HelpModalProps {
    onClose: () => void;
}

type HelpPage = 'main' | 'tendon-detail';

const HelpModal: React.FC<HelpModalProps> = ({ onClose }) => {
    const [currentPage, setCurrentPage] = useState<HelpPage>('main');

    return (
        <div className="absolute inset-0 bg-black bg-opacity-70 flex items-center justify-center z-50 p-8">
            <div className="bg-gray-800 rounded-lg shadow-2xl w-full max-w-4xl h-full max-h-[90vh] flex flex-col relative animate-fade-in-up">
                <button onClick={onClose} className="absolute top-4 right-4 text-gray-400 hover:text-white z-10">
                    <X className="w-6 h-6" />
                </button>

                {currentPage !== 'main' && (
                    <button 
                        onClick={() => setCurrentPage('main')} 
                        className="absolute top-4 left-4 text-gray-400 hover:text-white flex items-center space-x-1 z-10"
                    >
                        <ArrowLeft className="w-5 h-5" />
                        <span className="text-sm">Back</span>
                    </button>
                )}

                <div className="p-6 border-b border-gray-700">
                    <h2 className="text-2xl font-bold text-white">
                        {currentPage === 'main' ? 'Robot Link Forge - User Guide' : '🧵 Tendon System - Detailed Guide'}
                    </h2>
                </div>

                <div className="overflow-y-auto p-8 text-gray-300 space-y-8">
                    {currentPage === 'main' && <MainGuide setPage={setCurrentPage} />}
                    {currentPage === 'tendon-detail' && <TendonDetailGuide />}
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

// Main Guide Component
const MainGuide: React.FC<{ setPage: (page: HelpPage) => void }> = ({ setPage }) => {
    return (
        <>
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
                        <button 
                            onClick={() => setPage('tendon-detail')}
                            className="mt-3 text-orange-400 hover:text-orange-300 text-sm font-semibold flex items-center space-x-1"
                        >
                            <span>→ View Step-by-Step Guide</span>
                        </button>
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
                    <div className="bg-gray-900 p-4 rounded">
                        <h4 className="font-bold text-red-400 mb-2">🔴 Recording Modes</h4>
                        <ul className="space-y-2 text-sm text-gray-400">
                            <li><strong>Slider Mode:</strong> Manually move joints using the sidebar sliders. Keyframes are saved automatically.</li>
                            <li><strong>Camera Mode:</strong> Use Hand Control to act out movements. Press "Capture Keyframe" (or Spacebar) to save poses.</li>
                        </ul>
                    </div>

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
        </>
    );
};

// Tendon Detail Guide Component
const TendonDetailGuide: React.FC = () => {
    return (
        <>
            <section>
                <div className="bg-orange-900/20 border border-orange-600 rounded-lg p-4 mb-6">
                    <p className="text-orange-300 text-sm">
                        💡 <strong>What you'll learn:</strong> How to add tendon-driven mechanisms to your robot using 3D click-based routing.
                        Perfect for robotic hands, fingers, and underactuated systems!
                    </p>
                </div>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">Step 1: Add a Tendon</h3>
                <p className="text-gray-300 mb-3">
                    Scroll down the sidebar to find the <strong className="text-orange-400">🧵 Tendon System</strong> section. Click to expand it.
                </p>
                <div className="bg-gray-900 p-4 rounded border-l-4 border-orange-500 mb-3">
                    <p className="text-sm text-gray-300 mb-2">You'll see two buttons:</p>
                    <div className="space-y-2">
                        <div className="flex items-center space-x-3">
                            <div className="bg-orange-600 text-white px-3 py-1 rounded text-xs font-semibold">+ Active Tendon</div>
                            <span className="text-xs text-gray-400">🟠 Motor-driven (e.g., close fingers)</span>
                        </div>
                        <div className="flex items-center space-x-3">
                            <div className="bg-blue-600 text-white px-3 py-1 rounded text-xs font-semibold">+ Passive Tendon</div>
                            <span className="text-xs text-gray-400">🔵 Spring-based (e.g., return force)</span>
                        </div>
                    </div>
                </div>
                <p className="text-sm text-gray-400">
                    Click the button for the type you want. A tendon item (e.g., <span className="font-mono text-orange-400">tendon_1</span>) will appear in the list.
                </p>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">Step 2: Enter Routing Mode</h3>
                <p className="text-gray-300 mb-3">
                    Click on your newly created tendon to expand it. You'll see a section called <strong>Routing Points</strong>.
                </p>
                <div className="bg-gray-900 p-4 rounded mb-3">
                    <div className="flex items-center justify-between mb-2">
                        <span className="text-gray-400 font-semibold text-sm">Routing Points (0)</span>
                        <button className="bg-orange-600 hover:bg-orange-700 px-3 py-1 rounded text-xs text-white font-semibold">
                            + Add Points
                        </button>
                    </div>
                    <p className="text-xs text-gray-500">Click "Add Points" then click on link surfaces...</p>
                </div>
                <div className="space-y-2">
                    <p className="text-sm text-gray-300">
                        ✨ Click the <strong className="text-orange-400">[+ Add Points]</strong> button!
                    </p>
                    <div className="bg-orange-900/30 border border-orange-600 rounded p-3">
                        <p className="text-xs text-orange-300 font-semibold mb-1">🎯 Routing Mode Active</p>
                        <ul className="text-xs text-gray-400 space-y-1">
                            <li>• Orange banner appears at the top of the screen</li>
                            <li>• Your mouse cursor changes to a <strong className="text-white">crosshair (+)</strong></li>
                            <li>• Button changes to <strong className="text-green-400">[✓ Done]</strong></li>
                        </ul>
                    </div>
                </div>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">Step 3: Click on Link Surfaces</h3>
                <p className="text-gray-300 mb-3">
                    Now <strong>click on the 3D robot model</strong> where you want the tendon to pass through. The tendon will follow the order you click.
                </p>
                <div className="bg-gray-900 p-4 rounded border-l-4 border-blue-500 mb-3">
                    <h4 className="font-bold text-blue-400 mb-2">Example: Finger Tendon</h4>
                    <div className="space-y-2 text-sm">
                        <div className="flex items-start space-x-3">
                            <span className="text-orange-400 font-bold">1.</span>
                            <div>
                                <p className="text-gray-300">Click on <strong>palm</strong> link</p>
                                <p className="text-xs text-gray-500">→ First routing point created ✓</p>
                            </div>
                        </div>
                        <div className="flex items-start space-x-3">
                            <span className="text-orange-400 font-bold">2.</span>
                            <div>
                                <p className="text-gray-300">Click on <strong>first finger joint</strong> (proximal)</p>
                                <p className="text-xs text-gray-500">→ Second routing point ✓</p>
                            </div>
                        </div>
                        <div className="flex items-start space-x-3">
                            <span className="text-orange-400 font-bold">3.</span>
                            <div>
                                <p className="text-gray-300">Click on <strong>middle finger joint</strong></p>
                                <p className="text-xs text-gray-500">→ Third routing point ✓</p>
                            </div>
                        </div>
                        <div className="flex items-start space-x-3">
                            <span className="text-orange-400 font-bold">4.</span>
                            <div>
                                <p className="text-gray-300">Click on <strong>fingertip</strong> (distal)</p>
                                <p className="text-xs text-gray-500">→ Fourth routing point ✓</p>
                            </div>
                        </div>
                    </div>
                </div>
                <div className="bg-yellow-900/20 border border-yellow-600 rounded p-3">
                    <p className="text-yellow-300 text-sm font-semibold mb-1">⚠️ Important Notes:</p>
                    <ul className="text-xs text-gray-400 space-y-1 ml-4">
                        <li>• Only <strong>links</strong> are clickable (not joints)</li>
                        <li>• Each click adds a small 🟠 orange dot</li>
                        <li>• <strong>Order matters!</strong> Click from start to end</li>
                        <li>• Need at least <strong>2 points</strong> to see the tendon line</li>
                    </ul>
                </div>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">Step 4: Finish Routing</h3>
                <p className="text-gray-300 mb-3">
                    When you've added all the points you need, exit routing mode:
                </p>
                <div className="space-y-2 text-sm text-gray-400">
                    <p>• Click <strong className="text-green-400">[✓ Done Routing]</strong> in the orange banner at the top</p>
                    <p>• OR click <strong className="text-green-400">[✓ Done]</strong> in the tendon panel</p>
                </div>
                <div className="bg-gray-900 p-4 rounded mt-3">
                    <p className="text-sm text-gray-300 mb-2">✨ <strong>Visual Result:</strong></p>
                    <ul className="text-xs text-gray-400 space-y-1 ml-4">
                        <li>• Cursor returns to normal arrow</li>
                        <li>• <strong className="text-orange-400">Active Tendon:</strong> Thick orange solid line 🟠</li>
                        <li>• <strong className="text-blue-400">Passive Tendon:</strong> Blue dashed line - - - 🔵</li>
                        <li>• Small spheres mark each routing point</li>
                    </ul>
                </div>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">Step 5: Configure Physics Parameters</h3>
                <p className="text-gray-300 mb-3">
                    Expand the tendon item to adjust its physical properties:
                </p>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div className="bg-gray-900 p-4 rounded border-l-4 border-orange-500">
                        <h4 className="font-bold text-orange-400 mb-2 text-sm">🟠 Active Tendon (Motor)</h4>
                        <table className="w-full text-xs">
                            <tbody>
                                <tr className="border-b border-gray-800">
                                    <td className="py-1 text-gray-400">Stiffness</td>
                                    <td className="py-1 text-white font-mono">0 N/m</td>
                                    <td className="py-1 text-gray-500 text-[10px]">Keep 0</td>
                                </tr>
                                <tr className="border-b border-gray-800">
                                    <td className="py-1 text-gray-400">Damping</td>
                                    <td className="py-1 text-white font-mono">0.1 N·s/m</td>
                                    <td className="py-1 text-gray-500 text-[10px]">Low damping</td>
                                </tr>
                                <tr className="border-b border-gray-800">
                                    <td className="py-1 text-gray-400">Rest Length</td>
                                    <td className="py-1 text-white font-mono">0.1 m</td>
                                    <td className="py-1 text-gray-500 text-[10px]">Natural length</td>
                                </tr>
                                <tr>
                                    <td className="py-1 text-gray-400">Moment Arm</td>
                                    <td className="py-1 text-orange-400 font-mono font-bold">0.01 m</td>
                                    <td className="py-1 text-orange-300 text-[10px] font-semibold">Important!</td>
                                </tr>
                            </tbody>
                        </table>
                        <p className="text-[10px] text-gray-500 mt-2">Moment arm converts motor torque → tendon tension</p>
                    </div>

                    <div className="bg-gray-900 p-4 rounded border-l-4 border-blue-500">
                        <h4 className="font-bold text-blue-400 mb-2 text-sm">🔵 Passive Tendon (Spring)</h4>
                        <table className="w-full text-xs">
                            <tbody>
                                <tr className="border-b border-gray-800">
                                    <td className="py-1 text-gray-400">Stiffness</td>
                                    <td className="py-1 text-blue-400 font-mono font-bold">50 N/m</td>
                                    <td className="py-1 text-blue-300 text-[10px] font-semibold">Spring strength!</td>
                                </tr>
                                <tr className="border-b border-gray-800">
                                    <td className="py-1 text-gray-400">Damping</td>
                                    <td className="py-1 text-white font-mono">0.1 N·s/m</td>
                                    <td className="py-1 text-gray-500 text-[10px]">Low damping</td>
                                </tr>
                                <tr>
                                    <td className="py-1 text-gray-400">Rest Length</td>
                                    <td className="py-1 text-white font-mono">0.1 m</td>
                                    <td className="py-1 text-gray-500 text-[10px]">Natural length</td>
                                </tr>
                            </tbody>
                        </table>
                        <p className="text-[10px] text-gray-500 mt-2">Higher stiffness = stronger return force</p>
                    </div>
                </div>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">Step 6: Review & Edit Points</h3>
                <p className="text-gray-300 mb-3">
                    The <strong>Routing Points</strong> section shows all added points:
                </p>
                <div className="bg-gray-900 p-3 rounded font-mono text-xs space-y-2">
                    <div className="flex items-center justify-between bg-gray-800 p-2 rounded">
                        <div>
                            <span className="text-gray-500">1.</span>
                            <span className="text-gray-300 ml-2">palm</span>
                            <span className="text-gray-600 ml-2 text-[10px]">[0.010, 0.020, 0.030]</span>
                        </div>
                        <button className="text-red-400 hover:text-red-300">✕</button>
                    </div>
                    <div className="flex items-center justify-between bg-gray-800 p-2 rounded">
                        <div>
                            <span className="text-gray-500">2.</span>
                            <span className="text-gray-300 ml-2">finger_proximal</span>
                            <span className="text-gray-600 ml-2 text-[10px]">[0.015, 0.025, 0.080]</span>
                        </div>
                        <button className="text-red-400 hover:text-red-300">✕</button>
                    </div>
                    <p className="text-gray-600 text-center">...</p>
                </div>
                <p className="text-sm text-gray-400 mt-2">
                    💡 Made a mistake? Click the <strong className="text-red-400">✕</strong> button to delete individual points, then add them again.
                </p>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">Step 7: Export to MuJoCo</h3>
                <p className="text-gray-300 mb-3">
                    Once your tendons are configured, export your robot:
                </p>
                <ol className="list-decimal ml-5 space-y-2 text-sm text-gray-400">
                    <li>Click <strong className="text-white">Export</strong> button in the sidebar</li>
                    <li>Select <strong className="text-purple-400">MJCF (MuJoCo)</strong></li>
                    <li>Enter a robot name</li>
                    <li>Click <strong className="text-white">Export</strong></li>
                </ol>
                <div className="bg-gray-900 p-4 rounded mt-3 border-l-4 border-purple-500">
                    <h4 className="font-bold text-purple-400 mb-2 text-sm">Generated MJCF XML includes:</h4>
                    <ul className="text-xs text-gray-400 space-y-1 ml-4">
                        <li>• <code className="text-orange-400">&lt;site&gt;</code> elements for each routing point</li>
                        <li>• <code className="text-orange-400">&lt;tendon&gt;&lt;spatial&gt;</code> section with all tendons</li>
                        <li>• <code className="text-orange-400">&lt;actuator&gt;&lt;general&gt;</code> for active tendons</li>
                        <li>• Full physics parameters (stiffness, damping, ranges)</li>
                    </ul>
                </div>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">💡 Pro Tips</h3>
                <div className="space-y-3">
                    <div className="bg-blue-900/20 border border-blue-600 rounded p-3">
                        <p className="text-blue-300 font-semibold text-sm mb-1">Tip 1: Can't see the tendon line?</p>
                        <ul className="text-xs text-gray-400 ml-4 space-y-0.5">
                            <li>• Need at least <strong>2 routing points</strong></li>
                            <li>• Try changing the color (Color picker in tendon panel)</li>
                            <li>• Points too close together = line appears tiny</li>
                        </ul>
                    </div>

                    <div className="bg-green-900/20 border border-green-600 rounded p-3">
                        <p className="text-green-300 font-semibold text-sm mb-1">Tip 2: Multiple tendons</p>
                        <ul className="text-xs text-gray-400 ml-4 space-y-0.5">
                            <li>• Add one tendon per finger</li>
                            <li>• Mix Active (close) + Passive (open) for realistic behavior</li>
                            <li>• Example: 4 fingers = 4 Active + 4 Passive tendons</li>
                        </ul>
                    </div>

                    <div className="bg-yellow-900/20 border border-yellow-600 rounded p-3">
                        <p className="text-yellow-300 font-semibold text-sm mb-1">Tip 3: Order is critical!</p>
                        <ul className="text-xs text-gray-400 ml-4 space-y-0.5">
                            <li>• Wrong order = tendon goes through the robot body</li>
                            <li>• Plan the path before clicking</li>
                            <li>• Start from base (palm) → end at tip (fingertip)</li>
                        </ul>
                    </div>

                    <div className="bg-red-900/20 border border-red-600 rounded p-3">
                        <p className="text-red-300 font-semibold text-sm mb-1">Tip 4: Crosshair won't go away?</p>
                        <ul className="text-xs text-gray-400 ml-4 space-y-0.5">
                            <li>• You're still in routing mode</li>
                            <li>• Click <strong className="text-green-400">[✓ Done]</strong> button</li>
                            <li>• Look for the orange banner at the top</li>
                        </ul>
                    </div>
                </div>
            </section>

            <section>
                <h3 className="text-xl font-semibold text-white mb-3">🔍 Troubleshooting</h3>
                <table className="w-full text-sm border border-gray-700 rounded">
                    <thead className="bg-gray-800">
                        <tr>
                            <th className="py-2 px-3 text-left text-gray-400 border-b border-gray-700">Problem</th>
                            <th className="py-2 px-3 text-left text-gray-400 border-b border-gray-700">Solution</th>
                        </tr>
                    </thead>
                    <tbody className="text-xs">
                        <tr className="border-b border-gray-800">
                            <td className="py-2 px-3 text-gray-300">Clicks don't create points</td>
                            <td className="py-2 px-3 text-gray-400">Routing mode is OFF. Click [+ Add Points] again</td>
                        </tr>
                        <tr className="border-b border-gray-800">
                            <td className="py-2 px-3 text-gray-300">Clicking empty space does nothing</td>
                            <td className="py-2 px-3 text-gray-400">Must click on link surfaces (colored parts)</td>
                        </tr>
                        <tr className="border-b border-gray-800">
                            <td className="py-2 px-3 text-gray-300">Only 1 point, no line visible</td>
                            <td className="py-2 px-3 text-gray-400">Need minimum 2 points to draw a line</td>
                        </tr>
                        <tr>
                            <td className="py-2 px-3 text-gray-300">Tendon goes through robot body</td>
                            <td className="py-2 px-3 text-gray-400">Wrong point order. Delete points and re-add in correct sequence</td>
                        </tr>
                    </tbody>
                </table>
            </section>

            <section>
                <div className="bg-gradient-to-r from-orange-900/40 to-purple-900/40 border border-orange-600 rounded-lg p-6 text-center">
                    <h4 className="text-xl font-bold text-white mb-2">🎉 You're Ready!</h4>
                    <p className="text-gray-300 text-sm">
                        You now know how to create tendon-driven mechanisms. Try building a robotic hand and see the tendons in action in MuJoCo!
                    </p>
                </div>
            </section>
        </>
    );
};

export default HelpModal;
