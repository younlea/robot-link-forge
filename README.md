# RobotLinkForge

A web-based URDF builder bridging the gap between Mechanical Design and Robot Simulation.

## 🌟 Core Features (Step 1 Implemented)

*   **Interactive 3D Viewer**: Visualize your robot model in real-time using React Three Fiber.
*   **Recursive Tree Structure**: Supports complex, multi-branching robot designs (e.g., hands with multiple fingers).
*   **Dynamic Joint & Link Creation**: Add new joints and links to your robot on the fly.
*   **Real-time Inspector Panel**:
    *   Select and modify Joint properties (origin, axis, angle).
    *   Select and modify Link properties (visuals, dimensions).
    *   Control joint angles with sliders for quick testing.
*   **Transform Controls**: Pan, zoom, and rotate the camera for a better view.
*   **Component-based Architecture**: Clean and maintainable code with React, Zustand, and TypeScript.

## 🚀 Getting Started

1.  Navigate to the `src/frontend` directory.
2.  Install dependencies: `npm install`
3.  Run the development server: `npm run dev`
4.  Open your browser to `http://localhost:5173`.