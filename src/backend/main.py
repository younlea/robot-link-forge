from fastapi import FastAPI, File, UploadFile, HTTPException, Form, BackgroundTasks, Request
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from typing import Dict, List, Optional, Tuple, Any
import os
import uuid
import shutil
import json
import tempfile
import re
import math
from datetime import datetime

# --- Refactored Modules ---
from .robot_models import RobotData, RobotLink, RobotJoint, Visual
from .utils import to_snake_case, generate_unique_names
from .exporters.urdf_exporter import generate_urdf_xml
from .exporters.mjcf_exporter import generate_mjcf_xml
from .exporters.stl_utils import ensure_binary_stl

app = FastAPI()

# Allow all origins for development purposes
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Static File Serving for Meshes ---
MESH_DIR = "static/meshes"
os.makedirs(MESH_DIR, exist_ok=True)

# --- Project Storage ---
PROJECTS_DIR = "saved_projects"
os.makedirs(PROJECTS_DIR, exist_ok=True)

app.mount("/static", StaticFiles(directory="static"), name="static")


@app.post("/api/export-urdf")
async def export_urdf_package(
    background_tasks: BackgroundTasks,
    robot_data: str = Form(...), 
    robot_name: str = Form(...), 
    files: Optional[List[UploadFile]] = File(None)
):
    try:
        data_dict = json.loads(robot_data)
        robot = RobotData.parse_obj(data_dict)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid robot data format: {e}")

    sanitized_robot_name = to_snake_case(robot_name)
    if not sanitized_robot_name:
        sanitized_robot_name = "my_robot"
    
    tmpdir = tempfile.mkdtemp()
    
    package_dir = os.path.join(tmpdir, sanitized_robot_name)
    urdf_dir = os.path.join(package_dir, "urdf")
    mesh_dir = os.path.join(package_dir, "meshes")
    launch_dir = os.path.join(package_dir, "launch")

    os.makedirs(urdf_dir, exist_ok=True)
    os.makedirs(mesh_dir, exist_ok=True)
    os.makedirs(launch_dir, exist_ok=True)

    # Pre-calculate unique names for consistent file mapping and URDF naming
    unique_link_names = generate_unique_names(robot)

    # Process and save mesh files
    mesh_files_map = {}
    if files:
        for file in files:
            form_field_name = file.filename
            # Robustly parse link_id
            if form_field_name and form_field_name.startswith('mesh_'):
                link_id = form_field_name[5:] 
            else:
                link_id = form_field_name
            
            # print(f"DEBUG: Processing file {form_field_name}, extracted ID: {link_id}")

            if link_id in robot.links:
                # print(f"DEBUG: ID {link_id} found in LINKS")
                # Use clean unique name for file (e.g. leg_1.stl)
                clean_name = unique_link_names[link_id]
                safe_filename = f"{clean_name}.stl"
                file_path = os.path.join(mesh_dir, safe_filename)
                
                with open(file_path, "wb") as f:
                    shutil.copyfileobj(file.file, f)
                
                mesh_files_map[link_id] = safe_filename
            elif link_id in robot.joints:
                 # print(f"DEBUG: ID {link_id} found in JOINTS")
                 # It's a joint mesh. Use snake_case joint name.
                 # Since joints don't have a global unique name map passed here yet effectively (unique_link_names is only links),
                 # we will generate a safe name here.
                 joint_name = to_snake_case(robot.joints[link_id].name)
                 safe_filename = f"{joint_name}_{link_id[:4]}.stl" # Append ID snippet for collision avoidance
                 file_path = os.path.join(mesh_dir, safe_filename)

                 with open(file_path, "wb") as f:
                    shutil.copyfileobj(file.file, f)
                 
                 mesh_files_map[link_id] = safe_filename
            else:
                pass # print(f"DEBUG: ID {link_id} NOT FOUND ...")

    # Generate and save URDF file
    urdf_content, base_link_name, _ = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names)
    with open(os.path.join(urdf_dir, f"{sanitized_robot_name}.urdf"), "w") as f:
        f.write(urdf_content)

    # Create other package files
    package_xml = f"""<package format=\"2\">\n<name>{sanitized_robot_name}</name>\n<version>0.1.0</version>\n<description>A description of {sanitized_robot_name}</description>\n<maintainer email=\"user@example.com\">Your Name</maintainer>\n<license>MIT</license>\n<buildtool_depend>catkin</buildtool_depend>\n<depend>roslaunch</depend>\n<depend>robot_state_publisher</depend>\n<depend>rviz</depend>\n<depend>joint_state_publisher_gui</depend>\n</package>\n"""
    with open(os.path.join(package_dir, "package.xml"), "w") as f:
        f.write(package_xml)

    cmakelists_txt = f"""cmake_minimum_required(VERSION 3.0.2)\nproject({sanitized_robot_name})\nfind_package(catkin REQUIRED COMPONENTS roslaunch robot_state_publisher rviz joint_state_publisher_gui)\ncatkin_package()\n"""
    with open(os.path.join(package_dir, "CMakeLists.txt"), "w") as f:
        f.write(cmakelists_txt)


    display_launch = f"""<launch>\n<arg name=\"model\" default=\"$(find {sanitized_robot_name})/urdf/{sanitized_robot_name}.urdf\"/>\n<arg name=\"gui\" default=\"true\" />\n<param name=\"robot_description\" textfile=\"$(arg model)\" />\n<node name=\"joint_state_publisher_gui\" pkg=\"joint_state_publisher_gui\" type=\"joint_state_publisher_gui\" />\n<node name=\"robot_state_publisher\" pkg=\"robot_state_publisher\" type=\"robot_state_publisher\" />\n<node name=\"rviz\" pkg=\"rviz\" type=\"rviz\" args=\"-d $(find {sanitized_robot_name})/launch/display.rviz\" required=\"true\" />\n</launch>\n"""
    with open(os.path.join(launch_dir, "display.launch"), "w") as f:
        f.write(display_launch)
    
    rviz_config = f"""Panels:
- Class: rviz/Displays
  Help Height: 78
  Name: Displays
  Property Tree Widget:
    Expanded:
      - /Global Options1
      - /Status1
      - /RobotModel1
    Splitter Ratio: 0.5
  Tree Height: 600
- Class: rviz/Selection
  Name: Selection
- Class: rviz/Tool Properties
  Expanded:
    - /2D Nav Goal1
    - /2D Pose Estimate1
    - /Publish Point1
  Name: Tool Properties
  Splitter Ratio: 0.588679
- Class: rviz/Views
  Expanded:
    - /Current View1
  Name: Views
  Splitter Ratio: 0.5
- Class: rviz/Time
  Name: Time
  SyncMode: 0
  SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Description File: ""
      Description Source: Topic
      Description Topic: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      Robot Description: "robot_description"
      Shadow Technique: 0
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {base_link_name}

    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0.5
        Z: 0.5
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.4
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 1.57
"""
    with open(os.path.join(launch_dir, "display.rviz"), "w") as f:
        f.write(rviz_config)

    guide_src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'URDF_export.md'))
    if os.path.exists(guide_src_path):
        shutil.copy(guide_src_path, os.path.join(package_dir, 'URDF_export_guide.md'))

    # Create the zip archive
    archive_path = shutil.make_archive(
        base_name=os.path.join(tmpdir, sanitized_robot_name),
        format='zip',
        root_dir=tmpdir,
        base_dir=sanitized_robot_name
    )
    
    # Add a background task to clean up the temporary directory after the response is sent
    background_tasks.add_task(shutil.rmtree, tmpdir)
    
    # Return the zip file from disk
    return FileResponse(
        path=archive_path, 
        media_type='application/zip', 
        filename=f"{sanitized_robot_name}_ros_package.zip"
    )

@app.post("/api/export-urdf-ros2")
async def export_urdf_package_ros2(
    background_tasks: BackgroundTasks,
    robot_data: str = Form(...),
    robot_name: str = Form(...),
    files: Optional[List[UploadFile]] = File(None)
):
    try:
        try:
            data_dict = json.loads(robot_data)
            robot = RobotData.parse_obj(data_dict)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"Invalid robot data format: {e}")

        sanitized_robot_name = to_snake_case(robot_name)
        if not sanitized_robot_name:
            sanitized_robot_name = "my_robot"
        
        tmpdir = tempfile.mkdtemp()
        
        package_dir = os.path.join(tmpdir, sanitized_robot_name)
        urdf_dir = os.path.join(package_dir, "urdf")
        mesh_dir = os.path.join(package_dir, "meshes")
        launch_dir = os.path.join(package_dir, "launch")
        rviz_dir = os.path.join(package_dir, "rviz")

        os.makedirs(urdf_dir, exist_ok=True)
        os.makedirs(mesh_dir, exist_ok=True)
        os.makedirs(launch_dir, exist_ok=True)
        os.makedirs(rviz_dir, exist_ok=True)
        os.makedirs(rviz_dir, exist_ok=True)
        
        # Pre-calculate unique names for consistent file mapping and URDF naming
        unique_link_names = generate_unique_names(robot)

        # Process and save mesh files
        mesh_files_map = {}
        if files:
            for file in files:
                form_field_name = file.filename
                # Robustly parse link_id
                if form_field_name and form_field_name.startswith('mesh_'):
                    link_id = form_field_name[5:] 
                else:
                    link_id = form_field_name
                
                if link_id in robot.links:
                    # Use clean unique name for file (e.g. leg_1.stl)
                    clean_name = unique_link_names[link_id]
                    safe_filename = f"{clean_name}.stl"
                    file_path = os.path.join(mesh_dir, safe_filename)
                    
                    with open(file_path, "wb") as f:
                        shutil.copyfileobj(file.file, f)
                    
                    mesh_files_map[link_id] = safe_filename
                elif link_id in robot.joints:
                    # It's a joint mesh. Use snake_case joint name.
                    joint_name = to_snake_case(robot.joints[link_id].name)
                    safe_filename = f"{joint_name}_{link_id[:4]}.stl"
                    file_path = os.path.join(mesh_dir, safe_filename)

                    with open(file_path, "wb") as f:
                        shutil.copyfileobj(file.file, f)
                    
                    mesh_files_map[link_id] = safe_filename

        # Generate and save URDF file
        urdf_content, base_link_name, _ = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names)
        with open(os.path.join(urdf_dir, f"{sanitized_robot_name}.urdf"), "w") as f:
            f.write(urdf_content)

        # Create ROS2 specific files
        package_xml = f"""<?xml version="1.0"?>\n<package format=\"3\">\n  <name>{sanitized_robot_name}</name>\n  <version>0.1.0</version>\n  <description>A description of {sanitized_robot_name}</description>\n  <maintainer email=\"user@example.com\">Your Name</maintainer>\n  <license>MIT</license>\n\n  <buildtool_depend>ament_cmake</buildtool_depend>\n\n  <exec_depend>joint_state_publisher</exec_depend>\n  <exec_depend>joint_state_publisher_gui</exec_depend>\n  <exec_depend>robot_state_publisher</exec_depend>\n  <exec_depend>rviz2</exec_depend>\n  <exec_depend>xacro</exec_depend>\n\n  <test_depend>ament_lint_auto</test_depend>\n  <test_depend>ament_lint_common</test_depend>\n\n  <export>\n    <build_type>ament_cmake</build_type>\n  </export>\n</package>\n"""
        with open(os.path.join(package_dir, "package.xml"), "w") as f:
            f.write(package_xml)

        cmakelists_txt = f"""cmake_minimum_required(VERSION 3.8)
project({sanitized_robot_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY urdf launch meshes rviz
    DESTINATION share/${{PROJECT_NAME}})

ament_package()
"""
        with open(os.path.join(package_dir, "CMakeLists.txt"), "w") as f:
            f.write(cmakelists_txt)

        display_launch_py = f"""import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')

    urdf_file_name = '{sanitized_robot_name}.urdf'
    urdf = os.path.join(
        get_package_share_directory('{sanitized_robot_name}'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    rviz_config_file = os.path.join(
        get_package_share_directory('{sanitized_robot_name}'),
        'rviz',
        'display.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Whether to use the joint state publisher GUI'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'use_sim_time': use_sim_time, 'robot_description': robot_desc}}]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(use_gui)),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_gui)),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file])
    ])
"""
        with open(os.path.join(launch_dir, "display.launch.py"), "w") as f:
            f.write(display_launch_py)
        rviz_config = f"""Panels:
- Class: rviz_common/Displays
  Help Height: 78
  Name: Displays
  Property Tree Widget:
    Expanded:
      - /Global Options1
      - /Status1
      - /RobotModel1
    Splitter Ratio: 0.5
  Tree Height: 600
- Class: rviz_common/Selection
  Name: Selection
- Class: rviz_common/Tool Properties
  Expanded:
    - /2D Nav Goal1
    - /2D Pose Estimate1
    - /Publish Point1
  Name: Tool Properties
  Splitter Ratio: 0.588679
- Class: rviz_common/Views
  Expanded:
    - /Current View1
  Name: Views
  Splitter Ratio: 0.5
- Class: rviz_common/Time
  Name: Time
  SyncMode: 0
  SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Description File: ""
      Description Source: Topic
      Description Topic: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      Robot Description: "robot_description"
      Shadow Technique: 0
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {base_link_name}

    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0.5
        Z: 0.5
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.4
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 1.57
"""
        with open(os.path.join(rviz_dir, "display.rviz"), "w") as f:
            f.write(rviz_config)

        guide_src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'URDF_export_ros2.md'))
        if os.path.exists(guide_src_path):
            shutil.copy(guide_src_path, os.path.join(package_dir, 'URDF_export_guide_ros2.md'))

        # Create build_and_launch.sh script
        build_script = f"""#!/bin/bash
# Convenience script to build and launch the package
# Usage: Run this script from your ROS2 workspace root after placing the package in src/

echo "Building package {sanitized_robot_name}..."
colcon build --packages-select {sanitized_robot_name} --symlink-install

if [ $? -eq 0 ]; then
    echo "Build successful. Sourcing setup..."
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    else
        echo "Warning: install/setup.bash not found. Sourcing might fail."
    fi
    
    echo "Launching {sanitized_robot_name}..."
    ros2 launch {sanitized_robot_name} display.launch.py
else
    echo "Build failed!"
    exit 1
fi
"""
        script_path = os.path.join(package_dir, "build_and_launch.sh")
        with open(script_path, "w") as f:
            f.write(build_script)
        
        # Make executable
        os.chmod(script_path, 0o755)

        # Create the zip archive
        archive_path = shutil.make_archive(
            base_name=os.path.join(tmpdir, sanitized_robot_name),
            format='zip',
            root_dir=tmpdir,
            base_dir=sanitized_robot_name
        )
        
        background_tasks.add_task(shutil.rmtree, tmpdir)
        
        return FileResponse(
            path=archive_path, 
            media_type='application/zip', 
            filename=f"{sanitized_robot_name}_ros2_package.zip"
        )
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Export Failed: {str(e)}")

@app.get("/")
def read_root():
    """
    Root endpoint for basic API health check.
    """
    return {"message": "RobotLinkForge Backend is running."}

# --- API Endpoints for File Upload and Project Management ---

@app.post("/api/upload-stl")
async def upload_stl_file(file: UploadFile = File(...)):
    if not file.filename.lower().endswith('.stl'):
        raise HTTPException(status_code=400, detail="Invalid file type. Only .stl files are accepted.")

    unique_filename = f"{uuid.uuid4().hex}.stl"
    file_path = os.path.join(MESH_DIR, unique_filename)

    try:
        with open(file_path, "wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Could not save file: {e}")

    return {"url": f"/{file_path}"}


# --- Project Save/Load Endpoints ---

@app.post("/api/projects")
async def save_project(file: UploadFile = File(...), project_name: str = Form(...)):
    # Sanitize project name
    safe_name = "".join(c for c in project_name if c.isalnum() or c in (' ', '_', '-')).strip()
    if not safe_name:
        raise HTTPException(status_code=400, detail="Invalid project name")
    
    filename = f"{safe_name}.zip"
    file_location = os.path.join(PROJECTS_DIR, filename)
    
    try:
        with open(file_location, "wb+") as file_object:
            file_object.write(await file.read())
        return {"message": "Project saved successfully", "filename": filename}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to save project: {str(e)}")

@app.get("/api/projects")
async def list_projects():
    try:
        if not os.path.exists(PROJECTS_DIR):
            return {"projects": []}
        # Filter out "soft deleted" files (part of the prompt requirement)
        files = [f for f in os.listdir(PROJECTS_DIR) if f.endswith('.zip') and "_deleted_" not in f]
        return {"projects": sorted(files)}
    except Exception as e:
        return {"projects": [], "error": str(e)}

@app.delete("/api/projects/{filename}")
async def delete_project(filename: str, request: Request):
    """
    Soft Delete: Renames the file instead of deleting it.
    New name: {original_stem}_deleted_{timestamp}_{client_ip}.zip
    """
    file_path = os.path.join(PROJECTS_DIR, filename)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="Project not found")
    
    try:
        # Get client IP
        client_ip = request.client.host if request.client else "unknown_ip"
        
        # Get Timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Construct new filename
        # Assume valid zip extension from list_projects filtering
        original_stem = os.path.splitext(filename)[0]
        # Format: ProjectName_deleted_20260105_123.123.123.123.zip
        new_filename = f"{original_stem}_deleted_{timestamp}_{client_ip}.zip"
        new_file_path = os.path.join(PROJECTS_DIR, new_filename)
        
        os.rename(file_path, new_file_path)
        
        return {"message": f"Project soft-deleted. Renamed to {new_filename}", "deleted_tag": new_filename}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to delete project: {str(e)}")

@app.get("/api/projects/{filename}")
async def load_project(filename: str):
    file_path = os.path.join(PROJECTS_DIR, filename)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="Project not found")
    return FileResponse(file_path, media_type='application/zip', filename=filename)




@app.post("/api/export-mujoco-urdf")
async def export_mujoco_urdf(
    background_tasks: BackgroundTasks,
    robot_data: str = Form(...),
    robot_name: str = Form(...),
    files: Optional[List[UploadFile]] = File(None)
):
    try:
        try:
            data_dict = json.loads(robot_data)
            robot = RobotData.parse_obj(data_dict)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"Invalid robot data format: {e}")

        sanitized_robot_name = to_snake_case(robot_name) or "my_robot"
        tmpdir = tempfile.mkdtemp()
        package_dir = os.path.join(tmpdir, sanitized_robot_name)
        mesh_dir = os.path.join(package_dir, "meshes")
        os.makedirs(mesh_dir, exist_ok=True)
        
        unique_link_names = generate_unique_names(robot)
        mesh_files_map = {}
        
        if files:
            for file in files:
                link_id = file.filename[5:] if file.filename.startswith('mesh_') else file.filename
                if link_id in robot.links:
                    clean_name = unique_link_names[link_id]
                    safe_filename = f"{clean_name}.stl"
                    dest_path = os.path.join(mesh_dir, safe_filename)
                    # Save upload to temp file first to process it
                    with tempfile.NamedTemporaryFile(delete=False) as tmp_upload:
                         shutil.copyfileobj(file.file, tmp_upload)
                         tmp_upload_path = tmp_upload.name
                    
                    try:
                        ensure_binary_stl(tmp_upload_path, dest_path)
                    finally:
                         if os.path.exists(tmp_upload_path):
                             os.remove(tmp_upload_path)

                    mesh_files_map[link_id] = safe_filename
                elif link_id in robot.joints:
                     joint_name = to_snake_case(robot.joints[link_id].name)
                     safe_filename = f"{joint_name}_{link_id[:4]}.stl"
                     dest_path = os.path.join(mesh_dir, safe_filename)
                     
                     with tempfile.NamedTemporaryFile(delete=False) as tmp_upload:
                         shutil.copyfileobj(file.file, tmp_upload)
                         tmp_upload_path = tmp_upload.name

                     try:
                        ensure_binary_stl(tmp_upload_path, dest_path)
                     finally:
                         if os.path.exists(tmp_upload_path):
                             os.remove(tmp_upload_path)

                     mesh_files_map[link_id] = safe_filename

        # Generate URDF
        urdf_content, _, joint_infos = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names, for_mujoco=True)

        # Generate Actuators XML
        actuators_xml = ""
        for j_info in joint_infos:
            j_name = j_info['name']
            j_type = j_info['type']
            # Add position actuator for moving joints
            # Default kp=50 is decent for simple models.
            # ctrlrange limits the slider.
            if j_info.get('limit'):
                lower = j_info['limit'].lower
                upper = j_info['limit'].upper
                ctrl_range = f'{lower} {upper}'
            else:
                ctrl_range = "-3.14 3.14"

            actuators_xml += f'    <position name="{j_name}_act" joint="{j_name}" kp="50" ctrlrange="{ctrl_range}" />\n'
        
        # FIX for MuJoCo: 
        # 1. Strip full package path to leave just filename: "package://robot/meshes/foo.stl" -> "foo.stl"
        urdf_content = urdf_content.replace(f'package://{sanitized_robot_name}/meshes/', '')
        
        # 2. Inject <mujoco> block with meshdir AND actuators
        # Find position after <robot ...>
        import re
        match = re.search(r'<robot\s+name="[^"]+">', urdf_content)
        if match:
            insert_pos = match.end()
            mujoco_tag = f'\n  <mujoco>\n    <compiler meshdir="meshes" balanceinertia="true" discardvisual="false"/>\n    <option gravity="0 0 -9.81"/>\n    <actuator>\n{actuators_xml}    </actuator>\n  </mujoco>'
            urdf_content = urdf_content[:insert_pos] + mujoco_tag + urdf_content[insert_pos:]
        
        urdf_filename = f"{sanitized_robot_name}.urdf"
        with open(os.path.join(package_dir, urdf_filename), "w") as f:
            f.write(urdf_content)

        # Generate Python Script
        script_content = f"""
import mujoco
try:
    import mujoco.viewer
except ImportError:
    print("Please install mujoco with 'pip install mujoco'")
    exit(1)
import time
import argparse

parser = argparse.ArgumentParser(description='Visualize MuJoCo model.')
parser.add_argument('--kinematic', action='store_true', help='Run in kinematic mode (gravity disabled)')
parser.add_argument('--set-joint', nargs=2, action='append', metavar=('JOINT_NAME', 'VALUE'), help='Set initial joint value. Example: --set-joint joint_1 1.57')
args = parser.parse_args()

model_path = "{urdf_filename}"
print(f"Loading model from {{model_path}}...")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

if args.kinematic:
    print("Kinematic mode enabled: Gravity disabled.")
    model.opt.gravity = (0, 0, 0)

# Apply Set-Joint Arguments
if args.set_joint:
    print("Setting initial joint values:")
    for j_name, j_val_str in args.set_joint:
        try:
            val = float(j_val_str)
            # Find joint ID
            j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
            if j_id != -1:
                # Find qpos address
                q_addr = model.jnt_qposadr[j_id]
                data.qpos[q_addr] = val
                print(f"  - Joint '{{j_name}}' -> {{val}}")
                
                # Also set Actuator Control if exists (to prevent fighting)
                # Our main.py generates actuators named "{{j_name}}_act"
                act_name = j_name + "_act"
                act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                if act_id != -1:
                     data.ctrl[act_id] = val
                     print(f"    (Set actuator '{{act_name}}' to {{val}})")
            else:
                print(f"  ! Warning: Joint '{{j_name}}' not found.")
        except ValueError:
            print(f"  ! Invalid value for joint {{j_name}}: {{j_val_str}}")

print("Launching viewer...")
# Use blocking viewer for better UI/Controls handling
mujoco.viewer.launch(model, data)
"""
        with open(os.path.join(package_dir, "visualize_urdf.py"), "w") as f:
            f.write(script_content)

        # Generate Launch Scripts
        # 1. launch.sh
        launch_sh = f"""#!/bin/bash
python3 visualize_urdf.py "$@"
"""
        with open(os.path.join(package_dir, "launch.sh"), "w") as f:
            f.write(launch_sh)
        os.chmod(os.path.join(package_dir, "launch.sh"), 0o755)

        # 2. launch_kinematic.sh
        launch_kin_sh = f"""#!/bin/bash
python3 visualize_urdf.py --kinematic "$@"
"""
        with open(os.path.join(package_dir, "launch_kinematic.sh"), "w") as f:
            f.write(launch_kin_sh)
        os.chmod(os.path.join(package_dir, "launch_kinematic.sh"), 0o755)

        # 3. launch.bat (Windows)
        launch_bat = f"""@echo off
python visualize_urdf.py %*
pause
"""
        with open(os.path.join(package_dir, "launch.bat"), "w") as f:
            f.write(launch_bat)

        # Generate README
        readme_content = f"""
# MuJoCo Visualization (URDF)

This package contains the URDF model of **{robot_name}** ready for MuJoCo.

## Prerequisites
- Python 3.8+
- MuJoCo library

## Installation
```bash
pip install mujoco
```

## Running the Simulation
Run the visualization script:
```bash
python visualize_urdf.py
```

### Kinematic Mode (No Gravity)
To run without gravity (e.g., to check joint limits or move joints manually without them falling):
```bash
python visualize_urdf.py --kinematic
```

### Setting Joint Positions (CLI)
You can set initial joint angles (and actuator targets) using `--set-joint`:
```bash
python visualize_urdf.py --set-joint joint_1 1.57 --set-joint joint_2 -0.5
```

Or drag and drop `{urdf_filename}` into the standalone MuJoCo simulator.

## Files
- `{urdf_filename}`: The robot model description.
- `meshes/`: STL files for links.
- `visualize_urdf.py`: Simple python script to load and view the model.
"""
        with open(os.path.join(package_dir, "README_MUJOCO.md"), "w") as f:
            f.write(readme_content)

        # Zip it
        # Zip it
        shutil.make_archive(package_dir, 'zip', root_dir=tmpdir, base_dir=sanitized_robot_name)
        return FileResponse(f"{package_dir}.zip", media_type='application/zip', filename=f"{sanitized_robot_name}_mujoco_urdf.zip")

    except Exception as e:
        print(f"Export Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/export-mujoco-mjcf")
async def export_mujoco_mjcf(
    background_tasks: BackgroundTasks,
    robot_data: str = Form(...),
    robot_name: str = Form(...),
    files: Optional[List[UploadFile]] = File(None)
):
    try:
        try:
            data_dict = json.loads(robot_data)
            robot = RobotData.parse_obj(data_dict)
        except Exception as e:
            raise HTTPException(status_code=400, detail=f"Invalid robot data format: {e}")

        sanitized_robot_name = to_snake_case(robot_name) or "my_robot"
        tmpdir = tempfile.mkdtemp()
        package_dir = os.path.join(tmpdir, sanitized_robot_name)
        mesh_dir = os.path.join(package_dir, "meshes")
        os.makedirs(mesh_dir, exist_ok=True)
        
        unique_link_names = generate_unique_names(robot)
        mesh_files_map = {}
        
        if files:
            for file in files:
                link_id = file.filename[5:] if file.filename.startswith('mesh_') else file.filename
                if link_id in robot.links:
                    clean_name = unique_link_names[link_id]
                    safe_filename = f"{clean_name}.stl"
                    dest_path = os.path.join(mesh_dir, safe_filename)
                    with tempfile.NamedTemporaryFile(delete=False) as tmp_upload:
                         shutil.copyfileobj(file.file, tmp_upload)
                         tmp_upload_path = tmp_upload.name
                    try:
                        ensure_binary_stl(tmp_upload_path, dest_path)
                    finally:
                        if os.path.exists(tmp_upload_path): os.remove(tmp_upload_path)
                    mesh_files_map[link_id] = safe_filename
                elif link_id in robot.joints:
                     joint_name = to_snake_case(robot.joints[link_id].name)
                     safe_filename = f"{joint_name}_{link_id[:4]}.stl"
                     dest_path = os.path.join(mesh_dir, safe_filename)
                     with tempfile.NamedTemporaryFile(delete=False) as tmp_upload:
                         shutil.copyfileobj(file.file, tmp_upload)
                         tmp_upload_path = tmp_upload.name
                     try:
                        ensure_binary_stl(tmp_upload_path, dest_path)
                     finally:
                        if os.path.exists(tmp_upload_path): os.remove(tmp_upload_path)
                     mesh_files_map[link_id] = safe_filename

        # Generate MJCF XML
        mjcf_content = generate_mjcf_xml(robot, sanitized_robot_name, mesh_files_map)
        mjcf_filename = f"{sanitized_robot_name}.xml"
        with open(os.path.join(package_dir, mjcf_filename), "w") as f:
            f.write(mjcf_content)

        # Generate Python Script
        script_content = f"""
import mujoco
try:
    import mujoco.viewer
except ImportError:
    print("Please install mujoco with 'pip install mujoco'")
    exit(1)
import time

model_path = "{mjcf_filename}"
print(f"Loading model from {{model_path}}...")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("Launching viewer...")
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(model, data)
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
"""
        with open(os.path.join(package_dir, "visualize_mjcf.py"), "w") as f:
            f.write(script_content)

        # Generate README
        readme_content = f"""
# MuJoCo Visualization (MJCF)

This package contains the **Native MJCF XML** model of **{robot_name}**.

## Prerequisites
- Python 3.8+
- MuJoCo library

## Installation
```bash
pip install mujoco
```

## Running the Simulation
Run the visualization script:
```bash
python visualize_mjcf.py
```

Or drag and drop `{mjcf_filename}` into the standalone MuJoCo simulator.

## Files
- `{mjcf_filename}`: The native MJCF robot description.
- `meshes/`: STL files for geometries.
- `visualize_mjcf.py`: Python script to load and view the model.
"""
        with open(os.path.join(package_dir, "README_MUJOCO.md"), "w") as f:
            f.write(readme_content)

        # Zip it
        # Zip it
        shutil.make_archive(package_dir, 'zip', root_dir=tmpdir, base_dir=sanitized_robot_name)
        return FileResponse(f"{package_dir}.zip", media_type='application/zip', filename=f"{sanitized_robot_name}_mujoco_mjcf.zip")

    except Exception as e:
        print(f"Export Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
