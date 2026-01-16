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
from robot_models import RobotData, RobotLink, RobotJoint, Visual
from utils import to_snake_case, generate_unique_names, generate_unique_joint_names
from exporters.urdf_exporter import generate_urdf_xml
from exporters.mjcf_exporter import generate_mjcf_xml
from exporters.gazebo_exporter import inject_gazebo_tags, generate_gazebo_launch, inject_gazebo_ros2_tags, generate_gazebo_ros2_launch
from exporters.motion_exporter import process_recordings_for_export, generate_ros2_playback_node, generate_mujoco_playback_script
from exporters.stl_utils import ensure_binary_stl

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

# --- Recordings Storage ---
RECORDINGS_DIR = "saved_recordings"
os.makedirs(RECORDINGS_DIR, exist_ok=True)

app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/api/recordings")
def list_recordings():
    files = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith(".json")]
    return {"files": files}

@app.post("/api/recordings")
async def save_recordings(request: Request):
    data = await request.json()
    filename = data.get("filename", "recordings")
    recordings = data.get("recordings", [])
    
    # Sanitize filename
    filename = re.sub(r'[^a-zA-Z0-9_\-\.]', '', filename)
    if not filename.endswith(".json"):
        filename += ".json"
    
    filepath = os.path.join(RECORDINGS_DIR, filename)
    with open(filepath, "w") as f:
        json.dump(recordings, f, indent=2)
    return {"message": "Saved successfully", "filename": filename}

@app.get("/api/recordings/{filename}")
def load_recordings(filename: str):
    # Sanitize
    filename = re.sub(r'[^a-zA-Z0-9_\-\.]', '', filename)
    if not filename.endswith(".json"):
        filename += ".json"
        
    filepath = os.path.join(RECORDINGS_DIR, filename)
    if not os.path.exists(filepath):
        raise HTTPException(status_code=404, detail="Recording file not found")
        
    with open(filepath, "r") as f:
        recordings = json.load(f)
    return {"recordings": recordings}

@app.delete("/api/recordings/{filename}")
def delete_recording_file(filename: str):
    # Sanitize
    filename = re.sub(r'[^a-zA-Z0-9_\-\.]', '', filename)
    if not filename.endswith(".json"):
        filename += ".json"
        
    filepath = os.path.join(RECORDINGS_DIR, filename)
    if not os.path.exists(filepath):
        raise HTTPException(status_code=404, detail="Recording file not found")
    
    try:
        os.remove(filepath)
        return {"message": "Deleted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))



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
    recordings: Optional[str] = Form(None),
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
        urdf_content, base_link_name, generated_joints_info = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names)
        with open(os.path.join(urdf_dir, f"{sanitized_robot_name}.urdf"), "w") as f:
            f.write(urdf_content)

        # Create ROS2 specific files
        package_xml = f"""<?xml version="1.0"?>\n<package format=\"3\">\n  <name>{sanitized_robot_name}</name>\n  <version>0.1.0</version>\n  <description>A description of {sanitized_robot_name}</description>\n  <maintainer email=\"user@example.com\">Your Name</maintainer>\n  <license>MIT</license>\n\n  <buildtool_depend>ament_cmake</buildtool_depend>\n\n  <exec_depend>joint_state_publisher</exec_depend>\n  <exec_depend>joint_state_publisher_gui</exec_depend>\n  <exec_depend>robot_state_publisher</exec_depend>\n  <exec_depend>rviz2</exec_depend>\n  <exec_depend>xacro</exec_depend>\n\n  <test_depend>ament_lint_auto</test_depend>\n  <test_depend>ament_lint_common</test_depend>\n\n  <export>\n    <build_type>ament_cmake</build_type>\n  </export>\n</package>\n"""
        with open(os.path.join(package_dir, "package.xml"), "w") as f:
            f.write(package_xml)

        # Process Motion Recordings if present
        extra_install_dirs = "urdf launch meshes rviz"
        install_program_cmds = ""
        
        if recordings:
            try:
                recs_raw = json.loads(recordings)
                processed_recs = process_recordings_for_export(recs_raw, generated_joints_info, robot)
                
                # 1. Save recordings.json to config/
                config_dir = os.path.join(package_dir, "config")
                os.makedirs(config_dir, exist_ok=True)
                with open(os.path.join(config_dir, "recordings.json"), "w") as f:
                    json.dump(processed_recs, f, indent=2)
                extra_install_dirs += " config"
                
                # 2. Generate Replay Node in scripts/
                scripts_dir = os.path.join(package_dir, "scripts")
                os.makedirs(scripts_dir, exist_ok=True)
                replay_script = generate_ros2_playback_node(sanitized_robot_name)
                replay_script_path = os.path.join(scripts_dir, "replay_node.py")
                with open(replay_script_path, "w") as f:
                    f.write(replay_script)
                os.chmod(replay_script_path, 0o755)
                
                # Install replay_node.py to lib/pkg_name so 'ros2 run' finds it
                # We also assume 'rclpy' is available.
                install_program_cmds += f"install(PROGRAMS scripts/replay_node.py DESTINATION lib/${{PROJECT_NAME}})\n"

                # 3. Generate replay.launch.py (Exclusive launch preventing conflict with joint_state_publisher)
                replay_launch_py = f"""import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    recording_index = LaunchConfiguration('recording_index', default='0')

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
            'recording_index',
            default_value='0',
            description='Index of the recording to play'),
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'use_sim_time': use_sim_time, 'robot_description': robot_desc}}]),

        # Replay Node (replaces joint_state_publisher)
        Node(
            package='{sanitized_robot_name}',
            executable='replay_node.py',
            name='replay_node',
            output='screen',
            parameters=[{{'recording_index': recording_index}}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file])
    ])
"""
                launch_dir = os.path.join(package_dir, "launch")
                with open(os.path.join(launch_dir, "replay.launch.py"), "w") as f:
                    f.write(replay_launch_py)

                # 4. Generate Shell Scripts for each recording
                for i, rec in enumerate(processed_recs):
                    rec_name_clean = to_snake_case(rec.get('name', f'rec_{i}'))
                    sh_filename = f"replay_{i}_{rec_name_clean}.sh"
                    sh_path = os.path.join(package_dir, sh_filename)
                    
                    sh_content = f"""#!/bin/bash
# Replay recording #{i}: {rec.get('name')}
# Runs the Replay Node ONLY.
# Ensure you are running 'visualize_passive.sh' in another terminal!

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "../install/setup.bash" ]; then
    source ../install/setup.bash
else
    echo "Warning: setup.bash not found. Attempting to run anyway..."
fi

echo "Launching Replay Node for Recording #{i}..."
ros2 run {sanitized_robot_name} replay_node.py --ros-args -p recording_index:={i}
"""
                    with open(sh_path, "w") as f:
                        f.write(sh_content)
                    os.chmod(sh_path, 0o755)
                
                # Install visualize_passive.sh script as regular program (optional but good)
                install_program_cmds += f"install(PROGRAMS visualize_passive.sh DESTINATION lib/${{PROJECT_NAME}})\n"
                
            except Exception as e:
                print(f"Error processing recordings: {e}")

        # Update CMakeLists to match
        cmakelists_txt = f"""cmake_minimum_required(VERSION 3.8)
project({sanitized_robot_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY {extra_install_dirs}
    DESTINATION share/${{PROJECT_NAME}})

{install_program_cmds}

ament_package()
"""
        with open(os.path.join(package_dir, "CMakeLists.txt"), "w") as f:
            f.write(cmakelists_txt)

        display_launch_py = f"""import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')
    use_jsp = LaunchConfiguration('use_jsp', default='true')

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
        DeclareLaunchArgument(
            'use_jsp',
            default_value='true',
            description='Whether to enable joint_state_publisher (gui or non-gui)'),

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
            parameters=[{{'source_list': ['replay_joint_states']}}],
            condition=IfCondition(PythonExpression(["'", use_jsp, "' == 'true' and '", use_gui, "' == 'false'"]))),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(PythonExpression(["'", use_jsp, "' == 'true' and '", use_gui, "' == 'true'"]))),
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
            
        # Generate visualize_passive.sh
        passive_sh = f"""#!/bin/bash
# Launch RViz without Joint State Publisher (Passive Mode)
# This allows replay_node or other external scripts to control joints.
source install/setup.bash
echo "Launching RViz in Passive Mode..."
ros2 launch {sanitized_robot_name} display.launch.py use_jsp:=true use_gui:=false
"""
        with open(os.path.join(package_dir, "visualize_passive.sh"), "w") as f:
            f.write(passive_sh)
        os.chmod(os.path.join(package_dir, "visualize_passive.sh"), 0o755)

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
    
    # Ask user for mode
    echo ""
    echo "Select Mode:"
    echo "1) Standard (Enable Sliders) - Default"
    echo "2) Passive (Disable Sliders) - Use this for Replay!"
    read -p "Enter choice [1]: " choice
    
    if [ "$choice" = "2" ]; then
        echo "Launching {sanitized_robot_name} in PASSIVE mode..."
        # We enable JSP but disable GUI, so it bridges replay_joint_states -> joint_states
        ros2 launch {sanitized_robot_name} display.launch.py use_jsp:=true use_gui:=false
    else
        echo "Launching {sanitized_robot_name} in STANDARD mode..."
        ros2 launch {sanitized_robot_name} display.launch.py
    fi
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

        # Generate README
        readme_content = f"""# {sanitized_robot_name} ROS 2 Package

## Installation
1. Copy this folder to your ROS 2 workspace (e.g. `src/`).
2. Build the workspace:
   ```bash
   colcon build --packages-select {sanitized_robot_name} --symlink-install
   source install/setup.bash
   ```

## Visualization
Launch the robot in RViz with:
```bash
ros2 launch {sanitized_robot_name} display.launch.py
```
"""
        if recordings:
             readme_content += f"""
## Motion Replay
This package includes exported motion recordings (from Robot Link Forge).

### Playback
To play back the recorded motions:
1. Launch the robot visualization as above.
2. In a new terminal, source the workspace:
   ```bash
   source install/setup.bash
   ```
3. Run the replay node:
   ```bash
   ros2 run {sanitized_robot_name} replay_node.py
   ```
   
   The robot should start moving according to the recording!
"""
        with open(os.path.join(package_dir, "README.md"), "w") as f:
            f.write(readme_content)

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
    recordings: Optional[str] = Form(None),
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

        # Process Motion Recordings if present
        if recordings:
            try:
                recs_raw = json.loads(recordings)
                processed_recs = process_recordings_for_export(recs_raw, joint_infos, robot)
                
                # Save recordings.json
                with open(os.path.join(package_dir, "recordings.json"), "w") as f:
                    json.dump(processed_recs, f, indent=2)
                
                # Generate Replay Script
                replay_py = generate_mujoco_playback_script(urdf_filename)
                with open(os.path.join(package_dir, "replay_recording.py"), "w") as f:
                    f.write(replay_py)

                # Generate Shell Scripts for each recording (Bash)
                for i, rec in enumerate(processed_recs):
                    rec_name_clean = to_snake_case(rec.get('name', f'rec_{i}'))
                    sh_filename = f"replay_{i}_{rec_name_clean}.sh"
                    sh_path = os.path.join(package_dir, sh_filename)
                    
                    sh_content = f"""#!/bin/bash
# Replay recording #{i}: {rec.get('name')}
python3 replay_recording.py {i}
"""
                    with open(sh_path, "w") as f:
                        f.write(sh_content)
                    os.chmod(sh_path, 0o755)

            except Exception as e:
                print(f"Error processing recordings: {e}")

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
Or use the convenience scripts:
```bash
./launch.sh
```

### Kinematic Mode (No Gravity)
To run without gravity (e.g., to check joint limits or move joints manually without them falling):
```bash
./launch_kinematic.sh
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
        if recordings:
            readme_content += """
## Motion Replay
This package includes exported motion recordings.

### How to Replay
We have generated convenient bash scripts for each recording.
Simply run the script corresponding to the recording you want to play:

```bash
./replay_0_recording_name.sh
```

Alternatively, you can run the python script directly:
```bash
python3 replay_recording.py [index]
```
(Where [index] is 0, 1, 2...)
"""

        with open(os.path.join(package_dir, "README.md"), "w") as f:
            f.write(readme_content)


        # Zip it
        shutil.make_archive(package_dir, 'zip', root_dir=tmpdir, base_dir=sanitized_robot_name)
        return FileResponse(f"{package_dir}.zip", media_type='application/zip', filename=f"{sanitized_robot_name}_mujoco_mjcf.zip")

    except Exception as e:
        print(f"Export Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/export-gazebo")
async def export_gazebo(
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
        urdf_dir = os.path.join(package_dir, "urdf")
        mesh_dir = os.path.join(package_dir, "meshes")
        launch_dir = os.path.join(package_dir, "launch")
        
        os.makedirs(urdf_dir, exist_ok=True)
        os.makedirs(mesh_dir, exist_ok=True)
        os.makedirs(launch_dir, exist_ok=True)
        
        unique_link_names = generate_unique_names(robot)
        mesh_files_map = {}
        
        # Process Meshes
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
                    try: ensure_binary_stl(tmp_upload_path, dest_path)
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
                     try: ensure_binary_stl(tmp_upload_path, dest_path)
                     finally:
                        if os.path.exists(tmp_upload_path): os.remove(tmp_upload_path)
                     mesh_files_map[link_id] = safe_filename

        # Generate Base URDF
        # Note: Gazebo works best with standard URDF, collisions enabled.
        # We reuse the standard export and then inject tags.
        urdf_content, _, _ = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names, for_mujoco=False)
        
        # Inject Gazebo Tags
        urdf_content = inject_gazebo_tags(urdf_content, robot)
        
        urdf_filename = f"{sanitized_robot_name}.urdf"
        with open(os.path.join(urdf_dir, urdf_filename), "w") as f:
            f.write(urdf_content)
            
        # Generate Launch File
        launch_content = generate_gazebo_launch(sanitized_robot_name)
        with open(os.path.join(launch_dir, "gazebo.launch"), "w") as f:
            f.write(launch_content)
            
        # CMakeLists and Package.xml for completeness (Minimal ROS package)
        package_xml = f"""<package format="2">
  <name>{sanitized_robot_name}</name>
  <version>0.1.0</version>
  <description>Gazebo package for {sanitized_robot_name}</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>gazebo_ros</depend>
  <depend>robot_state_publisher</depend>
</package>
"""
        with open(os.path.join(package_dir, "package.xml"), "w") as f:
            f.write(package_xml)
            
        cmake_content = f"""cmake_minimum_required(VERSION 3.0.2)
project({sanitized_robot_name})
find_package(catkin REQUIRED COMPONENTS gazebo_ros)
catkin_package()
"""
        with open(os.path.join(package_dir, "CMakeLists.txt"), "w") as f:
            f.write(cmake_content)

        # Generate README
        readme_content = f"""
# Gazebo Simulation Package

This package contains the URDF and Launch files to simulate **{robot_name}** in Gazebo.

## Prerequisites
- ROS Noetic (or compatible)
- Gazebo
- `gazebo_ros_pkgs`
- `gazebo_ros_control`

## Installation
1. Place this folder in your catkin workspace `src/` directory.
   ```bash
   cp -r {sanitized_robot_name} ~/catkin_ws/src/
   ```
2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Running
Launch the simulation:
```bash
roslaunch {sanitized_robot_name} gazebo.launch
```
This will:
1. Start Gazebo with an empty world.
2. Spawn the robot model.
3. Start the `robot_state_publisher`.

## Notes
- To control joints, you may need to add transmission tags and load controllers.
- This export includes a basic `gazebo_ros_control` plugin hook.
"""
        with open(os.path.join(package_dir, "README_GAZEBO.md"), "w") as f:
            f.write(readme_content)

        # Zip it
        shutil.make_archive(package_dir, 'zip', root_dir=tmpdir, base_dir=sanitized_robot_name)
        return FileResponse(f"{package_dir}.zip", media_type='application/zip', filename=f"{sanitized_robot_name}_gazebo.zip")

    except Exception as e:
        print(f"Export Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/export-gazebo-ros2")
async def export_gazebo_ros2(
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
        urdf_dir = os.path.join(package_dir, "urdf")
        mesh_dir = os.path.join(package_dir, "meshes")
        launch_dir = os.path.join(package_dir, "launch")
        
        os.makedirs(urdf_dir, exist_ok=True)
        os.makedirs(mesh_dir, exist_ok=True)
        os.makedirs(launch_dir, exist_ok=True)
        
        unique_link_names = generate_unique_names(robot)
        mesh_files_map = {}
        
        # Process Meshes
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
                    try: ensure_binary_stl(tmp_upload_path, dest_path)
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
                     try: ensure_binary_stl(tmp_upload_path, dest_path)
                     finally:
                        if os.path.exists(tmp_upload_path): os.remove(tmp_upload_path)
                     mesh_files_map[link_id] = safe_filename

        # Generate Base URDF (Collision enabled)
        urdf_content, _, _ = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names, for_mujoco=False)
        
        # Inject ROS 2 Gazebo Tags
        urdf_content = inject_gazebo_ros2_tags(urdf_content, robot)
        
        urdf_filename = f"{sanitized_robot_name}.urdf"
        with open(os.path.join(urdf_dir, urdf_filename), "w") as f:
            f.write(urdf_content)
            
        # Generate Launch File (Python)
        launch_content = generate_gazebo_ros2_launch(sanitized_robot_name)
        with open(os.path.join(launch_dir, "gazebo.launch.py"), "w") as f:
            f.write(launch_content)
            
        # package.xml (ament_cmake)
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{sanitized_robot_name}</name>
  <version>0.0.0</version>
  <description>Gazebo ROS 2 package for {sanitized_robot_name}</description>
  <maintainer email="user@todo.todo">User</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>gazebo_ros_pkgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
        with open(os.path.join(package_dir, "package.xml"), "w") as f:
            f.write(package_xml)
            
        # CMakeLists.txt (ament_cmake)
        cmake_content = f"""cmake_minimum_required(VERSION 3.8)
project({sanitized_robot_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install directories
install(DIRECTORY launch
  DESTINATION share/${{PROJECT_NAME}}
)

install(DIRECTORY urdf
  DESTINATION share/${{PROJECT_NAME}}
)

install(DIRECTORY meshes
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
"""
        with open(os.path.join(package_dir, "CMakeLists.txt"), "w") as f:
            f.write(cmake_content)

        # Generate README
        readme_content = f"""
# Gazebo ROS 2 Package

This package contains the configuration to simulate **{robot_name}** in Gazebo with ROS 2 (Humble/Jazzy/Rolling).

## Prerequisites
- ROS 2 (Humble recommended)
- **Gazebo Classic** (ver 11). This package uses `gazebo_ros`, NOT `ros_gz` (Fortress/Harmonic).
  - Install via: `sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs`
- `gazebo_ros_pkgs`
- `ros2_control` & `gazebo_ros2_control` (optional but recommended for joint control)

## Installation
1. Place this folder in your colcon workspace `src/` directory.
   ```bash
   cp -r {sanitized_robot_name} ~/ros2_ws/src/
   ```
2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select {sanitized_robot_name}
   source install/setup.bash
   ```

## Running
Launch the simulation:
```bash
ros2 launch {sanitized_robot_name} gazebo.launch.py
```
This will start Gazebo and spawn the robot.
"""
        with open(os.path.join(package_dir, "README_GAZEBO_ROS2.md"), "w") as f:
            f.write(readme_content)

        # Generate One-Click Launch Script
        launch_script_content = f"""#!/bin/bash
set -e

# Check if ROS_DISTRO is set
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS_DISTRO is not set."
    echo "Please source your ROS 2 installation first."
    echo "Example: source /opt/ros/humble/setup.bash"
    exit 1
fi

PACKAGE_NAME="{sanitized_robot_name}"
CURRENT_DIR=$(pwd)
WORKSPACE_DIR="$CURRENT_DIR/standalone_ws"

echo "------------------------------------------------"
echo "Setting up standalone workspace for $PACKAGE_NAME"
echo "Location: $WORKSPACE_DIR"
echo "------------------------------------------------"

# 1. Create Workspace
mkdir -p "$WORKSPACE_DIR/src/$PACKAGE_NAME"

# 2. Copy Package Files
# We explicitly copy only the package files to avoid recursion if we were to copy '.'
echo "Copying package files..."
cp -r package.xml CMakeLists.txt urdf launch meshes "$WORKSPACE_DIR/src/$PACKAGE_NAME/"


# 3. Resolve Dependencies
if [ "$SKIP_DEPENDENCIES" = "1" ]; then
    echo "Skipping dependency check (SKIP_DEPENDENCIES=1)..."
else
    echo "Installing dependencies..."
    cd "$WORKSPACE_DIR"

    # Explicit check for Gazebo Classic packages
    if ! dpkg -l | grep -q "ros-$ROS_DISTRO-gazebo-ros"; then
        echo "------------------------------------------------"
        echo "WARNING: Gazebo Classic packages not found!"
        echo "This export targets Gazebo Classic (gazebo_ros)."
        echo "Please ensure you have installed: ros-$ROS_DISTRO-gazebo-ros-pkgs"
        echo "------------------------------------------------"
    fi

    echo "Running rosdep install..."
    # We capture the exit code to give better advice
    set +e # Temporarily disable strict exit
    rosdep install --from-paths src --ignore-src -r -y
    ROSDEP_EXIT=$?
    set -e # Re-enable strict exit

    if [ $ROSDEP_EXIT -ne 0 ]; then
        echo "------------------------------------------------"
        echo "WARNING: 'rosdep install' failed."
        echo "If you are behind a corporate firewall/proxy:"
        echo "1. Export proxy variables:"
        echo "   export http_proxy=http://your-proxy:port"
        echo "   export https_proxy=http://your-proxy:port"
        echo "2. Configure apt proxy in /etc/apt/apt.conf"
        echo ""
        echo "If you believe you have all dependencies installed,"
        echo "you can bypass this check by running:"
        echo "   export SKIP_DEPENDENCIES=1; ./setup_and_launch.sh"
        echo "------------------------------------------------"
        echo "Attempting to continue build..."
    fi
fi

# 4. Build
echo "Building package..."
cd "$WORKSPACE_DIR"
colcon build --packages-select "$PACKAGE_NAME"


# 4. Source and Launch
echo "------------------------------------------------"
echo "Build complete. Launching simulation..."
echo "------------------------------------------------"

source install/setup.bash
ros2 launch "$PACKAGE_NAME" gazebo.launch.py
"""
        script_path = os.path.join(package_dir, "setup_and_launch.sh")
        with open(script_path, "w") as f:
            f.write(launch_script_content)
        
        # Make executable
        os.chmod(script_path, 0o755)

        # Zip it
        shutil.make_archive(package_dir, 'zip', root_dir=tmpdir, base_dir=sanitized_robot_name)
        return FileResponse(f"{package_dir}.zip", media_type='application/zip', filename=f"{sanitized_robot_name}_gazebo_ros2.zip")

    except Exception as e:
        print(f"Export Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))




@app.post("/api/export-mujoco-mjcf")
async def export_mujoco_mjcf(
    background_tasks: BackgroundTasks,
    robot_data: str = Form(...),
    robot_name: str = Form(...),
    files: Optional[List[UploadFile]] = File(None),
    mesh_collision: bool = Form(False),
    direct_hand: bool = Form(False),
    recordings: str = Form(None)
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

        # Generate MJCF XML and get joint info
        mjcf_content, generated_joints_info = generate_mjcf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names, use_mesh_collision=mesh_collision, direct_hand=direct_hand)
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
# Use blocking viewer for better control (Pause, Reset, Sliders)
mujoco.viewer.launch(model, data)
"""
        with open(os.path.join(package_dir, "visualize_mjcf.py"), "w") as f:
            f.write(script_content)

        # Generate Shell Scripts
        launch_sh = f"""#!/bin/bash
python3 visualize_mjcf.py "$@"
"""
        with open(os.path.join(package_dir, "launch.sh"), "w") as f:
            f.write(launch_sh)
        os.chmod(os.path.join(package_dir, "launch.sh"), 0o755)

        # Process Recordings for MuJoCo
        if recordings:
            try:
                 recs_raw = json.loads(recordings)
                 processed_recs = process_recordings_for_export(recs_raw, generated_joints_info, robot)
                 
                 # 1. Save recordings.json
                 with open(os.path.join(package_dir, "recordings.json"), "w") as f:
                     json.dump(processed_recs, f, indent=2)
                     
                 # 2. Generate Replay Script
                 replay_script = generate_mujoco_playback_script(mjcf_filename)
                 with open(os.path.join(package_dir, "replay_mujoco.py"), "w") as f:
                     f.write(replay_script)
                 
                 # 3. Generate Replay Shell Scripts
                 for i, rec in enumerate(processed_recs):
                     rec_name_clean = to_snake_case(rec.get('name', f'rec_{i}'))
                     sh_filename = f"replay_{i}_{rec_name_clean}.sh"
                     sh_path = os.path.join(package_dir, sh_filename)
                     
                     sh_content = f"""#!/bin/bash
# Replay recording #{i}: {rec.get('name')}
python3 replay_mujoco.py {i}
"""
                     with open(sh_path, "w") as f:
                         f.write(sh_content)
                     os.chmod(sh_path, 0o755)

            except Exception as e:
                 print(f"Error processing MuJoCo recordings: {e}")

        launch_bat = f"""@echo off
python visualize_mjcf.py %*
pause
"""
        with open(os.path.join(package_dir, "launch.bat"), "w") as f:
            f.write(launch_bat)

        # Generate requirements.txt
        # Simplified requirements:
        # 1. mediapipe installs its own opencv-contrib-python. 
        #    Installing 'opencv-python' alongside it causes conflicts.
        # 2. Letting mediapipe pick its own protobuf version (likely 3.19+ or 4.x)
        req_content = "mujoco\nmediapipe==0.10.14\nmatplotlib\nnumpy\n"
        with open(os.path.join(package_dir, "requirements.txt"), "w") as f:
            f.write(req_content)

        # Generate Setup Venv Script (Linux/Mac)
        setup_sh = f"""#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status.

echo "Creating clean virtual environment..."
# --clear ensures we start fresh even if venv exists
python3 -m venv --clear venv

echo "Activating virtual environment..."
source venv/bin/activate

echo "Upgrading pip and installing wheel..."
pip install --upgrade pip wheel

echo "Installing dependencies from requirements.txt..."
# Force re-install without cache to fix any broken downloads
pip install --no-cache-dir -r requirements.txt

echo "--- DEBUG INFO ---"
echo "Python location: $(which python)"
pip list
pip show mujoco
echo "------------------"

echo "Done! You can now run ./run_demo.sh"
"""
        with open(os.path.join(package_dir, "setup_venv.sh"), "w") as f:
            f.write(setup_sh)
        os.chmod(os.path.join(package_dir, "setup_venv.sh"), 0o755)

        # Generate Run Demo Script (Linux/Mac)
        run_demo_sh = f"""#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${{BASH_SOURCE[0]}}" )" &> /dev/null && pwd )"
VENV_PYTHON="$SCRIPT_DIR/venv/bin/python"

if [ ! -f "$VENV_PYTHON" ]; then
    echo "Virtual environment python not found at $VENV_PYTHON"
    echo "Please run ./setup_venv.sh first."
    exit 1
fi

echo "Running with Python: $VENV_PYTHON"
# Debug: Print sys.path to confirm we are in the venv
"$VENV_PYTHON" -c "import sys; print('DEBUG: sys.path is', sys.path)"

"$VENV_PYTHON" "$SCRIPT_DIR/demo_hand_control.py"
"""
        with open(os.path.join(package_dir, "run_demo.sh"), "w") as f:
            f.write(run_demo_sh)
        os.chmod(os.path.join(package_dir, "run_demo.sh"), 0o755)


        # Generate Demo Hand Control Script
        demo_script = f"""

import mujoco
import mujoco.viewer
import cv2
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import sys
import os

# --- Configuration ---
MODEL_PATH = "{mjcf_filename}"

# --- MediaPipe Imports with Robustness ---
try:
    import mediapipe as mp
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
except ImportError:
    print("Standard import failed, trying fallback...")
    try:
        import mediapipe as mp
    except ImportError as e:
        print(f"CRITICAL ERROR: {{e}}")
        print("MediaPipe not found.")
        sys.exit(1)

# Initialize MediaPipe Hands
try:
    if hasattr(mp.solutions, 'hands'):
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
    else:
        # Fallback for some versions
        import mediapipe.python.solutions.hands as mp_hands
        hands = mp_hands.Hands(max_num_hands=2)
except Exception as e:
    print(f"Error initializing MediaPipe: {{e}}")
    sys.exit(1)

# --- MuJoCo Setup ---
if not os.path.exists(MODEL_PATH):
    print(f"Error: Model file '{{MODEL_PATH}}' not found!")
    sys.exit(1)

print(f"Loading model from {{MODEL_PATH}}...")
try:
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading MuJoCo model: {{e}}")
    sys.exit(1)

# --- Smart Actuator Mapping (Per-Finger) ---
finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
actuator_map = {{name: [] for name in finger_names}}
# Fallback for actuators that don't match a specific finger name
actuator_map['general'] = [] 

print("="*60)
print(f"Auto-Mapping Actuators (Total: {{model.nu}}):")

for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    if not name: name = f"actuator_{{i}}"
    lower = name.lower()
    
    # Get Control Range for Debugging
    ctrl_range = model.actuator_ctrlrange[i]
    range_str = f"[{{ctrl_range[0]:.2f}}, {{ctrl_range[1]:.2f}}]"
    
    # Skip Roll/Yaw/Spread joints (hold them steady)
    if 'roll' in lower or 'yaw' in lower or 'spread' in lower:
        print(f"  [ID {{i}}] {{name}} -> IGNORED (Spread/Roll) | Range: {{range_str}}")
        continue

    # STRICT 1st Joint Filter: Only allow PITCH for the first joint (Proximal).
    if '1st' in lower and 'pitch' not in lower:
        print(f"  [ID {{i}}] {{name}} -> IGNORED (1st Joint Non-Pitch) | Range: {{range_str}}")
        continue

    # Assign to finger bucket
    assigned = False
    for fname in finger_names:
        # Check standard name OR alias (little -> pinky)
        if fname in lower or (fname == 'pinky' and 'little' in lower):
            actuator_map[fname].append(i)
            print(f"  [ID {{i}}] {{name}} -> {{fname.upper()}} | Range: {{range_str}}")
            assigned = True
            break
    
    if not assigned:
        actuator_map['general'].append(i)
        print(f"  [ID {{i}}] {{name}} -> GENERAL | Range: {{range_str}}")

print("="*60)

# --- Helper: Calculate Per-Finger Curl ---
def calculate_finger_curl(landmarks, finger_name):
    wrist = landmarks.landmark[mp_hands.HandLandmark.WRIST]
    
    if finger_name == 'thumb':
        # Geometric Angle Calculation (CMC-MCP vs MCP-IP)
        # 1. Get Landmarks
        cmc = landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC]
        mcp = landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP]
        ip  = landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
        
        # 2. Vectors (XY Only - Ignore Z for stability)
        # v1: CMC -> MCP
        v1 = np.array([mcp.x - cmc.x, mcp.y - cmc.y])
        # v2: MCP -> IP
        v2 = np.array([ip.x - mcp.x, ip.y - mcp.y])
        
        # 3. Angle
        # dot product
        dot = np.dot(v1, v2)
        # magnitudes
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        # cos theta, clip for safety
        cos_angle = np.clip(dot / (norm_v1 * norm_v2 + 1e-6), -1.0, 1.0)
        angle_rad = np.arccos(cos_angle)
        
        # 4. Normalize
        # Map 0..PI/2 to 0..1
        curl = np.clip(angle_rad / (np.pi / 2), 0.0, 1.0)
        return curl
    else:
        # Standard fingers
        if finger_name == 'index':
            tip_id = mp_hands.HandLandmark.INDEX_FINGER_TIP
            mcp_id = mp_hands.HandLandmark.INDEX_FINGER_MCP
        elif finger_name == 'middle':
            tip_id = mp_hands.HandLandmark.MIDDLE_FINGER_TIP
            mcp_id = mp_hands.HandLandmark.MIDDLE_FINGER_MCP
        elif finger_name == 'ring':
            tip_id = mp_hands.HandLandmark.RING_FINGER_TIP
            mcp_id = mp_hands.HandLandmark.RING_FINGER_MCP
        elif finger_name == 'pinky':
            tip_id = mp_hands.HandLandmark.PINKY_TIP
            mcp_id = mp_hands.HandLandmark.PINKY_MCP
        else:
            return 0.0

        tip = landmarks.landmark[tip_id]
        mcp = landmarks.landmark[mcp_id]
        
        dist_tip = np.sqrt((tip.x - wrist.x)**2 + (tip.y - wrist.y)**2)
        dist_mcp = np.sqrt((mcp.x - wrist.x)**2 + (mcp.y - wrist.y)**2)
        
        ratio = dist_tip / (dist_mcp + 1e-6)
        
        # Uniform Sensitivity (Reverted Pinky tuning)
        max_ratio = 1.8
        
        curl = np.clip((max_ratio - ratio) / 1.0, 0.0, 1.0)
        return curl

# --- Matplotlib Setup ---
print("Initializing Matplotlib...")
plt.ion()
fig, ax = plt.subplots()
sensor_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i) for i in range(model.nsensor)]
print(f"Found Sensors: {{sensor_names}}")

history_len = 100
sensor_data_history = {{name: deque([0]*history_len, maxlen=history_len) for name in sensor_names}}
lines = {{}}
for name in sensor_names:
    line, = ax.plot(range(history_len), sensor_data_history[name], label=name)
    lines[name] = line

ax.set_ylim(-1, 50) 
ax.legend(loc='upper left')
plt.title("Sensor Force")

# --- Main Loop ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit(1)

print("Starting Loop.")
frame_count = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.flags[1] = 1 # mjVIS_SITE
    viewer.opt.flags[9] = 1 # mjVIS_CONTACTPOINT
    
    while viewer.is_running() and cap.isOpened():
        start_time = time.time()
        frame_count += 1
        
        ret, frame = cap.read()
        if not ret: break
        
        frame = cv2.flip(frame, 1)
        results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        curls = {{f: 0.0 for f in finger_names}}
        curls['general'] = 0.0
        
        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label
                if label != 'Right': continue 

                mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                for fname in finger_names:
                    curls[fname] = calculate_finger_curl(hand_landmarks, fname)
                
                curls['general'] = (curls['index'] + curls['middle']) / 2.0
                break
        
        # Apply Control
        for fname, act_indices in actuator_map.items():
            # Smart Gain Direction based on Joint Limits
            # Curl is 0..1. We want 90 degrees movement.
            # If range is [0, pos], target is pos (curl * 1.57).
            # If range is [neg, 0], target is neg (curl * -1.57).
            
            curl = curls[fname]
            gain_base = 1.57
            
            if fname == 'general':
                cmd_val = curls['general'] * -1.57 # Default general to neg?
                for idx in act_indices:
                    data.ctrl[idx] = cmd_val
                continue

            for idx in act_indices:
                # Per-Joint Direction Logic
                jid = model.actuator_trnid[idx, 0]
                min_val, max_val = model.jnt_range[jid]
                
                sign = -1.0
                if min_val >= 0: sign = 1.0
                elif max_val <= 0: sign = -1.0
                
                # Apply
                val = curl * gain_base * sign
                # Clamp to limits
                val = max(min_val, min(max_val, val))
                data.ctrl[idx] = val
                
        cv2.imshow('Hand Control (Right Hand)', frame)
        if cv2.waitKey(1) & 0xFF == 27: break

        # Sync Physics to Real-Time (Webcam ~30fps, Physics ~2ms step)
        # Cap read blocks for ~33ms. We need multiple physics steps per frame.
        dt_frame = time.time() - start_time
        # Standard webcam is 30Hz-60Hz. If efficient, dt is small, but cap.read blocks.
        # Let's assume at least 33ms passed.
        # Robust approach: Step until simulation time catches up to wall clock, 
        # but reset offset to avoid spiral.
        
        # Simple fixed catch-up: 1 step is too slow. 
        # Approx 33ms / 2ms = 16 steps.
        # We perform a fixed burst to keep it responsive but stable.
        for _ in range(15): 
            mujoco.mj_step(model, data)
            
        viewer.sync()
        
        # Update Plots
        active_sensors = []
        for name in sensor_names:
            val = data.sensor(name).data[0] 
            sensor_data_history[name].append(val)
            lines[name].set_ydata(sensor_data_history[name])
            if val > 0.001:
                active_sensors.append(f"{{name}}={{val:.2f}}")
        
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        
        # Detailed Logging
        if frame_count % 5 == 0:
             print("-" * 60)
             print(f"[Frame {{frame_count}}] MP Status:")
             for fname in finger_names:
                 print(f"  {{fname.upper():<8}} | MP: {{curls[fname]:.2f}} | Cmd: {{curls[fname]* -2.0:.2f}}")
                 
             print("  Robot Joints:")
             any_act = False
             for fname, indices in actuator_map.items():
                 if not indices: continue
                 for idx in indices:
                     any_act = True
                     name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, idx)
                     # Angle
                     joint_id = model.actuator_trnid[idx, 0]
                     qpos_adr = model.jnt_qposadr[joint_id]
                     angle = data.qpos[qpos_adr]
                     cmd_val = data.ctrl[idx]
                     
                     print(f"    > {{name:<20}} | Cmd: {{cmd_val:.2f}} | Ang: {{angle:.2f}}")
                     
             if not any_act:
                 print("    (No actuators mapped!)")

             if active_sensors:
                 print(f"  >>> SENSOR HIT: {{', '.join(active_sensors)}}")
             print("-" * 60)

        time_until_next_step = model.opt.timestep - (time.time() - start_time)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            
cap.release()
cv2.destroyAllWindows()
plt.close()
"""
        with open(os.path.join(package_dir, "demo_hand_control.py"), "w") as f:
            f.write(demo_script)

        # Generate README
        readme_content = f"""
# MuJoCo Visualization (MJCF)

This package contains the **Native MJCF XML** model of **{robot_name}**.

## Prerequisites
- Python 3.8+
- Webcam (for hand control)

## Quick Start (Recommended)

1. **Setup Environment**:
   Run the setup script to create a virtual environment and install dependencies (prevents conflicts):
   ```bash
   ./setup_venv.sh
   ```

2. **Run Demo**:
   Start the hand tracking and simulation:
   ```bash
   ./run_demo.sh
   ```

## Manual Installation

If you prefer to install manually:
```bash
pip install -r requirements.txt
```

## Running the Simulation
Run the visualization script:
```bash
python visualize_mjcf.py
```

### Hand Control Demo
1. Ensure your webcam is connected.
2. Run:
   ```bash
   python demo_hand_control.py
   ```
3. Pinch your thumb and index finger to control the robot.

## Files
- `{mjcf_filename}`: The native MJCF robot description.
- `meshes/`: STL files for geometries.
- `visualize_mjcf.py`: Python script to load and view the model.
- `demo_hand_control.py`: Webcam control and sensor graph demo.
- `setup_venv.sh`: Helper to create venv.
- `run_demo.sh`: Helper to run demo in venv.
"""
        if recordings:
            readme_content += """
## Motion Replay
This package includes exported motion recordings.

### How to Replay
We have generated convenient bash scripts for each recording.
Simply run the script corresponding to the recording you want to play:

```bash
./replay_0_recording_name.sh
```

Alternatively, you can run the python script directly:
```bash
python3 replay_mujoco.py [index]
```
(Where [index] is 0, 1, 2...)
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
