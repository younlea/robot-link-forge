from fastapi import FastAPI, File, UploadFile, HTTPException, Form, BackgroundTasks
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field
from typing import Dict, List, Optional, Tuple, Any
import os
import uuid
import shutil
import json
import tempfile
import re

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
app.mount("/static", StaticFiles(directory="static"), name="static")

# --- Pydantic Models for Robot Data ---
class Visual(BaseModel):
    type: str
    dimensions: Optional[List[float]] = None
    color: Optional[str] = None
    meshUrl: Optional[str] = Field(None, alias='meshUrl')
    meshScale: Optional[List[float]] = Field(None, alias='meshScale')
    meshOrigin: Optional[Dict[str, List[float]]] = Field(None, alias='meshOrigin')

class RobotLink(BaseModel):
    id: str
    name: str
    visual: Visual
    childJoints: List[str] = Field(alias='childJoints')

class JointDOF(BaseModel):
    roll: bool
    pitch: bool
    yaw: bool

class JointLimit(BaseModel):
    lower: float
    upper: float

class JointOrigin(BaseModel):
    xyz: List[float]
    rpy: List[float]

class RobotJoint(BaseModel):
    id: str
    name: str
    parentLinkId: str = Field(alias='parentLinkId')
    childLinkId: Optional[str] = Field(None, alias='childLinkId')
    type: str
    dof: JointDOF
    axis: Optional[List[float]] = None
    limits: Dict[str, JointLimit]
    origin: JointOrigin

class RobotData(BaseModel):
    links: Dict[str, RobotLink]
    joints: Dict[str, RobotJoint]
    baseLinkId: str = Field(alias='baseLinkId')

# --- Helper Functions ---
def to_snake_case(name: str) -> str:
    """Converts a string to snake_case and ensures it's a valid identifier."""
    s = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    s = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s).lower()
    s = re.sub(r'\W+', '_', s) # Replace non-alphanumeric with _
    s = re.sub(r'^_|_$', '', s) # Remove leading/trailing _
    return s if s else "link"


def generate_urdf_xml(robot_data: RobotData, robot_name: str, mesh_files: Dict[str, str]) -> str:
    """Generates the URDF XML content as a string."""
    
    links_xml = f'  <link name="{to_snake_case(robot_data.links[robot_data.baseLinkId].name)}"/>\n'
    joints_xml = ""

    processed_links = {robot_data.baseLinkId}
    
    # Traverse from base link
    q = [robot_data.baseLinkId]
    while q:
        parent_link_id = q.pop(0)
        parent_link = robot_data.links[parent_link_id]

        for joint_id in parent_link.childJoints:
            joint = robot_data.joints.get(joint_id)
            if not joint or not joint.childLinkId:
                continue

            child_link_id = joint.childLinkId
            if child_link_id in processed_links:
                continue
            
            child_link = robot_data.links[child_link_id]
            processed_links.add(child_link_id)
            q.append(child_link_id)

            # --- Link XML ---
            links_xml += f'  <link name="{to_snake_case(child_link.name)}">\n'
            links_xml += '    <visual>\n'
            links_xml += f'      <origin xyz="0 0 0" rpy="0 0 0" />\n'
            links_xml += '      <geometry>\n'
            
            vis = child_link.visual
            if vis.type == 'mesh' and child_link_id in mesh_files:
                mesh_origin_xyz = " ".join(map(str, vis.meshOrigin['xyz'])) if vis.meshOrigin and 'xyz' in vis.meshOrigin else "0 0 0"
                mesh_origin_rpy = " ".join(map(str, vis.meshOrigin['rpy'])) if vis.meshOrigin and 'rpy' in vis.meshOrigin else "0 0 0"
                mesh_scale = " ".join(map(str, vis.meshScale)) if vis.meshScale else "1 1 1"
                
                links_xml = links_xml.replace('<origin xyz="0 0 0" rpy="0 0 0" />', f'<origin xyz="{mesh_origin_xyz}" rpy="{mesh_origin_rpy}" />')
                links_xml += f'        <mesh filename="package://{robot_name}/meshes/{mesh_files[child_link_id]}" scale="{mesh_scale}"/>\n'

            elif vis.type == 'box' and vis.dimensions:
                links_xml += f'        <box size="{" ".join(map(str, vis.dimensions))}" />\n'
            elif vis.type == 'cylinder' and vis.dimensions:
                links_xml += f'        <cylinder radius="{vis.dimensions[0]}" length="{vis.dimensions[2]}" />\n'
            elif vis.type == 'sphere' and vis.dimensions:
                links_xml += f'        <sphere radius="{vis.dimensions[0]}" />\n'
            else:
                 links_xml += '        <box size="0.01 0.01 0.01" />\n'

            links_xml += '      </geometry>\n'
            if vis.color:
                r, g, b = int(vis.color[1:3], 16)/255, int(vis.color[3:5], 16)/255, int(vis.color[5:7], 16)/255
                links_xml += f'      <material name="{to_snake_case(child_link.name)}_color">\n'
                links_xml += f'        <color rgba="{r:.3f} {g:.3f} {b:.3f} 1.0" />\n'
                links_xml += '      </material>\n'
            links_xml += '    </visual>\n'
            links_xml += links_xml.replace('<visual>', '<collision>').replace('</visual>', '</collision>')
            links_xml += '  </link>\n\n'


            # --- Joint XML ---
            joint_type = "revolute" if joint.type == 'rotational' else joint.type
            joints_xml += f'  <joint name="{to_snake_case(joint.name)}" type="{joint_type}">\n'
            joints_xml += f'    <parent link="{to_snake_case(parent_link.name)}"/>\n'
            joints_xml += f'    <child link="{to_snake_case(child_link.name)}"/>\n'
            joints_xml += f'    <origin xyz="{" ".join(map(str, joint.origin.xyz))}" rpy="{" ".join(map(str, joint.origin.rpy))}"/>\n'
            
            if joint.type == 'rotational':
                active_dof = None
                if joint.dof.roll:
                    active_dof = 'roll'
                    axis_str = "1 0 0"
                elif joint.dof.pitch:
                    active_dof = 'pitch'
                    axis_str = "0 1 0"
                elif joint.dof.yaw:
                    active_dof = 'yaw'
                    axis_str = "0 0 1"

                if active_dof:
                    joints_xml += f'    <axis xyz="{axis_str}"/>\n'
                    limit = joint.limits[active_dof]
                    joints_xml += f'    <limit lower="{limit.lower}" upper="{limit.upper}" effort="10" velocity="1.0"/>\n'

            elif joint.type == 'prismatic':
                axis_str = " ".join(map(str, joint.axis)) if joint.axis else "1 0 0"
                joints_xml += f'    <axis xyz="{axis_str}"/>\n'
                limit = joint.limits['displacement']
                joints_xml += f'    <limit lower="{limit.lower}" upper="{limit.upper}" effort="10" velocity="1.0"/>\n'

            joints_xml += '  </joint>\n\n'

    return f'<robot name="{robot_name}">\n{links_xml}{joints_xml}</robot>'


# --- API Endpoints ---
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

    # Process and save mesh files
    mesh_files_map = {}
    if files:
        for file in files:
            form_field_name = file.name
            link_id = form_field_name.replace('mesh_', '')

            if link_id in robot.links:
                safe_filename = to_snake_case(robot.links[link_id].name) + ".stl"
                file_path = os.path.join(mesh_dir, safe_filename)
                
                with open(file_path, "wb") as f:
                    shutil.copyfileobj(file.file, f)
                
                mesh_files_map[link_id] = safe_filename

    # Generate and save URDF file
    urdf_content = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map)
    with open(os.path.join(urdf_dir, f"{sanitized_robot_name}.urdf"), "w") as f:
        f.write(urdf_content)

    # Create other package files
    package_xml = f"""<package format="2">\n<name>{sanitized_robot_name}</name>\n<version>0.1.0</version>\n<description>A description of {sanitized_robot_name}</description>\n<maintainer email="user@example.com">Your Name</maintainer>\n<license>MIT</license>\n<buildtool_depend>catkin</buildtool_depend>\n<depend>roslaunch</depend>\n<depend>robot_state_publisher</depend>\n<depend>rviz</depend>\n<depend>joint_state_publisher_gui</depend>\n</package>\n"""
    with open(os.path.join(package_dir, "package.xml"), "w") as f:
        f.write(package_xml)

    cmakelists_txt = f"""cmake_minimum_required(VERSION 3.0.2)\nproject({sanitized_robot_name})\nfind_package(catkin REQUIRED COMPONENTS roslaunch robot_state_publisher rviz joint_state_publisher_gui)\ncatkin_package()\n"""
    with open(os.path.join(package_dir, "CMakeLists.txt"), "w") as f:
        f.write(cmakelists_txt)

    display_launch = f"""<launch>\n<arg name="model" default="$(find {sanitized_robot_name})/urdf/{sanitized_robot_name}.urdf"/>\n<arg name="gui" default="true" />\n<param name="robot_description" textfile="$(arg model)" />\n<node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />\n<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />\n<node name="rviz" pkg="rviz" type="rviz" args="-d $(find {sanitized_robot_name})/launch/display.rviz" required="true" />\n</launch>\n"""
    with open(os.path.join(launch_dir, "display.launch"), "w") as f:
        f.write(display_launch)
    
    rviz_config = """Panels:
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
      Class: rviz/Grid
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
    - Class: rviz/RobotModel
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
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
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
        base_name=os.path.join(tmpdir, sanitized_robot_name), # e.g., /tmp/xyz/my_robot
        format='zip',
        root_dir=tmpdir,           # The archive will be rooted at /tmp/xyz
        base_dir=sanitized_robot_name # The folder to zip is `my_robot`
    )
    
    # Add a background task to clean up the temporary directory after the response is sent
    background_tasks.add_task(shutil.rmtree, tmpdir)
    
    # Return the zip file from disk
    return FileResponse(
        path=archive_path, 
        media_type='application/zip', 
        filename=f"{sanitized_robot_name}_ros_package.zip"
    )


@app.get("/")
def read_root():
    """
    Root endpoint for basic API health check.
    """
    return {"message": "RobotLinkForge Backend is running."}
