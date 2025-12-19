from fastapi import FastAPI, File, UploadFile, HTTPException, Form
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from typing import Dict, List, Optional, Tuple, Any
import os
import uuid
import shutil
import json
import zipfile
import io
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
    limit: Optional[JointLimit] = None
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
            links_xml += f'      <origin xyz="0 0 0" rpy="0 0 0" />\n' # Visual origin is relative to link origin
            links_xml += '      <geometry>\n'
            
            vis = child_link.visual
            if vis.type == 'mesh' and child_link_id in mesh_files:
                mesh_origin_xyz = " ".join(map(str, vis.meshOrigin['xyz'])) if vis.meshOrigin and 'xyz' in vis.meshOrigin else "0 0 0"
                mesh_origin_rpy = " ".join(map(str, vis.meshOrigin['rpy'])) if vis.meshOrigin and 'rpy' in vis.meshOrigin else "0 0 0"
                mesh_scale = " ".join(map(str, vis.meshScale)) if vis.meshScale else "1 1 1"
                
                # URDF visual origin needs to combine mesh origin from app and geometry offset
                links_xml = links_xml.replace('<origin xyz="0 0 0" rpy="0 0 0" />', f'<origin xyz="{mesh_origin_xyz}" rpy="{mesh_origin_rpy}" />')
                links_xml += f'        <mesh filename="package://{robot_name}/meshes/{mesh_files[child_link_id]}" scale="{mesh_scale}"/>\n'

            elif vis.type == 'box' and vis.dimensions:
                links_xml += f'        <box size="{" ".join(map(str, vis.dimensions))}" />\n'
            elif vis.type == 'cylinder' and vis.dimensions:
                links_xml += f'        <cylinder radius="{vis.dimensions[0]}" length="{vis.dimensions[2]}" />\n'
            elif vis.type == 'sphere' and vis.dimensions:
                links_xml += f'        <sphere radius="{vis.dimensions[0]}" />\n'
            else:
                 links_xml += '        <box size="0.01 0.01 0.01" />\n' # Placeholder for 'none' type

            links_xml += '      </geometry>\n'
            if vis.color:
                r, g, b = int(vis.color[1:3], 16)/255, int(vis.color[3:5], 16)/255, int(vis.color[5:7], 16)/255
                links_xml += f'      <material name="{to_snake_case(child_link.name)}_color">\n'
                links_xml += f'        <color rgba="{r:.3f} {g:.3f} {b:.3f} 1.0" />\n'
                links_xml += '      </material>\n'
            links_xml += '    </visual>\n'
            # Add collision block (simplified copy of visual)
            links_xml += links_xml.replace('<visual>', '<collision>').replace('</visual>', '</collision>')
            links_xml += '  </link>\n\n'


            # --- Joint XML ---
            joint_type = "revolute" if joint.type == 'rotational' else joint.type
            joints_xml += f'  <joint name="{to_snake_case(joint.name)}" type="{joint_type}">\n'
            joints_xml += f'    <parent link="{to_snake_case(parent_link.name)}"/>\n'
            joints_xml += f'    <child link="{to_snake_case(child_link.name)}"/>\n'
            joints_xml += f'    <origin xyz="{" ".join(map(str, joint.origin.xyz))}" rpy="{" ".join(map(str, joint.origin.rpy))}"/>\n'
            
            if joint.type != 'fixed':
                axis_str = " ".join(map(str, joint.axis)) if joint.axis else "0 0 1"
                joints_xml += f'    <axis xyz="{axis_str}"/>\n'
                if joint.limit:
                    joints_xml += f'    <limit lower="{joint.limit.lower}" upper="{joint.limit.upper}" effort="10" velocity="1.0"/>\n'
            
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
async def export_urdf_package(robot_data: str = Form(...), files: List[UploadFile] = File(...)):
    try:
        data_dict = json.loads(robot_data)
        robot = RobotData.parse_obj(data_dict)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid robot data format: {e}")

    robot_name = to_snake_case(robot.links[robot.baseLinkId].name)
    if not robot_name:
        robot_name = "my_robot"
    
    with tempfile.TemporaryDirectory() as tmpdir:
        package_dir = os.path.join(tmpdir, robot_name)
        urdf_dir = os.path.join(package_dir, "urdf")
        mesh_dir = os.path.join(package_dir, "meshes")
        launch_dir = os.path.join(package_dir, "launch")

        os.makedirs(urdf_dir, exist_ok=True)
        os.makedirs(mesh_dir, exist_ok=True)
        os.makedirs(launch_dir, exist_ok=True)

        # Process and save mesh files
        mesh_files_map = {}
        for file in files:
            # Extract linkId from `mesh_<linkId>` form field name
            match = re.match(r"mesh_(link-\w+)", file.filename)
            # Fallback to just the filename if no match (less reliable)
            link_id_from_name = file.filename.split('_', 1)[1] if '_' in file.filename else None
            
            # The name from FormData is in `file.name`. We need to parse that.
            # The actual filename is `file.filename`
            form_field_name = file.name # This should be `mesh_<linkId>`
            link_id = form_field_name.replace('mesh_', '')

            if link_id in robot.links:
                # Sanitize filename
                safe_filename = to_snake_case(robot.links[link_id].name) + ".stl"
                file_path = os.path.join(mesh_dir, safe_filename)
                
                with open(file_path, "wb") as f:
                    shutil.copyfileobj(file.file, f)
                
                mesh_files_map[link_id] = safe_filename

        # Generate and save URDF file
        urdf_content = generate_urdf_xml(robot, robot_name, mesh_files_map)
        with open(os.path.join(urdf_dir, f"{robot_name}.urdf"), "w") as f:
            f.write(urdf_content)

        # Create package.xml
        package_xml = f"""<package format="2">
  <name>{robot_name}</name>
  <version>0.1.0</version>
  <description>A description of {robot_name}</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roslaunch</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz</depend>
  <depend>joint_state_publisher_gui</depend>
</package>
"""
        with open(os.path.join(package_dir, "package.xml"), "w") as f:
            f.write(package_xml)

        # Create CMakeLists.txt
        cmakelists_txt = f"""cmake_minimum_required(VERSION 3.0.2)
project({robot_name})
find_package(catkin REQUIRED COMPONENTS roslaunch robot_state_publisher rviz joint_state_publisher_gui)
catkin_package()
"""
        with open(os.path.join(package_dir, "CMakeLists.txt"), "w") as f:
            f.write(cmakelists_txt)

        # Create display.launch
        display_launch = f"""<launch>
  <arg name="model" default="$(find {robot_name})/urdf/{robot_name}.urdf"/>
  <arg name="gui" default="true" />
  <param name="robot_description" textfile="$(arg model)" />
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find {robot_name})/launch/display.rviz" required="true" />
</launch>
"""
        with open(os.path.join(launch_dir, "display.launch"), "w") as f:
            f.write(display_launch)

        # Create default display.rviz
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

        # Copy the guide file into the package
        guide_src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'URDF_export.md'))
        if os.path.exists(guide_src_path):
            shutil.copy(guide_src_path, os.path.join(package_dir, 'URDF_export_guide.md'))

        # Zip the directory
        shutil.make_archive(os.path.join(tmpdir, robot_name), 'zip', package_dir)
        
        # Stream the zip file back
        zip_path = os.path.join(tmpdir, f"{robot_name}.zip")
        
        # Use a BytesIO object to hold the zip file in memory
        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, _, files_in_dir in os.walk(package_dir):
                for file_in_dir in files_in_dir:
                    file_path = os.path.join(root, file_in_dir)
                    arcname = os.path.relpath(file_path, package_dir)
                    zipf.write(file_path, arcname)

        zip_buffer.seek(0)
        return StreamingResponse(zip_buffer, media_type="application/zip", headers={"Content-Disposition": f"attachment; filename={robot_name}_ros_package.zip"})


@app.get("/")
def read_root():
    return {"message": "RobotLinkForge Backend is running."}
