from fastapi import FastAPI, File, UploadFile, HTTPException, Form, BackgroundTasks, Request
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
import math
from datetime import datetime

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
    visual: Optional[Visual] = None

class RobotData(BaseModel):
    links: Dict[str, RobotLink]
    joints: Dict[str, RobotJoint]
    baseLinkId: str = Field(alias='baseLinkId')

# --- Helper Functions ---
def to_snake_case(name: str) -> str:
    """Converts a string to snake_case and ensures it's a valid identifier."""
    # Ensure it's a string
    name = str(name).strip()
    # Explicitly replace spaces first to be sure
    name = name.replace(' ', '_')
    
    s = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    s = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s).lower()
    s = re.sub(r'\W+', '_', s) # Replace non-alphanumeric with _
    s = re.sub(r'_+', '_', s) # Dedupe underscores
    s = re.sub(r'^_|_$', '', s) # Remove leading/trailing _
    return s if s else "link"


def calculate_cylinder_transform(start_point: List[float], end_point: List[float]) -> Tuple[float, List[float], List[float]]:
    """
    Calculates the length, midpoint (origin), and rotation (rpy) to align a cylinder 
    from start_point to end_point.
    Returns: (length, origin_xyz, origin_rpy)
    """
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    dz = end_point[2] - start_point[2]
    
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    # Midpoint
    mx = (start_point[0] + end_point[0]) / 2.0
    my = (start_point[1] + end_point[1]) / 2.0
    mz = (start_point[2] + end_point[2]) / 2.0
    
    # Rotation (Align Z-axis to vector D)
    # Pitch (Y) and Yaw (Z). Roll (X) is 0.
    
    # Yaw: atan2(dy, dx)
    yaw = math.atan2(dy, dx)
    
    # Horizontal projection length
    horizontal_dist = math.sqrt(dx*dx + dy*dy)
    
    # Pitch: angle between Z and vector.
    # Positive pitch around Y moves Z toward X.
    # If dx=1, dz=0 (Forward X), we want pitch +90.
    # pitch = atan2(horizontal_dist, dz) gives 90 for (1,0) and 0 for (0,1). Correct.
    pitch = math.atan2(horizontal_dist, dz)
    
    return length, [mx, my, mz], [0.0, pitch, yaw]


def euler_xyz_to_rotation_matrix(x, y, z):
    """
    Constructs a rotation matrix from Euler angles (Intrinsic X-Y-Z order).
    This matches the Three.js 'XYZ' Euler order used in the frontend.
    """
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    
    # R = R_x * R_y * R_z
    # (Row 0): [ cy*cz,              -cy*sz,             sy     ]
    # (Row 1): [ cx*sz + sx*sy*cz,   cx*cz - sx*sy*sz,   -sx*cy ]
    # (Row 2): [ sx*sz - cx*sy*cz,   sx*cz + cx*sy*sz,   cx*cy  ]
    
    R = [
        [cy*cz,              -cy*sz,              sy],
        [cx*sz + sx*sy*cz,   cx*cz - sx*sy*sz,   -sx*cy],
        [sx*sz - cx*sy*cz,   sx*cz + cx*sy*sz,   cx*cy]
    ]
    return R

def rotation_matrix_to_euler_zyx(R):
    """
    Decomposes Rotation Matrix into Euler ZYX (Intrinsic) / XYZ (Fixed).
    Matches URDF <origin rpy="r p y" /> which expects Fixed Axis rotations.
    """
    r20 = R[2][0]
    r21 = R[2][1]
    r22 = R[2][2]
    r10 = R[1][0]
    r00 = R[0][0]
    
    # Pitch (Y axis rotation) - R20 = -sin(p)
    if r20 < 1.0:
        if r20 > -1.0:
            p = math.asin(-r20)
            r = math.atan2(r21, r22)
            y = math.atan2(r10, r00)
        else:
             # Gimbal lock: p = 90deg
            p = math.pi / 2
            y = 0
            r = math.atan2(R[0][1], R[0][2])
    else:
        # Gimbal lock: p = -90deg
        p = -math.pi / 2
        y = 0
        r = math.atan2(-R[1][2], R[1][1])

    return [r, p, y]

def convert_euler_xyz_to_zyx(rpy):
    """
    Converts a list/tuple of [roll, pitch, yaw] from Intrinsic XYZ (Frontend)
    to Intrinsic ZYX (URDF Standard).
    """
    if not rpy or len(rpy) != 3:
        return [0.0, 0.0, 0.0]
    R = euler_xyz_to_rotation_matrix(rpy[0], rpy[1], rpy[2])
    return rotation_matrix_to_euler_zyx(R)



def _generate_link_xml(link_id: str, link_name: str, robot_data: RobotData, robot_name: str, mesh_files: Dict[str, str], for_mujoco: bool = False) -> str:
    """Generates the XML content for a single link (visual, collision, etc.)."""
    
    link: RobotLink = robot_data.links[link_id]
    
    
    # --- Inertial XML (Required for MuJoCo if collision is missing) ---
    # We add a default inertial block to ensuring the body has mass.
    inertial_xml = '    <inertial>\n'
    inertial_xml += '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
    inertial_xml += '      <mass value="1.0"/>\n'
    inertial_xml += '      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>\n'
    inertial_xml += '    </inertial>\n'

    # --- Visual and Collision XML ---
    visual_xml = '    <visual>\n'
    
    origin_xyz_str = "0 0 0"
    origin_rpy_str = "0 0 0"

    vis = link.visual
    
    # Apply origin if present (regardless of type)
    if vis.meshOrigin:
        origin_xyz_str = " ".join(map(str, vis.meshOrigin.get('xyz', [0,0,0])))
        origin_rpy_str = " ".join(map(str, vis.meshOrigin.get('rpy', [0,0,0])))

    geometry_xml = ""
    override_origin_xyz = None
    override_origin_rpy = None

    # Check for Dynamic Cylinder Case (Connecting Link)
    if vis.type == 'cylinder' and link.childJoints and len(link.childJoints) == 1:
        child_joint_id = link.childJoints[0]
        child_joint = robot_data.joints.get(child_joint_id)
        
        # Only apply dynamic transform if valid connection exists AND user hasn't manually set origin
        if child_joint and child_joint.origin and not vis.meshOrigin:
            length, mid_xyz, mid_rpy = calculate_cylinder_transform(
                [0,0,0], # Start at parent link origin (0,0,0)
                child_joint.origin.xyz
            )
            
            # If length is tiny, might want to fallback, but trust it if > 0.001
            if length > 0.001:
                # Override the Radius from dimensions (index 0)
                radius = vis.dimensions[0] if (vis.dimensions and len(vis.dimensions) > 0) else 0.05
                # Override calculated length
                geometry_xml = f'        <cylinder radius="{radius}" length="{length}" />\n'
                
                # Override Origin
                override_origin_xyz = " ".join(map(str, mid_xyz))
                override_origin_rpy = " ".join(map(str, mid_rpy))

    if not geometry_xml:
        if vis.type == 'mesh' and link_id in mesh_files:
            mesh_scale = " ".join(map(str, vis.meshScale)) if vis.meshScale else "1 1 1"
            geometry_xml = f'        <mesh filename="package://{robot_name}/meshes/{mesh_files[link_id]}" scale="{mesh_scale}"/>\n'
        elif vis.type == 'box' and vis.dimensions:
            geometry_xml = f'        <box size="{" ".join(map(str, vis.dimensions))}" />\n'
        elif vis.type == 'cylinder' and vis.dimensions:
            # FIX: Index 1 is length, not 2
            length_val = vis.dimensions[1] if len(vis.dimensions) > 1 else 1.0
            geometry_xml = f'        <cylinder radius="{vis.dimensions[0]}" length="{length_val}" />\n'
        elif vis.type == 'sphere' and vis.dimensions:
            geometry_xml = f'        <sphere radius="{vis.dimensions[0]}" />\n'
        else:
            # Fallback or default
            geometry_xml = '        <box size="0.01 0.01 0.01" />\n'

    # Determine Final Origin
    origin_xyz_str = "0 0 0"
    origin_rpy_str = "0 0 0"

    # Priority 1: Dynamic Calculation (Override)
    if override_origin_xyz:
        origin_xyz_str = override_origin_xyz
        origin_rpy_str = override_origin_rpy
    # Priority 2: Manual meshOrigin from visual properties (only if not overridden)
    elif vis.meshOrigin:
        origin_xyz_str = " ".join(map(str, vis.meshOrigin.get('xyz', [0,0,0])))
        # Convert Frontend XYZ Euler to URDF ZYX Euler
        raw_rpy = vis.meshOrigin.get('rpy', [0,0,0])
        converted_rpy = convert_euler_xyz_to_zyx(raw_rpy)
        origin_rpy_str = " ".join(map(str, converted_rpy))
    elif vis.type == 'cylinder' and not override_origin_xyz:
         # Fix for Three.js (Y-up) vs URDF (Z-up) cylinder mismatch
         origin_rpy_str = "-1.570796 0 0"

    visual_xml += f'      <origin xyz="{origin_xyz_str}" rpy="{origin_rpy_str}" />\n'
    visual_xml += '      <geometry>\n'
    visual_xml += geometry_xml
    visual_xml += '      </geometry>\n'

    if vis.color:
        try:
            r, g, b = int(vis.color[1:3], 16)/255, int(vis.color[3:5], 16)/255, int(vis.color[5:7], 16)/255
            visual_xml += f'      <material name="{link_name}_color">\n'
            visual_xml += f'        <color rgba="{r:.3f} {g:.3f} {b:.3f} 1.0" />\n'
            visual_xml += '      </material>\n'
        except Exception:
            pass # Handle invalid color hex gracefully
    
    visual_xml += '    </visual>\n'
    
    # COLLISION LOGIC
    if for_mujoco:
        # MuJoCo Stability Fix:
        # We disable collision entirely for the export to prevent "explosions" due to 
        # overlapping default collision boxes at joints (since we use a default box if no mesh).
        # Users mainly use this for visualization or can add custom collision meshes later.
        collision_xml = ''
    else:
        # Standard: Duplicate Visual as Collision
        collision_xml = visual_xml.replace('<visual>', '<collision>', 1).replace('</visual>', '</collision>', 1)

    # --- Link XML ---
    link_xml = f'  <link name="{link_name}">\n'
    link_xml += inertial_xml
    link_xml += visual_xml
    link_xml += collision_xml

    # --- Joint Visuals (Attached to Parent Link) ---
    for jid in link.childJoints:
        joint = robot_data.joints.get(jid)
        if joint and joint.visual and joint.visual.type != 'none':
            # Use Joint Origin as the base placement for the visual
            j_xyz = " ".join(map(str, joint.origin.xyz))
            # Convert Joint RPY (XYZ) to URDF (ZYX)
            j_rpy_zyx = convert_euler_xyz_to_zyx(joint.origin.rpy)
            j_rpy = " ".join(map(str, j_rpy_zyx))
            
            # Check for mesh file
            j_geom = ""
            j_vis = joint.visual
            
            if j_vis.type == 'mesh' and joint.id in mesh_files:
                j_scale = " ".join(map(str, j_vis.meshScale)) if j_vis.meshScale else "1 1 1"
                j_geom = f'        <mesh filename="package://{robot_name}/meshes/{mesh_files[joint.id]}" scale="{j_scale}"/>\n'
            elif j_vis.type == 'box' and j_vis.dimensions:
                j_geom = f'        <box size="{" ".join(map(str, j_vis.dimensions))}" />\n'
            elif j_vis.type == 'cylinder' and j_vis.dimensions:
                l_val = j_vis.dimensions[1] if len(j_vis.dimensions) > 1 else 1.0
                j_geom = f'        <cylinder radius="{j_vis.dimensions[0]}" length="{l_val}" />\n'
            elif j_vis.type == 'sphere' and j_vis.dimensions:
                j_geom = f'        <sphere radius="{j_vis.dimensions[0]}" />\n'
                
            if j_geom:
                j_visual_xml = f'    <visual>\n'
                j_visual_xml += f'      <origin xyz="{j_xyz}" rpy="{j_rpy}" />\n'
                j_visual_xml += f'      <geometry>\n{j_geom}      </geometry>\n'
                
                if j_vis.color:
                    try:
                        r, g, b = int(j_vis.color[1:3], 16)/255, int(j_vis.color[3:5], 16)/255, int(j_vis.color[5:7], 16)/255
                        j_visual_xml += f'      <material name="{joint.name}_color">\n'
                        j_visual_xml += f'        <color rgba="{r:.3f} {g:.3f} {b:.3f} 1.0" />\n'
                        j_visual_xml += '      </material>\n'
                    except: pass
                    
                j_visual_xml += '    </visual>\n'
                
                # Optional: Add collision for joint visual too? 
                # Usually motors have collision. Let's add it.
                j_collision_xml = j_visual_xml.replace('<visual>', '<collision>').replace('</visual>', '</collision>')
                
                link_xml += j_visual_xml
                link_xml += j_collision_xml

    link_xml += '  </link>\n\n'
    
    return link_xml


def generate_unique_names(robot_data: RobotData) -> Dict[str, str]:
    """Generates a mapping of link_id -> unique_clean_name."""
    unique_link_names = {}
    used_names = set()

    # Pre-sort IDs to ensure deterministic order (e.g. by creation time if implied, or just stable ID sort)
    # Since IDs are UUIDs, sorting by them is effectively random but stable.
    # Ideally we'd traverse the tree, but flat list is fine for uniqueness.
    # Tree traversal order is better for "link_1, link_2" numbering to match hierarchy.
    
    # Do a BFS traversal to give meaningful sequential numbers
    sorted_ids = []
    q = [robot_data.baseLinkId]
    visited = {robot_data.baseLinkId}
    while q:
        curr = q.pop(0)
        sorted_ids.append(curr)
        if curr in robot_data.links:
             for child_joint in robot_data.links[curr].childJoints:
                 joint = robot_data.joints.get(child_joint)
                 if joint and joint.childLinkId and joint.childLinkId not in visited:
                     visited.add(joint.childLinkId)
                     q.append(joint.childLinkId)
    
    # Add any disconnected links (if any) at the end
    for lid in robot_data.links:
        if lid not in visited:
            sorted_ids.append(lid)

    for link_id in sorted_ids:
        link = robot_data.links[link_id]
        base_name = to_snake_case(link.name)
        if not base_name: base_name = "link"
        
        final_name = base_name
        counter = 1
        while final_name in used_names:
            final_name = f"{base_name}_{counter}"
            counter += 1
        
        unique_link_names[link_id] = final_name
        used_names.add(final_name)
    
    return unique_link_names

def generate_urdf_xml(robot_data: RobotData, robot_name: str, mesh_files: Dict[str, str], unique_link_names: Dict[str, str], for_mujoco: bool = False) -> Tuple[str, str]:
    """Generates the URDF XML content and returns (urdf_string, base_link_name)."""

    if robot_data.baseLinkId not in unique_link_names:
         raise ValueError(f"Base Link ID {robot_data.baseLinkId} not found in links map.")

    base_link_name = unique_link_names[robot_data.baseLinkId]
    
    # Generate XML for the base link
    links_xml = _generate_link_xml(robot_data.baseLinkId, base_link_name, robot_data, robot_name, mesh_files, for_mujoco=for_mujoco)
    
    joints_xml = ""

    processed_links = {robot_data.baseLinkId}
    processed_joints = set()
    processed_links = {robot_data.baseLinkId}
    processed_joints = set()
    generated_joint_names = set()
    generated_joints_info = [] # Store info for actuators
    
    # Traverse from base link
    q = [robot_data.baseLinkId]
    while q:
        parent_link_id = q.pop(0)
        # Verify parent exists (it should if it was in queue)
        if parent_link_id not in unique_link_names:
             continue
        parent_link_name = unique_link_names[parent_link_id]
        # Ensure childJoints is iterable
        child_joints = robot_data.links[parent_link_id].childJoints or []
        
        for joint_id in child_joints:
            if joint_id in processed_joints:
                continue
            
            joint = robot_data.joints.get(joint_id)
            # Check for missing child link in JOINT definition
            if not joint or not joint.childLinkId:
                continue
            
            # CRITICAL CHECK: Does the child link actually EXIST?
            child_link_id = joint.childLinkId
            if child_link_id not in unique_link_names:
                 print(f"Warning: Joint {joint_id} points to non-existent Child Link {child_link_id}")
                 continue

            processed_joints.add(joint_id)
            
            if child_link_id in processed_links:
                continue
            
            child_link_name = unique_link_names[child_link_id]
            processed_links.add(child_link_id)
            q.append(child_link_id)

            # --- Link XML for Child ---
            links_xml += _generate_link_xml(child_link_id, child_link_name, robot_data, robot_name, mesh_files, for_mujoco=for_mujoco)

            # Determine Active DOFs
            active_rotations = []
            if joint.type == 'rotational':
                if joint.dof.roll: active_rotations.append(('roll', '1 0 0', joint.limits['roll']))
                if joint.dof.pitch: active_rotations.append(('pitch', '0 1 0', joint.limits['pitch']))
                if joint.dof.yaw: active_rotations.append(('yaw', '0 0 1', joint.limits['yaw']))
            
            # Identify Prismatic vs Fixed loops for generic handling
            sub_joints = []
            if joint.type == 'rotational' and active_rotations:
                # Multi-DOF (or Single-DOF) Rotational
                for dof_name, axis_val, limit_val in active_rotations:
                    sub_joints.append({
                        'type': 'revolute',
                        'suffix': dof_name,
                        'axis': axis_val,
                        'limit': limit_val
                    })
            elif joint.type == 'prismatic':
                 axis_str = " ".join(map(str, joint.axis)) if joint.axis else "1 0 0"
                 sub_joints.append({
                     'type': 'prismatic',
                     'suffix': 'prism',
                     'axis': axis_str,
                     'limit': joint.limits['displacement']
                 })
            else:
                 # Fixed or empty rotational
                 sub_joints.append({
                     'type': 'fixed',
                     'suffix': 'fixed',
                     'axis': '0 0 0',
                     'limit': None
                 })

            # Base name for constructing unique joint names
            current_parent_link = parent_link_name
            base_joint_unique_id = f"{to_snake_case(joint.name)}_{joint.id[:6]}"
            
            # Iterate through sub-joints (Decomposition)
            for i, sub in enumerate(sub_joints):
                is_last = (i == len(sub_joints) - 1)
                is_first = (i == 0)
                
                # Determine Name
                raw_name = f"{base_joint_unique_id}_{sub['suffix']}"
                # Ensure uniqueness globally
                final_joint_name = raw_name
                ctr = 1
                while final_joint_name in generated_joint_names:
                    final_joint_name = f"{raw_name}_{ctr}"
                    ctr += 1
                generated_joint_names.add(final_joint_name)
                
                # Determine Child Link
                if is_last:
                    current_child_link = child_link_name
                else:
                    # Create Dummy Link
                    dummy_link_name = f"dummy_{final_joint_name}_link"
                    links_xml += f'  <link name="{dummy_link_name}">\n'
                    links_xml += f'    <inertial><mass value="0.001"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>\n'
                    links_xml += f'  </link>\n'
                    current_child_link = dummy_link_name

                # XML Generation
                joints_xml += f'  <joint name="{final_joint_name}" type="{sub["type"]}">\n'
                joints_xml += f'    <parent link="{current_parent_link}"/>\n'
                joints_xml += f'    <child link="{current_child_link}"/>\n'
                
                # Origin: Only the First joint uses the structural origin. Others are 0 0 0.
                if is_first:
                    converted_origin_rpy = convert_euler_xyz_to_zyx(joint.origin.rpy)
                    joints_xml += f'    <origin xyz="{" ".join(map(str, joint.origin.xyz))}" rpy="{" ".join(map(str, converted_origin_rpy))}"/>\n'
                else:
                    joints_xml += f'    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                
                # Axis & Limit (if applicable)
                if sub['type'] != 'fixed':
                    joints_xml += f'    <axis xyz="{sub["axis"]}"/>\n'
                    # Store for actuator generation
                    generated_joints_info.append({
                        'name': final_joint_name,
                        'type': sub['type'],
                        'limit': sub['limit']
                    })
                    
                    if sub['limit']:
                        joints_xml += f'    <limit lower="{sub["limit"].lower}" upper="{sub["limit"].upper}" effort="10" velocity="1.0"/>\n'
                    else:
                        joints_xml += f'    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>\n'
                    joints_xml += f'    <dynamics damping="5.0" friction="1.0" armature="0.1"/>\n'
                else:
                    joints_xml += f'    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>\n'
                
                joints_xml += '  </joint>\n\n'
                
                # Advance parent
                current_parent_link = current_child_link
 
    return f'<robot name="{robot_name}">\n{links_xml}{joints_xml}</robot>', base_link_name, generated_joints_info

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
        urdf_content, base_link_name = generate_urdf_xml(robot, sanitized_robot_name, mesh_files_map, unique_link_names)
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


# --- Mesh Processing Helper ---

def ensure_binary_stl(input_path: str, output_path: str):
    """
    Checks if an STL file is valid and converts to Binary using trimesh.
    If face count > 150,000, it decimates the mesh to be safe for MuJoCo (limit ~200k).
    """
    import shutil
    import os
    
    try:
        import trimesh
    except ImportError:
        print("trimesh not installed. Falling back to simple copy.")
        if input_path != output_path:
            shutil.copy2(input_path, output_path)
        return

    try:
        print(f"Processing STL with trimesh: {input_path}")
        # Load mesh (trimesh handles ASCII/Binary automatically)
        mesh = trimesh.load(input_path, file_type='stl')
        
        # If it loaded as a Scene (multiple meshes), flatten it
        if isinstance(mesh, trimesh.Scene):
             if len(mesh.geometry) == 0:
                 print("  Warning: Empty scene.")
                 return # Nothing to save
             # Concatenate all geometries
             mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])

        num_faces = len(mesh.faces)
        print(f"  Faces: {num_faces}")

        LIMIT = 150000 # Safety margin below 200,000
        
        if num_faces > LIMIT:
            print(f"  Face count {num_faces} exceeds safety limit {LIMIT}. Decimating...")
            # Decimate
            # simplify_quadratic_decimation is robust in trimesh (requires scipy or open3d usually)
            # fallback to simple vertex clustering if needed, but allow trimesh to try best method.
            try:
                # Use quadric decimation (standard in trimesh)
                # Must use keyword argument 'face_count' or it defaults to percent and fails
                mesh = mesh.simplify_quadric_decimation(face_count=LIMIT)
                print(f"  Decimated to: {len(mesh.faces)} faces")
            except Exception as e:
                print(f"  Decimation failed ({e}). Trying to export original.")
        
        if len(mesh.faces) == 0:
             print("  Warning: Mesh has 0 faces after load/decimate.")
        
        # Save as Binary STL
        mesh.export(output_path, file_type='stl')
        
    except Exception as e:
        print(f"Error processing STL with trimesh: {e}. Falling back to raw copy.")
        if input_path != output_path:
            shutil.copy2(input_path, output_path)

# --- MuJoCo Export Helpers ---

def generate_mjcf_xml(robot: RobotData, robot_name: str, mesh_files_map: Dict[str, str]):
    """
    Generates a native MuJoCo XML string (MJCF) for the robot.
    Recursive function to build the body tree from base link.
    """
    xml = [f'<mujoco model="{robot_name}">']
    xml.append('  <compiler angle="radian" meshdir="meshes"/>')
    xml.append('  <option gravity="0 0 -9.81"/>')
    xml.append('  <asset>')
    # Add all meshes to asset section
    for link_id, filename in mesh_files_map.items():
        # Mesh name in MJCF usually needs to be unique. We use the filename (without ext) as the mesh name.
        mesh_name = os.path.splitext(filename)[0]
        xml.append(f'    <mesh name="{mesh_name}" file="{filename}"/>')
    
    # Add simple materials
    xml.append('    <material name="gray" rgba="0.5 0.5 0.5 1"/>')
    xml.append('    <material name="collision" rgba="1 0 0 0.5"/>')
    xml.append('  </asset>')
    
    xml.append('  <worldbody>')
    xml.append('    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>')
    xml.append('    <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>')

    # Recursive function to build body hierarchy
    def build_body(link_id: str, parent_joint_id: Optional[str] = None, indent_level: int = 2):
        indent = '  ' * indent_level
        link = robot.links.get(link_id)
        if not link:
            return

        body_name = link.name
        
        # Determine body position/orientation relative to parent
        pos_str = "0 0 0"
        euler_str = "0 0 0"

        if parent_joint_id:
            joint = robot.joints.get(parent_joint_id)
            if joint:
                pos_str = " ".join(map(str, joint.origin.xyz))
                # For basic MJCF, we can try using Euler angles. 
                # Ideally we'd use 'quat' if we had full quats, but rpy/euler is supported.
                euler_str = " ".join(map(str, joint.origin.rpy))
        else:
            # Base link placement
            pos_str = "0 0 0"
            euler_str = "0 0 0"

        xml.append(f'{indent}<body name="{body_name}" pos="{pos_str}" euler="{euler_str}">')

        # 1. Add Joint (defined inside the child body in MJCF)
        if parent_joint_id:
            joint = robot.joints.get(parent_joint_id)
            if joint and joint.type != 'fixed':
                # Map joint types
                j_type = "hinge" if joint.type == 'rotational' else "slide"
                if joint.type == 'connected': j_type = "hinge" # fallback
                
                # Axis mapping
                axis = "0 0 1" # default Z
                if joint.axis == 'x': axis = "1 0 0"
                elif joint.axis == 'y': axis = "0 1 0"
                elif joint.axis == 'z': axis = "0 0 1"
                
                # Limits
                range_str = ""
                if j_type == "hinge":
                    # For simplicty, use primary roll/pitch/yaw based on axis
                    limit = joint.limits.roll 
                    if joint.axis == 'x': limit = joint.limits.roll
                    elif joint.axis == 'y': limit = joint.limits.pitch
                    elif joint.axis == 'z': limit = joint.limits.yaw
                    range_str = f'range="{limit.lower} {limit.upper}"'

                elif j_type == "slide":
                     limit = joint.limits.displacement
                     range_str = f'range="{limit.lower} {limit.upper}"'

                xml.append(f'{indent}  <joint name="{joint.name}" type="{j_type}" axis="{axis}" {range_str} />')

        # 2. Add Visual Geom
        if link.visual and link.visual.type != 'none':
            v = link.visual
            # Geom Position relative to body
            v_pos = "0 0 0"
            v_euler = "0 0 0"
            if v.type == 'mesh':
                if v.meshOrigin:
                    v_pos = " ".join(map(str, v.meshOrigin.xyz))
                    v_euler = " ".join(map(str, v.meshOrigin.rpy))
            
            if v.type == 'mesh' and link_id in mesh_files_map:
                 mesh_name = os.path.splitext(mesh_files_map[link_id])[0]
                 scale = "1 1 1"
                 if v.meshScale:
                     scale = " ".join(map(str, v.meshScale))
                 
                 # Split Visual and Collision for Mesh
                 # Visual: group 1, no collision
                 vis_geom = f'type="mesh" mesh="{mesh_name}" scale="{scale}" group="1" contype="0" conaffinity="0"'
                 
                 # Helper to get color attr
                 color_attr = f'rgba="{v.color_rgba}"' if getattr(v, 'color', None) else 'rgba="0.5 0.5 0.5 1"'
                 # Need to safely get color:
                 rgb_str = "0.5 0.5 0.5 1"
                 if v.color:
                     try:
                         r, g, b = int(v.color[1:3], 16)/255, int(v.color[3:5], 16)/255, int(v.color[5:7], 16)/255
                         rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                     except: pass
                 color_attr = f'rgba="{rgb_str}"'

                 xml.append(f'{indent}  <geom {vis_geom} pos="{v_pos}" euler="{v_euler}" {color_attr} />')
                 
                 # Collision: group 0, simple box
                 coll_geom = 'type="box" size="0.1 0.1 0.1" group="0" rgba="1 0 0 0"'
                 xml.append(f'{indent}  <geom {coll_geom} pos="{v_pos}" euler="{v_euler}" />')

            elif v.type != 'none' and v.type != 'mesh':
                # Standard Primitives
                geom_str = ""
                if v.type == 'box':
                    # MJCF box size is half-extents
                    size = " ".join([str(d/2) for d in v.dimensions])
                    geom_str = f'type="box" size="{size}"'
                elif v.type == 'cylinder':
                     # MJCF cylinder size is radius half-height
                     radius = v.dimensions[0]
                     height = v.dimensions[1] / 2
                     geom_str = f'type="cylinder" size="{radius} {height}"'
                elif v.type == 'sphere':
                     radius = v.dimensions[0]
                     geom_str = f'type="sphere" size="{radius}"'
                
                if geom_str:
                     rgb_str = "0.5 0.5 0.5 1"
                     if v.color:
                         try:
                             r, g, b = int(v.color[1:3], 16)/255, int(v.color[3:5], 16)/255, int(v.color[5:7], 16)/255
                             rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                         except: pass
                     
                     xml.append(f'{indent}  <geom {geom_str} pos="{v_pos}" euler="{v_euler}" rgba="{rgb_str}" />')
            

                


        # 3. Recurse for children
        for child_joint_id in link.childJoints:
            child_joint = robot.joints.get(child_joint_id)
            if child_joint and child_joint.childLinkId:
                build_body(child_joint.childLinkId, child_joint_id, indent_level + 1)
        
        xml.append(f'{indent}</body>')

    # Start recursion
    build_body(robot.baseLinkId, indent_level=2)

    xml.append('  </worldbody>')
    xml.append('</mujoco>')
    return "\n".join(xml)


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
