from typing import Dict, Tuple, Set, Optional
from robot_models import RobotData, RobotLink, RobotJoint
from utils import to_snake_case, calculate_cylinder_transform, convert_euler_xyz_to_zyx

def _generate_link_xml(link_id: str, link_name: str, robot_data: RobotData, robot_name: str, mesh_files: Dict[str, str], parent_joint: Optional[RobotJoint] = None, for_mujoco: bool = False) -> str:
    """Generates the XML content for a single link (visual, collision, etc.)."""
    
    link: RobotLink = robot_data.links[link_id]
    
    # --- Inertial XML (Required for MuJoCo if collision is missing or general stability) ---
    inertial_xml = '    <inertial>\n'
    inertial_xml += '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
    inertial_xml += '      <mass value="1.0"/>\n'
    inertial_xml += '      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>\n'
    inertial_xml += '    </inertial>\n'

    # --- Visual XML ---
    visual_xml = '    <visual>\n'
    
    vis = link.visual
    
    geometry_xml = ""
    override_origin_xyz = None
    override_origin_rpy = None

    # Check for Dynamic Cylinder Case (Connecting Link)
    if vis.type == 'cylinder' and link.childJoints and len(link.childJoints) == 1:
        child_joint_id = link.childJoints[0]
        child_joint = robot_data.joints.get(child_joint_id)
        
        if child_joint and child_joint.origin and not vis.meshOrigin:
            length, mid_xyz, mid_rpy = calculate_cylinder_transform(
                [0,0,0], 
                child_joint.origin.xyz
            )
            
            if length > 0.001:
                radius = vis.dimensions[0] if (vis.dimensions and len(vis.dimensions) > 0) else 0.05
                geometry_xml = f'        <cylinder radius="{radius}" length="{length}" />\n'
                override_origin_xyz = " ".join(map(str, mid_xyz))
                override_origin_rpy = " ".join(map(str, mid_rpy))

    if not geometry_xml:
        if vis.type == 'mesh' and link_id in mesh_files:
            mesh_scale = " ".join(map(str, vis.meshScale)) if vis.meshScale else "1 1 1"
            geometry_xml = f'        <mesh filename="package://{robot_name}/meshes/{mesh_files[link_id]}" scale="{mesh_scale}"/>\n'
        elif vis.type == 'box' and vis.dimensions:
            geometry_xml = f'        <box size="{" ".join(map(str, vis.dimensions))}" />\n'
        elif vis.type == 'cylinder' and vis.dimensions:
            length_val = vis.dimensions[1] if len(vis.dimensions) > 1 else 1.0
            geometry_xml = f'        <cylinder radius="{vis.dimensions[0]}" length="{length_val}" />\n'
        elif vis.type == 'sphere' and vis.dimensions:
            geometry_xml = f'        <sphere radius="{vis.dimensions[0]}" />\n'
        else:
            geometry_xml = '        <box size="0.01 0.01 0.01" />\n'

    # Determine Final Origin
    origin_xyz_str = "0 0 0"
    origin_rpy_str = "0 0 0"

    if override_origin_xyz:
        origin_xyz_str = override_origin_xyz
        origin_rpy_str = override_origin_rpy
    elif vis.meshOrigin:
        origin_xyz_str = " ".join(map(str, vis.meshOrigin.get('xyz', [0,0,0])))
        raw_rpy = vis.meshOrigin.get('rpy', [0,0,0])
        converted_rpy = convert_euler_xyz_to_zyx(raw_rpy)
        origin_rpy_str = " ".join(map(str, converted_rpy))

    visual_xml += f'      <origin xyz="{origin_xyz_str}" rpy="{origin_rpy_str}" />\n'
    visual_xml += '      <geometry>\n'
    visual_xml += geometry_xml
    visual_xml += '      </geometry>\n'
    
    if vis.color:
        try:
            r = int(vis.color[1:3], 16) / 255.0
            g = int(vis.color[3:5], 16) / 255.0
            b = int(vis.color[5:7], 16) / 255.0
            visual_xml += f'      <material name="material_{link_id}">\n'
            visual_xml += f'        <color rgba="{r:.2f} {g:.2f} {b:.2f} 1.0"/>\n'
            visual_xml += f'      </material>\n'
        except:
             visual_xml += f'      <material name="material_{link_id}">\n'
             visual_xml += f'        <color rgba="0.5 0.5 0.5 1.0"/>\n'
             visual_xml += f'      </material>\n'
             
    visual_xml += '    </visual>\n'
    
    # COLLISION LOGIC
    # Standard: Duplicate Visual as Collision
    collision_xml = visual_xml.replace('<visual>', '<collision>', 1).replace('</visual>', '</collision>', 1)

    # --- Joint Visuals (Attached to CHILD Link, i.e., THIS link) ---
    # If this link is the child of a joint, and that joint has a visual, we render it here.
    if parent_joint and parent_joint.visual and parent_joint.visual.type != 'none':
        j_vis = parent_joint.visual
        
        # Origin relative to Child Link (Joint Frame) is 0 0 0 unless visual has offset
        j_xyz = "0 0 0"
        j_rpy = "0 0 0"
        
        if j_vis.meshOrigin:
            j_xyz = " ".join(map(str, j_vis.meshOrigin.get('xyz', [0,0,0])))
            j_r_raw = j_vis.meshOrigin.get('rpy', [0,0,0])
            j_r_zyx = convert_euler_xyz_to_zyx(j_r_raw)
            j_rpy = " ".join(map(str, j_r_zyx))

        j_geom = ""
        if j_vis.type == 'mesh' and parent_joint.id in mesh_files:
             j_scale = " ".join(map(str, j_vis.meshScale)) if j_vis.meshScale else "1 1 1"
             j_geom = f'<mesh filename="package://{robot_name}/meshes/{mesh_files[parent_joint.id]}" scale="{j_scale}"/>'
        elif j_vis.type == 'box' and j_vis.dimensions:
             j_geom = f'<box size="{" ".join(map(str, j_vis.dimensions))}" />'
        elif j_vis.type == 'cylinder' and j_vis.dimensions:
             l_val = j_vis.dimensions[1] if len(j_vis.dimensions) > 1 else 1.0
             j_geom = f'<cylinder radius="{j_vis.dimensions[0]}" length="{l_val}" />'
        elif j_vis.type == 'sphere' and j_vis.dimensions:
             j_geom = f'<sphere radius="{j_vis.dimensions[0]}" />'

        if j_geom:
             j_visual_xml = '    <visual>\n'
             j_visual_xml += f'      <origin xyz="{j_xyz}" rpy="{j_rpy}" />\n'
             j_visual_xml += f'      <geometry>\n        {j_geom}\n      </geometry>\n'
             if j_vis.color:
                try:
                    r, g, b = int(j_vis.color[1:3], 16)/255, int(j_vis.color[3:5], 16)/255, int(j_vis.color[5:7], 16)/255
                    j_visual_xml += f'      <material name="mat_{parent_joint.id}">\n'
                    j_visual_xml += f'        <color rgba="{r:.2f} {g:.2f} {b:.2f} 1.0"/>\n'
                    j_visual_xml += f'      </material>\n'
                except: pass
             j_visual_xml += '    </visual>\n'
             
             # Append to link xml
             link_xml = f'  <link name="{link_name}">\n'
             link_xml += inertial_xml
             link_xml += visual_xml
             link_xml += collision_xml
             
             # Add Joint Visual
             link_xml += j_visual_xml
             j_collision_xml = j_visual_xml.replace('<visual>', '<collision>').replace('</visual>', '</collision>')
             link_xml += j_collision_xml
             
             link_xml += '  </link>\n\n'
             return link_xml

    # --- Link XML (Default return if no joint visual or processed above) ---

    # --- Link XML ---
    link_xml = f'  <link name="{link_name}">\n'
    link_xml += inertial_xml
    link_xml += visual_xml
    link_xml += collision_xml
    link_xml += '  </link>\n\n'
    
    return link_xml

def generate_urdf_xml(robot_data: RobotData, robot_name: str, mesh_files: Dict[str, str], unique_link_names: Dict[str, str], for_mujoco: bool = False) -> Tuple[str, str, list]:
    """Generates the URDF XML content and returns (urdf_string, base_link_name, generated_joints_info)."""

    if robot_data.baseLinkId not in unique_link_names:
         raise ValueError(f"Base Link ID {robot_data.baseLinkId} not found in links map.")

    base_link_name = unique_link_names[robot_data.baseLinkId]
    
    links_xml = _generate_link_xml(robot_data.baseLinkId, base_link_name, robot_data, robot_name, mesh_files, parent_joint=None, for_mujoco=for_mujoco)
    
    joints_xml = ""

    processed_links = {robot_data.baseLinkId}
    processed_joints = set()
    generated_joint_names = set()
    generated_joints_info = [] 
    
    q = [robot_data.baseLinkId]
    while q:
        parent_link_id = q.pop(0)
        if parent_link_id not in unique_link_names:
             continue
        parent_link_name = unique_link_names[parent_link_id]
        child_joints = robot_data.links[parent_link_id].childJoints or []
        
        for joint_id in child_joints:
            if joint_id in processed_joints:
                continue
            
            joint = robot_data.joints.get(joint_id)
            if not joint or not joint.childLinkId:
                continue
            
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

            links_xml += _generate_link_xml(child_link_id, child_link_name, robot_data, robot_name, mesh_files, parent_joint=joint, for_mujoco=for_mujoco)

            active_rotations = []
            if joint.type == 'rotational':
                if joint.dof.roll: active_rotations.append(('roll', '1 0 0', joint.limits['roll']))
                if joint.dof.pitch: active_rotations.append(('pitch', '0 1 0', joint.limits['pitch']))
                if joint.dof.yaw: active_rotations.append(('yaw', '0 0 1', joint.limits['yaw']))
            
            sub_joints = []
            if joint.type == 'rotational' and active_rotations:
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
                 sub_joints.append({
                     'type': 'fixed',
                     'suffix': 'fixed',
                     'axis': '0 0 0',
                     'limit': None
                 })

            current_parent_link = parent_link_name
            base_joint_unique_id = f"{to_snake_case(joint.name)}_{joint.id[:6]}"
            
            for i, sub in enumerate(sub_joints):
                is_last = (i == len(sub_joints) - 1)
                is_first = (i == 0)
                
                raw_name = f"{base_joint_unique_id}_{sub['suffix']}"
                final_joint_name = raw_name
                ctr = 1
                while final_joint_name in generated_joint_names:
                    final_joint_name = f"{raw_name}_{ctr}"
                    ctr += 1
                generated_joint_names.add(final_joint_name)
                
                if is_last:
                    current_child_link = child_link_name
                else:
                    dummy_link_name = f"dummy_{final_joint_name}_link"
                    links_xml += f'  <link name="{dummy_link_name}">\n'
                    links_xml += f'    <inertial><mass value="0.001"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>\n'
                    links_xml += f'  </link>\n'
                    current_child_link = dummy_link_name

                joints_xml += f'  <joint name="{final_joint_name}" type="{sub["type"]}">\n'
                joints_xml += f'    <parent link="{current_parent_link}"/>\n'
                joints_xml += f'    <child link="{current_child_link}"/>\n'
                
                if is_first:
                    converted_origin_rpy = convert_euler_xyz_to_zyx(joint.origin.rpy)
                    joints_xml += f'    <origin xyz="{" ".join(map(str, joint.origin.xyz))}" rpy="{" ".join(map(str, converted_origin_rpy))}"/>\n'
                else:
                    joints_xml += f'    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                
                if sub['type'] != 'fixed':
                    joints_xml += f'    <axis xyz="{sub["axis"]}"/>\n'
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
                current_parent_link = current_child_link
 
    return f'<robot name="{robot_name}">\n{links_xml}{joints_xml}</robot>', base_link_name, generated_joints_info
