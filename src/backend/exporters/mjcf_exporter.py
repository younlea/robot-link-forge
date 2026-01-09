import os
from typing import Dict, Optional
from robot_models import RobotData
from utils import to_snake_case

def generate_mjcf_xml(robot: RobotData, robot_name: str, mesh_files_map: Dict[str, str], unique_link_names: Dict[str, str]):
    """
    Generates a native MuJoCo XML string (MJCF) for the robot.
    Recursive function to build the body tree from base link.
    """
    
    # 1. Collect all used meshes and their scales to define unique assets
    # Map: (filename, scale_str) -> asset_name
    mesh_assets = {} # Dict[Tuple[str, str], str]
    
    def get_mesh_asset_name(link_or_joint_id, filename, scale_list):
        scale_str = "1 1 1"
        if scale_list:
            scale_str = " ".join(map(str, scale_list))
        
        key = (filename, scale_str)
        if key not in mesh_assets:
            # Create a unique name. Using the ID helps readability but uniqueness is key.
            # We can just append a counter or use the ID if unique.
            # To stay simple: mesh_{id} might conflict if same ID used multiple times (unlikely for links)
            # but joints might map to same file.
            # Let's use the nice name from filename + index
            base_name = os.path.splitext(filename)[0]
            idx = len(mesh_assets)
            asset_name = f"{base_name}_{idx}" 
            mesh_assets[key] = asset_name
            return asset_name
        return mesh_assets[key]

    # Pre-scan could be done, or we can build assets on the fly?
    # Actually, XML requires assets to be defined BEFORE usage in worldbody (usually).
    # So we should pre-scan.
    
    # Scan Links
    for link_id, link in robot.links.items():
        if link.visual and link.visual.type == 'mesh' and link_id in mesh_files_map:
             get_mesh_asset_name(link_id, mesh_files_map[link_id], link.visual.meshScale)

    # Scan Joints
    for joint_id, joint in robot.joints.items():
        if joint.visual and joint.visual.type == 'mesh' and joint_id in mesh_files_map:
             get_mesh_asset_name(joint_id, mesh_files_map[joint_id], joint.visual.meshScale)

    xml = [f'<mujoco model="{robot_name}">']
    xml.append('  <compiler angle="radian" meshdir="meshes"/>')
    xml.append('  <option gravity="0 0 -9.81"/>')
    
    # 2. Write Assets
    xml.append('  <asset>')
    for (filename, scale_str), asset_name in mesh_assets.items():
        xml.append(f'    <mesh name="{asset_name}" file="{filename}" scale="{scale_str}"/>')
    
    xml.append('    <material name="gray" rgba="0.5 0.5 0.5 1"/>')
    xml.append('    <material name="collision" rgba="1 0 0 0.5"/>')
    xml.append('  </asset>')
    
    xml.append('  <worldbody>')
    xml.append('    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>')
    xml.append('    <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>')

    actuators = []
    sensors = []

    def build_body(link_id: str, parent_joint_id: Optional[str] = None, indent_level: int = 2):
        indent = '  ' * indent_level
        link = robot.links.get(link_id)
        if not link:
            return

        body_name = unique_link_names.get(link_id, link.name)
        # Fallback to to_snake_case if not in map (should not happen if map is comprehensive)
        if link_id not in unique_link_names:
             body_name = to_snake_case(body_name)
        
        pos_str = "0 0 0"
        euler_str = "0 0 0"

        if parent_joint_id:
            joint = robot.joints.get(parent_joint_id)
            if joint:
                pos_str = " ".join(map(str, joint.origin.xyz))
                euler_str = " ".join(map(str, joint.origin.rpy))
        else:
            pos_str = "0 0 0"
            euler_str = "0 0 0"

        xml.append(f'{indent}<body name="{body_name}" pos="{pos_str}" euler="{euler_str}">')

        if parent_joint_id:
            joint = robot.joints.get(parent_joint_id)
            if joint and joint.type != 'fixed':
                j_type = "hinge" if joint.type == 'rotational' else "slide"
                if joint.type == 'connected': j_type = "hinge"
                
                if joint.type in ['rotational', 'connected']:
                    active_axes = []
                    if joint.dof.roll: active_axes.append(('roll', '1 0 0'))
                    if joint.dof.pitch: active_axes.append(('pitch', '0 1 0'))
                    if joint.dof.yaw: active_axes.append(('yaw', '0 0 1'))

                    if not active_axes and joint.axis:
                        # Fallback for weird cases: try to guess or use default
                        active_axes.append(('custom', f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}"))

                    for dof_name, axis_str in active_axes:
                         suffix = f"_{dof_name}" if len(active_axes) > 1 else ""
                         joint_xml_name = f"{joint.name}{suffix}"
                         
                         limit = joint.limits.get(dof_name)
                         range_str = ""
                         if limit:
                             range_str = f'range="{limit.lower} {limit.upper}"'
                         
                         xml.append(f'{indent}  <joint name="{joint_xml_name}" type="hinge" axis="{axis_str}" {range_str} />')
                         
                         ctrl_range = range_str.replace("range=", "ctrlrange=") if range_str else 'ctrlrange="-3.14 3.14"'
                         actuators.append(f'{indent}    <position name="{joint_xml_name}_act" joint="{joint_xml_name}" kp="800" kv="50" {ctrl_range}/>')

                elif joint.type == 'prismatic':
                    axis_val = joint.axis if joint.axis else [1, 0, 0]
                    axis_str = f"{axis_val[0]} {axis_val[1]} {axis_val[2]}"
                    curr_limit = joint.limits.get('displacement')
                    range_str = ""
                    if curr_limit:
                        range_str = f'range="{curr_limit.lower} {curr_limit.upper}"'
                    
                    xml.append(f'{indent}  <joint name="{joint.name}" type="slide" axis="{axis_str}" {range_str} />')
                    
                    ctrl_range = range_str.replace("range=", "ctrlrange=") if range_str else 'ctrlrange="-1 1"'
                    actuators.append(f'{indent}    <position name="{joint.name}_act" joint="{joint.name}" kp="1000" kv="50" {ctrl_range}/>')

        # --- Joint Visuals ---
        if parent_joint_id:
            p_joint = robot.joints.get(parent_joint_id)
            if p_joint and p_joint.visual and p_joint.visual.type != 'none':
                jv = p_joint.visual
                jv_pos = "0 0 0"
                jv_euler = "0 0 0"
                if jv.meshOrigin:
                    xyz = jv.meshOrigin.get('xyz', [0.0, 0.0, 0.0])
                    rpy = jv.meshOrigin.get('rpy', [0.0, 0.0, 0.0])
                    jv_pos = " ".join(map(str, xyz))
                    jv_euler = " ".join(map(str, rpy))
                
                j_geom_str = ""
                if jv.type == 'mesh' and parent_joint_id in mesh_files_map:
                     # Lookup asset name
                     j_asset_name = get_mesh_asset_name(parent_joint_id, mesh_files_map[parent_joint_id], jv.meshScale)
                     # NO scale attribute here, it's in the asset
                     j_geom_str = f'type="mesh" mesh="{j_asset_name}"'
                elif jv.type == 'box':
                     size = " ".join([str(d/2) for d in jv.dimensions])
                     j_geom_str = f'type="box" size="{size}"'
                elif jv.type == 'cylinder':
                     radius = jv.dimensions[0]
                     height = jv.dimensions[1] / 2
                     j_geom_str = f'type="cylinder" size="{radius} {height}"'
                
                if j_geom_str:
                     rgb_str = "0.5 0.5 0.5 1"
                     if jv.color:
                         try:
                             r, g, b = int(jv.color[1:3], 16)/255, int(jv.color[3:5], 16)/255, int(jv.color[5:7], 16)/255
                             rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                         except: pass
                     xml.append(f'{indent}  <geom {j_geom_str} pos="{jv_pos}" euler="{jv_euler}" rgba="{rgb_str}" group="1" />')
                     xml.append(f'{indent}  <geom {j_geom_str} pos="{jv_pos}" euler="{jv_euler}" group="0" rgba="1 0 0 0" />')

        # --- Link Visuals ---
        if link.visual and link.visual.type != 'none':
            v = link.visual
            v_pos = "0 0 0"
            v_euler = "0 0 0"
            
            if v.type == 'mesh' and v.meshOrigin:
                xyz = v.meshOrigin.get('xyz', [0.0, 0.0, 0.0])
                rpy = v.meshOrigin.get('rpy', [0.0, 0.0, 0.0])
                v_pos = " ".join(map(str, xyz))
                v_euler = " ".join(map(str, rpy))
            
            if v.type == 'mesh' and link_id in mesh_files_map:
                 # Lookup asset name
                 asset_name = get_mesh_asset_name(link_id, mesh_files_map[link_id], v.meshScale)
                 
                 # NO scale attribute here
                 vis_geom = f'type="mesh" mesh="{asset_name}" group="1" contype="0" conaffinity="0"'
                 
                 rgb_str = "0.5 0.5 0.5 1"
                 if v.color:
                     try:
                         r, g, b = int(v.color[1:3], 16)/255, int(v.color[3:5], 16)/255, int(v.color[5:7], 16)/255
                         rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                     except: pass
                 color_attr = f'rgba="{rgb_str}"'

                 xml.append(f'{indent}  <geom {vis_geom} pos="{v_pos}" euler="{v_euler}" {color_attr} />')
                 
                 # Simplified Collision: Use Primitive Cylinder for ALL Mesh parts
                 # This prevents jagged STL interference while maintaining approximate shape.
                 # Heuristic: Radius 12mm, Length 40mm, Oriented along X (Bone axis)
                 coll_geom = 'type="cylinder" size="0.012 0.02" pos="0.02 0 0" euler="0 1.5708 0" group="0" rgba="0 0 1 0.3"'
                 xml.append(f'{indent}  <geom {coll_geom} />')

            elif v.type != 'none' and v.type != 'mesh':
                geom_str = ""
                if v.type == 'box':
                    size = " ".join([str(d/2) for d in v.dimensions])
                    geom_str = f'type="box" size="{size}"'
                elif v.type == 'cylinder':
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

        # Determine if we should add a sensor
        # It is a leaf if it has no child joints, OR if its child joints don't lead to valid bodies
        is_leaf = True
        if link.childJoints:
            for child_joint_id in link.childJoints:
                cj = robot.joints.get(child_joint_id)
                if cj and cj.childLinkId:
                    # Found a valid child link, so this is NOT a leaf
                    is_leaf = False
                    break
        
        # Heuristic: User might name things "finger_tip" etc.
        name_lower = body_name.lower()
        is_target_name = "tip" in name_lower or "finger" in name_lower or "hand" in name_lower
        
        # Add sensor if leaf or specifically named, AND it has some visual (otherwise it's a dummy frame)
        # Actually, dummy frames are fine for sites.
        if is_leaf or is_target_name:
            # Add site for sensor
            site_name = f"site_{body_name}"
            # Make site visible as a "pad" sensor on the surface
            # Heuristic: Thin box, offset significantly (4.5cm) to try and reach the fingertip surface
            xml.append(f'{indent}  <site name="{site_name}" type="box" pos="0.045 0 0" size="0.008 0.015 0.002" rgba="0 1 0 0.5" />')
            
            # Add sensor definition
            sensor_name = f"sensor_{body_name}"
            sensors.append(f'    <touch name="{sensor_name}" site="{site_name}" />')

        for child_joint_id in link.childJoints:
            child_joint = robot.joints.get(child_joint_id)
            if child_joint and child_joint.childLinkId:
                build_body(child_joint.childLinkId, child_joint_id, indent_level + 1)
        
        xml.append(f'{indent}</body>')

    build_body(robot.baseLinkId, indent_level=2)

    xml.append('  </worldbody>')
    
    if actuators:
        xml.append('  <actuator>')
        for act in actuators:
            xml.append(act)
        xml.append('  </actuator>')

    if sensors:
        xml.append('  <sensor>')
        for sens in sensors:
            xml.append(sens)
        xml.append('  </sensor>')
        
    xml.append('</mujoco>')
    return "\n".join(xml)
