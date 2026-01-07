import os
from typing import Dict, Optional
from robot_models import RobotData
from utils import to_snake_case

def generate_mjcf_xml(robot: RobotData, robot_name: str, mesh_files_map: Dict[str, str]):
    """
    Generates a native MuJoCo XML string (MJCF) for the robot.
    Recursive function to build the body tree from base link.
    """
    xml = [f'<mujoco model="{robot_name}">']
    xml.append('  <compiler angle="radian" meshdir="meshes"/>')
    xml.append('  <option gravity="0 0 -9.81"/>')
    xml.append('  <asset>')
    for link_id, filename in mesh_files_map.items():
        mesh_name = os.path.splitext(filename)[0]
        xml.append(f'    <mesh name="{mesh_name}" file="{filename}"/>')
    
    xml.append('    <material name="gray" rgba="0.5 0.5 0.5 1"/>')
    xml.append('    <material name="collision" rgba="1 0 0 0.5"/>')
    xml.append('  </asset>')
    
    xml.append('  <worldbody>')
    xml.append('    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>')
    xml.append('    <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>')

    def build_body(link_id: str, parent_joint_id: Optional[str] = None, indent_level: int = 2):
        indent = '  ' * indent_level
        link = robot.links.get(link_id)
        if not link:
            return

        body_name = link.name
        
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
                
                axis = "0 0 1"
                if joint.axis == 'x': axis = "1 0 0"
                elif joint.axis == 'y': axis = "0 1 0"
                elif joint.axis == 'z': axis = "0 0 1"
                
                range_str = ""
                if j_type == "hinge":
                    limit = joint.limits.roll 
                    if joint.axis == 'x': limit = joint.limits.roll
                    elif joint.axis == 'y': limit = joint.limits.pitch
                    elif joint.axis == 'z': limit = joint.limits.yaw
                    range_str = f'range="{limit.lower} {limit.upper}"'

                elif j_type == "slide":
                     limit = joint.limits.displacement
                     range_str = f'range="{limit.lower} {limit.upper}"'

                xml.append(f'{indent}  <joint name="{joint.name}" type="{j_type}" axis="{axis}" {range_str} />')

        if link.visual and link.visual.type != 'none':
            v = link.visual
            v_pos = "0 0 0"
            v_euler = "0 0 0"
            
            # --- FIX: Safe Dictionary Access for meshOrigin ---
            if v.type == 'mesh' and v.meshOrigin:
                # v.meshOrigin is a Dict, so use keys 'xyz' and 'rpy'
                xyz = v.meshOrigin.get('xyz', [0.0, 0.0, 0.0])
                rpy = v.meshOrigin.get('rpy', [0.0, 0.0, 0.0])
                v_pos = " ".join(map(str, xyz))
                v_euler = " ".join(map(str, rpy))
            
            if v.type == 'mesh' and link_id in mesh_files_map:
                 mesh_name = os.path.splitext(mesh_files_map[link_id])[0]
                 scale = "1 1 1"
                 if v.meshScale:
                     scale = " ".join(map(str, v.meshScale))
                 
                 vis_geom = f'type="mesh" mesh="{mesh_name}" scale="{scale}" group="1" contype="0" conaffinity="0"'
                 
                 rgb_str = "0.5 0.5 0.5 1"
                 if v.color:
                     try:
                         r, g, b = int(v.color[1:3], 16)/255, int(v.color[3:5], 16)/255, int(v.color[5:7], 16)/255
                         rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                     except: pass
                 color_attr = f'rgba="{rgb_str}"'

                 xml.append(f'{indent}  <geom {vis_geom} pos="{v_pos}" euler="{v_euler}" {color_attr} />')
                 
                 coll_geom = 'type="box" size="0.1 0.1 0.1" group="0" rgba="1 0 0 0"'
                 xml.append(f'{indent}  <geom {coll_geom} pos="{v_pos}" euler="{v_euler}" />')

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

        for child_joint_id in link.childJoints:
            child_joint = robot.joints.get(child_joint_id)
            if child_joint and child_joint.childLinkId:
                build_body(child_joint.childLinkId, child_joint_id, indent_level + 1)
        
        xml.append(f'{indent}</body>')

    build_body(robot.baseLinkId, indent_level=2)

    xml.append('  </worldbody>')
    xml.append('</mujoco>')
    return "\n".join(xml)
