import re
import math
from typing import List, Tuple, Dict
from robot_models import RobotData

def to_snake_case(name: str) -> str:
    """Converts a string to snake_case and ensures it's a valid identifier."""
    name = str(name).strip()
    name = name.replace(' ', '_')
    s = re.sub('(.)([A-Z][a-z]+)', r'\\1_\\2', name)
    s = re.sub('([a-z0-9])([A-Z])', r'\\1_\\2', s).lower()
    s = re.sub(r'\\W+', '_', s)
    s = re.sub(r'_+', '_', s)
    s = re.sub(r'^_|_$', '', s)
    return s if s else "link"

def calculate_cylinder_transform(start_point: List[float], end_point: List[float]) -> Tuple[float, List[float], List[float]]:
    """Calculates cylinder transform between two points."""
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    dz = end_point[2] - start_point[2]
    
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    mx = (start_point[0] + end_point[0]) / 2.0
    my = (start_point[1] + end_point[1]) / 2.0
    mz = (start_point[2] + end_point[2]) / 2.0
    
    yaw = math.atan2(dy, dx)
    horizontal_dist = math.sqrt(dx*dx + dy*dy)
    pitch = math.atan2(horizontal_dist, dz)
    
    return length, [mx, my, mz], [0.0, pitch, yaw]

def euler_xyz_to_rotation_matrix(x, y, z):
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    
    R = [
        [cy*cz,              -cy*sz,              sy],
        [cx*sz + sx*sy*cz,   cx*cz - sx*sy*sz,   -sx*cy],
        [sx*sz - cx*sy*cz,   sx*cz + cx*sy*sz,   cx*cy]
    ]
    return R

def rotation_matrix_to_euler_zyx(R):
    r20 = R[2][0]
    r21 = R[2][1]
    r22 = R[2][2]
    r10 = R[1][0]
    r00 = R[0][0]
    
    if r20 < 1.0:
        if r20 > -1.0:
            p = math.asin(-r20)
            r = math.atan2(r21, r22)
            y = math.atan2(r10, r00)
        else:
            p = math.pi / 2
            y = 0
            r = math.atan2(R[0][1], R[0][2])
    else:
        p = -math.pi / 2
        y = 0
        r = math.atan2(-R[1][2], R[1][1])

    return [r, p, y]

def convert_euler_xyz_to_zyx(rpy):
    if not rpy or len(rpy) != 3:
        return [0.0, 0.0, 0.0]
    R = euler_xyz_to_rotation_matrix(rpy[0], rpy[1], rpy[2])
    return rotation_matrix_to_euler_zyx(R)

def generate_unique_names(robot_data: RobotData) -> Dict[str, str]:
    """Generates a mapping of link_id -> unique_clean_name."""
    unique_link_names = {}
    used_names = set()

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
