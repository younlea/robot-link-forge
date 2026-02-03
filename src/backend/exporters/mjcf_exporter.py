import os
from typing import Dict, Optional, Tuple, List
from robot_models import RobotData
from utils import to_snake_case
from .stl_utils import calculate_inertia_from_stl


def generate_mjcf_xml(
    robot: RobotData,
    robot_name: str,
    mesh_files_map: Dict[str, str],
    unique_link_names: Dict[str, str],
    use_mesh_collision: bool = False,
    direct_hand: bool = False,
    mesh_dir: Optional[str] = None,
) -> Tuple[str, List[Dict]]:
    """
    Generates MuJoCo MJCF XML.
    Returns: (xml_content, generated_joints_info)

    Args:
        mesh_dir: Directory where STL files are stored (for inertia calculation)
    """
    # Reuse URDF generation logic used for MuJoCo to extract joint split info
    # We call generate_urdf_xml with for_mujoco=True to get the splits
    # Then we wrap it in MJCF.

    # Actually, generate_mjcf_xml re-implements the logic?
    # Let's check the implementation.
    pass
    """
    Generates a native MuJoCo XML string (MJCF) for the robot.
    Recursive function to build the body tree from base link.
    """

    # 1. Collect all used meshes and their scales to define unique assets
    # Map: (filename, scale_str) -> asset_name
    mesh_assets = {}  # Dict[Tuple[str, str], str]

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
        if link.visual and link.visual.type == "mesh" and link_id in mesh_files_map:
            get_mesh_asset_name(link_id, mesh_files_map[link_id], link.visual.meshScale)

    # Scan Joints
    for joint_id, joint in robot.joints.items():
        if joint.visual and joint.visual.type == "mesh" and joint_id in mesh_files_map:
            get_mesh_asset_name(
                joint_id, mesh_files_map[joint_id], joint.visual.meshScale
            )

    xml = [f'<mujoco model="{robot_name}">']
    xml.append('  <compiler angle="radian" meshdir="meshes"/>')
    # Improved Global Physics
    xml.append(
        '  <option timestep="0.002" iterations="50" solver="Newton" tolerance="1e-10" gravity="0 0 -9.81"/>'
    )

    # 2. Write Assets
    xml.append("  <asset>")
    for (filename, scale_str), asset_name in mesh_assets.items():
        xml.append(
            f'    <mesh name="{asset_name}" file="{filename}" scale="{scale_str}"/>'
        )

    xml.append('    <material name="gray" rgba="0.5 0.5 0.5 1"/>')
    xml.append('    <material name="collision" rgba="1 0 0 0.5"/>')
    xml.append("  </asset>")

    xml.append("  <worldbody>")
    xml.append('    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>')
    xml.append('    <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>')

    actuators = []
    sensors = []
    generated_joints_info = []
    actuator_counter = {}  # Track actuator names to avoid duplicates
    joint_counter = {}  # Track joint names to avoid duplicates

    def build_body(
        link_id: str, parent_joint_id: Optional[str] = None, indent_level: int = 2
    ):
        indent = "  " * indent_level
        link = robot.links.get(link_id)
        if not link:
            return

        body_name = unique_link_names.get(link_id, link.name)
        # Fallback to to_snake_case if not in map (should not happen if map is comprehensive)
        if link_id not in unique_link_names:
            body_name = to_snake_case(body_name)

        # Determine if it is a leaf (Pre-calculation for Collision & Sensors)
        is_leaf = True
        if link.childJoints:
            for child_joint_id in link.childJoints:
                cj = robot.joints.get(child_joint_id)
                if cj and cj.childLinkId:
                    is_leaf = False
                    break

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

        xml.append(
            f'{indent}<body name="{body_name}" pos="{pos_str}" euler="{euler_str}">'
        )

        # Add default inertial properties for non-fixed bodies
        # MuJoCo requires mass and inertia for moving bodies
        if parent_joint_id:
            joint = robot.joints.get(parent_joint_id)
            if joint and joint.type != "fixed":
                # Try to calculate inertia from STL
                inertia_data = None
                if mesh_dir and link_id in mesh_files_map:
                    stl_filename = mesh_files_map[link_id]
                    stl_path = os.path.join(mesh_dir, stl_filename)
                    if os.path.exists(stl_path):
                        # Get scale from link visual properties
                        scale = (
                            link.visual.meshScale
                            if (link.visual and link.visual.meshScale)
                            else None
                        )
                        inertia_data = calculate_inertia_from_stl(stl_path, scale=scale)

                if inertia_data:
                    mass = inertia_data["mass"]
                    com = inertia_data["center_of_mass"]
                    inertia_diag = inertia_data["inertia_diagonal"]
                    com_str = f"{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}"
                    inertia_str = f"{inertia_diag[0]:.6f} {inertia_diag[1]:.6f} {inertia_diag[2]:.6f}"
                    xml.append(
                        f'{indent}  <inertial pos="{com_str}" mass="{mass:.6f}" diaginertia="{inertia_str}" />'
                    )
                else:
                    # Use better default values (reasonable for robot links)
                    # mass: 0.05 kg (50g), inertia scaled accordingly
                    xml.append(
                        f'{indent}  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001" />'
                    )

        if parent_joint_id:
            joint = robot.joints.get(parent_joint_id)
            if joint and joint.type != "fixed":
                j_type = "hinge" if joint.type == "rotational" else "slide"
                if joint.type == "connected":
                    j_type = "hinge"

                if joint.type in ["rotational", "connected"]:
                    active_axes = []
                    if joint.dof.roll:
                        active_axes.append(("roll", "1 0 0"))
                    if joint.dof.pitch:
                        active_axes.append(("pitch", "0 1 0"))
                    if joint.dof.yaw:
                        active_axes.append(("yaw", "0 0 1"))

                    if not active_axes and joint.axis:
                        # Fallback for weird cases: try to guess or use default
                        active_axes.append(
                            (
                                "custom",
                                f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}",
                            )
                        )

                    for dof_name, axis_str in active_axes:
                        suffix = f"_{dof_name}" if len(active_axes) > 1 else ""
                        joint_xml_name = f"{joint.name}{suffix}"

                        # Ensure unique joint name
                        if joint_xml_name in joint_counter:
                            joint_counter[joint_xml_name] += 1
                            new_joint_name = (
                                f"{joint_xml_name}_{joint_counter[joint_xml_name]}"
                            )
                            print(
                                f"WARNING: Duplicate joint name '{joint_xml_name}' detected. Renamed to '{new_joint_name}'"
                            )
                            joint_xml_name = new_joint_name
                        else:
                            joint_counter[joint_xml_name] = 0

                        limit = joint.limits.get(dof_name)
                        range_str = ""
                        if limit:
                            range_str = f'range="{limit.lower} {limit.upper}"'

                        # Add damping and armature for stability
                        xml.append(
                            f'{indent}  <joint name="{joint_xml_name}" type="hinge" axis="{axis_str}" '
                            f'{range_str} damping="0.5" armature="0.001" />'
                        )

                        ctrl_range = (
                            range_str.replace("range=", "ctrlrange=")
                            if range_str
                            else 'ctrlrange="-3.14 3.14"'
                        )

                        # Ensure unique actuator name
                        act_name = f"{joint_xml_name}_act"
                        if act_name in actuator_counter:
                            actuator_counter[act_name] += 1
                            new_act_name = (
                                f"{joint_xml_name}_act_{actuator_counter[act_name]}"
                            )
                            print(
                                f"WARNING: Duplicate actuator name '{act_name}' detected. Renamed to '{new_act_name}'"
                            )
                            act_name = new_act_name
                        else:
                            actuator_counter[act_name] = 0

# Tuned for aggressive trajectory tracking
                        # kp=2500, kv=250 (10% damping ratio), high force limit
                        # User can adjust per-joint for different motor specs
                        actuators.append(
                            f'{indent}    <position name="{act_name}" joint="{joint_xml_name}" '
                            f'kp="2500" kv="250" gear="1" forcelimited="true" forcerange="-300 300" {ctrl_range}/>'  
                        )

                        # Capture Info for Replay Mapping
                        # Note: for rotational joints we used 'active_axes' logic.
                        # generated_joints_info needs {original_id, suffix, name}
                        generated_joints_info.append(
                            {
                                "original_id": parent_joint_id,
                                "suffix": dof_name,
                                "name": joint_xml_name,
                            }
                        )

                elif joint.type == "prismatic":
                    axis_val = joint.axis if joint.axis else [1, 0, 0]
                    axis_str = f"{axis_val[0]} {axis_val[1]} {axis_val[2]}"
                    curr_limit = joint.limits.get("displacement")
                    range_str = ""
                    if curr_limit:
                        range_str = f'range="{curr_limit.lower} {curr_limit.upper}"'

                    # Ensure unique joint name
                    joint_xml_name = joint.name
                    if joint_xml_name in joint_counter:
                        joint_counter[joint_xml_name] += 1
                        new_joint_name = (
                            f"{joint_xml_name}_{joint_counter[joint_xml_name]}"
                        )
                        print(
                            f"WARNING: Duplicate joint name '{joint_xml_name}' detected. Renamed to '{new_joint_name}'"
                        )
                        joint_xml_name = new_joint_name
                    else:
                        joint_counter[joint_xml_name] = 0

                    xml.append(
                        f'{indent}  <joint name="{joint_xml_name}" type="slide" axis="{axis_str}" '
                        f'{range_str} damping="0.5" armature="0.001" />'
                    )

                    ctrl_range = (
                        range_str.replace("range=", "ctrlrange=")
                        if range_str
                        else 'ctrlrange="-1 1"'
                    )

                    # Ensure unique actuator name
                    act_name = f"{joint_xml_name}_act"
                    if act_name in actuator_counter:
                        actuator_counter[act_name] += 1
                        new_act_name = (
                            f"{joint_xml_name}_act_{actuator_counter[act_name]}"
                        )
                        print(
                            f"WARNING: Duplicate actuator name '{act_name}' detected. Renamed to '{new_act_name}'"
                        )
                        act_name = new_act_name
                    else:
                        actuator_counter[act_name] = 0

                    # Tuned for stable tracking
                    actuators.append(
                        f'{indent}    <position name="{act_name}" joint="{joint_xml_name}" '
                        f'kp="800" kv="80" gear="1" forcelimited="true" forcerange="-150 150" {ctrl_range}/>'
                    )

                    # Capture Info for Replay Mapping
                    generated_joints_info.append(
                        {
                            "original_id": parent_joint_id,
                            "suffix": "prism",
                            "name": joint_xml_name,
                        }
                    )

        # --- Joint Visuals ---
        if parent_joint_id:
            p_joint = robot.joints.get(parent_joint_id)
            if p_joint and p_joint.visual and p_joint.visual.type != "none":
                jv = p_joint.visual
                jv_pos = "0 0 0"
                jv_euler = "0 0 0"
                if jv.meshOrigin:
                    xyz = jv.meshOrigin.get("xyz", [0.0, 0.0, 0.0])
                    rpy = jv.meshOrigin.get("rpy", [0.0, 0.0, 0.0])
                    jv_pos = " ".join(map(str, xyz))
                    jv_euler = " ".join(map(str, rpy))

                j_geom_str = ""
                if jv.type == "mesh" and parent_joint_id in mesh_files_map:
                    # Lookup asset name
                    j_asset_name = get_mesh_asset_name(
                        parent_joint_id, mesh_files_map[parent_joint_id], jv.meshScale
                    )
                    # NO scale attribute here, it's in the asset
                    j_geom_str = f'type="mesh" mesh="{j_asset_name}"'
                elif jv.type == "box":
                    size = " ".join([str(d / 2) for d in jv.dimensions])
                    j_geom_str = f'type="box" size="{size}"'
                elif jv.type == "cylinder":
                    radius = jv.dimensions[0]
                    height = jv.dimensions[1] / 2
                    j_geom_str = f'type="cylinder" size="{radius} {height}"'

                if j_geom_str:
                    rgb_str = "0.5 0.5 0.5 1"
                    if jv.color:
                        try:
                            r, g, b = (
                                int(jv.color[1:3], 16) / 255,
                                int(jv.color[3:5], 16) / 255,
                                int(jv.color[5:7], 16) / 255,
                            )
                            rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                        except:
                            pass
                    xml.append(
                        f'{indent}  <geom {j_geom_str} pos="{jv_pos}" euler="{jv_euler}" rgba="{rgb_str}" group="1" contype="0" conaffinity="0" />'
                    )
                    # REMOVED: Duplicate collision geom for joint visual (user requested clean up)

        # --- Link Visuals ---
        if link.visual and link.visual.type != "none":
            v = link.visual
            v_pos = "0 0 0"
            v_euler = "0 0 0"

            if v.type == "mesh" and v.meshOrigin:
                xyz = v.meshOrigin.get("xyz", [0.0, 0.0, 0.0])
                rpy = v.meshOrigin.get("rpy", [0.0, 0.0, 0.0])
                v_pos = " ".join(map(str, xyz))
                v_euler = " ".join(map(str, rpy))

            if v.type == "mesh" and link_id in mesh_files_map:
                # Lookup asset name
                asset_name = get_mesh_asset_name(
                    link_id, mesh_files_map[link_id], v.meshScale
                )

                # NO scale attribute here
                # Determine collision properties
                # REVERTED: Restricted to tips only to prevent self-locking.
                # "Use STL Mesh" means we use the visual mesh for collision geom, but only for tips.
                is_collidable = False
                nm_low = body_name.lower()
                if body_name.endswith("-end"):
                    is_collidable = True
                elif any(k in nm_low for k in ["tip", "distal", "end", "3rd"]):
                    is_collidable = True
                elif direct_hand and (
                    is_leaf
                    or "index" in nm_low
                    or "middle" in nm_low
                    or "ring" in nm_low
                    or "little" in nm_low
                    or "thumb" in nm_low
                ):
                    # Heuristic: If direct_hand mode, maybe we want more finger segments to collide?
                    # User said "finger ends". Let's stick to the stricter tip heuristic first or leaves.
                    pass
                    if is_leaf:
                        is_collidable = True

                c_val = "1" if is_collidable else "0"

                # Create Single Mesh Geom (Visual + Collision)
                # group="1" is standard for visual. In MJCF sample, group="1" is also used for collision mesh.
                vis_geom = f'type="mesh" mesh="{asset_name}" group="1" contype="{c_val}" conaffinity="{c_val}" condim="3" margin="0.002"'

                rgb_str = "0.5 0.5 0.5 1"
                if v.color:
                    try:
                        r, g, b = (
                            int(v.color[1:3], 16) / 255,
                            int(v.color[3:5], 16) / 255,
                            int(v.color[5:7], 16) / 255,
                        )
                        rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                    except:
                        pass
                color_attr = f'rgba="{rgb_str}"'

                xml.append(
                    f'{indent}  <geom {vis_geom} pos="{v_pos}" euler="{v_euler}" {color_attr} />'
                )

                # REMOVED: Duplicate primitive cylinder generation. User explicitly requested removal.

            elif v.type != "none" and v.type != "mesh":
                geom_str = ""
                if v.type == "box":
                    size = " ".join([str(d / 2) for d in v.dimensions])
                    geom_str = f'type="box" size="{size}"'
                elif v.type == "cylinder":
                    radius = v.dimensions[0]
                    height = v.dimensions[1] / 2
                    geom_str = f'type="cylinder" size="{radius} {height}"'
                elif v.type == "sphere":
                    radius = v.dimensions[0]
                    geom_str = f'type="sphere" size="{radius}"'

                if geom_str:
                    rgb_str = "0.5 0.5 0.5 1"
                    if v.color:
                        try:
                            r, g, b = (
                                int(v.color[1:3], 16) / 255,
                                int(v.color[3:5], 16) / 255,
                                int(v.color[5:7], 16) / 255,
                            )
                            rgb_str = f"{r:.3f} {g:.3f} {b:.3f} 1.0"
                        except:
                            pass

                    xml.append(
                        f'{indent}  <geom {geom_str} pos="{v_pos}" euler="{v_euler}" rgba="{rgb_str}" />'
                    )

        # Determine if we should add a sensor
        # is_leaf is already calculated at the top

        # Heuristic: User might name things "finger_tip" etc.
        name_lower = body_name.lower()
        is_target_name = (
            "tip" in name_lower or "finger" in name_lower or "hand" in name_lower
        )

        # Add sensor if leaf or specifically named, AND it has some visual (otherwise it's a dummy frame)
        # Actually, dummy frames are fine for sites.
        # Check if we should add sensors
        # For direct_hand, we correspond strictly to "last" links (leaves) or explicit "tip"
        # For standard, we use the broader heuristic

        apply_standard_sensor = False
        apply_direct_sensor = False

        if direct_hand:
            # User requirement: "very last" (leaf)
            # Also accept "tip", "distal", "end", "3rd" explicitly in name just in case it's not strictly a leaf in tree but logic implies it
            if is_leaf or any(k in name_lower for k in ["tip", "distal", "end", "3rd"]):
                apply_direct_sensor = True
        else:
            if is_leaf or is_target_name:
                apply_standard_sensor = True

        # Debug Print for User Feedback
        if "tip" in name_lower or "end" in name_lower or "thumb" in name_lower:
            print(
                f"[DEBUG_MJCF] Body: {body_name} | Leaf: {is_leaf} | Direct: {apply_direct_sensor} | Standard: {apply_standard_sensor}"
            )

        if apply_direct_sensor or apply_standard_sensor:
            if apply_direct_sensor:
                # --- Direct Hand Sensor Logic (Hardcoded Grid) ---
                # Coordinates from mjcf_sample.xml

                # Common Grid for Little, Ring, Middle, Index
                # Note: The usage in sample implies local frame coordinates.
                # The sample bodies are named like "new_link_12", "new_link_13" but the sensors are named "Little_..."
                pass

                sensor_configs = {
                    "little": [
                        ("0_1", "0.03   0.03   0.275"),
                        ("0_2", "0.03   0      0.28 "),
                        ("0_3", "0.03   -0.03  0.275"),
                        ("1_1", "0      0.03   0.275"),
                        ("1_2", "0      0      0.275"),
                        ("1_3", "0      -0.03  0.275"),
                        ("2_1", "-0.03  0.03   0.26"),
                        ("2_2", "-0.03  0      0.26"),
                        ("2_3", "-0.03  -0.03  0.26"),
                        ("3_1", "-0.045 0.03   0.23"),
                        ("3_2", "-0.045 0      0.23"),
                        ("3_3", "-0.045 -0.03  0.23"),
                        ("4_1", "-0.05  0.03   0.2"),
                        ("4_2", "-0.05  0      0.2"),
                        ("4_3", "-0.05  -0.03  0.2"),
                        ("5_1", "-0.06  0.03   0.17"),
                        ("5_2", "-0.06  0      0.17"),
                        ("5_3", "-0.06  -0.03  0.17"),
                        ("6_1", "-0.065 0.03   0.14"),
                        ("6_2", "-0.065 0      0.14"),
                        ("6_3", "-0.065 -0.03  0.14"),
                    ],
                    "ring": [
                        ("0_1", "0.03   0.03   0.275"),
                        ("0_2", "0.03   0      0.28 "),
                        ("0_3", "0.03   -0.03  0.275"),
                        ("1_1", "0      0.03   0.275"),
                        ("1_2", "0      0      0.275"),
                        ("1_3", "0      -0.03  0.275"),
                        ("2_1", "-0.03  0.03   0.26"),
                        ("2_2", "-0.03  0      0.26"),
                        ("2_3", "-0.03  -0.03  0.26"),
                        ("3_1", "-0.045 0.03   0.23"),
                        ("3_2", "-0.045 0      0.23"),
                        ("3_3", "-0.045 -0.03  0.23"),
                        ("4_1", "-0.05  0.03   0.2"),
                        ("4_2", "-0.05  0      0.2"),
                        ("4_3", "-0.05  -0.03  0.2"),
                        ("5_1", "-0.06  0.03   0.17"),
                        ("5_2", "-0.06  0      0.17"),
                        ("5_3", "-0.06  -0.03  0.17"),
                        ("6_1", "-0.065 0.03   0.14"),
                        ("6_2", "-0.065 0      0.14"),
                        ("6_3", "-0.065 -0.03  0.14"),
                    ],
                    "middle": [
                        ("0_1", "0.03   0.03   0.275"),
                        ("0_2", "0.03   0      0.28 "),
                        ("0_3", "0.03   -0.03  0.275"),
                        ("1_1", "0      0.03   0.275"),
                        ("1_2", "0      0      0.275"),
                        ("1_3", "0      -0.03  0.275"),
                        ("2_1", "-0.03  0.03   0.26"),
                        ("2_2", "-0.03  0      0.26"),
                        ("2_3", "-0.03  -0.03  0.26"),
                        ("3_1", "-0.045 0.03   0.23"),
                        ("3_2", "-0.045 0      0.23"),
                        ("3_3", "-0.045 -0.03  0.23"),
                        ("4_1", "-0.05  0.03   0.2"),
                        ("4_2", "-0.05  0      0.2"),
                        ("4_3", "-0.05  -0.03  0.2"),
                        ("5_1", "-0.06  0.03   0.17"),
                        ("5_2", "-0.06  0      0.17"),
                        ("5_3", "-0.06  -0.03  0.17"),
                        ("6_1", "-0.065 0.03   0.14"),
                        ("6_2", "-0.065 0      0.14"),
                        ("6_3", "-0.065 -0.03  0.14"),
                    ],
                    "index": [
                        ("0_1", "0.03   0.03   0.275"),
                        ("0_2", "0.03   0      0.28 "),
                        ("0_3", "0.03   -0.03  0.275"),
                        ("1_1", "0      0.03   0.275"),
                        ("1_2", "0      0      0.275"),
                        ("1_3", "0      -0.03  0.275"),
                        ("2_1", "-0.03  0.03   0.26"),
                        ("2_2", "-0.03  0      0.26"),
                        ("2_3", "-0.03  -0.03  0.26"),
                        ("3_1", "-0.045 0.03   0.23"),
                        ("3_2", "-0.045 0      0.23"),
                        ("3_3", "-0.045 -0.03  0.23"),
                        ("4_1", "-0.05  0.03   0.2"),
                        ("4_2", "-0.05  0      0.2"),
                        ("4_3", "-0.05  -0.03  0.2"),
                        ("5_1", "-0.06  0.03   0.17"),
                        ("5_2", "-0.06  0      0.17"),
                        ("5_3", "-0.06  -0.03  0.17"),
                        ("6_1", "-0.065 0.03   0.14"),
                        ("6_2", "-0.065 0      0.14"),
                        ("6_3", "-0.065 -0.03  0.14"),
                    ],
                    "thumb": [
                        ("0_1", "0.275 0.03   -0.03"),
                        ("0_2", "0.28     0   -0.03"),
                        ("0_3", "0.275 -0.03  -0.03"),
                        ("1_1", "0.275 0.03   0"),
                        ("1_2", "0.275 0      0"),
                        ("1_3", "0.275 -0.03  0"),
                        ("2_1", "0.26 0.03   0.03"),
                        ("2_2", "0.26 0      0.03"),
                        ("2_3", "0.26 -0.03  0.03"),
                        ("3_1", "0.23 0.03   0.045"),
                        ("3_2", "0.23 0      0.045"),
                        ("3_3", "0.23 -0.03   0.045"),
                        ("4_1", "0.2 0.03   0.05"),
                        ("4_2", "0.2 0      0.05"),
                        ("4_3", "0.2 -0.03  0.05"),
                        ("5_1", "0.17 0.03  0.06"),
                        ("5_2", "0.17 0     0.06"),
                        ("5_3", "0.17 -0.03 0.06"),
                        ("6_1", "0.14 0.03  0.065"),
                        ("6_2", "0.14 0     0.065"),
                        ("6_3", "0.14 -0.03 0.065"),
                    ],
                }

                found_config = None
                finger_prefix = "sensor"

                nm = body_name.lower()

                # FALLBACK: If body name is generic "new_link...", check the MESH NAME if available.
                # 'asset_name' is defined in the block above (Link Visuals) if it's a mesh.
                # Variable 'asset_name' scope is inside "if v.type == 'mesh'..." block.
                # We need to capture it.
                # Let's inspect 'link.visual.meshUrl' or similar if available here?
                # Actually, we can just look at 'mesh_files_map' again if needed or save it earlier.

                # Better approach: check 'asset_name' variable from earlier scope?
                # Python variable scoping in function: 'asset_name' might be defined if we entered that block.
                # But safer to re-derive or check 'link' visual properties.

                mesh_name_hint = ""
                if (
                    link.visual
                    and link.visual.type == "mesh"
                    and link_id in mesh_files_map
                ):
                    # We can try to guess from the filename in mesh_files_map
                    fpath = mesh_files_map[link_id]
                    mesh_name_hint = os.path.basename(fpath).lower()

                target_name = nm
                if mesh_name_hint:
                    target_name = nm + " " + mesh_name_hint  # Search both

                if "little" in target_name or "pinky" in target_name:
                    found_config, finger_prefix = sensor_configs["little"], "Little"
                elif "ring" in target_name:
                    found_config, finger_prefix = sensor_configs["ring"], "Ring"
                elif "middle" in target_name:
                    found_config, finger_prefix = sensor_configs["middle"], "Middle"
                elif "index" in target_name:
                    found_config, finger_prefix = sensor_configs["index"], "index"
                elif "thumb" in target_name:
                    found_config, finger_prefix = sensor_configs["thumb"], "thumb"

                if found_config:
                    print(f"[DEBUG_MJCF]   -> Config FOUND for {finger_prefix}")
                    for suffix, pos in found_config:
                        # Use body_name prefix to avoid collision if multiple hands/fingers exist
                        # e.g. "Right_Index_Tip_sensor_0_1"
                        site_name = f"{body_name}_sensor_{suffix}"

                        # Sample uses size="0.01" for all
                        xml.append(
                            f'{indent}  <site name="{site_name}" pos="{pos}" size="0.01" rgba="1 0 0 1"/>'
                        )

                        # Add sensor
                        sensor_name = f"sensor_{site_name}"
                        sensors.append(
                            f'    <touch name="{sensor_name}" site="{site_name}" />'
                        )

            # Fallback: If direct_hand is ON but we didn't find a matching finger config (e.g. wrist?),
            # do we add a standard sensor? User said "delete the old ones".
            # So if it IS a leaf but matches no finger, maybe we just don't add anything or add standard?
            # User said "The ones I made before are inside the link... delete them".
            # Safest is: If direct_hand is True, ONLY add the specialized grid. If no grid matches, add NOTHING.

            elif apply_standard_sensor:
                # Add site for sensor
                site_name = f"site_{body_name}"
                # Make site visible as a "pad" sensor on the surface
                # Heuristic: Thin box, offset significantly (4.5cm) to try and reach the fingertip surface
                xml.append(
                    f'{indent}  <site name="{site_name}" type="box" pos="0.045 0 0" size="0.008 0.015 0.002" rgba="0 1 0 0.5" />'
                )

                # Add sensor definition
                sensor_name = f"sensor_{body_name}"
                sensors.append(f'    <touch name="{sensor_name}" site="{site_name}" />')

        for child_joint_id in link.childJoints:
            child_joint = robot.joints.get(child_joint_id)
            if child_joint and child_joint.childLinkId:
                build_body(child_joint.childLinkId, child_joint_id, indent_level + 1)

        xml.append(f"{indent}</body>")

    build_body(robot.baseLinkId, indent_level=2)

    xml.append("  </worldbody>")

    if actuators:
        xml.append("  <actuator>")
        for act in actuators:
            xml.append(act)
        xml.append("  </actuator>")

    if sensors:
        xml.append("  <sensor>")
        for sens in sensors:
            xml.append(sens)
        xml.append("  </sensor>")

    xml.append("</mujoco>")
    xml.append("</mujoco>")
    return "\n".join(xml), generated_joints_info
