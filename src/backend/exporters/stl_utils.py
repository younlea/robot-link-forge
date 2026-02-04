import shutil
import os


def ensure_binary_stl(input_path: str, output_path: str):
    """
    Checks if an STL file is valid and converts to Binary using trimesh.
    If face count > 150,000, it decimates the mesh to be safe for MuJoCo (limit ~200k).
    """

    try:
        import trimesh
    except ImportError:
        print("trimesh not installed. Falling back to simple copy.")
        if input_path != output_path:
            shutil.copy2(input_path, output_path)
        return

    try:
        print(f"Processing STL with trimesh: {input_path}")
        mesh = trimesh.load(input_path, file_type="stl")

        if isinstance(mesh, trimesh.Scene):
            if len(mesh.geometry) == 0:
                print("  Warning: Empty scene.")
                return
            mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])

        num_faces = len(mesh.faces)
        print(f"  Faces: {num_faces}")

        LIMIT = 150000

        if num_faces > LIMIT:
            print(
                f"  Face count {num_faces} exceeds safety limit {LIMIT}. Decimating..."
            )
            try:
                mesh = mesh.simplify_quadric_decimation(face_count=LIMIT)
                print(f"  Decimated to: {len(mesh.faces)} faces")
            except Exception as e:
                print(f"  Decimation failed ({e}). Trying to export original.")

        if len(mesh.faces) == 0:
            print("  Warning: Mesh has 0 faces after load/decimate.")

        mesh.export(output_path, file_type="stl")

    except Exception as e:
        print(f"Error processing STL with trimesh: {e}. Falling back to raw copy.")
        if input_path != output_path:
            shutil.copy2(input_path, output_path)


def calculate_inertia_from_stl(
    stl_path: str, density: float = 1000.0, scale: list = None
):
    """
    Calculate mass and inertia from STL file.

    Args:
        stl_path: Path to STL file
        density: Material density in kg/m³ (default: 1000 for plastic/resin)
        scale: Mesh scale factors [sx, sy, sz] applied in simulation (e.g., [0.01, 0.01, 0.01])

    Returns:
        dict with 'mass', 'center_of_mass', 'inertia' (3x3 matrix)
        Returns None if calculation fails
    """
    try:
        import trimesh
        import numpy as np
    except ImportError:
        return None

    try:
        mesh = trimesh.load(stl_path, file_type="stl")

        if isinstance(mesh, trimesh.Scene):
            if len(mesh.geometry) == 0:
                return None
            mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])

        # Calculate physical properties
        # CRITICAL: STL files are typically in mm, but MuJoCo needs SI units (kg, m)

        # Set density in kg/mm³ (convert from kg/m³)
        mesh.density = density / 1e9

        # Get properties from trimesh (all in mm units with correct density)
        mass_in_mm_units = (
            mesh.mass
        )  # This is in kg (correct because density is in kg/mm³)
        center_of_mass_mm = mesh.center_mass  # in mm
        inertia_mm = mesh.moment_inertia  # in kg·mm²

        # Convert to SI units (meters)
        mass = mass_in_mm_units  # Already in kg
        center_of_mass = center_of_mass_mm / 1000  # mm to m
        inertia = inertia_mm / 1e6  # kg·mm² to kg·m²

        # Do NOT apply scale - MuJoCo scale is for visualization only
        # Mass and inertia are physical properties, not visual
        if scale is not None:
            pass  # Intentionally do nothing - keep physical values correct

        # Ensure reasonable values
        if mass < 0.0001:  # Less than 0.1 gram
            mass = 0.001  # Default to 1 gram for very small parts

        # CRITICAL: Validate and fix inertia to satisfy MuJoCo constraints
        # MuJoCo requires: Ixx + Iyy >= Izz, Iyy + Izz >= Ixx, Izz + Ixx >= Iyy
        inertia_diag = np.diag(inertia)
        ixx, iyy, izz = inertia_diag[0], inertia_diag[1], inertia_diag[2]

        # Check triangle inequality
        needs_fix = False
        if ixx + iyy < izz or iyy + izz < ixx or izz + ixx < iyy:
            needs_fix = True
            print(
                f"  WARNING: Inertia violates triangle inequality: [{ixx:.6f}, {iyy:.6f}, {izz:.6f}]"
            )

        if needs_fix or np.any(inertia_diag <= 0) or np.any(np.isnan(inertia_diag)):
            # Use simple box approximation instead
            # Get bounding box dimensions in mm
            bounds = mesh.bounds
            dx = (bounds[1][0] - bounds[0][0]) / 1000  # Convert to m
            dy = (bounds[1][1] - bounds[0][1]) / 1000
            dz = (bounds[1][2] - bounds[0][2]) / 1000

            # Box inertia: I = m/12 * (b² + c²) for each axis
            ixx = mass * (dy**2 + dz**2) / 12
            iyy = mass * (dx**2 + dz**2) / 12
            izz = mass * (dx**2 + dy**2) / 12

            # Ensure minimum values
            min_inertia = mass * 0.0001
            ixx = max(ixx, min_inertia)
            iyy = max(iyy, min_inertia)
            izz = max(izz, min_inertia)

            inertia = np.diag([ixx, iyy, izz])
            print(f"  -> Fixed to box approximation: [{ixx:.6f}, {iyy:.6f}, {izz:.6f}]")

        return {
            "mass": float(mass),
            "center_of_mass": center_of_mass.tolist(),
            "inertia_matrix": inertia.tolist(),
            "inertia_diagonal": np.diag(inertia).tolist(),
        }

    except Exception as e:
        print(f"Error calculating inertia from STL: {e}")
        return None
