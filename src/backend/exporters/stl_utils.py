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
        mesh.density = density

        mass = mesh.mass
        center_of_mass = mesh.center_mass
        inertia = mesh.moment_inertia

        # Apply scale factor if provided (e.g., STL in mm but simulation in m)
        # Mass scales as volume: scale³
        # Inertia scales as mass * length²: scale⁵
        if scale is not None:
            scale_factor = scale[0] * scale[1] * scale[2]  # Volume scaling
            mass *= scale_factor
            # Inertia scaling: scale⁵ = scale³ (mass) * scale² (length²)
            inertia_scale = scale_factor * (
                scale[0] * scale[0]
            )  # Assuming uniform scale
            inertia *= inertia_scale
            center_of_mass *= scale[0]  # Linear scaling for COM

        # Ensure reasonable values
        if mass < 0.0001:  # Less than 0.1 gram
            mass = 0.001  # Default to 1 gram for very small parts
        # Note: Removed upper limit - let real STL mass be used
        # Heavy robots need heavy links!

        # Ensure inertia is positive definite
        inertia_diag = np.diag(inertia)
        if np.any(inertia_diag <= 0) or np.any(np.isnan(inertia_diag)):
            # Use simple formula for cylinder
            h = mesh.bounds[1][2] - mesh.bounds[0][2]
            r = (
                max(
                    mesh.bounds[1][0] - mesh.bounds[0][0],
                    mesh.bounds[1][1] - mesh.bounds[0][1],
                )
                / 2
            )
            # Apply scale to dimensions
            if scale is not None:
                h *= scale[2]
                r *= scale[0]

            ixx = iyy = mass * (3 * r**2 + h**2) / 12
            izz = mass * r**2 / 2

            # Minimum inertia to avoid numerical issues
            min_inertia = mass * 0.0001
            ixx = max(ixx, min_inertia)
            iyy = max(iyy, min_inertia)
            izz = max(izz, min_inertia)

            inertia = np.diag([ixx, iyy, izz])

        return {
            "mass": float(mass),
            "center_of_mass": center_of_mass.tolist(),
            "inertia_matrix": inertia.tolist(),
            "inertia_diagonal": np.diag(inertia).tolist(),
        }

    except Exception as e:
        print(f"Error calculating inertia from STL: {e}")
        return None
