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
        # 
        # Step 1: Calculate mass correctly assuming STL is in mm
        volume_mm3 = mesh.volume
        volume_m3 = volume_mm3 / 1e9  # 1 m³ = 1e9 mm³
        mass_kg = volume_m3 * density  # density is in kg/m³
        
        # Step 2: Get center of mass (in mm from STL)
        center_of_mass_mm = mesh.center_mass
        
        # Step 3: Calculate inertia in SI units (kg·m²)
        # trimesh.moment_inertia needs density set correctly
        mesh.density = density / 1e9  # Convert to kg/mm³ for mm-scale STL
        inertia_SI = mesh.moment_inertia / 1e6  # Convert mm² to m²: (kg·mm²) / 1e6 = kg·m²
        
        # Now we have SI units: mass in kg, COM in mm, inertia in kg·m²
        mass = mass_kg
        center_of_mass = center_of_mass_mm / 1000  # Convert mm to m
        inertia = inertia_SI

        # NOTE: MuJoCo scale in <mesh scale="..."/> only affects VISUAL geometry
        # Mass and inertia should be REAL physical values in SI units
        # So we do NOT apply scale to mass/inertia!
        # 
        # However, if the scale parameter here represents the actual physical scaling
        # (e.g., STL was designed at 10x size), then we DO need to scale mass.
        # 
        # Best practice: Assume STL is correct physical size in mm,
        # and scale in MuJoCo is just for visualization
        # → Do NOT scale mass/inertia
        #
        # But our current code expects to apply scale... let's keep it for backward compatibility
        # but document that this is wrong for most cases
        if scale is not None:
            # WARNING: This scales the physical properties, which is usually WRONG
            # Scale in MuJoCo <mesh> tag is for visualization only
            # TODO: Remove this scaling or make it optional
            pass  # Do NOT scale mass - use real physical values

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
