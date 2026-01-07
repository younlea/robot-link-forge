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
        mesh = trimesh.load(input_path, file_type='stl')
        
        if isinstance(mesh, trimesh.Scene):
             if len(mesh.geometry) == 0:
                 print("  Warning: Empty scene.")
                 return
             mesh = trimesh.util.concatenate([g for g in mesh.geometry.values()])

        num_faces = len(mesh.faces)
        print(f"  Faces: {num_faces}")

        LIMIT = 150000 
        
        if num_faces > LIMIT:
            print(f"  Face count {num_faces} exceeds safety limit {LIMIT}. Decimating...")
            try:
                mesh = mesh.simplify_quadric_decimation(face_count=LIMIT)
                print(f"  Decimated to: {len(mesh.faces)} faces")
            except Exception as e:
                print(f"  Decimation failed ({e}). Trying to export original.")
        
        if len(mesh.faces) == 0:
             print("  Warning: Mesh has 0 faces after load/decimate.")
        
        mesh.export(output_path, file_type='stl')
        
    except Exception as e:
        print(f"Error processing STL with trimesh: {e}. Falling back to raw copy.")
        if input_path != output_path:
            shutil.copy2(input_path, output_path)
