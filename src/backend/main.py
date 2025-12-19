from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import os
import uuid
import shutil

app = FastAPI()

# Allow all origins for development purposes
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Ensure the 'static/meshes' directory exists
MESH_DIR = "static/meshes"
os.makedirs(MESH_DIR, exist_ok=True)

# Mount the 'static' directory to serve files
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.post("/api/upload-stl")
async def upload_stl_file(file: UploadFile = File(...)):
    """
    Accepts an STL file, saves it with a unique name in the 'static/meshes'
    directory, and returns the URL to access it.
    """
    if not file.filename.lower().endswith('.stl'):
        raise HTTPException(status_code=400, detail="Invalid file type. Only .stl files are accepted.")

    # Generate a unique filename to prevent overwrites
    unique_filename = f"{uuid.uuid4().hex}.stl"
    file_path = os.path.join(MESH_DIR, unique_filename)

    try:
        # Save the uploaded file
        with open(file_path, "wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Could not save file: {e}")

    # Return the relative URL for the frontend to use
    return {"url": f"/{file_path}"}

@app.get("/")
def read_root():
    """
    Root endpoint for basic API health check.
    """
    return {"message": "RobotLinkForge Backend is running."}
