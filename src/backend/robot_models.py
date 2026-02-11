from pydantic import BaseModel, Field
from typing import Dict, List, Optional


class Visual(BaseModel):
    type: str
    dimensions: Optional[List[float]] = None
    color: Optional[str] = None
    meshUrl: Optional[str] = Field(None, alias="meshUrl")
    meshScale: Optional[List[float]] = Field(None, alias="meshScale")
    meshOrigin: Optional[Dict[str, List[float]]] = Field(None, alias="meshOrigin")


class RobotLink(BaseModel):
    id: str
    name: str
    visual: Visual
    childJoints: List[str] = Field(alias="childJoints")


class JointDOF(BaseModel):
    roll: bool
    pitch: bool
    yaw: bool


class JointLimit(BaseModel):
    lower: float
    upper: float


class JointOrigin(BaseModel):
    xyz: List[float]
    rpy: List[float]


# --- Rolling Contact Joint ---
class RollingContactParams(BaseModel):
    curvatureRadius: float = Field(alias="curvatureRadius")
    contactFriction: float = Field(alias="contactFriction")
    surfaceType: str = Field("convex", alias="surfaceType")


class RobotJoint(BaseModel):
    id: str
    name: str
    parentLinkId: str = Field(alias="parentLinkId")
    childLinkId: Optional[str] = Field(None, alias="childLinkId")
    type: str
    dof: JointDOF
    axis: Optional[List[float]] = None
    limits: Dict[str, JointLimit]
    origin: JointOrigin
    visual: Optional[Visual] = None
    rollingParams: Optional[RollingContactParams] = Field(None, alias="rollingParams")


# --- Tendon System ---
class TendonRoutingPoint(BaseModel):
    id: str
    linkId: str = Field(alias="linkId")
    localPosition: List[float] = Field(alias="localPosition")


class Tendon(BaseModel):
    id: str
    name: str
    type: str  # 'active' | 'passive'
    routingPoints: List[TendonRoutingPoint] = Field(
        default_factory=list, alias="routingPoints"
    )
    stiffness: float = 0.0
    damping: float = 0.0
    restLength: float = Field(0.0, alias="restLength")
    actuatorMotorId: Optional[str] = Field(None, alias="actuatorMotorId")
    momentArm: Optional[float] = Field(None, alias="momentArm")
    color: str = "#ff6600"
    width: float = 0.002


# --- Obstacle System (fixed-position) ---
class ObstaclePhysics(BaseModel):
    friction: float = 0.5
    solref: List[float] = [0.02, 1.0]
    solimp: List[float] = [0.9, 0.95, 0.001]


class Obstacle(BaseModel):
    id: str
    name: str
    shape: str  # 'box' | 'sphere' | 'cylinder'
    dimensions: List[float]
    position: List[float]
    rotation: List[float]
    color: str = "#888888"
    physics: ObstaclePhysics = ObstaclePhysics()
    enabled: bool = True


# --- Sensor System ---
class SensorDef(BaseModel):
    id: str
    type: str  # 'touch' | 'force'
    linkId: str = Field(alias="linkId")
    localPosition: List[float] = Field(alias="localPosition")
    localRotation: List[float] = Field(default=[0, 0, 0], alias="localRotation")
    siteName: str = Field(alias="siteName")


# --- Root Data (backward-compatible: new fields are Optional with defaults) ---
class RobotData(BaseModel):
    links: Dict[str, RobotLink]
    joints: Dict[str, RobotJoint]
    baseLinkId: str = Field(alias="baseLinkId")
    tendons: Optional[Dict[str, Tendon]] = Field(default_factory=dict)
    obstacles: Optional[Dict[str, Obstacle]] = Field(default_factory=dict)
    sensors: Optional[Dict[str, SensorDef]] = Field(default_factory=dict)
