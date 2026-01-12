from pydantic import BaseModel, Field
from typing import Dict, List, Optional

class Visual(BaseModel):
    type: str
    dimensions: Optional[List[float]] = None
    color: Optional[str] = None
    meshUrl: Optional[str] = Field(None, alias='meshUrl')
    meshScale: Optional[List[float]] = Field(None, alias='meshScale')
    meshOrigin: Optional[Dict[str, List[float]]] = Field(None, alias='meshOrigin')

class RobotLink(BaseModel):
    id: str
    name: str
    visual: Visual
    childJoints: List[str] = Field(alias='childJoints')

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

class RobotJoint(BaseModel):
    id: str
    name: str
    parentLinkId: str = Field(alias='parentLinkId')
    childLinkId: Optional[str] = Field(None, alias='childLinkId')
    type: str
    dof: JointDOF
    axis: Optional[List[float]] = None
    limits: Dict[str, JointLimit]
    origin: JointOrigin
    visual: Optional[Visual] = None
    visuals: Optional[Dict[str, Visual]] = None

class RobotData(BaseModel):
    links: Dict[str, RobotLink]
    joints: Dict[str, RobotJoint]
    baseLinkId: str = Field(alias='baseLinkId')
