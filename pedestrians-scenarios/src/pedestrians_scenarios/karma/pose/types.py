from typing import Dict
from .skeleton import CARLA_SKELETON

PoseDict = Dict[CARLA_SKELETON, 'carla.Transform']
Pose2DDict = Dict[CARLA_SKELETON, 'carla.Vector2D']