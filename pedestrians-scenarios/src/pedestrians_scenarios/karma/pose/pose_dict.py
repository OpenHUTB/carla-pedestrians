from typing import Dict, List, Tuple, Union

from pedestrians_scenarios.karma.utils.conversions import convert_list_to_transform, convert_list_to_vector2d, convert_transform_to_list, convert_vector2d_to_list

from .skeleton import CARLA_SKELETON
from .types import PoseDict, Pose2DDict

def get_pedestrian_pose_dicts(pedestrian: Union['Walker', 'carla.Walker']) -> Tuple[PoseDict, PoseDict, PoseDict]:
    # TODO: should this be here or cached as a part of KarmaDataProvider?
    bones = pedestrian.get_bones().bone_transforms

    world_pose = [None] * len(CARLA_SKELETON)
    component_pose = [None] * len(CARLA_SKELETON)
    relative_pose = [None] * len(CARLA_SKELETON)

    for bone in bones:
        # we only care about the body pose, so ignore other bones
        if bone.name not in CARLA_SKELETON.__members__:
            continue
        bone_idx = CARLA_SKELETON[bone.name].value
        world_pose[bone_idx] = bone.world
        component_pose[bone_idx] = bone.component
        relative_pose[bone_idx] = bone.relative

    return (
        dict(zip(CARLA_SKELETON, world_pose)),
        dict(zip(CARLA_SKELETON, component_pose)),
        dict(zip(CARLA_SKELETON, relative_pose)),
    )


def convert_pose_dict_to_list(pose: PoseDict) -> List[float]:
    pose_flat = []

    for bone in pose.values():
        pose_flat.append(convert_transform_to_list(bone))

    return pose_flat


def convert_list_to_pose_dict(pose_list: List[float]) -> PoseDict:
    pose = {}

    for bone in CARLA_SKELETON:
        pose[bone] = convert_list_to_transform(pose_list[bone.value])

    return pose


def convert_pose_2d_dict_to_list(pose: Pose2DDict) -> List[float]:
    pose_flat = []

    for bone in pose.values():
        pose_flat.append(convert_vector2d_to_list(bone))

    return pose_flat


def convert_list_to_pose_2d_dict(pose_list: List[float]) -> Pose2DDict:
    pose = {}

    for bone in CARLA_SKELETON:
        pose[bone] = convert_list_to_vector2d(pose_list[bone.value])

    return pose
