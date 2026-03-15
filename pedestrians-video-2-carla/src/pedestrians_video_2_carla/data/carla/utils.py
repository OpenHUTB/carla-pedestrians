from functools import lru_cache
from typing import Any, Dict
import os
import warnings

from pedestrians_scenarios.karma.utils.deepcopy import deepcopy_location, deepcopy_rotation
from pedestrians_video_2_carla.data.base.utils import load_reference_file
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
try:
    import carla
except (ImportError, ModuleNotFoundError) as e:
    import pedestrians_video_2_carla.carla_utils.mock_carla as carla
    warnings.warn("Using mock carla.", category=ImportWarning)


@lru_cache(maxsize=10)
def load(type: str) -> Dict[str, Any]:
    """
    Loads the file with reference pose extracted from UE4 engine.

    :param type: One of 'adult_female', 'adult_male', 'child_female', 'child_male', or 'structure'.
    :type type: str
    :return: Dictionary containing pose structure or transforms.
    :rtype: Dict[str, Any]
    """
    try:
        filename = {
            "adult_female": 'sk_female_relative.yaml',
            "adult_male": 'sk_male_relative.yaml',
            "child_female": 'sk_girl_relative.yaml',
            "child_male": 'sk_kid_relative.yaml',
            "structure": 'structure.yaml',
        }[type]
    except KeyError:
        filename = type

    return load_reference_file(os.path.join(os.path.dirname(__file__), 'files', filename))


def yaml_to_pose_dict(unreal_transforms: Dict[str, Dict[str, Dict[str, float]]], is_abs: bool = False) -> Dict[str, 'carla.Transform']:
    """
    Convert dict with transforms read from unreal into carla-usable format.

    :param unreal_transforms: Data loaded using `load_reference()['transforms']`
    :type unreal_transforms: Dict[str, Dict[str, Dict[str, float]]]
    :return: Transforms data mapped to carla.Transform
    :rtype: Dict[str, carla.Transform]
    """
    pose_dict = {
        bone_name: carla.Transform(
            location=carla.Location(
                x=transform_dict['location']['x']/100.0,
                y=transform_dict['location']['y']/100.0,
                z=transform_dict['location']['z']/100.0,
            ),
            rotation=carla.Rotation(
                pitch=transform_dict['rotation']['pitch'],
                yaw=transform_dict['rotation']['yaw'],
                roll=transform_dict['rotation']['roll'],
            )
        ) for (bone_name, transform_dict) in unreal_transforms.items()
    }
    # remember original hips transform
    root_hips_transform = carla.Transform(
        location=deepcopy_location(pose_dict[CARLA_SKELETON.crl_hips__C.name].location),
        rotation=deepcopy_rotation(pose_dict[CARLA_SKELETON.crl_root.name].rotation),
    )
    # set hips to (0,0,0)
    if is_abs:
        # update all positions in-place
        for name in pose_dict.keys():
            pose_dict[name].location.x -= root_hips_transform.location.x
            pose_dict[name].location.y -= root_hips_transform.location.y
            pose_dict[name].location.z -= root_hips_transform.location.z
    else:
        pose_dict[CARLA_SKELETON.crl_hips__C.name].location = carla.Location()
    return pose_dict, root_hips_transform
