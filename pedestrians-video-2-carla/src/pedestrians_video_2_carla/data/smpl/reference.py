from functools import lru_cache
from typing import List
import torch

from pedestrians_video_2_carla.data.smpl.utils import convert_smpl_pose_to_absolute_loc_rot, load
from pedestrians_video_2_carla.walker_control.p3d_pose import P3dPose
from pedestrians_video_2_carla.data.smpl.skeleton import SMPL_SKELETON


SMPL_REFERENCE_SKELETON_TYPES = (
    ('adult', 'female'),
    ('adult', 'male'),
    ('adult', 'neutral'),
)


@lru_cache(maxsize=10)
def get_poses(device=torch.device('cpu'), as_dict=False):
    structure = load('structure')['structure']
    nodes_len = len(SMPL_SKELETON)
    poses: List[P3dPose] = []

    for _ in SMPL_REFERENCE_SKELETON_TYPES:
        p = P3dPose(structure=structure, device=device)

        p.tensors = (
            # TODO: get bones lengths from SMPL model
            torch.zeros((nodes_len, 3),
                        dtype=torch.float32, device=device),
            # reference pose in SMPL is all zeros
            torch.eye(3, device=device, dtype=torch.float32).reshape(
                (1, 3, 3)).repeat((nodes_len, 1, 1))
        )

        poses.append(p)

    if as_dict:
        return dict(zip(SMPL_REFERENCE_SKELETON_TYPES, poses))
    else:
        return poses


@lru_cache(maxsize=10)
def get_absolute_tensors(device=torch.device('cpu'), as_dict=False):
    poses = get_poses(device=device, as_dict=True)

    absolute_loc = []
    absolute_rot = []

    for (age, gender) in SMPL_REFERENCE_SKELETON_TYPES:
        relative_loc, relative_rot, abs_loc, abs_rot = convert_smpl_pose_to_absolute_loc_rot(
            gender=gender, reference_pose=poses[(age, gender)], device=device)

        absolute_loc.append(abs_loc)
        absolute_rot.append(abs_rot)

    if as_dict:
        return {
            k: (absolute_loc[i], absolute_rot[i])
            for i, k in enumerate(SMPL_REFERENCE_SKELETON_TYPES)
        }
    else:
        return (torch.stack(absolute_loc), torch.stack(absolute_rot))
