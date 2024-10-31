from functools import lru_cache
import os
from typing import Any, Dict

import torch

from pedestrians_video_2_carla.data.base.utils import load_reference_file
from pedestrians_video_2_carla.data.smpl.constants import SMPL_BODY_MODEL_DIR, SMPL_MODELS
from pedestrians_video_2_carla.data.smpl.skeleton import SMPL_SKELETON
from pedestrians_video_2_carla.utils.tensors import eye_batch
from pytorch3d.transforms.rotation_conversions import euler_angles_to_matrix

# make it possible to use the generated dataset,
# even if generation itself is impossible
# e.g. generate dataset in docker somewhere and then copy it for the agents
try:   
    from human_body_prior.body_model.body_model import BodyModel
except ModuleNotFoundError:
    from pedestrians_video_2_carla.utils.exceptions import NotAvailableException

    class BodyModel:
        def __init__(self, *args, **kwargs):
            raise NotAvailableException("BodyModel", "smpl_renderer")


@lru_cache(maxsize=10)
def load(type: str) -> Dict[str, Any]:
    """
    Loads the file with reference pose extracted from UE4 engine.

    :param type: 'structure'.
    :type type: str
    :return: Dictionary containing pose structure.
    :rtype: Dict[str, Any]
    """
    try:
        filename = {
            "structure": 'structure.yaml',
        }[type]
    except KeyError:
        filename = type

    return load_reference_file(os.path.join(os.path.dirname(__file__), 'files', filename))


@lru_cache(maxsize=10)
def get_body_model(gender, device=torch.device('cpu'), body_model_dir=SMPL_BODY_MODEL_DIR, body_models=SMPL_MODELS):
    model_path = os.path.join(body_model_dir, body_models[gender])
    return BodyModel(bm_fname=model_path).to(device)


@lru_cache(maxsize=10)
def get_conventions_rot(device=torch.device('cpu')):
    return torch.tensor((
        (1.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
        (0.0, 1.0, 0.0)
    ), dtype=torch.float32, device=device).reshape((1, 3, 3))


def convert_smpl_pose_to_absolute_loc_rot(gender: str, reference_pose: 'P3dPose', pose_body=None, root_orient=None, device=torch.device('cpu')):
    """
    Gets absolute location & rotation for a sequence of body poses.

    :param gender: One of 'male', 'female', 'neutral'
    :type gender: str
    :param reference_pose: P3dPose with SMPL skeleton structure loaded
    :type reference_pose: P3dPose
    :param pose_body: Sequence of relative poses in SMPL tensor notation, defaults to None
    :type pose_body: Tensor, optional
    :param root_orient: Sequence of root orientations
    :type root_orient: Tensor, optional
    :param device: Defaults to torch.device('cpu')
    :type device: torch.Device, optional
    :return: Absolute location and rotation tensors
    :rtype: Tuple[Tensor, Tensor]
    """

    body_model = get_body_model(gender, device)
    clip_length = len(pose_body)
    nodes_len = len(SMPL_SKELETON)

    conventions_rot = get_conventions_rot(device).repeat((clip_length, 1, 1))

    absolute_loc = body_model(
        pose_body=pose_body,
        root_orient=root_orient
    ).Jtr[:, :nodes_len]
    absolute_loc = SMPL_SKELETON.map_from_original(absolute_loc)
    absolute_loc = torch.bmm(absolute_loc, conventions_rot)

    ref_rel_loc, ref_rel_rot = reference_pose.tensors
    relative_loc = ref_rel_loc.unsqueeze(dim=0).repeat((clip_length, 1, 1))

    if pose_body is None:
        relative_rot = ref_rel_rot.unsqueeze(dim=0).repeat((clip_length, 1, 1, 1))
        absolute_rot = eye_batch(clip_length, device=device)
    else:
        relative_rot = euler_angles_to_matrix(SMPL_SKELETON.map_from_original(
            torch.cat((
                torch.zeros((clip_length, 1, 3), device=device),
                pose_body.reshape((clip_length, nodes_len-1, 3))
            ), dim=1)),
            'XYZ'
        )
        _, absolute_rot = reference_pose.relative_to_absolute(
            relative_loc,
            relative_rot
        )

    return relative_loc, relative_rot, absolute_loc, absolute_rot
