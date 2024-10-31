from typing import Dict, Tuple, Type

from torch import Tensor
from pedestrians_video_2_carla.data.base.skeleton import register_skeleton, Skeleton
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from enum import Enum


class _ORIG_SMPL_SKELETON(Enum):
    """
    SMPL skeleton as defined in https://meshcapade.wiki/SMPL#related-models-the-smpl-family
    """
    Pelvis = 0
    L_Hip = 1
    R_Hip = 2
    Spine1 = 3
    L_Knee = 4
    R_Knee = 5
    Spine2 = 6
    L_Ankle = 7
    R_Ankle = 8
    Spine3 = 9
    L_Foot = 10
    R_Foot = 11
    Neck = 12
    L_Collar = 13
    R_Collar = 14
    Head = 15
    L_Shoulder = 16
    R_Shoulder = 17
    L_Elbow = 18
    R_Elbow = 19
    L_Wrist = 20
    R_Wrist = 21


class SMPL_SKELETON(Skeleton):
    """
    SMPL skeleton.
    **The indices are as used in P3dPose representation, NOT as in original SMPL.**
    """
    Pelvis = 0
    Spine1 = 1
    Spine2 = 2
    Spine3 = 3
    L_Collar = 4
    L_Shoulder = 5
    L_Elbow = 6
    L_Wrist = 7
    Neck = 8
    Head = 9
    R_Collar = 10
    R_Shoulder = 11
    R_Elbow = 12
    R_Wrist = 13
    R_Hip = 14
    R_Knee = 15
    R_Ankle = 16
    R_Foot = 17
    L_Hip = 18
    L_Knee = 19
    L_Ankle = 20
    L_Foot = 21

    @classmethod
    def get_colors(cls) -> Dict['SMPL_SKELETON', Tuple[int, int, int, int]]:
        # try to match OpenPose color scheme for easier visual comparison
        return {
            SMPL_SKELETON.Pelvis: (255, 0, 0, 192),
            SMPL_SKELETON.Spine1: (255, 0, 0, 128),
            SMPL_SKELETON.Spine2: (255, 0, 0, 128),
            SMPL_SKELETON.Spine3: (255, 0, 0, 128),
            SMPL_SKELETON.L_Collar: (170, 255, 0, 128),
            SMPL_SKELETON.L_Shoulder: (170, 255, 0, 255),
            SMPL_SKELETON.L_Elbow: (85, 255, 0, 255),
            SMPL_SKELETON.L_Wrist: (0, 255, 0, 255),
            SMPL_SKELETON.Neck: (255, 0, 0, 192),
            SMPL_SKELETON.Head: (255, 0, 85, 255),
            SMPL_SKELETON.R_Collar: (255, 85, 0, 128),
            SMPL_SKELETON.R_Shoulder: (255, 85, 0, 255),
            SMPL_SKELETON.R_Elbow: (255, 170, 0, 255),
            SMPL_SKELETON.R_Wrist: (255, 255, 0, 255),
            SMPL_SKELETON.R_Hip: (0, 255, 85, 255),
            SMPL_SKELETON.R_Knee: (0, 255, 170, 255),
            SMPL_SKELETON.R_Ankle: (0, 255, 255, 255),
            SMPL_SKELETON.R_Foot: (0, 255, 255, 255),
            SMPL_SKELETON.L_Hip: (0, 170, 255, 255),
            SMPL_SKELETON.L_Knee: (0, 85, 255, 255),
            SMPL_SKELETON.L_Ankle: (0, 0, 255, 255),
            SMPL_SKELETON.L_Foot: (0, 0, 255, 255),
        }

    @classmethod
    def get_root_point(cls) -> 'SMPL_SKELETON':
        return SMPL_SKELETON.Pelvis

    @classmethod
    def get_neck_point(cls) -> 'SMPL_SKELETON':
        return SMPL_SKELETON.Neck

    @classmethod
    def get_hips_point(cls) -> 'SMPL_SKELETON':
        return SMPL_SKELETON.Pelvis

    @classmethod
    def get_flip_mask(cls) -> Tuple[int]:
        return (
            SMPL_SKELETON.Pelvis.value,
            SMPL_SKELETON.Spine1.value,
            SMPL_SKELETON.Spine2.value,
            SMPL_SKELETON.Spine3.value,
            SMPL_SKELETON.R_Collar.value,
            SMPL_SKELETON.R_Shoulder.value,
            SMPL_SKELETON.R_Elbow.value,
            SMPL_SKELETON.R_Wrist.value,
            SMPL_SKELETON.Neck.value,
            SMPL_SKELETON.Head.value,
            SMPL_SKELETON.L_Collar.value,
            SMPL_SKELETON.L_Shoulder.value,
            SMPL_SKELETON.L_Elbow.value,
            SMPL_SKELETON.L_Wrist.value,
            SMPL_SKELETON.L_Hip.value,
            SMPL_SKELETON.L_Knee.value,
            SMPL_SKELETON.L_Ankle.value,
            SMPL_SKELETON.L_Foot.value,
            SMPL_SKELETON.R_Hip.value,
            SMPL_SKELETON.R_Knee.value,
            SMPL_SKELETON.R_Ankle.value,
            SMPL_SKELETON.R_Foot.value,
        )

    @staticmethod
    def map_from_original(tensor: Tensor) -> Tensor:
        assert tensor.ndim >= 2

        n = [slice(None)] * max(tensor.ndim - 2, 1)
        s = tensor.shape
        n.append(tuple([
            _ORIG_SMPL_SKELETON[k].value for k in SMPL_SKELETON.__members__.keys()
        ]))

        return tensor.reshape((*s[:len(n)-1], len(SMPL_SKELETON), 3))[n]

    @staticmethod
    def map_to_original(tensor: Tensor, reshape=True) -> Tensor:
        assert tensor.ndim == 3

        n = [slice(None)]
        s = tensor.shape
        n.append(tuple([
            SMPL_SKELETON[k].value for k in _ORIG_SMPL_SKELETON.__members__.keys()
        ]))

        mapped = tensor[n]

        return mapped.reshape((s[0], -1)) if reshape else mapped


register_skeleton('SMPL_SKELETON', SMPL_SKELETON, [
    (CARLA_SKELETON.crl_hips__C, SMPL_SKELETON.Pelvis),
    (CARLA_SKELETON.crl_spine__C, SMPL_SKELETON.Spine1),
    (CARLA_SKELETON.crl_spine01__C, SMPL_SKELETON.Spine3),
    (CARLA_SKELETON.crl_shoulder__L, SMPL_SKELETON.L_Collar),
    (CARLA_SKELETON.crl_arm__L, SMPL_SKELETON.L_Shoulder),
    (CARLA_SKELETON.crl_foreArm__L, SMPL_SKELETON.L_Elbow),
    (CARLA_SKELETON.crl_hand__L, SMPL_SKELETON.L_Wrist),
    (CARLA_SKELETON.crl_neck__C, SMPL_SKELETON.Neck),
    (CARLA_SKELETON.crl_Head__C, SMPL_SKELETON.Head),
    (CARLA_SKELETON.crl_shoulder__R, SMPL_SKELETON.R_Collar),
    (CARLA_SKELETON.crl_arm__R, SMPL_SKELETON.R_Shoulder),
    (CARLA_SKELETON.crl_foreArm__R, SMPL_SKELETON.R_Elbow),
    (CARLA_SKELETON.crl_hand__R, SMPL_SKELETON.R_Wrist),
    (CARLA_SKELETON.crl_thigh__R, SMPL_SKELETON.R_Hip),
    (CARLA_SKELETON.crl_leg__R, SMPL_SKELETON.R_Knee),
    (CARLA_SKELETON.crl_foot__R, SMPL_SKELETON.R_Ankle),
    (CARLA_SKELETON.crl_toe__R, SMPL_SKELETON.R_Foot),
    (CARLA_SKELETON.crl_thigh__L, SMPL_SKELETON.L_Hip),
    (CARLA_SKELETON.crl_leg__L, SMPL_SKELETON.L_Knee),
    (CARLA_SKELETON.crl_foot__L, SMPL_SKELETON.L_Ankle),
    (CARLA_SKELETON.crl_toe__L, SMPL_SKELETON.L_Foot),
])
