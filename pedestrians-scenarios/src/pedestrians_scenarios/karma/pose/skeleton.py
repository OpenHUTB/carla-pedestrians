from enum import Enum
from typing import Dict, List, Tuple, Union
import numpy as np

try:
    from scipy.sparse import coo_matrix
    import torch
    import torch_geometric
except ImportError:
    pass


class Skeleton(Enum):
    """
    Basic skeleton interface.
    """
    @classmethod
    def get_colors(cls) -> Dict['Skeleton', Tuple[int, int, int, int]]:
        # try to match OpenPose color scheme for easier visual comparison
        # entries should be in the same order as in the enum,
        # so that .values() returns the same order and can be used for indexing
        raise NotImplementedError()

    @classmethod
    def get_edges(cls) -> List[Tuple['Skeleton', 'Skeleton']]:
        raise NotImplementedError()

    # unify extraction of some points of interest
    @classmethod
    def get_root_point(cls) -> Union['Skeleton', None]:
        return None

    @classmethod
    def get_neck_point(cls) -> Union['Skeleton', List['Skeleton']]:
        raise NotImplementedError()

    @classmethod
    def get_hips_point(cls) -> Union['Skeleton', List['Skeleton']]:
        raise NotImplementedError()

    @classmethod
    def get_flip_mask(cls) -> Tuple[int]:
        raise NotImplementedError()

    @classmethod
    def get_edge_index(cls) -> 'torch.Tensor':
        """
        Helper function to get the edge index of the skeleton in the torch geometric format.
        """
        if getattr(cls, '_edge_index', None) is not None:
            return cls._edge_index

        row = [edge[0].value for edge in cls.get_edges()] + \
            [edge[1].value for edge in cls.get_edges()]
        col = [edge[1].value for edge in cls.get_edges()] + \
            [edge[0].value for edge in cls.get_edges()]
        data = np.ones(len(row))
        sparse_mtx = coo_matrix((data, (row, col)), shape=(len(cls), len(cls)))
        edge_index, edge_attrs = torch_geometric.utils.from_scipy_sparse_matrix(
            sparse_mtx)
        cls._edge_index = edge_index

        return cls._edge_index


class CARLA_SKELETON(Skeleton):
    crl_root = 0
    crl_hips__C = 1
    crl_spine__C = 2
    crl_spine01__C = 3
    crl_shoulder__L = 4
    crl_arm__L = 5
    crl_foreArm__L = 6
    crl_hand__L = 7
    crl_neck__C = 8
    crl_Head__C = 9
    crl_eye__L = 10
    crl_eye__R = 11
    crl_shoulder__R = 12
    crl_arm__R = 13
    crl_foreArm__R = 14
    crl_hand__R = 15
    crl_thigh__R = 16
    crl_leg__R = 17
    crl_foot__R = 18
    crl_toe__R = 19
    crl_toeEnd__R = 20
    crl_thigh__L = 21
    crl_leg__L = 22
    crl_foot__L = 23
    crl_toe__L = 24
    crl_toeEnd__L = 25

    @classmethod
    def get_colors(cls) -> Dict['CARLA_SKELETON', Tuple[int, int, int, int]]:
        # try to match OpenPose color scheme for easier visual comparison
        return {
            CARLA_SKELETON.crl_root: (0, 0, 0, 128),
            CARLA_SKELETON.crl_hips__C: (255, 0, 0, 192),
            CARLA_SKELETON.crl_spine__C: (255, 0, 0, 128),
            CARLA_SKELETON.crl_spine01__C: (255, 0, 0, 128),
            CARLA_SKELETON.crl_shoulder__L: (170, 255, 0, 128),
            CARLA_SKELETON.crl_arm__L: (170, 255, 0, 255),
            CARLA_SKELETON.crl_foreArm__L: (85, 255, 0, 255),
            CARLA_SKELETON.crl_hand__L: (0, 255, 0, 255),
            CARLA_SKELETON.crl_neck__C: (255, 0, 0, 192),
            CARLA_SKELETON.crl_Head__C: (255, 0, 85, 255),
            CARLA_SKELETON.crl_eye__L: (170, 0, 255, 255),
            CARLA_SKELETON.crl_eye__R: (255, 0, 170, 255),
            CARLA_SKELETON.crl_shoulder__R: (255, 85, 0, 128),
            CARLA_SKELETON.crl_arm__R: (255, 85, 0, 255),
            CARLA_SKELETON.crl_foreArm__R: (255, 170, 0, 255),
            CARLA_SKELETON.crl_hand__R: (255, 255, 0, 255),
            CARLA_SKELETON.crl_thigh__R: (0, 255, 85, 255),
            CARLA_SKELETON.crl_leg__R: (0, 255, 170, 255),
            CARLA_SKELETON.crl_foot__R: (0, 255, 255, 255),
            CARLA_SKELETON.crl_toe__R: (0, 255, 255, 255),
            CARLA_SKELETON.crl_toeEnd__R: (0, 255, 255, 255),
            CARLA_SKELETON.crl_thigh__L: (0, 170, 255, 255),
            CARLA_SKELETON.crl_leg__L: (0, 85, 255, 255),
            CARLA_SKELETON.crl_foot__L: (0, 0, 255, 255),
            CARLA_SKELETON.crl_toe__L: (0, 0, 255, 255),
            CARLA_SKELETON.crl_toeEnd__L: (0, 0, 255, 255),
        }

    @classmethod
    def get_edges(cls) -> List[Tuple['CARLA_SKELETON', 'CARLA_SKELETON']]:
        return [
            [CARLA_SKELETON.crl_root, CARLA_SKELETON.crl_hips__C],
            [CARLA_SKELETON.crl_hips__C, CARLA_SKELETON.crl_spine__C],
            [CARLA_SKELETON.crl_spine__C, CARLA_SKELETON.crl_spine01__C],
            [CARLA_SKELETON.crl_spine01__C, CARLA_SKELETON.crl_shoulder__L],
            [CARLA_SKELETON.crl_shoulder__L, CARLA_SKELETON.crl_arm__L],
            [CARLA_SKELETON.crl_arm__L, CARLA_SKELETON.crl_foreArm__L],
            [CARLA_SKELETON.crl_foreArm__L, CARLA_SKELETON.crl_hand__L],
            [CARLA_SKELETON.crl_spine01__C, CARLA_SKELETON.crl_neck__C],
            [CARLA_SKELETON.crl_neck__C, CARLA_SKELETON.crl_Head__C],
            [CARLA_SKELETON.crl_Head__C, CARLA_SKELETON.crl_eye__L],
            [CARLA_SKELETON.crl_Head__C, CARLA_SKELETON.crl_eye__R],
            [CARLA_SKELETON.crl_spine01__C, CARLA_SKELETON.crl_shoulder__R],
            [CARLA_SKELETON.crl_shoulder__R, CARLA_SKELETON.crl_arm__R],
            [CARLA_SKELETON.crl_arm__R, CARLA_SKELETON.crl_foreArm__R],
            [CARLA_SKELETON.crl_foreArm__R, CARLA_SKELETON.crl_hand__R],
            [CARLA_SKELETON.crl_hips__C, CARLA_SKELETON.crl_thigh__R],
            [CARLA_SKELETON.crl_thigh__R, CARLA_SKELETON.crl_leg__R],
            [CARLA_SKELETON.crl_leg__R, CARLA_SKELETON.crl_foot__R],
            [CARLA_SKELETON.crl_foot__R, CARLA_SKELETON.crl_toe__R],
            [CARLA_SKELETON.crl_toe__R, CARLA_SKELETON.crl_toeEnd__R],
            [CARLA_SKELETON.crl_hips__C, CARLA_SKELETON.crl_thigh__L],
            [CARLA_SKELETON.crl_thigh__L, CARLA_SKELETON.crl_leg__L],
            [CARLA_SKELETON.crl_leg__L, CARLA_SKELETON.crl_foot__L],
            [CARLA_SKELETON.crl_foot__L, CARLA_SKELETON.crl_toe__L],
            [CARLA_SKELETON.crl_toe__L, CARLA_SKELETON.crl_toeEnd__L],
        ]

    @classmethod
    def get_root_point(cls) -> 'CARLA_SKELETON':
        return CARLA_SKELETON.crl_root

    @classmethod
    def get_neck_point(cls) -> 'CARLA_SKELETON':
        return CARLA_SKELETON.crl_neck__C

    @classmethod
    def get_hips_point(cls) -> 'CARLA_SKELETON':
        return CARLA_SKELETON.crl_hips__C

    @classmethod
    def get_flip_mask(cls) -> Tuple[int]:
        return (
            CARLA_SKELETON.crl_root.value,
            CARLA_SKELETON.crl_hips__C.value,
            CARLA_SKELETON.crl_spine__C.value,
            CARLA_SKELETON.crl_spine01__C.value,
            CARLA_SKELETON.crl_shoulder__R.value,
            CARLA_SKELETON.crl_arm__R.value,
            CARLA_SKELETON.crl_foreArm__R.value,
            CARLA_SKELETON.crl_hand__R.value,
            CARLA_SKELETON.crl_neck__C.value,
            CARLA_SKELETON.crl_Head__C.value,
            CARLA_SKELETON.crl_eye__R.value,
            CARLA_SKELETON.crl_eye__L.value,
            CARLA_SKELETON.crl_shoulder__L.value,
            CARLA_SKELETON.crl_arm__L.value,
            CARLA_SKELETON.crl_foreArm__L.value,
            CARLA_SKELETON.crl_hand__L.value,
            CARLA_SKELETON.crl_thigh__L.value,
            CARLA_SKELETON.crl_leg__L.value,
            CARLA_SKELETON.crl_foot__L.value,
            CARLA_SKELETON.crl_toe__L.value,
            CARLA_SKELETON.crl_toeEnd__L.value,
            CARLA_SKELETON.crl_thigh__R.value,
            CARLA_SKELETON.crl_leg__R.value,
            CARLA_SKELETON.crl_foot__R.value,
            CARLA_SKELETON.crl_toe__R.value,
            CARLA_SKELETON.crl_toeEnd__R.value,
        )
