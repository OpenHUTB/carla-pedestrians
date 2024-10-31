import os
from functools import lru_cache
from typing import Any, Dict, Tuple, Type, Union
import warnings

import torch
from pedestrians_video_2_carla.data.base.skeleton import get_common_indices
from pedestrians_video_2_carla.data.carla import reference as carla_reference
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.smpl.skeleton import SMPL_SKELETON
from pedestrians_video_2_carla.data.smpl.utils import (get_conventions_rot, load)
from pedestrians_video_2_carla.utils.tensors import eye_batch, get_bboxes
from pytorch3d.transforms.rotation_conversions import euler_angles_to_matrix
from pedestrians_video_2_carla.data.base.base_dataset import BaseDataset


class SMPLDataset(BaseDataset):
    def __init__(self,
                 set_filepath,
                 data_nodes: Union[Type[SMPL_SKELETON],
                                   Type[CARLA_SKELETON]] = SMPL_SKELETON,
                 device=torch.device('cpu'),
                 **kwargs
                 ) -> None:
        super().__init__(
            set_filepath=set_filepath,
            data_nodes=data_nodes,
            **kwargs
        )

        if self.data_nodes == CARLA_SKELETON and kwargs.get('skip_metadata', False):
            warnings.warn(
                'Skipping metadata when using CARLA_SKELETON results in using female adult pose for all clips.')

        self.nodes_len = len(self.data_nodes)

        self.structure = load('structure')['structure']
        self.device = device
        self.smpl_nodes_len = len(SMPL_SKELETON)

        # how to rotate axes when converting from SMPL to CARLA - rotation around X axis by 90deg
        self.reference_axes_rot = get_conventions_rot(device=self.device)
        # how to convert axis vales when converting from SMPL to CARLA - CARLA has negative X axis when compared to SMPL
        self.reference_axes_dir = torch.tensor((-1, 1, 1), device=self.device)

    def _get_targets(self, idx: int, raw_projection_2d: torch.Tensor, *args, **kwargs) -> Dict[str, torch.Tensor]:
        targets = super()._get_targets(idx, raw_projection_2d, *args, **kwargs)

        targets.update({
            'bboxes': get_bboxes(raw_projection_2d),

            'world_rot': torch.from_numpy(self.set_file['targets/world_rot'][idx]),

            # 'relative_pose_rot': torch.from_numpy(self.set_file['targets/relative_pose_rot'][idx]),
            # 'absolute_pose_rot': torch.from_numpy(self.set_file['targets/absolute_pose_rot'][idx]),
            # 'absolute_pose_loc': torch.from_numpy(self.set_file['targets/absolute_pose_loc'][idx]),
        })

        # if self.input_nodes == CARLA_SKELETON:
        #     m = self.meta[idx]
        #     age = m['age'] if 'age' in m else 'adult'
        #     gender = m['gender'] if 'gender' in m else 'female'

        #     smpl_pose = torch.from_numpy(self.set_file['targets/amass_body_pose'][idx])

        #     reference_pose = carla_reference.get_poses(device=self.device, as_dict=True)[(
        #         age, gender)]

        #     relative_rot, absolute_loc, absolute_rot = self.convert_smpl_to_carla(
        #         smpl_pose, age, gender, reference_pose)

        #     # those should be used less as targets and more as 'starting points'
        #     targets['carla_relative_pose_rot'] = relative_rot
        #     targets['carla_absolute_pose_rot'] = absolute_rot
        #     targets['carla_absolute_pose_loc'] = absolute_loc

        return targets

    @lru_cache(maxsize=10)
    def __get_local_rotation(self, clip_length, age, gender):
        _, carla_abs_ref_rot = carla_reference.get_absolute_tensors(self.device, as_dict=True)[
            (age, gender)]

        local_rot = torch.bmm(
            carla_abs_ref_rot,
            self.reference_axes_rot.repeat((len(carla_abs_ref_rot), 1, 1))
        ).unsqueeze(
            0).repeat((clip_length, 1, 1, 1))

        return local_rot

    @lru_cache(maxsize=10)
    def __get_carla_reference_relative_tensors(self, clip_length, age, gender):
        carla_rel_loc, carla_rel_rot = carla_reference.get_relative_tensors(self.device, as_dict=True)[
            (age, gender)]
        carla_rel_loc = carla_rel_loc.reshape(
            (1, self.nodes_len, 3)).repeat((clip_length, 1, 1))
        carla_rel_rot = carla_rel_rot.reshape(
            (1, self.nodes_len, 3, 3)).repeat((clip_length, 1, 1, 1))

        return carla_rel_loc, carla_rel_rot

    def convert_smpl_to_carla(self, smpl_pose, age, gender, reference_pose=None) -> Tuple[torch.Tensor, torch.Tensor]:
        clip_length = smpl_pose.shape[0]

        if reference_pose is None:
            reference_pose = carla_reference.get_poses(
                self.device, as_dict=True)[(age, gender)]

        local_rot = self.__get_local_rotation(clip_length, age, gender)

        # convert SMPL pose_body to changes rotation matrices
        nx_smpl_pose = SMPL_SKELETON.map_from_original(
            smpl_pose) * self.reference_axes_dir
        mapped_smpl = euler_angles_to_matrix(nx_smpl_pose, 'XYZ')

        # write changes for common joints (all SMPL except Spine2)
        ci, si = get_common_indices(SMPL_SKELETON)
        changes = eye_batch(clip_length, self.nodes_len, self.device)
        changes[:, ci] = mapped_smpl[:, si]

        # special spine handling, since SMPL has one more joint there
        changes[:, CARLA_SKELETON.crl_spine01__C.value] = torch.bmm(
            mapped_smpl[:, SMPL_SKELETON.Spine3.value],
            mapped_smpl[:, SMPL_SKELETON.Spine2.value]
        )

        # recalculate changes from the local reference system perspective
        local_changes = torch.bmm(
            torch.linalg.solve(local_rot.reshape((-1, 3, 3)),
                               changes.reshape((-1, 3, 3))),
            local_rot.reshape((-1, 3, 3))
        ).reshape((clip_length, -1, 3, 3))

        ref_carla_rel_loc, ref_carla_rel_rot = self.__get_carla_reference_relative_tensors(
            clip_length, age, gender)

        carla_abs_loc, carla_abs_rot, carla_rel_rot = reference_pose(local_changes,
                                                                     ref_carla_rel_loc,
                                                                     ref_carla_rel_rot)

        return carla_rel_rot, carla_abs_loc, carla_abs_rot
