import logging
import math
from typing import Any, Dict, Optional, Type

import numpy as np
import torch
from pedestrians_video_2_carla.data.base.base_dataset import BaseDataset, TorchIterableDataset, Projection2DMixin, ConfidenceMixin, GraphMixin
from pedestrians_video_2_carla.modules.layers.projection import \
    ProjectionModule
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pytorch3d.transforms import euler_angles_to_matrix

from pedestrians_video_2_carla.utils.tensors import get_bboxes

try:
    import h5pickle as h5py
except ModuleNotFoundError:
    import h5py
    import warnings
    warnings.warn("h5pickle not found, using h5py instead")


class Carla2D3DDataset(BaseDataset):
    def __init__(
        self,
        set_filepath: str,
        nodes: Type[CARLA_SKELETON] = CARLA_SKELETON,
        skip_metadata: bool = False,
        **kwargs
    ) -> None:
        super().__init__(data_nodes=nodes, **kwargs)

        self.set_file = h5py.File(set_filepath, 'r', driver='core')

        self.projection_2d = self.set_file['carla_2d_3d/projection_2d']

        if skip_metadata:
            self.meta = [{}] * len(self)
        else:
            self.meta = self.__decode_meta(self.set_file['carla_2d_3d/meta'])

    def __decode_meta(self, meta):
        logging.getLogger(__name__).debug(
            'Decoding meta for {}...'.format(self.set_file.filename))
        out = [{
            k: meta[k].attrs['labels'][v[idx]].decode("latin-1")
            for k, v in meta.items()
        } for idx in range(len(self))]
        logging.getLogger(__name__).debug('Meta decoding done.')

        return out

    def __len__(self) -> int:
        return len(self.projection_2d)

    def _get_raw_projection_2d(self, idx: int) -> torch.Tensor:
        return self.__extract_from_set('carla_2d_3d/projection_2d', idx), {}

    def _get_targets(self, idx: int, raw_projection_2d: torch.Tensor, intermediate_outputs: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        bboxes = get_bboxes(raw_projection_2d)

        # pose_changes_matrix = self.__extract_from_set(
        #     'carla_2d_3d/targets/pose_changes', idx)
        # world_rot_change_batch = self.__extract_from_set(
        #     'carla_2d_3d/targets/world_rot_changes', idx)
        # world_loc_change_batch = self.__extract_from_set(
        #     'carla_2d_3d/targets/world_loc_changes', idx)
        # relative_pose_loc = self.__extract_from_set(
        #     'carla_2d_3d/targets/relative_pose_loc', idx)
        # relative_pose_rot = self.__extract_from_set(
        #     'carla_2d_3d/targets/relative_pose_rot', idx)
        # absolute_pose_loc = self.__extract_from_set(
        #     'carla_2d_3d/targets/absolute_pose_loc', idx)
        # absolute_pose_rot = self.__extract_from_set(
        #     'carla_2d_3d/targets/absolute_pose_rot', idx)

        return {
            'bboxes': bboxes,
            # 'pose_changes': pose_changes_matrix,
            # 'world_loc_changes': world_loc_change_batch,
            # 'world_rot_changes': world_rot_change_batch,

            # # TODO: do we really need to keep those? maybe they should be calculated?
            # # speed vs memory issue? how to check what's the most efficient way?
            # # also, if we keep them, why not world_loc and world_rot?
            # 'relative_pose_loc': relative_pose_loc,
            # 'relative_pose_rot': relative_pose_rot,
            # 'absolute_pose_loc': absolute_pose_loc,
            # 'absolute_pose_rot': absolute_pose_rot,
        }

    def _get_meta(self, idx: int) -> Dict[str, Any]:
        return self.meta[idx]

    def __extract_from_set(self, set_name, idx):
        data = self.set_file[set_name][idx]
        return torch.from_numpy(data)


class Carla2D3DIterableDataset(Projection2DMixin, ConfidenceMixin, GraphMixin, TorchIterableDataset):
    def __init__(self,
                 batch_size: Optional[int] = 64,
                 clip_length: Optional[int] = 30,
                 random_changes_each_frame: Optional[int] = 3,
                 max_change_in_deg: Optional[int] = 5,
                 max_world_rot_change_in_deg: Optional[int] = 0,
                 max_initial_world_rot_change_in_deg: Optional[int] = 0,
                 points: Optional[Type[CARLA_SKELETON]] = CARLA_SKELETON,
                 **kwargs) -> None:
        super().__init__(**kwargs)

        self.nodes = points
        self.clip_length = clip_length
        self.random_changes_each_frame = random_changes_each_frame
        self.max_change_in_rad = np.deg2rad(max_change_in_deg)
        self.max_world_rot_change_in_rad = np.deg2rad(max_world_rot_change_in_deg)
        self.max_initial_world_rot_change_in_rad = np.deg2rad(
            max_initial_world_rot_change_in_deg)
        self.batch_size = batch_size

        self.projection = ProjectionModule(
            input_nodes=self.nodes,
            output_nodes=self.nodes
        )

    def __iter__(self):
        worker_info = torch.utils.data.get_worker_info()
        if worker_info is None:
            num_workers = 1
        else:
            num_workers = worker_info.num_workers

        bs = math.ceil(self.batch_size / num_workers)

        while True:
            inputs, targets, meta = self.generate_batch(bs)
            for idx in range(bs):
                out = (
                    inputs[idx],
                    {k: v[idx] for k, v in targets.items()},
                    {k: v[idx] for k, v in meta.items()}
                )
                yield self.process_graph(out)

    def generate_batch(self, batch_size=None):
        if batch_size is None:
            batch_size = self.batch_size
        nodes_size = len(self.nodes)
        nodes_nums = np.arange(nodes_size)
        pose_changes = torch.zeros(
            (batch_size, self.clip_length, nodes_size, 3))
        world_rot_change = torch.zeros(
            (batch_size, self.clip_length, 3))
        world_loc_change_batch = torch.zeros(
            (batch_size, self.clip_length, 3))

        for idx in range(batch_size):
            for i in range(self.clip_length):
                indices = np.random.choice(nodes_nums,
                                           size=self.random_changes_each_frame, replace=False)
                pose_changes[idx, i, indices] = (torch.rand(
                    (self.random_changes_each_frame, 3)) * 2 - 1) * self.max_change_in_rad

        pose_changes_batch = euler_angles_to_matrix(pose_changes, "XYZ")

        # only change yaw
        # TODO: for now, all initial rotations are equally probable
        if self.max_initial_world_rot_change_in_rad > 0:
            world_rot_change[:, 0, 2] = (torch.rand(
                (batch_size)) * 2 - 1) * self.max_initial_world_rot_change_in_rad
        # apply additional rotation changes during the clip
        if self.max_world_rot_change_in_rad != 0.0:
            world_rot_change[:, 1:, 2] = (torch.rand(
                (batch_size, self.clip_length-1)) * 2 - 1) * self.max_world_rot_change_in_rad
        world_rot_change_batch = euler_angles_to_matrix(world_rot_change, "XYZ")

        # TODO: introduce world location change at some point

        # TODO: we should probably take care of the "correct" pedestrians data distribution
        # need to find some pedestrian statistics
        age = np.random.choice(['adult', 'child'], size=batch_size)
        gender = np.random.choice(['male', 'female'], size=batch_size)

        self.projection.on_batch_start((pose_changes_batch, None, {
            'age': age,
            'gender': gender
        }), 0)
        orig_projection_2d, projection_outputs = self.projection(
            pose_inputs_batch=pose_changes_batch,
            world_rot_change_batch=world_rot_change_batch,
            world_loc_change_batch=world_loc_change_batch,
        )

        projection_2d, projection_targets = self.process_projection_2d(
            orig_projection_2d)
        projection_2d = self.process_confidence(projection_2d)

        return (
            projection_2d,
            {
                **projection_targets,

                'pose_changes': pose_changes_batch,
                'world_loc_changes': world_loc_change_batch,
                'world_rot_changes': world_rot_change_batch,

                **projection_outputs
            },
            {'age': age, 'gender': gender}
        )
