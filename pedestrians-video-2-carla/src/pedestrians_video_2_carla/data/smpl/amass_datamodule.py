import os
from typing import Any, Dict, Iterable, Tuple
import numpy as np
import pandas
import torch

from pedestrians_video_2_carla.data.base.base_datamodule import BaseDataModule
from pedestrians_video_2_carla.data.base.mixins.datamodule.pandas_datamodule_mixin import PandasDataModuleMixin
from pedestrians_video_2_carla.data.smpl.constants import AMASS_DIR
from pedestrians_video_2_carla.data.smpl.skeleton import SMPL_SKELETON
from pedestrians_video_2_carla.data.smpl.smpl_dataset import \
    SMPLDataset
from tqdm.std import tqdm
import glob
from pytorch3d.transforms.rotation_conversions import euler_angles_to_matrix
from pedestrians_video_2_carla.data.smpl import reference as smpl_reference
from pedestrians_video_2_carla.data.smpl.utils import convert_smpl_pose_to_absolute_loc_rot
from pedestrians_video_2_carla.walker_control.p3d_pose_projection import \
    P3dPoseProjection


class AMASSDataModule(PandasDataModuleMixin, BaseDataModule):
    def __init__(self,
                 **kwargs
                 ):
        self.available_datasets = []

        super().__init__(
            data_filepath=None,
            video_index=['dataset', 'id'],
            pedestrian_index=[],
            clips_index=['clip', 'frame'],
            **kwargs
        )

        self.amass_dir = os.path.join(self.datasets_dir, AMASS_DIR)

        if not os.path.exists(self.amass_dir):
            raise FileNotFoundError(
                f'AMASS directory not found at {self.amass_dir}.')

        self.smpl_nodes_len = len(SMPL_SKELETON)
        self.zero_world_loc = torch.zeros(
            (1, 3), dtype=torch.float32, device=torch.device('cpu'))
        self.zero_world_rot = torch.eye(
            3, dtype=torch.float32, device=torch.device('cpu')).reshape((1, 3, 3))

    @property
    def settings(self):
        return {
            **super().settings,
            'datasets': self.available_datasets
        }

    @staticmethod
    def add_subclass_specific_args(parent_parser):
        parser = parent_parser.add_argument_group('AMASS DataModule')

        parser.set_defaults(
            data_nodes=SMPL_SKELETON
        )

        return parent_parser

    def _read_data(self) -> pandas.DataFrame:
        # find out how many unique mocaps we have available
        # assumption - each file contains "unique" mocap, i.e. they could be randomly assigned to
        # different subsets (train, val, test)
        mocap_files = []
        base_len = len(self.amass_dir) + 1
        for mocap_file in glob.glob(os.path.join(self.amass_dir, '**', '*.npz'), recursive=True):
            mocap_files.append(mocap_file[base_len:])

        mocaps = []
        for mocap_name in tqdm(mocap_files, desc='Retrieving poses info', leave=False):
            with np.load(os.path.join(self.amass_dir, mocap_name), mmap_mode='r') as mocap:
                if 'poses' not in mocap:
                    continue
                mocaps.append({
                    'dataset': mocap_name.split(os.path.sep)[0],
                    'id': mocap_name.split(os.path.sep, 1)[1].rstrip('.npz'),
                    'poses': mocap['poses'],
                    'gender': mocap['gender'] if 'gender' in mocap else 'neutral',
                    'age': mocap['age'] if 'age' in mocap else 'adult'
                })

        mocaps_df = pandas.DataFrame(mocaps)

        # what datasets do we have?
        self.available_datasets = list(mocaps_df['dataset'].unique())

        return mocaps_df

    def _extract_clips(self, filtered_data: pandas.DataFrame) -> Iterable[pandas.DataFrame]:
        clips = []
        # Both CARLA and JAAD operate with FPS=30, while AMASS has FPS=60
        # actual decimation will be done on load, but we need to take this into account
        # when calculating the clips start and end.
        # Additionally, thanks to FPS being multiples, whe can get two clips that are interleaved,
        # which is better than trying to do the augmentation later on.
        fps_ratio = 2  # 60/30
        amass_clip_offset = self.clip_offset * fps_ratio
        amass_clip_length = self.clip_length * fps_ratio
        for _, mocap in tqdm(filtered_data.iterrows(), total=len(filtered_data), desc='Assembling clips', leave=False):
            start = 0
            end = mocap['poses'].shape[0] - amass_clip_length - fps_ratio + 1
            clip_idx = 0
            for amass_start_frame in range(start, end, amass_clip_offset):
                clip_info = {
                    'dataset': str(mocap['dataset']),
                    'id': str(mocap['id']),
                    'clip': clip_idx,
                    'gender': str(mocap['gender']),
                    'age': str(mocap['age']),
                    'frame': 0,
                    'projection_2d': np.array([]),
                    'relative_rot': np.array([]),
                    'absolute_loc': np.array([]),
                    'absolute_rot': np.array([]),
                    'world_rot': np.array([]),
                    'amass_body_pose': np.array([]),
                }
                clip_idx += 1

                amass_end_frame = amass_start_frame + amass_clip_length
                clip_length = (amass_end_frame - amass_start_frame) // fps_ratio

                amass_relative_pose_rot_rad = torch.tensor(
                    mocap['poses'][amass_start_frame:amass_end_frame:fps_ratio, :self.smpl_nodes_len*3], dtype=torch.float32)
                assert len(
                    amass_relative_pose_rot_rad) == clip_length, f'Clip has wrong length: actual {len(amass_relative_pose_rot_rad)}, expected {clip_length}'

                amass_relative_pose_rot_rad[:, 0:3], world_rot = self.__get_root_orient_and_world_rot(
                    amass_relative_pose_rot_rad)

                frames = pandas.DataFrame([clip_info] * clip_length)
                frames.loc[:, 'frame'] = list(range(clip_length))
                frames.loc[:, 'world_rot'] = [world_rot[i].numpy()
                                              for i in range(clip_length)]
                frames.loc[:, 'amass_body_pose'] = [amass_relative_pose_rot_rad[i].numpy().reshape((-1, 1))
                                                    for i in range(clip_length)]

                clips.append(frames)

                ################################################################
                # For debug run we're only getting a single clip from each mocap
                if self._fast_dev_run:
                    break
                ################################################################
        return clips

    def _extract_additional_data(self, clips: Iterable[pandas.DataFrame]) -> Iterable[pandas.DataFrame]:
        for clip in tqdm(clips, desc='Calculating projections', leave=False):
            clip_info = clip.iloc[0]
            clip_length = len(clip)

            # convert to absolute pose and projection
            relative_rot, absolute_loc, absolute_rot, projections = self.__get_clip_projection(
                smpl_pose=torch.from_numpy(
                    np.stack(clip.loc[:, 'amass_body_pose'].to_list()).squeeze(-1)),
                age=clip_info['age'],
                gender=clip_info['gender'],
                world_rot=torch.from_numpy(np.stack(clip.loc[:, 'world_rot'].to_list()))
            )

            clip.loc[:, 'projection_2d'] = [projections[i].numpy()
                                            for i in range(clip_length)]
            clip.loc[:, 'relative_rot'] = [relative_rot[i].numpy()
                                           for i in range(clip_length)]
            clip.loc[:, 'absolute_loc'] = [absolute_loc[i].numpy()
                                           for i in range(clip_length)]
            clip.loc[:, 'absolute_rot'] = [absolute_rot[i].numpy()
                                           for i in range(clip_length)]
        return clips

    def _get_raw_data(self, grouped: pandas.DataFrame) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, Any]]:
        # projections
        projection_2d = self._reshape_to_sequences(grouped, 'projection_2d')

        targets = {
            'relative_pose_rot': self._reshape_to_sequences(grouped, 'relative_rot'),
            'absolute_pose_loc': self._reshape_to_sequences(grouped, 'absolute_loc'),
            'absolute_pose_rot': self._reshape_to_sequences(grouped, 'absolute_rot'),
            'world_rot': self._reshape_to_sequences(grouped, 'world_rot'),
            'amass_body_pose': self._reshape_to_sequences(grouped, 'amass_body_pose').squeeze(),
        }

        # meta
        grouped_head, grouped_tail = grouped.head(1).reset_index(
            drop=False), grouped.tail(1).reset_index(drop=False)
        meta = {
            'video_id': grouped_tail.loc[:, 'dataset'].to_list(),
            'pedestrian_id': grouped_tail.loc[:, 'id'].to_list(),
            'clip_id': grouped_tail.loc[:, 'clip'].to_numpy().astype(np.int32),
            'age': grouped_tail.loc[:, 'age'].to_list(),
            'gender': grouped_tail.loc[:, 'gender'].to_list(),
            'start_frame': grouped_head.loc[:, 'frame'].to_numpy().astype(np.int32),
            'end_frame': grouped_tail.loc[:, 'frame'].to_numpy().astype(np.int32) + 1,
            # TODO: get some labels for the data one day
        }

        return projection_2d, targets, meta

    def __get_root_orient_and_world_rot(self, body_pose):
        """
        Tries to naively recover the root orientation of the body.
        """
        # TODO: try to make it better, recover changes in other planes than Z, but for now it will do
        # note for the future: do NOT set yaw rotation on root_orient, SMPL body mesh renderer cannot handle angles bigger than +/- 90deg

        batch_size = body_pose.shape[0]

        # try to determine which "canonical" orientation is closest to the original root orientation
        # this works best for somewhat longish clips
        # assumptions: camera is not moving during the clip and default human position is standing
        axes = body_pose[:, 0:3].clone()*2 / np.pi
        axes_rot = euler_angles_to_matrix(
            axes.mean(dim=0).round()*np.pi / 2, 'XYZ').round()

        root_orient_rot = torch.matmul(axes_rot, body_pose[:, 0:3].T).T

        # get only yaw axis - this is just approximation!
        yaw_rot = root_orient_rot.clone()
        yaw_rot[:, 0] = 0
        yaw_rot[:, 1] = 0
        yaw_rot_mtx = euler_angles_to_matrix(yaw_rot, 'XYZ')

        # reset, so that in the first frame we see the skeleton from the front
        first_frame = yaw_rot_mtx[0].T
        world_rot = torch.matmul(first_frame, yaw_rot_mtx)

        new_root_orient_euler = torch.zeros(
            (batch_size, 3), dtype=torch.float32, device=torch.device('cpu'))

        return new_root_orient_euler, world_rot

    def __get_clip_projection(self,
                              smpl_pose: torch.Tensor,
                              age: str = 'adult',
                              gender: str = 'female',
                              world_loc=None,
                              world_rot=None
                              ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        clip_length = smpl_pose.shape[0]

        if world_loc is None:
            world_loc = self.zero_world_loc.repeat((clip_length, 1))

        if world_rot is None:
            world_rot = self.zero_world_rot.repeat((clip_length, 1, 1))

        reference_pose = smpl_reference.get_poses(
            device=torch.device('cpu'), as_dict=True)[(age, gender)]
        _, relative_rot, absolute_loc, absolute_rot = convert_smpl_pose_to_absolute_loc_rot(
            gender=gender,
            reference_pose=reference_pose,
            pose_body=smpl_pose[:, 3:],
            root_orient=smpl_pose[:, :3],
            device=torch.device('cpu')
        )

        pose_projection = P3dPoseProjection(
            device=torch.device('cpu'),
            look_at=(0, 0, 0),
            camera_position=(3.1, 0, 0),
        )

        projections = pose_projection(
            absolute_loc,
            world_loc,
            world_rot,
        )

        return relative_rot, absolute_loc, absolute_rot, projections[..., :2]

    def _get_dataset_creator(self):
        return SMPLDataset
