from typing import Any, Callable, Dict, List, Tuple
import os
import numpy as np
import pandas as pd
from pedestrians_video_2_carla.data.base.mixins.datamodule.classification_datamodule_mixin import ClassificationDataModuleMixin

import torch
from pedestrians_video_2_carla.data.base.base_datamodule import BaseDataModule
from pedestrians_video_2_carla.data.base.mixins.datamodule.pandas_datamodule_mixin import PandasDataModuleMixin
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.carla.datasets.carla_recorded_dataset import CarlaRecordedDataset
import pandas as pd
import ast
from pytorch3d.transforms import euler_angles_to_matrix

from pedestrians_video_2_carla.utils.tensors import get_bboxes
from ..constants import CARLA_RECORDED_DIR, CARLA_RECORDED_DEFAULT_SET_NAME


def convert_to_list(x):
    try:
        return ast.literal_eval(x.replace('nan', '"nan"'))
    except ValueError:
        # for some reason pandas tries to convert the column name too...
        return str(x)


class CarlaRecordedDataModule(ClassificationDataModuleMixin, PandasDataModuleMixin, BaseDataModule):
    def __init__(self,
                 data_variant: str = CARLA_RECORDED_DEFAULT_SET_NAME,
                 **kwargs):
        self.data_variant = data_variant
        source_videos_dir = os.path.join(CARLA_RECORDED_DIR, self.data_variant)

        super().__init__(
            data_filepath=os.path.join(
                source_videos_dir, 'data.csv'),
            video_index=['id', 'camera.idx'],
            pedestrian_index=['pedestrian.idx'],
            clips_index=['clip', 'frame.idx'],
            converters={
                'camera.transform': convert_to_list,
                'pedestrian.spawn_point': convert_to_list,
                'frame.pedestrian.transform': convert_to_list,
                'frame.pedestrian.velocity': convert_to_list,
                'frame.pedestrian.pose.world': convert_to_list,
                'frame.pedestrian.pose.component': convert_to_list,
                'frame.pedestrian.pose.relative': convert_to_list,
                'frame.pedestrian.pose.camera': convert_to_list
            },
            **{
                'source_videos_dir': source_videos_dir,
                **kwargs
            }
        )

    @property
    def settings(self):
        return {
            **super().settings,
            'data_variant': self.data_variant,
        }

    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        parser = parent_parser.add_argument_group('CarlaRec Data Module')
        parser.add_argument('--data_variant', type=str,
                            default=CARLA_RECORDED_DEFAULT_SET_NAME)

        parser.set_defaults(
            data_nodes=CARLA_SKELETON,
            classification_targets_key='frame.pedestrian.is_crossing',
        )

        return parent_parser

    def _clean_filter_sort_data(self, df: pd.DataFrame) -> pd.DataFrame:
        df['camera.recording'] = df['camera.recording'].str.replace(
            '.mp4', '', regex=False)

        return super()._clean_filter_sort_data(df)

    def _clean_filter_sort_clips(self, clips: List[pd.DataFrame]) -> List[pd.DataFrame]:
        return [
            c
            for c in clips
            if self._has_pedestrian_in_all_frames(c)
        ]

    def _has_pedestrian_in_all_frames(self, clip: pd.DataFrame) -> bool:
        first_row = clip.iloc[0]

        frame_width = first_row.get('camera.width', 800)
        frame_height = first_row.get('camera.height', 600)

        projection_2d = np.array(
            clip.loc[:, 'frame.pedestrian.pose.camera'].to_list(), dtype=np.float32)

        has_pedestrian_in_all_frames = np.all(
            projection_2d >= 0) & np.all(
            projection_2d[..., 0] <= frame_width) & np.all(
            projection_2d[..., 1] <= frame_height)

        return has_pedestrian_in_all_frames

    def _get_raw_data(self, grouped: 'pd.DataFrameGroupBy') -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, Any]]:
        # projections
        projection_2d = self._reshape_to_sequences(
            grouped, 'frame.pedestrian.pose.camera')

        # targets
        bboxes = get_bboxes(torch.from_numpy(projection_2d))
        relative_pose_loc, relative_pose_rot = self._extract_transform(grouped,
                                                                       'frame.pedestrian.pose.relative'
                                                                       )
        absolute_pose_loc, absolute_pose_rot = self._extract_transform(grouped,
                                                                       'frame.pedestrian.pose.component'
                                                                       )
        world_pose_loc, world_pose_rot = self._extract_transform(grouped,
                                                                 'frame.pedestrian.pose.world'
                                                                 )
        world_loc, world_rot = self._extract_transform(grouped,
                                                       'frame.pedestrian.transform'
                                                       )
        velocity = self._reshape_to_sequences(grouped, 'frame.pedestrian.velocity')

        targets = {
            'bboxes': bboxes.numpy(),
            'relative_pose_loc': relative_pose_loc,
            'relative_pose_rot': relative_pose_rot,
            'absolute_pose_loc': absolute_pose_loc,
            'absolute_pose_rot': absolute_pose_rot,
            'world_pose_loc': world_pose_loc,
            'world_pose_rot': world_pose_rot,
            'world_loc': world_loc,
            'world_rot': world_rot,
            'velocity': velocity,
        }

        # meta
        grouped_head, grouped_tail = grouped.head(1).reset_index(
            drop=False), grouped.tail(1).reset_index(drop=False)
        meta = {
            'video_id': grouped_tail.loc[:, 'camera.recording'].to_list(),
            'pedestrian_id': grouped_tail.loc[:, ['camera.idx', 'pedestrian.idx']].apply(lambda x: '_'.join([str(y) for y in x]), axis=1).to_list(),
            'clip_id': grouped_tail.loc[:, 'clip'].to_numpy().astype(np.int32),
            'age': grouped_tail.loc[:, 'pedestrian.age'].to_list(),
            'gender': grouped_tail.loc[:, 'pedestrian.gender'].to_list(),
            'start_frame': grouped_head.loc[:, 'frame.idx'].to_numpy().astype(np.int32),
            'end_frame': grouped_tail.loc[:, 'frame.idx'].to_numpy().astype(np.int32) + 1,
            'clip_width': grouped_head.loc[:, 'camera.width'].to_numpy().astype(np.int32),
            'clip_height': grouped_head.loc[:, 'camera.height'].to_numpy().astype(np.int32),
        }

        self._add_classification_to_meta(grouped, grouped_tail, meta)

        return projection_2d, targets, meta

    def _extract_transform(self, grouped, column_name):
        carla_transforms = self._reshape_to_sequences(grouped, column_name)

        carla_transforms[..., 3:] = np.deg2rad(carla_transforms[..., 3:])
        carla_transforms = torch.from_numpy(carla_transforms)
        return carla_transforms[..., :3].numpy(), euler_angles_to_matrix(carla_transforms[..., 3:], 'XYZ').numpy()

    def _get_dataset_creator(self) -> Callable:
        return CarlaRecordedDataset

    def _set_class_labels(self, df: pd.DataFrame) -> None:
        """
        Sets classification labels for 'cross' column.

        :param df: DataFrame with labels
        :type df: pandas.DataFrame
        """
        self._class_labels = {
            # explicitly set crossing to be 1, so it potentially can be used in a binary classifier
            self._classification_targets_key: ['not-crossing', 'crossing'],
        }
