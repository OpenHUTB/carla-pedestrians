import math
import os
from typing import Any, Callable, Dict, Tuple
import warnings
import numpy as np

import pandas
import torch
from tqdm.auto import tqdm
from pedestrians_video_2_carla.data.base.base_datamodule import BaseDataModule
from pedestrians_video_2_carla.data.base.mixins.datamodule.pandas_datamodule_mixin import \
    PandasDataModuleMixin
from scipy.io import loadmat

from pedestrians_video_2_carla.utils.tensors import get_bboxes

from .constants import MPII_DIR, MPII_USECOLS
from .mpii_dataset import MPIIDataset
from .skeleton import MPII_SKELETON


class MPIIDataModule(PandasDataModuleMixin, BaseDataModule):
    def __init__(self,
                 data_variant: str = 'simple',
                 **kwargs
                 ):
        self.data_variant = data_variant

        super().__init__(
            set_name=MPII_DIR,
            data_filepath=os.path.join(MPII_DIR, 'mpii_human_pose_v1_u12_1.mat'),
            video_index=['video'],
            pedestrian_index=['image', 'rect_idx'],
            clips_index=['frame_sec'],
            df_usecols=MPII_USECOLS,
            **kwargs
        )

    @property
    def settings(self):
        return {
            **super().settings,
            'data_variant': self.data_variant,
        }

    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        parser = parent_parser.add_argument_group('MPII Data Module')
        parser.add_argument('--data_variant', type=str,
                            choices=['single', 'multiple'],
                            default='single')

        # update default settings
        parser.set_defaults(
            clip_length=1,
            clip_offset=1,
            test_set_frac=0,
            data_nodes=MPII_SKELETON
        )

        return parent_parser

    def _read_data(self) -> pandas.DataFrame:
        """Converts source *.mat file to pandas.DataFrame

        :return: _description_
        :rtype: pandas.DataFrame
        """
        mat_data = loadmat(self.data_filepath, simplify_cells=True)['RELEASE']
        anno_list = mat_data['annolist']
        video_list = mat_data['video_list']
        single_person_list = mat_data['single_person']
        train_images_mask = mat_data['img_train'].astype(bool)

        rows = []
        total = len(anno_list)
        for img_idx, image_annotations, is_train in tqdm(zip(range(total), anno_list, train_images_mask), desc='Reading data from *.mat', total=total, leave=False):
            if not is_train:
                # MPII dataset does not have test ground truth, so we skip it
                continue

            rectangles = image_annotations['annorect']
            if not isinstance(rectangles, list):
                rectangles = [rectangles]

            if self.data_variant == 'single':
                if isinstance(single_person_list[img_idx], int):
                    valid_rectangle_indices = [single_person_list[img_idx]-1]
                else:
                    valid_rectangle_indices = [i-1 for i in single_person_list[img_idx]]

                if not valid_rectangle_indices:
                    # no "sufficiently separated" individuals in this image
                    continue
            else:
                valid_rectangle_indices = range(len(rectangles))

            image_name = image_annotations['image']['name']
            vid_idx = image_annotations['vididx'] - \
                1 if image_annotations['vididx'] else None
            video_id = video_list[vid_idx] if vid_idx is not None else None
            frame_sec = image_annotations['frame_sec'] if image_annotations[
                'frame_sec'] or image_annotations['frame_sec'] == 0 else None

            for rect_idx in valid_rectangle_indices:
                rect = rectangles[rect_idx]
                if ('annopoints' in rect) and ('point' in rect['annopoints']):
                    anno_points = rect['annopoints']['point']
                    if not isinstance(anno_points, list):
                        anno_points = [anno_points]
                    keypoints = np.zeros((len(MPII_SKELETON), 2), dtype=np.float32)
                    joints_visibility = [True] * len(MPII_SKELETON)
                    for point in anno_points:
                        keypoints[point['id']][0] = point['x']
                        keypoints[point['id']][1] = point['y']
                        if 'is_visible' in point:
                            joints_visibility[point['id']] = bool(point['is_visible'])
                    row = {
                        'video': video_id,
                        'image': image_name,
                        'rect_idx': rect_idx,
                        'frame_sec': frame_sec,
                        'head_bbox': (rect['x1'], rect['y1'], rect['x2'], rect['y2']),
                        'keypoints': keypoints,
                        'joints_visibility': tuple(joints_visibility),
                        'scale': rect['scale'],
                        'objpos': (rect['objpos']['x'], rect['objpos']['y']),
                    }
                    rows.append(row)

        df = pandas.DataFrame(rows)
        df.set_index(self.primary_index, inplace=True)

        for k, v in self.extra_cols.items():
            df[k] = pandas.Series(dtype=v)

        return df

    def _get_dataset_creator(self) -> Callable:
        return MPIIDataset

    def _extract_clips(self, annotations_df: pandas.DataFrame) -> pandas.DataFrame:
        # do nothing, since MPII is 1 frame long
        return annotations_df

    def _concat_and_sort_clips(self, clips: pandas.DataFrame) -> pandas.DataFrame:
        clips.reset_index(drop=False, inplace=True)
        clips.set_index(self.full_index, inplace=True)
        clips.sort_index(inplace=True)
        return clips

    def _get_raw_data(self, grouped: pandas.DataFrame) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, Any]]:
        # projections
        projection_2d = self._reshape_to_sequences(grouped, 'keypoints')

        # targets
        bboxes = get_bboxes(torch.from_numpy(projection_2d).to(torch.float32)).numpy()

        targets = {
            'bboxes': bboxes
        }

        # meta
        meta, *_ = self._get_raw_meta(grouped)

        return projection_2d, targets, meta

    def _get_raw_meta(self, grouped: 'pandas.DataFrameGroupBy') -> Dict:
        grouped_head, grouped_tail = grouped.head(1).reset_index(
            drop=False), grouped.tail(1).reset_index(drop=False)

        meta = {
            'video_id': grouped_tail.loc[:, 'video'].to_list(),
            'image_id': grouped_tail.loc[:, 'image'].to_list(),
            'pedestrian_id': grouped_tail.loc[:, 'rect_idx'].to_list(),
        }

        return meta, grouped_head, grouped_tail
