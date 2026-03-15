import json
import logging
import os
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import pandas
from pandas.core.frame import DataFrame
from pedestrians_video_2_carla.data.base.base_datamodule import BaseDataModule
from pedestrians_video_2_carla.data.base.mixins.datamodule.classification_datamodule_mixin import ClassificationDataModuleMixin
from pedestrians_video_2_carla.data.base.mixins.datamodule.pandas_datamodule_mixin import PandasDataModuleMixin
from pedestrians_video_2_carla.data.openpose.constants import OPENPOSE_DIR
from pedestrians_video_2_carla.data.openpose.openpose_dataset import \
    OpenPoseDataset
from tqdm.auto import tqdm

from pedestrians_video_2_carla.data.openpose.skeleton import BODY_25_SKELETON


class OpenPoseDataModule(ClassificationDataModuleMixin, PandasDataModuleMixin, BaseDataModule):
    def __init__(self,
                 dataset_dirname: str,  # e.g. 'PIE' or 'JAAD'
                 strong_points: Optional[float] = 0,
                 iou_threshold: Optional[float] = 0.1,
                 **kwargs
                 ):
        self.strong_points = strong_points
        self.iou_threshold = iou_threshold

        super().__init__(
            extra_cols={'keypoints': 'object'},
            strong_points=strong_points,  # pass as kwarg to base class, because OpenPoseDataset needs it
            **kwargs
        )

        self.openpose_dir = os.path.join(self.datasets_dir, dataset_dirname, OPENPOSE_DIR)

    @property
    def settings(self):
        return {
            **super().settings,
            'strong_points': self.strong_points,
            'iou_threshold': self.iou_threshold,
        }

    @staticmethod
    def add_subclass_specific_args(parent_parser):
        parser = parent_parser.add_argument_group('OpenPose DataModule')
        parser.add_argument(
            '--strong_points',
            help='''
                Strong points threshold. If set to a value, only clips with
                with at least strong_points fraction of keypoints are used.
                ''',
            type=float,
            default=0
        )
        parser.add_argument(
            '--iou_threshold',
            help='''
                If best skeleton candidate & pedestran bbox IoU is lower than this thereshold,
                returns all zeros (skeleton not detected), defaults to 0.1.
                ''',
            type=float,
            default=0.1
        )

        parser.set_defaults(
            data_nodes=BODY_25_SKELETON
        )

        return parent_parser

    def _is_strong_points(self, clip: DataFrame) -> None:
        # TODO: use threshold instead of all or nothing?
        keypoints_list = clip.loc[:, 'keypoints'].tolist()
        keypoints = np.stack(keypoints_list)

        if self.strong_points < 1.0:
            return np.any(keypoints[..., :2], axis=-1).sum() >= self.strong_points * np.prod(keypoints.shape[:-1])
        else:
            return np.all(np.any(keypoints[..., :2], axis=-1))

    def _clean_filter_sort_clips(self, clips: List[DataFrame]) -> List[DataFrame]:
        if self.strong_points:
            # remove clips that contain missing data
            return [
                c
                for c in clips
                if self._is_strong_points(c)
            ]

        return clips

    def _get_dataset_creator(self) -> Callable:
        return OpenPoseDataset

    def _extract_additional_data(self, clips: List[DataFrame]):
        """
        Extract skeleton data from keypoint files. This potentially modifies data in place!

        :param clips: List of DataFrames
        :type clips: List[DataFrame]
        """
        updated_clips = []
        for clip in tqdm(clips, desc='Extracting skeleton data', leave=False):
            pedestrian_info = clip.reset_index().sort_values('frame')

            set_name = pedestrian_info.iloc[0]['set_name'] if 'set_name' in pedestrian_info.columns else ''
            video_id = pedestrian_info.iloc[0]['video']
            start_frame = pedestrian_info.iloc[0]['frame']
            stop_frame = pedestrian_info.iloc[-1]['frame'] + 1

            keypoints_root = os.path.join(self.openpose_dir, set_name, video_id)
            if not os.path.exists(keypoints_root):
                logging.getLogger(__name__).warning(
                    "Keypoints dir not found: {}".format(keypoints_root))
                continue

            all_keypoints = True
            for i, f in enumerate(range(start_frame, stop_frame, 1)):
                keypoints_path = os.path.join(keypoints_root, '{:s}_{:0>12d}_keypoints.json'.format(
                    video_id,
                    f
                ))
                if not os.path.exists(keypoints_path):
                    logging.getLogger(__name__).warning(
                        "Keypoints file not found: {}".format(keypoints_path))
                    all_keypoints = False
                    break

                gt_bbox = pedestrian_info.iloc[i][[
                    'x1', 'y1', 'x2', 'y2']].to_numpy().reshape((2, 2)).astype(np.float32)
                with open(keypoints_path) as jp:
                    people = json.load(jp)['people']
                    if not len(people):
                        # OpenPose didn't detect anything in this frame - append empty array
                        pedestrian_info.at[pedestrian_info.index[i], 'keypoints'] = np.zeros(
                            (len(self.data_nodes), 3)).tolist()
                    else:
                        # select the pose with biggest IOU with base bounding box
                        candidates = [np.array(p['pose_keypoints_2d']).reshape(
                            (-1, 3)) for p in people]
                        pedestrian_info.at[pedestrian_info.index[i], 'keypoints'] = self._select_best_candidate(
                            candidates, gt_bbox).tolist()

            if all_keypoints:
                updated_clips.append(pedestrian_info)

        return updated_clips

    def _select_best_candidate(self, candidates: List[np.ndarray], gt_bbox: np.ndarray) -> np.ndarray:
        """
        Selects the pose with the biggest overlap with ground truth bounding box.
        If the IOU is smaller than `near_zero` value, it returns empty array.
        When calculating candidate bounding box, not detected keypoints (0,0) are ignored.

        :param candidates: [description]
        :type candidates: List[np.ndarray]
        :param gt_bbox: [description]
        :type gt_bbox: np.ndarray
        :return: Best pose candidate for specified bounding box.
        :rtype: np.ndarray
        """
        candidates_bbox = np.array([
            np.array([
                c[np.any(c[:, 0:2], axis=1), 0:2].min(axis=0),
                c[np.any(c[:, 0:2], axis=1), 0:2].max(axis=0)
            ])
            for c in candidates
        ])

        gt_min = gt_bbox.min(axis=0)
        candidates_min = candidates_bbox.min(axis=1)

        gt_max = gt_bbox.max(axis=0)
        candidates_max = candidates_bbox.max(axis=1)

        intersection_min = np.maximum(gt_min, candidates_min)
        intersection_max = np.minimum(gt_max, candidates_max)

        intersection_area = (intersection_max -
                             intersection_min + 1).prod(axis=1)
        intersection_area[intersection_area < 0] = 0
        gt_area = (gt_max - gt_min + 1).prod(axis=0)
        candidates_area = (candidates_max - candidates_min + 1).prod(axis=1)

        iou = intersection_area / \
            (gt_area + candidates_area - intersection_area)

        best_iou_idx = np.argmax(iou)

        if iou[best_iou_idx] < self.iou_threshold:
            return np.zeros((len(self.data_nodes), 3))

        return candidates[best_iou_idx]
