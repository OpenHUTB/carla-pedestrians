from typing import Any, Dict, Tuple

import numpy as np
import pandas
from .openpose_datamodule import OpenPoseDataModule


class YorkUOpenPoseDataModule(OpenPoseDataModule):
    def __init__(self, converters=None, **kwargs):
        nc = kwargs.get('num_classes', 2)
        if nc == 2:
            def cross_converter(x): return x == '1'
        else:
            def cross_converter(x): return int(x) % nc

        super().__init__(
            converters=converters if converters is not None else {
                # single label in whole video telling if pedestrian will cross at some point:
                'crossing': cross_converter,
                # is pedestrian crossing in this particular frame:
                'cross': cross_converter,
            },
            **kwargs
        )

    def _get_raw_data(self, grouped: pandas.DataFrame) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, Any]]:
        # projections
        projection_2d = self._reshape_to_sequences(grouped, 'keypoints')

        # targets
        bboxes = np.stack([
            self._reshape_to_sequences(grouped, 'x1'),
            self._reshape_to_sequences(grouped, 'y1'),
            self._reshape_to_sequences(grouped, 'x2'),
            self._reshape_to_sequences(grouped, 'y2'),
        ], axis=-1).astype(np.float32)

        targets = {
            'bboxes': bboxes.reshape((*bboxes.shape[:-1], 2, 2))
        }

        # meta
        meta, *_ = self._get_raw_meta(grouped)

        return projection_2d, targets, meta

    def _get_raw_meta(self, grouped: 'pandas.DataFrameGroupBy') -> Dict:
        grouped_head, grouped_tail = grouped.head(1).reset_index(
            drop=False), grouped.tail(1).reset_index(drop=False)

        meta = {
            'set_name': grouped_tail.loc[:, 'set_name'].to_list() if 'set_name' in grouped_tail.columns else [''] * len(grouped_tail),
            'video_id': grouped_tail.loc[:, 'video'].to_list(),
            'pedestrian_id': grouped_tail.loc[:, 'id'].to_list(),
            'clip_id': grouped_tail.loc[:, 'clip'].to_numpy().astype(np.int32),
            'age': grouped_tail.loc[:, 'age'].to_list(),
            'gender': grouped_tail.loc[:, 'gender'].to_list(),
            'start_frame': grouped_head.loc[:, 'frame'].to_numpy().astype(np.int32),
            'end_frame': grouped_tail.loc[:, 'frame'].to_numpy().astype(np.int32) + 1,
            'clip_width': grouped_tail.loc[:, 'video_width'].to_numpy().astype(np.int32),
            'clip_height': grouped_tail.loc[:, 'video_height'].to_numpy().astype(np.int32),
        }

        self._add_classification_to_meta(grouped, grouped_tail, meta)

        return meta, grouped_head, grouped_tail

    def _set_class_labels(self, df: pandas.DataFrame) -> None:
        """
        Sets classification labels for 'crossing' column.

        :param df: DataFrame with labels
        :type df: pandas.DataFrame
        """
        self._class_labels = {
            # explicitly set crossing to be 1, so it potentially can be used in a binary classifier
            self._classification_targets_key: ['not-crossing', 'crossing'],
        }
