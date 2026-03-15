from typing import Tuple
import numpy as np

import pandas as pd
from pedestrians_video_2_carla.data.base.mixins.datamodule.benchmark_datamodule_mixin import BenchmarkDataModuleMixin
from pedestrians_video_2_carla.data.carla.datamodules.carla_recorded_datamodule import CarlaRecordedDataModule


class CarlaBenchmarkDataModule(BenchmarkDataModuleMixin, CarlaRecordedDataModule):
    def __init__(
        self,
        **kwargs
    ):
        super(CarlaBenchmarkDataModule, self).__init__(**{
            **kwargs,
            'extra_cols': {'crossing_point': int, 'crossing': int},
        })

    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        parent_parser = super().add_subclass_specific_args(parent_parser)
        parser = parent_parser.add_argument_group('CarlaBenchmark Data Module')
        parser = BenchmarkDataModuleMixin.add_cli_args(parser)

        return parent_parser

    def _clean_filter_sort_data(self, df: pd.DataFrame) -> pd.DataFrame:
        filtered_data = super()._clean_filter_sort_data(df)

        # add extra columns
        filtered_data['crossing_point'] = np.array(
            (-1,) * len(filtered_data))  # never crosses by default
        filtered_data['crossing'] = np.array(
            (2 if self._num_classes > 2 else 0,) * len(filtered_data))  # irrelevant or non-crossing by default

        crossing_groups = filtered_data.loc[filtered_data['frame.pedestrian.is_crossing']].groupby(
            self.primary_index)
        for name, group in crossing_groups:
            filtered_data.loc[name, 'crossing_point'] = group.head(1)['frame.idx']
            filtered_data.loc[name, 'crossing'] = 1

        return filtered_data
