from typing import Callable

from pedestrians_video_2_carla.data.base.mixins.dataset.video_mixin import VideoMixin
from pedestrians_video_2_carla.data.carla.datasets.carla_recorded_video_dataset import CarlaRecordedVideoDataset
from .carla_recorded_datamodule import CarlaRecordedDataModule


class CarlaRecordedVideoDataModule(CarlaRecordedDataModule):
    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        parent_parser = super().add_subclass_specific_args(parent_parser)

        parser = parent_parser.add_argument_group('CarlaRec Video Data Module')
        parser = VideoMixin.add_cli_args(parser)

        return parent_parser

    def _get_dataset_creator(self) -> Callable:
        return CarlaRecordedVideoDataset
