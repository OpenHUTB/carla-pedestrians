from pedestrians_video_2_carla.data.base.mixins.dataset.video_mixin import VideoMixin
from pedestrians_video_2_carla.data.carla.datasets.carla_recorded_dataset import CarlaRecordedDataset


class CarlaRecordedVideoDataset(VideoMixin, CarlaRecordedDataset):
    pass
