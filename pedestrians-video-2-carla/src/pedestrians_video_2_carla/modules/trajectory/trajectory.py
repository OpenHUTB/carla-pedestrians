from pedestrians_video_2_carla.modules.flow.base_model import BaseModel
from pedestrians_video_2_carla.modules.flow.output_types import TrajectoryModelOutputType


class TrajectoryModel(BaseModel):
    """
    Base model for trajectory prediction.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(prefix='trajectory', *args, **kwargs)

    @staticmethod
    def add_model_specific_args(parent_parser):
        BaseModel.add_model_specific_args(parent_parser, 'trajectory')
        return parent_parser

    @property
    def output_type(self) -> TrajectoryModelOutputType:
        return TrajectoryModelOutputType.changes
