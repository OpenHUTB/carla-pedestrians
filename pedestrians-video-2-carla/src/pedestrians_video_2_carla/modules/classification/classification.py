from pedestrians_video_2_carla.modules.flow.base_model import BaseModel
from pedestrians_video_2_carla.modules.flow.output_types import ClassificationModelOutputType


class ClassificationModel(BaseModel):
    def __init__(self,
                 num_classes: int = 2,
                 **kwargs):
        self.num_classes = num_classes

        super().__init__(prefix='classification', **kwargs)

    @property
    def output_type(self):
        return ClassificationModelOutputType.multiclass

    @staticmethod
    def add_model_specific_args(parent_parser):
        return BaseModel.add_model_specific_args(parent_parser, prefix='classification')
