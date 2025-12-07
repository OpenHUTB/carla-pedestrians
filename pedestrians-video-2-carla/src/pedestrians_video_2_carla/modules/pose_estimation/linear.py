from pedestrians_video_2_carla.modules.flow.output_types import PoseEstimationModelOutputType
from pedestrians_video_2_carla.utils.argparse import boolean
from torch import nn
from pedestrians_video_2_carla.modules.movements.movements import MovementsModel, MovementsModelOutputTypeMixin
from .pose_estimation import PoseEstimationModel


class Linear(PoseEstimationModel):
    """
    The simplest dummy model used to debug the flow.
    """

    def __init__(self,
                 stride=8,
                 **kwargs
                 ):
        super().__init__(
            **kwargs
        )

        self.__input_size = 3  # RGB
        self.__output_nodes_len = len(self.output_nodes)
        self.__output_size = self.__output_nodes_len + 1

        self.pool_center = nn.AvgPool2d(kernel_size=9, stride=stride, padding=1)
        self.linear = nn.Linear(
            self.__input_size,
            self.__output_size
        )

    @property
    def output_type(self) -> PoseEstimationModelOutputType:
        return PoseEstimationModelOutputType.heatmaps

    def forward(self, x, *args, **kwargs):
        b, t, c, h, w = x.shape
        x = x.view(b * t, c, h, w)
        x = self.pool_center(x)
        x = x.permute(0, 2, 3, 1)
        x = self.linear(x)
        x = x.permute(0, 3, 1, 2)

        rh, rw = x.shape[-2:]

        x = x.view(b, t, self.__output_size, rh, rw)

        return x
