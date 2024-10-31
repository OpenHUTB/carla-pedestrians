from pedestrians_video_2_carla.utils.argparse import boolean
from torch import nn
from pedestrians_video_2_carla.modules.movements.movements import MovementsModel, MovementsModelOutputTypeMixin


class Linear(MovementsModelOutputTypeMixin, MovementsModel):
    """
    The simplest dummy model used to debug the flow.
    """

    def __init__(self,
                 needs_confidence: bool = False,
                 **kwargs
                 ):
        super().__init__(
            **kwargs
        )

        self.__input_nodes_len = len(self.input_nodes)
        self.__input_features = 3 if needs_confidence else 2

        self.__output_nodes_len = len(self.output_nodes)
        self.__needs_confidence = needs_confidence

        self.__input_size = self.__input_nodes_len * self.__input_features
        self.__output_size = self.__output_nodes_len * self.output_features

        self.linear = nn.Linear(
            self.__input_size,
            self.__output_size
        )

    @property
    def needs_confidence(self) -> bool:
        return self.__needs_confidence

    @ staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = MovementsModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("Linear Model")
        parser = MovementsModelOutputTypeMixin.add_cli_args(parser)
        parser.add_argument(
            '--needs_confidence',
            dest='needs_confidence',
            type=boolean,
            default=False,
        )
        return parent_parser

    def forward(self, x, *args, **kwargs):
        original_shape = x.shape
        x = x.view((-1, self.__input_size))
        pose_change = self.linear(x)
        pose_change = pose_change.view(*original_shape[0:2],
                                       self.__output_nodes_len, self.output_features)

        return self._format_output(pose_change)
