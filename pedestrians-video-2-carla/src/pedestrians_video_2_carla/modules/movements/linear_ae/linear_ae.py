from torch import nn
from pedestrians_video_2_carla.modules.movements.movements import MovementsModel, MovementsModelOutputTypeMixin


class LinearAE(MovementsModelOutputTypeMixin, MovementsModel):
    """
    Very basic autoencoder utilizing only linear layers and ReLU.
    Inputs are flattened to a vector of size (clip_length * input_nodes_len * input_features).
    """
    # TODO: roll LinearAE2D into this class

    def __init__(self,
                 **kwargs
                 ):
        super().__init__(**kwargs)

        self.__input_nodes_len = len(self.input_nodes)
        self.__input_features = 2  # (x, y)

        self.__output_nodes_len = len(self.output_nodes)

        self.__input_size = self.__input_nodes_len * self.__input_features
        self.__output_size = self.__output_nodes_len * self.output_features

        self.__encoder = nn.Sequential(
            nn.Linear(self.__input_size, self.__input_size // 2),
            nn.ReLU(),
            nn.Linear(self.__input_size // 2, self.__input_size // 4),
            nn.ReLU(),
            nn.Linear(self.__input_size // 4, self.__input_size // 8),
            nn.ReLU(),
        )

        self.__decoder = nn.Sequential(
            nn.Linear(self.__input_size // 8, self.__output_size // 4),
            nn.ReLU(),
            nn.Linear(self.__output_size // 4, self.__output_size // 2),
            nn.ReLU(),
            nn.Linear(self.__output_size // 2, self.__output_size),
        )

    @staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = MovementsModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("LinearAE Model")
        parser = MovementsModelOutputTypeMixin.add_cli_args(parser)
        return parent_parser

    def forward(self, x, *args, **kwargs):
        original_shape = x.shape
        x = x.view((-1, self.__input_size))

        x = self.__encoder(x)
        outputs = self.__decoder(x)

        outputs = outputs.view(*original_shape[0:2],
                               self.__output_nodes_len, self.output_features)
        return self._format_output(outputs)
