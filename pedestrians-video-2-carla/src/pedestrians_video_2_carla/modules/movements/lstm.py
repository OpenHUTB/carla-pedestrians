from torch import nn
from pedestrians_video_2_carla.modules.movements.movements import MovementsModel, MovementsModelOutputTypeMixin


class LSTM(MovementsModelOutputTypeMixin, MovementsModel):
    """
    Very basic Linear + LSTM + Linear model.
    """

    def __init__(self,
                 hidden_size: int = 64,
                 num_layers: int = 2,
                 embeddings_size: int = None,
                 **kwargs
                 ):
        super().__init__(**kwargs)

        self.__input_nodes_len = len(self.input_nodes)
        self.__input_features = 2  # (x, y) points

        self.__output_nodes_len = len(self.output_nodes)

        self.__input_size = self.__input_nodes_len * self.__input_features
        self.__output_size = self.__output_nodes_len * self.output_features

        if embeddings_size:
            self.__embeddings_size = embeddings_size
            self.linear_1 = nn.Linear(
                self.__input_size,
                self.__embeddings_size
            )
        else:
            self.__embeddings_size = self.__input_size
            self.linear_1 = lambda x: x

        self.lstm_1 = nn.LSTM(
            input_size=self.__embeddings_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True
        )
        self.linear_2 = nn.Linear(hidden_size, self.__output_size)

        self._hparams.update({
            'hidden_size': hidden_size,
            'num_layers': num_layers,
            'embeddings_size': embeddings_size
        })

    @staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = MovementsModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("LSTM Movements Model")
        parser = MovementsModelOutputTypeMixin.add_cli_args(parser)
        parser.add_argument(
            '--embeddings_size',
            default=None,
            type=int,
        )
        parser.add_argument(
            '--num_layers',
            default=2,
            type=int,
        )
        parser.add_argument(
            '--hidden_size',
            default=64,
            type=int,
        )
        return parent_parser

    def forward(self, x, *args, **kwargs):
        original_shape = x.shape
        x = x.view(*original_shape[0:2], self.__input_size)
        x = self.linear_1(x)
        x, _ = self.lstm_1(x)
        out = self.linear_2(x)
        out = out.view(*original_shape[0:2],
                       self.__output_nodes_len, self.output_features)
        return self._format_output(out)
