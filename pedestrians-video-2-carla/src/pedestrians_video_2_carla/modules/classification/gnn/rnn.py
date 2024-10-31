from pedestrians_video_2_carla.modules.classification.classification import ClassificationModel

import torch.nn.functional as F
from torch.nn import Linear
from torch_geometric.nn import global_mean_pool


class GRNNModel(ClassificationModel):
    def __init__(self,
                 hidden_size: int = 128,
                 p_dropout: float = 0.2,
                 input_features: int = 2,  # (x, y) points
                 **kwargs
                 ):
        super().__init__(**kwargs)

        self._hidden_size = hidden_size
        self._p_dropout = p_dropout
        self._input_features = input_features

        self.setup_input_layers()

        self.lin = Linear(self._hidden_size, self.num_classes)

        self._hparams.update({
            'hidden_size': self._hidden_size,
            'p_dropout': self._p_dropout,
        })

    @staticmethod
    def add_model_specific_args(parent_parser):
        ClassificationModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("LSTM Classification Model")
        parser.add_argument(
            '--hidden_size',
            default=64,
            type=int,
        )
        parser.add_argument(
            '--p_dropout',
            default=0.2,
            type=float,
        )

        return parent_parser

    @property
    def needs_graph(self) -> bool:
        return True

    def setup_input_layers(self):
        raise NotImplementedError

    def forward_input_layers(self, x, edge_index):
        x = self.rnn1(x, edge_index)
        x = x.relu()
        x = self.rnn2(x, edge_index)
        x = x.relu()

        return x

    def forward(self, x, edge_index, batch_vector):
        x = self.forward_input_layers(x, edge_index)
        x = global_mean_pool(x, batch_vector)  # [batch_size, hidden_channels]

        # final classifier
        x = F.dropout(x, p=self._p_dropout, training=self.training)
        x = self.lin(x)

        return x
