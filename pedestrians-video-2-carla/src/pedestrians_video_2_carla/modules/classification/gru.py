from typing import Type
from pedestrians_video_2_carla.data.base.skeleton import Skeleton
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from .classification import ClassificationModel

from torch import nn


class GRU(ClassificationModel):
    """
    Very basic Linear + GRU + Linear model.
    """

    def __init__(self,
                 input_nodes: Type[Skeleton] = CARLA_SKELETON,
                 hidden_size: int = 64,
                 num_layers: int = 2,
                 embeddings_size: int = None,
                 p_dropout: float = 0.25,
                 input_features=2,  # (x, y) points
                 **kwargs
                 ):
        super().__init__(**kwargs)

        self.__input_nodes_len = len(input_nodes)
        self.__input_features = input_features  # (x, y) points

        self.__input_size = self.__input_nodes_len * self.__input_features
        self.__output_size = self.num_classes
        self.__p_dropout = p_dropout

        if embeddings_size:
            self.__embeddings_size = embeddings_size
            self.linear_1 = nn.Linear(
                self.__input_size,
                self.__embeddings_size
            )
        else:
            self.__embeddings_size = self.__input_size
            self.linear_1 = lambda x: x

        self.gru_1 = nn.GRU(
            input_size=self.__embeddings_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True
        )
        self.linear_2 = nn.Linear(hidden_size, self.__output_size)
        self.dropout = nn.Dropout(self.__p_dropout)

        self._hparams.update({
            'hidden_size': hidden_size,
            'num_layers': num_layers,
            'embeddings_size': embeddings_size,
            'p_dropout': self.__p_dropout
        })

    @staticmethod
    def add_model_specific_args(parent_parser):
        ClassificationModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("GRU Classification Model")
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
        parser.add_argument(
            '--p_dropout',
            default=0.25,
            type=float,
        )

        return parent_parser

    def forward(self, x, *args, **kwargs):
        original_shape = x.shape
        x = x.view(*original_shape[0:2], self.__input_size)
        x = self.linear_1(x)
        self.dropout(x)
        x, _ = self.gru_1(x)
        out = self.linear_2(x)
        self.dropout(x)

        return out[:, -1, :]
