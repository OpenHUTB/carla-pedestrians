import math
from typing import Dict, Union
from pedestrians_video_2_carla.modules.classification.classification import ClassificationModel
from torch_geometric_temporal.nn import GConvGRU
from torch import nn
import torch


class SpatialTemporalGNN(ClassificationModel):
    """
    Author: Abel García Romera
    Original code modified to fit with the standard flow & coding standards.
    """

    def __init__(self,
                 input_features: int = 3,  # (x, y, c) points
                 kernel_size=3,
                 **kwargs):
        super().__init__(**kwargs)

        self._input_features = input_features
        self._num_classes = self._num_classes
        self._num_input_nodes = len(self.input_nodes)

        # Definition of Conv layers:

        conv1mult = 1

        embeddings_size = self._input_features * conv1mult

        self.conv1 = GConvGRU(
            in_channels=self._input_features,
            out_channels=embeddings_size,
            K=kernel_size
        )

        self.conv2 = GConvGRU(
            in_channels=embeddings_size,
            out_channels=embeddings_size,
            K=kernel_size
        )

        # Definition of linear layers:

        self.size_in1 = embeddings_size * self._num_input_nodes
        size_out1 = int(self.size_in1 * 0.5)
        self.lin1 = nn.Linear(self.size_in1, size_out1)

        size_in2 = size_out1
        size_out2 = int(size_in2 * 0.5)
        self.lin2 = nn.Linear(size_in2, size_out2)

        self.lin3 = nn.Linear(size_out2, self._num_classes)

        # Definition of extras

        self.softmax = nn.Softmax(dim=-1)

        self.dropout3 = nn.Dropout(p=0.3)
        # self.dropout5 = nn.Dropout(p=0.5)
        # self.dropout7 = nn.Dropout(p=0.7)

        # Definition of activation functions

        self.relu = nn.ReLU()
        # self.sigmoid = nn.Sigmoid()

    @property
    def needs_graph(self) -> bool:
        return True

    @property
    def needs_confidence(self) -> bool:
        return True

    def forward(self, x, edge_index, batch):
        H_i = self.conv1(X=x, edge_index=edge_index)

        H_i = self.dropout3(H_i)

        H_i = self.relu(H_i)

        x = H_i

        #x = self.relu(x)

        x = x.view(int(math.ceil(batch.shape[0]/self._num_input_nodes)), self.size_in1)

        x = self.lin1(x)

        x = self.dropout3(x)

        x = self.relu(x)

        x = self.lin2(x)

        x = self.dropout3(x)

        x = self.relu(x)

        x = self.lin3(x)

        x = self.softmax(x)

        return x

    def configure_optimizers(self) -> Dict[str, Union[torch.optim.Optimizer, '_LRScheduler']]:
        optimizer = torch.optim.Adam(self.parameters(), lr=self.learning_rate)

        config = {
            'optimizer': optimizer,
        }

        return config
