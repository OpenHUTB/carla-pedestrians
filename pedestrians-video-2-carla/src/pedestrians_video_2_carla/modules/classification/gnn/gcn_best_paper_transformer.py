from typing import Dict, Union
import torch
from torch_geometric.nn import GCNConv, global_mean_pool, TransformerConv
from pedestrians_video_2_carla.modules.classification.classification import ClassificationModel

from torch.optim.lr_scheduler import StepLR

from pedestrians_video_2_carla.modules.flow.output_types import ClassificationModelOutputType

# This is the model from the paper: https://ieeexplore.ieee.org/document/8917118


class GCNBestPaperTransformer(ClassificationModel):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._num_input_nodes = len(self.input_nodes)
        self._n_heads = 1
        # self._out_channels = [64, 32]
        self._out_channels = [8, 4]

        self.conv1 = TransformerConv(
            in_channels=2,
            out_channels=self._out_channels[0],
            heads=self._n_heads,
            bias=True
        )
        self.conv2 = TransformerConv(
            in_channels=self._n_heads * self._out_channels[0],
            out_channels=self._out_channels[-1],
            heads=self._n_heads,
            bias=True
        )
        self.relu = torch.nn.ReLU()
        self.dropout = torch.nn.Dropout(p=0.5)
        self.linear = torch.nn.Linear(self._num_input_nodes, 1)

    def forward(self, x, edge_index, batch_vector):
        x = self.conv1(x, edge_index)
        x = self.relu(x)
        x = self.dropout(x)

        x = self.conv2(x, edge_index)
        x = self.relu(x)
        x = self.dropout(x)

        x = x.view((-1, self._num_input_nodes, self._out_channels[-1]*self._n_heads))
        x = torch.mean(x, dim=0)
        x = torch.mean(x, dim=-1)

        x = self.linear(x)

        return x

    @property
    def needs_graph(self):
        return True

    @property
    def output_type(self):
        return ClassificationModelOutputType.binary
