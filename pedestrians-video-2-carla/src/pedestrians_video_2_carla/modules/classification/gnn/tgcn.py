
from torch_geometric_temporal.nn import TGCN

from pedestrians_video_2_carla.modules.classification.gnn.rnn import GRNNModel


class TGCNModel(GRNNModel):
    def setup_input_layers(self):
        self.rnn1 = TGCN(self._input_features, self._hidden_size)
        self.rnn2 = TGCN(self._hidden_size, self._hidden_size)
