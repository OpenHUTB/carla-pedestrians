
from torch_geometric_temporal.nn import DCRNN
from pedestrians_video_2_carla.modules.classification.gnn.rnn import GRNNModel


class DCRNNModel(GRNNModel):
    def setup_input_layers(self):
        self.rnn1 = DCRNN(self._input_features, self._hidden_size, K=5)
        self.rnn2 = DCRNN(self._hidden_size, self._hidden_size, K=7)
