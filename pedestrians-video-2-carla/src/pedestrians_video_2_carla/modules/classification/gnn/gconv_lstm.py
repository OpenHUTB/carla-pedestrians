from pedestrians_video_2_carla.modules.classification.gnn.rnn import GRNNModel
from torch_geometric_temporal.nn import GConvLSTM


class GConvLSTMModel(GRNNModel):
    def setup_input_layers(self):
        self.rnn1 = GConvLSTM(self._input_features, self._hidden_size, 5)
        self.rnn2 = GConvLSTM(self._hidden_size, self._hidden_size, 7)

    def forward_input_layers(self, x, edge_index):
        x, _ = self.rnn1(x, edge_index)
        x = x.relu()
        x, _ = self.rnn2(x, edge_index)
        x = x.relu()

        return x
