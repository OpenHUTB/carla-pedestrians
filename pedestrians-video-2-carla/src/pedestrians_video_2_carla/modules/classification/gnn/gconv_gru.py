from pedestrians_video_2_carla.modules.classification.gnn.rnn import GRNNModel

from torch_geometric_temporal.nn import GConvGRU


class GConvGRUModel(GRNNModel):
    def setup_input_layers(self):
        self.rnn1 = GConvGRU(self._input_features, self._hidden_size, 5)
        self.rnn2 = GConvGRU(self._hidden_size, self._hidden_size, 7)
