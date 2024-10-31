import torch
from torch.nn import Linear as Linear
from torch.nn import Sequential as Seq
from torch.nn import BatchNorm1d as BN
from torch.nn import ReLU, Identity
from torch_geometric.nn.conv import PointTransformerConv

from torch.optim.lr_scheduler import ReduceLROnPlateau
from pedestrians_video_2_carla.modules.movements.movements import MovementsModel
from pedestrians_video_2_carla.modules.movements.movements import MovementsModelOutputType


try:
    from torch_geometric.nn import GAE, VGAE, GCNConv
except ModuleNotFoundError:
    from pedestrians_video_2_carla.utils.exceptions import NotAvailableException

    # dummy class to ensure model list works
    # TODO: models listed as available should be actually available ;)
    class GNNLinearAutoencoder:
        def __init__(self, *args, **kwargs):
            raise NotAvailableException("GNNLinearAutoencoder", "gnn")

    class VariationalGcn:
        def __init__(self, *args, **kwargs):
            raise NotAvailableException("VariationalGCN", "gnn")


class SpatialGnn(MovementsModel):

    @property
    def needs_graph(self) -> bool:
        return True

    @property
    def output_type(self) -> MovementsModelOutputType:
        return MovementsModelOutputType.pose_2d

    def configure_optimizers(self):
        optimizer = torch.optim.AdamW(self.parameters(), lr=1e-3)

        lr_scheduler = {
            'scheduler': ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=50, min_lr=1e-6),
            'interval': 'epoch',
            'monitor': 'val_loss/primary'
        }

        config = {
            'optimizer': optimizer,
            'lr_scheduler': lr_scheduler,
        }

        return config

def MLP(channels, batch_norm=True):
    return Seq(*[
        Seq(Linear(channels[i - 1], channels[i]),
            BN(channels[i]) if batch_norm else Identity(), ReLU())
        for i in range(1, len(channels))
    ])

class TransformerBlock(torch.nn.Module):
    # taken from here: https://github.com/pyg-team/pytorch_geometric/blob/master/examples/point_transformer_classification.py
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.lin_in = Linear(in_channels, in_channels)
        self.lin_out = Linear(out_channels, out_channels)

        self.pos_nn = MLP([3, 64, out_channels], batch_norm=False)

        self.attn_nn = MLP([out_channels, 64, out_channels], batch_norm=False)

        self.transformer = PointTransformerConv(in_channels, out_channels,
                                                pos_nn=self.pos_nn,
                                                attn_nn=self.attn_nn)

    def forward(self, x, pos, edge_index):
        x = self.lin_in(x).relu()
        x = self.transformer(x, pos, edge_index)
        x = self.lin_out(x).relu()
        return x

class TransformerGNN(SpatialGnn):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.transformer = TransformerBlock(in_channels, out_channels)

    def forward(self, x, edge_index, pos=3, **kwargs):
        x = self.transformer(x, pos, edge_index)
        return x

class GCNEncoder(SpatialGnn):
    def __init__(self, in_channels, out_channels, **kwargs):
        super().__init__()
        self.mult_factor = 256
        self.conv1 = GCNConv(in_channels, self.mult_factor * out_channels)
        self.conv2 = GCNConv(self.mult_factor * out_channels,
                             self.mult_factor * out_channels)
        self.conv3 = GCNConv(self.mult_factor * out_channels, out_channels)

    def forward(self, x, edge_index, **kwargs):
        x = self.conv1(x, edge_index).relu()
        x = self.conv2(x, edge_index).relu()
        return self.conv3(x, edge_index)


class VariationalGCNEncoder(torch.nn.Module):
    def __init__(self, in_channels, out_channels, **kwargs):
        super().__init__()
        self.mult_factor = 256
        self.conv1 = GCNConv(in_channels, self.mult_factor  * out_channels)
        self.conv2 = GCNConv(self.mult_factor  * out_channels, self.mult_factor  * out_channels)
        self.conv_mu = GCNConv(self.mult_factor  * out_channels, out_channels)
        self.conv_logstd = GCNConv(self.mult_factor  * out_channels, out_channels)

    def forward(self, x, edge_index, **kwargs):
        x = self.conv1(x, edge_index).relu()
        x = self.conv2(x, edge_index).relu()
        return self.conv_mu(x, edge_index), self.conv_logstd(x, edge_index)


class LinearEncoder(torch.nn.Module):
    def __init__(self, in_channels, out_channels, **kwargs):
        super().__init__()
        self.conv = GCNConv(in_channels, out_channels)

    def forward(self, x, edge_index):
        return self.conv(x, edge_index)


class LinearDecoder(torch.nn.Module):
    def __init__(self, in_channels, out_channels, **kwargs):
        super().__init__()
        self.conv = GCNConv(in_channels, out_channels)

    def forward(self, x, edge_index):
        return self.conv(x, edge_index)

class VariationalGcn(SpatialGnn):
    def __init__(self, in_channels=2, out_channels=2, **kwargs):
        super().__init__()
        self.model = VGAE(VariationalGCNEncoder(
            in_channels=in_channels, 
            out_channels=out_channels)
            )
    
    def forward(self, x, edge_index, **kwargs):
        return self.model.encode(x, edge_index)



class GNNLinearAutoencoder(SpatialGnn):
    def __init__(self, in_channels=2, out_channels=16, **kwargs):
        super().__init__()
        self.in_channels = in_channels
        self.encoder = LinearEncoder(in_channels, out_channels)
        self.decoder = LinearDecoder(out_channels, in_channels)

    def forward(self, x, edge_index, **kwargs):
        original_shape = x.shape
        x = self.encoder(x, edge_index)
        x = self.decoder(x, edge_index)
        return x


class VariationalLinearEncoder(SpatialGnn):
    def __init__(self, in_channels, out_channels, **kwargs):
        super().__init__()
        self.conv_mu = GCNConv(in_channels, out_channels)
        self.conv_logstd = GCNConv(in_channels, out_channels)

    def forward(self, x, edge_index):
        return self.conv_mu(x, edge_index), self.conv_logstd(x, edge_index)
