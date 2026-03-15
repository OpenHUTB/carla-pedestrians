from pedestrians_video_2_carla.modules.movements.movements import MovementsModel, MovementsModelOutputTypeMixin
from pedestrians_video_2_carla.modules.movements.movements import MovementsModelOutputType

from torch.nn import TransformerEncoderLayer, TransformerEncoder


class SimpleTransformer(MovementsModelOutputTypeMixin, MovementsModel):
    def __init__(self, n_heads=4, **kwargs):
        super().__init__(**kwargs)

        self.input_size = len(self.input_nodes) * self.output_features
        self.n_heads = n_heads

        # ensure input_size is divisible by nhead
        assert self.input_size % self.n_heads == 0, f"input_size ({self.input_size}) must be divisible by n_heads"

        self.encoder_layer = TransformerEncoderLayer(
            d_model=self.input_size,
            nhead=self.n_heads,
            batch_first=True
        )
        self.encoder = TransformerEncoder(self.encoder_layer, num_layers=6)

    @staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = MovementsModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("Simple Transformer Model")
        parser = MovementsModelOutputTypeMixin.add_cli_args(parser)

        parser.add_argument(
            '--n_heads',
            type=int,
            default=4,
            help='the number of heads in the encoder/decoder of the transformer model'
        )

        parser.set_defaults(
            movements_output_type=MovementsModelOutputType.pose_2d,
            movements_lr=1e-3,
            movements_weight_decay=1e-2,
            movements_scheduler_type='CosineAnnealingWarmRestarts',
            movements_enable_lr_scheduler=True,
            movements_scheduler_step_size=30  # 30 epochs
        )

        return parent_parser

    def forward(self, x, **kwargs):
        orig_shape = x.shape
        x = x.view(orig_shape[0], orig_shape[1], -1)
        x = self.encoder(x)
        x = x.view(orig_shape)
        return x
