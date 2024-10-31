from pedestrians_video_2_carla.modules.flow.output_types import MovementsModelOutputType
import torch
from pedestrians_video_2_carla.modules.movements.movements import MovementsModel, MovementsModelOutputTypeMixin


class ZeroMovements(MovementsModelOutputTypeMixin, MovementsModel):
    """
    Dummy module that is not changing the input at all.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if self.movements_output_type not in [
            MovementsModelOutputType.pose_changes,
            MovementsModelOutputType.pose_2d,
        ]:
            raise ValueError('Unsupported movements output type: {}'.format(
                self.movements_output_type))

        self.a = torch.nn.Linear(1, 1)

    @staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = MovementsModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("Zero Movements Model")
        parser = MovementsModelOutputTypeMixin.add_cli_args(parser)

        return parent_parser

    def forward(self, x, *args, **kwargs):
        original_shape = x.shape

        # 3D pose changes
        if self.movements_output_type == MovementsModelOutputType.pose_changes:
            return torch.eye(3, device=x.device, requires_grad=True).reshape(
                (1, 1, 1, 3, 3)
            ).repeat(
                (*original_shape[:2], len(self.output_nodes), 1, 1)
            )
        else:
            return x

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=1e-4)

        config = {
            'optimizer': optimizer,
        }

        return config
