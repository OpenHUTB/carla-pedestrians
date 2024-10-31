import torch
from pedestrians_video_2_carla.modules.pose_estimation.pose_estimation import PoseEstimationModel
from pedestrians_video_2_carla.utils.gaussian_kernel import gaussian_kernel


from torchvision.models.resnet import resnet101, resnet50
from .overrides import UniPoseLSTM as _UniPoseLSTM


class UniPoseLSTM(PoseEstimationModel):
    def __init__(self,
                 stride: int = 8,
                 output_stride: int = 16,
                 backbone: str = 'resnet101',
                 **kwargs
                 ):
        super().__init__(**kwargs)

        self._output_nodes_len = len(self.output_nodes)
        self._stride = stride
        self._output_stride = output_stride
        self._sigma = kwargs.get('sigma', 3)  # match with the dataset

        self.unipose = _UniPoseLSTM(
            num_classes=self._output_nodes_len,
            backbone=backbone,
            stride=self._stride,
            output_stride=self._output_stride,
            sync_bn=True,
            freeze_bn=False
        )

        # get the original ResNet weights
        orig_backbone = {
            'resnet50': resnet50,
            'resnet101': resnet101,
        }[backbone](pretrained=True, progress=True)
        orig_backbone_sd = orig_backbone.state_dict()

        # we need to replace matching backbone weights with our original
        backbone_sd = self.unipose.backbone.state_dict()
        for k in backbone_sd:
            if k in orig_backbone_sd:
                backbone_sd[k] = orig_backbone_sd[k]

        self.unipose.backbone.load_state_dict(backbone_sd)

        self._hparams.update({
            'stride': self._stride,
            'output_stride': self._output_stride,
            'backbone': backbone,
        })

    @staticmethod
    def add_model_specific_args(parent_parser):
        parent_parser = PoseEstimationModel.add_model_specific_args(parent_parser)

        parser = parent_parser.add_argument_group("UniPoseLSTM Pose Estimation Model")
        parser.add_argument(
            '--stride',
            default=8,
            type=int,
        )
        parser.add_argument(
            '--output_stride',
            default=16,
            type=int,
        )
        parser.add_argument(
            '--backbone',
            default='resnet101',
            type=str,
            choices=['resnet101', 'resnet50'],
        )
        return parent_parser

    def forward(self, input: torch.Tensor, **kwargs):
        bs, l, _, h, w = input.shape

        heatmaps = []
        cell = None
        hide = None

        # why is this even needed? it's a constant
        # in original code it came from dataset for whatever reason
        # a starting point for heatmaps maybe?
        centermap = gaussian_kernel(
            w, h, w // 2, h // 2, self._sigma)[None, None, ...].repeat(bs, l, 1, 1, 1).to(input.device)

        for iter in range(l):
            heatmap, cell, hide = self.unipose(input, centermap, iter, None, hide, cell)
            heatmaps.append(heatmap)

        heatmaps = torch.stack(heatmaps, dim=1)

        return heatmaps
