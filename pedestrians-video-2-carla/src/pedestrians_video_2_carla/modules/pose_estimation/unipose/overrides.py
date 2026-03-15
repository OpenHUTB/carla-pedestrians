# this imports, wraps & modifies original UniPoseLSTM model
# because we need to overwrite the init/forward to avoid hardcoded
# tensor sizes, forced CUDA and other stuff
# and also to handle the very ugly importing

import torch
import torch.nn.functional as F
from torchvision.models.resnet import Bottleneck
import sys
import importlib

# this is really unfortunate name to import from, but what can you do
sys.modules['model'] = importlib.import_module(
    'pedestrians_video_2_carla.third_party.unipose.model')

from model.modules import backbone, decoder
from model.modules.backbone.resnet import ResNet  # modified ResNet

# overwrite the original build_backbone to use different ResNets
def build_backbone(backbone, output_stride, BatchNorm):
    def bottleneck(inplanes, planes, stride=1, dilation=1, downsample=None, BatchNorm=None): return Bottleneck(
        inplanes=inplanes, planes=planes, stride=stride, dilation=dilation, downsample=downsample, norm_layer=BatchNorm
    )
    bottleneck.expansion = Bottleneck.expansion

    resnet_args = {
        'resnet101': (bottleneck, [3, 4, 23, 3]),
        'resnet50': (bottleneck, [3, 4, 6, 3]),
    }[backbone]

    return ResNet(
            *resnet_args,
            output_stride,
            BatchNorm,
            pretrained=False, # we're loading our own weights
        )

backbone.build_backbone = build_backbone

# overwrite the original build_decoder to tell it we're using resnet
orig_build_decoder = decoder.build_decoder
def build_decoder(dataset, num_classes, backbone, BatchNorm):
    return orig_build_decoder(dataset, num_classes, 'resnet', BatchNorm)
decoder.build_decoder = build_decoder

from model.uniposeLSTM import unipose as UniPoseLSTMOriginal, LSTM_0, LSTM

# cannot remove, otherwise it will not be found when training on GPU
# del sys.modules['model']

class UniPoseLSTM(UniPoseLSTMOriginal):
    def __init__(self, num_classes, **kwargs):
        super().__init__(num_classes=num_classes, **kwargs)

        # overwrite the layers that depend on the number of classes,
        # but were hardcoded in the original model
        self.lstm_0 = LSTM_0(num_classes + 2, num_classes + 2, 3, 1)
        self.lstm = LSTM(num_classes + 2, num_classes + 2, 3, 1)
        self.conv1 = torch.nn.Conv2d(num_classes + 2, 128, kernel_size=11, padding=5)
        self.conv5 = torch.nn.Conv2d(128, num_classes + 1, kernel_size=1, padding=0)

    def forward(self, input, centermap, iter, prev, prevHide, prevCell):
        # copied from parent class with prev/prevHide/prevCell modified
        # and overall implementation simplified

        x = input[:, iter, :, :, :]

        x, low_level_feat = self.backbone(x)
        x = self.wasp(x)
        x = self.decoder(x, low_level_feat)

        centermap = self.pool_center(centermap[:, iter, :, :, :])

        # it may happen that due to the downsampling, the centermap is smaller than the output of the decoder
        # in this case, we need to crop the decoder output to match the centermap
        if centermap.shape[-1] != x.shape[-1]:
            x = x[..., :centermap.shape[-1]]

        concatenated = torch.cat((x, centermap), dim=1)

        if iter == 0:
            cell, hide = self.lstm_0(concatenated)
        else:
            cell, hide = self.lstm(concatenated, prevHide, prevCell)

        heatmap = F.relu(self.conv1(hide))
        heatmap = F.relu(self.conv2(heatmap))
        heatmap = F.relu(self.conv3(heatmap))
        heatmap = F.relu(self.conv4(heatmap))
        heatmap = F.relu(self.conv5(heatmap))

        return heatmap, cell, hide