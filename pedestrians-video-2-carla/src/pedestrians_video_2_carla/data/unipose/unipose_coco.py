# this imports & wraps original UniPoseCOCO model
# to handle the very ugly importing

import sys
import importlib
import torch

# this is really unfortunate name to import from, let's only keep it as long as necessary
sys.modules['model'] = importlib.import_module(
    'pedestrians_video_2_carla.third_party.unipose.model')
from model.unipose import unipose
del sys.modules['model']


class UniPoseCOCO(unipose):
    def __init__(self, unipose_model_path=None):
        super().__init__(dataset='COCO', num_classes=16, backbone='resnet', output_stride=16,
                         sync_bn=True, freeze_bn=False, stride=8)

        if unipose_model_path is not None:
            pretrained_weights = torch.load(unipose_model_path, map_location='cpu')
            state_dict = self.state_dict()
            model_dict = {}

            for k, v in pretrained_weights.items():
                if k in state_dict:
                    model_dict[k] = v

            state_dict.update(model_dict)
            self.load_state_dict(state_dict)

            self.eval()
