import torch
from .extractor import Extractor
from pedestrians_video_2_carla.utils.tensors import get_bboxes


class BBoxExtractor(Extractor):
    def _get_shift_point(self, sample: torch.Tensor) -> torch.Tensor:
        # center of bbox
        bboxes = get_bboxes(sample, near_zero=self.near_zero)
        return bboxes.mean(dim=-2)

    def _get_scale_point(self, sample: torch.Tensor) -> torch.Tensor:
        # top center of bbox
        bboxes = get_bboxes(sample, near_zero=self.near_zero)
        x = bboxes.mean(dim=-2)[..., 0]
        y = bboxes.min(dim=-2)[0][..., 1]

        return torch.stack((x, y), dim=-1)
